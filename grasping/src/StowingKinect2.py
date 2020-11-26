#! /usr/bin/env python

import matplotlib.pyplot as plt
from utils.dataset_processing import grasp, image
from utils.dataset_processing import evaluation1, grasp
from utils.dataset_processing.grasp import GraspRectangle
import math
import torch
import time
from PIL import Image
from os.path import join
import numpy as np
import geometry_msgs.msg
import sys
import cv2
import scipy.ndimage as ndimage
import rospy
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from models.common import post_process_output
from utils.timeit import TimeIt
from helpers.transforms import *
from utils.dataset_processing import calibracionKinect
import tf.transformations as tft
rospy.init_node('save_img')
rate = rospy.Rate(1) # ROS Rate at 5Hz
device = torch.device('cpu')
crop_size=400
MODEL_FILE = 'ggcnn2_093'
rgbo = []
rgbfin = []
y_off=0
x_off=0
model = torch.load(MODEL_FILE, map_location='cpu')

bridge = CvBridge()
cmd_pub = rospy.Publisher('ggcnn/rvalues', Float32MultiArray, queue_size=1)

#iy=102
#ix=45
#Dy=196
#Dx=308
fx = 458.455478616934780
cx = 343.645038678435410
fy = 458.199272745572390
cy = 229.805975111304460
#fx = 585.6
cx = 320
#fy = 585.6
cy = 240
#iy=60
#ix=95
#Dy=300
#Dx=190


def process_depth_image(depth, crop_size, out_size=crop_size, return_mask=False, crop_y_offset=0):
    imh, imw = depth.shape
    #print(depth.shape)

    depth_crop = depth
    #fig = plt.figure(figsize=(10, 10))
    #ax = fig.add_subplot(1, 1, 1)
    #ax.imshow(depth_crop, cmap='gray')

    #ax.set_title('Depth')
    #ax.axis('off')
    #plt.show()

    with TimeIt('1'):

       #depth_crop1 = depth[(imh - crop_size) // 2 + y_off:(imh - crop_size) // 2 + crop_size + y_off,
                           #(imw - crop_size) // 2+x_off:(imw - crop_size) // 2 + crop_size+x_off]

       #depth_crop1 = depth_crop1[iy:iy+Dy,ix:ix+Dx]

       #depth_crop = depth_crop1.copy()
       depth_crop = cv2.normalize(depth_crop.astype('float32'), None, 1.0, -1.0, cv2.NORM_MINMAX)
       depth_crop =depth_crop/5
       #fig = plt.figure(figsize=(10, 10))
       #ax = fig.add_subplot(1, 1, 1)
       #ax.imshow(depth_crop, cmap='gray')
       #ax.set_title('1')
       #ax.axis('off')
       #plt.show()
    # OpenCV inpainting does weird things at the border.
    with TimeIt('2'):
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

    with TimeIt('3'):
        depth_crop[depth_nan_mask==1] = 0

    with TimeIt('4'):
        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

        with TimeIt('Inpainting'):
            depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

    with TimeIt('5'):
        # Resize
        depth_crop = cv2.resize(depth_crop, (out_size, out_size), interpolation = cv2.INTER_AREA)

    if return_mask:
        with TimeIt('6'):
            depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
            depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), interpolation = cv2.INTER_NEAREST)

        return depth_crop, depth_nan_mask
    else:
        return depth_crop

def predict(depth, process_depth=True, crop_size=crop_size, out_size=crop_size, depth_nan_mask=None, crop_y_offset=0, filters=(4.0, 2.0, 2.0)):
    if process_depth:
        depth, depth_nan_mask = process_depth_image(depth, crop_size, out_size=out_size, return_mask=True, crop_y_offset=0)

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)
    #depth = cv2.blur(depth,(5,5))
    depthn = depth.copy()

    #depthn = ndimage.filters.gaussian_filter(depthn, 1)
    #depth = depth - depthn -depth[69,102]
    depthT = torch.from_numpy(depth.reshape(1, 1, out_size, out_size).astype(np.float32)).to(device)
    with torch.no_grad():
        pred_out = model(depthT)

    points_out = pred_out[0].cpu().numpy().squeeze()
    points_out[depth_nan_mask] = 0

    # Calculate the angle map.
    cos_out = pred_out[1].cpu().numpy().squeeze()
    sin_out = pred_out[2].cpu().numpy().squeeze()
    ang_out = np.arctan2(sin_out, cos_out) / 2.0

    width_out = pred_out[3].cpu().numpy().squeeze() * 150.0  # Scaled 0-150:0-1

    # Filter the outputs.

    points_out = ndimage.filters.gaussian_filter(points_out, filters[0])  # 3.0
    ang_out = ndimage.filters.gaussian_filter(ang_out, filters[1])
    width_out = ndimage.filters.gaussian_filter(width_out, filters[2])

    points_out = np.clip(points_out, 0.0, 1.0-1e-3)

    # SM
    # temp = 0.15
    # ep = np.exp(points_out / temp)
    # points_out = ep / ep.sum()

    # points_out = (points_out - points_out.min())/(points_out.max() - points_out.min())

    return points_out, ang_out, width_out, depth.squeeze()


def pushing(grasps, ix1, iy1, Dx1, Dy1, Dx, Dy):
    pushlist = np.zeros(len(grasps), dtype=int)
    cont =0
    for g in grasps:

        ang = g.angle
        l1x=g.length*np.cos(ang)*Dx/(2*crop_size)
        l2x=g.width*np.sin(ang)*Dx/(2*crop_size)
        l1y=g.length*np.sin(ang)*Dy/(2*crop_size)
        l2y=g.width*np.cos(ang)*Dy/(2*crop_size)
        mx = g.center[1]*Dx/crop_size-l1x-l2x
        max= g.center[1]*Dx/crop_size+l1x+l2x
        my= g.center[0]*Dy/crop_size-l1y-l2y
        may= g.center[0]*Dy/crop_size+l1y+l2y
        #print('mx, ix', mx, ix1)
        #print('mx, ix', mx, ix1)
        #print('max, ix+Dx', max, ix1+Dx1)
        #print('my, iy', my, iy1)
        #print('may, iy+Dy', may, iy1+Dy1)

        if my < iy1:
            pushlist[cont]=3
            g.length = 5
            g.width = 5
        if may > iy1+Dy1:
            pushlist[cont]=4
            g.length = 5
            g.width = 5
        if mx < ix1:
            pushlist[cont]=1
            g.length = 5
            g.width = 5
        if max > ix1+Dx1:
            pushlist[cont]=2
            g.length = 5
            g.width = 5
        cont =cont+1


    return pushlist, grasps


def graspdata(points_out, depthfin, grasps, ix, iy, Dx, Dy):

    for g in grasps:
        #print('Dx y Dy:', Dx, Dy, ix, iy)
        #pyn = g.center[0]*Dy/crop_size + (480 - crop_size) // 2+y_off+iy
        #pxn = g.center[1]*Dx/crop_size + (640 - crop_size) // 2+x_off+ix
        pyn = g.center[0]*Dy/crop_size + iy+y_off
        pxn = g.center[1]*Dx/crop_size + ix+x_off
        ang=g.angle
        g.center = [pyn,pxn]
        l1 = g.length*np.cos(ang)*Dx/crop_size
        l2 = g.length*np.sin(ang)*Dy/crop_size
        l=np.sqrt(l1*l1+l2*l2)
        g.length =l
        g.width = l/2

    return grasps

def rvalues(grasp, depth, Dx, Dy):

    #print('length, width: ', grasp.length, grasp.width)
    #print('Nuevopixe    l: ', grasp.center[0],grasp.center[1])
    length = grasp.length
    width = grasp.width

    pyn=grasp.center[0]
    pxn=grasp.center[1]
    ang=grasp.angle
    point_depth = depth[pyn,pxn]/1000
    x = (pxn - 318)/(fx)*point_depth
    y = (pyn - 229)/(fy)*point_depth
    print('x1, y1' , x*100, y*100)
    x = (pxn-cx)*0.66/(Dx+13)
    y = (pyn-cy)*0.44/(Dy+25)
    z = point_depth
    x1 = (pxn+width*math.cos(ang)/2 - cx)/(fx)*point_depth
    y1 = (pyn+width*math.sin(ang)/2 - cy)/(fy)*point_depth
    x2 = (pxn-width*math.cos(ang)/2 - cx)/(fx)*point_depth
    y2 = (pyn-width*math.sin(ang)/2 - cy)/(fy)*point_depth

    rwidth =math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2))


    #print('x: ', x*100)
    #print('y: ', y*100)
    #print('z: ', z)
    #print('ang: ', ang*180/math.pi)
    #print('width: ', width)
    #print('rwidth: ', rwidth)


    return x,y,z, ang, rwidth


def find_pose():
        print('hola')
        #color_sub = rospy.Subscriber("/camera/color/image_raw",Image)
        rgbo = rospy.wait_for_message('/camera/rgb/image_rect_color', Image)
        print "depth"
        deptho = rospy.wait_for_message('/camera/depth/image_raw', Image)
    #    rgbo = rospy.wait_for_message('/camera/rgb/image_color', Image)
        depthfin = bridge.imgmsg_to_cv2(deptho)
        rgbfin = bridge.imgmsg_to_cv2(rgbo)
        rgb1=cv2.cvtColor(rgbfin,cv2.COLOR_BGR2RGB)
        #(y, x) = np.where(out2 == 255)
        #(topy, topx) = (np.min(y), np.min(x))
        #(bottomy, bottomx) = (np.max(y), np.max(x))
        #iy1=topy-13
        #ix1=topx-16
        #Dy1=bottomy-iy1+13
        #Dx1=bottomx-ix1+16
        #print('Calibracion, iy1, ix1, Dy1, Dx1:',iy1 , ix1, Dy1, Dx1 )
        #iy, ix, Dy, Dx, widthbinx, widthbiny, iy1, ix1, Dy1, Dx1 = calibracionKinect.calibracion(depthfin, rgbfin)

        #print(xx)
        iy, ix, Dy, Dx, widthbinx, widthbiny, iy1, ix1, Dy1, Dx1 =[147, 174, 177, 274, 274, 177, 0, 0, 176, 272]
        #depthfin1= depthfin.copy()
        depthfin1 = depthfin[iy:Dy+iy, ix:Dx+ix]
        #depthfin1 = depthfin1/2
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(depthfin, cmap='gray')
        ax.set_title('Depth')
        ax.axis('off')
        plt.show()

        #cv2.imwrite('rgb.png', rgbfin1)
        #MODEL_FILE = 'training2_084'
        points_out, ang_out, width_out, depth = predict(depthfin1)

        grasps = grasp.detect_grasps(points_out, ang_out, 0.7, width_img=width_out, no_grasps=5)
        maxgrasps = evaluation1.plot_output(width_out, depth, points_out, ang_out, grasps, rgbfin, crop_size, y_off, x_off)
        #m = evaluation1.plot_output(width_out, depth, points_out, ang_out, grasps, rgbfin, crop_size, y_off, x_off)
	#fig.savefig('plot1.png')
	#ENCONTRAR LA PROFUNDIDAD EN LA IMAGEN ORIGINAL
	#PIXEL CON VALOR MAXIMO
        pushlist, grasps = pushing(grasps, ix1, iy1, Dx1, Dy1, Dx, Dy)

        grasps = graspdata(points_out, depthfin, grasps, ix, iy, Dx, Dy)
        x, y, z, ang, rwidth =rvalues(grasps[maxgrasps[0][1]], depthfin, Dx, Dy)

        arr=[]
        for i in range(len(grasps)):
            gqmax = maxgrasps[i][1]
            mov = pushlist[gqmax]
            x, y, z, ang, rwidth =rvalues(grasps[gqmax], depthfin, Dx, Dy)
            arr.append(x)
            arr.append(y)
            arr.append(z)
            arr.append(ang)
            arr.append(rwidth)
            arr.append(mov)

        #print('arr', arr)


        cmd_msg = Float32MultiArray()

        cmd_msg.data =  arr        #print('publicado lol: ', cmd_msg)
        cmd_pub.publish(cmd_msg)
        #   t = tf2.Transformer(True, rospy.Duration(10.0))
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'world'
        m.child_frame_id = 'world'
        m.transform.translation = [0, 0, 0]
        m.transform.rotation = [0, 0, 0, 0]
        #t.setTransform(m)
        #print( t.lookupTransform('cam', 'world', rospy.Time(0)))

        punto=gmsg.Pose()
        #invertidos porque si
        #punto.position.x=z
        #punto.position.y=-x
        #punto.position.z=-y






        cont=0
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 3, 1)
        ax.imshow(depthfin, cmap='gray')
        for g in grasps:
            g.plot(ax)
            if pushlist[cont] == 1:
                ax.arrow(g.center[1], g.center[0], 50, 0, head_width=8, head_length=10, fc='lightblue', ec='red')
            if pushlist[cont] == 2:
                ax.arrow(g.center[1], g.center[0], -50, 0, head_width=8, head_length=10, fc='lightblue', ec='red')
            if pushlist[cont] == 3:
                ax.arrow(g.center[1], g.center[0], 0, 50, head_width=8, head_length=10, fc='lightblue', ec='red')
            if pushlist[cont] == 4:
                ax.arrow(g.center[1], g.center[0], 0, -50, head_width=8, head_length=10, fc='lightblue', ec='red')
            cont = cont+1
        ax.set_title('Depth')
        ax.axis('off')



        ax = fig.add_subplot(1, 3, 3)
        plot = ax.imshow(depthfin1, cmap='gray')
        ax.set_title('quality')
        ax.axis('off')

        ax = fig.add_subplot(1, 3, 2)
        plot = ax.imshow(points_out, cmap='jet', vmin=0, vmax=1)
        ax.set_title('quality')
        ax.axis('off')
        #ax = fig.add_subplot(2, 2, 3)
        #plot = ax.imshow(width_out, cmap='hsv', vmin=0, vmax=150)
        #ax.set_title('width')
        #ax.axis('off')
        #plt.colorbar(plot)


        #ax = fig.add_subplot(1, 3, 2)
        #ax.imshow(rgbfin)
        #ax.set_title('rgb')
        #ax.axis('off')
        plt.show()
    #rospy.spin()




while not rospy.is_shutdown():

    find_pose()

    rate.sleep()
