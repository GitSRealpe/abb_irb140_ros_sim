#! /usr/bin/env python

import matplotlib.pyplot as plt
from utils.dataset_processing import grasp_picking, image
from utils.dataset_processing import evaluation1, grasp_picking
from utils.dataset_processing.grasp import GraspRectangle
import math
import torch
import time
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

device = torch.device('cpu')


MODEL_FILE = 'ggcnn2_093'
rgbo = []
rgbfin = []
cajon =1
if cajon==1 or cajon==4 :
   crop_size=300
   y_off=50
   x_off=0
   robot =[1.57, -0.17, 0.05, 0, 0.52, 0.02]
   iy=40
   Dy=190
   Dx=300
elif cajon==2 or cajon==5:
   crop_size=320
   y_off=50
   x_off=0
   iy=55
   Dy=205
   Dx=320
   robot =[1.57, -0.07, 0.24, 0, 0.68, 0.02]
elif cajon==3 or cajon==6:
   crop_size=225
   y_off=50
   x_off=0
   iy=70
   Dy=145
   Dx=225
   robot =[1.57, -0.70, 0.93, 0, 0.48, 0.02]

#MODEL_FILE = 'jacquard_86'
#depthfin=cv2.resize(depthfin, (640, 480), interpolation = cv2.INTER_AREA)
model = torch.load(MODEL_FILE, map_location='cpu')

def process_depth_image(depth, crop_size, out_size=crop_size, return_mask=False, crop_y_offset=0):


    imh, imw = depth.shape
    print(depth.shape)

    with TimeIt('1'):

       fig = plt.figure(figsize=(10, 10))
       ax = fig.add_subplot(1, 1, 1)
       ax.imshow(depth, cmap='gray')
       ax.set_title('original')
       ax.axis('off')
       plt.show()

       #depth_crop = depth[20+y_off:460+y_off,100+x_off:540+x_off]
       #depth_crop = depth[170+x_off:470+x_off,90+y_off:390+y_off]
       depth_crop1 = depth[(imh - crop_size) // 2 + y_off:(imh - crop_size) // 2 + crop_size + y_off,
                           (imw - crop_size) // 2+x_off:(imw - crop_size) // 2 + crop_size+x_off]

       if cajon==1 or cajon==4:
           depth_crop = depth_crop1[iy:iy+Dy,0:300]
           #depth = depth - depthn -depth[240,150]
       elif cajon==2 or cajon==5 :
           depth_crop = depth_crop1[iy:iy+Dy,0:320]
           depth_crop = depth_crop1
       elif cajon==3 or cajon==6:
           depth_crop = depth_crop1[iy:iy+Dy,0:320]
           #depth_crop = depth_crop1


       fig = plt.figure(figsize=(10, 10))
       ax = fig.add_subplot(1, 1, 1)
       ax.imshow(depth_crop, cmap='gray')
       ax.set_title('crop')
       ax.axis('off')
       plt.show()
    # depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

    # Inpaint
    # OpenCV inpainting does weird things at the border.
    with TimeIt('2'):
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)


    with TimeIt('3'):
        depth_crop[depth_nan_mask==1] = 0
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(depth_crop, cmap='gray')
        ax.set_title('mask')
        ax.axis('off')
        plt.show()



    with TimeIt('4'):
        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.



        with TimeIt('Inpainting'):
            depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)
        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 2, 1)
        ax.imshow(depth_crop, cmap='gray')
        ax.set_title('inpainting')
        ax.axis('off')
        plt.show()    #print(depth_crop.shape)


    with TimeIt('5'):
        # Resize
        depth_crop = cv2.resize(depth_crop, (out_size, out_size), interpolation = cv2.INTER_AREA)

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(depth_crop, cmap='gray')
        ax.set_title('resize')
        ax.axis('off')
        plt.show()


    if return_mask:
        with TimeIt('6'):
            depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
            depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), interpolation = cv2.INTER_NEAREST)

        return depth_crop, depth_nan_mask
    else:
        return depth_crop


#def get_depth(depthorig, rot=0, zoom=1.0):
#        depth_img = image.DepthImage.from_tiff(depthorig)
        #center, left, top = depth_img._get_crop_attrs(0)
        #depth_img.rotate(0.8, [320,240])
        #depth_img.crop((top, left), (min(480, top + self.output_size), min(640, left + self.output_size)))
#        depth_img.normalise()
        #depth_img.zoom(zoom)
        #depth_img.zoom(0.5)
#        return depth_img.img

def predict(depth, process_depth=True, crop_size=crop_size, out_size=crop_size, depth_nan_mask=None, crop_y_offset=0, filters=(5.0, 3.0, 3.0)):
    if process_depth:
        depth, depth_nan_mask = process_depth_image(depth, crop_size, out_size=out_size, return_mask=True, crop_y_offset=0)

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)



    #depth = cv2.blur(depth,(5,5))
    depthn = depth.copy()
    #if cajon==1 or cajon==4:
       #depthn[40   :230,0:300] = depth[240,150]
       #depth = depth - depthn -depth[240,150]
    #elif cajon==2 or cajon==5 :
       #depthn[55:260,0:320] = depth[270,160]
       #depth = depth - depthn -depth[270,160]
    #elif cajon==3 or cajon==6:
       #depthn[70:215,0:320] = depth[60,160]
       #depth = depth - depthn -depth[60,160]

    #depthn = ndimage.filters.gaussian_filter(depthn, 1)

    #print(depth.shape)
    depthT = torch.from_numpy(depth.reshape(1, 1, out_size, out_size).astype(np.float32)).to(device)
    with torch.no_grad():
        pred_out = model(depthT)

    points_out = pred_out[0].cpu().numpy().squeeze()
    points_out[depth_nan_mask] = 0

    # Calculate the angle map.
    cos_out = pred_out[1].cpu().numpy().squeeze()
    sin_out = pred_out[2].cpu().numpy().squeeze()
    ang_out = np.arctan2(sin_out, cos_out) / 2.0

    width_out = pred_out[3].cpu().numpy().squeeze() * 120.0  # Scaled 0-150:0-1

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

def graspdata(points_out, depthfin, grasps, m):
    ROBOT_Z = 0
    fx = 458.455478616934780
    cx = 343.645038678435410
    fy = 458.199272745572390
    cy = 229.805975111304460
    py,px=grasps[m].center
    print("px,py: ",px,py)
    ang = grasps[m].angle
    print(ang)
    width = grasps[m].width*2
    print("width: ", width)
    print('qf: ',points_out[py, px])
    print('viejopixel: ', py, px)
    #max_pixel = ((np.array(max_pixel) / 440.0 * crop_size) + np.array([(480 - crop_size)//2+y_off, (640 - crop_size) // 2+x_off]))
    #max_pixel = ((np.array(max_pixel) / 300.0 * crop_size) + np.array([(640 - crop_size)//2+y_off, (480 - crop_size) // 2+x_off]))
    for g in grasps:
        pyn = g.center[0]*Dy/crop_size + (480 - crop_size) // 2+y_off+iy
        pxn = g.center[1]*Dx/crop_size + (640 - crop_size) // 2+x_off
        g.center = [pyn,pxn]
        l1 = g.length*np.cos(ang)*Dx/crop_size
        l2 = g.length*np.sin(ang)*Dy/crop_size
        l=np.sqrt(l1*l1+l2*l2)
        g.length =l
        g.width = l/2


            #pxn = np.round(pxn).astype(np.int)
            #pyn = np.round(pxn).astype(np.int)

    print('length, width: ', grasps[m].length, grasps[m].width)
    print('Nuevopixel: ', grasps[m].center[0],grasps[m].center[1])


    #ENCONTRAR VALORES REALES DE IMAGEN
            #point_depth = depthfin[max_pixel[0], max_pixel[1]]
    pyn=grasps[m].center[0]
    pxn=grasps[m].center[1]
    point_depth = depthfin[pyn,pxn]
    x = (pxn - cx)/(fx)*point_depth
    y = (pyn - cy)/(fy)*point_depth
    z = point_depth
    x1 = (pxn+width*math.cos(ang)/2 - cx)/(fx)*point_depth
    y1 = (pyn+width*math.sin(ang)/2 - cy)/(fy)*point_depth
    x2 = (pxn-width*math.cos(ang)/2 - cx)/(fx)*point_depth
    y2 = (pyn-width*math.sin(ang)/2 - cy)/(fy)*point_depth

    rwidth =math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2))
    print('x: ', x*100)
    print('y: ', y*100)
    print('z: ', z)
    print('ang: ', ang*180/math.pi)
    print('width: ', width)
    print('rwidth: ', rwidth)

    return x,y,z, ang, rwidth


class image_converter:

    def __init__(self, args):

        self.index = 0
        if len(sys.argv) > 1:
            self.index = int(sys.argv[1])
        print("hola")
        print("hola")
        bridge = CvBridge()
        #while not rospy.is_shutdown():
        #raw_input()
        rgbo = rospy.wait_for_message('/camera/color/image_raw', Image)
        print("rgb")
        cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)
        deptho = rospy.wait_for_message('/camera/depth/image_raw', Image)
        depthfin1 = bridge.imgmsg_to_cv2(deptho)
        rgbfin = bridge.imgmsg_to_cv2(rgbo)

        if cajon==1:
           crop_size=300
           y_off=50
           x_off=0
           depthfin = depthfin1/15
        elif cajon==2:
           crop_size=320
           y_off=50
           x_off=0
           depthfin = depthfin1/7
        elif cajon==3:
           crop_size=225
           y_off=50
           x_off=0
           depthfin = depthfin1/5
           #cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB, cv_image)
           #cv2.imwrite('/home/mateo/catkin_ws/src/grasping/src/img1.png', cv_image)#*255)

        points_out, ang_out, width_out, depth = predict(depthfin, crop_size=crop_size)
        grasps = grasp_picking.detect_grasps(points_out, ang_out, 0.7, width_img=width_out, no_grasps=5)

        for g in grasps:
	    corners=[]
	    cy,cx = g.center
	    a = g.angle
 	    w = g.width/2*np.sin(a)
            h = g.length/2*np.cos(a)
            corners=[cy+w+h,cy+w-h,cy-w+h,cy-w-h]
	    if cajon == 1:
		limit=225
	    elif cajon == 2:
		limit=255
	    elif cajon == 3:
		limit=210
	    print("corners: ", corners)
	    if	corners[0]>=limit or corners[1]>=limit or corners[2]>=limit or corners[3]>=limit:
	        g.angle = g.angle
    	    print("coordenadas: ",corners)

        m = evaluation1.plot_output(width_out, depth, points_out, ang_out, grasps, depthfin, crop_size, y_off, x_off)

	#ENCONTRAR LA PROFUNDIDAD EN LA IMAGEN ORIGINAL
        x, y, z, ang, rwidth = graspdata(points_out, depthfin1, grasps, m)
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [x, y, z, ang, rwidth]
        cmd_pub.publish(cmd_msg)

        punto=gmsg.Pose()
        #invertidos porque si
        punto.position.x=z
        punto.position.y=-x
        punto.position.z=-y
        print punto

        print convert_pose(punto,"cam_link","world")

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(rgbfin)
        for g in grasps:
            g.plot(ax)
        ax.set_title('rgb')
        ax.axis('off')
        plt.show()





if __name__=='__main__':
    rospy.init_node('save_img')
    image_converter(sys.argv)
    #rospy.spin()
