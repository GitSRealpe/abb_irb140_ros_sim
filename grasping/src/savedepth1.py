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

device = torch.device('cpu')
crop_size=400
MODEL_FILE = 'ggcnn2_093'
rgbo = []
rgbfin = []
y_off=0
x_off=0
model = torch.load(MODEL_FILE, map_location='cpu')

def process_depth_image(depth, crop_size, out_size=crop_size, return_mask=False, crop_y_offset=0):
    imh, imw = depth.shape
    print(depth.shape)

    with TimeIt('1'):
       depth_crop = depth[(imh - crop_size) // 2 + y_off:(imh - crop_size) // 2 + crop_size + y_off,
                           (imw - crop_size) // 2+x_off:(imw - crop_size) // 2 + crop_size+x_off]

    # Inpaint
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

def predict(depth, process_depth=True, crop_size=crop_size, out_size=crop_size, depth_nan_mask=None, crop_y_offset=0, filters=(5.0, 2.0, 2.0)):
    if process_depth:
        depth, depth_nan_mask = process_depth_image(depth, crop_size, out_size=out_size, return_mask=True, crop_y_offset=0)

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)
    #depth = cv2.blur(depth,(5,5))
    depthn = depth.copy()
    #Y,X
    depthn[120:285,68:338] =depth[115,65]
    #depthn = ndimage.filters.gaussian_filter(depthn, 1)
    depth = depth - depthn -depth[115,65]
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

    pyn = py + (480 - crop_size) // 2+y_off
    pxn = px + (640 - crop_size) // 2+x_off
            #pxn = np.round(pxn).astype(np.int)
            #pyn = np.round(pxn).astype(np.int)
    grasps[0].center= pyn,pxn
    print('Nuevopixel: ', pyn, pxn)

    #fig = plt.figure(figsize=(10, 10))
    #ax = fig.add_subplot(1, 1, 1)
    #plot = ax.imshow(rgbfin)
            #for g in grasps:
    #grasps[0].plot(ax)
    #ax.set_title('RGB')
    #ax.axis('off')
    #plt.show()
        #ENCONTRAR VALORES REALES DE IMAGEN
            #point_depth = depthfin[max_pixel[0], max_pixel[1]]
    point_depth = depthfin[pyn,pxn]
    x = (pxn - cx)/(fx)*point_depth
    y = (pyn - cy)/(fy)*point_depth
    z = point_depth
    x1 = (px+width*math.cos(ang)/2 - cx)/(fx)*point_depth
    y1 = (py+width*math.sin(ang)/2 - cy)/(fy)*point_depth
    x2 = (px-width*math.cos(ang)/2 - cx)/(fx)*point_depth
    y2 = (py-width*math.sin(ang)/2 - cy)/(fy)*point_depth

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
        rospy.init_node('save_img')
        bridge = CvBridge()
        #while not rospy.is_shutdown():
        #raw_input()
        #color_sub = rospy.Subscriber("/camera/color/image_raw",Image)
        cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)
        print "depth"
        rgbo = rospy.wait_for_message('/camera/color/image_raw', Image)
        print "depth"
        deptho = rospy.wait_for_message('/camera/depth/image_raw', Image)
    #    rgbo = rospy.wait_for_message('/camera/rgb/image_color', Image)
        depthfin = bridge.imgmsg_to_cv2(deptho)
        rgbfin = bridge.imgmsg_to_cv2(rgbo)
        depthfin = depthfin/2
        #rgbfin1= cv2.cvtColor(rgbfin, cv2.COLOR_BGR2RGB)
        #cv2.imwrite('rgb.png', rgbfin1)
        #MODEL_FILE = 'training2_084'
        points_out, ang_out, width_out, depth = predict(depthfin)
        grasps = grasp.detect_grasps(points_out, ang_out, 0.7, width_img=width_out, no_grasps=5)

        m = evaluation1.plot_output(width_out, depth, points_out, ang_out, grasps, rgbfin, crop_size, y_off, x_off)

        x, y, z, ang, rwidth = graspdata(points_out, depthfin, grasps, m)

        punto=gmsg.Pose()
        #invertidos porque si
        punto.position.x=x
        punto.position.y=y
        punto.position.z=0
        print punto

        print convert_pose(punto,"cam","world")

	#fig.savefig('plot1.png')
	#ENCONTRAR LA PROFUNDIDAD EN LA IMAGEN ORIGINAL
	#PIXEL CON VALOR MAXIMO



if __name__=='__main__':
    image_converter(sys.argv)
