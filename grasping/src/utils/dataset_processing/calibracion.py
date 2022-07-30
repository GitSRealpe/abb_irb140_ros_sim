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
import copy
from helpers.transforms import *

device = torch.device('cpu')
crop_size=400
MODEL_FILE = 'ggcnn2_093'
rgbo = []
rgbfin = []
y_off=0
x_off=0
model = torch.load(MODEL_FILE, map_location='cpu')
#iy=102
#ix=45
#Dy=196
#Dx=308
ROBOT_Z = 0
fx = 458.455478616934780
cx = 343.645038678435410
fy = 458.199272745572390
cy = 229.805975111304460
#fx = 585.6
cx = 320
#fy = 585.6
cy = 240





def calibracion(depthfin, rgbfin):



        gray=cv2.cvtColor(rgbfin,cv2.COLOR_RGB2GRAY)
        edged=cv2.Canny(gray,30,200)
        edged1 =edged.copy()
        _, contours, hierarchy=cv2.findContours(edged1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        print('Numbers of contours found=' + str(len(contours)))
        idx = 0
        #mask = np.zeros_like(rgbfin)
        out= depthfin.copy()
        cv2.drawContours(out, contours, idx, 255, -1)
        (y, x) = np.where(out == 255)
        (topy, topx) = (np.min(y), np.min(x))
        (bottomy, bottomx) = (np.max(y), np.max(x))
        #depth_crop = depth[topy:bottomy+1, topx:bottomx+1]
        #depth[out != 255] = 0.484
        print('top:',topx,bottomx, bottomx-topx )
        widthbinx=bottomx-topx
        widthbiny=bottomy-topy
        OFFx=25
        OFFy=15
        iy=topy+OFFy
        ix=topx+OFFx
        Dy=bottomy-OFFy-iy
        Dx=bottomx-OFFx-ix
        print('Calibracion, iy, ix, Dy, Dx:',iy , ix, Dy, Dx )
        print('Calibracion, widthx, widthy:',widthbinx, widthbiny )

        depthfin1 = depthfin[iy:Dy+iy, ix:Dx+ix]


        #out = np.zeros_like(rgbfin)
        #out[mask == 255] = rgbfin[mask == 255]
        #cv2.drawContours(rgbfin,contours,-1,(0,255,0),3)
        rgbfin1 = rgbfin[iy:Dy+iy, ix:Dx+ix]
        edged2=cv2.Canny(rgbfin1,30,200)
        _, contours1, hierarchy=cv2.findContours(edged2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        out2 = depthfin1.copy()
        cv2.drawContours(out2,contours1,-1,255,-1)
        (y, x) = np.where(out2 == 255)
        (topy, topx) = (np.min(y), np.min(x))
        (bottomy, bottomx) = (np.max(y), np.max(x))

        offx = 16
        offy = 13
        iy1=topy-offy
        ix1=topx-offx
        Dy1=bottomy-iy1+offy
        Dx1=bottomx-ix1+offx
        print('Calibracion, iy1, ix1, Dy1, Dx1:',iy1 , ix1, Dy1, Dx1 )
        depthfin2 = depthfin1[iy1:Dy1+iy1, ix1:Dx1+ix1]

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(2, 2, 1)
        ax.imshow(gray, cmap ='gray')
        ax.set_title('gray')
        ax.axis('off')


        ax = fig.add_subplot(2, 2, 2)
        ax.imshow(depthfin2, cmap ='gray')
        ax.set_title('Depth2')
        ax.axis('off')

        ax = fig.add_subplot(2, 2, 3)
        ax.imshow(out)
        ax.set_title('out')
        ax.axis('off')


        ax = fig.add_subplot(2, 2, 4)
        ax.imshow(out2)
        ax.set_title('rgb1')
        ax.axis('off')
        plt.show()




        return iy, ix, Dy, Dx, widthbinx, widthbiny, iy1, ix1, Dy1, Dx1
