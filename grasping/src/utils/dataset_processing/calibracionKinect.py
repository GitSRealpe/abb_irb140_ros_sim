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


        rgb1=cv2.cvtColor(rgbfin,cv2.COLOR_BGR2RGB)
        gray=cv2.cvtColor(rgbfin,cv2.COLOR_BGR2GRAY)
        edged=cv2.Canny(gray,5,255)
        edged2 = cv2.GaussianBlur(edged, (9,9),0)
        kernel =np.ones((3,3), np.uint8)
        edged1 =cv2.erode(edged2, kernel, iterations=1)
        #edged1 = cv2.medianBlur(edged1, 9)
        _, contours, hierarchy=cv2.findContours(edged1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        print('Numbers of contours found=' + str(len(contours)))
        idx = 2
        #mask = np.zeros_like(rgbfin)
        out= gray.copy()
        #cv2.drawContours(rgb1, contours, -1,(0,255,0), 3)
        cv2.drawContours(out, contours,idx, 255, -1)
        (y, x) = np.where(out == 255)
        (topy, topx) = (np.min(y), np.min(x))
        (bottomy, bottomx) = (np.max(y), np.max(x))
        #depth_crop = depth[topy:bottomy+1, topx:bottomx+1]
        #depth[out != 255] = 0.484
        print('top:',topx,bottomx, bottomx-topx )
        topx=158
        topy=171
        bottomx=432
        bottomy=348
        widthbinx=bottomx-topx
        widthbiny=bottomy-topy
        offx=16
        offy=24
        iy=topy-offy
        ix=topx+offx
        Dy=bottomy-iy-offy
        Dx=bottomx-ix+offx
        print('Calibracion, iy, ix, Dy, Dx:',iy , ix, Dy, Dx )
        print('Calibracion, widthx, widthy:',widthbinx, widthbiny )
        #iy=0
        #ix=0
        #Dy=100
        #Dx=100depthfin
        depthfin1 = depthfin[iy:Dy+iy, ix:Dx+ix]

        rgbfin1 = rgbfin[topy:bottomy,topx:bottomx]
        gray1 = cv2.cvtColor(rgbfin1,cv2.COLOR_BGR2GRAY)

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(rgb1)
        ax.set_title('gray1')
        ax.axis('off')
        plt.show()

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(depthfin1, cmap ='gray')
        ax.set_title('depth1')
        ax.axis('off')
        plt.show()
        #ax = fig.add_subplot(2, 2, 3)
        #ax.imshow(gray, cmap ='gray')
        #ax.set_title('gray')
        #ax.axis('off')


        #ax = fig.add_subplot(2, 2, 4)
        #ax.imshow(edged)
        #ax.set_title('Dgd')
        #ax.axis('off')

        plt.show()



        offrgbx = 10
        offrgby = 10
        rgbfin2 = rgbfin[topy+offrgby:bottomy-offrgby,topx+offrgbx:bottomx-offrgbx]


        edged3=cv2.Canny(rgbfin2,5,255)
        edged4 = cv2.GaussianBlur(edged3, (9,9),0)
        _, contours1, hierarchy=cv2.findContours(edged4,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        out2 = rgbfin2.copy()
        cv2.drawContours(out2,contours1,-1,255,-1)


        ix1=6
        Dx1 =320
        iy1=10
        Dy1=204
        print('Calibracion, iy1, ix1, Dy1, Dx1:',iy1 , ix1, Dy1, Dx1 )
        #(y, x) = np.where(out2 == 255)
        #(topy, topx) = (np.min(y), np.min(x))
        #(bottomy, bottomx) = (np.max(y), np.max(x))
        #iy1=topy-13
        #ix1=topx-16
        #Dy1=bottomy-iy1+13
        #Dx1=bottomx-ix1+16
        #print('Calibracion, iy1, ix1, Dy1, Dx1:',iy1 , ix1, Dy1, Dx1 )

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(3, 2, 1)
        ax.imshow(gray, cmap ='gray')
        ax.set_title('gray')
        ax.axis('off')


        ax = fig.add_subplot(3, 2, 2)
        ax.imshow(edged)
        ax.set_title('Dgd')
        ax.axis('off')

        ax = fig.add_subplot(3, 2, 3)
        ax.imshow(out,cmap ='gray')
        ax.set_title('out')
        ax.axis('off')


        ax = fig.add_subplot(3, 2, 4)
        ax.imshow(gray1,cmap ='gray')
        ax.set_title('gray1')
        ax.axis('off')


        ax = fig.add_subplot(3, 2, 5)
        ax.imshow(out2)
        ax.set_title('rgbfin2')
        ax.axis('off')

        ax = fig.add_subplot(3, 2, 6)
        ax.imshow(rgbfin2)
        ax.set_title('rgbfin2')
        ax.axis('off')

        plt.show()



        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(depthfin,cmap ='gray')
        ax.set_title('out2')
        ax.axis('off')
        plt.show()


        return iy, ix, Dy, Dx, widthbinx,widthbiny, iy1, ix1, Dy1, Dx1
    #rospy.spin()
