#! /usr/bin/env python

import matplotlib.pyplot as plt
from utils.dataset_processing import grasp, image
from utils.dataset_processing import evaluation1, grasp
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
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray

base_path = "/home/mateo/catkin_ws/src/grasping/src/img"
class image_converter:

    def __init__(self, args):
        self.index = 0
        if len(sys.argv) > 1:
            self.index = int(sys.argv[1])
        rospy.init_node('save_img')
        self.bridge = CvBridge()

        while not rospy.is_shutdown():
            raw_input()
            image_sub = rospy.Subscriber("/camera/depth/image_raw",Image)
            print "hola"
            data = rospy.wait_for_message('/camera/depth/image_raw', Image)
            print "mundo"

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            except CvBridgeError as e:
                print(e)
           
            print "Saved to: ", base_path+str(self.index)+".jpg"
            #cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB, cv_image)
            cv2.imwrite('/home/mateo/catkin_ws/src/grasping/src/img1.png', cv_image)#*255)
            self.index += 1
        rospy.spin()



if __name__=='__main__':
    image_converter(sys.argv)
