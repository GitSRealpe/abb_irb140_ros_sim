#! /usr/bin/env python

import time

import numpy as np
#import tensorflow as tf
#from keras.models import load_model
import sys
import cv2
import scipy.ndimage as ndimage
#from skimage.draw import circle
#from skimage.feature import peak_local_max

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray

#bridge = CvBridge()


#rospy.init_node('depth_converter')
#depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image,queue_size=1)
#depth = bridge.imgmsg_to_cv2(depth_message)

class image_converter:

   def __init__(self):
     self.image_pub = rospy.Publisher("image_topic_2",Image,  queue_size=1)

     cv2.namedWindow("Image window", 100)
     self.bridge = CvBridge()
     #self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
     #self.image_sub = rospy.wait_for_message("/camera/depth/image_raw",Image,timeout=None)
     data = rospy.wait_for_message("/camera/depth/image_raw",Image,timeout=None) 
     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
     cv2.ShowImage("Image window", cv_image)
     cv2.WaitKey(3) 
   def callback(self,data):
      try:
       cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      except CvBridgeError, e:
       print e
 
      (cols,rows) = cv2.GetSize(cv_image)
      if cols > 1000 and rows > 1000 :
        cv2.Circle(cv_image, (200,200), 100, 255)
      #print(cv_image.height)
      #print(cv_image.width) 
      cv2.ShowImage("Image window", cv_image)
      cv2.WaitKey(3)
 
      try:
        self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
      except CvBridgeError, e:
        print e
 
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
