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

device = torch.device('cpu')
crop_size=400
MODEL_FILE = 'ggcnn2_093'
rgbo = []
rgbfin = []
#MODEL_FILE = 'jacquard_86'
#depthfin=cv2.resize(depthfin, (640, 480), interpolation = cv2.INTER_AREA)
model = torch.load(MODEL_FILE, map_location='cpu')

def process_depth_image(depth, crop_size, out_size=crop_size, return_mask=False, crop_y_offset=0):
    imh, imw = depth.shape
    print(depth.shape)

    with TimeIt('1'):
       y_off=0
       x_off=0
       #depth_crop = depth[20+y_off:460+y_off,100+x_off:540+x_off]
       #depth_crop = depth[170+x_off:470+x_off,90+y_off:390+y_off]
       depth_crop = depth[(imh - crop_size) // 2 + y_off:(imh - crop_size) // 2 + crop_size + y_off,
                           (imw - crop_size) // 2+x_off:(imw - crop_size) // 2 + crop_size+x_off]

    print(depth_crop.shape)


    # depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

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
    #print(depth_crop.shape)

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


#def get_depth(depthorig, rot=0, zoom=1.0):
#        depth_img = image.DepthImage.from_tiff(depthorig)
        #center, left, top = depth_img._get_crop_attrs(0)
        #depth_img.rotate(0.8, [320,240])
        #depth_img.crop((top, left), (min(480, top + self.output_size), min(640, left + self.output_size)))
#        depth_img.normalise()
        #depth_img.zoom(zoom)
        #depth_img.zoom(0.5)
#        return depth_img.img

def predict(depth, process_depth=True, crop_size=crop_size, out_size=crop_size, depth_nan_mask=None, crop_y_offset=0, filters=(5.0, 2.0, 2.0)):
    if process_depth:
        depth, depth_nan_mask = process_depth_image(depth, crop_size, out_size=out_size, return_mask=True, crop_y_offset=0)

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)
    #depth = cv2.blur(depth,(5,5))
    depthn = depth.copy()
    depthn[60:340,113:288] = 0.00948*3
    #depthn = ndimage.filters.gaussian_filter(depthn, 1)
    depth = depth - depthn -0.00948*3
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

class image_converter:

    def __init__(self, args):

        self.index = 0
        if len(sys.argv) > 1:
            self.index = int(sys.argv[1])
        rospy.init_node('save_img')
        bridge = CvBridge()

	crop_size=400
        #while not rospy.is_shutdown():
        raw_input()

        #color_sub = rospy.Subscriber("/camera/color/image_raw",Image)
        print "depth"
        rgbo = rospy.wait_for_message('/camera/color/image_raw1', Image)
	#depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image)
        print "depth"
	cmd_pub = rospy.Publisher('ggcnn/out/command1', Float32MultiArray, queue_size=1)

	#rgbo = rospy.wait_for_message('/camera/color/image_raw1', Image)
        deptho = rospy.wait_for_message('/camera/depth/image_raw1', Image)
    #    rgbo = rospy.wait_for_message('/camera/rgb/image_color', Image)
        rgbfin = bridge.imgmsg_to_cv2(rgbo)
	depthfin = bridge.imgmsg_to_cv2(deptho)
  
        rgbfin1= cv2.cvtColor(rgbfin, cv2.COLOR_BGR2RGB)
        cv2.imwrite('rgb.png', rgbfin1)
    #for x in range(480):
    #    for y in range(480):
    #        if x <= 135:
    #            depthfin[x,y]=0.0705



        #cv2.imwrite('depthback.png', depthfin)
        #np.savetxt('depth.txt', depthfin)


            #try:
            #    cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            #except CvBridgeError as e:
            #    print(e)

            #print "Saved to: ", base_path+str(self.index)+".jpg"
            #cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB, cv_image)
            #cv2.imwrite('/home/mateo/catkin_ws/src/grasping/src/img1.png', cv_image)#*255)
            #self.index += 1
        ROBOT_Z = 0
        fx = 458.455478616934780
        cx = 343.645038678435410
        fy = 458.199272745572390
        cy = 229.805975111304460
        #MODEL_FILE = 'training2_084'


        points_out, ang_out, width_out, depth = predict(depthfin)

        grasps = grasp.detect_grasps(points_out, ang_out, 0.7, width_img=width_out, no_grasps=5)
	#if grasps == []:
	#	print('es')
	#imagen RGB
        #fileim=fname+'.png'
        #print(fileim)
        #rgb_img = image.Image.from_file(fileim)
        #rgb_img.img=cv2.resize(rgb_img.img, (640, 480), cv2.INTER_AREA)


        y_off=0
        x_off=0
        crop_size=400
        imh, imw = [480,640]
        rgbcrop = rgbfin[(imh - crop_size) // 2 + y_off:(imh - crop_size) // 2 + crop_size + y_off,
                           (imw - crop_size) // 2+x_off:(imw - crop_size) // 2 + crop_size+x_off]


        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(2, 2, 1)
        ax.imshow(depth, cmap='gray')
        for g in grasps:
            g.plot(ax)
        ax.set_title('Depth')
        ax.axis('off')


        ax = fig.add_subplot(2, 2, 2)
        plot = ax.imshow(points_out, cmap='jet', vmin=0, vmax=1)
	#grasps[0].plot(ax)
	#grasps[1].plot(ax)
        ax.set_title('quality')
        ax.axis('off')
        #plt.colorbar(plot)
        

	ax = fig.add_subplot(2, 2, 3)
	plot = ax.imshow(width_out, cmap='hsv', vmin=0, vmax=150)
	grasps[0].plot(ax)
	ax.set_title('width')
	ax.axis('off')
	plt.colorbar(plot)

        #ax = fig.add_subplot(2, 2, 4)
        #plot = ax.imshow(ang_out, cmap='hsv', vmin=-np.pi / 2, vmax=np.pi / 2)
        #grasps[0].plot(ax)
        #ax.set_title('ang')
        #ax.axis('off')
        #plt.colorbar(plot)

        ax = fig.add_subplot(2, 2, 4)
        plot = ax.imshow(rgbcrop)
        #for g in grasps:
        #    g.plot(ax)

        ax.set_title('RGB')
        ax.axis('off')

	#fig.savefig('plot1.png')

	#gr=GraspRectangles.load_from_array(grasps)
	#evaluation1.plot_output(width_out, depth, points_out , ang_out, no_grasps=1, rgb_img=None)
	#ENCONTRAR LA PROFUNDIDAD EN LA IMAGEN ORIGINAL
        for i in range(len(grasps)):
                 print('q',i, ': ',points_out[grasps[i].center[0], grasps[i].center[1]])
                 print('pix',i, ': ', grasps[i].center[0], grasps[i].center[1])
                 print('ang',i, ': ',ang_out[grasps[i].center[0], grasps[i].center[1]]*180/math.pi)
                 print('width',i, ': ',width_out[grasps[i].center[0], grasps[i].center[1]])
        print(len(grasps))

	#PIXEL CON VALOR MAXIMO
        #max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
        px,py=grasps[0].center
	max_pixel = np.array([px, py])
        ang = ang_out[max_pixel[0], max_pixel[1]]
        width = width_out[max_pixel[0], max_pixel[1]]
        print('qf: ',points_out[max_pixel[0], max_pixel[1]])
        print('viejopixel: ',max_pixel[0], max_pixel[1])
        crop_size=400
	y_off=0
        x_off=0
        #max_pixel = ((np.array(max_pixel) / 440.0 * crop_size) + np.array([(480 - crop_size)//2+y_off, (640 - crop_size) // 2+x_off]))
        max_pixel = ((np.array(max_pixel)) + np.array([(480 - crop_size)//2+y_off, (640 - crop_size) // 2+x_off]))
        max_pixel = np.round(max_pixel).astype(np.int)


        #scale=1024/480
        scale=1
        #grasps[0].center=[max_pixel[0]*1024/480, max_pixel[1]*1024/640]
        grasps[0].center=[max_pixel[0], max_pixel[1]]
        print('Nuevopixel: ',max_pixel[0], max_pixel[1])
        grasps[0].angle=ang
        grasps[0].length=width*scale
        grasps[0].width=width*scale/2
        #pfinal = [x, y, z, ang, width, depth_center]

        #ax = fig.add_subplot(2, 3, 6)
        #plot = ax.imshow(rgbfin)
        #grasps[0].plot(ax)
        #ax.set_title('RGBo')
        #ax.axis('off')
        #ENCONTRAR VALORES REALES DE IMAGEN
        #point_depth = depthfin[max_pixel[0], max_pixel[1]]

        point_depth = depthfin[px,py]
        x = (grasps[0].center[0] - cx)/(fx)*point_depth
        y = (grasps[0].center[1] - cy)/(fy)*point_depth
        z = point_depth
        x1 = (grasps[0].center[0]+width*math.cos(ang)/2 - cx)/(fx)*point_depth
        y1 = (grasps[0].center[1]+width*math.sin(ang)/2 - cy)/(fy)*point_depth
        x2 = (grasps[0].center[0]-width*math.cos(ang)/2 - cx)/(fx)*point_depth
        y2 = (grasps[0].center[1]-width*math.sin(ang)/2 - cy)/(fy)*point_depth

        rwidth =math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2))
        print('x: ', x)
        print('y: ', y)
        print('z: ', z)
        print('ang: ', ang*180/math.pi)
        print('width: ', width)
        print('rwidth: ', rwidth)

        cmd_msg = Float32MultiArray()
        cmd_msg.data = [x, y, z, ang, rwidth, point_depth]
        cmd_pub.publish(cmd_msg)

 	plt.show()
        #rospy.spin()





if __name__=='__main__':
    image_converter(sys.argv)
