#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from .grasp import GraspRectangles, detect_grasps
import cv2

def get_q(maxs):
    return maxs[0]


def plot_output(grasp_width_img, depth_img, grasp_q_img, grasp_angle_img, grasps, rgbfin, crop_size, y_off, x_off):
    """
    Plot the output of a GG-CNN
    :param rgb_img: RGB Image
    :param depth_img: Depth Image
    :param grasp_q_img: Q output of GG-CNN
    :param grasp_angle_img: Angle output of GG-CNN
    :param no_grasps: Maximum number of grasps to plot
    :param grasp_width_img: (optional) Width output of GG-CNN
    :return:
    """


    imh, imw = [480,640]
    #rgbcrop = rgbfin[(imh - crop_size) // 2 + y_off:(imh - crop_size)  // 2  + y_off, (imw - crop_size) // 2+x_off:(imw - crop_size) // 2 +x_off]
    rgbfin1 = rgbfin[151:373,215:532]

    #fig = plt.figure(figsize=(10, 10))
    #ax = fig.add_subplot(1, 3, 1)
    #ax.imshow(depth_img, cmap='gray')
    #for g in grasps:
    #    g.plot(ax)
    #ax.set_title('Depth')
    #ax.axis('off')



    #ax = fig.add_subplot(1, 3, 2)
    #plot = ax.imshow(grasp_q_img, cmap='jet', vmin=0, vmax=1)
    #ax.set_title('quality')
    #ax.axis('off')
    #plt.colorbar(plot)

    #ax = fig.add_subplot(2, 2, 3)
    #plot = ax.imshow(grasp_width_img, cmap='hsv', vmin=0, vmax=150)
    #ax.set_title('width')
    #ax.axis('off')
    #plt.colorbar(plot)

    #ax = fig.add_subplot(1, 3, 3)
    #ax.imshow(rgbfin1)
    #ax.set_title('rgb')
    #ax.axis('off')
    #plt.show()
    #grasps.sort(key = get_q)
    #print('grasps:', grasps)


    maxgrasps = []
    
    for i in range(len(grasps)):
                 maxgrasps.append([grasp_q_img[grasps[i].center], i])
                 print('q',i, ': ',grasp_q_img[grasps[i].center[0], grasps[i].center[1]])
                 print('pix',i, ': ', grasps[i].center[0], grasps[i].center[1])
                 print('ang',i, ': ',grasp_angle_img[grasps[i].center[0], grasps[i].center[1]]*180/math.pi)
                 print('width',i, ': ',grasp_width_img[grasps[i].center[0], grasps[i].center[1]])
    #print(len(grasps))
    #print('max_grasps: ', maxgrasps)
    maxgrasps.sort(key = get_q, reverse = True)
    #print('max_grasps: ', maxgrasps)
    max_val1=maxgrasps[0][1]
    #max_val=np.max(maxgrasps[0])
    #print(max_val1)


    #for i in range(len(grasps)):
    #            if maxgrasps[i] == max_val:
    #                ind.append(i)
    return maxgrasps


def calculate_iou_match(grasp_q, grasp_angle, ground_truth_bbs, no_grasps=1, grasp_width=None):
    """
    Calculate grasp success using the IoU (Jacquard) metric (e.g. in https://arxiv.org/abs/1301.3592)
    A success is counted if grasp rectangle has a 25% IoU with a ground truth, and is withing 30 degrees.
    :param grasp_q: Q outputs of GG-CNN (Nx300x300x3)
    :param grasp_angle: Angle outputs of GG-CNN
    :param ground_truth_bbs: Corresponding ground-truth BoundingBoxes
    :param no_grasps: Maximum number of grasps to consider per image.
    :param grasp_width: (optional) Width output from GG-CNN
    :return: success
    """

    if not isinstance(ground_truth_bbs, GraspRectangles):
        gt_bbs = GraspRectangles.load_from_array(ground_truth_bbs)
    else:
        gt_bbs = ground_truth_bbs
    gs = detect_grasps(grasp_q, grasp_angle, width_img=grasp_width, no_grasps=no_grasps)
    for g in gs:
        if g.max_iou(gt_bbs) > 0.25:
            return True
    else:
        return False
