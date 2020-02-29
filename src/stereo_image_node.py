#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import roslib
roslib.load_manifest('stereo_image')

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from stereo_image.cfg import StereoParamsConfig as ConfigType
import cv2
from image_geometry import StereoCameraModel
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

import matplotlib.pyplot as plt

class StereoImageNode(object):

    def __init__(self):

        # Parameters
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_callback)
        self.numDisparities = self.convert_num_disparities(rospy.get_param('~numDisparities', 2))
        self.blockSize = self.convert_block_size(rospy.get_param('~blockSize', 8))

        # Stereo matcher and model
        self.block_matcher = cv2.StereoBM_create(
            numDisparities=self.numDisparities, 
            blockSize=self.blockSize)
        self.model = StereoCameraModel()
        # TODO sg_block_matcher_ = cv::StereoSGBM::create(1, 1, 10);
        
        # Subscriber
        self.bridge = CvBridge()
        left_image_sub = message_filters.Subscriber('/kitti/camera_gray_left/image_raw', Image)
        right_image_sub = message_filters.Subscriber('/kitti/camera_gray_right/image_raw', Image)
        left_caminfo_sub = message_filters.Subscriber('/kitti/camera_gray_left/camera_info', CameraInfo)
        right_caminfo_sub = message_filters.Subscriber('/kitti/camera_gray_right/camera_info', CameraInfo)
        ts = message_filters.TimeSynchronizer([left_image_sub, right_image_sub, left_caminfo_sub, right_caminfo_sub], 10)
        ts.registerCallback(self.callback)

        # Publisher
        self.disparity_image_pub = rospy.Publisher("/kitti/disparity_image", Image, queue_size=10)
        self.depth_image_pub = rospy.Publisher("/kitti/depth_image", Image, queue_size=10)
        self.stereo_pointcloud_pub = rospy.Publisher("/kitti/stereo_pointcloud", PointCloud, queue_size=10)

    def callback(self, left_image, right_image, left_caminfo, right_caminfo):

        self.model.fromCameraInfo(left_caminfo, right_caminfo)

        try:
            left_cv_image = self.bridge.imgmsg_to_cv2(left_image, "mono8")
        except CvBridgeError as e:
            print(e)
        try:
            right_cv_image = self.bridge.imgmsg_to_cv2(right_image, "mono8")
        except CvBridgeError as e:
            print(e)

        disparity = self.processDisparity(left_cv_image, right_cv_image)

        pointcloud = self.processPointCloud(disparity, left_image.header.frame_id)

    def processDisparity(self, left_image, right_image):

        disparity16 = self.block_matcher.compute(left_image, right_image)
        rospy.loginfo("DISP16 %d %d" % (np.amin(disparity16), np.amax(disparity16)))
        rospy.loginfo(disparity16.dtype)
        # We convert from fixed-point to float disparity and also adjust for any x-offset between
        # the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
        # rospy.loginfo("%f %f" % (self.model.left.cx(), self.model.right.cx()))
        disparity = np.float32(disparity16) * (1.0 / 16) - (self.model.left.cx() - self.model.right.cx())
        rospy.loginfo("DISP   %d %d" % (np.amin(disparity), np.amax(disparity)))
        rospy.loginfo(disparity.dtype)

        disparity8 = (disparity16 + 15).astype(np.uint8)
        rospy.loginfo("DISP8  %d %d" % (np.amin(disparity8), np.amax(disparity8)))
        rospy.loginfo(disparity8.dtype)

        try:
            self.disparity_image_pub.publish(
                self.bridge.cv2_to_imgmsg(disparity8, "mono8"))
        except CvBridgeError as e:
            print(e)

        return disparity

    def processPointCloud(self, disparity, frame_id):

        stereo_pointcloud = PointCloud()
        counter = 0
        min_x = 1000
        max_x = 0
        min_y = 1000
        max_y = 0
        min_z = 1000
        max_z = 0
        depth_image = np.zeros(disparity.shape)
        for u, row in enumerate(disparity):
            for v, pixel in enumerate(row):
                if pixel > 0:
                    point = self.model.projectPixelTo3d((u,v), pixel)
                    depth_image[u][v] = point[2]
                    point32 = Point32(point[0], point[1], point[2])
                    stereo_pointcloud.points.append(point32)
                    counter +=1
                    min_x = min(min_x, point32.x)
                    max_x = max(max_x, point32.x)
                    min_y = min(min_y, point32.y)
                    max_y = max(max_y, point32.y)
                    min_z = min(min_z, point32.z)
                    max_z = max(max_z, point32.z)
                    #if counter % 10000 == 0:
                    #    print(u,v,pixel,point)
        rospy.loginfo("Min Max %f %f %f %f %f %f" % (min_x, max_x, min_y, max_y, min_z, max_z))
        rospy.loginfo("Depth completition rate %f", counter / disparity.shape[0] / disparity.shape[1])

        plt.figure(1)
        plt.subplot(211)
        plt.imshow(disparity, 'gray')
        plt.subplot(212)
        plt.imshow(depth_image, vmax=70)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()
        
        depth_image8 =  depth_image.astype(np.uint8)
        try:
            self.depth_image_pub.publish(
                self.bridge.cv2_to_imgmsg(depth_image8, "mono8"))
        except CvBridgeError as e:
            print(e)

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        stereo_pointcloud.header = header
        self.stereo_pointcloud_pub.publish(stereo_pointcloud)

        return stereo_pointcloud

   
    def reconfigure_callback(self, config, dummy):

        self.numDisparities = self.convert_num_disparities(config["numDisparities"])
        self.blockSize = self.convert_block_size(config["blockSize"])
        self.block_matcher = cv2.StereoBM_create(
            numDisparities=self.numDisparities, 
            blockSize=self.blockSize)
        rospy.loginfo("""Reconfigure Request: {numDisparities}, {blockSize},""".format(**config))
        # Check to see if node should be started or stopped.
        # if self.enable != config["enable"]:
        #     if config["enable"]:
        #         self.start()
        #     else:
        #         self.stop()
        # self.enable = config["enable"]

        # Return the new variables.
        return config

    def convert_num_disparities(self, numDisparities):
        return 16 * numDisparities

    def convert_block_size(self, blockSize):
        return 2 * blockSize + 5

if __name__ == '__main__':
    
    rospy.init_node('stereo_image_node')
    try:
        StereoImageNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
