#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('stereo_image')

import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
from matplotlib import pyplot as plt
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from stereo_params.cfg import StereoParamsConfig as ConfigType
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

class StereoImageNode(object):

    def __init__(self):
        self.numDisparities = rospy.get_param('~numDisparities', 16)
        self.blockSize = rospy.get_param('~blockSize', 15)
        self.baseline = 0.54
        self.focal = 721
        self.image_width = 1242
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
        self.depth_factor = self.baseline * self.focal / self.image_width
        self.stereo = cv2.StereoBM_create(
            numDisparities=self.numDisparities, 
            blockSize=self.blockSize)
        self.bridge = CvBridge()

        left_image_sub = message_filters.Subscriber('/kitti/camera_gray_left/image_raw', Image)
        right_image_sub = message_filters.Subscriber('/kitti/camera_gray_right/image_raw', Image)
        ts = message_filters.TimeSynchronizer([left_image_sub, right_image_sub], 10)
        ts.registerCallback(self.callback)

        self.disparity_image_pub = rospy.Publisher("/kitti/disparity_image", Image, queue_size=10)
        self.depth_image_pub = rospy.Publisher("/kitti/depth_image", Image, queue_size=10)
        self.stereo_pointcloud_pub = rospy.Publisher("/kitti/stereo_pointcloud", PointCloud, queue_size=10)

    def callback(self, left_image, right_image):
        try:
            left_cv_image = self.bridge.imgmsg_to_cv2(left_image, "mono8")
        except CvBridgeError as e:
            print(e)
        try:
            right_cv_image = self.bridge.imgmsg_to_cv2(right_image, "mono8")
        except CvBridgeError as e:
            print(e)

        disparity = self.stereo.compute(left_cv_image, right_cv_image)
        depth = self.depth_factor * disparity
        stereo_pointcloud = PointCloud()
        for i, row in enumerate(depth):
            for j, p in enumerate(row):
                stereo_pointcloud.points.append(Point32(i, j, p))


        disparity =  disparity.astype(np.uint16)
        depth =  depth.astype(np.uint16)
        try:
            self.disparity_image_pub.publish(
                self.bridge.cv2_to_imgmsg(disparity, "mono16"))
        except CvBridgeError as e:
            print(e)
        try:
            self.depth_image_pub.publish(
                self.bridge.cv2_to_imgmsg(depth, "mono16"))
        except CvBridgeError as e:
            print(e)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = left_image.header.frame_id
        stereo_pointcloud.header = header
        
        self.stereo_pointcloud_pub.publish(stereo_pointcloud)
   
    def reconfigure_cb(self, config, dummy):

        rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
        self.numDisparities = config["numDisparities"]
        self.blockSize = config["blockSize"]

        # Check to see if node should be started or stopped.
        # if self.enable != config["enable"]:
        #     if config["enable"]:
        #         self.start()
        #     else:
        #         self.stop()
        # self.enable = config["enable"]

        # Return the new variables.
        return config

if __name__ == '__main__':
    
    rospy.init_node('stereo_image_node')
    try:
        StereoImageNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
