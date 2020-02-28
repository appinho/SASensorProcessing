#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from sensor_msgs.msg import Image

def callback(right_image, left_image):
    rospy.loginfo("I heard !")

def listener():
    rospy.loginfo("Listen")
    right_image_sub = message_filters.Subscriber('/kitti/camera_color_right/image_raw', Image)
    left_image_sub = message_filters.Subscriber('/kitti/camera_color_left/image_raw', Image)
    ts = message_filters.TimeSynchronizer([right_image_sub, left_image_sub], 10)
    ts.registerCallback(callback)

if __name__ == '__main__':
    rospy.loginfo("MAIN")
    rospy.init_node('pylistener')
    listener()
    rospy.spin()
