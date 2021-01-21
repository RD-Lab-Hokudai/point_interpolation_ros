#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
from open3d_test.msg import PointsImages

combined_pub = rospy.Publisher(
    '/adapter/points_images', PointsImages, queue_size=1
)

thermal_cache = None
rgb_front_cache = None
rgb_right_cache = None
rgb_back_cache = None
rgb_left_cache = None


def thermalCallback(image):
    global thermal_cache
    thermal_cache = image


def rgbFrontCallback(image):
    global rgb_front_cache
    rgb_front_cache = image


def rgbRightCallback(image):
    global rgb_right_cache
    rgb_right_cache = image


def rgbBackCallback(image):
    global rgb_back_cache
    rgb_back_cache = image


def rgbLeftCallback(image):
    global rgb_left_cache
    rgb_left_cache = image


def lidarCallback(points):
    global thermal_cache, rgb_front_cache, rgb_right_cache, rgb_back_cache, rgb_left_cache
    msg = PointsImages()
    msg.thermal = thermal_cache
    msg.rgb_front = rgb_front_cache
    msg.rgb_right = rgb_right_cache
    msg.rgb_back = rgb_back_cache
    msg.rgb_left = rgb_left_cache
    msg.points = points
    combined_pub.publish(msg)


rospy.init_node('adapter', anonymous=True)

thermal_sub = rospy.Subscriber(
    '/thermal_corrected', Image, thermalCallback, queue_size=1, buff_size=5000000)
rgb_front_sub = rospy.Subscriber(
    '/usb_cam4/image_raw', Image, rgbFrontCallback, queue_size=1, buff_size=5000000)
rgb_right_sub = rospy.Subscriber(
    '/usb_cam4/image_raw', Image, rgbRightCallback, queue_size=1, buff_size=5000000)
rgb_back_sub = rospy.Subscriber(
    '/usb_cam4/image_raw', Image, rgbBackCallback, queue_size=1, buff_size=5000000)
rgb_left_sub = rospy.Subscriber(
    '/usb_cam4/image_raw', Image, rgbLeftCallback, queue_size=1, buff_size=5000000)
lidar_sub = rospy.Subscriber(
    '/os1_cloud_node/points', PointCloud2, lidarCallback, queue_size=1, buff_size=5000000)

rospy.spin()
