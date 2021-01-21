#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
from open3d_test.msg import PointsImagesFront

thermal_pub = rospy.Publisher(
    '/separater/thermal', Image, queue_size=1)
rgb_front_pub = rospy.Publisher(
    '/separater/rgb_front', Image, queue_size=1)
'''
rgb_right_pub = rospy.Publisher(
    '/separater/rgb_right', Image, queue_size=1)
rgb_back_pub = rospy.Publisher(
    '/separater/rgb_back', Image, queue_size=1)
rgb_left_pub = rospy.Publisher(
    '/separater/rgb_left', Image, queue_size=1)
'''
lidar_pub = rospy.Publisher(
    '/separater/points', PointCloud2, queue_size=1)


def dataCallback(pointsImages):
    thermal_pub.publish(pointsImages.thermal)
    rgb_front_pub.publish(pointsImages.rgb)
    '''
    rgb_right_pub.publish(pointsImages.rgb_right)
    rgb_back_pub.publish(pointsImages.rgb_back)
    rgb_left_pub.publish(pointsImages.rgb_left)
    '''
    lidar_pub.publish(pointsImages.points)


rospy.init_node('separater', anonymous=True)

data_sub = rospy.Subscriber(
    '/corrected/points_images_front', PointsImagesFront, dataCallback, queue_size=1, buff_size=5000000)
rospy.spin()
