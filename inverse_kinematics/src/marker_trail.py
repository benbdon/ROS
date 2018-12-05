#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import sys
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

def talker():
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.init_node('coord_publisher')
    rate = rospy.Rate(20) # 20hz
    coordinate = Point()
    dots = Marker()
    listener = tf.TransformListener()
    counter = 0
    while not rospy.is_shutdown():
        try :
            position, quaternion = listener.lookupTransform('/base_link', '/end_effector', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        coordinate.x = position[0]
        coordinate.y = position[1]
        coordinate.z = position[2]
        dots.header.frame_id = '/base_link'
        dots.header.stamp = rospy.Time.now()
        dots.action = 0 #add a dot rather than removing or modifying
        dots.lifetime = rospy.Duration(1)
        dots.type = dots.POINTS
        dots.id = counter

        dots.points = [coordinate]
        dots.color.r = .1
        dots.color.g = .3
        dots.color.b = .5
        dots.color.a = .7

        dots.scale = Vector3(0.05, 0.05, 0.05)

        dots.pose.orientation.x = 0
        dots.pose.orientation.y = 0
        dots.pose.orientation.z = 0
        dots.pose.orientation.w = 1

        pub.publish(dots)
        rate.sleep()
        counter = counter + 1

if  __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
