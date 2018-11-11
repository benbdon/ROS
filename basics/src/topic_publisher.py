#!/usr/bin/env python

import rospy #required for every ROS Python node
from std_msgs.msg import Int32 # define message type

rospy.init_node('topic_publisher') # initialize node with the name topic_publisher

pub = rospy.Publisher('counter', Int32, queue_size = 10, latch = True) # publish to a topic called counter

rate = rospy.Rate(2) # set a 2 Hz update rate on this node

count = 0
while not rospy.is_shutdown(): # run until shutdown
    pub.publish(count) 
    count += 1
    rate.sleep() 