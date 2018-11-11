#!/usr/bin/env python

import rospy #required for every ROS Python node
from std_msgs.msg import Int32 # define message type

rospy.init_node('topic_publisher') #initialize node
pub = rospy.Publisher('counter', Int32, queue_size = 10) #publish to a topic called counter

rate = rospy.Rate(2) #publish at 2 Hz (twice per second)

count = 0
while not rospy.is_shutdown(): #run until shutdown
    pub.publish(count) # publish current value of counter
    count += 1 # increment counter
    rate.sleep() # sleep long enough to ensure this loop happens at 2 Hz


#test 