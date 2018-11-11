#!/usr/bin/env python

import rospy 
from std_msgs.msg import Int32 

def callback(msg): #print message every time it updates
    print msg.data 

rospy.init_node('topic_subscriber') # initialize node

sub = rospy.Subscriber('counter', Int32, callback) # call callback function every time you get a new message on topic 'counter'

rospy.spin() # give control to ROS - only return when the node is ready to shutdown (shorthand for a top-level while loop)