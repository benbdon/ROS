#!/usr/bin/env python

import rospy

from basics.srv import WordCount,WordCountRequest

import sys

rospy.init_node('service_client')

rospy.wait_for_service('word_count') # wait until service is advertised

word_counter = rospy.ServiceProxy('word_count', 
WordCount) # turn the service into a local function

words = ' '.join(sys.argv[1:]) # read in all the arguments from console

request = WordCountRequest(words) # construct a service request object

word_count = word_counter(request) # call the service name wrapped up like a local function

print words, '->', word_count.count # print the result of the local function