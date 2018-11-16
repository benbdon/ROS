#!/usr/bin/env python

import rospy

from basics.srv import WordCount,WordCountResponse # since this is the node that hosts the service, 
                                                # seems likely that a node which uses this would
                                                # import WordCount and WordCountRequest

def count_words(request): #call back function that counts words in a string
    return WordCountResponse(len(request.words.split()))

rospy.init_node('service_server') #initialize node

service = rospy.Service('word_count', WordCount, count_words) # advertise the service

rospy.spin() #keep node alive (watching for callbacks) until shutdown