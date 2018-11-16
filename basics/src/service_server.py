#!/usr/bin/env python

import rospy

from basics.srv import WordCount,WordCountResponse # since this is the node that hosts the service, 
                                                # seems likely that a node which uses this would
                                                # import WordCount and WordCountRequest

def count_words(request): #call back function that counts words in a string
    return WordCountResponse(len(request.words.split())) #takes parameters that match the service-definition
                                                        #unsigned int

rospy.init_node('service_server') #initialize node

service = rospy.Service('word_count', WordCount, count_words) # advertise the service (assign a name for the
                                                            # service, map it to a service-defintion file of I/O 
                                                            # names and types, callback function name)

rospy.spin() #keep node alive (watching for callbacks) until shutdown