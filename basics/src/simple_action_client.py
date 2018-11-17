#!/usr/bin/env python
import rospy

import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', TimerAction) # (name of action server, message type)
client.wait_for_server() # wait for server (note this is not service like we do for service calls)
goal = TimerGoal() # create a TimerGoal message object
goal.time_to_wait = rospy.Duration.from_sec(5.0) # set wait time
client.send_goal(goal) # send out the TimerGoal message object
client.wait_for_result() # blocks until a result message comes back
print('Time elapsed: %f'%client.get_result().time_elapsed.to_sec()) # print
