#!/usr/bin/env  python
import rospy

import time
import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def do_timer(goal):
    start_time = time.time()
    update_count = 0

    if goal.time_to_wait.to_sec() > 60.0:
        result = TimerResult()
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count
        server.set_aborted(result, "Timer aborted due to too-long wait") # error handling to prevent timer being used for long waits
        return
    
    while (time.time() - start_time) < goal.time_to_wait.to_sec():
        if server.is_preempt_requested(): # returns true if client requests a new goal (another error check)
            result = TimerResult() # create a TimerResult message object
            result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
            result.updates_sent = update_count
            server.set_preempted(result, "Timer preempted")
            return
    
        feedback = TimerFeedback() # create a feedback object and update its values
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time) 
        feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed 
        server.publish_feedback(feedback) #publish the feedback
        update_count += 1

        time.sleep(1.0) # this is bad since we could possibly sleep past the goal time

    result = TimerResult() # create a TimerResult message object
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = update_count
    server.set_succeeded(result, "Timer completed successfully")

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
server.start()
rospy.spin()