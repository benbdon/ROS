#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import ApplyJointEffort

def set_joint_efforts():

    rospy.wait_for_service('/gazebo/apply_joint_effort')
    apply_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    joint_name = "joint1"
    effort = 10
    start_time = rospy.Duration.from_sec(0)
    duration = rospy.Duration.from_sec(1)
    joints = apply_effort_service(joint_name, effort, start_time, duration)
# rosservice call /gazebo/clear_joint_forces '{joint_name: joint2}'
if __name__ == '__main__':
    set_joint_efforts()
