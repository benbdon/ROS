#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from math import pow,atan2,sqrt,cos,sin,pi,acos

def joint_publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10, latch=True)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(50) # 50hz
    joint_str = JointState()
    joint_str.name = ['joint1', 'joint2', 'end_effector_joint']
    t0=rospy.Time.now().to_sec()
    t=0
    while t < 600:
        t =  rospy.Time.now().to_sec()-t0
        l1 = 1
        l2 = 1
        x = 0.5*cos(2*pi*t/5.0) + 1.25
        y = 0.5*sin(2*pi*t/5.0)
        beta = acos((l1**2 + l2**2 - x**2 - y**2)/(2*l1*l2))
        alpha = acos((x**2+y**2+l1**2-l2**2)/(2*l1*sqrt(x**2+y**2)))
        gamma = atan2(y,x)
        theta1 = gamma - alpha
        theta2 = pi - beta
        joint_str.position = [theta1,theta2,0]
        joint_str.header.stamp = rospy.Time.now()
        pub.publish(joint_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass
