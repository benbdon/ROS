#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('red_light_green_light')
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

red_light_twist = Twist() # The message constructor sets all fields to zero by default
green_light_twist = Twist()
green_light_twist.linear.x = 0.25 # This makes the TurtleBot drive in a straight line forward at 0.25 m/s.

driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist) # Publish the command velocities to achieve the two robot states we want
        #rospy.loginfo('Moving')
    else:
        cmd_vel_pub.publish(red_light_twist) # Publish the command velocities to achieve the two robot states we want
        #rospy.loginfo('Stopped')
    if light_change_time < rospy.Time.now(): #Checks the time and toggle between the red and green light
        rospy.logwarn('Switch')
        driving_forward = not driving_forwardd
        light_change_time  = rospy.Time.now() + rospy.Duration(3) # This allows us to change states after three seconds

    rate.sleep()