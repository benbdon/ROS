#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #Import the LaserScan message since we'll be using it

class Wandering(object):
    def __init__(self):
        ''' Initialize parameters'''
        rospy.init_node('wander')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.twist = Twist()
        self.state_change_time = rospy.Time.now()  #These are the two variables we'll use for our controller logic, driving state and time
        self.driving_forward = True #These are the two variables we'll use for our controller logic, driving state and time
        self.rate = rospy.Rate(10)

    def scan_callback(self,msg):
        range_ahead = min(msg.ranges) #This stores the minimum range our simulated laser scanner detects. We could decide to use the middle element of the ranges array instead with range_ahead = msg.ranges[len(msg.ranges)/2] . Alternatively we could use msg.range_min to use the minimum value the laser scanner is capable of sensing (assuming our driver has properly set this). How would these choices affect the TurtleBot's behavior.
        rospy.loginfo("range ahead: %0.1f" % range_ahead)
        if self.driving_forward: #Set the command velocities for moving forward at 0.2 m/s
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            rospy.loginfo('Moving')
            if range_ahead < 1.2 or rospy.Time.now() > self.state_change_time: #If there's no obstacle within 1.2m or the robot has been moving for 10 seconds, stop moving.
                self.driving_forward = False
                self.state_change_time = rospy.Time.now() + rospy.Duration(5) #Stop moving for 5 seconds.
        else:
            self.twist.angular.z = 1 #Spin on the spot for 5 seconds
            self.twist.linear.x = 0
            rospy.loginfo('Spinning')
            if rospy.Time.now() > self.state_change_time: #Check that 5 seconds have elapsed
                self.driving_forward = True
                rospy.loginfo('Switch')
                self.state_change_time = rospy.Time.now() + rospy.Duration(10) #Change states after 10 seconds


        self.cmd_vel_pub.publish(self.twist) #Publish the messages
        self.rate.sleep()

def main():
    wandering = Wandering()
    rospy.Subscriber('/scan', LaserScan, wandering.scan_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main()