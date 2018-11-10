#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from math import pow,atan2,sqrt,sin,pi

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

#    def get_distance(self, goal_x, goal_y):
#        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
#        return distance

    def move2goal(self):
        T = 15
        pub = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        pub(5.5,5.5,pi/2)
        goal_pose = Pose()
#        distance_tolerance = input("Set your tolerance:")
        vel_msg = Twist()
        t0 = rospy.Time.now().to_sec()
        t_prev = 0
        elapsed_t = 0
        while elapsed_t < T:

            #Time of new iteration
            t = rospy.Time.now().to_sec() - t0
            rospy.loginfo(t)
            #Time difference between iteration
            dt = t-t_prev

            #Update goal given t
            goal_pose.x = 5.5 + 3*sin(4*pi*t/T)
            goal_pose.y = 5.5 + 3*sin(2*pi*t/T)

            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z=atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta
            rospy.loginfo(vel_msg.angular.z)


            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            #Start time of current iteration
            t_prev = t

            #Summation of dt
            elapsed_t = elapsed_t + dt

        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()

    except rospy.ROSInterruptException: pass
