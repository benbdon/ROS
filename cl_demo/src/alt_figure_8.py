#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,cos,sin,pi
from turtlesim.srv import TeleportAbsolute

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        #self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    #def callback(self, data):
    #    self.pose = data
    #    self.pose.x = round(self.pose.x, 4)
    #    self.pose.y = round(self.pose.y, 4)

    def move2goal(self):
        #goal_pose = Pose()
        pub = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        pub(5.5,5.5,pi/8)
        vel_msg = Twist()
        T=rospy.get_param("~time", 15) #seconds
        t_0 = rospy.Time.now().to_sec()
        elapsed_t=0
        t_prev = 0

        while elapsed_t <= T:

            t = rospy.Time.now().to_sec() - t_0
            dt = t - t_prev
            #compute instantaneous x and it's first and second derivatives
            x = 5.5 + 3*sin(4*pi*t/T)
            xdot = 12*pi/T*cos(4*pi*t/T)
            xddot = -48*pow(pi,2)/pow(T,2)*sin(4*pi*t/T)

            #compute instantaneous y and it's first and second derivatives
            y = 5.5 + 3*sin(2*pi*t/T)
            ydot = 6*pi/T*cos(2*pi*t/T)
            yddot = -12*pow(pi,2)/pow(T,2)*sin(2*pi*t/T)

            #compute linear velocity
            velturt = sqrt(pow(xdot,2)+pow(ydot,2))

            #compute omega ie theta_dot
            omegaturt = -(xddot*ydot - xdot*yddot)/(pow(xdot,2)+pow(ydot,2))

            #linear velocity in the x-axis:
            vel_msg.linear.x = velturt
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = omegaturt

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
        turt = turtlebot()
        turt.move2goal()

    except rospy.ROSInterruptException: pass
