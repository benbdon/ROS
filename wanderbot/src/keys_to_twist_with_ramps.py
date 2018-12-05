#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {'w':[0,1],'a':[-1,0],'s':[0,-1],'d':[1,0],'x':[0,0]} #1 A dictionary that maps key presses to movement
# w: Forward,
# s: Backwards,
# x: stop key
# a: rotate clockwise,
# d: anti-clockwise

class KeysTwistWithRamp(object):
    '''
    Ramp Motion Commands
    '''
    def __init__(self):
        rospy.init_node('keys_to_twist_with_ramps')
        self.g_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.g_last_twist = Twist()
        self.g_target_twist = Twist()
        self.g_last_twist_send_time = rospy.Time.now()
        self.rate = rospy.Rate(20) #2 Publish the Twist message to the topic at 20Hz
        self.g_vel_scales = [0.1, 0.1]  #3 Initialize the scale at a small value. Scaling in theory allows us to use this car on multiple platforms. 1 m/sm/s is fast for a TurtleBot, however it's slow for cars or a bigger mobile platforms.
        self.g_vel_ramps = [1,1] #4 Units: m/s^2

    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate):
        step = ramp_rate * (t_now - t_prev).to_sec() #5 Convert time to seconds
        sign = 1.0 if (v_target > v_prev) else -1.0
        error = math.fabs(v_target-v_prev)
        if error < step: #6 If we can get there within this timestep we're done
            return v_target
        else:
            return v_prev + sign * step #7 Take a step toward the target

    def ramped_twist(self, prev, target, t_prev, t_now, ramps): #8 In this function we instantiate a new twist. We then ramp the velocities coming in and split it into linear and angular velocities and return the new twist with the ramped velocities.
        tw = Twist()
        tw.angular.z = self.ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
        tw.linear.x =  self.ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
        return tw

    def send_twist(self): #9 In this function,the ramp is applied to the twist message being published
        t_now = rospy.Time.now()
        self.g_last_twist = self.ramped_twist(self.g_last_twist,self.g_target_twist,self.g_last_twist_send_time,t_now,self.g_vel_ramps)
        self.g_last_twist_send_time = t_now #10 Reset current time for next function call
        self.g_twist_pub.publish(self.g_last_twist)

    def keys_callback(self, msg):
        if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
            return  #11 Return nothing if the key pressed isn't in key_mapping
        vels = key_mapping[msg.data[0]] #12 Take the first key pressed as the key for the dictionary and assign the value to vels. Remember the dictionary command velocities are stored as a list
        self.g_target_twist.angular.z = vels[0] * self.g_vel_scales[0]
        self.g_target_twist.linear.x = vels[1] * self.g_vel_scales[1]

    def fetch_param(self,name,default): #13 A function to get parameter names or use default values
        if rospy.has_param(name): #14 Check for the existence of parameters and fetch parameter value from parameter server if they exist
            return rospy.get_param(name)
        else:
            rospy.logwarn("parameter %s not defined. Defaulting to %.3f" % (name, default)) #15 Warn if we don't have any parameters and return the default value we specify. logwarn() is a ROS logging cal that prints colourized text to the screen which is useful in debugging. Other logging calls are loginfo(), logwarn(), logerr() and logfatal()
            return default


def main():
    key_pub_rate = KeysTwistWithRamp()
    rospy.Subscriber('keys', String, key_pub_rate.keys_callback) #16 Subscribe to the keys topic and set the scale and ramp values to the parameter values(either the ones we specify or the default ones)
    key_pub_rate.g_vel_scales[0] = key_pub_rate.fetch_param('~angular_scale', 0.1)
    key_pub_rate.g_vel_scales[1] = key_pub_rate.fetch_param('~linear_scale', 0.1)
    key_pub_rate.g_vel_ramps[0]  = key_pub_rate.fetch_param('~angular_accel', 1.0)
    key_pub_rate.g_vel_ramps[1]  = key_pub_rate.fetch_param('~linear_accel', 1.0)

    while not rospy.is_shutdown():
        key_pub_rate.send_twist()
        key_pub_rate.rate.sleep()


if __name__=='__main__':
    main()
