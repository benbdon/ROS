#!/usr/bin/env python
import rospy
from std_msgs.msg import String
def talker():
    rospy.init_node('talker_param', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) #10hz
    msg = rospy.get_param('~name',default='Anybody') #1 Note that line #1 gets the private parameter name or defaults to "Anybody" if that parameter is not available.
    while not rospy.is_shutdown():
        hello_str = "Hello I'm %s %s" % (msg,rospy.get_time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
