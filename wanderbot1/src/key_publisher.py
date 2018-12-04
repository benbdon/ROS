#!/usr/bin/env python
import sys,select,tty,termios
import rospy
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node("keyboard_driver")
    key_pub = rospy.Publisher('keys', String, queue_size=1)
    rate = rospy.Rate(100)
    #save attributes
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print "Publishing keystrokes. Press Ctrl-C to exit..."
    while not rospy.is_shutdown():
        if select.select([sys.stdin],[],[],0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()
    #Reset console to standard mode
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_attr)
