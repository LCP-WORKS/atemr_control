#! /usr/bin/env python3

import sys, select, tty, termios, time
import rospy
from rospy.core import is_shutdown
from std_msgs.msg import String

if  __name__ == '__main__':
    key_pub = rospy.Publisher('keys', String, queue_size=1)
    rospy.init_node('keyboard_driver')
    rate = rospy.Rate(100)
    zero_msg = String()
    zero_msg.data = "x"
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rospy.loginfo("Publishing keystrokes. Press Ctrl-C to exit...")
    start_time = time.time()
    while (not rospy.is_shutdown()):
        if (select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]):
            key_pub.publish(sys.stdin.read(1))
            start_time = time.time()
        else:
          if((time.time() - start_time)*1000.0 > 30.0):
            key_pub.publish(zero_msg)
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
