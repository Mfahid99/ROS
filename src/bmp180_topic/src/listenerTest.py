#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def printstuff(msg):
    it=msg.data
    print(it)


rospy.init_node('bmp180Listener_node')
pub = rospy.Subscriber('bmp180',Int32,printstuff)
rospy.spin()

