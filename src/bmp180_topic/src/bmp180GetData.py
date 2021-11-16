#!/usr/bin/env python
# https://github.com/m-rtijn/bmp180/blob/master/bmp180/bmp180.py
import rospy
import bmp180
from std_msgs.msg import Int32
rospy.init_node('bmp180_node')
pub = rospy.Publisher('bmp180',Int32,queue_size=1)
rate=rospy.Rate(2)
count = 0
bmp180_=bmp180()
while not rospy.is_shutdown():
        pub.publish(count)
        count+=1
        rate.sleep()


