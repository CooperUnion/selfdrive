#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16, String

if __name__ == 'main':
    
    rospy.init_node('publisher', anonymous=True)
    pubInt = rospy.Publisher('/number', Int16)
    pubStr = rospy.Publisher('/string', String)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        pubInt.publish(5)
        pubStr = rospy.Publisher('Hello World')
        rate.sleep()