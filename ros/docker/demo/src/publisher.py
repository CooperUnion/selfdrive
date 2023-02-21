#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String

if __name__ == '__main__':

    rospy.init_node('publisher', anonymous=True)
    number = rospy.Publisher('/number', Int32, queue_size=1)
    string = rospy.Publisher('/string', String, queue_size=1)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        number.publish(5)
        string.publish("Tech Team!!!")
        rate.sleep()
