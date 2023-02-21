#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16

class Subscriber:
    def __init__(self):
        rospy.Subscriber('/number', Int16, callback)
    def callback(self,msg):
        rospy.loginfo("I heard %s", msg.data)

if __name__ == 'main':

    rospy.init_node('subscriber', anonymous=True)
    sub = Subscriber()
    rospy.spin()
        