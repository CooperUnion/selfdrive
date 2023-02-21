#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

class Subscriber:
    def __init__(self):
        rospy.Subscriber('/number', Int32, self.callback)
    def callback(self,msg):
        rospy.loginfo("I received %s:", msg.data)

if __name__ == "__main__":

    rospy.init_node('subscriber', anonymous=True)
    sub = Subscriber()
    rospy.spin()