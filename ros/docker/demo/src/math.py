#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

class Subscriber:
    def __init__(self):
        rospy.Subscriber('/number', Int32, self.callback)
        self.number = 1
    def callback(self,msg):
        self.number = msg.data

if __name__ == "__main__":

    rospy.init_node('subscriber', anonymous=True)
    sub = Subscriber()
    pub_add = rospy.Publisher('/number/add', Int32, queue_size=1)    
    pub_multi = rospy.Publisher('/number/multi', Int32, queue_size=1)    
    rate = rospy.Rate(1)

    add = Int32()
    multi = Int32()

    add = 1
    multi = 1

    while not rospy.is_shutdown():
        add = add + sub.number
        multi = multi * sub.number
        pub_add.publish(add)
        pub_multi.publish(multi)
        rate.sleep()