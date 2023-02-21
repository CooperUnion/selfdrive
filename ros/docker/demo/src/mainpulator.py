#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16

class Subscriber:
    def __init__(self):
        rospy.Subscriber('/number', Int16, callback)
        self.num = 0
    def callback(self,msg):
        rospy.loginfo("I heard %s", msg.data)
        self.num = msg.data

if __name__ == 'main':

    rospy.init_node('subscriber', anonymous=True)
    
    sub = Subscriber()

    pub_add = rospy.Publisher('/number/add', Int16)
    pub_multi = rospy.Publisher('/number/mult', Int16)

    add = Int16()
    multi = Int16()

    add = 1
    multi = 1

    while not rospy.is_shutdown():
        add = add + sub.num
        multi = add + multi
        pub_add.publish(add)
        pub_add.publish(multi)