#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class Subscribe:
    def __init__(self):
        self.sub_twist = rospy.Subscriber('/cmd_vel', Twist, self.callback) #orange line is the node
        #subscribing to twist message from cmd_vel
        self.sub_path = rospy.Subscriber('/global_planner/plan', Path, self.callback)
        #subscribing to global_planner node
    def callback(msg):
        while True:
            rospy.loginfo("I heard %s", msg.data) 

class Publish:
    def __init__(self):
        self.pose = pose
        self.pub_twist = rospy.Publisher('/pure_pursuit/twist', Twist, queue_size=2)
        #high level controls publishes our own twist message
        self.rate = rospy.Rate(2)
    def publish(self,pose):
        while not rospy.is_shutdown():
            self.pub_twist.publish(pose)
            #publishing the pose
    
class Controller:
    def __init__(self):
        self.sub = Subscribe()
        self.pub = Publish()
    def pure_pursuit(self):
        while True:
            self.sub.sub_twist.linear.x += 10
            self.sub.sub_twist.linear.y += 10

            #pose = {linear: {x: controllerx, y: contollery }}
            self.pub.publish(self.pose)
            

if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=True)
    Controller()
    rospy.spin()