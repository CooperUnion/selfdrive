#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import cand

CANmessage = 'DBW_VelocityCmd'
ROStopic = '/cmd_vel'

class ROStouCAN:
    def __init__(self):
        self.bus = cand.client.Bus()
        rospy.Subscriber(ROStopic, Twist, self.callback)
    def callback(self, msg):
        self.bus.send(CANmessage, {
            'linearVelCmd': msg.linear.x,
            'angularVelCmd': msg.angular.z
        })
        while True:
            rospy.loginfo("I heard %s", msg.linear.x)
    
if __name__ == '__main__':
    rospy.init_node('roscan!', anonymous=True)
    ROStouCAN()
    rospy.spin()

# sudo apt install python3.9
# python3.9 -m pip install opencan-cand

# start redis container
# set up vcan on local machine sudo modprobe vcan
# start cand on local machine candump vcan0
# redis local host flag on cand --redis_host localhost