#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import cand

CANmessage = 'DBW_VelocityCmd'
ROStopic = '/cmd_vel'


class CANtouROS:
    def __init__(self):
        self.bus = cand.client.Bus()
        pub = rospy.Publisher(ROStopic, Twist, self.callback)
    def callback(self,msg):
        self.bus.send(CANmessage, {
            'linearVelCmd': 1.0,
            'angularVelCmd': 1.0
        })
        data_rec = self.bus.get(CANmessage)
        data_time, data_data = data_rec
        msg.linear.x = data_data['linearVelCmd']
        msg.angular.z = data_data['angularVelCmd']

        

if __name__ == '__main__':
    rospy.init_node('canros?', anonymous=True)
    CANtouROS()
    rospy.spin()

        