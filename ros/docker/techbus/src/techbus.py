#!/usr/bin/env python3

import rospy
import cand
import math
from math import atan2

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray, String

# CAN Messages
EncoderCAN = 'CTRL_EncoderData'
VelocityCAN = 'DBW_VelocityCommand'
SteerCAN = 'DBW_SteeringCommand'

# ROS Topics
PPTwist = '/cmd_vel'
EncoderTicks = '/encoder_ticks'


# Creates subscriber object that listens for the /cmd_vel topic
# Whenever the subscriber receives data, the callback function runs
# The callback function extracts a usable linear velocity and steering angle from the data
# It sends these commands over the DBW_VelocityCmd and DBW_SteeringCommand CAN messages
class ROStouCAN:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        rospy.Subscriber(PPTwist, Twist, self.callback)

    def callback(self, msg):
        angle = 0
        if msg.linear.x != 0 and msg.angular.z != 0:
            angle = -atan2(msg.angular.z * 1.8, msg.linear.x)
        else:
            rospy.loginfo(
                f"Warning: invalid steering angle combo:lin.x:{msg.linear.x}, ang.z:{msg.angular.z}"
            )
            angle = 0
        self.bus.send(VelocityCAN, {'DBW_linearVelocity': msg.linear.x})
        self.bus.send(SteerCAN, {'DBW_steeringAngleCmd': angle})


# Establishes publisher for the /encoder_ticks topic
# Receives data from the CAN message CTRL_EncoderData
# Publishes that data as a ROS topic
class CANtouROS:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        self.msg = UInt16MultiArray()
        self.pub = rospy.Publisher(
            EncoderTicks, UInt16MultiArray, queue_size=2
        )
        self.rate = rospy.Rate(50)

    def publish(self):
        data = self.bus.get_data(EncoderCAN)
        if data is None:
            return
        self.msg.data = [data['CTRL_encoderLeft'], data['CTRL_encoderRight']]
        self.pub.publish(self.msg)
        self.rate.sleep()


# Initialize the node
# Establish instances of the subscriber and publisher objects
# Continuously publish until the program shuts down
if __name__ == '__main__':
    rospy.init_node('roscan', anonymous=True)
    # pub = rospy.Publisher('/encoder_string', String, queue_size=2)
    can_publisher = ROStouCAN()
    ros_publisher = CANtouROS()
    while not rospy.is_shutdown():
        # pub.publish("hello")
        ros_publisher.publish()

# sudo apt install python3.9
# python3.9 -m pip install opencan-cand

# start redis container
# set up vcan on local machine
# sudo modprobe vcan
# sudo ip link add dev vcan0 type vcan
# sudo ip link set up vcan0
# start cand on local machine candump vcan0
# redis local host flag on cand --redis_host localhost
