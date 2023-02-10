import rospy
import cand
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray

# CAN Messages
EncoderCAN = 'CTRL_EncoderData'
VelocityCAN = 'DBW_VelocityCmd'
SteerCAN = 'STEER_SteeringCmd'

# ROS Topics
PPTwist = '/cmd_vel'
EncoderTicks = '/encoder_ticks'

class ROStouCAN:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        rospy.Subscriber(PPTwist, Twist, self.callback)
    def callback(self, msg):
        while True:
            angle = 0
            if msg.linear.x != 0 and msg.angular.z != 0:
                angle = -np.arctan(msg.angular.z * 1.8 / msg.linear.x)
            else:
                rospy.loginfo(f"Warning: invalid steering angle combo:lin.x:{msg.linear.x}, ang.z:{msg.angular.z}")
                angle = 0
            rospy.loginfo("I heard %s", msg.linear.x)
            self.bus.send(VelocityCAN, {
                'DBW_linearVelCmd': msg.linear.x
            })
            self.bus.send(SteerCAN, {
                'STEER_angleCmd': angle
            })

class CANtouROS:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        self.msg = UInt16MultiArray()
        self.pub = rospy.Publisher(EncoderTicks, UInt16MultiArray, queue_size=2)
        self.rate = rospy.Rate(1)
    def publish(self):
        while not rospy.is_shutdown():
            data = self.bus.get_data(EncoderCAN)
            if data is None:
                continue
            self.msg.data = [data['CTRL_encoderLeft'], data['CTRL_encoderRight']]
            self.pub.publish(self.msg)
        
if __name__ == '__main__':
    rospy.init_node('roscan', anonymous=True)
    can_publisher = ROStouCAN()
    ros_publisher = CANtouROS()
    ros_publisher.publish()
    rospy.spin()

# sudo apt install python3.9
# python3.9 -m pip install opencan-cand

# start redis container
# set up vcan on local machine 
# sudo modprobe vcan
# sudo ip link add dev vcan0 type vcan
# sudo ip link set up vcan0
# start cand on local machine candump vcan0
# redis local host flag on cand --redis_host localhost