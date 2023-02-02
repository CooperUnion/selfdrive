import rospy
import cand

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray

# CAN Messages
EncoderCAN = 'DBW_EncoderData'
VelocityCAN = 'DBW_VelocityCmd'

# ROS Topics
PPTwist = '/cmd_vel'
EncoderTicks = '/encoder_ticks'

class ROStouCAN:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        rospy.Subscriber(PPTwist, Twist, self.callback)
    def callback(self, msg):
        while True:
            rospy.loginfo("I heard %s", msg.linear.x)
            self.bus.send(VelocityCAN, {
                'linearVelCmd': msg.linear.x,
                'angularVelCmd': msg.angular.z
            })

class CANtouROS:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        self.pub = rospy.Publisher(EncoderTicks, UInt16MultiArray, queue_size=2)
        self.rate = rospy.Rate(1)
    def publish(self):
        data = self.bus.get_data(EncoderCAN)
        # ticks = [data['encoderLeft'], data['encoderRight']]
        # Not sure why this not working ^
        ticks = [15000, 121]
        msgEncoder = UInt16MultiArray(data=ticks)
        while not rospy.is_shutdown():
            self.pub.publish(msgEncoder)
        
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