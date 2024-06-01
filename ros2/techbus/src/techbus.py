import cand

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray, Float32MultiArray

from math import atan2

##### DEFINES ######

# CAN Messages
EncoderCAN = 'CTRL_EncoderData'
VelocityCAN = 'DBW_VelocityCommand'
SteerCAN = 'DBW_SteeringCommand'

# ROS Topics
cmdStanley = '/cmd_stanley'
EncoderTicks = '/encoder_ticks'

#####################


# Creates subscriber object that listens for the /cmd_vel topic
# Whenever the subscriber receives data, the callback function runs
# The callback function extracts a usable linear velocity and steering angle from the data
# It sends these commands over the DBW_VelocityCmd and DBW_SteeringCommand CAN messages
class ROStouCAN(Node):
    def __init__(self):
        super().__init__('ROStouCAN_Node')

        # self.bus = cand.client.Bus(redis_host='redis')
        self.bus = cand.client.Bus()
        self.encoder_ticks_subscription = self.create_subscription(
            Float32MultiArray, cmdStanley, self.cmd_callback, 10
        )
        self.encoder_ticks_subscription  # prevent unused variable warning

    # msg.data[0] is the steer_cmd in radians
    #msg.data[1] is the vel_cmd in m/s
    def cmd_callback(self, msg):
        angle = 0
        steer_cmd = msg.data[0]
        vel_cmd = msg.data[1]
        #bounding for safety reasons: 2.32 m/s is competition maximum
        if vel_cmd > 2.32:
            vel_cmd = 2.32

        if steer_cmd != 0 and vel_cmd != 0:
            angle = -atan2(steer_cmd * 1.8, vel_cmd)
        else:
            self.get_logger().info( 
                f"Warning: invalid steering angle combo: steer_cmd:{steer_cmd}, vel_cmd:{vel_cmd}"
            )
            angle = 0
        self.bus.send(VelocityCAN, {'DBW_linearVelocity': vel_cmd})
        self.bus.send(SteerCAN, {'DBW_steeringAngleCmd': angle})


# Establishes publisher for the /encoder_ticks topic
# Receives data from the CAN message CTRL_EncoderData
# Publishes that data as a ROS topic
class CANtouROS(Node):
    def __init__(self):
        super().__init__('CANtouROS_Node')

        # self.bus = cand.client.Bus(redis_host='redis')
        self.bus = cand.client.Bus()

        self.encoder_ticks_publisher = self.create_publisher(
            UInt16MultiArray, EncoderTicks, 10
        )

        # 50Hz publish timer
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = self.bus.get_data(EncoderCAN)
        if data is None:
            return
        encoder_msg = UInt16MultiArray()
        encoder_msg.data = [
            data['CTRL_encoderLeft'],
            data['CTRL_encoderRight'],
        ]
        self.encoder_ticks_publisher.publish(encoder_msg)


# Initialize the node
# Establish instances of the subscriber and publisher objects
# Continuously publish until the program shuts down


def main(args=None):
    try:
        rclpy.init(args=args)

        roustoucan = ROStouCAN()
        cantouros = CANtouROS()

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(roustoucan)
        executor.add_node(cantouros)

        executor.spin()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print("Starting shutdown")
        roustoucan.destroy_node()
        cantouros.destroy_node()
        rclpy.try_shutdown()
        print("Finished shutdown")


if __name__ == '__main__':
    main()


# CAN setup instructions instructions
# sudo apt install python3.9
# python3.9 -m pip install opencan-cand

# start redis container
# set up vcan on local machine
# sudo modprobe vcan
# sudo ip link add dev vcan0 type vcan
# sudo ip link set up vcan0
# start cand on local machine candump vcan0
# redis local host flag on cand --redis_host localhost
