## This script is not meant to be integrated into the stack it is to test if logging object history works 

## This will be done by making a ros2 subscriber node that takes in the objects distance from the vehicle and its name in order to log it 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseWithCovariance
from std_msgs.msg import String

class ObjectHistory(Node):

    def __init__(self):
        super().__init__('objecthistorytest')

        self.object_history = []
        self.object_position_x = 0.0
        self.object_position_y = 0.0

        self.object_name_subscription = self.create_subscription(String, '/obj_name', self.object_detection_callback, 10) 
        self.object_location_subscription = self.create_subscription(PoseWithCovariance,'/obj_location',self.object_location_callback,10)

    def object_location_callback(self,msg):
        self.object_position_x = msg.pose.position.x
        self.object_position_y = msg.pose.position.y

    def object_detection_callback(self, msg):
        object_destription = (msg.data,self.object_position_x,self.object_position_y)
        self.object_history.append(object_destription)

        self.get_logger().info("\n ----------------------------------------------\n")
        self.get_logger().info('Object History: \n')
        print(self.object_history)
        self.get_logger().info("\n ----------------------------------------------\n")

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 progra
    node = ObjectHistory()  # Instantiate Node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



    

    








        



