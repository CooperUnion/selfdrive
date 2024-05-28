
import rclpy
from rclpy.node import Node

from controller.stanley import StanleyController, coterminal_angle
from controller.clothoid_path_planner import generate_clothoid_path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray

import math
import time
import numpy as np
from collections import namedtuple


class LaneFollow(Node): 
    def __init__(self):
        super().__init__('lane_change_node')

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/encoder_odom',    # Temporary until we can replace with filtered odometry message
            # '/odometry/filtered' 
            self.odom_callback,
            10
        )
        
        self.cmd_publisher = self.create_publisher(
            Float32MultiArray,
            '/cmd_stanley'
        )

        # Car Info
        self.xpos = 0.0
        self.ypos = 0.0 
        self.yaw = 0.0                
        self.vel = 0.0


    def odom_callback(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z
        self.vel = msg.twist.twist.linear.x

        self.get_logger().info('Received: "%s"' % self.msg)

    def lane_follow(self):
        stanley = StanleyController()

        # pick a point in front of us
        # maybe just consider at x:1, y:0, yaw:0?
        # provide cross_track_error from lane_detection algorithm
        # provide heading error from lane detection algorithm
        # maybe add lane_follow function to Stanley Controller class

def main(args=None):
    rclpy.init(args=args)

    lane_follow = LaneFollow()

    rclpy.spin(lane_follow)

    lane_follow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()