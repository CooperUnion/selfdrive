
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


Point = namedtuple("Point", ["x", "y"])

def calculate_yaw(pathx, pathy, start_yaw):
	yaw = [start_yaw]
	for i in range(1, len(pathx)):
		dx = pathx[i] - pathx[i-1]
		dy = pathy[i] - pathy[i-1]
		angle = np.arctan2(dy, dx) # calculate the angle between the two points
		yaw.append(angle)
	return yaw


class LaneChange(Node): 
    def __init__(self):
        super().__init__('lane_change_node')

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/encoder_odom',    # Temporary until we can replace with filtered odometry message
            self.odom_callback,
            10
        )

        self.path_publisher = self.create_publisher(
            Path,
            '/lane_change_path',
            10)
        
        self.cmd_publisher = self.create_publisher(
            Float32MultiArray,
            '/cmd_stanley'
        )

        # Car Info
        self.xpos = 0.0
        self.ypos = 0.0 
        self.yaw = 0.0                
        self.vel = 0.0

        # Path Info
        self.n_points = 99 # Arbitrary number of waypoints
        self.max_dist = 1 # max distance from goal point set in meters
        self.pathx = np.zeros(self.n_points)  
        self.pathy = np.zeros(self.n_points)
        self.pathyaw = np.zeros(self.n_points)


    def odom_callback(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z
        self.vel = msg.twist.twist.linear.x

        self.get_logger().info('Received: "%s"' % self.msg)


    def create_path(self, relative_x, relative_y, end_yaw):
        wheelbase = 1.27 # double check measurement 
        lead_dist = math.copysign(wheelbase/2, self.vel)

        start_x = self.xpos
        start_y = self.ypos
        start_yaw = self.yaw
        end_x = start_x + relative_x
        end_y = start_y + relative_y

        # correct for backwards paths -- yaw refers to the yaw of the car
        if self.vel < 0:
            start_yaw = coterminal_angle(start_yaw + math.pi)
            end_yaw = coterminal_angle(end_yaw + math.pi)

        self.pathx, self.pathy = generate_clothoid_path(
             Point(start_x, start_y), start_yaw,
             Point(end_x, end_y), end_yaw    
        )
        self.pathyaw = calculate_yaw(self.pathx, self.pathy, start_yaw)

        # publish path message
        path_msg = Path()

        path_msg.poses.pose.position.x = self.pathx
        path_msg.poses.pose.position.y = self.pathy
        path_msg.poses.pose.orientation.z = self.pathyaw

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Publishing: "%s"' % path_msg)

    
    def follow_path(self):
        stanley = StanleyController(self.pathx, self.pathy, self.vel)
        cmd = Float32MultiArray()

        while np.hypot(self.pathx[-1]-self.xpos, self.pathy[-1]-self.ypos) > self.max_dist:
            # may want to set target velocity for stanley curvature rather than actual velocity
            [steer_next, vel_next] = stanley.curvature(self.xpos, self.ypos, self.yaw, self.vel)
            
            cmd.data = [steer_next, vel_next]
            self.cmd_publisher.publish(cmd)

            time.sleep(.05)

        ### Find a way to cancel path when stanley loop is finished
        # maybe publish path message at the end with all 0's when the path is complete


def main(args=None):
    rclpy.init(args=args)

    lane_change = LaneChange()

    rclpy.spin(lane_change)

    delta_x = 5
    delta_y = 0
    delta_yaw = 0

    lane_change.create_path(delta_x, delta_y, delta_yaw)
    lane_change.follow_path()

    lane_change.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()