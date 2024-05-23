import rclpy
from rclpy.node import Node

from controller.stanley import StanleyController, coterminal_angle
from controller.clothoid_path_planner import generate_clothoid_path
from odom_sub import OdomSubscriber

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

import math
import time
import numpy as np
from collections import namedtuple


Point = namedtuple("Point", ["x", "y"])


def calculate_yaw(pathx, pathy, start_yaw):
    yaw = [start_yaw]
    for i in range(1, len(pathx)):
        dx = pathx[i] - pathx[i - 1]
        dy = pathy[i] - pathy[i - 1]
        angle = np.arctan2(
            dy, dx
        )  # calculate the angle between the two points
        yaw.append(angle)
    return yaw


class LaneChange(Node):
    def __init__(self, odom_sub):
        super().__init__('lane_change_node')

        self.path_publisher = self.create_publisher(
            Path, '/lane_change_path', 10
        )

        self.cmd_publisher = self.create_publisher(
            Float32MultiArray, '/cmd_stanley', 10
        )

        self.odom_sub = odom_sub

        # Path Info
        self.n_points = 99  # Arbitrary number of waypoints
        self.max_dist = 5  # FIX max distance from goal point set in meters
        self.pathx = np.zeros(self.n_points)
        self.pathy = np.zeros(self.n_points)
        self.pathyaw = np.zeros(self.n_points)


    def create_path(self, relative_x, relative_y, end_yaw):

        start_x = self.odom_sub.xpos
        start_y = self.odom_sub.ypos
        # Unsure about lead_axle, this position is relative to encoders
        start_yaw = self.odom_sub.yaw
        end_x = start_x + relative_x
        end_y = start_y + relative_y

        # correct for backwards paths -- yaw refers to the yaw of the car
        if self.odom_sub.vel < 0:
            start_yaw = coterminal_angle(start_yaw + math.pi)
            end_yaw = coterminal_angle(end_yaw + math.pi)

        self.pathx, self.pathy = generate_clothoid_path(
            Point(start_x, start_y),
            start_yaw,
            Point(end_x, end_y),
            end_yaw,
            self.n_points,
        )
        self.pathyaw = calculate_yaw(self.pathx, self.pathy, start_yaw)

        # publish path topic
        path_msg = Path()

        path_msg.header.frame_id = "map"

        for i in range(self.n_points - 1):
            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.pose.position.x = self.pathx[i]
            pose.pose.position.y = self.pathy[i]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = self.pathyaw[i]
            pose.pose.orientation.w = 0.0

            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)


    def follow_path(self):

        stanley = StanleyController(
            self.pathx, self.pathy, self.pathyaw, self.max_dist
        )
        cmd = Float32MultiArray()

        while np.hypot(self.pathx[-1]-self.odom_sub.xpos, self.pathy[-1]-self.odom_sub.ypos) > self.max_dist:
        # may want to set target velocity for stanley curvature rather than actual velocity
            [steer_next, vel_next] = stanley.curvature(
                self.odom_sub.xpos, self.odom_sub.ypos, self.odom_sub.yaw, self.odom_sub.vel
            )
            cmd.data = [
                steer_next,
                vel_next,
            ]  # Ensure signs are correct based on right hand rule
            print(f"Steering Command: {steer_next}, Velocity Command: {vel_next}")
            # self.cmd_publisher.publish(cmd)

            time.sleep(.05)

        ### Find a way to cancel path when stanley loop is finished
        # maybe publish path message at the end with all 0's when the path is complete


def main(args=None):
    rclpy.init(args=args)

    odom_sub = OdomSubscriber()
    lane_change = LaneChange(odom_sub)

    relative_x = 5
    relative_y = 0
    end_yaw = 0

    lane_change.create_path(relative_x, relative_y, end_yaw)

    lane_change.follow_path()

    lane_change.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
