import time
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16MultiArray


### Might need to add tf2 for publishing transfrom from encoder_odom frame to world frame


def Euler2Quaternion(phi=0, theta=0, psi=0):
    """
    Converts an euler angle attitude to a quaternian attitude
    :param euler: Euler angle attitude in a np.matrix(phi, theta, psi)
    :return: Quaternian attitude in np.array(e0, e1, e2, e3)
    """
    orientation_msg = Quaternion()

    orientation_msg.x = np.cos(psi / 2.0) * np.cos(theta / 2.0) * np.cos(
        phi / 2.0
    ) + np.sin(psi / 2.0) * np.sin(theta / 2.0) * np.sin(phi / 2.0)
    orientation_msg.y = np.cos(psi / 2.0) * np.cos(theta / 2.0) * np.sin(
        phi / 2.0
    ) - np.sin(psi / 2.0) * np.sin(theta / 2.0) * np.cos(phi / 2.0)
    orientation_msg.z = np.cos(psi / 2.0) * np.sin(theta / 2.0) * np.cos(
        phi / 2.0
    ) + np.sin(psi / 2.0) * np.cos(theta / 2.0) * np.sin(phi / 2.0)
    orientation_msg.w = np.sin(psi / 2.0) * np.cos(theta / 2.0) * np.cos(
        phi / 2.0
    ) - np.cos(psi / 2.0) * np.sin(theta / 2.0) * np.sin(phi / 2.0)

    return orientation_msg


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('Encoder_Odometry_Node')

        self.encoder_ticks_subscription = self.create_subscription(
            UInt16MultiArray, '/encoder_ticks', self.calc_odom_callback, 10
        )
        self.encoder_ticks_subscription  # prevent unused variable warning

        self.odometry_publisher = self.create_publisher(
            Odometry, '/encoder_odom', 10
        )

        self.time_now = 0.0
        self.time_prev = 0.0

        # parameters
        self.circumference = 1.7954
        self.enc_res = 30536  # Number of ticks for one rotation
        self.tick_distance = self.circumference / (self.enc_res)
        self.track_width = 1.25  # Unsure if this is wheel base or track width

        self.left_ticks = 0
        self.prev_left_ticks = 0

        self.right_ticks = 0
        self.prev_right_ticks = 0

        self.delta_left = 0
        self.delta_right = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # yaw

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

        self.init = 0

    def calc_odom_callback(self, msg):

        self.time_now = time.time()

        # Update encoder ticks from received '/encoder_ticks' topic
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

        # Required to initalize absolute encoder count to set starting position to origin
        if self.init == 0:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.time_prev = self.time_now
            self.init = 1
            print("Tried Initializing")
            return

        # Get change in time for derivatives
        delta_time = self.time_now - self.time_prev

        # Calculate Odometry
        # self.delta_left = float(c_int16(self.left_ticks - self.prev_left_ticks).value) * self.tick_distance
        # self.delta_right = float(c_int16(self.right_ticks - self.prev_right_ticks).value) * self.tick_distance

        # Account for encoder overflow in the forward direction
        # Probably want to add velocity to the conidtion to determine overflow in either direction
        if (
            self.left_ticks < self.prev_left_ticks
            or self.right_ticks < self.prev_right_ticks
        ):
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            return

        self.delta_left = (
            self.left_ticks - self.prev_left_ticks
        ) * self.tick_distance
        self.delta_right = (
            self.right_ticks - self.prev_right_ticks
        ) * self.tick_distance

        v_left = self.delta_left / delta_time
        v_right = self.delta_right / delta_time

        self.vx = (v_left + v_right) / 2.0
        self.vy = 0.0
        self.w = (v_right - v_left) / (self.track_width)

        delta_x = (
            self.vx * np.cos(self.th) - self.vy * np.sin(self.th)
        ) * delta_time
        delta_y = (
            self.vx * np.sin(self.th) + self.vy * np.cos(self.th)
        ) * delta_time
        delta_th = self.w * delta_time

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # update the variables for next calculation
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.time_prev = self.time_now

        # Create Odometry message
        odom_msg = Odometry()

        # odom_msg.header.frame_id = "encoder_odom"
        odom_msg.header.frame_id = "map"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Euler2Quaternion(phi=self.th)

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.w

        self.odometry_publisher.publish(odom_msg)
        print(f"xpos: {self.x}, ypos: {self.y}, yaw: {self.th}")
        print(f"xvel: {self.vx}, w: {self.w}")
        # self.get_logger().info('Publishing: "%s"' % odom_msg)


def main(args=None):
    rclpy.init(args=args)

    encoder_odom = EncoderOdom()

    rclpy.spin(encoder_odom)

    encoder_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
