#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
import math
from math import sin, cos, pi

# Used for Arudino
import pyfirmata
import time

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

# The Odometry message type contains the following messages:
# header (Coordinate frame for the pose)
# child_frame_id (Coordinate frame for the twist)
# pose (with covariance if provided by ekf)
# twist (with covariance if provided by ekf)
class Publish:
    def __init__(self):
        self.odom = rospy.Publisher('/encoder_odom', Odometry, queue_size=2)
        # increase rate for RTABmap
        self.rate = rospy.Rate(1.0)
    def publish(self,data):
        while not rospy.is_shutdown():
            self.odom.publish(data)

# vx and vth come from encoders
# Not sure whether to use EPAS encoder for th or use encoders for vth to calculate th
# vy might always be zero
class Encoder_Odom:
    def __init__(self):
        self.sub = Subscribe()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.dt = 0.0
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_th = 0.0
        
    def calc_odom(self,enc_vel,enc_omega,current_time,last_time):
        
        # Change these lines
        self.vx = enc_vel
        self.vth = enc_omega

        self.dt = (current_time - last_time).to_sec()
        self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        self.delta_th = self.vth * self.dt
        
        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        self.odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th)
        self.broadcaster.sendTransform(
            (self.x, self.y, 0.),
            self.odom_quat,
            current_time,
            "base_link",
            "odom"
        )

# Encoder variables
wheel_radius = 1.0
circumfrence = 3.1415926 * 2 * wheel_radius
enc_res = 2048
tick_distance = circumfrence/enc_res
wheel_spacing = 1.0
prev_left_ticks = 0
prev_right_ticks = 0

# Initalize Arduino board variables
board = pyfirmata.Arduino('/dev/ttyACM0')
left_enc_pin = 5
right_enc_pin = 6
# Used to continuously read values from Arduino board
it = pyfirmata.util.Iterator(board)
it.start()

board.analog[left_enc_pin].mode = pyfirmata.INPUT
board.analog[right_enc_pin].mode = pyfirmata.INPUT

# Initialize time variables
current_time = rospy.Time.now()
last_time = rospy.Time.now()

# Not sure whether or not to initialize outside of the loop
rospy.init_node('encoder_odom', anonymous=True)
encoder_odom = Encoder_Odom()

if __name__ == '__main__':
    
    # rospy.init_node('encoder_odom', anonymous=True)

    current_time = rospy.Time.now()
    left_ticks = board.analog[left_enc_pin].read
    right_ticks = board.analog[right_enc_pin].read
    
    # Not sure if we need to include sleep time since the measurements are limited by the rate of the publisher
    # time.sleep(0.1)
    delta_left = left_ticks - prev_left_ticks
    delta_right = right_ticks - prev_right_ticks

    v_left = (delta_left * tick_distance) / (current_time - last_time).to_sec()
    v_right = (delta_right * tick_distance) / (current_time - last_time).to_sec()

    # 10 is a scaling factor used to characterize system
    enc_vel = ((v_right + v_left) / 2) * 10
    enc_omega = ((v_right - v_left) / wheel_spacing) * 10

    encoder_odom.calc_odom(enc_vel,enc_omega,current_time,last_time)

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    
    # set the position
    odom_msg.pose.pose = Pose(Point(encoder_odom.x, encoder_odom.y, 0.), Quaternion(*encoder_odom.odom_quat))

    # set the velocity
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist = Twist(Vector3(encoder_odom.vx, encoder_odom.vy, 0), Vector3(0, 0, encoder_odom.vth))
    
    last_time = current_time
    prev_left_ticks = left_ticks
    prev_right_ticks = right_ticks
    rospy.spin()