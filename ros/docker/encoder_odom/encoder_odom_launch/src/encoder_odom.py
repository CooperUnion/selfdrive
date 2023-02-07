#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import math
from math import sin, cos, pi

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Twist, TransformStamped 
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class Subscribe:
    def __init__(self):
        self.vel = 0
        self.angle = 0
        rospy.Subscriber('/enc_vel', Float32, self.callback_vel)
        rospy.Subscriber('/enc_angle', Float32, self.callback_angle)
    def callback_vel(msg):
        while True:
            self.vel = msg.data
    def callback_angle(msg):
        while True:
            self.angle = msg.data

# The Odometry message type contains the following messages:
# header (Coordinate frame for the pose)
# child_frame_id (Coordinate frame for the twist)
# pose (with covariance if provided by ekf)
# twist (with covariance if provided by ekf)
class Publish:
    def __init__(self):
        self.odom = rospy.Publisher('/encoder_odom', Odometry, queue_size=50)
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
    # def send_transform(self,current_time):
    #     t = TransformStamped()
    
    #     # double check this 
    #     t.header.stamp = current_time
    #     t.header.frame_id = "odom"
    #     t.child_frame_id = "base_link"
    #     t.transform.translation.x = self.x
    #     t.transform.translation.y = self.y
    #     t.transform.translation.z = 0.0
    #     q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]

    #     self.broadcaster.sendTransform(t)
    
    def calc_odom(self,current_time,last_time):
        
        self.vx = self.sub.vel
        self.vth = self.sub.angle

        self.dt = (current_time - last_time).to_sec()
        self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        self.delta_th = self.vth * self.dt
        
        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        t = TransformStamped()
    
        # double check this 
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        self.q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = self.q[0]
        t.transform.rotation.y = self.q[1]
        t.transform.rotation.z = self.q[2]
        t.transform.rotation.w = self.q[3]
        
        self.broadcaster.sendTransform(t)

        # self.send_transform(self,current_time)

        # self.odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th)
        # self.broadcaster.sendTransform(
        #     (self.x, self.y, 0.),
        #     self.odom_quat,
        #     current_time,
        #     "base_link",
        #     "odom"
        # )

# Initialize node
rospy.init_node('encoder_odom', anonymous=True)
current_time = rospy.Time.now()
last_time = rospy.Time.now()
encoder_odom = Encoder_Odom()

if __name__ == '__main__':

    current_time = rospy.Time.now()
    encoder_odom.calc_odom(current_time,last_time)

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    
    # set the position
    odom_msg.pose.pose = Pose(Point(encoder_odom.x, encoder_odom.y, 0.), Quaternion(*encoder_odom.q))

    # set the velocity
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist = Twist(Vector3(encoder_odom.vx, encoder_odom.vy, 0), Vector3(0, 0, encoder_odom.vth))
    
    last_time = current_time
    
    rospy.spin()