#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
import math
from math import sin, cos, pi

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class Subscribe:
    def __init__(self):
        self.vel = rospy.Subscriber('/enc_vel', Float32, self.callback)
        self.angle = rospy.Subscriber('/enc_angle', Float32, self.callback)
        # self.steer = rospy.Subscriber('/enc_steer', Float32, self.callback)
    def callback(msg):
        while True:
            rospy.loginfo("I heard %s", msg.data)

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
        
    def calc_odom(self,current_time,last_time):
        
        self.vx = self.sub.vel
        self.vth = self.sub.angle

        self.dt = (current_time - last_time).to_sec()
        self.delta_x = (vx * cos(th) - vy * sin(th)) * dt
        self.delta_y = (vx * sin(th) + vy * cos(th)) * dt
        self.delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, th)
        self.broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

# Initialize time variables
current_time = rospy.Time.now()
last_time = rospy.Time.now()

# Not sure whether or not to initialize outside of the loop
rospy.init_node('encoder_odom', anonymous=True)
encoder_odom = Encoder_Odom()

if __name__ == '__main__':
    
    # rospy.init_node('encoder_odom', anonymous=True)

    current_time = rospy.Time.now()
    encoder_odom.calc_odom(current_time,last_time)

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    
    # set the position
    odom_msg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
    last_time = current_time
    
    rospy.spin()