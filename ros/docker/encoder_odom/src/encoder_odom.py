#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import math
from math import sin, cos, pi

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Twist, TransformStamped 
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
 
class Subscribe:
    def __init__(self):
        self.left_ticks = 0
        self.right_ticks = 0
        rospy.Subscriber('/encoder_ticks', UInt16MultiArray, self.callback)
    def callback(msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

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

        self.delta_left = 0.0
        self.delta_right = 0.0
        self.v_left = 0.0
        self.v_right = 0.0

        #starting the node at 0 ticks might cause a problem with absolute encoders???
        self.prev_left_ticks = 0.0
        self.prev_right_ticks = 0.0

    def send_transform(self,current_time):
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
    def calc_odom(self,current_time,last_time):
        
        # Car Variables
        wheel_radius = 0.3302   #13 inch ~= 0.3302 meter
        circumfrence = 1.899156
        enc_res = 65536
        tick_distance = circumfrence/enc_res
        wheel_spacing = 1.27    #Approximately 50 inch ~= 1.27 meter
 
        # Overflow condition for unsigned integer subtraction
        if(self.sub.left_ticks > self.prev_left_ticks):
            self.delta_left = self.sub.left_ticks - self.prev_left_ticks
        else:
            self.delta_left = self.sub.prev_left_ticks - self.left_ticks 

        if(self.sub.right_ticks > self.prev_right_ticks):
            self.delta_right = self.sub.right_ticks - self.prev_right_ticks
        else:
            self.delta_right = self.sub.prev_right_ticks - self.right_ticks        

        self.v_left = (self.delta_left * tick_distance) / (current_time - last_time).to_sec()
        self.v_right = (self.delta_right * tick_distance) / (current_time - last_time).to_sec()

        # 10 is a scaling factor used to characterize system
        self.vx = ((self.v_right + self.v_left) / 2) * 10
        self.vth = ((self.v_right - self.v_left) / wheel_spacing) * 10

        self.dt = (current_time - last_time).to_sec()
        self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        self.delta_th = self.vth * self.dt
        
        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        self.send_transform(current_time)

        self.prev_left_ticks = self.sub.left_ticks
        self.prev_right_ticks = self.sub.right_ticks

if __name__ == '__main__':

    rospy.init_node('encoder_odom', anonymous=True)

    odom_pub = rospy.Publisher('/encoder_odom', Odometry, queue_size=50)
    rate = rospy.Rate(1.0)

    encoder_odom = Encoder_Odom() 

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
       
    while not rospy.is_shutdown():
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
        
        odom_pub.publish(odom_msg)

        last_time = current_time
        rate.sleep()