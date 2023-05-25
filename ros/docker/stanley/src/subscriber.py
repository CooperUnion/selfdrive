#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

import math
import numpy as np

# Subscriber Class that updates odometry data and waypoints as the local path changes
class Subscriber:
    def __init__(self):    
        self.odom = rospy.Subscriber('/encoder_odom', Odometry, self.car_update)
        self.path = rospy.Subscriber('/path', Path, self.set_waypoints)
        self.car = Car()
        self.xgoal = 0
        self.ygoal = 0
    
    def car_update(self,msg):
        self.car.prev_xpos = self.car.xpos
        self.car.xpos = msg.pose.pose.position.x
        self.car.ypos = msg.pose.pose.position.y
        self.car.yaw = msg.pose.pose.orientation.z
        self.car.vel = msg.twist.twist.linear.x
        
    def set_waypoints(self,msg):
        waypoints[:,0] = msg.PoseStamped[:].pose.position.x
        waypoints[:,1] = msg.PoseStamped[:].pose.position.y
        self.xgoal = msg.PoseStamped[N].pose.position.x
        self.ygoal = msg.PoseStamped[N].pose.position.y

# Car class uses Odometry data to update the current position, yaw, and velocity of the car
class Car():
    def __init__(self, xpos=0, prev_xpos=0, ypos=0, yaw=0, vel=0):
        self.xpos = xpos
        self.prev_xpos = prev_xpos
        self.ypos = ypos
        self.yaw = yaw
        self.vel = vel