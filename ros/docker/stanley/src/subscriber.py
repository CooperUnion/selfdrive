#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

import math
import numpy as np

# Arbitrary number of waypoints
n_points = 10

def calculate_yaw(pathx, pathy, start_yaw):
	yaw = [start_yaw]
	for i in range(1, len(pathx)):
		dx = pathx[i] - pathx[i-1]
		dy = pathy[i] - pathy[i-1]
		angle = np.arctan2(dy, dx) # calculate the angle between the two points
		yaw.append(angle)
	return yaw

# Subscriber Class that updates odometry data and waypoints as the local path changes
class Subscriber:
    def __init__(self):    
        # Subscribed topics
        # Swap with fused odom topic from ekf in testing
        # self.odom = rospy.Subscriber('/encoder_odom', Odometry, self.update_car)
        self.odom = rospy.Subscriber('/gazebo/odom', Odometry, self.update_car)
        # self.path = rospy.Subscriber('/path', Path, self.set_waypoints)
        self.path = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.set_waypoints)
        self.goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal)

        # Car Info
        self.xpos = 0
        self.ypos = 0
        self.yaw = 0
        self.vel = 0.5

        # Path Info
        self.pathx = np.zeros(n_points)  
        self.pathy = np.zeros(n_points)
        self.pathyaw = np.zeros(n_points)

        # Goal Info
        self.xgoal = 0
        self.ygoal = 0
        self.yawgoal = 0

    # Decode Odometry topic to get current position, velocity, and yaw
    def update_car(self,msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z
        # self.vel = msg.twist.twist.linear.x
        
    # Decode Path topic into a series of waypoints
    # The nearest waypoint will be chosen by the Stanley controller
    def set_waypoints(self,msg):
        for i in range(0,n_points):
            self.pathx[i] = msg.poses[i].pose.position.x    # Sets x coordinate of waypoint 
            self.pathy[i] = msg.poses[i].pose.position.y    # Sets y coordinate of waypoint 
        self.pathyaw = calculate_yaw(self.pathx,self.pathy,self.yaw)

    # Decode Goal topic into final x, y, and yaw
    # Not sure how to properly use this
    def update_goal(self,msg):
        self.xgoal = msg.pose.position.x
        self.ygoal = msg.pose.position.y
        self.yawgoal = msg.pose.orientation.z