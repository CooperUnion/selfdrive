#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from stan_path_planner import path_generation

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
        self.odom = rospy.Subscriber('/encoder_odom', Odometry, self.update_car)
        self.path = rospy.Subscriber('/path', Path, self.set_waypoints)
        self.goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal)

        self.state = rospy.Subscriber('/func_state', Int16, self.update_path)
        # Car Info
        self.xpos = 0
        self.ypos = 0
        self.yaw = 0
        self.vel = 0

        # Path Info
        self.pathx = np.zeros(n_points)  
        self.pathy = np.zeros(n_points)
        self.pathyaw = np.zeros(n_points)

        # Goal Info
        self.xgoal = 0
        self.ygoal = 0
        self.yawgoal = 0

        self.pg = path_generation()


    def update_path(self, msg):
        pg =self.pg 
        n_points = 100

        course_event = {
        # Course 1 Events 
        0: pg.right_turn(x,y,n_points),
        1: pg.swerve_R2L(x,y,n_points),
        2: pg.vertical_line(x,y,n_points),
        3: pg.swerve_L2R(x,y,n_points),
        4: pg.left_turn(x,y,n_points),
        5: pg.horizontal_line(x,y,n_points),
        6: pg.left_turn(x,y,n_points),
        8: pg.vertical_line(x,y,n_points),
        9: pg.vertical_line(x,y,n_points),
        10: pg.left_turn(x,y,n_points),

        #Course 2 Events
        11: pg.left_turn(x,y,n_points),
        12: pg.vertical_line(x,y,n_points),
        2: pg.swerve_R2L(x,y,n_points),
        3: pg.vertical_line(x,y,n_points),
        4: pg.swerve_L2R(x,y,n_points),
        5: pg.left_turn(x,y,n_points),
        6: pg.horizontal_line(x,y,n_points),
        8: pg.left_turn(x,y,n_points),
        9: pg.swerve_R2L(x,y,n_points),
        10: pg.vertical_line(x,y,n_points),
        11: pg.swerve_L2R(x,y,n_points),
        12: pg.left_turn(x,y,n_points),
        13: pg.horizontal_line(x,y,n_points),
        14: pg.horizontal_line(x,y,n_points),
        15: pg.left_turn(x,y,n_points)
        
        
    }
        path = course_event.get(msg.data)
        path_points = pg.stan_inputs(path)


        self.pathx = path_points[0]
        self.pathy = path_points[1]
        self.pathyaw = path_points[2]




         

    # Decode Odometry topic to get current position, velocity, and yaw
    def update_car(self,msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z
        self.vel = msg.twist.twist.linear.x
        
    # Decode Path topic into a series of waypoints
    # The nearest waypoint will be chosen by the Stanley controller
    def set_waypoints(self,msg):
        
        for i in range(0,n_points-1):
            self.pathx[i] = msg.PoseStamped[i].pose.position.x    # Sets x coordinate of waypoint 
            self.pathy[i] = msg.PoseStamped[i].pose.position.y    # Sets y coordinate of waypoint 
        self.pathyaw = calculate_yaw(self.pathx,self.pathy,self.yaw)

    # Decode Goal topic into final x, y, and yaw
    # Not sure how to properly use this
    def update_goal(self,msg):
        self.xgoal = msg.pose.position.x
        self.ygoal = msg.pose.position.y
        self.yawgoal = msg.pose.orientation.z
