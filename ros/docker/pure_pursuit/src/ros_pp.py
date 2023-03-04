#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Float32MultiArray, PoseStamped
from nav_msgs.msg import Odometry, Path

import math
import numpy as np

ld = 1.18 #define lookahead distance
wheel_base = 1.75 # double check

n_points = 7

waypoints = np.zeros([n_points,2])

# Subscriber Class that updates odometry data and waypoints as the local path changes
class Subscriber:
    def __init__(self):    
        self.odom = rospy.Subscriber('/encoder_odom', Odometry, self.car_update)
        self.path = rospy.Subscriber('/teb_local_planner/local_plan', Path, self.set_waypoints)
        self.path = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal)
        self.car = Car()
        self.xgoal = 0
        self.ygoal = 0
    
    def car_update(self,msg):
        self.car.prev_xpos = self.car.xpos
        self.car.xpos = msg.pose.pose.position.x
        self.car.ypos = msg.pose.pose.position.y
        self.car.yaw = msg.pose.pose.orientation.z
        self.car.vel = msg.twist.twist.linear.x
        
    def set_waypoints(msg):
        for i in range(0,n_points-1):
            waypoints[i,0] = msg.PoseStamped[i].pose.position.x    # Sets x coordinate of waypoint 
            waypoints[i,1] = msg.PoseStamped[i].pose.position.y    # Sets y coordinate of waypoint  

    def set_goal(self,msg):
        self.xgoal = msg.pose.position.x
        self.ygoal = msg.pose.position.y

# Publisher class that will publish Twist messages as the output of the controller
class Publisher:
    def __init__(self):
        self.cmd = rospy.Publisher('/pure_pursuit/ppc_cmd', Float32MultiArray, queue_size=2)
        self.rate = rospy.Rate(100)
    
    def publish(self,msg):
        self.cmd.publish(msg)
        self.rate.sleep()

class NoIntersectionException(Exception):
    def __init__(self):
        self.message = 'no intersection'

# Car class uses Odometry data to update the current position, yaw, and velocity of the car
class Car():
    def __init__(self, xpos=0, prev_xpos=0, ypos=0, yaw=0, vel=0):
        self.xpos = xpos
        self.prev_xpos = prev_xpos
        self.ypos = ypos
        self.yaw = yaw
        self.vel = vel

#--------------------------------------------(Steering Angle)--------------------------------------#

# Calculates distance between the current position and a point along the path
# Returns the index of the nearest waypoint
def nearest_point(curr_x, curr_y):
    dist = np.zeros(n_points)
    dist = np.sqrt(((waypoints[:, 0] - curr_x)**2) + ((waypoints[:, 1] - curr_y)**2))
    return np.argmin(dist)

# Calculates the lookahead point based on the current position and the nearest waypoint 
def lookahead(curr_x, curr_y):
    position = np.array([curr_x, curr_y]).T 
    wp = nearest_point(curr_x, curr_y)
    try:
        f = np.subtract(waypoints[wp] , position)
        d = np.subtract(waypoints[wp+1] , waypoints[wp])

        a = np.dot(d,d); 
        b = 2*np.dot(f,d); 
        c = np.dot(f,f) - ld*ld
        discriminant = (b*b) - (4*a*c) 
        
        if (discriminant<0):
            raise NoIntersectionException
        else:
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant)/(2*a)
            t2 = (-b + discriminant)/(2*a)
            if (t1>=0 and t1<=1.0):
                point = waypoints[wp] + t1*d
                pointx = point[0]
                pointy = point[1]
                return pointx, pointy
                 #x and y value of ld
            elif (t2 >= 0 and t2 <=1.0):
                point = waypoints[wp] + t2*d
                pointx = point[0]
                pointy = point[1]
                return pointx, pointy
            else:
                return 0,0

    except NoIntersectionException as e:
        print("Path lookahead no intersection")
        return -999, -999

# Calculates the steering angle needed to follow the arc connecting the current position and the lookahead point
def steering_angle(lookahead_x, lookahead_y, xpos, ypos, yaw):
    
    angle = np.tan(yaw)
    a = -1 * angle
    c = angle * xpos - ypos
    dist_x = abs(a*lookahead_x + lookahead_y + c)/math.sqrt(((a)**2)+1) 
    curvature = (2*dist_x)/(ld*ld) 

    signed = np.sign(math.sin(angle) * (lookahead_x-xpos) - math.cos(angle) * (lookahead_y-ypos))

    signed_curvature = -1*curvature*signed
    steering_angle = np.arctan(signed_curvature*wheel_base)
    return steering_angle

#---------------------------------------(Velocity)---------------------------------# 

# Calculates the distance between two points
# Used to calculate the distance between the car's current position and the endpoint 
def distance(x0, x1, y0, y1):
  X = x1 - x0
  Y = y1 - y0
  return math.sqrt(pow(X, 2) + pow(Y, 2))

# Checks if the desired velocity commands are within a valid range 
# Velocity must be less than 5 (unit ?????)
def newvel(car_xpos, car_prev_xpos, a):
  #first case: thresh - car location = first element 
  #second case and beyond: prev car location - car curr location = next element 

  dx = car_xpos - car_prev_xpos
  v = math.sqrt(2 * a * dx)

  if 0 <= v < 5:
    return v
  else:
    return 0

# Decelerate if approaching the endpoint
def decel(thresh_d, curr_d, car_prev_xpos, v_max, a_max, car_xpos):
  if thresh_d > curr_d:
    return newvel(car_xpos, car_prev_xpos, a_max)
  else:
    return v_max

if __name__ == "__main__":
    
    rospy.init_node('pure_pursuit', anonymous=True)
    subscriber = Subscriber()
    publisher = Publisher()
    ppc_cmd = Float32MultiArray()

    # Max distance from goal before controller stops
    dist = 0
    dmax = 10

    # decel params
    v_m = 2  # Constant 2m/s that won't change (for now)
    decel_dist = 0.06 
    

    while not rospy.is_shutdown():

        # Loop through until the goal is within the stopping distance
        while dist < dmax:
            a_max = 0.9 
            dist = distance(subscriber.car.xpos, subscriber.xgoal, subscriber.car.ypos, subscriber.ygoal)

            if dist > decel_dist:
                lookahead_x, lookahead_y = lookahead(subscriber.car.xpos, subscriber.car.ypos)
                steer_next = steering_angle(lookahead_x, lookahead_y, subscriber.car.xpos, subscriber.car.ypos, subscriber.car.yaw)
            if dist < decel_dist:
                steer_next = steering_angle(subscriber.xgoal, subscriber.ygoal, subscriber.car.xpos, subscriber.car.ypos, subscriber.car.yaw)    
            v_next = decel(decel_dist, dist, subscriber.car.prev_xpos, v_m, a_max, subscriber.car.xpos)

            ppc_cmd.data = [v_next, steer_next]
            publisher.publish(ppc_cmd)
        
        # Once we get within a predefined distance of the endpoint, we want to stop accelerating
        a_max = 0
        v_next = decel(decel_dist, dist, v_m, a_max, subscriber.car.xpos)

        ppc_cmd.data = [v_next, steer_next]
        publisher.publish(ppc_cmd)