#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Float32MultiArray, PoseStamped
from nav_msgs.msg import Odometry, Path

import math
import numpy as np
import turtle

ld = 1.18 #define lookahead distance
wheel_base = 1.65 # double check

n_points = 7
N = 1000 # Update first dimension based on how many points we generate

waypoints = np.zeros([N,2])
x_path = waypoints[:, 0]
y_path = waypoints[:, 1]

class NoIntersectionException(Exception):
    def __init__(self):
        self.message = 'no intersection'

# Car class uses Odometry data to update the current position, yaw, and velocity of the car
class Car():
    def __init__(self, xpos=0, ypos=0, yaw=0, vel=0):
        self.xpos = xpos
        self.ypos = ypos
        self.yaw = yaw
        self.vel = vel

#--------------------------------------------(Steering Angle)--------------------------------------#

# Calculates distance between the current position and a point along the path
# Returns the index of the nearest waypoint
def nearest_point(curr_x, curr_y):
    dist = np.zeros(n_points)
    dist = np.sqrt(((x_path - curr_x)**2) + ((y_path - curr_y)**2))
    return np.argmin(dist)

# Calculates the lookahead point based on the current position and the nearest waypoint 
def lookahead(curr_x, curr_y):
    position = np.array([curr_x, curr_y]).T 
    wp = nearest_point(curr_x, curr_y)
    try:
        f = np.subtract(waypoints[wp] , position)
        d = np.subtract(waypoints[wp+1] , waypoints[wp])   

        # if (wp < n_points):
        #     d = np.subtract(waypoints[wp+1] , waypoints[wp])
        # else:
        #     d = 0

        a = np.dot(d,d); 
        b = 2*np.dot(f,d); 
        c = np.dot(f,f) - ld*ld
        discriminant = (b*b) - (4*a*c) 
        
        if (discriminant<0):
            raise NoIntersectionException
        else:
            discriminant= np.sqrt(discriminant)
            t1 = (-b- discriminant)/(2*a)
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
def newvel(d, a):
  v_1 = ((10 - a) + a * math.sqrt(1 + (4 * d))) / 2
  v_2 = ((10 - a) - a * math.sqrt(1 + (4 * d))) / 2
  if 0 < v_1 < 5:
    return v_1
  elif 0 < v_2 < 5:
    return v_2  # What would you return if these conditions aren't met ???

# Decelerate if approaching the endpoint
def decel(thresh_d, curr_d, v_max, a_max):
  if thresh_d > curr_d:
    return newvel(curr_d, a_max)
  else:
    return v_max

# Subscriber Class that updates odometry data and waypoints as the local path changes
class Subscriber:
    def __init__(self):    
        self.odom = rospy.Subscriber('/encoder_odom', Odometry, self.car_update)
        self.path = rospy.Subscriber('/path', Path, self.set_waypoints)
        self.car = Car()
        self.goal = [0,0]
    
    def car_update(msg):
        car.xpos = msg.pose.pose.position.x
        car.ypos = msg.pose.pose.position.y
        car.yaw = msg.pose.pose.orientation.z
        car.vel = msg.twist.twist.linear.x
        
    def set_waypoints(msg):
        waypoints[:,0] = msg.PoseStamped[:].pose.position.x
        waypoints[:,1] = msg.PoseStamped[:].pose.position.y
        x_path = waypoints[:, 0]
        y_path = waypoints[:, 1]
        self.goal[0] = msg.PoseStamped[N].pose.position.x
        self.goal[1] = msg.PoseStamped[N].pose.position.y

# Publisher class that will publish Twist messages as the output of the controller
class Publisher:
    def __init__(self):
        self.cmd = rospy.Publisher('/pure_pursuit/ppc_cmd', Float32MultiArray, queue_size=2)
        self.rate = rospy.Rate(100)
    
    def publish(self,msg):
        self.cmd.publish(msg)
        self.rate.sleep()

if __name__ == "__main__":
    
    rospy.init_node('pure_pursuit', anonymous=True)
    subscriber = Subscriber()
    publisher = Publisher()
    ppc_cmd = Float32MultiArray()

    # Max distance from goal before controller stops
    dmax = 10

    # decel params
    v_m = 2  # Constant 2m/s that won't change (for now)
    decel_dist = 7 
    a_max = -2 

    while not rospy.is_shutdown():

        # Loop through until the goal is within the decel distance
        while distance(subscriber.car.x, subscriber.goal[0], subscriber.car.y, subscriber.goal[1]) < dmax
            
            lookahead_x, lookahead_y = lookahead(subscriber.car.x, subscriber.car.y)
            
            steer_next = steering_angle(lookahead_x, lookahead_y, subscriber.car.x, subscriber.car.y, subscriber.car.yaw)

            dist = distance(subscriber.car.x, subscriber.goal[0], subscriber.car.y, subscriber.goal[1])
            v_next = decel(decel_dist, dist, v_m, a_max)

            ppc_cmd.data = [v_next, steer_next]
            publisher.publish(ppc_cmd)
