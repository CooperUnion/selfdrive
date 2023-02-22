#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path

import math
import numpy as np
import turtle

ld = 1.18 #define lookahead distancess

# Where does this come from ???
dt = 0.01 #time tick
WB = .98 #m
#rw= 0.5 #measure this

# Why even bother with waypoints if we can just use x and y???
waypoints = np.zeros((7,2))
x = waypoints[:, 0]
y = waypoints[:, 1]

#how to initialize 2 column array of zeros
N = 1000
n = len(waypoints)

# Subscriber Class that updates odometry data and waypoints as the local path changes
class Subscriber:
    def __init__(self):    
        self.odom = rospy.Subscriber('/encoder_odom', Odometry, self.callback)
        self.path = rospy.Subscriber('/teb_local_planner/local_plan', Path, self.set_waypoints)
        self.car = Car()

    def callback(msg):
        car.update(msg.twist.linear.x, msg.twist.angular.z)
    
    def set_waypoints(msg):
        for i in range(1,n):
            waypoints((i,0)) = msg.data[0].x    # Sets x coordinate of waypoint 
            waypoints((i,1)) = msg.data[0].y    # Sets y coordinate of waypoint
            x = waypoints[:, 0]
            y = waypoints[:, 1]
            
# Publisher class that will publish Twist messages as the output of the controller
class Publisher:
    def __init__(self):
        self.cmd = rospy.Publisher('/pure_pursuit/cmd_vel', Twist)
    def publish(self,msg):
        self.cmd.publish(msg)

class NoIntersectionException(Exception):
    def __init__(self):
        self.message = 'no intersection'

class Car():

    def __init__(self, xcar=0, ycar=0, yaw=0, vel=0, steering_angle=0):
        """
        Define a vehicle class
        :param x: float, x position
        :param y: float, y position
        :param yaw: float, vehicle heading
        :param vel: float, velocity
        """
        self.xcar = xcar
        self.ycar = ycar
        self.yaw = yaw
        self.vel = vel
        self.steering_angle = steering_angle

    def update(self, vel, steering_angle):
        """
        Vehicle motion model, here we are using simple bycicle model
        :param acc: float, acceleration
        :param delta: float, heading control
        """
        self.vel = vel
        self.steering_angle = steering_angle
        #yaw is heading angle, relative to x-axis, orientation from tech?
        self.xcar = self.xcar + (self.vel * math.cos(self.yaw) * dt)
        self.ycar = self.ycar + (self.vel * math.sin(self.yaw) * dt)
        self.yaw = self.vel * (math.tan(self.steering_angle) / WB) * dt + self.yaw
        return self.xcar, self.ycar, self.yaw
        #ycar prints 0. Does this make sense? 

#--------------------------------------------(Steering Angle)--------------------------------------#

# waypoints doesnt have to be an argument here
def nearest_point(waypoints, curr_x, curr_y): #might be useful for not wanting to travel through the back of the path.
    dist=np.zeros(N)
    #distance between robot and points along the path
    dist = np.sqrt(((x - curr_x)**2) + ((y - curr_y)**2))
    #index of current position < index of nearest waypoint/position to follow
    #return index of which waypoint/nearest point is closest
    return np.argmin(dist)

def lookahead(curr_x, curr_y):
    position = np.array([curr_x, curr_y]).T  #come back to it
    wp = nearest_point(waypoints, curr_x, curr_y)
    #-------------------(wp works)---------------------# 
    try:
        #for i in range (1,100): , don't need for now because will be running 100 times in main
        f = np.subtract(waypoints[wp] , position)
        if (wp<6):
            d = np.subtract(waypoints[wp+1] , waypoints[wp]) #think this is wrong. Might be wp and wp + 1 
        else:
            d = np.subtract(waypoints[wp] , waypoints[wp]) # isnt this just 0 ???
        a = np.dot(d,d); 
        b = 2*np.dot(f,d); 
        c = np.dot(f,f) - ld*ld
        discriminant = (b*b) - (4*a*c) #not way this is wrong computers don't fuck up math lol 
        
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


def Curvature(pointx, pointy, position):
    #pointx,pointy are lkx and lky

    #following the arc to the lookahead point
    #15 should be car's orientation
    robot_angle = position[2]
    robo_x = position[0]
    robo_y = position[1]
    robo_angle = np.tan(robot_angle)
    a= -1 * robo_angle
    c = robo_angle * robo_x - robo_y
    dist_x = abs(a*pointx + pointy + c)/math.sqrt(((a)**2)+1)   #b is 1 for some reason
    curvature = (2*dist_x)/(ld*ld) #dist_x lookaheag ponint??? 

    signed = np.sign(math.sin(robo_angle) * (pointx-robo_x) - math.cos(robo_angle) * (pointy-robo_y))

    signed_curvature = -1*curvature*signed
    return signed_curvature

def steering_angle(signed_curvature):
    steering_angle = np.arctan(signed_curvature*WB)
    return steering_angle

#---------------------------------------(Velocity)---------------------------------# 
def distance(x0, x1, y0, y1):
  X = x1 - x0
  Y = y1 - y0
  return math.sqrt(pow(X, 2) + pow(Y, 2))
  #used to calculate the distance between endpoint and car's current distance
def newvel(d, a):
  v_1 = ((10 - a) + a * math.sqrt(1 + (4 * d))) / 2
  v_2 = ((10 - a) - a * math.sqrt(1 + (4 * d))) / 2
  #now need to check if the computed velocity values are valid
  #we want to have a velocity less than 5.
  if 0 < v_1 < 5:
    return v_1

  #return these velocity values
  elif 0 < v_2 < 5:
    #return these velocity values
    return v_2

  #velocity that goes to low level
def decel(thresh_d, curr_d, v_max, a_max):
  if thresh_d > curr_d:
    return newvel(curr_d, a_max)
  else:
    return v_max

def main():
    car = Car()
    # v_m = car.vel
    v_m = 2  #constant 2m/s that won't change (for now)
    endpoint = [36, 6]   #get from tech team from globalcost map
    decel_dist = 7 
    a_max = -2   #passed into new vel
    #change curr_point to update function
    #distance btwn currpoint and endpoint 
    #calculate the velocity 
    rp = [0,0,0]

    for i in range (800):
        
        lkx, lky = lookahead(rp[0], rp[1])
        
        curv = Curvature(lkx, lky, rp)

        steer = steering_angle(curv)
        dist = distance(rp[0], endpoint[0], rp[1], endpoint[1])
        v_next = decel(decel_dist, dist, v_m, a_max)
        rp = car.update(v_next,steer)

        print(f"steering angle: {steer} | curvature: {curv} rbx: {rp[0]}| rby: {abs(rp[1])} | vel: {v_next} | robo_angle: {rp[2]} | lkx, lky: {lkx, lky}")

if __name__ == "__main__":
    main()

#step1: make tan(robo_angle) = to lky -roboy / lkx - robox 

#step2: check over the signed thing 

#can check by using lk point and robot point along with radius 
#to reverse engineer the circle and see if it looks right 