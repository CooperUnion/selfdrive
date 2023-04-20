import math
import numpy as np
import matplotlib.pyplot as plt

dt=0.01 #time tick
WB= 1.75 #m
K = 0.9
#rw= 0.5 #measure this
waypoints=np.array([   #create random sample waypoints that tech may give me based on desired spacing
        # [0,0],
        # [.1,.1],  #constant spacing
        # [.2,.2],
        # [.3,.3], 
        # [.4,.4],
        # [.5,.5],
        # [.6,.6]
        [0,0,0],
        [1,1,10],
        [2,2,20],
        [3,4,30], 
        [4,5,40],
        [5,7,50], 
        [5,8,90]
    ])

x=waypoints[:, 0]
y=waypoints[:, 1]
desired_orientation = waypoints[:,2]

#how to initialize 2 column array of zeros
N=1000
n=len(waypoints)

class Car():
    def __init__(self, xcar=0, ycar=0, yaw=0, vel=0):
        #need to initialize self.index or else it will delete right after this
        self.xcar = xcar
        self.ycar = ycar
        self.yaw = yaw
        self.vel = vel

    def update(self, vel, steering_angle):
        self.vel = vel
        self.steering_angle = steering_angle
        #yaw is heading angle, relative to x-axis, orientation from tech?
        self.xcar = self.xcar + (self.vel * math.cos(self.yaw) * dt)
        self.ycar = self.ycar + (self.vel * math.sin(self.yaw) * dt)
        self.yaw = self.vel * (math.tan(self.steering_angle) / WB) * dt + self.yaw
        return self.xcar, self.ycar, self.yaw

#stanley controller: finding desired heading angle--> steering angle, cross track error

#--------------------(Steering_Angle)--------------------#

def nearest_point(waypoints, curr_x, curr_y): #might be useful for not wanting to travel through the back of the path.
    dist=np.zeros(N)
           #distance between robot and points along the path
    dist = np.sqrt(((x - curr_x)**2) + ((y - curr_y)**2))
            #index of current position < index of nearest waypoint/position to follow
            #return index of which waypoint/nearest point is closest
    return np.argmin(dist)

def t_index(waypoints,curr_x,curr_y):
    wp = nearest_point(waypoints, curr_x, curr_y)
    dist_wp = np.sqrt(((y[wp + 1] - y[wp])**2) + (((x[wp + 1] - x[wp])**2)))
    #distance between waypoinys
    
    dist_actual = np.sqrt(((curr_y - y[wp])**2) + (((curr_x - x[wp])**2)))
    #distance between former waypoint and current position 

    t_index =  dist_actual/dist_wp # "theoretically this should work" - Azra

    return t_index 
 
def compute_heading_angle(wp_orientation,curr_x,curr_y):
    t1 = 2 
    t0 = 1 
    #t1 - t0 set to 1 for now, but will calculate the actual values later
    #position = np.array([curr_x, curr_y]).T
    wp = nearest_point(waypoints, curr_x, curr_y)
    m  = (wp_orientation[wp+1] - wp_orientation[wp])/ (t1 -t0)
  
    heading_angle = wp_orientation[wp] + m * (t1 - t0)

    return heading_angle

def compute_heading_error(vehicle_yaw,desired_heading_angle):

    heading_error = desired_heading_angle - vehicle_yaw
    return heading_error

def compute_CTE(waypoints,curr_x,curr_y): 
  #use the two waypoints to make a line 
  #get the a,b,and,c of the line 
  #use formula plugging in current location of the car 
  wp = nearest_point(waypoints, curr_x, curr_y)
  [x0, y0, theta0] = waypoints[wp]
  [x1, y1, theta1] = waypoints[wp + 1]

  a  = (y0 - y1)
  b  = (x1 - x0)
  c  = (x0*y1 - x1*y0)

  CTE  = (abs(a*curr_x + b*curr_y + c)) / math.sqrt(a**2 + b**2)
  return CTE 

def compute_Steering_Angle (CTE, k, heading_error,velocity):
    delta = heading_error + math.atan((k * CTE)/velocity)
    return delta 

#--------------------(Velocity)--------------------#
def distance(x0, x1, y0, y1):
  X = x1 - x0
  Y = y1 - y0
  return math.sqrt(pow(X, 2) + pow(Y, 2))

def newvel(car_x_location, d, a):
  #first case: thresh - car location = first element 
  #second case and beyond: prev car location - car curr location = next element 

  n = len(car_x_location) - 1 
  delta_x = car_x_location[n] - car_x_location[n-1]

  v_1 = math.sqrt(2 * a * delta_x)
  #now need to check if the computed velcoiy values are valid
  #we want to have a velcoity less than 5.
  if 0 <= v_1 < 5:
    return v_1

  #velcoity that goes to low level
def decel(thresh_d, curr_d, v_max, a_max,carx):
  if thresh_d > curr_d:
    return newvel(carx, curr_d, a_max)
  else:
    return v_max
    

D = 600
def main(): 
  
  car = Car()
  rp = [0,0,0]
  robotx = []
  roboty = []

  curr_car_x = rp[0]
  curr_car_y = rp[1]
  curr_car_yaw = rp[2]

  #--------------------(Velocity_Paramters)--------------------#
  v_m = 2  #constant 2m/s that won't change (for now)
  endpoint = waypoints[len(waypoints) - 1]  #get from tech team from globalcost map
  decel_dist = 0.06
  a_max = 0.9   #passed into new vels

  for i in range (D):

    #--------------------(Velocity)--------------------#
    dist = distance(rp[0], endpoint[0], rp[1], endpoint[1]) 
    #how close is the robot to approaching the endpoint
    v_next = decel(decel_dist, dist, v_m, a_max,robotx) 
    #velcoity the car is going at 
    if (dist < 0.0062 ): 
      a_max = 0

    #--------------------(Stanley_Controller)--------------------#
    heading_angle = compute_heading_angle(desired_orientation,curr_car_x,curr_car_y)
    CTE = compute_CTE(waypoints,curr_car_x,curr_car_y)
    heading_error = compute_heading_error(curr_car_yaw,heading_angle)
    Steering_Angle = compute_Steering_Angle(CTE,K,heading_error,v_next)

    #--------------------(Update_Car_Location)--------------------#
    rp =  car.update(v_next,Steering_Angle)
    robotx.append(rp[0])
    roboty.append(rp[1])

   #--------------------(Printing/Plotting_Info)--------------------#
  
    #print(f"robo_angle: {rp[2]}")
    k = robotx[i] 
    j = roboty[i]
    plt.plot(k,j, 'b', marker="o", markersize=5)


  
  plt.title("Purse Pursuit Controller") 
  plt.xlabel("x axis") 
  plt.ylabel("y axis") 
  
  for i  in range(7):
       plt.plot(x[i], y[i], marker="o", markersize=5)
  plt.show()

    
if __name__ == "__main__":
  main()

  

    
  

    





