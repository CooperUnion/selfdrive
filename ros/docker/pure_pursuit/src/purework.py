import math
import numpy as np
import matplotlib.pyplot as plt
import turtle
    



ld=1.18#define lookahead distancess
dt=0.01 #time tick
WB= .98 #m
#rw= 0.5 #measure this
waypoints=np.array([   #create random sample waypoints that tech may give me based on desired spacing
        # [0,0],
        # [1,1],  #constant spacing
        # [2,2],
        # [3,5], 
        # [4,6],
        # [25,5],
        # [36,6]
        [0,0],
        [1,1],
        [2,2],
        [3, 4], 
        [4, 5],
        [5, 7], 
        [5, 8]
    ])
# orientation=np.array(
#     [15,12,12,13,14,18,10]
# )
x=waypoints[:, 0]
y=waypoints[:, 1]


#how to initialize 2 column array of zeros
N=1000
n=len(waypoints)
#have functions outside of class and the tech box as the class  
#how far along the path the waypoint is, 7 points

class NoIntersectionException(Exception):
    def __init__(self):
        self.message = 'no intersection'

class Car():

    def __init__(self, xcar=0, ycar=0, yaw=0, vel=0):
        """
        Define a vehicle class
        :param x: float, x position
        :param y: float, y position
        :param yaw: float, vehicle heading
        :param vel: float, velocity
        """
        #need to initialize self.index or else it will delete right after this
        self.xcar = xcar
        self.ycar = ycar
        self.yaw = yaw
        self.vel = vel

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
        #self.vel[self.index] = self.vel[self.index-1]+acc*dt
        #ycar prints 0. Does this make sense? 

#--------------------------------------------(Steering Angle)--------------------------------------#
# def distance(waypoints): #may come into use in curvature function, not being used anywhere
#         dist=np.zeros(N)
#         x=waypoints[:, 0]
#         y=waypoints[:, 1]
#         min_index=0
#         for i in range (1,n):
#         #running sum of the distances between points
#             dist[i]= dist[i-1]+ math.sqrt((x[i]-x[i-1])**2+(y[i]-y[i-1])**2)
#             if(dist[i]<dist[min_index]):
#                 min_index=i 
#                 print(dist)
#                 return dist[i]


def nearest_point(waypoints, curr_x, curr_y): #might be useful for not wanting to travel through the back of the path.
        dist=np.zeros(N)
           #distance between robot and points along the path
        dist = np.sqrt(((x - curr_x)**2) + ((y - curr_y)**2))
            #index of current position < index of nearest waypoint/position to follow
            #return index of which waypoint/nearest point is closest
        return np.argmin(dist)

def lookahead(curr_x, curr_y):
    #position= np.array(self.xcar[self.index] , self.ycar[self.index])
    position= np.array([curr_x, curr_y]).T  #come back to it
    wp=nearest_point(waypoints, curr_x, curr_y)
    #-------------------(wp works)---------------------# 
    try:
        #for i in range (1,100): , don't need for now because will be running 100 times in main
        f= np.subtract(waypoints[wp] , position)
        if (wp<6):
            d= np.subtract(waypoints[wp+ 1] , waypoints[wp]) #think this is wrong. Might be wp and wp + 1 
        else:
            d= np.subtract(waypoints[wp] , waypoints[wp])
        a= np.dot(d,d); 
        b=2*np.dot(f, d); 
        c= np.dot(f,f) - ld*ld
        discriminant= (b*b)- (4*a*c) #not way this is wrong computers don't fuck up math lol 
        #print(discriminant)
        
        if (discriminant<0):
            raise NoIntersectionException
        else:
            discriminant= np.sqrt(discriminant)
            t1= (-b- discriminant)/(2*a)
            t2= (-b + discriminant)/(2*a)
            #print(f"t1: {t1}| t2: {t2}" )
            if (t1>=0 and t1<=1.0):
                point= waypoints[wp] + t1*d
                pointx= point[0]
                pointy=point[1]
                return pointx, pointy
                 #x and y value of ld
            elif (t2 >= 0 and t2 <=1.0):
                point = waypoints[wp] + t2*d
                pointx = point[0]
                pointy =point[1]
                return pointx, pointy
            else:
                #print("NOTHING LOLOLOLL")
                return 0,0

    except NoIntersectionException as e:
        print("Path lookahead no intersection")
        return -999, -999


def Curvature(pointx, pointy, position):
    #pointx,pointy are lkx and lky

    #following the arc to the lookahead point
    #15 should be car's orientation
    robot_angle= position[2]
    #car=Car()
    #wp=nearest_point(waypoints, [car.xcar[car.index], car.ycar[car.index], car.yaw[car.index]])
    robo_x= position[0]
    robo_y= position[1]
    robo_angle= np.tan(robot_angle)
    a= -1 * robo_angle
    c = robo_angle * robo_x - robo_y
    dist_x= abs(a*pointx + pointy + c)/math.sqrt(((a)**2)+1)   #b is 1 for some reason
    curvature=(2*dist_x)/(ld*ld) #dist_x lookaheag ponint??? 
    #d=ld-pointx

    # radius=np.sqrt((pointx-robo_x)**2+(pointy-robo_y)**2)
    # curvature=1/radius
    signed=np.sign(math.sin(robo_angle) * (pointx-robo_x) - math.cos(robo_angle) * (pointy-robo_y))

    signed_curvature= -1*curvature*signed
    return signed_curvature

def steering_angle(signed_curvature):
    steering_angle=np.arctan(signed_curvature*WB)
    return steering_angle

#---------------------------------------(Velcoity)---------------------------------# 
def distance(x0, x1, y0, y1):
  X = x1 - x0
  Y = y1 - y0
  return math.sqrt(pow(X, 2) + pow(Y, 2))
  #used to calculate the distance between endpoint and car's current distance
def newvel(d, a):
  v_1 = ((10 - a) + a * math.sqrt(1 + (4 * d))) / 2
  v_2 = ((10 - a) - a * math.sqrt(1 + (4 * d))) / 2
  #now need to check if the computed velcoiy values are valid
  #we want to have a velcoity less than 5.
  if 0 < v_1 < 5:
    return v_1

  #return these velcoity values
  elif 0 < v_2 < 5:
    #return these velcoity values
    return v_2


  #velcoity that goes to low level
def decel(thresh_d, curr_d, v_max, a_max):
  if thresh_d > curr_d:
    return newvel(curr_d, a_max)
  else:
    return v_max

# def newposition(steering_angle, ctrl_velocity):
#     ind=1
#     x_= np.zeros(N)
#     y_= np.zeros(N)
#     x_[ind] = x_[ind-1] + (5 * np.cos(steering_angle) * dt)
#     y_[ind] = y_[ind-1]  + (5 * np.sin(steering_angle) * dt)
#     ind+=1
#     return x_[ind-1] , y_[ind-1]
#     #hardcoded 5 for now, ask isaiah what his does''
def main():
    car= Car()
    v_m = 2  #constant 2m/s that won't change (for now)
    endpoint = [36, 6]   #get from tech team from globalcost map
    decel_dist = 7 
    a_max = -2   #passed into new vel
    #change curr_point to update function
     #distance btwn currpoint and endpoint 
     #calculate the velocity 
    rp = [0,0,0]

    #plt.figure(3)
    #fig, ax = plt.subplots(1)


    robotx = []
    roboty = []
    plt.title("Generic Matplotlib demo") 
    plt.xlabel("x axis caption") 
    plt.ylabel("y axis caption") 
    for i in range (800):
        
            #list but function takes in ints
        #rp = [car.xcar[car.index], car.ycar[car.index], car.yaw[car.index]]
        lkx, lky = lookahead(rp[0], rp[1])
        
        #print(f"rp: {rp} | lookahead: {lkx, lky}")
        # if(lkx > -990 or lky > -990):
        #     circ = plt.Circle((rp[0], rp[1]), radius=ld, fill=False)
        #     ax.plot(lkx, lky, 'r.')
        #     ax.add_patch(circ)
        curv = Curvature(lkx, lky, rp)

        steer= steering_angle(curv)
        dist = distance(rp[0], endpoint[0], rp[1], endpoint[1])
        v_next = decel(decel_dist, dist, v_m, a_max)
       # (curr_x, curr_y, curr_yaw) =  car.update(v_next,steer)
        rp =  car.update(v_next,steer)
        robotx.append(rp[0])
        roboty.append(rp[1])
        #plt.plot(lkx,lky, 'r', marker="o", markersize=5)

        print(f"steering angle: {steer} | curvature: {curv} rbx: {rp[0]}| rby: {abs(rp[1])} | vel: {v_next} | robo_angle: {rp[2]} | lkx, lky: {lkx, lky}")
        #print(f"orientation: {rp[2]}")




    for i in range(800): 
        k = robotx[i] 
        j = roboty[i]
        plt.plot(k,j, 'b', marker="o", markersize=5)
        


    plt.title("Matplotlib demo") 
    plt.xlabel("x axis caption") 
    plt.ylabel("y axis caption") 
    for i  in range(7):
        plt.plot(x[i], y[i], marker="o", markersize=5)
    plt.show()

    
if __name__ == "__main__":
    main()



#step1: make tan(robo_angle) = to lky -roboy / lkx - robox 

#step2: check over the signed thing 

#can check by using lk point and robot point along with radius 
#to reverse engineer the circle and see if it looks right 