import math
import numpy as np
import matplotlib.pyplot as plt


ld=5 #define lookahead distance
dt=0.01 #time tick
WB= .98 #m
#rw= 0.5 #measure this
waypoints=np.array([   #create random sample waypoints that tech may give me based on desired spacing
        [0,0],
        [3,4],  #constant spacing
        [6,8],
        [9,12], 
        [12,16],
        [15,20],
        [18,24]
    ])
#how to initialize 2 column array of zeros
N=1000
n=len(waypoints)
#have functions outside of class and the tech box as the class  
#how far along the path the waypoint is, 7 points

class Car():

    def distance(self,waypoints): #may come into use in curvature function
        dist=np.zeros(N)
        x=waypoints[:, 0]
        y=waypoints[:, 1]
        min_index=0
        for i in range (1,n):
        #running sum of the distances between points
            dist[i]= dist[i-1]+ math.sqrt((x[i]-x[i-1])**2+(y[i]-y[i-1])**2)
            if(dist[i]<dist[min_index]):
                min_index=i 
                return dist[i]

    def nearest_point(self, waypoints, robotPose=[3,n]): #might be useful for not wanting to travel through the back of the path.
        x=waypoints[:, 0] 
        y=waypoints[:, 1]
        dist=np.zeros(N)
        robotPose= [self.xcar[self.index], self.ycar[self.index], self.yaw[self.index]]
        xR=robotPose[:, 0]
        yR=robotPose[:, 1]
        for i in range (1,n):
           #distance between robot and points along the path
            dist[i]= math.sqrt((x-self.xcar[self.index])**2+(y-self.ycar[self.index])**2)
            if(self.index<i): 
            #index of current position < index of nearest waypoint/position to follow
            #return index of which waypoint/nearest point is closest
                min_index=i 
                return min_index

    def __init__(self, xcar=np.zeros(N), ycar=np.zeros(N), yaw=np.zeros(N), vel=np.zeros(N)):
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
        self.index=0

    def update(self, acc=0, delta=0):
        #acc and delta are not arrays
        """
        Vehicle motion model, here we are using simple bycicle model
        :param acc: float, acceleration
        :param delta: float, heading control
        """
        self.index+=1
        self.xcar[self.index] = self.xcar[self.index-1]+ self.vel[self.index-1] * np.cos(self.yaw[self.index-1]) * dt
        self.ycar[self.index] = self.xcar[self.index-1]+self.vel[self.index-1] * np.sin(self.yaw[self.index-1]) * dt
        self.yaw[self.index] = self.vel[self.index-1] * np.tan(delta) / WB * dt
        self.vel[self.index] = self.vel[self.index-1]+acc*dt

def main():
    #create an initialized object of Car, no need params cuz already 0s
    car= Car()
    for i in range (5):
        car.update(2, 1)
        rp= [car.xcar[car.index], car.ycar[car.index], car.yaw[car.index]]
        print(Car.nearest_point(waypoints, rp))
    print(Car.distance(waypoints))
    plt.figure(1)
    plt.plot(car.xcar,car.ycar,'b.')
    plt.show()

if __name__ == "__main__":
    main()





