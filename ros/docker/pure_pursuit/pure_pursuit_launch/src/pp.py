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

class NoIntersectionException(Exception):
    def __init__(self):
        self.message = 'no intersection'

class Car():

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
        self.xcar[self.index] = self.xcar[self.index-1]+ (self.vel[self.index-1] * np.cos(self.yaw[self.index-1]) * dt)
        self.ycar[self.index] = self.ycar[self.index-1]+ (self.vel[self.index-1] * np.sin(self.yaw[self.index-1]) * dt)
        self.yaw[self.index] = self.vel[self.index-1] * (np.tan(delta) / WB) * dt + self.yaw[self.index-1]
        self.vel[self.index] = self.vel[self.index-1]+acc*dt

#--------------------------------------------(Steering Angle)--------------------------------------#
def distance(waypoints): #may come into use in curvature function
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


def nearest_point(waypoints, rp): #might be useful for not wanting to travel through the back of the path.
        x=waypoints[:, 0] 
        y=waypoints[:, 1]
        dist=np.zeros(N)
           #distance between robot and points along the path
        dist = np.sqrt(((x - rp[0])**2) + ((y - rp[1])**2))
            #index of current position < index of nearest waypoint/position to follow
            #return index of which waypoint/nearest point is closest
        return np.argmin(dist)

def lookahead(position):
    a=b=c=f=d=point=pointx=pointy=discriminant=np.zeros(n) 
    #position= np.array(self.xcar[self.index] , self.ycar[self.index])
    position= np.array([position[0], position[1]]).T  

    try:
        for i in range (1,7):
            f= np.subtract(waypoints[i-1] , position)
            d= np.subtract(waypoints[i] , waypoints[i-1])
            a= np.dot(d,d); b=2*np.dot(f, d); c= np.dot(f,f) - ld*ld
            discriminant= b*b-4*a*c
            if (discriminant<0):
                raise NoIntersectionException
            else:
                discriminant= np.sqrt(discriminant)
                t1= (-b- discriminant)/(2*a)
                t2= (-b + discriminant)/(2*a)
                if (t1>=0 and t1<=1.0):
                    point= waypoints[i-1] + t1*d
                    #pointx=point[:, 0]
                    pointx= point[0]
                    #pointy=point[:, 1]
                    pointy=point[1]
                    return pointx , pointy #x and y value of ld
                elif (t2 >= 0 and t2 <=1.0):
                    point= waypoints[i-1] + t2*d
                    #pointx=point[:, 0]
                    pointx= point[0]
                    #pointy=point[:, 1]
                    pointy=point[1]
                    return pointx , pointy
        raise Exception
    except NoIntersectionException as e:
        print("Path lookahead no intersection")
        return -999, -999

def Curvature(pointx, pointy):

        #following the arc to the lookahead point
        x=waypoints[:, 0] 
        y=waypoints[:, 1]
        ldx=a=c=signed=signed_curvature=mag=orientation=curvature=np.zeros(n) 
        for i in range (1,n):

            mag= np.sqrt(-np.tan(orientation)*(-np.tan(orientation))+1)

            ldx= np.abs(-np.tan(orientation)*pointx+pointy+np.tan(orientation)*x-y)/(mag)

            curvature[i]=(2*ldx[i])/(ld*ld) #this is just magnitude

            #but this is unsigned, need to know which sign it is on, so cross product
 
            signed=np.sin(orientation)*(pointx-np.cos(orientation))-np.cos(orientation)*(pointy-np.sin(orientation))
            signed_curvature[i]= curvature[i]*signed[i]

            return signed_curvature

def steering_angle(signed_curvature):
    steering_angle=np.arctan(signed_curvature*ld)
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

def main():
    car= Car()

#--------------------------------(First_Block)--------------------------------------------------
    '''
    for i in range (500):
        car.update(2, 5)
        rp = [car.xcar[car.index], car.ycar[car.index], car.yaw[car.index]]

    nearest_point(waypoints, rp)  
    print([car.xcar, car.ycar, car.yaw])
    print(nearest_point)
    plt.figure(1)
    plt.plot(car.xcar,car.ycar,'b.')
    plt.show()
    '''
#------------------------(Second_Block)------------------------------------------------------
    v_m = 5  #constant 5mph that won't change (for now)
    endpoint = [7, 7]
    curr_point = [2, 5]  #list but function takes in ints
    decel_dist = 7
    a_max = -2  #passed into new vel #m

    dist = distance(curr_point[0], endpoint[0], curr_point[1], endpoint[1])
    v_next = decel(decel_dist, dist, v_m, a_max)


    fig, ax = plt.subplots(1)
    for i in range (50):
        car.update(2,0)
        rp= [car.xcar[car.index], car.ycar[car.index], car.yaw[car.index]]
        lkx, lky = lookahead(rp)
        print(f"rp: {rp} | lookahead: {lkx, lky}")
        if(lkx > -990 or lky > -990):
            circ = plt.Circle((rp[0], rp[1]), radius=ld, fill=False)
            ax.plot(lkx, lky, 'r.')
            ax.add_patch(circ)
        curv = Curvature(lkx, lky)
        steer= steering_angle(curv)
        print(f"steering angle: {steer} | curvature: {curv}" )
    
   # num  = nearest_point(waypoints, rp) 
    #print(num) 

    #print(curv)

    #print(lookahead(rp))
    #print([car.xcar, car.ycar, car.yaw])
    plt.figure(1)
    plt.plot(waypoints[:, 0], curv,'b.')
    plt.plot()
    plt.show()

    plt.figure(2)
    plt.plot(waypoints[:, 0], steer,'r.')
    plt.plot()
    plt.show()

    print(v_next)
    

if __name__ == "__main__":
    main()
