from clothoid_path_planner import generate_clothoid_path
from collections import namedtuple
import matplotlib.pyplot as plt 
import numpy as np
import sympy as sym 
import math 

Point = namedtuple("Point", ["x", "y"])
TYPE  = 0 

#type of section based on function 



class path_generation: 

    # def __init__(self, start_point, start_orientation, goal_point, goal_orientation, num_path_points):

    #     self.start_pont = start_point
    #     self.start_orientation = start_orientation
    #     self.goal_point = goal_point
    #     self.goal_orientation = goal_orientation
    #     self.num_path_points = num_path_points

    def plot_clothoid(self,clothoid_path,start_point,goal_point,A,B,C,center,radius):
        figure, axes = plt.subplots()
        plt.plot(start_point[0],start_point[1], 'o', color = "red")
        plt.plot(goal_point[0],goal_point[1], 'o', color = "green")
        plt.plot(clothoid_path[0],clothoid_path[1])
        plt.plot(clothoid_path[0],clothoid_path[1])
        plt.title("Making A Circle w/ 3 points")
        # plt.xlabel("car_reference x_axis")
        # plt.ylabel("car_reference y axis")
        plt.plot(A[0], A[1], marker="X", color="c", markersize=5)
        plt.plot(B[0], B[1], marker="X", color="c", markersize=5)
        plt.plot(C[0], C[1], marker="X", color="c", markersize=5)

        plt.plot(clothoid_path[0][50], clothoid_path[1][50], marker="X", color="r", markersize=5)
        


        #plot a circle 

        Drawing_uncolored_circle = plt.Circle( (center[0], center[1] ),
                                      radius ,
                                      fill = False )
 
        axes.set_aspect( 1 )
        axes.add_artist( Drawing_uncolored_circle )
        plt.title( 'Circle' )

    
        plt.show()
    def calculate_yaw(self, cx, cy, start_yaw):
        yaw = [start_yaw]
        for i in range(1, len(cx)):
            dx = cx[i] - cx[i-1]
            dy = cy[i] - cy[i-1]
            angle = np.arctan2(dy, dx) # calculate the angle between the two points
            yaw.append(angle)
            
        return yaw
    
    def vertical_line(self,start_point,start_orientation,goal_point,goal_orientation):

        global TYPE 
        TYPE = 1

        # start_point = Point((19.25/2), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((19.25/2), 114) #assume in ft
        # goal_orientation = np.pi / 2
        num_path_points = 100

        vertical_line = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return vertical_line

    def horizontal_line(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 2
        # start_point = Point(0, 114) 
        # start_orientation = 0
        # goal_point = Point((19.25/2), 114) #assume in ft
        # goal_orientation = 0
        num_path_points = 100

        horizontal_line = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return horizontal_line


    def right_turn(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 3
        # start_point = Point((0), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((5.76), 5.76) #assume in m
        # goal_orientation = 0
        num_path_points = 100

        right_turn = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return right_turn


    def left_turn(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 4
        # start_point = Point(0,0) 
        # start_orientation =  np.pi / 2
        # goal_point = Point((-9.5), (9.5)) #assume in ft
        # goal_orientation = np.pi
        num_path_points = 100

        left_turn = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return left_turn
    
    def swerve_R2L(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 5
        # start_point = Point(0,0) 
        # start_orientation =  np.pi / 2
        # goal_point = Point((1.175), (3)) #assume in ft
        # goal_orientation = np.pi / 2
        num_path_points = 10

        swerve = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return swerve
    def swerve_L2R(self,start_point,start_orientation,goal_point,goal_orientation):
        #measure this tommorow 
        global TYPE 
        TYPE = 6
        # start_point = Point(0,0) 
        # start_orientation =  np.pi / 2
        # goal_point = Point((1.175), (3)) #assume in ft
        # goal_orientation = np.pi / 2
        num_path_points = 100

        swerve = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return swerve
    
    def par_park(self,start_point,start_orientation,goal_point,goal_orientation):
        #measure this tommorow 
        global TYPE 
        TYPE = 7
        # start_point = Point(0,0) 
        # start_orientation =  (3 * np.pi) / 2
        # goal_point = Point((3), (-4.9)) #assume in ft
        # goal_orientation = (3 *np.pi) / 2
        num_path_points = 10

        swerve = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return swerve
    
    def merge(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 8
        # start_point = Point((0), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((-4.5), 6.4) #assume in m
        # goal_orientation = (np.pi * 3) / 4
        num_path_points = 100

        merge = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return merge
    
    def Curve_Lane_C1(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 8
        # start_point = Point((0), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((-1.5), 2.6) #assume in m
        # goal_orientation = (np.pi * 2) / 3
        num_path_points = 100

        curve_lane = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return curve_lane
    
    def Curve_Lane_C2(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 8
        # start_point = Point((0), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((-4.5), 6.4) #assume in m
        # goal_orientation = (np.pi * 3) / 4
        num_path_points = 100

        curve_lane = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return curve_lane
    
    def Curve_Lane_C3(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 8
        # start_point = Point((0), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((-4.5), 6.4) #assume in m
        # goal_orientation = (np.pi * 3) / 4
        num_path_points = 100

        curve_lane = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return curve_lane
    
    def Curve_Lane_C4(self,start_point,start_orientation,goal_point,goal_orientation):
        global TYPE 
        TYPE = 8
        # start_point = Point((0), 0) 
        # start_orientation = np.pi / 2
        # goal_point = Point((-4.5), 6.4) #assume in m
        # goal_orientation = (np.pi * 3) / 4
        num_path_points = 100

        curve_lane = generate_clothoid_path(start_point, start_orientation, goal_point, goal_orientation, num_path_points)

        return curve_lane

    def perp_bi(self,P1, P2):
        mp = [(P1[0] + P2[0]) / 2, (P1[1] + P2[1]) / 2]
        slope = (P2[1] - P1[1]) / (P2[0] - P1[0])
        pb_slope = -1 / slope
        x = sym.Symbol('x')
        # y = sym.Symbol('y')
        perp_line = (pb_slope * (x - mp[0])) + mp[1]
        # wp_line = (slope * (x - P2[0]) + P2[1])
        # sym.plot((perp_line,(x, -3, 3)), (wp_line,(x, -3, 3)))
        #print(perp_line)
        #print(wp_line)
        return perp_line 
    
    
    def steering_angle(self,path):
        wheel_base = 1.65
        num_path_points = 100
        x = sym.Symbol('x')
        y = sym.Symbol('y')
        #if at last waypoint go two back
        #if at first waypoint go two foward
        #not at either do one foward and one behind

        #assign a and b to be random points from the circle 




        x_p = path[0]
        y_p = path[1]

        delta_array = []


        for n in range(2,num_path_points - 1):
            A  = (x_p[n-1],y_p[n-1])
            B  = (x_p[n],y_p[n])
            C  = (x_p[n+1],y_p[n+1])

        # A = (x_p[0],y_p[0])
        # B = (x_p[50],y_p[50])
        # C = (x_p[-1],y_p[-1])
        

        #-----------------------------(Defining A Circle)-----------------------------#
         
        # print(wp)
        # if (wp == 0):
        #     print("First waypoint detected")
        #     A = waypoints[wp]
        #     B = waypoints[wp + 1]
        #     C = waypoints[wp + 2]
        # elif (wp == len(waypoints) - 1):
        #     print("Last waypoint found")

        #     A = waypoints[wp]
        #     B = waypoints[wp - 1]
        #     C = waypoints[wp - 2]
        # else:
        #     print("Neither lol")
        #     A = waypoints[wp - 1]
        #     B = waypoints[wp]
        #     C = waypoints[wp + 1]

        #find the midpoint
            line1 = self.perp_bi(A, B) - y #m
            line2 = self.perp_bi(B, C) - y #m
            # print(line1)
            # print(line2)
            center = list(sym.solve([line1, line2], [x, y], dict=False).values()) 
            radius = math.sqrt((A[0] - center[0])**2 + (A[1] - center[1])**2)  #m

            start_point = Point(0,0)
            goal_point = Point(5.76,5.76)  
            # self.plot_clothoid(path,start_point, goal_point,A,B,C,center,radius)
            

            # figure, axes = plt.subplots()
            # plt.xlim([0, 500])
            # # plt.ylim([0, 500])
            # Drawing_uncolored_circle = plt.Circle((.5, .5),2,fill=False)
            # axes.set_aspect(1)
            # axes.add_artist(Drawing_uncolored_circle)
            # plt.show()

                 

            steering_angle = math.degrees(math.atan( wheel_base * (1/radius)))


            if(TYPE == 5):
                if(n >= 50):
                    steering_angle = steering_angle * -1
            if(TYPE == 6): 
                if(n <= 50):
                    steering_angle = steering_angle * -1
            


            steering_angle = np.clip(steering_angle, -25, 25)


            


            delta_array.append(steering_angle)
            arc_length = abs(goal_point[0]) * (np.pi / 2)
            arc_array = np.linspace(0,arc_length,(n-1))
            

            #math: 10.411953035418401
            #machine: 


        delta_array1 = np.array(delta_array)
        print(max(delta_array1))
        # print ((arc_array))
        # print((delta_array1))
        # print(len(delta_array)) 
        # print(len(arc_array))

        # combined = np.vstack((arc_length, delta_array1)).T
        # print(combined)

        # con = np.concatenate((arc_array, delta_array1))
        con = np.stack((arc_array, delta_array1), axis=1)
        print(con)



        return steering_angle
    
        # print(center[0], center[1], radius)

        #0,0,0
        #9.5,9.5
        #5.76,5.76

    def arclength(self,path):
        x_p = path[0]  #x points
        y_p = path[1]
        N = 10

        dist = 0
        for i in range (N - 1):
            dist += np.sqrt(((x_p[i] - x_p[i+1])**2) + ((y_p[i] - y_p[i+1])**2))
            time = dist/1.5
            # get total distance along arc 
        return time 
    
    def stan_inputs(self,path):
        cx = path[0]
        cy = path[1]
        start_yaw = np.pi / 2
        cyaw = self.calculate_yaw(cx,cy,start_yaw)
        cyaw = np.array(cyaw)
        con = np.stack((cx, cy,cyaw), axis=1)

        return con
        

def main(): 
    # start_point = Point((19.25/2), 114) 
    # start_orientation = np.pi / 2
    # goal_point = Point((19.25/2 + 22 + 9), 114 + 22) #assume in ft
    # goal_orientation = 0
    start_point = Point((0), 0) 
    start_orientation = np.pi / 2
    goal_point = Point((-1.5), 2.6) #assume in m
    goal_orientation = (np.pi * 2) / 3
    num_path_points = 100

    # make_path = path_generation(start_point, start_orientation, goal_point, goal_orientation, num_path_points) 
    make_path = path_generation()
    path = make_path.Curve_Lane_C1(start_point,start_orientation,goal_point,goal_orientation)
    
    print(TYPE)
    delta = make_path.steering_angle(path)
    distance_4_swerve = make_path.arclength(path)
    yaws = make_path.stan_inputs(path)


    print("AC")
    print(distance_4_swerve)
    print(yaws)

    # print(delta)

if __name__ == "__main__":
    main()


    #reference is 5ft from the zed to the barrel 

#start and end orientation of the car do not affect the curvature 
        


        



