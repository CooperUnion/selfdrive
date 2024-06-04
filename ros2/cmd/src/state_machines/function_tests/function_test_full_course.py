# Full course 1
from State_Machine_Executor import main
from State_Machine_Interface import Interface

import math


class Function_Test_Full_Course_1():
    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        distance_threshold = 5
        stop_line_count = 0
        barrel_count = 0
        pothole_detected = False
        stop_line_data = []
        barrel_data = []
        pothole_data = []

        while stop_line_count<1:
            stop_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["Stop Line"]
            )

            if stop_line_data[0]:
                stop_line_count+=1
                self.interface.car_sm.Turn_Trigger()
                self.interface.Run(stop_line_data[1])
                

        self.interface.Run([4.5, -7.6, math.pi / 2])
        self.interface.car_sm.Return_To_Follow()

        while barrel_count < 1:
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"],check_in_lane=True)
            self.interface.Run()
            if(barrel_data[0]):
                barrel_detected += 1
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                

        self.interface.car_SM.Obj_Avoidance()
        self.interface.Run([3,0,0]) # might need to change this rel x 
        self.interface.Run([0.254,-3.048,0])# come back to this 

        self.interface.car_sm.Return_To_Follow()



        while stop_line_count==1:
            stop_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["Stop Line"]
            )
            self.interface.Run()

            if stop_line_data[0]:
                stop_line_count+=1
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(stop_line_data[1]) # input param needs to be changed

        self.interface.car_sm.Turn_Trigger()
   
        self.interface.Run([4.5, -7.6, math.pi / 2])
        self.interface.car_sm.Return_To_Follow()


        while stop_line_count==2:
            self.interface.Run() # checking in lane might present an issue 
            person_data = self.interface.Object_Detection(3.5, cared_objects=["Stop Line"], check_in_lane=True)
            if(person_data[0]):
                self.interface.carSM.Obj_Avoidance()
                multiplier = 1 if self.interface.current_lane() else -1
                self.interface.Run(person_data[1],person_data[2] + multiplier * 3 , 0.0)
                stop_line_count+=1

        self.interface.car_SM.Return_To_Follow()
        self.interface.Run()



        while not pothole_detected:
            pothole_data = self.interface.Object_Detection(distance_threshold, object_lists=["PotHole"],check_in_lane=True)
            self.interface.Run()
            if(pothole_data[0]):
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                pothole_detected = True

        self.interface.car_sm.Return_To_Follow()

        while barrel_count==2:
            barrel_data = self.interface.Object_Detection(0.9, cared_objects=["Barrel"],)
            self.interface.Run()

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
        
        self.interface.car_SM.Return_To_Follow()
        self.interface.Run()

        while stop_line_count==3:
            stop_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["Stop Line"]
            )
            self.interface.Run()

            if stop_line_data[0]:
                stop_line_count+=1
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(stop_line_data[1])

if __name__ == "__main__":
    main("Function_Test_Q3", Function_Test_Full_Course_1)
