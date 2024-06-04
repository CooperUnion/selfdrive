# Full course 2
from State_Machine_Executor import main
from State_Machine_Interface import Interface

import math


class Function_Test_Full_Course_2():
    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        distance_threshold = 5
        stop_line_count = 0
        tire_count = 0
        barrel_detected = False
        stop_line_data = []
        barrel_data = []
        stop_sign_data = []
        tire_data = []


        self.interface.car_SM.Obj_Avoidance()
        self.interface.Run([5,0,0]) #probably need to tune this distance
        self.interface.car_sm.Return_To_Follow()

        while tire_count< 1:
            tire_data = self.interface.Object_Detection(distance_threshold, object_lists=["Tire"], check_in_lane=True)
            self.interface.Run()
            if(tire_data[0]):
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                    tire_count+=1

        self.interface.car_sm.Return_To_Follow()

        while tire_count==1:
            tire_data = self.interface.Object_Detection(distance_threshold, object_lists=["Tire"], check_in_lane=True)
            self.interface.Run()
            if(tire_data[0]):
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                    tire_count+=1

        self.interface.car_sm.Return_To_Follow()


        #STOP SIGN STUFF
        while stop_line_count<1:
            stop_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["Stop Line"]
            )
            stop_sign_data = self.interface.Object_Detection(
                distance_threshold, object_list=["Stop Sign"]
            )

            if stop_sign_data[0]:
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(stop_line_data[1])
            else:
                self.interface.car_sm.Turn_Trigger()
                self.interface.Run([4.5, 7.6, math.pi / 2])
                
        self.interface.car_SM.Obj_Avoidance()
        self.interface.Run([3,0,0]) #probably need to tune this distance
        self.interface.car_sm.Return_To_Follow()

        while not barrel_detected:
            barrel_data = self.interface.Object_Detection(distance_threshold, object_list=["Barrel"])
            self.interface.Run()

        if barrel_data[0]:
            barrel_detected = True
            self.interface.car_sm.Stop_Trigger()
            self.interface.Run(barrel_data[1]) # input param needs to be changed


if __name__ == "__main__":
    main("Function_Test_Q3", Function_Test_Full_Course_2)
