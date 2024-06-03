# Full course 1
from State_Machine_Executor import main
from State_Machine_Interface import Interface

import math


class Function_Test_Q5:
    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        distance_threshold = 5
        white_lines_count = 0
        barrel_count = 0
        pothole_detected = False
        white_line_data = []
        barrel_data = []

        while white_lines_count<1:
            white_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["White Line"]
            )

            if white_line_data[0]:
                white_lines_count+=1
                self.interface.car_sm.Turn_Trigger()
                self.interface.Run(white_line_data[1])
                

        self.interface.Run([4.5, 7.6, math.pi / 2])
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
        self.interface.Run([0.254,-3.048,0])

        while white_lines_count==1:
            white_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["White Line"]
            )
            self.interface.Run()

            if white_line_data[0]:
                white_lines_count+=1
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(white_line_data[1])

        self.interface.car_sm.Turn_Trigger()
   
        self.interface.Run([4.5, 7.6, math.pi / 2])
        self.interface.car_sm.Return_To_Follow()


        while white_lines_count==2:
            self.interface.Run()
            person_data = self.interface.Object_Detection(3.5, cared_objects=["Person"], check_in_lane=True)
            if(person_data[0]):
                self.interface.carSM.Obj_Avoidance()
                multiplier = 1 if self.interface.current_lane() else -1
                self.interface.Run(person_data[1],person_data[2] + multiplier * 3 , 0.0)
                white_lines_count+=1

        self.interface.car_SM.Return_To_Follow()
        self.interface.Run()

        while barrel_count==2:
            barrel_data = self.interface.Object_Detection(0.9, cared_objects=["Barrel"],)
            self.interface.Run()

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run((barrel_data[1]+0.4) - 1.524)
        
        self.interface.car_SM.Return_To_Follow()
        self.interface.Run()

        while not pothole_detected:
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["PotHole"],check_in_lane=True)
            self.interface.Run()
            if(barrel_data[0]):
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                barrel_detected += 1

        self.interface.car_sm.Return_To_Follow()

        while white_lines_count==3:
            white_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["White Line"]
            )
            self.interface.Run()

            if white_line_data[0]:
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(white_line_data[1])
                white_lines_count+=1

if __name__ == "__main__":
    main("Function_Test_Q3", Function_Test_Q5)
