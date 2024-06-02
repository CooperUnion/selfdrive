'''
Test FIII.2. Intersection Testing. Left Turn
1. Test Goal
This test is intended to evaluate if a vehicle is able to stop at the 'Stop' traffic sign, make a left turn
across the traffic, merge into expected lane and drive within this lane until an obstacle is detected.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface
import math 

class Function_Test_F3_2():
    def __init__(self, interface):
        self.interface = interface


    # State transistion logic
    def function_test(self):
        white_line_detected = False
        barrel_detected = False 
        white_line_data = []
        barrel_data = []
        self.interface.car_SM.Resume()
        distance_threshold = 5
        ZED_TO_BUMPER = 0.4
        stop_dist = 1.524

        while white_line_detected:   
            white_line_data = self.interface.Object_Detection(self.distance_threshold, object_lists=["White Line"]) 
            self.interface.Run()

            if(white_line_data[0]):
                white_line_detected = True 
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run(white_line_data[1])
        
        self.interface.car_SM.Turn_Trigger()
        self.interface.Run([4.5,7.6,math.pi/2])

        distance_threshold = 5
        while not barrel_detected:
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"])
            self.interface.Run()  

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
            self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)

if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F3_2)
