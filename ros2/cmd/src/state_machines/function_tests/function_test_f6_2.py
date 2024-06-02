'''
1. Test Goal
This test is intended to evaluate Ego vehicle’s ability to stay in the lane on a curved road, and be able to stop at
the obstacle within a current lane. This test consists of 4 possible case scenarios: driving in right lane on the left
curve, driving in left lane on the left curve, driving in right lane on the right curve and driving in left lane on the right
curve. Any of above scenarios could be chosen at judges’ discretion.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface
import math 

class Function_Test_F6_1():
    distance_threshold = 5 
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        distance_threshold = 5
        ZED_TO_BUMPER = 0.4
        stop_dist = 1.524

        barrel_detected_count = 0
        barrel_data = []

        self.interface.car_SM.Resume()

        distance_threshold = 3 # About 10ft 
        while barrel_detected_count < 2:
            
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"],check_in_lane = True)
            self.interface.Run()  

            if(barrel_data[0]):
                barrel_detected_count += 1

                if(barrel_detected_count == 1):
                    multiplier = 1 if self.interface.current_lane() else -1
                    self.interface.Run(barrel_data[1],barrel_data[2] + multiplier * 3 , 0.0)
                    self.interface.Run() 

                    
                if(barrel_detected_count == 2):
                    self.interface.car_SM.Stop_Trigger()







                


            self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)

