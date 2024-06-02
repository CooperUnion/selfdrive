'''
Test FVI.2 Curved Road Evaluation. Lane Changing
1. Test Goal
This test is intended to evaluate if a vehicle is able to perform a lane change on the curved road if
obstacles are detected. This test consists of 4 possible case scenarios: changing right lane on the left
curve, changing left lane on the left curve, changing right lane on the right curve and changing left lane
on the right curve. Any of above scenarios could be chosen as this year’s test.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface

class Function_Test_F6_2():
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

if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F6_2)

