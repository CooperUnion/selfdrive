'''
Test FIII.3. Intersection Testing. Right Turn
1. Test Goal
This test is intended to evaluate if a vehicle is able to stop at the 'Stop' traffic sign, make a right turn,
merge into the lane and drive within a lane until an obstacle is detected.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface
import math 

class Function_Test_F3_3():
    def __init__(self, interface: Interface):
        self.interface = interface


    # State transistion logic
    def function_test(self):
        stop_line_detected = False
        barrel_detected = False 
        stop_line_data = []
        barrel_data = []
        self.interface.car_SM.Resume()
        distance_threshold = 5
        ZED_TO_BUMPER = 0.4
        stop_dist = 1.524

        while stop_line_detected:   
            stop_line_data = self.interface.Object_Detection(self.distance_threshold, object_lists=["Stop Line"]) 
            self.interface.Run()

            if(stop_line_data[0]):
                stop_line_detected = True 
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(stop_line_data[1])
        
        self.interface.car_SM.Turn_Trigger()
        self.interface.Run([1, 5, -1.5, math.pi / 2])

        distance_threshold = 5
        while not barrel_detected:
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"])
            self.interface.Run()  

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
            self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)

if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F3_3)

