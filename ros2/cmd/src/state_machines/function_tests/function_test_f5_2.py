'''
1. Test Goal
This test evaluates ability of Ego vehicle to stop if an obstructed by barrel pedestrian (mannequin)
suddenly starts crossing an Ego’s vehicle lane.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface
import math 

class Function_Test_F5_2():
    distance_threshold = 5 
    def __init__(self, interface):
        self.interface = interface


    # State transistion logic
    def function_test(self):

        person_detected = False
        barrel_detected = False
        person_stop_threshold = 5 
        person_data = []
        barrel_data = []
        self.interface.car_SM.Resume()
        distance_threshold = 5
        ZED_TO_BUMPER = 0.4
        stop_dist = 1.524

        while person_detected:   
            person_data = self.interface.Object_Detection(self.distance_threshold, object_lists=["Person"],check_in_lane=True) 
            self.interface.Run()

            if(person_data[0]):
                person_detected = True 
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run(person_data[1] - person_stop_threshold)

        
        self.interface.car_SM.Resume() 

        distance_threshold = 5
        while not barrel_detected:
            
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"])
            self.interface.Run()  

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
            self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)
            
if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F5_2)

