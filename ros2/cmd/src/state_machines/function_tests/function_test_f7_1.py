'''
Test FVII.1 Pothole Detection
1. Test Goal
This test is intended to evaluate Ego vehicle’s ability to detect a pothole and safely change lane.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface

class Function_Test_F7_1():
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        pothole_detected = False
        barrel_detected = False
        pothole_data = []
        barrel_data = []
        self.interface.car_SM.Resume()

        ZED_TO_BUMPER = 0.4
        pothole_distance_threshold = 4.572
        barrel_distance_threshold = 3
        stop_dist = 0.9144

        while not pothole_detected:
            pothole_data = self.interface.Object_Detection(pothole_distance_threshold, object_lists=["PotHole"],check_in_lane=True)
            self.interface.Run()
            if(pothole_data[0]):
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                pothole_detected = True

        while not barrel_detected:
            barrel_data = self.interface.Object_Detection(barrel_distance_threshold, object_lists=["Barrel"],check_in_lane=True)
            self.interface.Run()  
            if(barrel_data[0]):
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)
                barrel_detected = True

if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F7_1)






