'''
Test FIII.1. Lane Keeping
1. Test Goal
This test is intended to evaluate if the vehicle is able maneuver within lane boundaries, without wheels
crossing the line or driving on the line. Additionally, this test evaluates if the vehicle stops at the “Stop”
sign at the intersection, goes straight through intersection, and stops before an obstacle placed on the
road.
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface

class Function_Test_F3_1():
    def __init__(self, interface: Interface):
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
        self.interface.Run([4.5,0,0])

        distance_threshold = 5
        while not barrel_detected:
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"])
            self.interface.Run()  

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
            self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)
            
if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F3_1)


