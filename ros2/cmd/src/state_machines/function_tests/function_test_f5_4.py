#Test FV4. Obstacle detection. Lane Changing (goes straight, detects obstacle, and changes lanes)


from State_Machine_Executor import main
from State_Machine_Interface import Interface

class Function_Test_F3_1():
    def __init__(self, interface):
        self.interface = interface


    # State transistion logic
    def function_test(self):
        barrel_count = 0
        barrel_data = []
        self.interface.car_SM.Resume()

        ZED_TO_BUMPER = 0.4
        barrel1_distance_threshold = 4.572
        barrel2_distance_threshold = 3
        stop_dist = 0.9144

        while barrel_count < 1:
            barrel_data = self.interface.Object_Detection(barrel1_distance_threshold, object_lists=["Barrel"],check_in_lane=True)
            self.interface.Run()
            if(barrel_data[0]):
                self.interface.car_SM.Obj_Avoidance()
                if(self.interface.current_lane):
                    self.interface.Run([0.254,-3.048,0])
                else: 
                    self.interface.Run([0.254,3.048,0])
                barrel_detected += 1

        while barrel_count == 1:
            barrel_data = self.interface.Object_Detection(barrel2_distance_threshold, object_lists=["Barrel"],check_in_lane=True)
            self.interface.Run()  
            if(barrel_data[0]):
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)







