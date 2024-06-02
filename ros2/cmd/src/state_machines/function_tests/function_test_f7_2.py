from State_Machine_Executor import main
from State_Machine_Interface import Interface

class Function_Test_F7_2:
    # Get data from 5 meters away
    threshold_distance = 5  # meters

    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        x = 1 
        y = 1 
        yaw = 1 
        barrel = True 
        distance = 0
        ZED_TO_BUMPER = 0.4
        stop_dist = 1.524
        barrel_detected_count = 0
        barrel_data = []

        self.interface.car_SM.Turn_Trigger()
        self.interface.car_SM.Run()

        self.interface.car_SM.Return_To_Following

        distance_threshold = 5
        while not barrel_detected:
            
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"])
            self.interface.Run()  

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
            self.interface.Run((barrel_data[1]+ZED_TO_BUMPER) - stop_dist)




