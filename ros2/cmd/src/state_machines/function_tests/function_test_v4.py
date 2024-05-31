#Test FV4. Obstacle detection. Lane Changing (goes straight, detects obstacle, and changes lanes)

threshold_distance = 4.572

class Function_Test_4():
    def __init__(self, interface):
        self.interface = interface
        pass

    # State transistion logic
    def function_test(self):
        barrel_counter = 0
        self.interface.car_SM.Resume()
        while barrel_counter < 2:            
            if(self.interface.Object_Detection(threshold_distance, cared_objects=["Barrel"],)):
                self.interface.car_SM.Obj_Avoidance()
                barrel += 1
            self.interface.Run()
        
        self.interface.car_SM.Stop_Trigger()
        self.interface.Run()
                




