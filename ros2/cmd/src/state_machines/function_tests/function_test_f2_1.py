#Test FVII.1 Pothole Detection

threshold_distance = 4.572

class Function_Test_4():
    def __init__(self, interface):
        self.interface = interface
        pass

    # State transistion logic
    def function_test(self):
        object_counter = 0
        self.interface.car_SM.Resume()
        while object_counter < 2:            
            if(self.interface.Object_Detection(threshold_distance, cared_objects=["Pothole"])):
                self.interface.car_SM.Obj_Avoidance()
                object_counter+=1
            if(self.interface.Object_Detection(cared_objects=["Barrel"],)):
                self.interface.car_SM.Stop_Trigger()
                object_counter+=1
                self.interface.Run()
