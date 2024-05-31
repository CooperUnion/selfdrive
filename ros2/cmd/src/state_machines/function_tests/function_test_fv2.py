#Test FV.2 Obstructed DYNAMIC pedestrian detection

distance_threshold = 1.524

class Function_Test_FV2():
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        self.interface.car_SM.Resume()
        while self.interface.Object_Detection(distance_threshold, cared_objects=["Person"]):             
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run()
                
        self.interface.car_SM.Resume()

        if(self.interface.Object_Detection(distance_threshold, cared_objects=["Barrel"],)):
            self.interface.car_SM.Stop_Trigger()
            self.interface.Run()


