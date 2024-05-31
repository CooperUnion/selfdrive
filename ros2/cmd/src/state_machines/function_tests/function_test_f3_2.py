#Test FIII.1. Lane Keeping (vehicle stops next to stop sign makes right turn then continues and stops at barrel)

import time

t=5

class Function_Test_F3_1():
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        object_counter = 0
        self.interface.car_SM.Resume()
        while object_counter < 1:            
            if(self.interface.Object_Detection(cared_objects=["Stop Sign"],)): #obj Could be white line or stop sign 
                #make a right turn
                time.sleep(5)
                self.interface.car_SM.Obj_Avoidance()
                object_counter += 1
            self.interface.Run()
        
        self.interface.car_SM.Return_To_Follow()
        self.interface.Run()

        if(self.interface.Object_Detection(cared_objects=["Barrel"],)):
            self.interface.car_SM.Stop_Trigger()
            self.interface.Run()


