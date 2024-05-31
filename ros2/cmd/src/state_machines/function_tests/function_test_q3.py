class Function_Test_Q3():
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        barrel_counter = 0
        self.interface.car_SM.Resume()
        while barrel_counter < 1:            
            if(self.interface.Object_Detection(cared_objects=["Barrel"],)):
                self.interface.car_SM.Obj_Avoidance()
                barrel_counter += 1
            self.interface.Run()
        
        self.interface.car_SM.Stop_Trigger()
        self.interface.Run()
