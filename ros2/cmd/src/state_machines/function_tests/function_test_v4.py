class Function_Test_4():
    def __init__(self, interface):
        self.interface = interface
        pass

    # State transistion logic
    def function_test(self):
        barrel_counter = 0
        self.interface.car_SM.Resume()
        while barrel_counter < 2:            
            if(self.interface.Object_Detection(cared_objects=["Barrel"])):
                self.interface.car_SM.Obj_Avoidance()
                barrel += 1
            self.interface.Run()
        
        self.interface.car_SM.CStop()
        self.interface.Run()
                




