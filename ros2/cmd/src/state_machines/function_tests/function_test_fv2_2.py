class Function_Test_FV2():
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        self.interface.car_SM.Resume()
        self.interface.Run()
                
        # WANT SOME TIME TO WAIT UNTIL DOING THIS 
        self.interface.car_SM.Obj_Avoidance()
        self.interface.Run()

        if(self.interface.Object_Detection(object_list=["Barrel"],)):
            self.interface.car_SM.Stop_Trigger()
            self.interface.Run()


