# Test Q.3 Lane Keeping (Go straight until barrel detected and stop)

distance_threshold = 5


class Function_Test_4:
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        barrel = 0
        self.interface.car_sm.Resume()
        while barrel < 1:
            if self.interface.Object_Detection(
                distance_threshold, object_list=["barrel"]
            ):
                print("barrel Detected")
                #                self.interface.car_sm.Obj_Avoidance()
                barrel += 1
            self.interface.Run()

        self.interface.car_sm.Stop_Trigger()
        self.interface.Run()
