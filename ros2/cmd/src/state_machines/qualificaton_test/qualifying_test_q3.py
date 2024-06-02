# Test Q.3 Lane Keeping (Go straight and stop at barrel)

# Get data from 2 metres away
threshold_distance = 5  # meters


class Function_Test_Q3:
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        barrel_counter = 0
        distance = 0
        self.interface.car_sm.Resume()
        while barrel_counter < 1:
            obj_data = self.interface.Object_Detection(
                threshold_distance, object_list=["Barrel"]
            )
            if obj_data[0]:
                distance = obj_data[1]
                barrel_counter += 1
            self.interface.Run()
        # 0.9144 = 3 feet to meters
        self.interface.car_sm.Stop_Trigger()
        self.interface.Run(distance - 0.9144)
