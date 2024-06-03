# Executor
from State_Machine_Executor import main
from State_Machine_Interface import Interface


# Test Q.3 Lane Keeping (Go straight and stop at barrel)


class Function_Test_Q3:
    # Get data from 5 meters away
    threshold_distance = 5  # meters

    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        OFFSET = 1.65 # Meter offset from barrel before we stop.
        barrel_counter = 0
        distance = 0
        self.interface.car_sm.Resume()
        while barrel_counter < 1:
            obj_data = self.interface.Object_Detection(
                self.threshold_distance, object_list=["Barrel"]
            )
            if obj_data[0]:
                distance = obj_data[1]
                barrel_counter += 1
                self.interface.car_sm.Obj_Avoidance()
                args = [distance-OFFSET,obj_data[2],0]
                self.interface.run(args)
        # 0.9144 = 3 feet to meters
        self.interface.car_sm.Stop_Trigger()
        self.interface.Run(distance - OFFSET)


if __name__ == "__main__":
    main("Function_Test_Q3", Function_Test_Q3)
