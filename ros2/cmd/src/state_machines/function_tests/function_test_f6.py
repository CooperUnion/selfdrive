#Executor
from State_Machine_Executor import main
from State_Machine_Interface import Interface


# Test FV.1 (Go straight and stop five feet away from detected person)

class Function_Test_FV_1:
    # Get data from 5 meters away
    threshold_distance = 5  # meters

    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        person_detected = False
        distance = 0
        self.interface.car_sm.Resume()
        while not person_detected:
            obj_data = self.interface.Object_Detection(
                self.threshold_distance, object_list=["Person"]
            )
            if obj_data[0]:
                distance = obj_data[1]
                person_detected = True
            self.interface.Run()
        # 1.524 = 5 feet to meters
        self.interface.car_sm.Stop_Trigger()
        self.interface.Run(distance - 1.524)


if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_FV_1)