# Test Q.5 Left Turn
from State_Machine_Executor import main
from State_Machine_Interface import Interface

import math


class Function_Test_Q5:
    def __init__(self, interface: Interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        distance_threshold = 5
        white_line_detected = False
        barrel_detected = False
        white_line_data = []
        barrel_data = []

        self.interface.car_sm.Resume()
        while not white_line_detected:
            white_line_data = self.interface.Object_Detection(
                distance_threshold, object_list=["White Line"]
            )
            self.interface.Run()

            if white_line_data[0]:
                white_line_detected = True
                self.interface.car_sm.Stop_Trigger()
                self.interface.Run(white_line_data[1])

        self.interface.car_sm.Turn_Trigger()
        # y might be negative or positive
        self.interface.Run([4.5, 7.6, math.pi / 2])
        self.interface.car_sm.Return_To_Follow()
        # Turn
        # x: 4,5 m
        # y: 7.6 m
        # t: math.pi / 2
        # distance_threshold = 5
        while not barrel_detected:
            barrel_data = self.interface.Object_Detection(
                distance_threshold, object_list=["Barrel"]
            )
            self.interface.Run()

            if barrel_data[0]:
                barrel_detected = True
                self.interface.car_sm.Stop_Trigger()
            self.interface.Run((barrel_data[1] + 0.4) - 1.524)

        # self.interface.car_sm.Stop_Trigger()
        # self.interface.Run()


if __name__ == "__main__":
    main("Function_Test_Q3", Function_Test_Q5)
