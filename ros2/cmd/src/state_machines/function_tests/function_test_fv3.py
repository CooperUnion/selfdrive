from State_Machine_Executor import main
from State_Machine_Interface import Interface

distance_threshold = 3
class Function_Test_FV3():
    def __init__(self, interface):
        self.interface = interface

    def function_test(self):
        barrel_detected = False
        person_detected = False
        person_data = []
        barrel_data = []
        self.interface.car_SM.Resume()
        while not person_detected:
            self.interface.Run()
            person_data = self.interface.Object_Detection(3.5, cared_objects=["Person"], check_in_lane=True))
            if(person_data[0]):
                person_detected = True
                self.interface.carSM.Obj_Avoidance()
                multiplier = 1 if self.interface.current_lane() else -1
                self.interface.Run(person_data[1],person_data[2] + multiplier * 3 , 0.0)

        self.interface.car_SM.Return_To_Follow()
        self.interface.Run()

        while not barrel_detected:
            barrel_data = self.interface.Object_Detection(0.9, cared_objects=["Barrel"],))
            self.interface.Run()

            if(barrel_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run((barrel_data[1]+0.4) - 1.524)

