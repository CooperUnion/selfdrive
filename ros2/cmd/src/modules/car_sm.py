from statemachine import StateMachine, State


class CarSM(StateMachine):
    "State Machine for the Cooper Union Intelligent Ground Vehicle"

    IMG_PATH = "/home/autonomylab/selfdrive/live_SM.png"

    # C stop is Controlled, E stop is Emergency
    Cstop = State("CS", initial=True)
    Lane_Following = State("LF")
    Lane_Change = State("LC")
    Estop = State("ES", final=True)

    Resume = Cstop.to(Lane_Following)
    Obj_Avoidance = Lane_Following.to(Lane_Change)
    Stop_Trigger = Lane_Following.to(Cstop) | Lane_Change.to(Cstop)
    Emergency_Trigger = (
        Lane_Following.to(Estop) | Lane_Change.to(Estop) | Cstop.to(Estop)
    )
    Turn_Trigger = Cstop.to(Lane_Change)
    Return_To_Follow = Lane_Change.to(Lane_Following)


    # The Lane Follow State
    @Lane_Following.enter
    def Follow(self):
        self._graph().write_png(CarSM.IMG_PATH)
        print("Following lane lines")

    @Cstop.enter
    def on_CStop(self):
        self._graph().write_png(CarSM.IMG_PATH)
        print("Executing a controlled stop")

    @Estop.enter
    def on_EStop(self):
        self._graph().write_png(CarSM.IMG_PATH)
        print("Executing an emergency stop")

    @Lane_Change.enter
    def on_Lane_change(self):
        self._graph().write_png(CarSM.IMG_PATH)
        print("Changing lanes")


if __name__ == "__main__":
    car_sm = CarSM()
    car_sm.Resume()
