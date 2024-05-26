from statemachine import StateMachine, State

class MegaStateMachine(StateMachine):
    "State Machine for the Cooper Union Intelligent Ground Vehicle"
    # C stop is Controlled, E stop is Emergency
    Cstop = State("CS",initial=True)
    Lane_Following = State("LF")
    Lane_Change = State("LC")
    Estop = State("ES",final=True)

    Resume = Cstop.to(Lane_Following)
    Obj_Avoidance = Lane_Following.to(Lane_Change)
    Stop_Trigger = Lane_Following.to(Cstop) | Lane_Change.to(Cstop)
    Emergency_Trigger =(
                        Lane_Following.to(Estop)
                        | Lane_Change.to(Estop) 
                        | Cstop.to(Estop)
                        )
    Return_To_Follow = Lane_Change.to(Lane_Following)

#The Lane Follow State 
    @Lane_Following.enter
    def Follow(self):
        print("Following Lanes")

    @Cstop.enter
    def on_CStop(self):
        print("Car is Stopped temporarily")

    @Estop.enter
    def on_EStop(self):
        print("UH OH")
    
    @Lane_Change.enter
    def on_Lane_change(self):
        print("changing lanes")


if __name__ == "__main__":
    test_machine = MegaStateMachine()
    img_path = "diagram.png"
    test_machine._graph().write_png(img_path)
