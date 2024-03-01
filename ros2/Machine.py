from statemachine import StateMachine, State

class CarMachine(StateMachine):
    "State Machine for the Cooper Union Intelligent Ground Vehicle"
    Lane_Following = State()
    Lane_Change = State()
    # C stop is Controlled, E stop is Emergency
    Cstop = State(initial=True)
    Estop = State(final=True)

    Resume = Cstop.to(Lane_Following)
    Object_Avoidance = Lane_Following.to(Lane_Change)
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


if __name__ == "__main__":
    test_machin = CarMachine()
    img_path = "visualizer.png"
    print(img_path)
    test_machin._graph().write_png(img_path)