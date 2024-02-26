from statemachine import StateMachine, State

class Car(StateMachine):
    # State Machine for the Cooper Union Intelligent Ground Vehicle
    Lane_Following = State("LF")
    Lane_Change = State("LC")
    # C stop is Controlled, E stop is Emergency
    Cstop = State("CS",initial=True)
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
    def Follow():
        print("Following Lanes")


    @Cstop.enter
    def on_CStop():
        print("Car is Stopped temporarily")

    @Estop.enter
    def on_EStop():
        print("UH OH")

if "__name__" == "__main__":
    test_machin = Car()
    img_path = "visualizer.png"
    test_machin._graph().write_png(img_path)