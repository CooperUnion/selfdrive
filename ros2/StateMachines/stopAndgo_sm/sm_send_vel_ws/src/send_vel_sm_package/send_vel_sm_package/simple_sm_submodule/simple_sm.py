from statemachine import StateMachine, State

class firstAlgoSM(StateMachine):
    def __init__(self):
        super().__init__()
        self.vehicle_vel = 0.0
 
    # Define states
    lane_line_state = State(name = "Lane Line Following")
    vehicle_stop_state = State(name = "Vehicle Stop",initial=True)
 
    # Events/Transition
    follow_lane_lines = vehicle_stop_state.to(lane_line_state)
    stop_vehicle = lane_line_state.to(vehicle_stop_state)
 
    # actions that take place on entry to the state
    @lane_line_state.enter
    def lane_line_state_on(self):
        print("FUCK")
        self.vehicle_vel = .3
        
    @vehicle_stop_state.enter
    def vehicle_stop_state_on(self):
        print("RUH ROH RAGGY")
        self.vehicle_vel = 0.0
        