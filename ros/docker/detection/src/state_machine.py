class statemachine():
    current_table = [][]

##If right_lane_detection doesn't catch the lane for 30 frames (1/2 a second), enter right turn state
## Once right turn state exits, return to right_lane state

    ##TODO
    ## Get Distance Travelled
    ## Using this, determine angle using Jeanettes Table
    ## Send steering angle to Jacussy
    ## try catch array index out of bounds, re enter lane_detection



    ## Get Distance Travelled
    ## Using this, determine angle using Jeanettes Table
    ## Send steering angle to Jacussy
    ## try catch array index out of bounds, re enter lane_detection


# def left_lane_state():
#     ##Same principle on my end, but left lane table


# def lane_change():
#     ##Self Explanatory
#     ##Also Steering Table, two tables for right to left and left to right
#     ##Triggered by Barrel Detection

def angle_determ(table,distance):
    if table is not None:
        return 0
    else:
        return None

    ##computes steering angles from the tables of dist & steering angle, using current distance

# def parking_in():
#     ##Use Table, compute steering angle based on distance 
#     # don't EXIT THIS STATE, WHEN FINISHED VEL = 0

# def parallel_park():

# def parking_out():

def main(distance,state):
    match state:
        case 0: ##Right Turn
            table = [][]
            return angle_determ(table,distance)
        case 1: ##Right Turn
            table = [][]
            return angle_determ(table,distance)
        case 2: ## Right Lane
            return mezhibungus(image,True)
        case 3: ## Left Lane
            return mezhibungus(image,False)
        



        case _:
            table = None


#True Means Right, False means Left