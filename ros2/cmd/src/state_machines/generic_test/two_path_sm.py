'''
Test F0 - Follow two consective Paths
1. Test Goal
This test is intended to evaluate if a vehicle is able to follow one clothoid path 
then immeadeatly transition into following another
'''

from State_Machine_Executor import main
from State_Machine_Interface import Interface
import math 

class Function_Test_F0_1():
    def __init__(self, interface):
        self.interface = interface


    # State transistion logic
    def function_test(self):
        
        # Go foward and make a left turn 
        self.interface.car_SM.Turn_Trigger()
        self.interface.Run([3,0,0])
        self.interface.Run([4.5,7.6,math.pi/2])

if __name__ == "__main__":
    main("Function_Test_Q3",Function_Test_F0_1)
