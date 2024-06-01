#Test Q.6 Right Turn
import math 
distance_threshold = 5
class Function_Test_Q5():
    def __init__(self, interface):
        self.interface = interface

    # State transistion logic
    def function_test(self):
        white_line_detected = False
        barrel_detected = False 
        white_line_data = []
        
        self.interface.car_SM.Resume() 
        while not white_line_detected: 
            white_line_data = self.interface.Object_Detection(distance_threshold, object_lists=["White Line"])   
            self.interface.Run()        
            
            if(white_line_data[0]):
                white_line_detected = True 
                self.interface.car_SM.Stop_Trigger()
                self.interface.Run(white_line_data[1])
            
        self.interface.car_SM.Turn_Trigger()
        # y might be negative or positive 
        self.interface.Run([1,5,1.5,math.pi/2])

        while not barrel_detected:
            
            barrel_data = self.interface.Object_Detection(distance_threshold, object_lists=["Barrel"])
            self.interface.Run()  

            if(white_line_data[0]):
                barrel_detected = True
                self.interface.car_SM.Stop_Trigger()
            self.interface.Run((barrel_data[1]+0.4) - 1.524)
        # self.interface.car_SM.Stop_Trigger()
        # self.interface.Run()


        # 1.542 5ft 

