from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseWithCovariance

from state_machines import car_sm


class Interface(Node):
    def __init__(self, function_test_label, lane_change, lane_follow):
        super().__init__(function_test_label)
        
        self.cmd_publisher = self.create_publisher(
            Float32MultiArray, '/cmd_stanley', 10)
        
        self.lane_change = lane_change
        self.lane_follow = lane_follow                                                                                         
        self.car_sm = car_sm.CarSM()

    def Lane_Follow_Action(self):
        cmd = Float32MultiArray()

        if self.lane_follow.empty_error == True:
            self.car_sm.Emergency_Trigger()  # ESTOP if we lose the lane lines in our vision
            self.stop()
            return
        [steer_cmd, vel_cmd] = self.lane_follow.follow_lane(1/20) # Each command takes 0.05s so this will take 30 seconds
                
        # Publish Lane Following command
        cmd.data = [
            steer_cmd,
            vel_cmd,
        ]
        # self.cmd_publisher.publish(cmd)


    def Lane_Change_Action(self):
        # TODO: add function to calculate relative position for path
        relative_x = 3 # replace this with subscriber data from obj detection
        relative_y = 0 # replace this with subscriber data from obj detection
        end_yaw = 0

        self.lane_change.create_path(relative_x, relative_y, end_yaw)

        # TODO: add error checking for lane change to estop transitions
        self.lane_change.follow_path()
    

    def Cstop_Action(self):
        cmd = Float32MultiArray()
        cmd.data = [
            0,
            0,
        ]
        self.cmd_publisher.publish(cmd)

    def Estop_Action(self,error="Entered Error State"):
        self.Cstop_Action()
        #TODO: Send Estop Message to DBW
        raise State_Machine_Failure(error)
    
    def Object_Detection(self,object_list=[],check_in_lane=False):
        pass


    def Run(self):
        function_dict = {"LC": self.Lane_Change_Action(),
        "LF": self.Lane_Follow_Action(),
        "Cstop": self.Cstop_Action(),
        "Estop": self.Estop_Action(),
        }
        function_dict[self.car_sm.state]
        
    
class State_Machine_Failure(Exception):
    pass

