from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String
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

        self.object_position_x = None
        self.object_position_y = None
        self.object_distance = None
        self.object_name = None
        self.obj_list_index = 0
        self.prev_object_history_length = 0 
        self.object_history = []

        # We want the name of the object and how far it is (distance)
        self.object_name_subscription = self.create_subscription(String, '/obj_name', self.object_name_callback, 10)
        self.object_distance_subscription = self.create_subscription(Float32MultiArray, '/obj_pointcloud', self.object_distance_callback, 10)

        # Here we want the object's location realtive to the origin (starting point)
        self.object_location_subscription = self.create_subscription(PoseWithCovariance,'/obj_location',self.object_location_callback,10)

    def object_name_callback(self,msg):
        self.object_name = msg.data 

    def object_location_callback(self,msg):
        self.object_position_x = msg.pose.position.x
        self.object_position_y = msg.pose.position.y
    
    def object_distance_callback(self,msg):
        self.object_distance = msg.data[2]

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
    
    def Unique_Object(self):
        # Every time this function gets called. We need to see whether or not we want to log an object as unique or not 
        # Maybe we can check the length of this unique list compared to its previous lenght. It grows by 1 then we have seen a unqie object

        # So what has to be done to recongnize a seen object is take the:
        # x odom,y odom AND x car, y car 
        # In theory since the object never moves the sum of the x and y values should always stay the same 
        # So I guess we can just check for major changes in object_location_subscription

        # if object pos x and y change by some large factor log it. Think we can also use the length to string log it like barrel 1. 
        # Can change the equal to to see if the string has name of object in it 

        # need to define this condition 
        
        if(True):
            self.prev_object_history_length = len(self.object_history)
            object_destription = (self.object_name,self.object_position_x,self.object_position_y)
            self.object_history.append(object_destription)
    
    def Object_Detection(self,distance_threshold,object_list=[],check_in_lane=False):

        # if the name is what we expected
        # is it in our lane (add after)
        # is it close enough 
        # is it a distinct object ** 

        self.Unique_Object()

        if((object_list[self.obj_list_index] == self.object_name) and (self.object_distance <= distance_threshold) and (len(self.object_history) > self.prev_object_history_length)): 
            return True 
        else:  
            return False 


    def Run(self):
        function_dict = {"LC": self.Lane_Change_Action(),
        "LF": self.Lane_Follow_Action(),
        "Cstop": self.Cstop_Action(),
        "Estop": self.Estop_Action(),
        }
        function_dict[self.car_sm.state]
        
    
class State_Machine_Failure(Exception):
    pass

