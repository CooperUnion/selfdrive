'''
Test FV.3 STATIC Pedestrian Detection. Lane Changing

This test imitates a situation of a broken vehicle in a current lane with STATIC pedestrian standing in
FRONT of barrel(s) in the same lane as Ego vehicle. Ego vehicle must slow down, and safely change
into an adjacent lane
'''

import sys
import select
import rclpy
import time
from rclpy.node import Node
from statemachine import MegaStateMachine
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

# PROBABLY GONNA REFACTOR YOLO TO GET RID OF THE CUSTOM MESSAGE 
# BREAK into an array with floats and a string message 

# Changing between information: happens here in the state machine 
# How do we choose who is publishing? 
# So in each state make an instance of the class and call some function called follow path 

# Sub to the follow:
# Yolo (to get object name)
# Pose (to get object location)
# Stanley_CMD 

# Ros 2 messages 
lanechange_range = 6.096 # meters 
 
#Lane following state
def LaneFollowing(self, msg, msg_out):
    self.sm.send(self.sm.Resume)
    #will eventually not hardcode velocity
    msg_out.linear_x = 5.0
 
#Lane change state
def LaneChange(self, msg, msg_out):
    self.sm.send(self.sm.Obj_Avoidance)
    #need to know where velocity and angular velocity are coming from
 
#Controlled stop state
def CStop(self, msg, msg_out):
    self.sm.send(self.sm.Stop_Trigger)
    #will eventually not hardcode velocity
    msg_out.linear_x = 0.0
 
def Estop(self, msg, msg_out):
    self.sm.send(self.sm.Emergency_Trigger)
 
class Interface(Node):
    def init(self):
        super().init('Interface_Node')

        #wait for five seconds on initialization
        time.sleep(5)

        # We want the name of the object and how far it is 
        self.object_information_subscription = self.create_subscription(Yolo, '/Yolo', self.objectdetection_callback, 10)

        # Here we want the object's location realtive to the car 
        self.object_location_subscription = self.create_subscription()

        #Lane Detection
        self.stanley_controller_subscription = self.create_subscription()

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sm = MegaStateMachine()
    
    #callback function for Object Detection
    def objectdetection_callbacK(self, msg):
        msg_out = Twist()
 
        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0
 
        msg_out.angular.x = 0.0
        msg_out.angular.y = 0.0
        msg_out.angular.z = 0.0
 
        if msg.name == "mannequin"and msg.dist<=lanechange_range and self.sm.current_state.id == "LaneFollowing":
            LaneChange(self, msg, msg_out)
        elif msg.name == "barrel" and msg.dist<=0.3 and self.sm.current_state.id == "LaneFollowing":
            CStop(self, msg, msg_out)
 
        self.vel_publisher.publish(msg_out)
      
    #callback function for Lane Detection
    def LaneDetectionCallbacK(self, msg):
        msg_out = Yolo()
 
        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0
 
        msg_out.angular.x = 0.0
        msg_out.angular.y = 0.0
        msg_out.angular.z = 0.0
 
        self.vel_publisher.publish(msg_out)
 
    #callback function for Lane Change
    def LaneChangeCallbacK(self, msg):
        msg_out = Yolo()
 
        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0
 
        msg_out.angular.y = 0.0
        msg_out.angular.z = 0.0
 
        self.vel_publisher.publish(msg_out)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if name == 'main':
    main()


# Idea: I will have 3 subscribers. Whenever they recieve information they will update member variable inside of the class 
# We'll make a generic template for all state machines that sets this stuff up
# On call (which is different for each node) we will do checks and publish the right things accordingly 

# When the path finishes we change the state 