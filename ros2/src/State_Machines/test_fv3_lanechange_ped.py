#test FV.3 detecting a pedestrian and switching lanes then detecting a barrel and stopping

import sys
import select
import rclpy
import time
from rclpy.node import Node
import Yolo.msg
from sm_library import MegaStateMachine

lanechange_range = 6.096

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
    def __init__(self):
        super().__init__('Interface_Node')
        #wait for five seconds on initialization
        time.sleep(5)
        #Object Detection
        self.subscription1 = self.create_subscription(Yolo, '/Yolo', self.objectdetection_callback, 10)
        #Lane Change
        #self.subscription2 = self.create_subscription()
        #Lane Detection
        #self.subscription3 = self.create_subscription()
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sm = MegaStateMachine()
    
    #callback function for Object Detection
    def objectdetection_callbacK(self, msg):
        msg_out = Yolo()

        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0
 
        msg_out.angular.x = 0.0
        msg_out.angular.y = 0.0
        msg_out.angular.z = 0.0

        #double check what the message object names are
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

if __name__ == 'main':
    main()





        
        