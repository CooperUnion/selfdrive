import sys
import select
import rclpy
import time
from rclpy.node import Node
import Yolo.msg
from lane_change_sm import LaneChangeSM

#Lane following state
def LaneFollowing(self, msg, msg_out):
    self.sm.send(self.sm.Resume)
    msg_out.linear_x = 5.0

#Controlled stop state
def CStop(self, msg, msg_out):
    self.sm.send(self.sm.Stop_Trigger)
    msg_out.linear_x = 0.0

class Interface(Node):
    def __init__(self):
        super().__init__('Interface_Node')
        #Object Detection
        self.subscription1 = self.create_subscription(Yolo, '/Yolo', self.objectdetection_callback, 10)
        #Lane Change
        #self.subscription2 = self.create_subscription()
        #Lane Detection
        #self.subscription3 = self.create_subscription()
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sm = LaneChangeSM()
    
    #callback function for Object Detection
    def objectdetection_callbacK(self, msg):
        msg_out = Yolo()

        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0
 
        msg_out.angular.x = 0.0
        msg_out.angular.y = 0.0
        msg_out.angular.z = 0.0

        #Do we have to keep track of state in ROS or can I reference from the python library
        if (msg.name == "pedestrian" or msg.name == "barrel") and msg.dist<=0.914 and state == LaneFollowing:
            CStop(self, msg, msg_out)
        elif msg.name == "stop" and msg.dist<=0.3 and state == LaneFollowing:
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





        
        