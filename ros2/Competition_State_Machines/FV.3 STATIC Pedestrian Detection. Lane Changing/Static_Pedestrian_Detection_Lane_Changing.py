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
from geometry_msgs.msg import Pose, PoseWithCovariance
from std_msgs.msg import Float32MultiArray
from ...cmd.src.odom_sub import OdomSubscriber
from ...cmd.src.lane_change import LaneChange
from ...cmd.src.lane_follow import LaneFollow
import math
 

# PROBABLY GONNA REFACTOR YOLO TO GET RID OF THE CUSTOM MESSAGE 
# BREAK into an array with floats and a string message 

# Changing between information: happens here in the state machine 
# How do we choose who is publishing? 
# So in each state make an instance of the class and call some function called follow path 

# Sub to the follow:
# Yolo (to get object name)
# Pose (to get object location)
# Stanley_CMD 

lanechange_range = 6.096 # meters 

#Lane following state
def lane_following_state(self):
    self.sm.send(self.sm.Resume)

#Lane change state
def lane_change_state(self):
    self.sm.send(self.sm.Obj_Avoidance)
    #need to know where velocity and angular velocity are coming from
 
#Controlled stop state
def c_stop_state(self, msg):
    self.sm.send(self.sm.Stop_Trigger)

# Emergency stop state
def e_stop_state(self, msg, msg_out):
    self.sm.send(self.sm.Emergency_Trigger)
 
class Interface(Node):
    def init(self,lane_change_node,lane_follow_node):
        super().init('Interface_Node')

        self.object_position_x = None
        self.object_position_y = None
        self.lane_change_node = lane_change_node
        self.lane_follow_node = lane_follow_node

        # Empty List
        self.object_history = []

        #wait for five seconds on initialization
        time.sleep(5)

        # We want the name of the object and how far it is 
        self.object_information_subscription = self.create_subscription(Yolo, '/Yolo', self.object_detection_callback, 10)

        # Here we want the object's location realtive to the origin (starting point)
        self.object_location_subscription = self.create_subscription(PoseWithCovariance,'/objLocation',self.object_location_callback,10)

        self.sm = MegaStateMachine()

    def object_location_callback(self,msg):
        self.object_position_x = msg.pose.position.x
        self.object_position_y = msg.pose.position.y

    #callback function for Object Detection
    def object_detection_callback(self, msg):

        '''
        In the controlled stop state. Lane Detection and Object Detection both check that there respective
        sensor (ZED and webcams) are on. We are not sure if we see the starting barrel (that's something)
        we'll see once we get to comp. So for now we're gonna check that we are in our initial state. If
        we are then switch to lane following 
        '''
        if self.sm.current_state.id == "CS":
            lane_following_state(self)
            self.lane_follow_node.lane_follow() 
        
        # TODO: Add check that makes sure the object is in our lane 
        elif msg.name == "person" and msg.dist<=lanechange_range and self.sm.current_state.id == "LF":
            lane_change_state(self)
            
            '''
            Determining relative x,y, and yaw. 

            Not sure what these values are suppose to be as it depends on the challenge. 

            For this specific challenge we want to end this path before the final barrel. 
            We want to end in the center of the lane we swtiched to (ideally)

            For now I am going to put 1 meter before the location of the pedestrian when we first see it 
            in the realtive_x
            
            For relative_y I will put 1.524 m (5ft) to the left (assuming the pedestrian is in the right lane)
            from when we first see it. The reason for this being assuming the object is in the center of the
            lane and the lanes are 10ft wide then shifting by 5ft will set the end point to be in the center
            of the opposite lane
            '''
            self.lane_change_node.create_path(self.object_position_x - 1, self.object_position_y - 5, math.pi / 2 )
            self.lane_change_node.follow_path()
            self.lane_change_node.odom_sub.destroy_node()
            self.lane_change_node.destroy_node()

            object_destription = (msg.name,self.object_position_x,self.object_position_y)
            self.object_history.append(object_destription)

        # Do we want to publish some sort of indication that we see the left and right line
        elif self.sm.current_state.id == "LC":
            lane_following_state(self)
            self.lane_follow_node.lane_follow()

        elif msg.name == "barrel" and msg.dist<=0.3 and self.sm.current_state.id == "LF":
            c_stop_state(self)
            object_destription = (msg.name,self.object_position_x,self.object_position_y)
            self.object_history.append(object_destription)
            # Stopping Logic??



def main(args=None):

    '''
    Note: This is not what the final main will look like.
    I think we might have to use a multi-threaded executor
    
    '''

    # Initialize odom Subscriber,Lane Change and Lane Follow Nodes 
    odom_sub = OdomSubscriber()
    lane_change = LaneChange(odom_sub)
    lane_follow = LaneFollow()

    rclpy.init(args=args)
    node = Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == 'main':
    main()


# Idea: I will have 3 subscribers. Whenever they recieve information they will update member variable inside of the class 
# We'll make a generic template for all state machines that sets this stuff up
# On call (which is different for each node) we will do checks and publish the right things accordingly 

# When the path finishes we change the state 