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
from std_msgs.msg import String
from ...cmd.src.odom_sub import OdomSubscriber
from ...cmd.src.lane_change import LaneChange
from ...cmd.src.lane_follow import LaneFollow
import math
from threading import Thread
 
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
        self.object_distance = None
        self.lane_change_node = lane_change_node
        self.lane_follow_node = lane_follow_node
        
        # Empty List
        self.object_history = []

        #wait for five seconds on initialization
        time.sleep(5)

        # We want the name of the object and how far it is (distance)
        self.object_name_subscription = self.create_subscription(String, '/obj_name', self.object_detection_callback, 10)
        self.object_distance_subscription = self.create_subscription(Float32MultiArray, '/obj_pointcloud', self.object_distance_callback, 10)

        # Here we want the object's location realtive to the origin (starting point)
        self.object_location_subscription = self.create_subscription(PoseWithCovariance,'/obj_location',self.object_location_callback,10)
        self.sm = MegaStateMachine()

    def object_location_callback(self,msg):
        self.object_position_x = msg.pose.position.x
        self.object_position_y = msg.pose.position.y
    
    def object_distance_callback(self,msg):
        self.object_distance = msg.data[2]
    
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
        elif msg.data == "person" and msg.data[2]<=lanechange_range and self.sm.current_state.id == "LF":
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

        elif msg.name == "barrel" and msg.data[2]<=0.3 and self.sm.current_state.id == "LF":
            c_stop_state(self)
            object_destription = (msg.name,self.object_position_x,self.object_position_y)
            self.object_history.append(object_destription)
            # Stopping Logic (are we using controlled stop thing nathan made?)

def main(args=None):

    ''' 
    Not to sure about Node Lifecycle Management

    Does adding lane change and lane follow to the same executor mean they can run at the same time? 

    Should we put the executor inside of the interface? 

    '''

    rclpy.init(args=args) 
    odom_sub = OdomSubscriber() 
    lane_change = LaneChange(odom_sub) 
    lane_follow = LaneFollow() 
    interface = Interface(lane_change,lane_follow)
    executor = rclpy.executors.MultiThreadedExecutor() 
    executor.add_node(lane_change) 
    executor.add_node(lane_change.odom_sub) 
    executor.add_node(lane_follow) 
    executor.add_node(interface)


    # xxx: TODO: Need to add proper initialization sequence, for now spinning the executor to make sure transform topics are all proper before # calling create_path function 
    for i in range( 100 ): # Do this to make sure transform broadcaster is properly initalized 
        executor.spin_once()

    executor_thread = Thread(target=executor.spin, daemon=True) 
    executor_thread.start()


if __name__ == 'main':
    main()


# Idea: I will have 3 subscribers. Whenever they recieve information they will update member variable inside of the class 
# We'll make a generic template for all state machines that sets this stuff up
# On call (which is different for each node) we will do checks and publish the right things accordingly 

# When the path finishes we change the state 