import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from car_sm import CarSM

import time

class FunctionTest(Node):
    def __init__(self, lane_change, lane_follow):
        super().__init__('function_test_v4_node')
        
        self.cmd_publisher = self.create_publisher(
            Float32MultiArray, '/cmd_stanley', 10
        )

        self.lane_change = lane_change
        self.lane_follow = lane_follow

        self.car_sm = CarSM()
        self.img_path = "./diagrams/live_SM.png"
        self.car_sm._graph().write_png(self.img_path)


    def function_test(self):
        time.sleep(1) # Stay in CStop state for 1 second on startup

        # Transition to Lane Following state to begin test
        self.car_sm.send(self.car_sm.Resume)  
        self.car_sm._graph().write_png(self.img_path)
        # TODO: Replace time based transistion with data from obstacle detection subscriber
        for i in range(600):
            self.follow_lane_lines()

        # Transition to Lane Change to avoid obstacle
        self.car_sm.send(self.car_sm.Obj_Avoidance)  
        self.car_sm._graph().write_png(self.img_path)
        self.change_lanes()

        # Transition back to Lane Following State on success of lane change state
        self.car_sm.send(self.car_sm.Return_To_Follow)  
        self.car_sm._graph().write_png(self.img_path)
        # TODO: Replace time based transistion with data from obstacle detection subscriber
        for i in range(600):
            self.follow_lane_lines()

        # Transition to ESTOP state to signal end of test after detecting obstacle
        self.car_sm.send(self.car_sm.Emergency_Trigger)  
        self.car_sm._graph().write_png(self.img_path)
        self.stop()


    def follow_lane_lines(self):
        cmd = Float32MultiArray()

        if self.lane_follow.empty_error == True:
            self.car_sm.send(self.car_sm.Emergency_Trigger)  # ESTOP if we lose the lane lines in our vision
            self.car_sm._graph().write_png(self.img_path)
            self.stop()
            return
            
        [steer_cmd, vel_cmd] = self.lane_follow.follow_lane() # Each command takes 0.05s so this will take 30 seconds
                
        # Publish Lane Following command
        cmd.data = [
            steer_cmd,
            vel_cmd,
        ]
        self.cmd_publisher.publish(cmd)


    def change_lanes(self):
        # TODO: add function to calculate relative position for path
        relative_x = 10
        relative_y = 10
        end_yaw = 0

        self.lane_change.create_path(relative_x, relative_y, end_yaw)

        # TODO: add error checking for lane change to estop transitions
        self.lane_change.follow_path()
    

    def stop(self):
        cmd = Float32MultiArray()

        # Publish ESTOP command
        cmd.data = [
            0,
            0,
        ]

        self.cmd_publisher.publish(cmd)
