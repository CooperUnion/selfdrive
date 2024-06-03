from datetime import datetime
import math
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseWithCovariance
from car_sm import CarSM
from lane_behaviors.lane_follower import LaneFollower
from lane_behaviors.lane_change import LaneChange


class Interface(Node):
    LANE_WIDTH = 3.048  # METERS
    MIN_SAFE_BRAKING_DIST = 2.2
    # Given that we are moving at 5 mph (2.35 m/s)
    # We should start braking at least x meters before our end goal, given the safe braking decel Jeanette has specified.

    def __init__(
        self,
        function_test_label,
        lane_change: LaneChange,
        lane_follow: LaneFollower,
    ):
        super().__init__(function_test_label)

        self.cmd_publisher = self.create_publisher(
            Float32MultiArray, '/cmd_stanley', 10
        )

        self.lane_change = lane_change
        self.lane_follow = lane_follow
        self.car_sm = CarSM()

        self.object_global_position_x = None
        self.object_global_position_y = None

        self.object_local_position_x = None
        self.object_local_position_y = None

        self.object_distance = None
        self.object_name = None
        self.obj_list_index = 0
        self.prev_object_history_length = 0
        self.object_history = []

        # We want the name of the object and how far it is (distance)
        self.object_name_subscription = self.create_subscription(
            String, '/obj_name', self.object_name_callback, 10
        )
        self.object_distance_subscription = self.create_subscription(
            Float32MultiArray,
            '/obj_pointcloud',
            self.object_distance_callback,
            10,
        )

        # Here we want the object's location realtive to the origin (starting point)
        self.object_location_subscription = self.create_subscription(
            PoseWithCovariance,
            '/obj_location',
            self.object_location_callback,
            10,
        )

    def object_name_callback(self, msg):
        self.object_name = msg.data

    def object_location_callback(self, msg):
        self.object_position_x = msg.pose.position.x
        self.object_position_y = msg.pose.position.y
        # If we're within 1 meter, we should probably freak out

    def object_distance_callback(self, msg):
        self.object_local_position_x = msg.data[0]
        self.object_local_position_y = msg.data[1]
        self.object_distance = msg.data[2]
        if (
            self.object_local_position_x < 1
            and math.abs(self.object_local_position_y) < 1
        ):
            self.Estop_Action(error="Too Close for Comfort With an Obstacle")

    # Lane Following Action: takes no arguments: We follow the lanes, and if an empty_error is thrown, we panic!
    def Lane_Follow_Action(self, args=None):
        cmd = Float32MultiArray()

        if self.lane_follow.empty_error:
            self.car_sm.Emergency_Trigger()
            self.Run()  # ESTOP if we lose the lane lines in our vision

        [steer_cmd, vel_cmd] = self.lane_follow.follow_lane(1 / 40)
        # Publish Lane Following command
        cmd.data = [
            steer_cmd,
            vel_cmd,
        ]
        self.cmd_publisher.publish(cmd)

    # self.cmd_publisher.publish(cmd)

    # Lane Change Action: args[0] is our relative x coordinate, and args[1] is the relative y coordinate. args[2] is the end yaw.
    # (yaw relative to current heading)
    def Lane_Change_Action(self, args=None):
        print("In Lane Change")
        if args is None:
            self.Estop_Action(error="No Lane Data Provided", args=[True])
        else:
            # TODO: add function to calculate relative position for path
            relative_x = args[
                0
            ]  # replace this with subscriber data from obj detection
            relative_y = args[
                1
            ]  # replace this with subscriber data from obj detection
            end_yaw = args[
                2  # We should never be sending an end yaw of more than zero
            ]
            self.lane_change.create_path(relative_x, relative_y, end_yaw)
            self.lane_change.follow_path()

    # Cstop Action: When args[0], we are computing slope based on distance: we don't exceed 1.32 in this action
    # Those calculations should be done before_hand, in the state machines.
    def Cstop_Action(self, args=None):
        # We need to parametrize this, slope in m/s^2
        slope = 1.32
        # d = (vf^2-vi^2)/2a
        # (vf is zero)
        # vi^2 / d = 2a
        current_speed = self.lane_follow.odom_sub.vel
        if args is not None:
            dist = args[0]
            attempted_slope = (current_speed**2) / (2 * dist)
            # Guaranteed that slope is not greater than 1.32
            slope = min(attempted_slope, slope)

        cmd = Float32MultiArray()
        initial = datetime.now()
        # Catching any precision errors & I'm not entirely sure how odom velocity is calculated
        # In other words, lazy programming. TODO: Determine appropriate bounds:)
        while current_speed > 0.05:
            current_speed = self.lane_follow.odom_sub.vel
            current_time = datetime.now()
            difference = (initial - current_time).total_seconds()
            cmd.data = [
                0.0,
                max(current_speed - (slope * difference), 0),
            ]  # Allows us to keep slope @ set time
            initial = current_time
            self.cmd_publisher.publish(cmd)

    # Estop Action: When things break. args[0] is "soft estop", no args or args[0] false is a hard estop.
    # hard estop is HARD
    def Estop_Action(self, error="Entered Error State", args=None):
        print("ESTOP REACHED")
        # slope = 2.0
        slope = 1.32
        # Args[0] is a "soft" estop: We aren't in a physical emergency, but something has gone wrong.
        if args is not None and args[0]:
            slope = 1.32
        current_speed = self.lane_follow.odom_sub.vel
        cmd = Float32MultiArray()
        initial = datetime.now()
        # Catching any precision errors & I'm not entirely sure how odom velocity is calculated
        # In other words, lazy programming. TODO: Determine appropriate bounds:)
        while current_speed > 0.05:
            current_speed = self.lane_follow.odom_sub.vel
            current_time = datetime.now()
            difference = (initial - current_time).total_seconds()
            cmd.data = [
                0.0,
                current_speed - (slope * difference),
            ]  # Allows us to keep slope @ set time
            initial = current_time
            self.cmd_publisher.publish(cmd)
        print(error)
        raise State_Machine_Failure(error)

    def Unique_Object(self):
        # Every time this function gets called. We need to see whether or not we want to log an object as unique or not .
        # Maybe we can check the length of this unique list compared to its previous length. It grows by 1 then we have seen a unique object

        # So what has to be done to recongnize a seen object is take the:
        # x odom,y odom AND x car, y car
        # In theory since the object never moves the sum of the x and y values should always stay the same
        # So I guess we can just check for major changes in object_location_subscription

        # if object pos x and y change by some large factor log it. Think we can also use the length to string log it like barrel 1.
        # Can change the equal to to see if the string has name of object in it

        # need to define this condition
        if True:
            self.prev_object_history_length = len(self.object_history)
            object_destription = (
                self.object_name,
                self.object_position_x,
                self.object_position_y,
            )
            self.object_history.append(object_destription)

    # This returns true - position_x - position_y - distance if an object in the object list is detected.
    # If check_in_lane is called, the experimental lane checker is run.

    # wrapper to tell you what lane we are currently in.
    def current_lane(self):
        return self.lane_follow._Left_Lane

    def Object_Detection(
        self, distance_threshold, object_list=[], check_in_lane=False
    ):
        # if the name is what we expected
        # is it in our lane (add after)
        # is it close enough
        # is it a distinct object **
        # self.Unique_Object()

        if (
            (object_list[self.obj_list_index] == self.object_name)
            and (self.object_distance <= distance_threshold)
            and (len(self.object_history) > self.prev_object_history_length)
        ):
            reference_x = self.object.local_position_x
            reference_y = self.object.local_position_y
            # TODO: Test the lane checker.
            if not check_in_lane:
                return (
                    True,
                    self.object_local_position_x,
                    self.object_global_position_y,
                    self.object_distance,
                )
            else:
                crosstrack = self.lane_follow.cross_track_error
                projected_left_line = (
                    self.lane_follow.left_slope * reference_x
                    + crosstrack
                    - (Interface.LANE_WIDTH / 2)
                )
                projected_right_line = (
                    self.lane_follow.right_slope * reference_x
                    + crosstrack
                    + (Interface.LANE_WIDTH / 2)
                )
                # Slope is up/down projections,
                # If we sit in the middle, the crosstrack is zero. The lanes are approximately 10 meters wide
                # Hence why I'm checking the crosstrack against 5 meters
                # If the object sits in the lane, it's y value will be greater than projected left line and greater than projected_right_line
                if (
                    projected_left_line < reference_y
                    and projected_right_line > reference_y
                ):
                    return (
                        True,
                        self.object_local_position_x,
                        self.object_local_position_y,
                        self.object_distance,
                    )
        return False, -1, -1, -1

    # Executes the current state
    # This might seem arbitrary, but the idea is that the state machine is what's handling things: We can prevent illegal states
    # and other failures by using this model.
    # Eventually, we should pipe this interface directly into the state machine, and have the function tests loop over the state:
    # This would theoretically give us more modularity
    def Run(self, args=None):
        function_dict = {
            "Lane_Change": self.Lane_Change_Action,
            "Lane_Following": self.Lane_Follow_Action,
            "Cstop": self.Cstop_Action,
            "Estop": self.Estop_Action,
        }
        function_dict[self.car_sm.current_state.id](args=args)


class State_Machine_Failure(Exception):
    pass


class No_Lane_Change_Data(Exception):
    pass
