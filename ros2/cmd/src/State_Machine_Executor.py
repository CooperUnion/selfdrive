import rclpy
from threading import Thread

from state_machines.function_tests import function_test_v4 as FunctionTest
# This is one of two lines that needs to be changed every time
from State_Machine_Interface import Interface as SM_Interface
from lane_behaviors.odom_sub import OdomSubscriber
from lane_behaviors.lane_change import LaneChange
from lane_behaviors.lane_follower import LaneFollower



def main(args=None):

    try:
        rclpy.init(args=args)

        max_dist_to_goal = 0.5
        max_dist_to_path = 1.5

        odom_sub = OdomSubscriber()
        lane_change = LaneChange(odom_sub, max_dist_to_goal, max_dist_to_path)
        lane_follow = LaneFollower(odom_sub)

        # Change this to specify which function test to run
        Interface = SM_Interface("Function Test V4", lane_change, lane_follow)
        function_test = FunctionTest(Interface)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(Interface)
        executor.add_node(lane_change)
        executor.add_node(lane_follow)
        executor.add_node(odom_sub)

        # xxx: TODO: Need to add proper initialization sequence, for now spinning the executor to make sure transform topics are all proper before
        # calling create_path function
        for i in range(100):  # Do this to make sure transform broadcaster is properly initalized
            executor.spin_once()

        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        function_test.function_test()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Starting shutdown")

        function_test.interface.EStop()
        function_test.destroy_node()
        lane_change.destroy_node()
        lane_follow.destroy_node()
        odom_sub.destroy_node()

        rclpy.try_shutdown()
        print("Finished shutdown")


if __name__ == '__main__':
    main()
