import rclpy
from rclpy.node import Node
from threading import Thread

from std_msgs.msg import Float32MultiArray

from lane_behaviors.odom_sub import OdomSubscriber
from lane_behaviors.lane_change import LaneChange
from lane_behaviors.lane_follower import LaneFollower
from state_machine.function_test_v4 import FunctionTest


def main(args=None):
    rclpy.init(args=args)

    max_dist_to_goal = 0.5
    max_dist_to_path = 1.5

    odom_sub = OdomSubscriber()
    lane_change = LaneChange(odom_sub, max_dist_to_goal, max_dist_to_path)
    lane_follow = LaneFollower(odom_sub)
    function_test = FunctionTest(lane_change, lane_follow)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(function_test)
    executor.add_node(lane_change)
    executor.add_node(lane_follow)
    executor.add_node(odom_sub)

    # xxx: TODO: Need to add proper initialization sequence, for now spinning the executor to make sure transform topics are all proper before
    # calling create_path function
    for i in range(
        100
    ):  # Do this to make sure transform broadcaster is properly initalized
        executor.spin_once()

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    function_test.function_test()

    function_test.destroy_node()
    lane_change.destroy_node()
    lane_follow.destroy_node()
    odom_sub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
