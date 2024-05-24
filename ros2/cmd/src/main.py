import rclpy
from threading import Thread

from odom_sub import OdomSubscriber
from lane_change import LaneChange


def executor_function(executor):
    executor.spin()


def main(args=None):
    rclpy.init(args=args)

    odom_sub = OdomSubscriber()
    lane_change = LaneChange(odom_sub)

    relative_x = 5
    relative_y = 5
    end_yaw = 0

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lane_change)
    executor.add_node(lane_change.odom_sub)

    # xxx: TODO: Need to add proper initialization sequence, for now spinning the executor to make sure transform topics are all proper before
    # calling create_path function
    for i in range(
        100
    ):  # Do this to make sure transform broadcaster is properly initalized
        executor.spin_once()

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    lane_change.create_path(relative_x, relative_y, end_yaw)

    lane_change.follow_path()
    # print(f"xpos: {odom_sub.xpos}, ypos: {odom_sub.ypos}")

    odom_sub.destroy_node()
    lane_change.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
