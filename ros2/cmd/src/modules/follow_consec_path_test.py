import rclpy
from threading import Thread

from lane_behaviors.odom_sub import OdomSubscriber
from lane_behaviors.lane_change import LaneChange


def main(args=None):
    try:
        rclpy.init(args=args)

        max_dist_to_goal = 0.5
        max_dist_to_path = 1.5

        odom_sub = OdomSubscriber()
        lane_change = LaneChange(odom_sub, max_dist_to_goal, max_dist_to_path)

        relative_x1 = 10
        relative_y1 = 10
        end_yaw1 = 0

        relative_x2 = 10
        relative_y2 = 10
        end_yaw2 = 0

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(lane_change)
        executor.add_node(odom_sub)
        # executor.add_node(lane_change.odom_sub)

        # xxx: TODO: Need to add proper initialization sequence, for now spinning the executor to make sure transform topics are all proper before
        # calling create_path function
        for i in range(
            100
        ):  # Do this to make sure transform broadcaster is properly initalized
            executor.spin_once()

        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        lane_change.create_path(relative_x1, relative_y1, end_yaw1)
        lane_change.follow_path()

        lane_change.create_path(relative_x2, relative_y2, end_yaw2)
        lane_change.follow_path()
        # print(f"xpos: {odom_sub.xpos}, ypos: {odom_sub.ypos}")

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print("Starting shutdown")
        odom_sub.destroy_node()
        lane_change.destroy_node()
        rclpy.try_shutdown()
        print("Finished shutdown")


if __name__ == '__main__':
    main()
