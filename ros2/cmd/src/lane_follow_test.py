import rclpy
from threading import Thread

from lane_behaviors.odom_sub import OdomSubscriber
from lane_behaviors.lane_follower import LaneFollower


def main(args=None):
    try:
        rclpy.init(args=args)

        odom_sub = OdomSubscriber()
        lane_follow = LaneFollower(odom_sub)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(lane_follow)
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

        n = 20
        while True:
            lane_follow.follow_lane(
                1 / n
            )  # Each command takes 0.05s so this loop will take 30s

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print("Starting shutdown")
        odom_sub.destroy_node()
        lane_follow.destroy_node()
        rclpy.try_shutdown()
        print("Finished shutdown")


if __name__ == '__main__':
    main()
