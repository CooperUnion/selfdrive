
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('point_publisher')
        self.pose_publisher = self.create_publisher(PoseStamped, '/pose', 10)
        self.obstacle_publisher = self.create_publisher(PoseWithCovarianceStamped, '/obstacle', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"PUBLISHED {pose_msg}")

        obstacle_msg = PoseWithCovarianceStamped()
        obstacle_msg.header.frame_id = "map"

        obstacle_msg.pose.pose.position.x = 5.0
        obstacle_msg.pose.pose.position.y = 5.0
        obstacle_msg.pose.pose.position.z = 0.0

        obstacle_msg.pose.pose.orientation.x = 0.0
        obstacle_msg.pose.pose.orientation.y = 0.0
        obstacle_msg.pose.pose.orientation.z = 0.0
        obstacle_msg.pose.pose.orientation.w = 0.0

        obstacle_msg.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
        
        self.obstacle_publisher.publish(obstacle_msg)
        self.get_logger().info(f"PUBLISHED {obstacle_msg}")

def main(args=None):
    rclpy.init(args=args)

    obstacle_publisher = ObstaclePublisher()

    rclpy.spin(obstacle_publisher)

    obstacle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()