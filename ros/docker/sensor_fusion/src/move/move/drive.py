import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class MovementPublisher(Node):

    def __init__(self):
        super().__init__('movement_publisher')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg_out = Twist()
        msg_out.linear.x = .2
        msg_out.angular.z = .75
        self.vel_publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    movement_publisher = MovementPublisher()
    rclpy.spin(movement_publisher)
    movement_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()