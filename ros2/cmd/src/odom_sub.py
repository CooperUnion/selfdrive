import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('Odometry_Subscriber_Node')

        self.odometry_subscriber = self.create_subscription(
                Odometry,
                '/encoder_odom',    # This will change if we switch to filtered odom
                self.odom_callback,
                10)
        self.odometry_subscriber

        self.xpos = 0.0
        self.ypos = 0.0
        self.yaw = 0.0
        self.vel = 0.0

    def odom_callback(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = euler_from_quaternion(quaternion)   # euler = [R, P, Y]

        self.yaw = euler[2]
        self.vel = msg.twist.twist.linear.x

        self.get_logger().info("Odom topic received")

def main(args=None):
    rclpy.init(args=args)

    odom_sub = OdomSubcriber()

    rclpy.spin(odom_sub)

    odom_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()