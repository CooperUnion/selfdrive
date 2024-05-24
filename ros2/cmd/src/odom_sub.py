import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('Odometry_Subscriber_Node')

        self.callback_group = ReentrantCallbackGroup()
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/encoder_odom',  # This will change if we switch to filtered odom
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self.odometry_subscriber

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.trans = TransformStamped()

        self.xpos = 0.0
        self.ypos = 0.0
        self.yaw = 0.0
        self.vel = 0.0

    def odom_callback(self, msg):
        # self.xpos = msg.pose.pose.position.x
        # self.ypos = msg.pose.pose.position.y

        # quaternion = (
        #     msg.pose.pose.orientation.x,
        #     msg.pose.pose.orientation.y,
        #     msg.pose.pose.orientation.z,
        #     msg.pose.pose.orientation.w,
        # )
        # euler = euler_from_quaternion(quaternion)  # euler = [R, P, Y]

        # self.yaw = euler[2]
        self.vel = msg.twist.twist.linear.x
        try:
            self.trans = self.tfBuffer.lookup_transform(
                'world', 'front_wheel_center', rclpy.time.Time()
            )

            # print(f"Trans: {self.trans}")

            self.xpos = self.trans.transform.translation.x
            self.ypos = self.trans.transform.translation.y

            quaternion = (
                self.trans.transform.rotation.x,
                self.trans.transform.rotation.y,
                self.trans.transform.rotation.z,
                self.trans.transform.rotation.w,
            )
            euler = euler_from_quaternion(quaternion)  # euler = [R, P, Y]

            self.yaw = euler[2]
        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    odom_sub = OdomSubscriber()

    rclpy.spin(odom_sub)

    odom_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
