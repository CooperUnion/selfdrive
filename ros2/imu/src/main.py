
from y3SpaceDriver import Y3SpaceDriver

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class YostPublisher(Node):
    def __init__(self):
        super().__init__('yost_publisher')
        self.imu_driver = Y3SpaceDriver(115200,6)

        self.publisher = self.create_publisher(
            Imu,
            '/yost_imu',
            10)
        
        timer_period = 0.0001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.imu_driver.prepare()

    def timer_callback(self):
        self.imu_driver.run()
        
        msg = Imu()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "imu"
        msg.orientation.x = self.imu_driver.orientation_x
        msg.orientation.y = self.imu_driver.orientation_y
        msg.orientation.z = self.imu_driver.orientation_z
        msg.orientation.w = self.imu_driver.orientation_w
        msg.angular_velocity.x = self.imu_driver.angular_velocity_x
        msg.angular_velocity.y = self.imu_driver.angular_velocity_y
        msg.angular_velocity.z = self.imu_driver.angular_velocity_z
        msg.linear_acceleration.x = self.imu_driver.linear_acceleration_x
        msg.linear_acceleration.y = self.imu_driver.linear_acceleration_y
        msg.linear_acceleration.z = self.imu_driver.linear_acceleration_z
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    yost_publisher = YostPublisher()

    rclpy.spin(yost_publisher)

    yost_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
