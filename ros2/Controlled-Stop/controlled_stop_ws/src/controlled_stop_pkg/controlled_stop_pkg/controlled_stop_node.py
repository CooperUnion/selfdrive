import rclpy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.node import Node
import numpy as np


class Controlled_Stop(Node):
    def __init__(self):
        super().__init__('controlled_stop_node')
        self.error_subscriber = self.create_subscription(
            Float64, 'dist_to_obj', self.error_callback, 10)
        self.velocity_publisher = self.create_publisher(
            Twist, 'robot_3/cmd_vel', 10)
        self.t_0 = time.time()
        self.timelist = []
        rows, cols = (0, 2)
        self.prev_time_error = np.zeros(shape=(2, 1))
        self.curr_time_error = np.zeros(shape=(2, 1))
        self.prev_error_integrals = 0
        self.first_through = 1

    def error_callback(self, dist):
        kp = (0.001)
        ki = (0.001)
        x_d = (1000.0)
        error = -(x_d - dist.data)

        p_term = error*kp

        # add conditional to skip calcuolation on the first point

        if self.first_through == 0:
            trap_integral = np.float64((self.curr_time_error[1]+self.prev_time_error[1]) * (
                self.curr_time_error[0]+self.prev_time_error[0]) * 0.5)
            self.prev_error_integrals = np.float64(
                self.prev_error_integrals + trap_integral)
            i_term = np.float64(ki * self.prev_error_integrals)
        else:
            i_term = 0.0
            self.first_through = 0

        u = np.float64(p_term + i_term)

        if u < 0.0:
            u = 0.0

        msg_out = Twist()
        msg_out.linear.x = u
        print(msg_out)
        self.velocity_publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 program
    node = Controlled_Stop()  # Instantiate Node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
