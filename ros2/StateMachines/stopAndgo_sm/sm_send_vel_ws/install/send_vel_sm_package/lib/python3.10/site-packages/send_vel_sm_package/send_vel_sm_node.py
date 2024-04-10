import sys
import select
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from algoFSM_ex import firstAlgoSM
from .simple_sm_submodule.simple_sm import firstAlgoSM


# Current issue is that it is not taking input from the terminal
class sendVel(Node):
    def __init__(self):
        super().__init__('send_vel_node')
        self.toggle_param = True 

        self.last_toggle_time = time.time()  # Initialize the last toggle time

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sm = firstAlgoSM()
 
    def timer_callback(self):
        msg_out = Twist()
 
        msg_out.linear.x = self.sm.vehicle_vel
        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0
 
        msg_out.angular.x = 0.0
        msg_out.angular.y = 0.0
        msg_out.angular.z = 0.0
 
        # Check if 5 seconds have passed since the last toggle
        if time.time() - self.last_toggle_time >= 5:
            if self.toggle_param:
                self.sm.send("follow_lane_lines")
            else:
                self.sm.send("stop_vehicle")
            self.toggle_param = not self.toggle_param
            self.last_toggle_time = time.time()  # Update the last toggle time

        self.vel_publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 program
    node = sendVel() # Instantiate Node
    rclpy.spin(node) # Puts the node in an infinite loop
    
    # Clean shutdown should be at the end of every node
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
