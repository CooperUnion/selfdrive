# from irobot_create_msgs.msg import WheelTicks, WheelTicks 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# publishing twist message top the turtle bot 
# subscribing to whatever the velocity and angle topics 

class TurtleBot4VelCtrlNode(Node):

    def __init__(self):
        super().__init__('turtlebot4_velctrl_node')
        
        self.camData_subscriber = self.create_subscription(
            Float64,
            '/cam_data',
            self.camData_callback,
            qos_profile_sensor_data)


        self.vel_publisher = self.create_publisher(Twist,'robot_0/cmd_vel', 10)

    def camData_callback(self,msg_in):
        msg_out = Twist()

        k = 1.0
        err_z = msg_in.data

        ang_input = err_z * k

        msg_out.linear.x = 0.1 #define constant speed
        msg_out.linear.y = 0.0
        msg_out.linear.z = 0.0

        msg_out.angular.x = 0.0
        msg_out.angular.y = 0.0
        msg_out.angular.z = ang_input

        self.vel_publisher.publish(msg_out)
    
def main(args=None): 
    rclpy.init(args=args) # Initialize ROS2 program
    node = TurtleBot4VelCtrlNode() # Instantiate Node
    rclpy.spin(node) # Puts the node in an infinite loop
    
    # Clean shutdown should be at the end of every node
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
