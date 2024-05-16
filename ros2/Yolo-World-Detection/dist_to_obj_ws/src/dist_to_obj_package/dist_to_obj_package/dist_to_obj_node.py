from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from object_detection_submodule.object_detection import ObjectDetection
import cv2
import sys
 
class Dist_To_Object(Node):
    def __init__(self,model):
        super().__init__('dist_to_obj_node')
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.distToObj_publisher = self.create_publisher(Float64, '/dist_to_obj',10)
        self.model = model
        self.model.init_model()
 
    def timer_callback(self):
        msg_out = Float64()

        # Get keyboard input
        key = cv2.waitKeyEx(1)

        # Exit loop if 'q' key is pressed
        if key == ord('q') or key == 27:  # 'q' or Esc key
            self.model.close_cam()  # Close camera before exiting
            self.get_logger().info('Exiting node...')
            sys.exit()
            # rclpy.shutdown()  # Shutdown ROS2
            # return

        # Run object detection
        self.model.objectDetection()
        msg_out.data = self.model.distance
        self.distToObj_publisher.publish(msg_out)
        self.get_logger().info('Distance to object: %f' % self.model.distance)

        

def main(args=None):
    # rclpy.init(args=args)  # Initialize ROS2 program
    # model = ObjectDetection(display=True)
    # node = Dist_To_Object(model=model)  # Instantiate Node
    # rclpy.spin(node)  # Puts the node in an infinite loop
    # # Clean shutdown should be at the end of every node
    # node.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=args)  # Initialize ROS2 program
    model = ObjectDetection(display=True)
    node = Dist_To_Object(model=model)  # Instantiate Node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
