from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from .object_detection_submodule.object_detection import ObjectDetection
import cv2
import sys
from cv_bridge import CvBridge

# from yolo_interfaces.msg import Yolo
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


class Dist_To_Object(Node):
    def __init__(self, model):
        super().__init__('dist_to_obj_node')
        self._bridge = CvBridge()
        timer_period = 0.016  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.img_publisher = self.create_publisher(
            Image, "/obj_detection_img", 10
        )
        self.object_point_cloud_publisher = self.create_publisher(
            Float64MultiArray, '/obj_pointcloud', 10
        )
        self.object_name_publisher = self.create_publisher(
            String, '/obj_name', 10
        )

        self.model = model
        self.model.init_model()

    def timer_callback(self):
        object_point_cloud_msg = Float64MultiArray()
        object_name_msg = String()

        # # Get keyboard input
        # key = cv2.waitKeyEx(1)

        # Exit loop if 'q' key is pressed
        # if key == ord('q') or key == 27:  # 'q' or Esc keyr
        #     self.model.close_cam()  # Close camera before exiting
        #     self.get_logger().info('Exiting node...')
        #     sys.exit()
        # rclpy.shutdown()  # Shutdown ROS2
        # return

        # Run object detection
        self.model.objectDetection()
        image = self.model.annotated_frame
        image = self._bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.img_publisher.publish(image)

        object_name_msg.data = self.model.obj_name

        print(self.model.obj_x)
        print(self.model.obj_y)
        print(self.model.distance)

        object_point_cloud_msg.data = [
            self.model.obj_x,
            self.model.obj_y,
            self.model.distance,
        ]

        self.object_name_publisher.publish(object_name_msg)
        self.object_point_cloud_publisher.publish(object_point_cloud_msg)

        self.get_logger().info(
            "\n ----------------------------------------------\n"
        )
        self.get_logger().info(
            'X Coord: %f \n' % object_point_cloud_msg.data[0]
        )
        self.get_logger().info(
            'Y Coord: %f \n' % object_point_cloud_msg.data[1]
        )
        self.get_logger().info(
            'Distance to object: %f \n' % object_point_cloud_msg.data[2]
        )
        self.get_logger().info('Name to object: %s \n' % object_name_msg.data)
        self.get_logger().info(
            "\n ----------------------------------------------\n"
        )


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
