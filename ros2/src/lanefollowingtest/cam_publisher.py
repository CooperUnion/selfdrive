import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class WebcamImagePublisher(Node):
    def __init__(self):
        super().__init__('webcam_image_publisher')
        self.logger = self.get_logger()

        # Set desired frame rate (may need adjustment based on hardware)
        self.frame_rate = 60

        # Initialize OpenCV video capture object
        self.cap = cv2.VideoCapture("/dev/video0")  # 0 for default webcam

        if not self.cap.isOpened():
            self.logger.error("Failed to open webcam!")
            rclpy.shutdown()
            return

        # Initialize CvBridge instance
        self.bridge = CvBridge()

        # Create image publisher
        self.image_pub = self.create_publisher(
            Image, '/camera/image', 10)  # Adjust queue size if needed
        
                # Create image publisher
        self.hsv_pub = self.create_publisher(
            Image, '/camera/image_hsv', 10)  # Adjust queue size if needed


        # Timer to capture and publish images
        self.timer = self.create_timer(
            1 / self.frame_rate, self.capture_and_publish)

        self.logger.info("Webcam image publisher node started!")

    def capture_and_publish(self):
        ret, cv_image = self.cap.read()

        if not ret:
            self.logger.warn("Failed to capture image!")
            return

        # Convert OpenCV image to ROS image message
        raw_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_image = self.bridge.cv2_to_imgmsg(hsv_img, encoding="passthrough")
        # Publish the image
        self.image_pub.publish(raw_image)
        self.hsv_pub.publish(hsv_image)


def main():
    rclpy.init()
    node = WebcamImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
