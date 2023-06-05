import cuber as cuber
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import numpy as np


if __name__ == '__main__':
    # Initialize ros instance
    ros = cuber.ros()   

    # Prepare a publisher
    ros.init_publisher('/test_topic', std_msgs.String, 10)

    # Define a callback function that will be called when a subscriber receives a new message
    def callback(msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        ros.publish(str(img.shape))
    
    # Run subscriber
    ros.subscribe('/zed/zed_node/rgb/image_rect_color', sensor_msgs.Image, callback)
