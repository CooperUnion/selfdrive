import cuber.cuber as cuber
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

def callback(msg):
    print(type(msg))

if __name__ == '__main__':
    ros = cuber.ros()

    # ros.subscribe('/zed/zed_node/rgb/image_rect_color', sensor_msgs.Image, callback)
    ros.init_publisher('/test_topic', std_msgs.String, 10)
    ros.publish('asdf')
