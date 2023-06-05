# Cuber

Cuber is a Python package that provides a convenient way to subscribe to and publish ROS messages. It simplifies the process of interacting with ROS topics by providing an easy-to-use interface.

# Usage
1. Import the cuber package:
```
import cuber as cb
```

2. Subscribing to a `rostopic` can be done as follows:
```
# Initialize the ros instance
ros = cb.ros()

# Define a callback function
def callback(msg):
    # Do something with the message

# Subscribe to a topic by topic name, type, and a callback function
ros.subscribe({rostopic_name}, {rostopic_type}, callback)
```
Replace `{rostopic_name}` with the name of the ROS topic you want to publish to. Specify the message type `{rostopic_type}` that matches the topic you are publishing to.

3. Publishing a ros message can be done as follows:
```
# Initialize the ros instance
ros = cb.ros()

# Initialize the publisher by defining the topic name, type, and queue_size
ros.init_publisher({rostopic_name}, {rostopic_type}, queue_size)

# Publish the message
ros.publish({message})
```

# Example
Here's an example demonstrating the basic usage of the cuber package:
```
import cuber as cb
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

def callback(msg):
    print(type(msg))

if __name__ == '__main__':
    ros = cb.ros()

    # Subscribe to a topic
    ros.subscribe('/zed/zed_node/rgb/image_rect_color', sensor_msgs.Image, callback)

    # Publish to a topic
    ros.init_publisher('/test_topic', std_msgs.String, 10)
    ros.publish('asdf')
```
