from typing import Callable
import numpy as np
import rospy

class ros(object):
    def __init__(self):
        self.node = 'zed_node'
        rospy.init_node(self.node, anonymous=True)
        self.rate = rospy.Rate(10)
        self.subscribe_topic = None
        self.publish_topic = None
        self.publisher = None
    
    def __repr__(self):
        return f'''node: {self.node}
rate: {1.0 / self.rate.sleep_dur.to_sec()}
subscribe topic: {self.subscribe_topic}
publish topic: {self.publish_topic}'''

    def set_rate(self, rate:int):
        self.rate = rospy.Rate(rate)

    def set_node(self, node:str):
        self.node = node
        rospy.init_node(self.node, anonymous=True)

    def subscribe(self, rostopic:str, rostopic_type:type, callback:Callable[..., None]):
        print(f'Subscribing to {rostopic}')
        rospy.Subscriber(rostopic, rostopic_type, callback)
        while not rospy.is_shutdown():
            self.rate.sleep()
    
    def init_publisher(self, rostopic:str, rostopic_type:type, queue_size:int):
        self.publisher = rospy.Publisher(rostopic, rostopic_type, queue_size=queue_size)

    def publish(self, message:type):
        while not rospy.is_shutdown():
            self.publisher.publish(message)
            self.rate.sleep()
