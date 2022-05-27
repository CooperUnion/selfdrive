import rospy

from geometry_msgs.msg import Twist


class Decode:
    def __init__(self, topic: str):
        self._topic = rospy.Subscriber(topic, Twist, self._callback)

        self.vel = 0.0

    def _callback(self, msg: Twist):
        vel = msg.linear.x
