import rospy

from geometry_msgs.msg import Twist


class Decode:
    def __init__(self, topic: str):
        self._topic = rospy.Subscriber(topic, Twist, self._callback)

        self.angle = 0.0
        self.vel   = 0.0

    def _callback(self, msg: Twist):
        angle = msg.angular.z
        vel   = msg.linear.x
