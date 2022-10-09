import logging

import numpy as np
import rospy

from geometry_msgs.msg import Twist


class Decode:
    def __init__(self, topic: str):

        self._log   = logging.getLogger('twist::Decode')
        self._topic = rospy.Subscriber(topic, Twist, self._callback)

        rospy.loginfo('Initialized rospy subscriber to twist msg.')

        self.angle = 0.0
        self.vel   = 0.0

    def _callback(self, msg: Twist):
        if msg.linear.x != 0 and msg.angular.z != 0:
            self.angle = -np.arctan(msg.angular.z * 1.8 / msg.linear.x)

        else:
            rospy.loginfo(f'Warning: invalid steering angle combo:lin.x:{msg.linear.x}, ang.z:{msg.angular.z}')
            self.angle = 0

        print(f'ang.z:{msg.angular.z}, final angle: {self.angle}')

        self.angle = self.angle
        self.vel   = msg.linear.x

        rospy.loginfo('Callback complete :)')
