#! /usr/bin/env python
import unittest
import rospy
import rostest

from spawn_model_test import SpawnModelTest
from joint_state_topic_test import JointStateTopicTest
from twist_topic_test import TwistTopicTest
from gps_topic_test import GpsTopicTest
from scan_topic_test import ScanTopicTest
from sonar_topic_test import SonarTopicTest


class GazeboTests(unittest.TestSuite):
    def __init__(self):
        rospy.init_node('igvc_self_drive_gazebo_tests')
        super(GazeboTests, self).__init__()
        tests = [
            SpawnModelTest('vehicle', False, False, True),  # Don't publish TF, raw actuator mode
            JointStateTopicTest(),
            TwistTopicTest(False),
            GpsTopicTest(False),
            ScanTopicTest(False),
            SonarTopicTest(False),
            SpawnModelTest('vehicle_2', True, True, True)    # Publish TF, twist mode
        ]

        self.addTests(tests)


if __name__ == '__main__':
    rostest.rosrun('igvc_self_drive_gazebo_plugins', 'gazebo_tests', 'gazebo_tests.GazeboTests')
