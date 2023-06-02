#! /usr/bin/env python
import unittest
import rospy
import collections
from sensor_msgs.msg import JointState


class JointStateTopicTest(unittest.TestCase):
    def __init__(self):
        super(JointStateTopicTest, self).__init__('jointStateTopicTest')
        self.joint_states = JointState()
        self.joint_state_topic = '/joint_states'

    def setUp(self):
        self.sub_joints = rospy.Subscriber(self.joint_state_topic, JointState, self.__recvJointStates)

    def tearDown(self):
        self.sub_joints.unregister()

    def jointStateTopicTest(self):

        # Wait for a JointState message sample on the appropriate topic
        timeout_t = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            if self.joint_states.header.stamp != rospy.Time(0):
                break
            rospy.sleep(0.01)

        self.assertTrue(self.joint_states.header.stamp != rospy.Time(0),
                        msg='Joint states topic [%s] not received' % self.joint_state_topic)

        # Make sure the joint names are correct
        correct_joints = ['steer_fl', 'steer_fr', 'wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']
        self.assertTrue(collections.Counter(self.joint_states.name) == collections.Counter(correct_joints),
                        msg='Joint names %s incorrect, should be %s' % (str(self.joint_states.name), str(correct_joints))
                        )

        # Make sure position array has the appropriate number of elements
        self.assertTrue(len(self.joint_states.position) == len(correct_joints),
                        msg='Joint state position array not correct length (len = %d, should be %d)' % (len(self.joint_states.position), len(correct_joints))
                        )

        # Make sure velocity array has the appropriate number of elements
        self.assertTrue(len(self.joint_states.velocity) == len(correct_joints),
                        msg='Joint state velocity array not correct length (len = %d, should be %d)' % (len(self.joint_states.velocity), len(correct_joints))
                        )

    def __recvJointStates(self, msg):
        self.joint_states = msg
