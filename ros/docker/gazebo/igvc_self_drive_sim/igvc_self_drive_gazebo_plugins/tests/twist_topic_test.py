#! /usr/bin/env python
import unittest
import rospy
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.srv import DeleteModel


class TwistTopicTest(unittest.TestCase):
    def __init__(self, delete_model):
        super(TwistTopicTest, self).__init__('twistTopicTest')
        self.twist = TwistStamped()
        self.twist_topic = '/twist'
        self.delete_model = delete_model

    def setUp(self):
        self.sub_twist = rospy.Subscriber(self.twist_topic, TwistStamped, self.__recvTwist)

    def tearDown(self):
        self.sub_twist.unregister()

        if self.delete_model:
            delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            try:
                delete_srv.wait_for_service(1.0)
                delete_srv.call(model_name='vehicle')
            except rospy.ServiceException as e:  # service call failed
                rospy.logerr(str(e))
            except rospy.ROSInterruptException as e:  # ROS shutdown during timeout
                rospy.logerr(str(e))
            except rospy.ROSException as e:  # timeout expired
                rospy.logerr(str(e))

    def twistTopicTest(self):

        # Wait for a twist feedback message sample on the appropriate topic
        timeout_t = rospy.Time.now() + rospy.Duration(1)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            if self.twist.header.stamp != rospy.Time(0):
                break
            rospy.sleep(0.01)

        self.assertTrue(self.twist.header.stamp != rospy.Time(0),
                        msg='TwistStamped topic [%s] not received' % self.twist_topic)

        # Make sure frame_id is correct
        self.assertEqual(first=self.twist.header.frame_id,
                         second='base_footprint',
                         msg='TwistStamped frame_id [%s] should be [%s]' % (self.twist.header.frame_id, 'base_footprint')
                         )

    def __recvTwist(self, msg):
        self.twist = msg
