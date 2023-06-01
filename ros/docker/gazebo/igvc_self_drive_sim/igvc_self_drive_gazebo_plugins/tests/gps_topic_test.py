#! /usr/bin/env python
import unittest
import rospy
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.srv import DeleteModel


class GpsTopicTest(unittest.TestCase):
    def __init__(self, delete_model):
        super(GpsTopicTest, self).__init__('gpsTopicTest')
        self.fix = NavSatFix()
        self.fix_topic = '/fix'
        self.delete_model = delete_model

    def setUp(self):
        self.sub_fix = rospy.Subscriber(self.fix_topic, NavSatFix, self.__recvFix)

    def tearDown(self):
        self.sub_fix.unregister()

        if self.delete_model:
            model_name = 'vehicle'
            delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            try:
                delete_srv.wait_for_service(1.0)
                delete_srv(model_name=model_name)
            except rospy.ServiceException:  # service call failed
                pass
            except rospy.ROSInterruptException:  # ROS shutdown during timeout
                pass
            except rospy.ROSException:  # timeout expired
                pass

    def gpsTopicTest(self):

        # Wait for a NavSatFix message sample on the appropriate topic
        timeout_t = rospy.Time.now() + rospy.Duration(1)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            if self.fix.header.stamp != rospy.Time(0):
                break
            rospy.sleep(0.01)

        self.assertTrue(self.fix.header.stamp != rospy.Time(0),
                        msg='NavSatFix topic [%s] not received' % self.fix_topic)

        # Make sure frame_id is correct
        self.assertEqual(first=self.fix.header.frame_id,
                         second='world',
                         msg='NavSatFix frame_id [%s] should be [%s]' % (self.fix.header.frame_id, 'world')
                         )

    def __recvFix(self, msg):
        self.fix = msg
