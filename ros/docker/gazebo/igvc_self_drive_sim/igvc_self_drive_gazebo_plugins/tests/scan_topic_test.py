#! /usr/bin/env python
import unittest
import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import DeleteModel


class ScanTopicTest(unittest.TestCase):
    def __init__(self, delete_model):
        super(ScanTopicTest, self).__init__('scanTopicTest')
        self.scan = LaserScan()
        self.scan_topic = '/scan'
        self.delete_model = delete_model

    def setUp(self):
        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.__recvScan)

    def tearDown(self):
        self.sub_scan.unregister()

        if self.delete_model:
            delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            try:
                delete_srv.wait_for_service(1.0)
                delete_srv(model_name='vehicle')
            except rospy.ServiceException as e:  # service call failed
                rospy.logerr(str(e))
            except rospy.ROSInterruptException as e:  # ROS shutdown during timeout
                rospy.logerr(str(e))
            except rospy.ROSException as e:  # timeout expired
                rospy.logerr(str(e))

    def scanTopicTest(self):

        # Wait for a NavSatFix message sample on the appropriate topic
        timeout_t = rospy.Time.now() + rospy.Duration(1)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            if self.scan.header.stamp != rospy.Time(0):
                break
            rospy.sleep(0.01)

        self.assertTrue(self.scan.header.stamp != rospy.Time(0),
                        msg='LaserScan topic [%s] not received' % self.scan_topic)

        # Make sure frame_id is correct
        self.assertEqual(first=self.scan.header.frame_id,
                         second='laser_frame',
                         msg='LaserScan frame_id [%s] should be [%s]' % (self.scan.header.frame_id, 'laser_frame')
                         )

    def __recvScan(self, msg):
        self.scan = msg
