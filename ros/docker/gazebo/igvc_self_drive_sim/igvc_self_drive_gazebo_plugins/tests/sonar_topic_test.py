#! /usr/bin/env python
import unittest
import rospy
from sensor_msgs.msg import Range
from gazebo_msgs.srv import DeleteModel


class SonarTopicTest(unittest.TestCase):
    def __init__(self, delete_model):
        super(SonarTopicTest, self).__init__('sonarTopicTest')
        self.sonar_topics = [
            'front_left',
            'front_center',
            'front_right',
            'rear_left',
            'rear_center',
            'rear_right',
            'left_side_front',
            'left_side_rear',
            'right_side_front',
            'right_side_rear'
            ]
        self.ranges = {}
        self.sonar_subs = {}
        self.delete_model = delete_model

    def setUp(self):

        for topic in self.sonar_topics:
            self.sonar_subs[topic] = rospy.Subscriber('sonar/' + topic,
                                                      Range,
                                                      callback=self.__recvSonar,
                                                      callback_args=topic)
            self.ranges[topic] = Range()

    def tearDown(self):
        for topic in self.sonar_topics:
            self.sonar_subs[topic].unregister()

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

    def sonarTopicTest(self):

        for topic in self.sonar_topics:
            timeout_t = rospy.Time.now() + rospy.Duration(2)
            while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
                if self.ranges[topic].header.stamp != rospy.Time(0):
                    break
                rospy.sleep(0.01)

            self.assertTrue(self.ranges[topic].header.stamp != rospy.Time(0),
                            msg='Range topic [%s] not received' % ('sonar/' + topic))

            self.assertEqual(first=self.ranges[topic].header.frame_id,
                             second=topic + '_link',
                             msg='TwistStamped frame_id [%s] should be [%s]' % (self.ranges[topic].header.frame_id, topic + '_link')
                            )

    def __recvSonar(self, msg, topic):
        self.ranges[topic] = msg
