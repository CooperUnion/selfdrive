#!/usr/bin/env python3

import rospy


from nav_msgs.msg import Odometry


class OdomEKF:
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_ekf', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = rospy.Publisher(
            '/odom_fucking_ekf', Odometry, queue_size=10
        )
        print("hello_start")
        # Wait for the /odom topic to become available
        rospy.wait_for_message('/novatel/oem7/odom', Odometry)
        print("available!")
        # Subscribe to the /robot_pose_ekf/odom_combined topic
        rospy.Subscriber('/novatel/oem7/odom', Odometry, self.pub_ekf_odom)

        rospy.loginfo("Publishing combined odometry on /odom_ekf")

        self.adjusted = False

        # self._adjusted = false

    def pub_ekf_odom(self, msg):
        # print("hello!!!!!!!!!!!!!!!!!!!!1")
        if not self.adjusted:
            self._x_offset = msg.pose.pose.position.x
            self._y_offset = msg.pose.pose.position.y
            self._z_offset = msg.pose.pose.position.z
            self.adjusted = True

        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'novatel_center'
        odom.pose = msg.pose
        odom.pose.pose.position.x -= self._x_offset
        odom.pose.pose.position.y -= self._y_offset
        odom.pose.pose.position.z -= self._z_offset

        odom.pose.pose.position.y = -odom.pose.pose.position.y
        odom.pose.pose.orientation.y = -odom.pose.pose.orientation.y

        self.ekf_pub.publish(odom)


if __name__ == '__main__':
    print("I LIVE")
    try:
        odom = OdomEKF()
        rospy.spin()
    except Exception:
        pass
