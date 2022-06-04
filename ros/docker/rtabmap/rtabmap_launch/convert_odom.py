#!/usr/bin/env python3

import roslib
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF():
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_ekf', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = rospy.Publisher('/odom_fucking_ekf', Odometry)
        print("hello_start")
        # Wait for the /odom topic to become available
        rospy.wait_for_message('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped)
        print("available!")
        # Subscribe to the /robot_pose_ekf/odom_combined topic
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.pub_ekf_odom)
        
        rospy.loginfo("Publishing combined odometry on /odom_ekf")
        
    def pub_ekf_odom(self, msg):
        # print("hello!!!!!!!!!!!!!!!!!!!!1")
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'base_link'
        odom.pose = msg.pose
        odom.pose.pose.position.x -= 318333
        odom.pose.pose.position.y -= 4726314
        
        self.ekf_pub.publish(odom)
        
if __name__ == '__main__':
    try:
        O = OdomEKF()
        rospy.spin()
    except:
        pass
        

        