#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path

import math
import numpy as np
import turtle

if __name__ == "__main__":
    rospy.init_node('path_gen', anonymous=True)
    pub_path = rospy.Publisher('/path', Path, queue_size=2)
    rate = rospy.Rate(5)

    path = Path()
    n_points = 1000

    for i in range(0, n_points - 1):
        path.PoseStamped[i].pose.position.x = i / 100
        path.PoseStamped[i].pose.position.y = 10 * np.sin(
            0.1 * i / 100
        )  # Replace with whatever function you want
        # path.PoseStamped[i].pose.position.y = np.sqrt(1+10*i)
        # path.PoseStamped[i].pose.position.y = 0

    while not rospy.is_shutdown():
        pub_path.publish(path)
        rate.sleep()
