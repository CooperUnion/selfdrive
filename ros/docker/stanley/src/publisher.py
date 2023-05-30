#!/usr/bin/env python3

import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

from subscriber import Subscriber
from controller import StanleyController

# Max distance from goal point set in meters
max_dist = 5

def main():

    rospy.init_node('stanley', anonymous=True)
    sub = Subscriber()
    
    # Start publisher
    pub = rospy.Publisher('/cmd_stanley', Float32MultiArray, queue_size=2)
    rate = rospy.Rate(100) # tune based on performance

    cmd = Float32MultiArray()
    stanley = StanleyController(sub.pathx, sub.pathy, sub.pathyaw)

    while not rospy.is_shutdown():
        
        [steer_next, vel_next] = stanley.curvature(sub.xpos, sub.ypos, sub.yaw, sub.vel)

        if np.hypot(sub.xgoal-sub.xpos, sub.ygoal-sub.ypos) > max_dist:
            cmd.data = [steer_next, vel_next]
        else:
            cmd.data = [0,0] # Not sure what to set steering angle to, maybe just leave it in final position?
        pub.publish(cmd)

if __name__ == '__main__':
    main()