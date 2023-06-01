#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from subscriber import Subscriber
from controller import StanleyController

# Max distance from goal point set in meters
max_dist = 3

def main():

    rospy.init_node('stanley', anonymous=True)
    sub = Subscriber()
    
    # Start publisher
    # pub = rospy.Publisher('/cmd_stanley', Float32MultiArray, queue_size=2)
    pub = rospy.Publisher('/cmd_stanley', Twist, queue_size=2)
    rate = rospy.Rate(1) # tune based on performance

    # cmd = Float32MultiArray()
    cmd = Twist()

    stanley = StanleyController(sub.pathx, sub.pathy, sub.pathyaw)

    cmd.linear.y = cmd.linear.z = cmd.angular.x = cmd.angular.y = 0 # Needed for Twist message

    while not rospy.is_shutdown():
        # cmd.linear.x = 0.5
        # cmd.angular.z = -0.01

        stanley.cx = sub.pathx
        stanley.cy = sub.pathy  
        stanley.cyaw = sub.pathyaw

        # xpos = 1.955
        # ypos = 0.02
        # yaw = 0.19
        # vel = 0.01
        # [steer_next, vel_next] = stanley.curvature(1.955, 0.02, 0.19, 0.01)
        # print(sub.pathx)
        # print(sub.pathy)
        [steer_next, vel_next] = stanley.curvature(sub.xpos, sub.ypos, sub.yaw, sub.vel)
        # print(f'xpos={sub.xpos} ypos={sub.ypos} yaw={sub.yaw} vel={sub.vel}')

        if np.hypot(sub.xgoal-sub.xpos, sub.ygoal-sub.ypos) > max_dist:
            cmd.linear.x = vel_next
            cmd.angular.z = steer_next
            # cmd.data = [steer_next, vel_next]
        else:
            cmd.linear.x = 0
            cmd.angular.z = 0
            # cmd.data = [0,0] 
            # Not sure what to set steering angle to, maybe just leave it in final position?
            # If it's delta, then this makes sense
        
        print(f'Steering angle={cmd.angular.z} Velocity={cmd.linear.x}')
        pub.publish(cmd)

if __name__ == '__main__':
    main()