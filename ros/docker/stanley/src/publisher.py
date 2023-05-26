#!/usr/bin/env python3

import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

from subscriber import Subscriber
from controller import StanleyController

def main():

    rospy.init_node('stanley', anonymous=True)
    subscriber = Subscriber()
    
    # Start publisher
    stanley_pub = rospy.Publisher('/stanley_cmd', Float32MultiArray, queue_size=2)
    stanley_cmd = Float32MultiArray()
    rate = rospy.Rate(100) # tune based on performance

    while not rospy.is_shutdown():

        stanley_cmd.data = [v_next, steer_next]
        stanley_pub.publish(stanley_cmd)

if __name__ == '__main__':
    main()