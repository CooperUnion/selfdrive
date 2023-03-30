#!/usr/bin/env python3

import serial
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray, String
EncoderTicks = '/encoder_ticks'

arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=1)
RPS = 200 #Number of Pulses per Rev, Modify on Motor Driver, Then Here
WIDTH = 5 # Meters
WHEEL_RADIUS = 0.5 #Units: Meters, Modify Based on Wheel

def micros_between(vel):
    micros = ['0','0']
    for i in range(0,len(vel)):
        if vel[i] != 0:
            steps_per = vel[i] * RPS
            seconds_between = 1/steps_per
            micros[i] = str(int(seconds_between))

def send(msg):        
    arduino.write(bytes(msg,'utf-8'))
    return

class Publisher:
    def __init__(self):
        self.msg = UInt16MultiArray()
        self.pub = rospy.Publisher(EncoderTicks, UInt16MultiArray, queue_size=2)
        self.rate = rospy.Rate(50)

    def publish(self):
        str = arduino.readline()
        if str is None:
            return
        data = str.split('#')
        self.msg.data = [int(data[0]),int(data[1])]
        self.pub.publish(self.msg)
        self.rate.sleep()
    

class Subscriber:   
    def __init__(self):
        rospy.Subscriber('/twist',Twist,self.callback)
    def callback(self,msg):
        v = msg.linear.x
        w = msg.angular.z
        vel = [((2 * v) + (w * WIDTH)) / (4 * WHEEL_RADIUS * math.pi),
            ((2 * v) - (w * WIDTH)) / (4 * WHEEL_RADIUS * math.pi)] 
        micros = micros_between(vel)
        msg = micros[0] + "#" + micros[1] + "\n"
        send(msg)


if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    sub = Subscriber()
    pub = Publisher()
    while not rospy.is_shutdown():
        pub.publish()
    rospy.spin()
