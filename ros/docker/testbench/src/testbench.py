import serial
import math
import rospy
from geometry_msgs.msg import Twist
arduino = serial.Serial(port='/dev/ttyUSB0',baudrate=9600,timeout=1)
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
    return micros

def send(msg):        
    arduino.write(bytes(msg,'utf-8'))
    return


class Subscriber:
   
    def __init__(self):
        rospy.Subscriber('/twist',Twist,self.callback)
    
    def callback(self,msg):
        v = msg.linear.x
        w = msg.angular.z
        vel = [((2 * v) + (w * WIDTH)) / (4 * WHEEL_RADIUS * math.pi),
            ((2 * v) - (w * WIDTH)) / (4 * WHEEL_RADIUS * math.pi)] 
        micros = micros_between(vel)
        msg = micros[0] + " " + micros[1] + "\n"
        send(msg)


if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    sub = Subscriber()
    rospy.spin()
