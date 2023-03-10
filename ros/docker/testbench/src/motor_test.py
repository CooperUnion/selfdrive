import serial
import math
import time
#import rospy
#from geometry_msgs.msg import Twist
arduino = serial.Serial(port='/dev/ttyACM1',baudrate=9600,timeout=1)
RPS = 200 #Number of Pulses per Rev, Modify on Motor Driver, Then Here
WIDTH = 5 # Meters
WHEEL_RADIUS = 0.5 #Units: Meters, Modify Based on Wheel

def micros_between(vel):
    micros = ['0','0']
    for i in range(0,len(vel)):
        if vel[i] != 0:
            steps_per = vel[i] * RPS
            seconds_between = 1/steps_per
            m_between = seconds_between * 10**6
            micros[i] = str(int(m_between))
    return micros

def send(msg):        
    arduino.write(bytes(msg,'utf-8'))
    return


#class Subscriber:
   
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
#    rospy.init_node('subscriber', anonymous=True)
#    sub = Subscriber()
        v = 10
        w = 5
#        vel = [((2 * v) + (w * WIDTH)) / (4 * WHEEL_RADIUS * math.pi),
#            ((2 * v) - (w * WIDTH)) / (4 * WHEEL_RADIUS * math.pi)] 
        msg = '5000' + " " + '5000' + "\n"
        print('message Sent')
        send(msg)
        time.sleep(5)
        msg = '2000' + " " + '2000' + "\n"
        send(msg)
        time.sleep(2)        
        msg = '5000' + " " + '-5000' + "\n"
        send(msg)
        time.sleep(5)
        msg = '0' + " " + '0' + "\n"
        send(msg)


#    rospy.spin()
