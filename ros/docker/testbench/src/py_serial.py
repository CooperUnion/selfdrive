import serial
import time
import math
arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=1)
RPS = 200 #Number of Pulses per Rev, Modify on Motor Driver, Then Here

WHEEL_RADIUS = 0.5 #Units: Meters, Modify Based on Wheel
Lin_Vel = [5,5] #Signed m/s for each wheel, Controlled by CMD_Vel

sbs = [0,0] #Definition of 0 speed, needed a value for that
for i in range(len(Lin_Vel)):
    if Lin_Vel[i] == 0:
        sbs[i] = '0'
    else:
        ang_vel = Lin_Vel[i]/(WHEEL_RADIUS*2*math.pi) #revolutions/sec
        s = (2*(10**6))/(ang_vel * RPS) 
        sbs[i] = str(int(s)) #seconds/sec, dividing by two for positive and negative edges

def send(msg):
    arduino.write(bytes(msg,'utf-8'))
    return


if __name__ == '__main__':
    print('booted')
    while True:
        #msg = sbs[0] + " " + sbs[1] + "\n"
        msg = "200 200 \n"

        send(msg)
        time.sleep(1)
