import pyfirmata
import time
import math

board = pyfirmata.ArduinoMega('/dev/ttyACM0')
RPS = 360/200 #Number of Pulses per Rev, Modify on Motor Driver, Then Here
WHEEL_RADIUS = 0.5 #Units: Meters, Modify Based on Wheel
Forward = True #Controlled by CMD_Vel
Lin_Vel = [0,0] #M/S for each wheel, Controlled by CMD_Vel
angvel = [i/WHEEL_RADIUS for i in Lin_Vel] #Rad/Sec
dps = [i*180/math.pi for i in Lin_Vel] #Degrees/Sec
sbs = [RPS/(2*i) for i in dps] #Steps/Sec
motor1 = board.digital[11]
motor2 = board.digital[12]
if __name__ == '__main__':
    print('booted')
    while True:
        if Forward:
            board.digital[5].write(1)
        else:
            board.digital[5].write(0)
        Forward = True
        Lin_Vel = [0,0]
        angvel = [i/WHEEL_RADIUS for i in Lin_Vel]
        dps = [i*180/math.pi for i in Lin_Vel]
        sbs = [RPS/(2*i) for i in dps]
         
        motor1.write(1)
        time.sleep(sbs)
        motor1.write(0)
        time.sleep(sbs)