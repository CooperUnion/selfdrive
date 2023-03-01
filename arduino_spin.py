import pyfirmata
import time
angvel = 60 ##In RPM
steps = 200 
t = 30/(angvel*steps)
dir = 1
if __name__ == "__main__":
    board = pyfirmata.ArduinoMega("/dev/ttyACM0")
    print('howdy')
    board.digital[5].write(dir)
    while True:
        board.digital[11].write(1)
        time.sleep(t)
        board.digital[11].write(0)
        time.sleep(t)






