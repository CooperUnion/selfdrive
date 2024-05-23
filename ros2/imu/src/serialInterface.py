import serial
import serial.tools.list_ports

import time


class SerialInterface:

    def __init__(self, baudrate, timeout):
        # port_name = self.findSensorPort()
        port_name = "/dev/ttyACM0"
        self.port = serial.Serial(port = port_name, baudrate=baudrate,bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=timeout)

    def __del__(self):
        self.close()

    # Serial Functions
    def findSensorPort(self):
        '''Find the Road Temperature Sensor Port Name'''
        port = ''
        while len(port)==0:
            try:
                for i in list(serial.tools.list_ports.comports()):
                    if("3 Space Sensor" in str(i)):
                        return str(i).split(' - ')[0]
            except:
                print("Can't find the 3-Space Sensor. CHeck your connections. Trying again in 5 seconds")
                time.sleep(5)


        return port


    def close(self):
        ''' Close the serial port.'''
        self.port.close()

    def writeCommand(self, command):
        """ Write command """
        write_len = self.port.write(command)

        if(write_len != len(command)):
            print("Warning: Lenght of bytes sended are different of the recieved")

    def readLine(self):
        """ Read buffer until \n """
        dataRead = self.port.readline()

        if(len(dataRead)>0):
            try:
                return dataRead.decode().replace('\r\n','')
            except:
                print("Error on remove \r\n. Returning the data with this element")
                return dataRead.decode()

        else:
            print("Wrong command! Try again.")
            return ''



