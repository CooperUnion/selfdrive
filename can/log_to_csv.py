import csv
import numpy as np
import cantools
import shlex
import re
import matplotlib.pyplot as plt

class EncoderData:
    def __init__(self, dbc_filename:str):
        self.Encoder0 = np.array([])
        self.Encoder1 = np.array([])
        self.dt = np.array([])
        self.absolute_time = np.array([])
    
    def to_csv(self):
        raise NotImplementedError

class ThrottleData:
    def __init__(self, dbc_filename:str):
        self.ThrottlePercent = np.array([])
        self.absolute_time = np.array([])

    def to_csv(self):
        raise NotImplementedError
        
class ControlsData:
    pass

class LogParser:
    def __init__(self):
        pass
    
    @staticmethod
    def log_to_data_classes(filename: str, dbc_filename:str):
        logfile = open(filename) # Open Log file
        lines = logfile.readlines() # Read log file line by line
        database = cantools.database.load_file(dbc_filename) # Load the dbc file into the database
        # Messages we care about
        encmsg = database.get_message_by_name('dbwNode_Encoder_Data')
        throttlemsg = database.get_message_by_name('dbwNode_Accel_Data')

        # Data storage classes
        enc_data = EncoderData(dbc_filename)
        throttle_data = ThrottleData(dbc_filename)

        i=0
        for l in lines:
            try:
                # Split string by whitespaces
                tokenized = shlex.split(l)
                # Get the raw data length from the tokenized string
                datalen = int(tokenized[3].replace('[', '').replace(']',''))
                # Convert the raw string data into a bytearray from hexformate
                rawdata = bytearray.fromhex(''.join(tokenized[4:4+datalen]))
                
                # If message is an encoder message append data to the encoder data class
                if(encmsg._frame_id == int(tokenized[2], base=16)):
                    print("Encoder message found")
                    data = database.decode_message(encmsg._frame_id, rawdata) # Decode the raw data into dictionary
                    # Append data to the encoder data class variables
                    enc_data.Encoder0 = np.append(enc_data.Encoder0, data['Encoder0'])
                    enc_data.Encoder1 = np.append(enc_data.Encoder1, data['Encoder1'])
                    enc_data.dt = np.append(enc_data.dt, data['Time'])
                    enc_data.absolute_time = np.append(enc_data.absolute_time, float(tokenized[0].replace('(', '').replace(')', '')))
                # If message is a throttle message append data to the encoder data class
                if(throttlemsg._frame_id == int(tokenized[2], base=16)):
                    print("Throttle message found")
                    data = database.decode_message(throttlemsg._frame_id, rawdata) # Decode the raw data into dictionary
                    # Append data to the encoder data class variables
                    throttle_data.ThrottlePercent = np.append(throttle_data.ThrottlePercent, data['Percent'])
                    throttle_data.absolute_time = np.append(throttle_data.absolute_time, float(tokenized[0].replace('(', '').replace(')', '')))
                            
            
            except Exception as e:
                print(e)
                continue

            # Return the data classes
        return (enc_data, throttle_data)


(edata, tdata) = LogParser.log_to_data_classes('test.log', 'igvc_can.dbc')

plt.figure()
plt.ion()
plt.plot(edata.absolute_time, edata.Encoder0, 'r-')
plt.plot(tdata.absolute_time, tdata.ThrottlePercent, 'b-')
plt.show(block=False)
plt.pause(0.01)
print('wait')