"""
     Below is a list of commands that can be written via the
     serial.write() function to send raw commands to the 3-Space
     firmware. Unless otherwise noted, these are derived directly
     from the 3-Space API
"""

class Commands:
    
    def __init__(self):
    
        # Orientation Sensor Data Commands
        self.GET_TARED_ORIENTATION_AS_QUATERNION        = b':0\n'
        self.GET_TARED_ORIENTATION_AS_EULER_ANGLES      = b':1\n'
        self.GET_TARED_ORIENTATION_AS_ROTATION_MATRIX   = b':2\n'
        self.GET_TARED_ORIENTATION_AS_AXIS_ANGLE        = b':3\n'
        self.GET_TARED_ORIENTATION_AS_TWO_VECTOR        = b':4\n'
        self.GET_DIFFERENCE_QUATERNION                  = b':5\n'
        self.GET_UNTARED_ORIENTATION_AS_QUATERNION      = b':6\n'
        self.GET_UNTARED_ORIENTATION_AS_EULER_ANGLES    = b':7\n'
        self.GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = b':8\n'
        self.GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE      = b':9\n'
        self.GET_UNTARED_ORIENTATION_AS_TWO_VECTOR      = b':10\n'
        self.GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME       = b':11\n'
        self.GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME     = b':12\n'
    
        # Corrected Raw Data Commands
        self.GET_ALL_CORRECTED_COMPONENT_SENSOR                = b':37\n'
        self.GET_CORRECTED_GYRO_RATE                           = b':38\n'
        self.GET_CORRECTED_ACCELEROMETER_VECTOR                = b':39\n'
        self.GET_CORRECTED_COMPASS_VECTOR                      = b':40\n'
        self.GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = b':41\n'
        self.CORRECT_RAW_GYRO_DATA                             = b':48\n'
        self.CORRECT_RAW_ACCEL_DATA                            = b':49\n'
        self.CORRECT_RAW_COMPASS_DATA                          = b':50\n'
    
        # Misc. Raw Data Commands
        self.GET_TEMPERATURE_C     = b':43\n'
        self.GET_TEMPERATURE_F     = b':44\n'
        self.GET_CONFIDENCE_FACTOR = b':45\n'
    
        # Uncorrected Raw Data Commands
        self.GET_ALL_RAW_COMPONENT_SENSOR_DATA = b':64\n'
        self.GET_RAW_GYRO_RATE                 = b':65\n'
        self.GET_RAW_ACCEL_DATA                = b':66\n'
        self.GET_RAW_COMPASS_DATA              = b':67\n'
    
        # Streaming Commands
        self.SET_STREAMING_SLOTS_EULER_TEMP        = b':80,1,43,255,255,255,255,255,255\n'
        self.SET_STREAMING_SLOTS_EULER_QUATERNION  = b':80,1,0,255,255,255,255,255,255\n'
        self.SET_STREAMING_SLOTS_QUATERNION_EULER  = b':80,0,1,255,255,255,255,255,255\n'
        self.SET_STREAMING_SLOTS_EULER             = b':80,1,255,255,255,255,255,255,255\n'
        self.SET_STREAMING_SLOTS_QUATERNION        = b':80,0,255,255,255,255,255,255,255\n'
        self.SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR_IN_GLOBAL = b':80,0,38,41,255,255,255,255,255\n'

        self.GET_STREAMING_SLOTS                   = b':81\n'
        self.SET_STREAMING_TIMING_100_MS           = b':82,100000,0,0\n'
        self.SET_STREAMING_TIMING_1000_MS          = b':82,1000000,0,0\n'
        self.SET_STREAMING_TIMING_5000_MS          = b':82,5000000,0,0\n'
        self.GET_STREAMING_TIMING                  = b':83\n'
        self.GET_STREAMING_BATCH                   = b':84\n'
        self.START_STREAMING                       = b':85\n'
        self.STOP_STREAMING                        = b':86\n'
        self.UPDATE_CURRENT_TIMESTAMP              = b':95\n'
    
        # Settings Configuration READ Commands
        self.GET_AXIS_DIRECTION           = b':143\n'
        self.GET_FILTER_MODE              = b':152\n'
        self.GET_EULER_DECOMPOSTION_ORDER = b':156\n'
        self.GET_MI_MODE_ENABLED          = b':136\n'
    
        # Settings Configuration WRITE Commands
        self.SET_EULER_ANGLE_DECOMP_ORDER_XYZ = b':16,0\n'
        self.SET_EULER_ANGLE_DECOMP_ORDER_YZX = b':16,1\n'
        self.SET_EULER_ANGLE_DECOMP_ORDER_ZXY = b':16,2\n'
        self.SET_EULER_ANGLE_DECOMP_ORDER_ZYX = b':16,3\n'
        self.SET_EULER_ANGLE_DECOMP_ORDER_XZY = b':16,4\n'
        self.SET_EULER_ANGLE_DECOMP_ORDER_YXZ = b':16,5\n'
        self.OFFSET_WITH_CURRENT_ORIENTATION  = b':19\n'
        self.TARE_WITH_CURRENT_ORIENTATION    = b':96\n'
        self.TARE_WITH_CURRENT_QUATERNION     = b':97\n'
        self.SET_MI_MODE_ENABLED              = b':112,1\n'
        self.SET_MI_MODE_DISABLED             = b':112,0\n'
        self.BEGIN_MI_MODE_FIELD_CALIBRATION  = b':114\n'
        self.SET_AXIS_DIRECTIONS_ENU          = b':116,8\n'
        self.SET_AXIS_DIRECTIONS_DEFAULT      = b':116,5\n'
    
        # Calibration Commands
        self.BEGIN_GYRO_AUTO_CALIB       = b':165\n'
        self.SET_CALIB_MODE_BIAS         = b':169,0\n'
        self.SET_CALIB_MODE_SCALE_BIAS   = b':169,1\n'
        self.GET_CALIB_MODE              = b':170\n'
    
        # System Commands
        self.GET_FIRMWARE_VERSION_STRING = b':223\n'
        self.RESTORE_FACTORY_SETTINGS    = b':224\n'
        self.SOFTWARE_RESET              = b':226\n'
