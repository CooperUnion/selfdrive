from cand.client import Bus
import time
import datetime

import rospy
from std_msgs.msg import Int16, Float32, Bool
import math

class LaneOffsetSubscriber:
    def __init__(self):
        self.offset = 0
        self.dist = 0.0
        self.stop = False
        self.stop_line_detected = False
        rospy.Subscriber('/lane_offset', Int16, self.callback)
        rospy.Subscriber('/barrel_bool', Bool, self.stop_cb)
        rospy.Subscriber('/barrel_dist', Float32, self.dist_cb)
        rospy.Subscriber('/stop_line_detected', Bool, self.stop_line_cb)
    def callback(self, msg):
        self.offset = msg.data
    def stop_cb(self, msg):
        self.stop = msg.data
        # rospy.loginfo(self.stop)
    def dist_cb(self, msg):
        self.dist = msg.data
        # rospy.loginfo(self.dist)
    def stop_line_cb(self, msg):
        self.stop_line_detected = msg.data

def main():
    rospy.init_node('lanefollow', anonymous=True)
    bus = Bus(redis_host='redis')
    lane_offset = LaneOffsetSubscriber()
    start = datetime.datetime.now()

    stop_found = False
    stop_found_time = None
    while not rospy.is_shutdown():
        STEER_OFFSET_DESIRED = 580
        STEER_OFFSET_KP = 0.1
        steer_deviation = STEER_OFFSET_DESIRED - lane_offset.offset
        steer_angle = -STEER_OFFSET_KP * steer_deviation
        bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': math.radians(steer_angle)})

        if lane_offset.stop_line_detected and not stop_found:
            stop_found = True
            stop_found_time = datetime.datetime.now()
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
        
        if stop_found and datetime.datetime.now() - stop_found_time > datetime.timedelta(seconds=0.4):
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 0})
        else:
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1.5})

        if stop_found and datetime.datetime.now() - stop_found_time > datetime.timedelta(seconds=10.5):
            break
        time.sleep(0.005)
        phase = 0
    
    phase_start_time = datetime.datetime.now()
    angle_times = [
        (-0.1, 7.5),
    ]

    while True:
        if phase >= len(angle_times):
            break

        angle, sec = angle_times[phase]
        if datetime.datetime.now() - phase_start_time > datetime.timedelta(seconds=sec):
            phase_start_time = datetime.datetime.now()
            phase = phase + 1
            continue
        bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': math.radians(angle)})
        bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1.5})
        time.sleep(0.005)

    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    print('### LANE KEEPING ###')
    start = datetime.datetime.now()
    while not rospy.is_shutdown():
        STEER_OFFSET_DESIRED = 600
        STEER_OFFSET_KP = 0.1
        steer_deviation = STEER_OFFSET_DESIRED - lane_offset.offset
        steer_angle = -STEER_OFFSET_KP * steer_deviation
        bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': math.radians(steer_angle)})

        if lane_offset.stop:# and (datetime.datetime.now() - start) > datetime.timedelta(seconds=1):
            DIST_FULLSTOP = 90
            BASE_VEL = 1.3
            vel_slap = (1.3 * ((lane_offset.dist - DIST_FULLSTOP) / DIST_FULLSTOP))
            vel = BASE_VEL - vel_slap if vel_slap > 0 else 0
            # print(f"vel is slowing: {vel}")
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': max(0, vel)})
        else:
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1.3})
        time.sleep(0.005)
    # while True:
    #     if(lane_offset.stop == True):
    #         bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 0})
    #         bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': 0})
    #         time.sleep(0.005)'

    # # while True:
    # #     bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 0})
    # #     bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': 0})
    # #     time.sleep(0.005)

if __name__ == "__main__":
    main()