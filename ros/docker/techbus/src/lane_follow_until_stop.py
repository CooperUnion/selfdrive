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
        rospy.loginfo(self.stop)
    def dist_cb(self, msg):
        self.dist = msg.data
        rospy.loginfo(self.dist)
    def stop_line_cb(self, msg):
        self.stop_line_detected = msg.data

def main():
    rospy.init_node('lanefollow', anonymous=True)
    bus = Bus(redis_host='redis')
    lane_offset = LaneOffsetSubscriber()
    start = datetime.datetime.now()

    stop_found = False
    while not rospy.is_shutdown():
        STEER_OFFSET_DESIRED = 580
        STEER_OFFSET_KP = 0.1
        steer_deviation = STEER_OFFSET_DESIRED - lane_offset.offset
        steer_angle = -STEER_OFFSET_KP * steer_deviation
        bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': math.radians(steer_angle)})

        if lane_offset.stop_line_detected and not stop_found:
            stop_found = True
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
            print("### FOUND STOP ###")c
        
        if stop_found:
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 0})
        else:
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1.0})
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