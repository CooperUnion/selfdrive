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
        rospy.Subscriber('/lane_offset', Int16, self.callback)
        rospy.Subscriber('/barrel_bool', Bool, self.stop_cb)
        rospy.Subscriber('/barrel_dist', Float32, self.dist_cb)
    def callback(self, msg):
        self.offset = msg.data
    def stop_cb(self, msg):
        self.stop = msg.data
        rospy.loginfo(self.stop)
    def dist_cb(self, msg):
        self.dist = msg.data
        rospy.loginfo(self.dist)

def main():
    rospy.init_node('lanefollow', anonymous=True)
    bus = Bus(redis_host='redis')
    lane_offset = LaneOffsetSubscriber()
    start = datetime.datetime.now()
    stopping = False
    while not rospy.is_shutdown():
        STEER_OFFSET_DESIRED = 580
        STEER_OFFSET_KP = 0.1
        steer_deviation = STEER_OFFSET_DESIRED - lane_offset.offset
        steer_angle = -STEER_OFFSET_KP * steer_deviation
        bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': math.radians(steer_angle)})

        if lane_offset.stop and not stopping:
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
            print("### FOUND STOP ###")
        
        if stopping or lane_offset.stop:
            stopping = True
            print("SLOWING/STOPPING")
            DIST_FULLSTOP = 60
            BASE_VEL = 1.2
            vel_slap = (1.2 * ((lane_offset.dist - DIST_FULLSTOP) / DIST_FULLSTOP))
            vel = BASE_VEL - vel_slap if vel_slap > 0 else 0
            print(f"vel is slowing: {vel}")
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': max(0, vel)})
        else:
            print("DRIVE OK")
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1.2})
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