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

    phase = 0
    phase_start_time = datetime.datetime.now()
    angle_times = [
        (0, 4),
        (-25, 5.5),
        (0, 0.25),
    ]

    while True:
        if phase >= len(angle_times):
            break

        angle, sec = angle_times[phase]
        elapsed = datetime.datetime.now() - phase_start_time
        print(f"phase {phase}: {elapsed}")
        if datetime.datetime.now() - phase_start_time > datetime.timedelta(seconds=sec):
            phase_start_time = datetime.datetime.now()
            phase = phase + 1
            continue
        bus.send('DBW_SteeringCommand', {'DBW_steeringAngle': math.radians(angle)})
        bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1})
        time.sleep(0.005)

    start = datetime.datetime.now()
    while not rospy.is_shutdown():
        if (datetime.datetime.now() - start) > datetime.timedelta(seconds=.25):
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 0})
        else:
            bus.send('DBW_VelocityCommand', {'DBW_linearVelocity': 1})
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