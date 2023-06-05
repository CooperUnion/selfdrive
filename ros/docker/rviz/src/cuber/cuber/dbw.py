#!/usr/bin/env python3

#import cuber.cuber as cuber
import rospy
import cand
from std_msgs.msg import Bool

# Want to publish
# - SUP status for each node
# - Node status
# - Bootloader status

class CANtouROS():
    # Create ROS publisher
    def __init__(self, topic_name, topic_type, topic_rate):
        self.bus = cand.client.Bus(redis_host='redis')
        self.msg = topic_type,
        self.pub = rospy.Publisher(topic_name, topic_type, queue_size=topic_rate)
        self.rate = rospy.Rate(50)

    def publishBl(self, node):
        data = self.bus.get_data(node + 'BL_Status')
        if data is None:
            return
        self.msg.data = [data[node + 'BL_BlStatus']]
        self.publish(self.msg)
        self.rate.sleep()

# Initialize publisher
if __name__ == '__main__':
    rospy.init_node('rosdbw', anonymous=True)

    # Sup
    SUP_Node = CANtouROS('SUP_Authorization', Bool, 2)

    SUP_data = SUP_Node.bus.get_data('SUP_Authroization')

    SUP_Node.msg.data = [SUP_data['SUP_brakeAuthorized'],
                         SUP_data['SUP_throttleAuthorized'],
                         SUP_data['SUP_steerAuthorized']]

    SUP_Node.pub.publish(SUP_Node.msg)
    SUP_Node.rate.sleep()

    # Brake
    BRAKEBL_Node = CANtouROS('BRAKEBL_Status', Bool, 2)

    # CTRL
    CTRLBL_Node = CANtouROS('CTRLBL_Status', Bool, 2)

    # STEER
    STEERBL_Node = CANtouROS('STEERBL_Status', Bool, 2)

    # SUP
    SUPBL_Node = CANtouROS('SUPBL_Status', Bool, 2)

    # THROTTLE
    THROTTLEBL_Node = CANtouROS('THROTTLEBL_Status', Bool, 2)

    #
    while not rospy.is_shutdown():
        SUP_data = SUP_Node.bus.get_data('SUP_Authroization')

        SUP_Node.msg.data = [SUP_data['SUP_brakeAuthorized'],
                             SUP_data['SUP_throttleAuthorized'],
                             SUP_data['SUP_steerAuthorized']]

        SUP_Node.pub.publish(SUP_Node.msg)
        SUP_Node.rate.sleep()

        BRAKEBL_Node.publishBL('Brake')
        CTRLBL_Node.publishBL('Ctrl')
        STEERBL_Node.publishBL('Steer')
        SUPBL_Node.publishBL('SUP')
        THROTTLEBL_Node.publishBL('Throttle')