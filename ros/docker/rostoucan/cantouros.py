import rospy
import cand

from geometry_msgs.msg import Twist

CANmessage = 'DBW_VelocityCmd'
ROStopic = '/cmd_vel'

class CANtouROS:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        self.msg = Twist()
        self.pub = rospy.Publisher(ROStopic, Twist, queue_size=2)
        self.rate = rospy.Rate(2)
    def publish(self):
        data = self.bus.get_data(CANmessage)
        self.msg.linear.x = data['linearVelCmd']
        self.msg.angular.z = data['angularVelCmd']
        
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
        
if __name__ == '__main__':
    rospy.init_node('canros', anonymous=True)
    listener = CANtouROS()
    listener.publish()
    rospy.spin()

        