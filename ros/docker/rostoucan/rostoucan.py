import rospy
import cand

from geometry_msgs.msg import Twist

CANmessage = 'DBW_VelocityCmd'
ROStopic = '/cmd_vel'

class ROStouCAN:
    def __init__(self):
        self.bus = cand.client.Bus(redis_host='redis')
        rospy.Subscriber(ROStopic, Twist, self.callback)
    def callback(self, msg):
        while True:
            rospy.loginfo("I heard %s", msg.linear.x)
            self.bus.send(CANmessage, {
                'linearVelCmd': msg.linear.x,
                'angularVelCmd': msg.angular.z
            })
    
if __name__ == '__main__':
    rospy.init_node('roscan', anonymous=True)
    ROStouCAN()
    rospy.spin()

# sudo apt install python3.9
# python3.9 -m pip install opencan-cand

# start redis container
# set up vcan on local machine 
# sudo modprobe vcan
# sudo ip link add dev vcan0 type vcan
# sudo ip link set up vcan0
# start cand on local machine candump vcan0
# redis local host flag on cand --redis_host localhost

