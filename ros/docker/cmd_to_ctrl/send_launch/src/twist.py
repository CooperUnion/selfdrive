import rospy
from geometry_msgs.msg import Twist

from cand.client import Client

MESSAGE = "dbwNode_Accel_Cntrls_Cmd"
CHAR_MODE = 0
K_P = 1
K_I = 1
K_D = 1

class TwistCan:
    def __init__(self):
        self.sub_twist = rospy.Subscriber('/twist', Twist, self.callback)
        self.can = Client()
    def callback(self, msg):
        self.can.send(MESSAGE, {
            "CharMode": CHAR_MODE,
            "TargetVel": msg.linear.x / 1000,
            "Kp": K_P / 1000,
            "Ki": K_I / 1000,
            "Kd": K_D / 1000
        })

if __name__ == '__main__':
    rospy.init_node('twist-can', anonymous=True)
    TwistCan()
    rospy.spin()