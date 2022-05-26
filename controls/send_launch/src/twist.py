import rospy

from geometry_msgs.msg import Twist


class Decode:
    def __init__(self, topic: str):
        self._topic = rospy.Subscribe(topic, Twist, self._callback)

        self.vel = 0.0

    def _callback(self, msg: Twist):
        vel = msg.linear.x


class TwistCan:
    def __init__(self):
        self.sub_twist = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.vel_ctrl = vel_ctrl()
    def callback(self, msg):
        self.vel_ctrl.ctrl_from_twist(msg)

if __name__ == '__main__':
    rospy.init_node('twist-can', anonymous=True)
    rospy.Rate(10)
    TwistCan()
    rospy.spin()
