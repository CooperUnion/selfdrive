import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class Subscribe:
    def __init__(self):
        self.sub_twist = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.sub_path = rospy.Subscriber('/global_planner/plan', Path, self.callback)
    def callback(msg):
        while True:
            rospy.loginfo("I heard %s", msg.data)

class Publish:
    def __init__(self, pose):
        self.pose = pose
        self.pub_twist = rospy.Publisher('/pure_pursuit/twist', Twist, queue_size=2)
        self.rate = rospy.Rate(2)
    def publish(self,pose):
        while not rospy.is_shutdown():
            self.pub_twist.publish(pose)
    
class Controller:
    def __init__(self):
        self.sub = Subscribe()
        self.pub = Publish()
        self.pose = self.sub.sub_twist.data
        self.path - self.sub.sub_path.data
    def pure_pursuit(self):
        while True:
            self.pose.linear.x += 10
            self.pose.linear.y += 10
            self.pub.publish(self.pose)
            

if __name__ == '__main__':
    rospy.init_node('pure-pursuit', anonymous=True)
    Controller()
    rospy.spin()