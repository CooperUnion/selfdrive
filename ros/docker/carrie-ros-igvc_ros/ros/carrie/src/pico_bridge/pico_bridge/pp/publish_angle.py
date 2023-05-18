import rospy 
from geometry_msgs.msg import Twist
import parallel_parking
import fake_vals

class pub_steeringAngle:
    def __init__(self):
        self.msg = Twist()
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 2)
        self.rate = rospy.rate(2) #2G Hz

    def publish(self): 
        self.msg.linear.x = fake_vals.v
        self.msg.angular.z = fake_vals.delta 

        while not rospy.is_shutdown():
                self.pub.publish(self.msg)

if __name__ == '__main__': 
     rospy.init_node('stan_ctrl',anoymous = True)
     publisher = pub_steeringAngle()
     publisher.publish()
     rospy.spin()