import rospy 
import csv

from geometry_msgs.msg import PoseStamped

class Collector:
    def __init__(self):
        rospy.init_node('collector', anonymous=True)

        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
        rospy.spin()

    def callback(self, msg):
        with open('waypoints.txt', 'a', newline='') as f:
            writer = csv.writer(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ])

if __name__ == '__main__':
    Collector()