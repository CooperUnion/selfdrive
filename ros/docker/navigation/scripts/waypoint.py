# https://www.programcreek.com/python/?code=carla-simulator%2Fros-bridge%2Fros-bridge-master%2Fcarla_waypoint_publisher%2Fsrc%2Fcarla_waypoint_publisher%2Fcarla_waypoint_publisher.py
# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import csv

class Waypointers:
    def __init__(self):
        rospy.init_node('waypointers', anonymous=True)

        # subscribe to move_base action server
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # wait for connection to be established
        self.move_base.wait_for_server(rospy.Duration(10))

        self.waypoints = list()

        self.read_waypoints()

        for waypoint in self.waypoints:
            finished = self.send_goal(waypoint)
            if finished == -1:
                break
            elif finished == 1:
                continue

        rospy.spin()

    def read_waypoints(self):
        filename = 'waypoints.txt'

        with open(filename, newline='') as file:
            reader = csv.reader(file, delimiter=',', skipinitialspace=True, quotechar='|')
            for row in reader:
                waypoint = Pose(Point(row[0], row[1], row[2]), Quaternion(row[3], row[4], row[5], row[6]))
                self.waypoints.append(waypoint)
    
    def send_goal(self, waypoint):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint

        return self.move(goal)

    def move(self, goal):
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_results(rospy.Duration(10))

        if not finished:
            self.move_base.cancel_goal()
            return -1
        else:
            state = self.move_base.get_state()
            print(state)
            return 1

# waypoints = list()

# # input: a list of coordinate pairs [x, y]
# # output: a list of coordinate pairs [x, y]
# def find_fit(coordinate_list):
#     x = list()
#     y = list()

#     for coordinate in coordinate_list:
#         x.append(coordinate[0])
#         y.append(coordinate[1])
    
#     fit = np.polyfit(x, y, 2)
#     polynomial = np.poly1d(fit)

#     steps = np.linspace(x[0], x[len(x)-1], 10)

#     steps_coordinate = list()
#     for step in steps:
#         steps_coordinate.append([step, polynomial(step)])

#     return steps_coordinate

# def callback(data):
#     print(data)
#     waypoints.append(data)

# def waypointer():
#     # pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
#     # sub = rospy.Subscriber('/rtabmap/goal_out', PoseStamped, callback)
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         pass

if __name__ == '__main__':
    try:
        Waypointers()
    except rospy.ROSInterruptException:
        pass