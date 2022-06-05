import rospy

from geometry_msgs.msg import PoseStamped

waypoints = list()

def callback(data):
    print(data)
    waypoints.append(data)

def waypointer():
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    sub = rospy.Subscriber('/rtabmap/goal_out', PoseStamped, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    rospy.init_node('waypoint', anonymous=True)
    try:
        waypointer()
    except rospy.ROSInterruptException:
        pass