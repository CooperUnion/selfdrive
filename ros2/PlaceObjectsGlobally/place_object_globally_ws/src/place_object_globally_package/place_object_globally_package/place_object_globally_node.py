import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance


class ObjGlobalLocation(Node):
    def __init__(self):
        super().__init__('Object_Global_Location_Node')

        # Might need to apply on offset because the object location is given wrt the zed's left camera 

        # in this contect we only care about the vehicle's x and y pos
        self.vehicle_x = None
        self.vehicle_y = None

        self.obj_x = None
        self.obj_y = None

        # Subscribe to get information on the vehicle and object positions
        self.obj_information_subscription = self.create_subscription(
            Yolo,
            'dist_at_obj',
            self.obj_information_callback,
            10)
        
        self.obj_information_subscription
        
        self.vehicle_position_subscription = self.create_subscription(
            Odometry,
            'encoder_odom',
            self.vehicle_position_callback,
            10)
        
        self.vehicle_position_subscription

        # Publish the object's location wrt to where we started 
        self.object_global_location_publisher = self.create_publisher(Pose, '/pose', 10) # want to change to pose with covar in the future

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def obj_information_callback(self,msg):
        self.obj_x = msg.x
        self.obj_y = msg.y

    def vehicle_position_callback(self,msg):
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y

    def timer_callback(self):
        obj_location = PoseWithCovariance()

        # Covariance Matrix
        obj_location.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
        
        obj_location.pose.position.x = self.vehicle_x + self.obj_x
        obj_location.pose.position.y = self.vehicle_y + self.obj_y
        obj_location.pose.position.z = 0.0

        obj_location.pose.orientation.x = 0.0
        obj_location.pose.orientation.y = 0.0
        obj_location.pose.orientation.z = 0.0
        obj_location.pose.orientation.w = 0.0

        self.object_global_location_publisher.publish(obj_location)