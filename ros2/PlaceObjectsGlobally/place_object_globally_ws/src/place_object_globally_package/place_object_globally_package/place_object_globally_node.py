import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance
from std_msgs.msg import Float32MultiArray


class ObjGlobalLocation(Node):
    def __init__(self):
        super().__init__('Object_Global_Location_Node')

        # Might need to apply on offset because the object location is given wrt the zed's left camera 

        # in this contect we only care about the vehicle's x and y pos
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0

        self.obj_x = 0.0
        self.obj_y = 0.0

        # Subscribe to get information on the vehicle and object positions
        self.obj_information_subscription = self.create_subscription(
            Float32MultiArray,
            'obj_pointcloud',
            self.obj_information_callback,
            10)
        
        self.vehicle_position_subscription = self.create_subscription(
            Odometry,
            'encoder_odom',
            self.vehicle_position_callback,
            10)
        
        self.vehicle_position_subscription

        # Publish the object's location wrt to where we started 
        self.object_global_location_publisher = self.create_publisher(PoseWithCovariance, '/obj_location', 10) # want to change to pose with covar in the future

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def obj_information_callback(self,msg):
        self.obj_x = msg.data[0]
        self.obj_y = msg.data[1]

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
        

        if not any(value is None for value in [self.vehicle_x, self.vehicle_y, self.obj_x, self.obj_y]):
            obj_location.pose.position.x = self.vehicle_x + self.obj_x
            obj_location.pose.position.y = self.vehicle_y + self.obj_y
            obj_location.pose.position.z = 0.0

            obj_location.pose.orientation.x = 0.0
            obj_location.pose.orientation.y = 0.0
            obj_location.pose.orientation.z = 0.0
            obj_location.pose.orientation.w = 0.0

            self.object_global_location_publisher.publish(obj_location)
        else:
            print("vehicle xy or object xy are None. Waiting...")

def main(args=None):

    rclpy.init(args=args)  # Initialize ROS2 program
    node = ObjGlobalLocation()  # Instantiate Node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()