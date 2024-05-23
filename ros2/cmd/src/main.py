import rclpy

from odom_sub import OdomSubscriber
from lane_change import LaneChange

def main(args=None):
	rclpy.init(args=args)

	odom_sub = OdomSubscriber()
	lane_change = LaneChange(odom_sub)

	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(odom_sub)
	executor.add_node(lane_change)

	executor.spin()

	odom_sub.destroy_node()
	lane_change.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()