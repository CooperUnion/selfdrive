import rclpy

from pico_bridge.command_reception import CommandReceiver
from pico_bridge.vicon_reception import ViconReceiver
from pico_bridge.comms.controller import CommsController

def main(args=None):
	rclpy.init(args=args)

	comms = CommsController('/dev/ttyACM0') # XXX

	vicon_receiver = ViconReceiver()
	command_receiver = CommandReceiver(comms, vicon_receiver)

	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(command_receiver)
	executor.add_node(vicon_receiver)
	executor.spin()

	command_receiver.destroy_node()
	vicon_receiver.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
