from rclpy.node import Node
import numpy as np

from vicon_receiver.msg import Position
from pico_bridge.pp import rotations

import threading
import time

vicon_scale = 1000 # mm

class PositionStruct:
	def __init__(self, msg=None):
		if msg is None:
			self.x = 0
			self.y = 0
			self.z = 0

			self.yaw_ = 0
			self.pitch = 0
			self.roll = 0

		else:
			self.x = msg.x_trans/vicon_scale
			self.y = msg.y_trans/vicon_scale
			self.z = msg.z_trans/vicon_scale

			self.yaw_, self.pitch, self.roll = \
				rotations.Quaternion2Euler(np.array([
					[msg.x_rot],
					[msg.y_rot],
					[msg.z_rot],
					[msg.w]
				]))

			self.frame_number = msg.frame_number

	def yaw(self):
		return self.yaw_

	def __repr__(self):
		return f'Position{{x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw_}, pitch={self.pitch}, roll={self.roll}}}'

class AtomicPos:
	def __init__(self):
		self.lock = threading.Lock()
		self.set_pos(Position())

	def set_pos(self, pos):
		self.lock.acquire()
		self._pos = pos
		self.lock.release()

	def pos(self):
		self.lock.acquire()
		pos = self._pos
		self.lock.release()
		return pos

class ViconReceiver(Node):
	def __init__(self):
		super().__init__('vicon_receiver')
		self.carrie = AtomicPos()
		self.barrel_1 = AtomicPos()
		self.barrel_2 = AtomicPos()

		# subscribe to position data
		self.carrie_sub = self.create_subscription(
			Position,
			'/vicon/carrie/carrie',
			self.position_callback,
			10)

		# subscribe to the barrels
		self.barrel_1_sub = self.create_subscription(
			Position,
			'/vicon/barrel_1/barrel_1',
			self.barrel_1_callback,
			10)

		# subscribe to the barrels
		self.barrel_2_sub = self.create_subscription(
			Position,
			'/vicon/barrel_2/barrel_2',
			self.barrel_2_callback,
			10)

		self.last_message = time.time()
		self.message_interval = 2 # seconds

		self.get_logger().info('(waiting for position data)')

	def maybe_send_message(self):
		now = time.time()
		if now > self.last_message + self.message_interval:
			self.get_logger().debug(f'we\'re at:\n\t{self.carrie.pos()}')
			self.get_logger().debug(f'barrels are at:\n\t#1 -> {self.barrel_1.pos()}\n\t#2 -> {self.barrel_2.pos()}')
			self.last_message = now

	def position_callback(self, msg):
		pos = PositionStruct(msg)
		self.carrie.set_pos(pos)
		self.maybe_send_message()

	def barrel_1_callback(self, msg):
		pos = PositionStruct(msg)
		self.barrel_1.set_pos(pos)
		self.maybe_send_message()

	def barrel_2_callback(self, msg):
		pos = PositionStruct(msg)
		self.barrel_2.set_pos(pos)
		self.maybe_send_message()
