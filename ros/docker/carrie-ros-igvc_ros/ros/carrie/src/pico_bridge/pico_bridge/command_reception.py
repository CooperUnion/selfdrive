import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import numpy as np

from carrie_interfaces.msg import ShortCommand, SwitchToConsole, SimpleMove
from carrie_interfaces.srv import Gains
from carrie_interfaces.action import Move

from pico_bridge.pp import parallel_parking, clothoid_path_planner
import pico_bridge.comms.messages as comms_msg

import math
import time
import traceback

class CommandReceiver(Node):
	def __init__(self, comms, vicon_receiver):
		super().__init__('command_receiver')
		self.comms = comms
		self.vicon_receiver = vicon_receiver

		# TODO: talk to the Pico for these
		self.kp = 0.0
		self.ki = 0.0
		self.kd = 0.0

		# just in case!
		self.switch_to_console_sub = self.create_subscription(
			SwitchToConsole,
			'/carrie/instructions',
			self.switch_to_console_callback,
			10)

		"""
		# subscribe to short commands
		self.short_command_sub = self.create_subscription(
			ShortCommand,
			'/carrie/short_commands',
			self.short_command_callback,
			10)
		"""

		"""
		# allow setting gains
		self.gains_service = self.create_service(
			Gains,
			'/carrie/instructions',
			self.gains_callback)
		"""

		self.move_action = ActionServer(
			self,
			Move,
			'/carrie/movement',
			execute_callback=self.move_callback,
			cancel_callback=self.stop_move_callback)

		self.simple_move_sub = self.create_subscription(
			SimpleMove,
			'/carrie/movement',
			self.simple_move_callback,
			10)

		self.moving = False
		self.get_logger().info('(waiting for stuff)')

	def switch_to_console_callback(self, msg):
		self.get_logger().info(f'switching to serial console...')
		switch_msg = comms_msg.SwitchToConsole()
		self.comms.send(switch_msg.pack())

	def short_command_callback(self, msg):
		self.get_logger().info(f'short command: {msg.command}')

	def gains_callback(self, request, response):
		if request.set:
			self.get_logger().info('(setting gains)')
			self.kp = request.kp
			self.ki = request.ki
			self.kd = request.kd

		response.kp = self.kp
		response.ki = self.ki
		response.kd = self.kd
		self.get_logger().info(f'gains: kp={self.kp} ki={self.ki} kd={self.kd}')

		return response

	def simple_move_callback(self, msg):
		self.get_logger().info(f'doing a simple move...')
		simple_move_msg = comms_msg.SimpleMove(msg)
		self.comms.send(simple_move_msg.pack())

	def stop_move_callback(self, goal):
		self.moving = False
		self.comms.send(comms_msg.Stop().pack())
		self.get_logger().info('stopping the move...')
		return rclpy.action.server.CancelResponse.ACCEPT

	def move_callback(self, goal):
		time.sleep(1)

		velocity = goal.request.velocity
		lead_dist = math.copysign(.22/2, velocity) # XXX

		pos = self.vicon_receiver.carrie.pos()
		start_yaw = pos.yaw()
		start_x, start_y = parallel_parking.lead_axle(
			pos.x, pos.y, start_yaw, lead_dist
		)
		tolerance = .1 # m, for now

		feedback = Move.Feedback()
#		feedback.curvature = goal.request.curvature
#		feedback.speed = goal.request.speed
#		feedback.distance = 0.0

		end_yaw = goal.request.yaw
		end_x = goal.request.x
		end_y = goal.request.y

#		end_x, end_y = parallel_parking.lead_axle(
#			goal.request.x, goal.request.y, end_yaw, lead_dist
#		)

		# XXX: hijack the move to go between the barrels
		# TODO: make this its own action!
#		barrel_1 = self.vicon_receiver.barrel_1.pos()
#		barrel_2 = self.vicon_receiver.barrel_2.pos()
#		spot = parallel_parking.ParkingSpot(
#			start_x, start_y, start_yaw,
#			barrel_1.x, barrel_1.y,
#			barrel_2.x, barrel_2.y
#		)
#
#		endpoint = spot.get_goal_point()
#		end_x = endpoint.x
#		end_y = endpoint.y
#		end_yaw = spot.get_goal_orientation()

		self.get_logger().info(f'moving to (x={end_x:.3}, y={end_y:.3}; yaw={end_yaw:.3})...')

		# correct for backwards paths -- yaw refers to the yaw of the car
		if velocity < 0:
			start_yaw = parallel_parking.coterminal_angle(start_yaw + math.pi)
			end_yaw = parallel_parking.coterminal_angle(end_yaw + math.pi)
		path_points = 99
		path_x, path_y = clothoid_path_planner.generate_clothoid_path(
			parallel_parking.Point(start_x, start_y), start_yaw,
			parallel_parking.Point(end_x, end_y), end_yaw,
			path_points
		)
		path_yaw = parallel_parking.calculate_yaw(
			path_x, path_y, start_yaw
		)

		self.get_logger().debug(f'start_x={start_x} start_y={start_y} start_yaw={start_yaw} end_x={end_x} end_y={end_y} end_yaw={end_yaw}')

#		self.get_logger().debug(f'\n\tpath_x={path_x}\n\tpath_y={path_y}\n\tpath_yaw={path_yaw}\n')
		print(f'      {"path_x":30} {"path_y":30} {"path_yaw":30}')
		for i in range(len(path_x)):
			print(f'#{i:4} {path_x[i]:-30} {path_y[i]:-30} {path_yaw[i]:-30}')

		controller = parallel_parking.StanleyController(
			path_x, path_y, path_yaw, tolerance
		)
		controller.set_logger(self.get_logger())

		x = start_x
		y = start_y
		rx = start_x
		ry = start_y
		yaw = start_yaw
		start_time = time.time()
		desired_curvature = 0
		curr_velocity = 0
		self.moving = True
		outbound_id = 1
		inbound_id = 0
		while self.moving and math.dist((rx, ry), (end_x, end_y)) > tolerance:
			while self.comms.has_packet():
				p = self.comms.get_packet()
				if p.id() == comms_msg.MoveFeedback.id():
					move_fb = comms_msg.MoveFeedback(p)
#					print(f'received: {move_fb}')
					curr_velocity = move_fb.velocity

					feedback.distance = move_fb.distance
					feedback.curvature = move_fb.curvature
					feedback.velocity = move_fb.velocity
					inbound_id = move_fb.trans_id

			pos = self.vicon_receiver.carrie.pos()
			x = pos.x
			y = pos.y
			yaw = pos.yaw()

			curr_velocity = velocity # XXX
			try:
				if curr_velocity != 0:
					desired_curvature, feedback.cross_track_error, feedback.dist_to_path, rx, ry \
						= controller.curvature(x, y, yaw, curr_velocity)
			except Exception as e:
				self.get_logger().error('miss reported!')
				traceback.print_exc()
				break

#			print(f'desired_curvature={desired_curvature} outbound_id={outbound_id} inbound_id={inbound_id}')
			move_msg = comms_msg.SimpleMove(
				(
					1, # distance, for now
					-desired_curvature, # XXX
					velocity, # desired, *not* current
				),
				outbound_id
			)
			self.comms.send(move_msg.pack())
			outbound_id = outbound_id + 1

			feedback.x = rx
			feedback.y = ry
			feedback.yaw = yaw
			goal.publish_feedback(feedback)

			print(f'rx={rx} ry={ry} dist={np.hypot(end_x - rx, end_y - ry)}')

			time.sleep(.05)

		was_canceled = not self.moving
		self.moving = False
		end_time = time.time()
		self.comms.send(comms_msg.Stop().pack())

		if math.dist((rx, ry), (end_x, end_y)) <= tolerance:
			if not was_canceled:
				goal.succeed()

			self.get_logger().info(f'done with the move!')
		else:
			if not was_canceled:
				goal.abort()

			self.get_logger().error(f'where the fuck are we?')


		result = Move.Result()
		result.elapsed_time = end_time - start_time
		result.error = math.dist((rx, ry), (end_x, end_y))
		return result
