#!/usr/bin/env python3

import math
import numpy as np

class StanleyController:
	def __init__(self, cx, cy, cyaw, tolerance=.1, wheelbase=.22, k_stanley=.5, logger=None):
		self.cx = cx
		self.cy = cy
		self.cyaw = cyaw

		self.tolerance = tolerance
		self.wheelbase = wheelbase
		self.halfwheelbase = wheelbase/2
		self.k_stanley = k_stanley

		self.nw = NearestWaypoint(cx, cy, halfwheelbase=self.halfwheelbase)
		self.nw_rev = NearestWaypoint(cx, cy, halfwheelbase=-self.halfwheelbase)
		self.logger = logger

	def set_logger(self, logger):
		self.logger = logger

	def curvature(self, x, y, yaw, v):
		'''
		x,y,yaw are current position
		v is current speed linear
		cx cy cyaw are of path
		halfwheelbase is distance from com to front axle
		k_stanley is gain
		'''
		if v < 0: # moving backwards!
			yaw = coterminal_angle(yaw + math.pi)
		target_idx, rx, ry = self.nw.get_target_idx(x, y, yaw)

		vel_sign = math.copysign(1, v)

		# cross-track error
		dx = self.cx[target_idx] - rx
		dy = self.cy[target_idx] - ry
		path_yaw = self.cyaw[target_idx]
		cross_track_error = dx*np.sin(path_yaw) - dy*np.cos(path_yaw)
		dist_to_path = np.hypot(dx, dy)
  
		# see if we missed the end of the path?
		if dist_to_path > (abs(cross_track_error) + self.tolerance):
			print(f'dist_to_path={dist_to_path} cross_track_error={cross_track_error}')
			raise Exception('missed the end!')

		# see if we're just generally far away
		if dist_to_path > 5*self.tolerance:
			raise Exception('too far away!')

		# Calculate heading error
		theta_e = yaw - self.cyaw[target_idx]

		# account for the branch cut
		theta_e = (theta_e + math.pi) % (2*math.pi) - math.pi

		# compute the actual steering angle and curvature
		scale = 1
		if v < 0:
			scale = 2 # what the fuck!?

		steer_angle = scale*theta_e*vel_sign + np.arctan(self.k_stanley * cross_track_error / v)
		max_steer_angle = 0.6 # in radians
		delta = np.clip(steer_angle, -max_steer_angle, max_steer_angle)
		curv = math.tan(delta)/self.wheelbase

		if self.logger is not None:
#			self.logger.debug(f'target_idx={target_idx} theta_e={theta_e} cross_track_error={cross_track_error} delta={delta} curv={curv}')
			print(f'target_idx={target_idx} theta_e={theta_e} cross_track_error={cross_track_error} delta={delta} curv={curv} path_yaw={path_yaw}')
			pass

		return delta, v