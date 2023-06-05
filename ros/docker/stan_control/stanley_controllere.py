import math
import numpy as np
from collections import namedtuple

from pico_bridge.pp import clothoid_path_planner


Point = namedtuple("Point", ["x", "y"])

class ParkingSpot:
	def __init__(self, x_0, y_0, yaw_0, barrel1_x, barrel1_y, barrel2_x, barrel2_y):
		self.start_point = Point(x_0, y_0)
		self.start_orientation = yaw_0
		self.barrel1 = Point(barrel1_x, barrel1_y)
		self.barrel2 = Point(barrel2_x, barrel2_y)
		self.goal_point = self.midpoint(self.barrel1, self.barrel2)
		self.goal_orientation = self.angle(self.barrel1, self.barrel2)

	def midpoint(self, point1, point2):
		x1, y1 = point1
		x2, y2 = point2
		x = (x1 + x2) / 2
		y = (y1 + y2) / 2
		return Point(x, y)

	def angle(self, point1, point2):
		x1, y1 = point1
		x2, y2 = point2
		dx = x2 - x1
		dy = y2 - y1
		angle = math.atan2(dy, dx)
		return angle

	def get_goal_point(self):
		return self.midpoint(self.barrel1, self.barrel2)

	def get_goal_orientation(self):
		return self.angle(self.barrel1, self.barrel2)

 

#x_0 = 0 #get from camera at t = 0, start of experiment
#y_0 = 0 #^
#yaw_0 = 0 #^
#
##end goal parking spot also from vicon
##competition will have 2 barrels in the way and need to park between them
##give x and y for each barrel to the parking spot generator
##will give location of center of mass~
#spot = ParkingSpot(x_0=0, y_0=0, yaw_0=0, barrel1_x=5, barrel1_y=5, barrel2_x=5, barrel2_y=5)
#
#start_point = spot.start_point
#start_orientation = spot.start_orientation
#goal_point = spot.goal_point
#goal_orientation = spot.goal_orientation
#
#num_path_points = 100 #might want to play with
#
#clothoid_path = clothoid_path_planner.generate_clothoid_path(
#	start_point, start_orientation,
#	goal_point, goal_orientation,
#	num_path_points)

def coterminal_angle(a):
	return (a + math.pi) % (2*math.pi) - math.pi

def calculate_yaw(cx, cy, start_yaw):
	yaw = [start_yaw]
	for i in range(1, len(cx)):
		dx = cx[i] - cx[i-1]
		dy = cy[i] - cy[i-1]
		angle = np.arctan2(dy, dx) # calculate the angle between the two points
		yaw.append(angle)
	return yaw

#note: these are values for center of mass
#cx, cy = clothoid_path
#cyaw = calculate_yaw(cx,cy)

# print(clothoid_path)

# def plot_path(path):
#	 x_values, y_values = path
#	 plt.plot(x_values, y_values)
#	 plt.title('Clothoid path')
#	 plt.show()

# plot_path(clothoid_path)

def lead_axle(x, y, yaw, dist):
	rx = x + dist*np.cos(yaw)
	ry = y + dist*np.sin(yaw)
	return rx, ry

class NearestWaypoint:
	def __init__(self, cx, cy, halfwheelbase):
		self.cx = cx
		self.cy = cy
		self.halfwheelbase = halfwheelbase

	def get_target_idx(self, x, y, yaw):
		# Calculate position of the front axle
		fx, fy = lead_axle(x, y, yaw, self.halfwheelbase)

		dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
		dy = [fy - icy for icy in self.cy] # Find the y-axis^

		d = np.hypot(dx, dy) # Find the distance from the front axle to the path
		target_idx = np.argmin(d) # Find the shortest distance in the array

#		print(f'{target_idx}: x={x} y={y} fx={fx} fy={fy}')

		return target_idx, fx, fy


#takes in current most values to calculate index of nearest waypoint
# target_idx = nw.get_target_idx(x = 1, y = 1, yaw = 0.2)
# print(target_idx)

# constexpr const static float wheelbase = 0.22f; // in m
# constexpr const static float track_width = 0.225f; // in m
# constexpr const static float wheel_radius = 0.04f; // in m

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

		return curv, cross_track_error, dist_to_path, rx, ry, delta, v
	
	
