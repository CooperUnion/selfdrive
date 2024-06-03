import math
import numpy as np


def coterminal_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# Tune k_stanley parameter based on testing
# double check wheelbase in meters
class StanleyController:
    def __init__(
        self,
        max_dist_to_path=0.1,
        wheelbase=1.75,
        k_stanley=0.8,
        max_steer_angle=0.35,
        logger=None,
    ):

        self.max_dist_to_path = max_dist_to_path
        self.wheelbase = wheelbase
        self.halfwheelbase = wheelbase / 2
        self.k_stanley = k_stanley
        self.max_steer_angle = max_steer_angle  # in radians

        self.logger = logger

    def set_logger(self, logger):
        self.logger = logger

    def get_target_idx(self, x, y, cx, cy):
        '''
        finds closest point on the path defined by cx,cy
        uses current position defined by x,y,yaw
        '''
        dx = [
            x - icx for icx in cx
        ]  # Find the x-axis of the front axle relative to the path
        dy = [y - icy for icy in cy]  # Find the y-axis^

        d = np.hypot(
            dx, dy
        )  # Find the distance from the front axle to the path
        target_idx = np.argmin(d)  # Find the shortest distance in the array

        return target_idx

    def follow_path(self, x, y, yaw, v, cx, cy, cyaw):
        '''
        x,y,yaw are current position
        v is current speed linear
        cx cy cyaw are of path
        halfwheelbase is distance from com to front axle
        k_stanley is gain
        '''
        print(f'Stanley Inputs: \n {x, y, yaw, v }')

        if v < 0:  # moving backwards!
            yaw = coterminal_angle(yaw + math.pi)

        # get index of nearest point on path
        target_idx = self.get_target_idx(x, y, cx, cy)
        print(f'target_idx={target_idx}')

        # compute cross-track error
        dx = cx[target_idx] - x
        dy = cy[target_idx] - y
        path_yaw = cyaw[target_idx]
        cross_track_error = -(dx * np.sin(path_yaw) - dy * np.cos(path_yaw))

        # May need to catch exceptions
        # see if we missed the end of the path?
        dist_to_path = np.hypot(dx, dy)

        if dist_to_path > (abs(cross_track_error) + self.max_dist_to_path):
            print(
                f'dist_to_path={dist_to_path} cross_track_error={cross_track_error}'
            )
            raise Exception('missed the end!')

        # see if we're just generally far away
        if dist_to_path > self.max_dist_to_path:
            raise Exception('too far away!')

        # Calculate heading error and account for the branch cut
        heading_error = cyaw[target_idx] - yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        steer_cmd = self.get_steering_cmd(heading_error, cross_track_error, v)
        return steer_cmd

    def get_steering_cmd(self, heading_error, cross_track_error, v):

        # compute the actual steering angle and curvature
        scale = 1
        if v < 0:
            scale = 2

        vel_sign = math.copysign(1, v)

        steer_angle = (
            scale * heading_error * vel_sign
            + np.arctan(self.k_stanley * cross_track_error / v)
            if v != 0
            else 0
        )
        delta = np.clip(
            steer_angle, -self.max_steer_angle, self.max_steer_angle
        )

        print(
            f'heading_error={heading_error} cross_track_error={cross_track_error} delta={delta} v={v}'
        )
        return delta
