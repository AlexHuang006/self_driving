#!/usr/bin/env python3

from planning_pkg.scripts.others.utils import nearest_point
from planning_pkg.scripts.others.utils import intersect_point
from planning_pkg.scripts.others.utils import get_actuation

import numpy as np
import warnings

class PurePursuitPlanner():
    def __init__(self, wheelbase=0.33, waypoints=None):
        self.max_reacquire = 20.
        self.wheelbase = wheelbase
        self.waypoints = waypoints

    def _get_current_waypoint(self, lookahead_distance, position, theta):

        nearest_p, nearest_dist, t, i = nearest_point(position, self.waypoints[:, 0:2])
        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = intersect_point(position,
                                                      lookahead_distance,
                                                      self.waypoints[:, 0:2],
                                                      i + t,
                                                      wrap=True)
            if i2 is None:
                return None
            current_waypoint = np.array([self.waypoints[i2, 0], self.waypoints[i2, 1], self.waypoints[i, 2]])
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return self.waypoints[i, :]
        else:
            return None

    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, waypoints=None):
        if waypoints is not None:
            if waypoints.shape[1] < 3 or len(waypoints.shape) != 2:
                raise ValueError('Waypoints needs to be a (Nxm), m >= 3, numpy array!')
            self.waypoints = waypoints
        else:
            if self.waypoints is None:
                raise ValueError('Please set waypoints to track during planner instantiation or when calling plan()')
        position = np.array([pose_x, pose_y])
        lookahead_point = self._get_current_waypoint(lookahead_distance,
                                                     position,
                                                     pose_theta)

        if lookahead_point is None:
            warnings.warn('Cannot find lookahead point, stopping...')
            return 0.0, 0.0

        speed, steering_angle = get_actuation(pose_theta,
                                              lookahead_point,
                                              position,
                                              lookahead_distance,
                                              self.wheelbase)

        return steering_angle, speed