import numpy as np
import rospy


class BaselineEKF:
    def __init__(self, x0, P0, Q, R, dt):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.R = R
        self.dt = dt