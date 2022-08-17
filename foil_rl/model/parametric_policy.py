from math import pi, sin
import numpy as np
import random

A1 = [0.8, 0.9]
A2 = [1.1, 1.2]
rotate_dalpha = 0.1
theta = [0.45, 0.5]
alpha1 = -pi
alpha3 = pi/1.67
alpha_j = [-pi/2, -pi/1.67]
Y_max = [0.02, 0.03]

class parametric_agent:
    def __init__(self, paras):
        self.paras = paras
        self.A1 = 0.8
        self.A2 = 1.1
        self.Y_max = 0.03
        self.alpha1 = -pi
        self.alpha3 = pi / 3
        self.alpha_j = -pi / 1.67
        self.rotate_dalpha = 0.1
        self.theta = 0.6

    def choose_action(self, t):
        act_t = t
        # omega1 = self.A1 * self.theta * self.rotate_dalpha * cos(self.rotate_dalpha * act_t + self.alpha1)
        # omega2 = self.A2 * self.theta * self.rotate_dalpha * cos(self.rotate_dalpha * act_t)
        # omega3 = self.theta * self.rotate_dalpha * cos(self.rotate_dalpha * act_t - self.alpha3)
        omega1 = self.A1 * self.theta * sin(self.rotate_dalpha * act_t + self.alpha1)
        omega2 = self.A2 * self.theta * sin(self.rotate_dalpha * act_t)
        omega3 = self.theta * sin(self.rotate_dalpha * act_t + self.alpha3)
        omega_j = self.theta * sin(self.rotate_dalpha * act_t + self.alpha_j)
        # yy = self.Ymax
        dangle1 = omega1 * 0.5
        dangle2 = omega2 * 0.5
        dangle3 = omega3 * 0.5
        dangle_j = omega_j * self.Ymax * 50

        # scale = 0
        return np.array([dangle1, dangle2, dangle3, dangle_j])

