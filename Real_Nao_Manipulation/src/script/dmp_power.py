#!/usr/bin/env python
"""
dynamical movement primitives
Author: Tianming Qiu
Oct 2018
"""
import numpy as np
import matplotlib.pyplot as plt


class DMP:
    def __init__(self, T):
        """
        :param T: the length of input data
        """
        self.T = T
        self.tau = T/500.0
        self.alpha_z = 0.1
        self.alpha = 10.0
        self.beta = 1.0

    def phase(self, n_steps,  t=None):
        """The phase variable replaces explicit timing.

        It starts with 1 at the beginning of the movement and converges
        exponentially to 0.
        """
        phases = np.exp(-self.alpha_z * self.tau * np.array(range(1, n_steps + 1)))
        if t is None:
            return phases
        else:
            return phases[t]

    def basis_function(self, n_features):
        h = self.T
        c_l = [1]
        for i in range(n_features-1):
            c_l.append(self.T * (i + 1) / float(n_features - 1))
        c = np.array(c_l)
        PHI = np.zeros((n_features, self.T))
        for t in range(1, self.T + 1):
            phi = np.exp(-(1.0 / (2 * h)) * ((t - c) ** 2))
            z = self.phase(self.T, t - 1)
            # PHI[:, t-1] = z * phi
            PHI[:, t-1] = z * phi / phi.sum()

        # return s * phi / phi.sum()
        # plt.plot(np.transpose(PHI), lw=4)  # if remove z, then you will get the basic function
        # plt.title('Basic functions')
        # plt.show()
        return PHI

    @staticmethod
    def diff_v(x_demo):
        v_demo = np.diff(x_demo)
        a_demo = np.diff(v_demo)

        return x_demo[2:], v_demo[1:], a_demo

    def imitation(self, x_demo):
        s_demo, v_demo, a_demo = self.diff_v(x_demo)
        g = s_demo[-1]
        desired_forcing = a_demo - self.alpha * self.beta * (g - s_demo) + self.alpha * v_demo
        # plt.plot(desired_forcing)

        PHI = self.basis_function(10)
        w = np.dot(np.linalg.inv(np.dot(PHI, np.transpose(PHI))), np.dot(PHI, desired_forcing))

        return w, PHI

    def predict(self, w, x0, g, PHI):

        learned_forcing = np.dot(w, PHI)
        # plt.plot(learned_forcing)
        # plt.show()

        predict_s = np.zeros((len(learned_forcing), ))
        s_t = x0
        v_t = 0.0
        for i in range(self.T):
            predict_s[i] = s_t

            a_t = self.alpha * self.beta * (g - s_t) + self.alpha * (- v_t) + learned_forcing[i]

            s_t = s_t + v_t*0.2 + a_t * 0.2**2/2.0
            v_t = v_t + a_t * 0.2

        return predict_s

    def implement(self, theta_demon):
        w, PHI = self.imitation(theta_demon)
        predict_s = self.predict(w, theta_demon[2], theta_demon[-1], PHI)
        plt.plot(predict_s)
        plt.plot(theta_demon[2:])
        plt.show()
        return predict_s



