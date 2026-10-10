"""Wind models. Wind is a velocity field (m/s); the vehicle feels it through
aerodynamic drag on the relative airspeed, not as a direct force."""

import math
import random


class GaussMarkov:
    """First-order Gauss-Markov process: a cheap stand-in for turbulence.

    dg = -g/tau dt + sigma*sqrt(2/tau) dW   ->  stationary std = sigma,
    correlation time = tau.
    """

    def __init__(self, sigma, tau, seed=0):
        self.sigma, self.tau = sigma, tau
        self.value = 0.0
        self.rng = random.Random(seed)

    def step(self, dt):
        if self.sigma <= 0:
            return 0.0
        a = math.exp(-dt / self.tau)
        self.value = a * self.value + self.sigma * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        return self.value


class Wind:
    """Mean wind + optional rotation of the mean vector + gusts.

    kind = "calm" | "steady" | "rotating"
    """

    def __init__(self, kind="calm", mean=(0.0, 0.0), gust_sigma=0.0,
                 gust_tau=1.5, seed=1):
        self.kind = kind
        self.mean = mean
        self.gx = GaussMarkov(gust_sigma, gust_tau, seed)
        self.gz = GaussMarkov(0.4 * gust_sigma, gust_tau, seed + 1)
        self.enabled = kind != "calm"
        self.t = 0.0
        self.vx = self.vz = 0.0

    def step(self, dt):
        self.t += dt
        if not self.enabled:
            self.vx = self.vz = 0.0
            return 0.0, 0.0
        if self.kind == "rotating":
            mx = self.mean[0] * math.cos(0.30 * self.t)
            mz = self.mean[1] * math.sin(0.45 * self.t + 1.0)
        else:
            mx, mz = self.mean
        self.vx = mx + self.gx.step(dt)
        self.vz = mz + self.gz.step(dt)
        return self.vx, self.vz
