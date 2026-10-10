"""Planar quadrotor rigid-body dynamics.

State vector (all SI):
    x, z      position of the CG, z up, ground at z = 0          [m]
    vx, vz    inertial velocity                                  [m/s]
    theta     tilt, positive = right side down (moves right)     [rad]
    q         tilt rate                                          [rad/s]
    wl, wr    left / right rotor speeds                          [rad/s]

Equations of motion (M = total mass, I = inertia about CG, l = arm):
    T_i      = n * k_T * w_i^2                     rotor-pair thrust
    M x''    = (T_l + T_r) sin(theta) + D_x
    M z''    = (T_l + T_r) cos(theta) + D_z - M g
    I theta''= l (T_l - T_r) - c_q q
    w_i'     = (w_cmd,i - w_i) / tau_m             motor lag
    D_j      = -1/2 rho CdA_j |u_j| u_j,  u = v - v_wind

Integrated with classical RK4 at a fixed step.
"""

import math
from .params import Vehicle, Payload, G, RHO_AIR, combined_mass_properties
from .battery import BatteryState


class Quadrotor2D:
    def __init__(self, vehicle: Vehicle, battery: BatteryState, x=0.0, z=0.0):
        self.v = vehicle
        self.batt = battery
        self.s = [x, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.payload = None
        self.mass, self.inertia = combined_mass_properties(vehicle, None)
        self.on_ground = z <= 0.0
        self.last_impact_speed = 0.0
        self.wind = (0.0, 0.0)
        self.cmd = (0.0, 0.0)
        self.accel = (0.0, 0.0)   # last computed inertial acceleration

    # convenient named access
    x = property(lambda self: self.s[0])
    z = property(lambda self: self.s[1])
    vx = property(lambda self: self.s[2])
    vz = property(lambda self: self.s[3])
    theta = property(lambda self: self.s[4])
    q = property(lambda self: self.s[5])
    wl = property(lambda self: self.s[6])
    wr = property(lambda self: self.s[7])

    def attach_payload(self, payload: Payload | None):
        self.payload = payload
        self.mass, self.inertia = combined_mass_properties(self.v, payload)

    def omega_max(self):
        """Rotor speed ceiling. A BLDC motor's speed scales with applied
        voltage (Kv), so a sagging pack directly limits available thrust."""
        if self.batt.empty:
            return 0.0
        return self.v.omega_max_nominal * self.batt.voltage / self.batt.spec.nominal_voltage

    def side_thrust(self, w):
        return self.v.rotors_per_side * self.v.k_thrust * w * w

    def rotor_power(self, thrust_per_rotor):
        """Electrical power for one rotor from actuator-disk momentum theory:
        P_ideal = T^1.5 / sqrt(2 rho A), corrected by figure of merit and
        drivetrain efficiency. Hover-based, so it underestimates in fast
        forward flight - acceptable for this speed range."""
        if thrust_per_rotor <= 0:
            return 0.0
        p_ideal = thrust_per_rotor ** 1.5 / math.sqrt(2 * RHO_AIR * self.v.disk_area)
        return p_ideal / (self.v.figure_of_merit * self.v.drivetrain_eff)

    def electrical_power(self):
        n = self.v.rotors_per_side
        tl = self.side_thrust(self.wl) / n
        tr = self.side_thrust(self.wr) / n
        return n * (self.rotor_power(tl) + self.rotor_power(tr)) + self.v.avionics_power

    def derivatives(self, s, cmd, wind):
        x, z, vx, vz, th, q, wl, wr = s
        v = self.v
        tl, tr = self.side_thrust(wl), self.side_thrust(wr)
        total = tl + tr
        ux, uz = vx - wind[0], vz - wind[1]
        dx = -0.5 * RHO_AIR * v.cda_x * abs(ux) * ux
        dz = -0.5 * RHO_AIR * v.cda_z * abs(uz) * uz
        ax = (total * math.sin(th) + dx) / self.mass
        az = (total * math.cos(th) + dz) / self.mass - G
        alpha = (v.arm * (tl - tr) - v.rot_damping * q) / self.inertia
        dwl = (cmd[0] - wl) / v.motor_tau
        dwr = (cmd[1] - wr) / v.motor_tau
        return [vx, vz, ax, az, q, alpha, dwl, dwr]

    def step(self, dt, w_cmd_l, w_cmd_r, wind=(0.0, 0.0)):
        wmax = self.omega_max()
        cmd = (min(max(w_cmd_l, 0.0), wmax), min(max(w_cmd_r, 0.0), wmax))
        self.cmd, self.wind = cmd, wind

        s = self.s
        k1 = self.derivatives(s, cmd, wind)
        k2 = self.derivatives([a + 0.5 * dt * b for a, b in zip(s, k1)], cmd, wind)
        k3 = self.derivatives([a + 0.5 * dt * b for a, b in zip(s, k2)], cmd, wind)
        k4 = self.derivatives([a + dt * b for a, b in zip(s, k3)], cmd, wind)
        self.s = [a + dt / 6.0 * (b + 2 * c + 2 * d + e)
                  for a, b, c, d, e in zip(s, k1, k2, k3, k4)]
        self.accel = (k1[2], k1[3])

        self.batt.draw(self.electrical_power(), dt)
        self._ground_contact(dt)

    def _ground_contact(self, dt):
        """Rigid ground with friction on the landing gear. Records the
        vertical speed at touchdown so the mission can decide if it was a
        crash."""
        s = self.s
        self.last_impact_speed = 0.0
        if s[1] <= 0.0:
            if not self.on_ground:
                self.last_impact_speed = -s[3]
            s[1] = 0.0
            if s[3] < 0:
                s[3] = 0.0
            s[2] *= max(0.0, 1.0 - 6.0 * dt)       # skid friction
            s[4] *= max(0.0, 1.0 - 12.0 * dt)      # gear levels the frame
            s[5] *= max(0.0, 1.0 - 12.0 * dt)
            self.on_ground = True
        else:
            self.on_ground = False
