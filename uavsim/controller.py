"""Flight controller: cascaded PID loops + motor mixer.

    position loop  ->  tilt command  ->  attitude loop  ->  torque  -+
    altitude loop  ->  collective thrust ----------------------------+-> mixer -> rotor speeds

Modes (same idea as hobby flight controllers):
    ASSIST stick commands velocity, holds position when released (default)
    ANGLE  stick commands tilt angle, self-levels when released
    RATE   stick commands tilt rate, throttle is direct (hardest)
    AUTO   fly to a waypoint, holds position and altitude

Attitude gains come from pole placement on the rigid body I*theta'' = tau.
With a PD law tau = Kp*e - Kd*q the closed loop is a 2nd-order system:
    wn = sqrt(Kp / I),   zeta = Kd / (2 sqrt(Kp I))
so Kp = I wn^2 and Kd = 2 zeta wn I. The rotor lag (tau_m = 40 ms, a pole at
25 rad/s) sets the ceiling on wn: keep the attitude loop well below it.
"""

import math
from .params import G

ASSIST, RATE, ANGLE, AUTO = "ASSIST", "RATE", "ANGLE", "AUTO"
MODES = (ASSIST, ANGLE, RATE, AUTO)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, i_limit=math.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, dt, derivative=0.0, integrate=True):
        """`derivative` is passed in (derivative on measurement) to avoid
        a kick when the setpoint jumps. `integrate=False` freezes the
        integrator while the actuators are saturated (anti-windup)."""
        if integrate and self.ki:
            self.integral = clamp(self.integral + error * dt, -self.i_limit, self.i_limit)
        return self.kp * error + self.ki * self.integral - self.kd * derivative


class FlightController:
    MAX_TILT = math.radians(25)
    STICK_TILT = math.radians(25)    # ANGLE mode full-stick tilt
    STICK_RATE = math.radians(200)   # RATE mode full-stick rate
    V_MAX_XY = 2.5                   # m/s, AUTO cruise limits
    V_MAX_Z = 2.0
    STICK_VX = 3.0                   # m/s, ASSIST full-stick speeds
    STICK_VZ = 2.0
    STICK_ACCEL = 4.0                # m/s^2, ASSIST setpoint ramp

    def __init__(self, quad, wn_att=14.0, zeta_att=0.8):
        self.quad = quad
        self.mode = ASSIST
        self.hold = [None, None]      # ASSIST position hold (x, z)
        self.v_sp = [0.0, 0.0]        # ASSIST smoothed velocity setpoint
        I = quad.v.inertia
        self.wn_att, self.zeta_att = wn_att, zeta_att
        self.att = PID(kp=I * wn_att ** 2, ki=0.4, kd=2 * zeta_att * wn_att * I, i_limit=0.3)
        self.rate = PID(kp=I * 25.0, ki=0.0)
        # outer loops: proportional position -> velocity setpoint,
        # PI velocity -> acceleration setpoint
        self.pos_x_kp, self.pos_z_kp = 1.2, 1.5
        self.vel_x = PID(kp=2.8, ki=0.6, i_limit=3.0)
        self.vel_z = PID(kp=4.0, ki=1.5, i_limit=3.0)
        self.throttle = 0.0              # 0..1 collective for manual modes
        self.theta_cmd = 0.0
        self.target = None
        self.target_is_final = True   # False for pass-through waypoints
        self.saturated = False

    def set_mode(self, mode):
        if mode == self.mode:
            return
        q = self.quad
        # bumpless transfer: start manual throttle at the current collective
        if mode != AUTO:
            t_now = q.side_thrust(q.wl) + q.side_thrust(q.wr)
            self.throttle = clamp(t_now / (2 * q.v.side_thrust_max()), 0.0, 1.0)
        for pid in (self.att, self.vel_x, self.vel_z):
            pid.reset()
        self.hold = [None, None]
        self.v_sp = [0.0, 0.0]
        self.mode = mode

    def hover_throttle(self):
        q = self.quad
        return q.mass * G / (2 * q.v.side_thrust_max())

    def update(self, dt, stick=0.0, throttle_input=0.0):
        """stick, throttle_input in [-1, 1]. Returns rotor speed commands."""
        q = self.quad
        t_side_max = q.v.side_thrust_max(q.omega_max())

        if self.mode == AUTO and self.target is not None:
            collective, theta_cmd = self._position_loop(dt)
        elif self.mode == ASSIST:
            collective, theta_cmd = self._assist(dt, stick, throttle_input)
        else:
            self.throttle = clamp(self.throttle + 0.35 * throttle_input * dt, 0.0, 1.0)
            collective = self.throttle * 2 * q.v.side_thrust_max()
            theta_cmd = stick * self.STICK_TILT

        if self.mode == RATE:
            rate_cmd = stick * self.STICK_RATE
            torque = self.rate.update(rate_cmd - q.q, dt)
            self.theta_cmd = q.theta
        else:
            self.theta_cmd = clamp(theta_cmd, -self.MAX_TILT, self.MAX_TILT)
            err = self.theta_cmd - q.theta
            torque = self.att.update(err, dt, derivative=q.q, integrate=not self.saturated)

        return self.mix(collective, torque, t_side_max)

    def _assist(self, dt, stick, throttle_input):
        """Stick sets a velocity. When a stick is released the vehicle brakes,
        and once it has nearly stopped it locks that position, so it stays
        put in wind instead of drifting."""
        q = self.quad
        sp = []
        for axis, (cmd, vmax, pos, vel, kp) in enumerate((
                (stick, self.STICK_VX, q.x, q.vx, self.pos_x_kp),
                (throttle_input, self.STICK_VZ, q.z, q.vz, self.pos_z_kp))):
            if cmd:
                self.hold[axis] = None
                target = cmd * vmax
            else:
                if self.hold[axis] is None and abs(vel) < 0.3:
                    self.hold[axis] = pos
                h = self.hold[axis]
                target = 0.0 if h is None else clamp(kp * (h - pos), -vmax, vmax)
            # ramp the setpoint so key presses feel smooth instead of jerky
            step = self.STICK_ACCEL * dt
            self.v_sp[axis] += clamp(target - self.v_sp[axis], -step, step)
            sp.append(self.v_sp[axis])
        if q.on_ground and sp[1] <= 0:
            # sitting on the ground with no climb command: motors idle
            self.vel_x.reset()
            self.vel_z.reset()
            self.hold = [None, None]
            self.v_sp = [0.0, 0.0]
            return 0.0, 0.0
        return self._velocity_loop(dt, sp[0], sp[1])

    def _velocity_loop(self, dt, vx_sp, vz_sp):
        q = self.quad
        ax = self.vel_x.update(vx_sp - q.vx, dt, integrate=not self.saturated)
        az = self.vel_z.update(vz_sp - q.vz, dt, integrate=not self.saturated)
        # small-angle inversion of the translational dynamics:
        #   x'' = g tan(theta)  ->  theta = atan(ax / g)
        theta_cmd = clamp(math.atan2(ax, G), -self.MAX_TILT, self.MAX_TILT)
        # gravity feed-forward with tilt compensation
        collective = q.mass * (G + az) / max(math.cos(q.theta), 0.5)
        return collective, theta_cmd

    def _position_loop(self, dt):
        q = self.quad
        tx, tz = self.target
        if self.target_is_final:
            # P on position gives a velocity setpoint, clipped to cruise limits
            vx_sp = clamp(self.pos_x_kp * (tx - q.x), -self.V_MAX_XY, self.V_MAX_XY)
            vz_sp = clamp(self.pos_z_kp * (tz - q.z), -self.V_MAX_Z, self.V_MAX_Z)
        else:
            # pass-through waypoint: fly toward it at cruise speed without
            # braking, so a chain of waypoints is flown as one smooth path
            dx, dz = tx - q.x, tz - q.z
            d = math.hypot(dx, dz) or 1.0
            vx_sp = self.V_MAX_XY * dx / d
            vz_sp = clamp(self.V_MAX_XY * dz / d, -self.V_MAX_Z, self.V_MAX_Z)
        return self._velocity_loop(dt, vx_sp, vz_sp)

    def mix(self, collective, torque, t_side_max):
        """Split collective thrust and torque into left/right rotor pairs.

            T_l = T/2 + tau/(2l),   T_r = T/2 - tau/(2l)

        When a motor would saturate, attitude authority is kept and the
        collective is sacrificed - losing a bit of altitude is better than
        losing attitude control."""
        q = self.quad
        half_diff = torque / (2 * q.v.arm)
        half_diff = clamp(half_diff, -t_side_max / 2, t_side_max / 2)
        half_coll = collective / 2
        lo, hi = abs(half_diff), t_side_max - abs(half_diff)
        self.saturated = not (lo <= half_coll <= hi) or abs(torque / (2 * q.v.arm)) > t_side_max / 2
        half_coll = clamp(half_coll, lo, hi)
        tl, tr = half_coll + half_diff, half_coll - half_diff
        k = q.v.rotors_per_side * q.v.k_thrust
        return math.sqrt(max(tl, 0.0) / k), math.sqrt(max(tr, 0.0) / k)
