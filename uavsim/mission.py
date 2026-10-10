"""Levels, mission logic and the fixed-step simulation loop.

Everything here is independent of rendering so the same code runs in the
game, the headless analysis scripts and the tests.
"""

import math
from .params import Vehicle, Battery, Payload
from .battery import BatteryState
from .dynamics import Quadrotor2D
from .controller import FlightController, AUTO
from .wind import Wind

WORLD_W, WORLD_H = 18.0, 11.0   # m
PHYS_DT = 1.0 / 240.0            # s, physics step
CHECKPOINT_R = 0.75              # m
ZONE_R = 0.76                    # m, cargo pickup / drop zone
HOVER_SPEED = 0.5                # m/s, max speed counted as "hovering"
HOVER_TIME = 1.5                 # s
CRASH_VZ = 2.5                   # m/s, max touchdown sink rate
CRASH_TILT = math.radians(35)


# Gates are gaps in a wall: (x_left, x_right, gap_bottom, gap_top).
LEVELS = {
    1: dict(
        name="Basic Navigation",
        start=(9.0, 0.0),
        checkpoints=[(4.0, 3.0), (13.0, 7.4), (8.0, 5.0), (15.0, 2.6), (3.0, 7.0),
                     (10.0, 9.0), (14.0, 4.0), (6.0, 1.4), (11.0, 6.0), (8.4, 8.0)],
        gates=[],
        wind=dict(kind="calm"),
        wind_delay=0.0,
        soc_start=1.0,
    ),
    2: dict(
        name="Gate Run in Crosswind",
        start=(1.6, 0.0),
        checkpoints=[(2.8, 5.6), (8.0, 5.6), (8.0, 9.4), (12.6, 6.6), (16.8, 4.6)],
        gates=[(5.6, 6.1, 3.8, 7.4), (10.4, 10.9, 4.8, 8.4), (14.6, 15.1, 2.8, 6.4)],
        wind=dict(kind="steady", mean=(5.0, 0.0), gust_sigma=1.2, gust_tau=1.5),
        wind_delay=2.0,
        soc_start=1.0,
    ),
    3: dict(
        name="Cargo Delivery, Low Battery",
        start=(1.6, 0.0),
        checkpoints=[],
        gates=[(6.2, 6.7, 3.6, 6.8), (11.8, 12.3, 1.8, 5.2)],
        pickup=(2.4, 4.0),
        dropzone=(16.0, 9.0),
        wind=dict(kind="rotating", mean=(4.0, 1.5), gust_sigma=0.8, gust_tau=1.0),
        wind_delay=None,      # wind starts when the cargo is picked up
        soc_start=0.18,
    ),
}


def gate_walls(gates):
    """Each gate becomes two solid rectangles (x0, z0, x1, z1)."""
    walls = []
    for x0, x1, lo, hi in gates:
        walls.append((x0, -1.0, x1, lo))
        walls.append((x0, hi, x1, WORLD_H + 1.0))
    return walls


def circle_hits_rect(cx, cz, r, rect):
    x0, z0, x1, z1 = rect
    nx = min(max(cx, x0), x1)
    nz = min(max(cz, z0), z1)
    return (cx - nx) ** 2 + (cz - nz) ** 2 < r * r


class Mission:
    def __init__(self, level=1, vehicle=None, battery=None, seed=7):
        self.level = level
        self.cfg = LEVELS[level]
        self.vehicle = vehicle or Vehicle()
        spec = battery or Battery()
        self.batt = BatteryState(spec, soc=self.cfg["soc_start"])
        sx, sz = self.cfg["start"]
        self.quad = Quadrotor2D(self.vehicle, self.batt, sx, sz)
        self.fc = FlightController(self.quad)
        w = dict(self.cfg["wind"])
        self.wind = Wind(seed=seed + level, **w)
        self.wind.enabled = False
        self.walls = gate_walls(self.cfg["gates"])
        self.t = 0.0
        self.cp_idx = 0
        self.cargo = False
        self.delivered = False
        self.hover_t = 0.0
        self.crashed = False
        self.crash_reason = ""
        self.complete = False
        self.log = []
        self._path = []
        self._path_goal = None
        self.events = []

    # objective handling

    def objective(self):
        """Current goal point, or None when finished."""
        if self.complete:
            return None
        if self.level == 3:
            return self.cfg["dropzone"] if self.cargo else self.cfg["pickup"]
        cps = self.cfg["checkpoints"]
        return cps[self.cp_idx] if self.cp_idx < len(cps) else None

    def _plan(self, goal):
        """Waypoints through any gates between the vehicle and the goal.
        Each gate is approached and left level at the middle of its gap."""
        q = self.quad
        path = []
        gates = sorted(self.cfg["gates"], key=lambda g: g[0], reverse=goal[0] < q.x)
        for x0, x1, lo, hi in gates:
            xc = 0.5 * (x0 + x1)
            if min(q.x, goal[0]) < xc < max(q.x, goal[0]):
                zc = 0.5 * (lo + hi)
                d = 1.0 if goal[0] > q.x else -1.0
                path += [(xc - d * 1.2, zc), (xc + d * 1.2, zc)]
        path.append(goal)
        return path

    def _update_autopilot(self):
        goal = self.objective()
        if goal is None:
            self.fc.target = (self.quad.x, self.quad.z)
            return
        if goal != self._path_goal:
            self._path = self._plan(goal)
            self._path_goal = goal
        q = self.quad
        while len(self._path) > 1 and math.hypot(q.x - self._path[0][0], q.z - self._path[0][1]) < 0.6:
            self._path.pop(0)
        self.fc.target = self._path[0]
        self.fc.target_is_final = len(self._path) == 1

    # main step

    def step(self, frame_dt, stick=0.0, throttle=0.0):
        """Advance by one display frame (several physics steps)."""
        if self.crashed or self.complete:
            return
        n = max(1, round(frame_dt / PHYS_DT))
        for _ in range(n):
            self._physics_step(PHYS_DT, stick, throttle)
            if self.crashed or self.complete:
                break
        self._record()

    def _physics_step(self, dt, stick, throttle):
        self.t += dt
        delay = self.cfg["wind_delay"]
        if delay is not None and self.t >= delay and self.cfg["wind"]["kind"] != "calm":
            self.wind.enabled = True
        wind = self.wind.step(dt)

        if self.fc.mode == AUTO:
            self._update_autopilot()
        wl, wr = self.fc.update(dt, stick, throttle)
        self.quad.step(dt, wl, wr, wind)
        self._keep_in_world()
        self._check_crash()
        if not self.crashed:
            self._check_objectives(dt)

    def _keep_in_world(self):
        s, r = self.quad.s, self.vehicle.collision_radius
        if s[0] < r:
            s[0], s[2] = r, max(s[2], 0.0)
        elif s[0] > WORLD_W - r:
            s[0], s[2] = WORLD_W - r, min(s[2], 0.0)
        if s[1] > WORLD_H - r:
            s[1], s[3] = WORLD_H - r, -0.3 * abs(s[3])

    def _check_crash(self):
        q = self.quad
        if any(circle_hits_rect(q.x, q.z, self.vehicle.collision_radius, w) for w in self.walls):
            self._crash("hit a wall")
        elif q.last_impact_speed > CRASH_VZ:
            self._crash(f"hard landing ({q.last_impact_speed:.1f} m/s)")
        elif q.on_ground and abs(q.theta) > CRASH_TILT and q.last_impact_speed > 0:
            self._crash("landed tilted")

    def _crash(self, reason):
        if self.batt.empty:
            reason = "battery depleted, " + reason
        self.crashed, self.crash_reason = True, reason

    def _check_objectives(self, dt):
        q = self.quad
        if self.level in (1, 2):
            cps = self.cfg["checkpoints"]
            cx, cz = cps[self.cp_idx]
            if math.hypot(q.x - cx, q.z - cz) < CHECKPOINT_R:
                self.cp_idx += 1
                self.events.append("checkpoint")
                if self.cp_idx >= len(cps):
                    self.complete = True
            return

        zone = self.cfg["dropzone"] if self.cargo else self.cfg["pickup"]
        speed = math.hypot(q.vx, q.vz)
        if math.hypot(q.x - zone[0], q.z - zone[1]) < ZONE_R and speed < HOVER_SPEED:
            self.hover_t += dt
            if self.hover_t >= HOVER_TIME:
                self.hover_t = 0.0
                self.events.append("checkpoint")
                if not self.cargo:
                    self.cargo = True
                    self.quad.attach_payload(Payload())
                    self.wind.enabled = True
                else:
                    self.cargo = False
                    self.delivered = True
                    self.quad.attach_payload(None)
                    self.complete = True
        else:
            self.hover_t = max(0.0, self.hover_t - 2 * dt)

    def pop_events(self):
        ev, self.events = self.events, []
        return ev

    def _record(self):
        q, b = self.quad, self.batt
        self.log.append({
            "t": round(self.t, 4),
            "level": self.level,
            "mode": self.fc.mode,
            "x": round(q.x, 4), "z": round(q.z, 4),
            "vx": round(q.vx, 4), "vz": round(q.vz, 4),
            "theta_deg": round(math.degrees(q.theta), 3),
            "theta_cmd_deg": round(math.degrees(self.fc.theta_cmd), 3),
            "q_dps": round(math.degrees(q.q), 3),
            "ax": round(q.accel[0], 4), "az": round(q.accel[1], 4),
            "w_l": round(q.wl, 2), "w_r": round(q.wr, 2),
            "T_l": round(q.side_thrust(q.wl), 4), "T_r": round(q.side_thrust(q.wr), 4),
            "wind_x": round(self.wind.vx, 3), "wind_z": round(self.wind.vz, 3),
            "mass": round(q.mass, 3),
            "V_batt": round(b.voltage, 3), "I_batt": round(b.current, 3),
            "P_elec": round(b.power, 2), "soc": round(b.soc, 5),
            "E_used_Wh": round(b.energy_used_wh, 5),
        })
