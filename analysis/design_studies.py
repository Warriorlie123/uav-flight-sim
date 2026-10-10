"""Engineering studies run headless on the simulator model.

    python -m analysis.design_studies          (from the repo root)

Writes figures to docs/img/ and prints the numbers quoted in the README:
  1. integrator verification against an analytic solution
  2. attitude step response vs. the 2nd-order design target
  3. altitude step response of the autopilot
  4. hover endurance and thrust-to-weight vs. payload
  5. available thrust vs. battery state of charge
"""

import math
import os

import matplotlib.pyplot as plt

from analysis import plotstyle as ps
from uavsim import Vehicle, Battery, Payload, BatteryState, Quadrotor2D, FlightController, G, ANGLE, AUTO
from uavsim.params import RHO_AIR, combined_mass_properties

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "img")


def step_metrics(t, y, y0, y1, band=0.02):
    """Rise time (10-90 %), percent overshoot and settling time (band)."""
    span = y1 - y0
    n = [(yi - y0) / span for yi in y]
    t10 = next(ti for ti, v in zip(t, n) if v >= 0.1)
    t90 = next(ti for ti, v in zip(t, n) if v >= 0.9)
    overshoot = max(0.0, (max(n) - 1.0) * 100)
    settle = 0.0
    for ti, v in zip(t, n):
        if abs(v - 1.0) > band:
            settle = ti
    return t90 - t10, overshoot, settle


def second_order(t, wn, zeta):
    wd = wn * math.sqrt(1 - zeta ** 2)
    return [1 - math.exp(-zeta * wn * ti) * (math.cos(wd * ti) + zeta / math.sqrt(1 - zeta ** 2) * math.sin(wd * ti))
            for ti in t]


def hovering_quad(z=5.0, payload=None, soc=1.0):
    v = Vehicle()
    quad = Quadrotor2D(v, BatteryState(Battery(), soc=soc), 9.0, z)
    quad.attach_payload(payload)
    w_hover = math.sqrt(quad.mass * G / (2 * v.rotors_per_side * v.k_thrust))
    quad.s[6] = quad.s[7] = w_hover
    return quad


# 1. Integrator verification

def integrator_study():
    """Motors off, falling with quadratic drag. Exact solution:
        vz(t) = -vt * tanh(g t / vt),   vt = sqrt(2 m g / (rho CdA))"""
    v = Vehicle()
    vt = math.sqrt(2 * v.mass * G / (RHO_AIR * v.cda_z))
    T = 3.0
    exact = -vt * math.tanh(G * T / vt)
    dts = [1 / 15, 1 / 30, 1 / 60, 1 / 120, 1 / 240, 1 / 480]
    err_rk4, err_euler = [], []
    for dt in dts:
        n = round(T / dt)
        q = Quadrotor2D(v, BatteryState(Battery()), 0.0, 1e4)
        for _ in range(n):
            q.step(dt, 0.0, 0.0)
        err_rk4.append(abs(q.vz - exact))
        q = Quadrotor2D(v, BatteryState(Battery()), 0.0, 1e4)
        s = q.s
        for _ in range(n):
            d = q.derivatives(s, (0.0, 0.0), (0.0, 0.0))
            s = [a + dt * b for a, b in zip(s, d)]
        err_euler.append(abs(s[3] - exact))

    order = math.log(err_rk4[2] / err_rk4[3]) / math.log(2)
    print(f"[integrator] terminal velocity {vt:.2f} m/s, RK4 observed order {order:.2f}, "
          f"error at dt=1/240 s: {err_rk4[4]:.2e} m/s")

    fig, ax = plt.subplots(figsize=(6.2, 4.0))
    ax.loglog(dts, err_euler, "o-", color=ORANGE, label="Forward Euler (1st order)")
    ax.loglog(dts, err_rk4, "o-", color=BLUE, label="RK4, used in the sim (4th order)")
    ref = [err_rk4[0] * (dt / dts[0]) ** 4 for dt in dts]
    ax.loglog(dts, ref, "--", color=ps.INK2, lw=1, label="slope 4 reference")
    ax.axvline(1 / 240, color=ps.GRID, lw=6, zorder=0)
    ax.text(1 / 240, ax.get_ylim()[1], " sim step\n 1/240 s", va="top", fontsize=8, color=ps.INK2)
    ax.minorticks_off()
    ax.set_xticks(dts)
    ax.set_xticklabels(["1/15", "1/30", "1/60", "1/120", "1/240", "1/480"])
    ax.set_xlabel("time step  dt  [s]")
    ax.set_ylabel("|vz error| at t = 3 s  [m/s]")
    ax.set_title("Integrator verification: free fall with quadratic drag")
    ax.legend(loc="lower right")
    ps.save(fig, os.path.join(OUT, "integrator_convergence.png"))


# 2. Attitude step

def attitude_study():
    dt, T, step_deg = 1 / 240, 1.2, 15.0
    t = [i * dt for i in range(int(T / dt))]
    results = {}
    for label, payload in (("no payload", None), ("0.6 kg payload", Payload())):
        q = hovering_quad(payload=payload)
        fc = FlightController(q)
        fc.set_mode(ANGLE)
        fc.throttle = q.mass * G / (2 * q.v.side_thrust_max())
        fc.set_mode(ANGLE)
        th = []
        stick = step_deg / math.degrees(fc.STICK_TILT)
        for _ in t:
            wl, wr = fc.update(dt, stick=stick)
            q.step(dt, wl, wr)
            th.append(math.degrees(q.theta))
        results[label] = th

    fc = FlightController(hovering_quad())
    ideal = [step_deg * y for y in second_order(t, fc.wn_att, fc.zeta_att)]
    rows = [("design target", ideal)] + list(results.items())
    print("[attitude] 15 deg step:")
    for name, y in rows:
        tr, os_, ts = step_metrics(t, y, 0.0, step_deg)
        print(f"   {name:15s} rise {tr * 1000:5.0f} ms  overshoot {os_:4.1f} %  settling(2%) {ts * 1000:5.0f} ms")

    fig, ax = plt.subplots(figsize=(6.6, 4.0))
    ax.plot(t, ideal, "--", color=ps.INK2, lw=1.4, label="my design")
    ax.plot(t, results["no payload"], color=BLUE, label="simulated drone")
    ax.plot(t, results["0.6 kg payload"], color=ORANGE, label="simulated, with cargo")
    ax.axhspan(step_deg * 0.98, step_deg * 1.02, color=ps.GRID, zorder=0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("tilt angle [deg]")
    ax.set_title("Tilt response to a 15 deg command")
    ax.legend(loc="lower right")
    ps.save(fig, os.path.join(OUT, "step_attitude.png"))


# 3. Altitude step

def altitude_study():
    dt, T = 1 / 240, 6.0
    t = [i * dt for i in range(int(T / dt))]
    out = {}
    for label, payload in (("no payload", None), ("0.6 kg payload", Payload())):
        q = hovering_quad(z=2.0, payload=payload)
        fc = FlightController(q)
        fc.set_mode(AUTO)
        fc.target = (9.0, 5.0)
        z, p = [], []
        for _ in t:
            wl, wr = fc.update(dt)
            q.step(dt, wl, wr)
            z.append(q.z)
            p.append(q.batt.power)
        out[label] = (z, p)
        tr, os_, ts = step_metrics(t, z, 2.0, 5.0)
        print(f"[altitude] {label:15s} rise {tr:4.2f} s  overshoot {os_:4.1f} %  settling(2%) {ts:4.2f} s  "
              f"peak power {max(p):.0f} W")

    fig, ax = plt.subplots(figsize=(6.6, 4.0))
    ax.plot(t, out["no payload"][0], color=BLUE, label="1.2 kg")
    ax.plot(t, out["0.6 kg payload"][0], color=ORANGE, label="1.8 kg (payload)")
    ax.axhspan(5.0 - 0.06, 5.0 + 0.06, color=ps.GRID, zorder=0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("altitude [m]")
    ax.set_title("Autopilot: 2 m to 5 m altitude step (climb rate limited to 2.5 m/s)")
    ax.legend(loc="lower right")
    ps.save(fig, os.path.join(OUT, "step_altitude.png"))


# 4. Payload trade study

def hover_power(total_mass, v: Vehicle):
    n_rot = 2 * v.rotors_per_side
    t_rot = total_mass * G / n_rot
    p_ideal = t_rot ** 1.5 / math.sqrt(2 * RHO_AIR * v.disk_area)
    return n_rot * p_ideal / (v.figure_of_merit * v.drivetrain_eff) + v.avionics_power


def payload_study():
    v, b = Vehicle(), Battery()
    usable = 0.8 * b.energy_wh
    payloads = [i * 0.02 for i in range(0, 101)]
    endurance = [usable / hover_power(v.mass + p, v) * 60 for p in payloads]
    tw = [2 * v.side_thrust_max() / ((v.mass + p) * G) for p in payloads]

    p_max_15 = 2 * v.side_thrust_max() / (1.5 * G) - v.mass
    print(f"[payload] hover power empty {hover_power(v.mass, v):.0f} W, endurance {endurance[0]:.1f} min; "
          f"with 0.6 kg {hover_power(v.mass + 0.6, v):.0f} W, {usable / hover_power(v.mass + 0.6, v) * 60:.1f} min; "
          f"T/W at 0.6 kg {2 * v.side_thrust_max() / (1.8 * G):.2f}; payload for T/W = 1.5: {p_max_15:.2f} kg")

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8))
    a1.plot(payloads, endurance, color=BLUE)
    a1.set_xlabel("payload [kg]")
    a1.set_ylabel("hover endurance [min]  (80 % of pack)")
    a1.set_title("Hover endurance vs payload")
    a1.plot([0.6], [usable / hover_power(1.8, v) * 60], "o", color=ORANGE, ms=8)
    a1.annotate("level 3 cargo", (0.6, usable / hover_power(1.8, v) * 60), xytext=(10, 10),
                textcoords="offset points", fontsize=8, color=ps.INK2)

    a2.plot(payloads, tw, color=BLUE)
    a2.axhline(1.5, color=ps.INK2, lw=1, ls="--")
    a2.text(2.0, 1.52, "T/W = 1.5, usual minimum for agile control", ha="right", fontsize=8, color=ps.INK2)
    a2.axhline(1.0, color=ps.INK2, lw=1, ls=":")
    a2.text(2.0, 1.02, "T/W = 1, cannot hover", ha="right", fontsize=8, color=ps.INK2)
    a2.plot([0.6], [2 * v.side_thrust_max() / (1.8 * G)], "o", color=ORANGE, ms=8)
    a2.set_xlabel("payload [kg]")
    a2.set_ylabel("thrust-to-weight ratio [-]")
    a2.set_title("Control margin vs payload")
    fig.tight_layout()
    ps.save(fig, os.path.join(OUT, "payload_trade.png"))


# 5. Battery sag

def max_thrust_at(soc, v: Vehicle):
    """Full-throttle thrust, solved self-consistently: more thrust draws more
    power, which sags the voltage, which lowers the rotor speed ceiling."""
    batt = BatteryState(Battery(), soc=soc)
    w = v.omega_max_nominal
    for _ in range(50):
        t_rot = v.k_thrust * w * w
        p = 4 * (t_rot ** 1.5 / math.sqrt(2 * RHO_AIR * v.disk_area)) / (v.figure_of_merit * v.drivetrain_eff)
        volt = batt.terminal_voltage(p + v.avionics_power)
        w = v.omega_max_nominal * volt / batt.spec.nominal_voltage
    return 4 * v.k_thrust * w * w


def battery_study():
    v = Vehicle()
    socs = [i / 100 for i in range(0, 101)]
    tmax = [max_thrust_at(s, v) for s in socs]
    w_empty, w_loaded = v.mass * G, (v.mass + Payload().mass) * G
    cross = next(s for s, tm in zip(socs, tmax) if tm > 1.3 * w_loaded)
    print(f"[battery] max thrust at 100 % {tmax[-1]:.1f} N, at 18 % {max_thrust_at(0.18, v):.1f} N, "
          f"at 5 % {max_thrust_at(0.05, v):.1f} N; loaded T/W falls below 1.3 at SoC {cross * 100:.0f} %")

    fig, ax = plt.subplots(figsize=(6.6, 4.0))
    ax.plot([s * 100 for s in socs], tmax, color=BLUE, label="max available thrust")
    ax.axhline(w_loaded, color=ORANGE, lw=1.4, ls="--", label="weight with 0.6 kg payload")
    ax.axhline(w_empty, color=AQUA, lw=1.4, ls="--", label="weight, no payload")
    ax.axvline(18, color=ps.GRID, lw=6, zorder=0)
    ax.text(18.8, ax.get_ylim()[0] + 1, "level 3 start", fontsize=8, color=ps.INK2)
    ax.set_xlabel("state of charge [%]")
    ax.set_ylabel("thrust [N]")
    ax.set_title("Voltage sag eats the thrust margin")
    ax.legend(loc="center right")
    ps.save(fig, os.path.join(OUT, "battery_thrust_margin.png"))


BLUE, ORANGE, AQUA = ps.BLUE, ps.ORANGE, ps.AQUA

if __name__ == "__main__":
    ps.apply()
    os.makedirs(OUT, exist_ok=True)
    integrator_study()
    attitude_study()
    altitude_study()
    payload_study()
    battery_study()
