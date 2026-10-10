"""Model checks against hand calculations.  Run with:  python -m pytest"""

import math

import pytest

from uavsim import Vehicle, Battery, Payload, BatteryState, Quadrotor2D, FlightController, Mission, G, AUTO
from uavsim.params import RHO_AIR, combined_mass_properties


def make_quad(z=50.0):
    return Quadrotor2D(Vehicle(), BatteryState(Battery()), 0.0, z)


def test_hover_equilibrium():
    """At w = sqrt(m g / (4 kT)) thrust equals weight: no vertical drift."""
    q = make_quad()
    v = q.v
    w = math.sqrt(q.mass * G / (2 * v.rotors_per_side * v.k_thrust))
    q.s[6] = q.s[7] = w
    for _ in range(240):
        q.step(1 / 240, w, w)
    assert abs(q.vz) < 1e-6
    assert abs(q.theta) < 1e-9


def test_free_fall_matches_analytic_drag_solution():
    q = make_quad(z=1e4)
    v = q.v
    vt = math.sqrt(2 * v.mass * G / (RHO_AIR * v.cda_z))
    for _ in range(2 * 240):
        q.step(1 / 240, 0.0, 0.0)
    assert q.vz == pytest.approx(-vt * math.tanh(G * 2.0 / vt), abs=1e-6)


def test_differential_thrust_sign():
    """More thrust on the left must roll right (positive theta) and push +x."""
    q = make_quad()
    v = q.v
    w = math.sqrt(q.mass * G / (2 * v.rotors_per_side * v.k_thrust))
    q.s[6], q.s[7] = w * 1.05, w * 0.95
    for _ in range(60):
        q.step(1 / 240, w * 1.05, w * 0.95)
    assert q.theta > 0 and q.vx > 0


def test_parallel_axis():
    v, p = Vehicle(), Payload()
    m, inertia = combined_mass_properties(v, p)
    cg = p.mass * p.offset / m
    expected = v.inertia + v.mass * cg ** 2 + p.mass * (p.offset - cg) ** 2
    assert m == pytest.approx(v.mass + p.mass)
    assert inertia == pytest.approx(expected)


def test_battery_terminal_voltage_satisfies_ohms_law():
    b = BatteryState(Battery(), soc=0.5)
    for p in (50.0, 200.0, 600.0):
        vt = b.terminal_voltage(p)
        i = p / vt
        assert vt == pytest.approx(b.open_circuit_voltage() - i * b.spec.r_internal)


def test_mixer_keeps_attitude_authority_when_saturated():
    q = make_quad()
    fc = FlightController(q)
    t_max = q.v.side_thrust_max()
    wl, wr = fc.mix(collective=2 * t_max, torque=0.5, t_side_max=t_max)
    tl, tr = q.side_thrust(wl), q.side_thrust(wr)
    assert tl <= t_max + 1e-9
    assert q.v.arm * (tl - tr) == pytest.approx(0.5)
    assert fc.saturated


@pytest.mark.parametrize("level", [1, 2, 3])
def test_autopilot_completes_every_level(level):
    m = Mission(level)
    m.fc.set_mode(AUTO)
    while not (m.complete or m.crashed) and m.t < 90:
        m.step(1 / 60)
    assert m.complete, m.crash_reason
