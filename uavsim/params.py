"""Vehicle, battery and environment parameters (SI units throughout).

The airframe is loosely based on a 450-size quadcopter with 10" props and a
4S LiPo. The simulation is planar (x-z plane, viewed from behind), so the
four rotors are lumped into a left pair and a right pair.
"""

from dataclasses import dataclass, field
import math

G = 9.81          # m/s^2
RHO_AIR = 1.225   # kg/m^3, ISA sea level


@dataclass
class Vehicle:
    mass: float = 1.20            # kg, airframe + battery
    arm: float = 0.20             # m, CG to rotor-pair thrust line
    inertia: float = 0.0125       # kg m^2, about the pitch/roll axis
    rotors_per_side: int = 2
    prop_diameter: float = 0.254  # m (10 in)
    k_thrust: float = 6.0e-6      # N/(rad/s)^2, static thrust coefficient
    omega_max_nominal: float = 1000.0  # rad/s at nominal pack voltage
    motor_tau: float = 0.04       # s, first-order motor/ESC time constant
    cda_x: float = 0.10           # m^2, side drag area (Cd * A)
    cda_z: float = 0.15           # m^2, vertical drag area
    rot_damping: float = 0.010    # N m s/rad, aerodynamic rotational damping
    figure_of_merit: float = 0.62
    drivetrain_eff: float = 0.80  # motor * ESC efficiency
    avionics_power: float = 5.0   # W, flight controller, receiver, etc.
    collision_radius: float = 0.30  # m

    @property
    def disk_area(self):
        return math.pi * (self.prop_diameter / 2) ** 2

    def side_thrust_max(self, omega_max=None):
        w = self.omega_max_nominal if omega_max is None else omega_max
        return self.rotors_per_side * self.k_thrust * w ** 2

    @property
    def thrust_to_weight(self):
        return 2 * self.side_thrust_max() / (self.mass * G)


@dataclass
class Payload:
    mass: float = 0.60     # kg
    offset: float = 0.12   # m below the airframe CG, rigidly mounted


def combined_mass_properties(v: Vehicle, payload: Payload | None):
    """Mass and inertia about the combined CG (parallel-axis theorem)."""
    if payload is None:
        return v.mass, v.inertia
    m = v.mass + payload.mass
    # the combined CG moves toward the payload; reduced-mass form of the
    # parallel axis theorem gives the inertia about the new CG directly
    inertia = v.inertia + (v.mass * payload.mass / m) * payload.offset ** 2
    return m, inertia


# Open-circuit voltage per LiPo cell vs state of charge (typical discharge
# curve at low C-rate, rested). The steep knee below ~10 % is what makes
# the last part of a flight dangerous.
LIPO_OCV = [
    (0.00, 3.27), (0.05, 3.50), (0.10, 3.68), (0.20, 3.74), (0.30, 3.77),
    (0.40, 3.80), (0.50, 3.84), (0.60, 3.87), (0.70, 3.92), (0.80, 3.98),
    (0.90, 4.06), (1.00, 4.20),
]


@dataclass
class Battery:
    cells: int = 4
    capacity_ah: float = 2.2
    r_internal: float = 0.045     # ohm, whole pack
    nominal_voltage: float = 14.8
    soc_start: float = 1.0
    ocv_table: list = field(default_factory=lambda: list(LIPO_OCV))

    @property
    def energy_wh(self):
        return self.capacity_ah * self.nominal_voltage
