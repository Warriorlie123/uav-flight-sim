"""LiPo pack model: open-circuit voltage curve + internal resistance."""

import math
from .params import Battery


def interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


class BatteryState:
    def __init__(self, spec: Battery, soc=None):
        self.spec = spec
        self.soc = spec.soc_start if soc is None else soc
        self.voltage = self.open_circuit_voltage()
        self.current = 0.0
        self.power = 0.0
        self.energy_used_wh = 0.0

    def open_circuit_voltage(self):
        return self.spec.cells * interp(self.spec.ocv_table, max(self.soc, 0.0))

    @property
    def empty(self):
        return self.soc <= 0.0

    def terminal_voltage(self, power):
        """Solve P = V*I with V = Voc - I*R for the terminal voltage.

        V^2 - Voc*V + P*R = 0  ->  take the high-voltage root. If the load
        exceeds what the pack can deliver (discriminant < 0) the pack is at
        its maximum power point, V = Voc/2.
        """
        voc = self.open_circuit_voltage()
        disc = voc * voc - 4.0 * power * self.spec.r_internal
        if disc <= 0:
            return voc / 2
        return 0.5 * (voc + math.sqrt(disc))

    def draw(self, power, dt):
        """Discharge the pack at electrical power `power` (W) for dt seconds."""
        if self.empty:
            self.voltage, self.current, self.power = 0.0, 0.0, 0.0
            return
        v = self.terminal_voltage(power)
        i = power / v if v > 0 else 0.0
        self.voltage, self.current, self.power = v, i, power
        self.soc -= i * dt / (3600.0 * self.spec.capacity_ah)
        self.energy_used_wh += power * dt / 3600.0
        if self.soc < 0:
            self.soc = 0.0
