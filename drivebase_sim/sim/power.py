"""Shared 6S LiPo bus: assumed 40 milliohm pack/wiring, no SOC dynamics."""

from dataclasses import dataclass
import numpy as np
from .motor import KE, KT, RESISTANCE, PEAK_TORQUE, PEAK_CURRENT


@dataclass
class Battery:
    open_circuit: float = 25.2
    resistance: float = 0.040
    undervoltage_threshold: float = 20.0
    minimum_voltage: float = float("inf")
    undervoltage: bool = False

    def solve(self, demands, speeds):
        """Simultaneously solve sag and voltage-dependent torque each 1 ms tick.

        The high-voltage operating branch is bracketed by Voc/2 and Voc.
        With six <=12 Nm motors and this pack, all allowed operating points
        remain on that branch. Raise on collapse rather than invent voltage.
        UV is flagged only: actual disable/re-enable firmware is not modeled.
        """
        demands, speeds = np.asarray(demands), np.asarray(speeds)
        ceiling = np.minimum(PEAK_TORQUE, KT * PEAK_CURRENT)

        def at(voltage):
            limit = np.minimum(ceiling, KT * np.maximum(0, voltage - KE * np.abs(speeds)) / RESISTANCE)
            torque = np.clip(demands, -limit, limit)
            power = np.maximum(0, (torque / KT) ** 2 * RESISTANCE + torque * speeds)
            bus_current = power.sum() / voltage
            return voltage + self.resistance * bus_current - self.open_circuit, torque, bus_current

        lo, hi = self.open_circuit / 2, self.open_circuit
        if at(lo)[0] > 0:
            raise RuntimeError("Battery operating point collapsed below Voc/2")
        for _ in range(28):
            mid = (lo + hi) / 2
            if at(mid)[0] > 0:
                hi = mid
            else:
                lo = mid
        voltage = (lo + hi) / 2
        _, torque, current = at(voltage)
        self.minimum_voltage = min(self.minimum_voltage, voltage)
        self.undervoltage |= voltage < self.undervoltage_threshold
        return voltage, current, torque


@dataclass
class DatasheetBattery(Battery):
    """Same pack as R1; PMSM loss bracket and 15 V protection threshold."""

    undervoltage_threshold: float = 15.0

    def solve(self, demands, speeds, motor):
        demands, speeds = np.asarray(demands), np.asarray(speeds)

        def at(voltage):
            limit = motor.limit(speeds, voltage)
            torque = np.clip(demands, -limit, limit)
            current = motor.power(torque, speeds).sum() / voltage
            return voltage + self.resistance * current - self.open_circuit, torque, current

        # Current-limited operation has constant power: solve it directly.
        torque = np.clip(demands, -motor.kt*motor.current_limit, motor.kt*motor.current_limit)
        power = motor.power(torque, speeds).sum()
        disc = self.open_circuit**2 - 4*self.resistance*power
        voltage = (self.open_circuit + np.sqrt(max(0, disc))) / 2
        if disc < 0 or np.any(np.abs(torque) > motor.limit(speeds, voltage) + 1e-10):
            lo, hi = self.open_circuit/2, self.open_circuit
            if at(lo)[0] > 0:
                raise RuntimeError("Battery operating point collapsed below Voc/2")
            for _ in range(28):
                mid = (lo+hi)/2
                if at(mid)[0] > 0:
                    hi = mid
                else:
                    lo = mid
            voltage = (lo+hi)/2
        _, torque, current = at(voltage)
        self.minimum_voltage = min(self.minimum_voltage, voltage)
        self.undervoltage |= voltage < self.undervoltage_threshold
        return float(voltage), float(current), torque
