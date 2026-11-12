"""Output-referred DM-J6006-2EC approximation, not identified firmware.

Kt = 4 Nm / 4 A = 1 Nm/A. Ke = 24 / (240*2*pi/60) =
0.95492966 V s/rad. R = (24 - Ke*150*2*pi/60)/4 = 2.25 ohm.
The rated point lies on this voltage line. These rounded catalogue points
give Kt != Ke; this is an empirical fit, not an energy-exact DC machine.
At 24 V stall, voltage limits torque/current to 10.6667 Nm/A, below the
12 Nm / 13 A ceilings. R is lumped, not measured phase resistance.

Assumed PI gains: Kp=1 Nm/(rad/s), Ki=2.040816 Nm/rad, sampled at 1 kHz.
For free-wheel J=0.0245 kg m2 and b=0.05 Nm s/rad (scene inertia plus
armature), Ki/Kp=b/J cancels the plant pole; 2% settling ~96 ms.
Instantaneous current control, conditional integration anti-windup.
Power = (torque/Kt)^2 R + torque*omega; ignores iron, gearbox, inverter,
idle and thermal losses. Negative net power is dumped locally (no regen).
"""

from dataclasses import dataclass
import math
import numpy as np

KT = 1.0
KE = 24.0 / (240 * 2 * math.pi / 60)
RESISTANCE = 2.25
PEAK_TORQUE = 12.0
PEAK_CURRENT = 13.0
FREE_INERTIA = 0.0245
FREE_DAMPING = 0.05


def torque_limit(omega: float, voltage: float) -> float:
    """Symmetric available torque envelope; no field weakening."""
    return min(PEAK_TORQUE, KT * PEAK_CURRENT,
               KT * max(0.0, voltage - KE * abs(omega)) / RESISTANCE)


def electrical(torque: float, omega: float, voltage: float) -> tuple[float, float, float]:
    """Return signed winding current, nonregenerating input power and bus current."""
    current = torque / KT
    power = max(0.0, current * current * RESISTANCE + torque * omega)
    return current, power, power / voltage


@dataclass
class Motor:
    kp: float = 1.0
    ki: float = FREE_DAMPING / FREE_INERTIA
    integral: float = 0.0
    target: float = 0.0

    def demand(self, omega: float) -> float:
        return self.kp * (self.target - omega) + self.integral

    def accept(self, omega: float, torque: float, dt: float = 0.001) -> None:
        error = self.target - omega
        requested = self.demand(omega)
        if abs(torque - requested) < 1e-10 or error * (requested - torque) <= 0:
            self.integral += self.ki * error * dt


PHASE_R = .33
PHASE_L = 305e-6
POLE_PAIRS = 14
GEAR = 6
FLUX = 24 / math.sqrt(3) / (POLE_PAIRS * GEAR * 240 * 2 * math.pi / 60)
IDEAL_KT = 1.5 * POLE_PAIRS * GEAR * FLUX


@dataclass(frozen=True)
class DatasheetMotor:
    """Id=0 PMSM envelope; current is phase amplitude in the voltage equation.

    Catalogue Kt and this amplitude convention are not energy reciprocal.
    See docs/FINDINGS.md, section 3, for the loss comparison.
    """

    kt: float = KT
    current_limit: float = 9.8
    power_bound: str = "copper"

    def limit(self, omega, voltage):
        we = POLE_PAIRS * GEAR * np.abs(omega)
        a = PHASE_R**2 + (we * PHASE_L)**2
        b = 2 * PHASE_R * we * FLUX
        c = (we * FLUX)**2 - (voltage / math.sqrt(3))**2
        # Stable positive quadratic root, zero above the no-load speed.
        root = np.maximum(0, -2*c / (b + np.sqrt(np.maximum(0, b*b-4*a*c))))
        return self.kt * np.minimum(self.current_limit, root)

    def power(self, torque, omega):
        current = np.asarray(torque) / self.kt
        loss = RESISTANCE if self.power_bound == "lumped" else 1.5 * PHASE_R
        return np.maximum(0, loss * current**2 + np.asarray(torque) * omega)


@dataclass
class ProtectedMotor(Motor):
    """Explicit enable latch; ordinary velocity commands never clear a fault."""

    behaviour: str = "limit"
    current_setting: float = 9.8
    kt: float = KT
    timeout: float | None = None
    last_command: float = 0.0
    error: int = 1

    def enable(self, now):
        self.error, self.integral, self.target = 1, 0.0, 0.0
        self.last_command = now

    def command(self, target, now):
        self.target, self.last_command = target, now

    def check(self, omega, voltage, now):
        if self.error != 1:
            return 0.0
        if voltage < 15:
            self.error = 9
        elif self.timeout is not None and now - self.last_command > self.timeout + 1e-12:
            self.error = 13
        elif self.behaviour == "trip" and abs(self.demand(omega)) / self.kt > self.current_setting:
            self.error = 10
        return self.demand(omega) if self.error == 1 else 0.0

    def accept(self, omega, torque, dt=.001):
        if self.error == 1:
            super().accept(omega, torque, dt)
