import math
import numpy as np
import pytest
from sim.motor import (Motor, KT, KE, RESISTANCE, FREE_INERTIA, FREE_DAMPING,
                       torque_limit, electrical)
from sim.power import Battery


def test_catalogue_fit():
    assert KT == 1
    assert KE == pytest.approx(0.9549296586)
    assert RESISTANCE == 2.25
    rated_speed = 150 * 2 * math.pi / 60
    assert torque_limit(rated_speed, 24) == pytest.approx(4)
    assert electrical(4, rated_speed, 24)[0] == 4
    assert torque_limit(0, 24) == pytest.approx(10.6666666667)
    assert electrical(torque_limit(0, 24), 0, 24)[0] == pytest.approx(10.6666666667)
    assert torque_limit(0, 48) == 12
    assert torque_limit(100, 24) == 0


def test_free_wheel_no_load_speed():
    speed = 0.0
    for _ in range(1500):
        speed += torque_limit(speed, 24) / FREE_INERTIA * .001
    assert speed * 60 / (2 * math.pi) == pytest.approx(240, rel=.02)


def test_pi_step_settles_about_100_ms():
    motor = Motor(target=10)
    speed, history = 0., []
    for _ in range(500):
        limit = torque_limit(speed, 24)
        torque = float(np.clip(motor.demand(speed), -limit, limit))
        motor.accept(speed, torque)
        speed += (torque - FREE_DAMPING * speed) / FREE_INERTIA * .001
        history.append(speed)
    last_outside = max(i for i, w in enumerate(history) if abs(w-10) > .2)
    settling = (last_outside + 1) * .001
    assert .075 <= settling <= .125
    assert speed == pytest.approx(10, abs=.01)


def test_shared_bus_stall_solution_and_sag():
    pack = Battery()
    voltage, current, torque = pack.solve([100]*6, [0]*6)
    expected = 25.2 / (1 + 6*.04/2.25)
    assert voltage == pytest.approx(expected, abs=1e-6)
    assert torque == pytest.approx([voltage/2.25]*6)
    assert current == pytest.approx(6*voltage/2.25)
    assert voltage == pytest.approx(pack.open_circuit - current*pack.resistance, abs=1e-6)
    assert pack.minimum_voltage == voltage
    assert not pack.undervoltage
    low_pack = Battery(open_circuit=21)
    low, _, _ = low_pack.solve([100]*6, [0]*6)
    assert low < 20 and low_pack.undervoltage


def test_power_and_no_regeneration():
    winding, power, bus = electrical(4, 150*2*math.pi/60, 24)
    assert power == pytest.approx(36 + 4*150*2*math.pi/60)
    assert bus == pytest.approx(power/24)
    assert winding == 4
    assert electrical(-1, 10, 24)[1:] == (0, 0)


def test_bus_at_moving_operating_points():
    for speed in (0, 5, 15, 24, 30):
        for demand in (-12, 0, 4, 12):
            battery = Battery()
            v, i, torques = battery.solve([demand]*6, [speed]*6)
            assert v == pytest.approx(25.2 - .04*i, abs=1e-6)
            assert all(abs(t) <= torque_limit(speed, v) + 1e-9 for t in torques)
