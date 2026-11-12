"""Datasheet, protection, virtual-work equilibrium and feedback governor checks."""

import math
import numpy as np
import pytest
from sim.motor import DatasheetMotor, ProtectedMotor, FLUX, IDEAL_KT, PHASE_R, PHASE_L
from sim.can_dm import CanBus, encode_velocity, encode_feedback, decode_feedback
from sim.power import DatasheetBattery
from sim.drive import TorqueGovernor
from sim.rocker_bogie import solve_loads, predict_loads


def test_datasheet_no_load_stall_and_voltage_equation():
    motor = DatasheetMotor()
    w0 = 240 * 2*math.pi/60
    assert motor.limit(w0, 24) == pytest.approx(0, abs=1e-12)
    assert motor.limit(w0*.999, 24) > 0
    assert motor.limit(0, 24) == pytest.approx(9.8)
    assert DatasheetMotor(kt=IDEAL_KT, current_limit=13).limit(0, 24) == pytest.approx(IDEAL_KT*13)
    for w in (15, 20, 24):
        current = motor.limit(w, 24)/motor.kt
        voltage_sq = (PHASE_R*current+84*w*FLUX)**2 + (84*w*PHASE_L*current)**2
        assert voltage_sq <= 24**2/3 + 1e-9
        if current < 9.8:
            assert voltage_sq == pytest.approx(24**2/3)


def test_envelope_monotonic_speed_voltage_and_power_bounds():
    speeds = np.linspace(0, 35, 300)
    motor = DatasheetMotor()
    previous = np.zeros_like(speeds)
    for voltage in (15, 22.2, 24, 25.2, 48):
        limit = motor.limit(speeds, voltage)
        assert np.all(np.diff(limit) <= 1e-10)
        assert np.all(limit >= previous-1e-10)
        previous = limit
    assert motor.power(9.8, 0) == pytest.approx(1.5*.33*9.8**2)
    assert DatasheetMotor(power_bound="lumped").power(9.8, 0) == pytest.approx(2.25*9.8**2)


def test_datasheet_unloaded_acceleration_converges_to_240_rpm():
    envelope = DatasheetMotor()
    speed = 0.
    for _ in range(1500):
        speed += float(envelope.limit(speed,24)) / .0245 * .001
    assert speed*60/(2*math.pi) == pytest.approx(240, rel=1e-6)


def test_datasheet_pack_voltage_limited_moving_points():
    envelope = DatasheetMotor()
    for speed in (0, 15, 20, 24, 30):
        for demand in (-12, 4, 12):
            pack = DatasheetBattery(open_circuit=22.2)
            v, current, torque = pack.solve([demand]*6, [speed]*6, envelope)
            assert v == pytest.approx(22.2-.04*current, abs=1e-6)
            assert np.all(np.abs(torque) <= envelope.limit(speed,v)+1e-9)


@pytest.mark.parametrize("fault", [9, 10, 13])
def test_fault_latches_and_reaches_actual_delayed_can_feedback(fault):
    motor = ProtectedMotor(behaviour="trip", timeout=.5)
    motor.command(20 if fault == 10 else 2, 0)
    assert motor.check(0, 14 if fault == 9 else 24, .6 if fault == 13 else .1) == 0
    assert motor.error == fault
    motor.command(0, 1)
    assert motor.check(0, 24, 1) == 0 and motor.error == fault
    bus = CanBus(motor_ids=[1])
    bus.send(encode_velocity(1, 0), 1)
    state = {1: (0, 0, 0, motor.error)}
    bus.advance(1.001, state)
    bus.advance(1.002, state)
    assert bus.feedback[1]["error"] == fault
    assert bus.feedback[1]["error_name"] == {9:"undervoltage", 10:"overcurrent", 13:"communication loss"}[fault]
    motor.enable(2)
    motor.command(2, 2)
    assert motor.check(0, 24, 2) == 2


def test_limit_driver_does_not_trip_at_high_demand_and_pack_root():
    motor, pack, envelope = ProtectedMotor(target=100), DatasheetBattery(), DatasheetMotor()
    demand = motor.check(0, 24, 0)
    v, current, torque = pack.solve([demand]*6, [0]*6, envelope)
    assert torque == pytest.approx([9.8]*6)
    assert current == pytest.approx(6*1.5*.33*9.8**2/v)
    assert v == pytest.approx(25.2-.04*current)
    assert motor.error == 1


def test_hand_checkable_suspension_moment_balances():
    # Each side: front x=1, middle x=0, rear x=-1, rocker pivot x=0,
    # bogie pivot x=-.5; 300 N centred chassis, massless suspension.
    # Sum N=300; equal side totals; zero global pitch; each bogie has
    # equal middle/rear loads; opposite rockers have equal moment sums.
    a = np.array([[1,1,1,1,1,1], [1,1,1,-1,-1,-1],
                  [1,0,-1,1,0,-1], [0,.5,-.5,0,0,0],
                  [0,0,0,0,.5,-.5], [1,0,-1,-1,0,1]], float)
    assert solve_loads(a, [-300,0,0,0,0,0]) == pytest.approx([50]*6)
    # Opposite 10 Nm moments on each bogie give a diagonal response.
    result = solve_loads(a, [-300,0,0,10,-10,20])
    assert result.sum() == pytest.approx(300)
    assert a @ result == pytest.approx([300,0,0,-10,10,-20])


def test_loaded_scene_static_prediction_and_force_sign():
    import mujoco
    model = mujoco.MjModel.from_xml_path("viz/scene/rover_scene.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    points = np.array([data.xpos[model.body("wheel_"+w).id] for w in ("lf","lm","lr","rf","rm","rr")])
    points[:, 2] -= .1
    static, residual = predict_loads(model, data, points, np.zeros((6,3)))
    assert static.sum() == pytest.approx(model.body_mass.sum()*9.81)
    assert static[:3] == pytest.approx(static[3:])
    assert np.max(np.abs(residual)) < 1e-10
    forces = np.zeros((6,3)); forces[:3,0] = -20; forces[3:,0] = 20
    turning, _ = predict_loads(model, data, points, forces)
    assert turning[0] > static[0] and turning[2] < static[2]
    assert turning[3] < static[3] and turning[5] > static[5]


@pytest.mark.parametrize("lock,removed", [("rockers",3), ("bogies",2), ("both",5)])
def test_counterfactuals_remove_dofs_and_keep_pivot_geometry(lock, removed):
    import mujoco
    from experiments.r3 import Case, load_scene
    from sim.rocker_bogie import geometry
    original = load_scene(Case(ground="plane"))
    locked = load_scene(Case(ground="plane",lock=lock))
    assert locked.nv == original.nv-removed
    data = mujoco.MjData(locked)
    mujoco.mj_forward(locked,data)
    assert len(geometry(locked,data)["pivots_world_m"]) == 4


def feedback(torque, error=1):
    return {i: decode_feedback(encode_feedback(i, 0, 0, torque, error)) for i in range(1,7)}


def test_governor_uses_quantized_feedback_and_slow_recovery():
    governor = TorqueGovernor()
    for _ in range(100):
        governor.update(.5, feedback(9.5))
    reduced = governor.scale
    assert 0 < reduced < .8
    assert governor.update(.5, feedback(8)) == pytest.approx(.5*reduced)
    for _ in range(100):
        governor.update(.5, feedback(2))
    assert governor.scale == pytest.approx(reduced+.03)
    assert governor.update(.5, feedback(0, 10)) == 0
    assert TorqueGovernor().update(.5, {}) == 0
