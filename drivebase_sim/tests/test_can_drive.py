import math
import random
import numpy as np
import pytest
import mujoco
from sim.can_dm import (Frame, Ranges, ERRORS, CanBus, encode_velocity, decode_velocity,
                        encode_mit, decode_mit, encode_feedback, decode_feedback)
from sim.motor import Motor, FREE_INERTIA, FREE_DAMPING, torque_limit
from sim.drive import Drive


@pytest.mark.parametrize("ranges", [Ranges(), Ranges(6.28, 25, 10)])
@pytest.mark.parametrize("fraction", [-1, 0, 1])
def test_mit_feedback_roundtrips(ranges, fraction):
    values = (fraction*ranges.p_max, fraction*ranges.v_max, (fraction+1)*250,
              (fraction+1)*2.5, fraction*ranges.t_max)
    decoded = decode_mit(encode_mit(6, *values, ranges=ranges), 6, ranges)
    steps = [2*ranges.p_max/65535, 2*ranges.v_max/4095, 500/4095, 5/4095, 2*ranges.t_max/4095]
    for original, actual, step in zip(values, decoded, steps):
        assert abs(original-actual) <= step/2 + 1e-12
    frame = encode_feedback(6, values[0], values[1], values[4], ranges=ranges)
    feedback = decode_feedback(frame, 6, ranges=ranges)
    for key, original, step in zip(("position", "velocity", "torque"),
                                   (values[0], values[1], values[4]), (steps[0], steps[1], steps[4])):
        assert abs(feedback[key]-original) <= step/2 + 1e-12


def test_documented_byte_examples():
    frame = encode_velocity(1, 10)
    assert frame.can_id == 0x201 and frame.data.hex() == "00002041"
    assert encode_mit(1, 0, 0, 0, 0, 0).data.hex() == "8000800000000800"
    assert encode_feedback(1, 0, 0, 0).data.hex() == "1180008008001919"
    assert decode_velocity(frame, 1) == 10
    assert encode_mit(1, 12.5, 30, 500, 5, 12).data == b"\xff"*8


def test_wrong_ids_and_malformed_lengths():
    assert decode_velocity(encode_velocity(2, 10), 1) is None
    assert decode_mit(encode_mit(2, 0, 0, 0, 0, 0), 1) is None
    assert decode_feedback(encode_feedback(2, 0, 0, 0), 1) is None
    assert decode_feedback(Frame(1, b"\0"*8)) is None
    for length in range(9):
        if length != 4:
            with pytest.raises(ValueError):
                decode_velocity(Frame(0x201, b"\0"*length), 1)
        if length != 8:
            for decoder in (lambda f: decode_mit(f, 1), decode_feedback):
                with pytest.raises(ValueError):
                    decoder(Frame(1, b"\0"*length))
    with pytest.raises(ValueError):
        decode_velocity(Frame(0x201, bytes.fromhex("0000c07f")), 1)


@pytest.mark.parametrize("code, expected", [(8, "overvoltage"), (9, "undervoltage"),
    (10, "overcurrent"), (11, "MOS overtemperature"), (12, "coil overtemperature"),
    (13, "communication loss"), (14, "overload")])
def test_error_codes(code, expected):
    decoded = decode_feedback(encode_feedback(6, 0, 0, 0, error=code))
    assert decoded["error"] == code and decoded["error_name"] == expected


def test_mixed_bit_fields():
    decoded = decode_mit(Frame(1, bytes.fromhex("123456789abcdef0")), 1)
    raw = (0x1234, 0x567, 0x89a, 0xbcd, 0xef0)
    bounds = ((-12.5, 12.5, 65535), (-30, 30, 4095), (0, 500, 4095),
              (0, 5, 4095), (-12, 12, 4095))
    expected = [low+u*(high-low)/maximum for u, (low, high, maximum) in zip(raw, bounds)]
    assert decoded == pytest.approx(expected)
    assert encode_mit(1, *expected).data.hex() == "123456789abcdef0"


def test_transport_delay_feedback_and_hold():
    bus = CanBus(motor_ids=[1])
    state = {1: (0, 2, 3, 1)}
    bus.send(encode_velocity(1, 10), 0)
    bus.advance(.0009, state)
    assert bus.targets[1] == 0
    bus.advance(.001, state)
    assert bus.targets[1] == 10 and not bus.feedback
    bus.advance(.002, state)
    assert bus.feedback[1]["velocity"] == pytest.approx(2, abs=30/4095)
    bus.drop_rate = 1
    bus.send(encode_velocity(1, 20), .01)
    bus.advance(.5, state)
    assert bus.targets[1] == 10
    bus.drop_rate = 0
    bus.send(encode_velocity(2, 99), .5)
    bus.advance(.501, state)
    assert bus.targets == {1: 10}


def test_twenty_percent_command_loss_tracks_for_half_second():
    bus, motor, speed = CanBus(motor_ids=[1], drop_rate=.2, seed=7), Motor(), 0.
    for tick in range(500):
        t = tick * .001
        if tick % 10 == 0:
            bus.send(encode_velocity(1, 10), t)
        bus.advance(t, {1: (0, speed, 0, 1)})
        motor.target = bus.targets[1]
        limit = torque_limit(speed, 24)
        torque = float(np.clip(motor.demand(speed), -limit, limit))
        motor.accept(speed, torque)
        speed += .001 * (torque-FREE_DAMPING*speed)/FREE_INERTIA
    assert bus.dropped > 0
    assert abs(speed-10) < .02
    assert bus.received > 25


def test_5000_random_eight_byte_payloads():
    rng = random.Random(20261109)
    for _ in range(5000):
        payload = rng.randbytes(8)
        assert all(math.isfinite(x) for x in decode_mit(Frame(1, payload), 1))
        feedback = decode_feedback(Frame(0, payload))
        assert all(math.isfinite(feedback[key]) for key in ("position", "velocity", "torque"))


def test_scene_kinematics():
    model = mujoco.MjModel.from_xml_path("viz/scene/rover_scene.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    drive = Drive.from_scene(model, data)
    assert drive.separation == pytest.approx(.7208, abs=1e-4)
    assert drive.radius == .1
    assert drive.speeds(.5, 0) == pytest.approx([5]*6)
    turn = drive.speeds(0, 1)
    assert turn[:3] == pytest.approx([-drive.separation*1.25/.2]*3)
    assert turn[3:] == pytest.approx(-turn[:3])
