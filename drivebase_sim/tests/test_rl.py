"""R4 equivalence, observation boundary, safety and deployable actor checks."""

from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from experiments.r3 import cases
from rl.baselines import Baseline
from rl.env import (OBSERVATION_FIELDS, OBSERVATION_SCALE, REWARD_WEIGHTS,
                    Scenario, TractionEnv)
from sim.drive import Drive, WHEELS
from sim.rig import DriveRig


@pytest.mark.parametrize('name', ['fix_mu0.9_trip_open', 'fix_mu0.9_limit_pi',
                                  'fix_mu0.45_limit_pi', 'fix_mu0.2_trip_open'])
def test_rig_matches_r3(name):
    summaries = {s['name']: s for s in json.loads(Path('results/r3/summary.json').read_text())}
    case = cases()[name]
    rig = DriveRig(case, record=True)
    env = SimpleNamespace(rig=rig, velocity=case.velocity, yaw=np.radians(case.yaw))
    controller = Baseline(case.controller, r3_exact=True)
    for _ in range(800):
        rig.advance(controller.targets(env))
    rows = np.asarray(rig.physics_rows)
    selected = rows[:, 0] >= 1
    heading = np.unwrap(rows[:, 1])
    yaw = float(np.degrees(heading[selected][-1]-heading[selected][0]) /
                (rows[selected, 0][-1]-rows[selected, 0][0]))
    assert abs(yaw-summaries[name]['yaw_deg_s']) <= 1e-9
    assert set(rig.faults) == set(summaries[name]['trips'])
    for wheel, fault in rig.faults.items():
        assert fault['error'] == summaries[name]['trips'][wheel]['error']
        assert fault['time_s'] == summaries[name]['trips'][wheel]['time_s']


def test_zero_action_is_exact_open_trajectory():
    left, right = TractionEnv(evaluation=True), TractionEnv(evaluation=True)
    options = {'scenario': Scenario()}
    left.reset(seed=640001, options=options)
    right.reset(seed=640001, options=options)
    baseline = Baseline('open')
    for _ in range(100):
        assert np.array_equal(left.action_targets(np.zeros(6)), baseline.targets(right))
        left.step(np.zeros(6))
        right.step_targets(baseline.targets(right))
        assert np.array_equal(left.rig.data.qpos, right.rig.data.qpos)
        assert np.array_equal(left.rig.torque, right.rig.torque)


def test_observation_has_only_allowlisted_host_channels():
    expected = ['command_forward', 'command_yaw', 'gyro_x', 'gyro_y', 'gyro_z',
                'gravity_x', 'gravity_y', 'gravity_z']
    for wheel in ('lf', 'lm', 'lr', 'rf', 'rm', 'rr'):
        expected += [f'can_{wheel}_velocity', f'can_{wheel}_torque', f'can_{wheel}_enabled']
    expected += [f'previous_action_{wheel}' for wheel in ('lf', 'lm', 'lr', 'rf', 'rm', 'rr')]
    assert list(OBSERVATION_FIELDS) == expected
    assert len(expected) == 32
    env = TractionEnv()
    env._np_random = np.random.default_rng(640002)
    env.scenario = Scenario(gyro_std=.01)
    env.velocity, env.yaw = .2, .3
    env.previous_action = np.linspace(-1, 1, 6)
    # This sensor facade has no pose, friction, power, contacts or true speed.
    # The feedback can be stale and is not reconstructed from wheel truth.
    feedback = {i: dict(velocity=2*i, torque=-i, error=1 if i != 3 else 10)
                for i in range(1, 7)}
    env.rig = SimpleNamespace(gyro=np.array([.1, .2, .3]), rotation=np.eye(3),
                              bus=SimpleNamespace(feedback=feedback))
    env._observe()
    expected_can = [value for i in range(1, 7) for value in (2*i, -i, float(i != 3))]
    assert np.array_equal(env.raw_observation[8:26], expected_can)
    assert np.array_equal(env.raw_observation[26:], env.previous_action.astype(np.float32))
    assert np.array_equal(env.observation, env.raw_observation/OBSERVATION_SCALE)
    del feedback[4]
    env._observe()
    assert np.array_equal(env.raw_observation[17:20], [0, 0, 0])


def test_same_seed_reproduces_episode_and_commands():
    a, b = TractionEnv(), TractionEnv()
    oa, _ = a.reset(seed=640003)
    ob, _ = b.reset(seed=640003)
    assert asdict(a.scenario) == asdict(b.scenario)
    assert (a.change_step, a.next_command) == (b.change_step, b.next_command)
    assert np.array_equal(oa, ob)
    for action in np.random.default_rng(640004).uniform(-.2, .2, (50, 6)):
        oa, ra, ta, xa, _ = a.step(action)
        ob, rb, tb, xb, _ = b.step(action)
        assert np.array_equal(oa, ob)
        assert (ra, ta, xa) == (rb, tb, xb)
        assert np.array_equal(a.rig.data.qpos, b.rig.data.qpos)
        if ta or xa:
            break


@pytest.mark.parametrize('evaluation', [False, True])
def test_fault_penalty_and_training_termination(evaluation):
    env = TractionEnv(evaluation=evaluation)
    env.reset(seed=640005, options={'scenario': Scenario(behaviour='limit')})
    env.rig.motors[0].error = 10
    _, reward, terminated, truncated, info = env.step(np.zeros(6))
    assert reward <= -REWARD_WEIGHTS['fault']
    assert terminated is (not evaluation)
    assert not truncated and info['fault']
    # A fresh episode resets the one-time penalty latch.
    env.reset(seed=640005, options={'scenario': Scenario(behaviour='limit')})
    assert not env._had_fault


def test_pi_backoff_freezes_integrator_from_decoded_torque():
    env = SimpleNamespace(velocity=0., yaw=.5, raw_observation=np.zeros(32),
        rig=SimpleNamespace(drive=Drive(.72, .1, 1.), bus=SimpleNamespace(
            feedback={i: dict(error=1, torque=9.5) for i in range(1, 7)})))
    baseline = Baseline('pi_backoff')
    targets = baseline.targets(env)
    assert 0 < baseline.governor.scale < 1
    assert baseline.pi.integral == 0
    base = env.rig.drive.speeds(0, .5)
    assert np.all(np.abs(targets-base) < np.abs(base*2))


def test_wheel_guard_decoded_feedback():
    kt = .8
    feedback = {i: dict(error=1, torque=0., velocity=float(i)) for i in range(1, 7)}
    env = SimpleNamespace(velocity=0., yaw=.5, raw_observation=np.zeros(32),
        rig=SimpleNamespace(drive=Drive(.72, .1, 1.), case=SimpleNamespace(kt=kt),
                            bus=SimpleNamespace(feedback=feedback)))
    guard, pi = Baseline('wheel_guard'), Baseline('pi')
    expected = pi.targets(env)
    assert np.array_equal(guard.targets(env), expected)
    feedback[2]['torque'] = .95 * kt * 9.8
    old_integral = guard.pi.integral
    expected = pi.targets(env)
    actual = guard.targets(env)
    assert guard.pi.integral == old_integral
    assert guard.wheel_scale[1] == pytest.approx(.8)
    assert actual[1] == pytest.approx(feedback[2]['velocity'] + .8 *
                                      (expected[1] - feedback[2]['velocity']))
    assert np.array_equal(actual[[0, 2, 3, 4, 5]], expected[[0, 2, 3, 4, 5]])
    feedback[2]['torque'] = 0.
    feedback[3]['error'] = 10
    assert guard.targets(env)[2] == 0.
    assert guard.wheel_scale[1] == pytest.approx(.805)


def test_c_export_matches_pytorch_and_speed_clipping(tmp_path):
    import torch
    from stable_baselines3 import PPO
    from rl.baselines import LearnedC
    from rl.export import generate, build, compare, elf_sizes
    torch.set_num_threads(1)
    model = PPO('MlpPolicy', TractionEnv(), seed=640006, device='cpu',
                policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64])))
    generate(model, tmp_path)
    library, _ = build(tmp_path)
    observations = np.random.default_rng(640007).uniform(-3, 3, (10000, 32)).astype(np.float32)
    assert compare(model, library, observations) < 1e-5
    from rl.baselines import ActorSnapshot
    assert np.max(np.abs(ActorSnapshot(model).predict(observations)[0] -
                         model.predict(observations, deterministic=True)[0])) < 1e-5
    c = LearnedC(library)
    pointer = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
    import ctypes
    c.library.rover_targets.argtypes = [pointer, ctypes.c_float, ctypes.c_float, pointer]
    c.library.rover_targets.restype = None
    env = TractionEnv(evaluation=True)
    env.reset(seed=640008, options={'scenario': Scenario(velocity=.5, yaw=40)})
    for raw in [env.raw_observation, np.full(32, 100, dtype=np.float32)]:
        env.velocity, env.yaw = float(raw[0]), float(raw[1])
        action = model.predict(raw/OBSERVATION_SCALE, deterministic=True)[0]
        expected = env.action_targets(action)
        actual = np.empty(6, dtype=np.float32)
        c.library.rover_targets(np.ascontiguousarray(raw), env.rig.drive.separation,
                                env.rig.drive.radius, actual)
        assert np.max(np.abs(actual-expected)) < 1e-5
    sizes = elf_sizes(tmp_path/'policy_m4.o')
    # Zero-initialized biases can be folded into instructions by the compiler.
    assert sizes['text'] > 0 and sizes['rodata'] >= (32*64+64*64+64*6)*4
    assert sizes['data'] == sizes['bss'] == 0


def test_live_atomic_format_and_exact_start_poses(tmp_path, monkeypatch):
    import os
    import mujoco
    from rl.live import TRACE_KEYS, atomic_episode, episode, write_meta
    result, trace, _, names = episode('open', Scenario(mu=.45, behaviour='limit'), 640009)
    expected = set(TRACE_KEYS) | {'timesteps', 'ep_return', 'mean_yaw_deg_s', 'drift_m', 'faults'}
    assert set(trace) == expected
    assert trace['t'].shape == (400,)
    assert trace['t'][0] == 0
    assert trace['t'][-1] == pytest.approx(7.98)
    assert trace['pos'].shape == (400, len(names), 3)
    assert trace['quat'].shape == (400, len(names), 4)
    assert np.max(np.abs(np.linalg.norm(trace['quat'], axis=2)-1)) < 1e-6
    assert json.loads(str(trace['faults'])) == result['faults']
    env = TractionEnv(evaluation=True)
    env.reset(seed=640009, options={'scenario': Scenario(mu=.45, behaviour='limit')})
    fresh = mujoco.MjData(env.rig.model)
    fresh.qpos[:] = env.rig.data.qpos
    mujoco.mj_kinematics(env.rig.model, fresh)
    assert np.max(np.abs(trace['pos'][0]-fresh.xpos[env.rig.body_ids])) < 1e-6
    replace = os.replace
    seen = []
    def checked_replace(source, destination):
        assert str(source).endswith('.part') and Path(source).is_file()
        if str(destination).endswith('.npz'):
            with np.load(source, allow_pickle=False) as saved:
                assert set(saved.files) == expected
        else:
            assert json.loads(Path(source).read_text())['body_names'] == names
        seen.append(Path(destination).name)
        replace(source, destination)
    monkeypatch.setattr(os, 'replace', checked_replace)
    atomic_episode(tmp_path/'ref_open_mid.npz', trace)
    write_meta(tmp_path, 'test', 20000, names, 'local')
    assert seen == ['ref_open_mid.npz', 'meta.json']
    assert not list(tmp_path.glob('*.part'))


def test_matrix_and_validation_seeds_are_disjoint():
    from rl.evaluate import TEST_SEEDS, test_matrix
    from rl.select import VALIDATION_SEEDS
    from rl.live import SHOWCASE_SEEDS
    assert len(test_matrix()) == 22
    assert not (set(TEST_SEEDS) & set(VALIDATION_SEEDS))
    assert not (set(SHOWCASE_SEEDS.values()) & (set(TEST_SEEDS) | set(VALIDATION_SEEDS)))
