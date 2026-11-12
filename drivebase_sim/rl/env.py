"""Host-observable 100 Hz traction allocation environment."""

from dataclasses import asdict, dataclass

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from experiments.r3 import Case, load_scene
from sim.drive import WHEELS
from sim.motor import KE
from sim.rig import DriveRig

PERIOD = .01
EPISODE_STEPS = 800
SPEED_LIMIT = 25.2 / KE
OBSERVATION_FIELDS = (
    'command_forward', 'command_yaw', 'gyro_x', 'gyro_y', 'gyro_z',
    'gravity_x', 'gravity_y', 'gravity_z',
    *(f'can_{wheel}_{field}' for wheel in WHEELS
      for field in ('velocity', 'torque', 'enabled')),
    *(f'previous_action_{wheel}' for wheel in WHEELS))
OBSERVATION_SCALE = np.array([.5, np.radians(40), 1, 1, 1, 1, 1, 1] +
                             [30, 12, 1]*6 + [1]*6, dtype=np.float32)
# Frozen before evaluation; costs are per second, integrated at 100 Hz.
REWARD_WEIGHTS = dict(yaw=4., forward=2., lateral=1., current=.5,
                      action_change=.02, power=.01, fault=100., tip=100.)


@dataclass(frozen=True)
class Scenario:
    ground: str = 'terrain'
    mu: float = .9
    behaviour: str = 'trip'
    supply: float = 25.2
    velocity: float = 0.
    yaw: float = 30.
    command_drop: float = 0.
    feedback_drop: float = 0.
    gyro_std: float = .01

    def rig_case(self):
        return Case(ground=self.ground, mu=self.mu, behaviour=self.behaviour,
                    supply=self.supply, velocity=self.velocity, yaw=self.yaw,
                    settle_limit=self.behaviour == 'trip')


def sample_command(rng):
    kind = rng.random()
    sign = rng.choice([-1., 1.])
    if kind < .5:
        return 0., sign*rng.uniform(10, 40)
    if kind < .75:
        return rng.uniform(.1, .4), sign*rng.uniform(5, 30)
    return rng.uniform(.2, .5), 0.


def sample_scenario(rng):
    velocity, yaw = sample_command(rng)
    return Scenario(ground=rng.choice(['terrain', 'plane']),
        mu=float(np.exp(rng.uniform(np.log(.1), np.log(1.)))),
        behaviour=rng.choice(['limit', 'trip']), supply=rng.uniform(22.2, 25.2),
        velocity=velocity, yaw=yaw, command_drop=rng.uniform(0, .05),
        feedback_drop=rng.uniform(0, .05), gyro_std=rng.uniform(.005, .02))


class TractionEnv(gym.Env):
    """The policy receives only the fixed-schema scaled observation.

    True physical state is used exclusively for reward, termination and logs.
    Gravity is an ideal low-pass attitude channel with additive 0.01 noise;
    this does not model a real accelerometer attitude estimator.
    """

    metadata = {'render_modes': []}

    def __init__(self, *, evaluation=False, record=False):
        self.evaluation, self.record = evaluation, record
        self.observation_space = spaces.Box(-np.inf, np.inf, (32,), np.float32)
        self.action_space = spaces.Box(-1., 1., (6,), np.float32)
        self.models = {}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        fixed = options.get('scenario')
        self.scenario = (Scenario(**fixed) if isinstance(fixed, dict) else fixed)
        if self.scenario is None:
            self.scenario = sample_scenario(self.np_random)
        self.velocity, self.yaw = self.scenario.velocity, np.radians(self.scenario.yaw)
        self.change_step, self.next_command = None, None
        if fixed is None and self.np_random.random() < .3:
            self.change_step = int(self.np_random.integers(300, 501))
            self.next_command = sample_command(self.np_random)
        case = self.scenario.rig_case()
        if case.ground not in self.models:
            self.models[case.ground] = load_scene(case)
        self.rig = DriveRig(case, seed=int(self.np_random.integers(0, 2**31)),
            command_drop=self.scenario.command_drop, feedback_drop=self.scenario.feedback_drop,
            model=self.models[case.ground], record=self.record)
        self.steps, self.episode_return = 0, 0.
        self._had_fault, self._had_tip = False, False
        self.previous_action = np.zeros(6, dtype=np.float32)
        self._observe()
        return self.observation.copy(), {'scenario': asdict(self.scenario)}

    def _observe(self):
        gyro = self.rig.gyro + self.np_random.normal(0, self.scenario.gyro_std, 3)
        gravity = self.rig.rotation[2, :] + self.np_random.normal(0, .01, 3)
        feedback = []
        for mid in range(1, 7):
            decoded = self.rig.bus.feedback.get(mid)
            feedback.extend([decoded['velocity'], decoded['torque'],
                             float(decoded['error'] == 1)] if decoded else [0., 0., 0.])
        self.raw_observation = np.array([self.velocity, self.yaw, *gyro, *gravity,
                                         *feedback, *self.previous_action], dtype=np.float32)
        self.observation = self.raw_observation / OBSERVATION_SCALE

    def action_targets(self, action):
        base = self.rig.drive.speeds(self.velocity, self.yaw)
        return np.clip(base + 5*np.clip(np.asarray(action), -1., 1.),
                       -SPEED_LIMIT, SPEED_LIMIT)

    def showcase(self, controller, scenario, seed, timesteps=0):
        """Run in an existing idle worker, avoiding a third local simulator."""
        from rl.live import episode
        return episode(controller, scenario, seed, timesteps=timesteps)

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1., 1.)
        return self.step_targets(self.action_targets(action), action)

    def step_targets(self, targets, action=None):
        """Baselines supply targets directly but share all integration/rewards."""
        if action is None:
            base = self.rig.drive.speeds(self.velocity, self.yaw)
            action = np.clip((np.asarray(targets)-base)/5, -1., 1.)
        electrical = self.rig.advance(targets)
        velocity = self.rig.body_velocity
        current = np.abs(electrical[:, 3:9]/self.rig.case.kt)
        # Squared hinge is continuous with a continuous first derivative.
        costs = dict(yaw=((self.rig.gyro[2]-self.yaw)/np.radians(30))**2,
            forward=((velocity[0]-self.velocity)/.5)**2,
            lateral=(velocity[1]/.5)**2,
            current=float(np.mean(np.maximum(current/(.8*9.8)-1, 0)**2)),
            action_change=float(np.mean((action-self.previous_action)**2)),
            power=float(np.mean(electrical[:, 1]*electrical[:, 2])/300))
        reward = -PERIOD*sum(REWARD_WEIGHTS[k]*v for k, v in costs.items())
        fault, tip = self.rig.failed, self.rig.rotation[2, 2] < .5
        # Evaluation also records a one-time penalty but always runs 800 steps.
        if fault and not self._had_fault:
            reward -= REWARD_WEIGHTS['fault']
        if tip and not self._had_tip:
            reward -= REWARD_WEIGHTS['tip']
        self._had_fault = self._had_fault or fault
        self._had_tip = self._had_tip or bool(tip)
        self.previous_action = np.asarray(action, dtype=np.float32).copy()
        self.steps += 1
        self.episode_return += reward
        if self.change_step == self.steps:
            self.velocity, yaw = self.next_command
            self.yaw = np.radians(yaw)
        self._observe()
        return (self.observation.copy(), float(reward),
                bool((fault or tip) and not self.evaluation),
                self.steps >= EPISODE_STEPS,
                dict(fault=fault, tipped=bool(tip), costs=costs))
