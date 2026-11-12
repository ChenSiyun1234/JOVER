"""Hand-designed comparisons and deterministic learned-policy adapters."""

import numpy as np

from rl.env import OBSERVATION_SCALE, SPEED_LIMIT
from sim.drive import TorqueGovernor, YawRatePI

CONTROLLERS = ('open', 'governor', 'pi', 'pi_backoff', 'wheel_guard', 'learned', 'learned_c')


class ActorSnapshot:
    """Detached float32 actor parameters for IPC to an idle showcase worker."""

    def __init__(self, model):
        from rl.export import actor_layers
        self.layers = actor_layers(model)

    def predict(self, observation):
        value = np.asarray(observation, dtype=np.float32)
        for i, (weight, bias) in enumerate(self.layers):
            value = value @ weight.T + bias
            if i < 2:
                value = np.tanh(value)
        return np.clip(value, -1, 1), None


class Baseline:
    """Use a single sampled gyro and the actual decoded CAN feedback.

    r3_exact is solely an equivalence-test adapter: the original PI's noise
    generator and loaded-voltage clip are retained there. Normal R4 comparisons
    use the shared observed gyro and fixed 25.2 V clipping for all controllers.
    """

    def __init__(self, name, *, r3_exact=False):
        if name not in CONTROLLERS[:5]:
            raise ValueError(name)
        self.name, self.r3_exact = name, r3_exact
        self.pi = YawRatePI(noise_std=.01 if r3_exact else 0.)
        self.governor = TorqueGovernor()
        self.wheel_scale = np.ones(6)

    def targets(self, env):
        rig = env.rig
        velocity, yaw = env.velocity, env.yaw
        base = rig.drive.speeds(velocity, yaw)
        if self.name == 'open':
            return base
        if self.name == 'governor':
            return rig.drive.speeds(velocity, self.governor.update(yaw, rig.bus.feedback))
        gyro = rig.gyro[2] if self.r3_exact else float(env.raw_observation[4])
        voltage = rig.voltage if self.r3_exact else 25.2
        old_integral = self.pi.integral
        targets = self.pi.speeds(rig.drive, velocity, yaw, gyro, voltage)
        if self.name == 'pi_backoff':
            self.governor.update(yaw, rig.bus.feedback)
            if self.governor.scale < 1:
                self.pi.integral = old_integral
            targets = np.clip(base + self.governor.scale*(targets-base),
                              -SPEED_LIMIT, SPEED_LIMIT)
        elif self.name == 'wheel_guard':
            for index in range(6):
                feedback = rig.bus.feedback.get(index + 1)
                if feedback is None:
                    targets[index] = 0.
                    continue
                ratio = abs(feedback['torque']) / (rig.case.kt * 9.8)
                if ratio > .85:
                    self.wheel_scale[index] *= .8
                elif ratio < .75:
                    self.wheel_scale[index] = min(1., self.wheel_scale[index] + .005)
                if feedback['error'] != 1:
                    targets[index] = 0.
                elif self.wheel_scale[index] < 1:
                    velocity_i = feedback['velocity']
                    targets[index] = velocity_i + self.wheel_scale[index] * (targets[index] - velocity_i)
            if np.any(self.wheel_scale < 1):
                self.pi.integral = old_integral
            targets = np.clip(targets, -SPEED_LIMIT, SPEED_LIMIT)
        return targets


class Learned:
    def __init__(self, path):
        import torch
        from stable_baselines3 import PPO
        torch.set_num_threads(1)
        self.model = PPO.load(path, device='cpu')

    def predict(self, observation):
        return self.model.predict(observation, deterministic=True)


class LearnedC:
    def __init__(self, library):
        import ctypes
        self.library = ctypes.CDLL(str(library))
        self.call = self.library.rover_policy
        pointer = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
        self.call.argtypes = [pointer, pointer]
        self.call.restype = None

    def predict(self, observation):
        raw = np.ascontiguousarray(observation*OBSERVATION_SCALE, dtype=np.float32)
        action = np.empty(6, dtype=np.float32)
        self.call(raw, action)
        return action, None
