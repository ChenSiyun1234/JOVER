"""Atomic 50 Hz body-pose traces and SB3 publication callback."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil

import numpy as np

from rl.baselines import ActorSnapshot, Baseline, CONTROLLERS
from rl.env import Scenario, TractionEnv
from sim.resources import wait_for_simulation

SHOWCASES = {'hard': Scenario(), 'mid': Scenario(mu=.45, behaviour='limit')}
SHOWCASE_SEEDS = {'hard': 510001, 'mid': 510002}
TRACE_KEYS = ('t', 'pos', 'quat', 'yaw_rate', 'yaw_cmd', 'current', 'error', 'bus_v')


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_suffix(path.suffix + '.part')
    part.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    os.replace(part, path)


def atomic_episode(path, values):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_suffix('.npz.part')
    with part.open('wb') as stream:
        np.savez_compressed(stream, **values)
    os.replace(part, path)


def metrics(env, yaw_commands):
    rig = env.rig
    a, p = np.asarray(rig.electrical_rows), np.asarray(rig.physics_rows)
    selected = p[:, 0] >= 1
    heading = np.unwrap(p[:, 1])
    current = np.abs(a[:, 3:9]/rig.case.kt)
    return dict(yaw_deg_s=float(np.degrees(heading[selected][-1]-heading[selected][0]) /
                               (p[selected, 0][-1]-p[selected, 0][0])),
        yaw_rmse_deg_s=float(np.sqrt(np.mean((np.degrees(p[selected, 2]) -
                                  np.repeat(yaw_commands, 5)[selected])**2))),
        mean_forward_error_m_s=float(np.mean(p[:, 6]-env.scenario.velocity))
            if env.scenario.velocity else None,
        mean_abs_forward_error_m_s=float(np.mean(np.abs(p[:, 6]-env.scenario.velocity)))
            if env.scenario.velocity else None,
        lateral_drift_m=float(np.abs(p[:, 7]).sum()*.002),
        displacement_m=float(np.linalg.norm(rig.data.qpos[:2]-rig.start[:2])),
        peak_bus_A=float(a[:, 2].max()), mean_bus_A=float(a[:, 2].mean()),
        energy_J=float(np.sum(a[:, 1]*a[:, 2])*.001),
        minimum_voltage_V=float(a[:, 1].min()),
        peak_wheel_current_A=current.max(axis=0).tolist(),
        mean_wheel_current_A=current.mean(axis=0).tolist(),
        faults=rig.faults, minimum_upright=float(p[:, 9].min()),
        ep_return=float(env.episode_return))


def episode(controller, scenario, seed, *, timesteps=0, trace=True):
    """One full evaluation episode, never terminating early on a fault."""
    env = TractionEnv(evaluation=True, record=True)
    obs, _ = env.reset(seed=seed, options={'scenario': scenario})
    baseline = Baseline(controller) if isinstance(controller, str) else None
    rows = {key: [] for key in TRACE_KEYS}
    observations, commands = [], []
    for step in range(800):
        observations.append(obs.copy())
        commands.append(float(np.degrees(env.yaw)))
        if trace and step % 2 == 0:
            row = env.rig.replay_row()
            row['yaw_cmd'] = commands[-1]
            for key in TRACE_KEYS:
                rows[key].append(row[key])
        if baseline:
            obs, _, _, _, _ = env.step_targets(baseline.targets(env))
        else:
            action, _ = controller.predict(obs)
            obs, _, _, _, _ = env.step(action)
    result = metrics(env, commands)
    data = {key: np.asarray(value, dtype=np.int32 if key == 'error' else np.float32)
            for key, value in rows.items()}
    data.update(timesteps=np.asarray(timesteps, dtype=np.int64),
        ep_return=np.asarray(result['ep_return']),
        mean_yaw_deg_s=np.asarray(result['yaw_deg_s']),
        drift_m=np.asarray(result['lateral_drift_m']),
        faults=np.asarray(json.dumps(result['faults'])))
    body_names = env.rig.body_names
    env.close()
    return result, data, np.asarray(observations), body_names


def write_meta(live, run_id, total_timesteps, body_names, source):
    atomic_json(Path(live)/'meta.json', dict(run_id=run_id,
        total_timesteps=total_timesteps,
        showcase_cases={name: asdict(case) for name, case in SHOWCASES.items()},
        controller_names=list(CONTROLLERS), body_names=body_names,
        scene_file='viz/scene/rover_scene.xml', source=source,
        sample_hz=50, quaternion_order='wxyz'))


def callback_class():
    # Keep SB3/torch out of pure simulation/evaluation workers.
    from stable_baselines3.common.callbacks import BaseCallback

    class LiveCallback(BaseCallback):
        def __init__(self, out, total, *, live_every=100000, checkpoint_every=250000,
                     source='local', references=True):
            super().__init__()
            self.out, self.total = Path(out), total
            self.live = self.out/'live'
            self.live_every, self.checkpoint_every = live_every, checkpoint_every
            self.source, self.references = source, references
            self.last_publication = None

        def _on_training_start(self):
            self.live.mkdir(parents=True, exist_ok=True)
            (self.out/'checkpoints').mkdir(parents=True, exist_ok=True)
            self.next_live = (self.num_timesteps//self.live_every+1)*self.live_every
            self.next_checkpoint = (self.num_timesteps//self.checkpoint_every+1)*self.checkpoint_every
            if self.references:
                for controller in CONTROLLERS[:5]:
                    for case, scenario in SHOWCASES.items():
                        path = self.live/f'ref_{controller}_{case}.npz'
                        if not path.exists():
                            _, data, _, names = self.training_env.env_method('showcase',
                                controller, scenario, SHOWCASE_SEEDS[case], indices=0)[0]
                            atomic_episode(path, data)
                            write_meta(self.live, self.out.name, self.total, names, self.source)

        def publish(self):
            if self.last_publication == self.num_timesteps:
                return
            actor = ActorSnapshot(self.model)
            for case, scenario in SHOWCASES.items():
                _, data, _, names = self.training_env.env_method('showcase',
                    actor, scenario, SHOWCASE_SEEDS[case],
                    self.num_timesteps, indices=0)[0]
                atomic_episode(self.live/f'ep_{self.num_timesteps:010d}_{case}.npz', data)
                write_meta(self.live, self.out.name, self.total, names, self.source)
            self.model.logger.record('time/total_timesteps', self.num_timesteps)
            self.model.logger.dump(self.num_timesteps)
            source = self.out/'progress.csv'
            if source.exists():
                part = self.live/'progress.csv.part'
                shutil.copyfile(source, part)
                os.replace(part, self.live/'progress.csv')
            self.last_publication = self.num_timesteps

        def checkpoint(self):
            path = self.out/'checkpoints'/f'ppo_{self.num_timesteps:010d}.zip'
            part = path.with_suffix('.zip.part')
            with part.open('wb') as stream:
                self.model.save(stream)
            os.replace(part, path)

        def _on_step(self):
            # SubprocVecEnv has returned all workers; reuse worker zero while
            # the others are idle. Final publication follows the final update.
            if self.num_timesteps < self.model._total_timesteps:
                if self.num_timesteps >= self.next_live:
                    self.publish()
                    self.next_live = (self.num_timesteps//self.live_every+1)*self.live_every
                if self.num_timesteps >= self.next_checkpoint:
                    self.checkpoint()
                    self.next_checkpoint = (self.num_timesteps//self.checkpoint_every+1)*self.checkpoint_every
            return True

        def _on_training_end(self):
            self.checkpoint()
            self.publish()

    return LiveCallback
