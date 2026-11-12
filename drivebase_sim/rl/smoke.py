"""Reproduce the local-only checks; never publish smoke numbers as results."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import tempfile

import numpy as np

from rl.env import TractionEnv
from rl.live import atomic_json
from sim.resources import wait_for_simulation


def throughput(out):
    wait_for_simulation()
    env = TractionEnv(evaluation=True)
    env.reset(seed=630002)
    started = time.perf_counter()
    for _ in range(2000):
        _, _, terminated, truncated, _ = env.step(np.zeros(6))
        if terminated or truncated:
            env.reset()
    wall = time.perf_counter()-started
    env.close()
    result = dict(steps=2000, wall_s=wall, steps_per_s=2000/wall,
        ideal_3M_8workers_hours=3000000/(8*2000/wall)/3600,
        note='Ideal linear scaling; excludes PPO, validation and showcase overhead')
    atomic_json(Path(out)/'throughput.json', result)
    print(json.dumps(result, indent=2), flush=True)
    return result


def command(args):
    print('RUN', ' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True)


def main(args):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if not args.reuse_training:
        throughput(out)
        command([sys.executable, '-m', 'rl.train', '--run-dir', out/'train',
                 '--steps', 20000, '--workers', 2, '--skip-selection'])
    model = out/'train/ppo.zip'
    library = args.export_dir/('policy.dll' if sys.platform == 'win32' else 'policy.so')
    command([sys.executable, '-m', 'rl.export', '--model', model, '--out', args.export_dir])
    command([sys.executable, '-m', 'rl.evaluate', '--model', model, '--workers', 2,
        '--library', library,
        '--out', out/'evaluation', '--controllers', 'open', 'pi_backoff', 'learned',
        '--cases', 'turn_mu0.9_trip_terrain', 'turn_mu0.45_limit_terrain',
        '--timesteps', 20000, '--smoke'])
    from rl.baselines import Learned
    from rl.export import compare
    with np.load(out/'evaluation/observations.npz') as data:
        observations = data['observations']
    error = compare(Learned(model).model, library.resolve(), observations)
    check = dict(matrix_smoke_observations=len(observations), max_abs_error=error,
                 note='Only two smoke cases; full 10000 distinct matrix observations checked on Colab')
    atomic_json(out/'c_matrix_smoke.json', check)
    if error > 1e-5:
        raise AssertionError(check)
    print(json.dumps(check, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path(tempfile.gettempdir()) / "rover_r4_smoke")
    parser.add_argument('--export-dir', type=Path, default=Path('rl/export'))
    parser.add_argument('--reuse-training', action='store_true')
    main(parser.parse_args())
