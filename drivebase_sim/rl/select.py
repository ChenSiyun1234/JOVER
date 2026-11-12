"""Select a checkpoint using only sixteen training-distribution seeds."""

from concurrent.futures import ProcessPoolExecutor
import multiprocessing as mp
from pathlib import Path
import shutil
import os

import numpy as np

from rl.live import atomic_json
from sim.resources import wait_for_simulation

VALIDATION_SEEDS = tuple(range(910000, 910016))
_models = {}


def validation_job(job):
    from rl.baselines import Learned
    from rl.env import TractionEnv
    checkpoint, seed = job
    if checkpoint not in _models:
        _models.clear()
        _models[checkpoint] = Learned(checkpoint)
    policy = _models[checkpoint]
    env = TractionEnv()
    observation, _ = env.reset(seed=seed)
    total = 0.
    while True:
        action, _ = policy.predict(observation)
        observation, reward, terminated, truncated, _ = env.step(action)
        total += reward
        if terminated or truncated:
            break
    env.close()
    return dict(checkpoint=checkpoint, seed=seed, return_value=total)


def select_checkpoints(run_dir, workers):
    from rl.train import worker_limit
    worker_limit(workers)
    run_dir = Path(run_dir)
    checkpoints = sorted((run_dir/'checkpoints').glob('ppo_*.zip'))
    if not checkpoints:
        raise FileNotFoundError('No saved checkpoints')
    wait_for_simulation()
    jobs = [(str(path), seed) for path in checkpoints for seed in VALIDATION_SEEDS]
    with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context('spawn')) as pool:
        scores = list(pool.map(validation_job, jobs))
    summaries = [dict(checkpoint=str(path), mean_return=float(np.mean([
        item['return_value'] for item in scores if item['checkpoint'] == str(path)])))
        for path in checkpoints]
    best = max(summaries, key=lambda item: item['mean_return'])
    part = run_dir/'best.zip.part'
    shutil.copyfile(best['checkpoint'], part)
    os.replace(part, run_dir/'best.zip')
    result = dict(seeds=list(VALIDATION_SEEDS), episodes=scores,
                  checkpoints=summaries, selected=best,
                  selection='Highest mean training-distribution validation return; no matrix cases used')
    atomic_json(run_dir/'validation.json', result)
    print(f"SELECTED {best['checkpoint']}: {best['mean_return']:.6f}", flush=True)
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=2 if os.name == 'nt' else 8)
    args = parser.parse_args()
    select_checkpoints(args.run_dir, args.workers)
