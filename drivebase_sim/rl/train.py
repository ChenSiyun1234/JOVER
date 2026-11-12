"""CPU PPO training; real budgets run on the Colab VM."""

import argparse
from functools import partial
import os
from pathlib import Path
import time

from rl.live import atomic_json, callback_class
from sim.resources import wait_for_simulation

TRAIN_SEED = 70000


def worker_limit(workers):
    maximum = 2 if os.name == 'nt' else 8
    if not 1 <= workers <= maximum:
        raise ValueError(f'workers must be in 1..{maximum} on this platform')


def make_env(seed):
    from stable_baselines3.common.monitor import Monitor
    from rl.env import TractionEnv
    env = Monitor(TractionEnv())
    env.reset(seed=seed)
    return env


def train(args):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv
    worker_limit(args.workers)
    if min(args.steps, args.live_every, args.checkpoint_every) <= 0:
        raise ValueError('steps and intervals must be positive')
    out = args.run_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if not args.resume and (out/'training.json').exists():
        raise FileExistsError('Use a fresh run directory or --resume')
    torch.set_num_threads(1)
    wait_for_simulation()
    env = SubprocVecEnv([partial(make_env, args.seed+i) for i in range(args.workers)],
                       start_method='spawn')
    try:
        if args.resume:
            model = PPO.load(args.resume, env=env, device='cpu', seed=args.seed)
        else:
            model = PPO('MlpPolicy', env, device='cpu', seed=args.seed,
                n_steps=1000, batch_size=250, n_epochs=10, learning_rate=3e-4,
                gamma=.995, gae_lambda=.95, clip_range=.2, ent_coef=args.ent_coef,
                vf_coef=.5, max_grad_norm=.5,
                policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]),
                                   activation_fn=torch.nn.Tanh, log_std_init=-1.5),
                verbose=0)
            # Start the deterministic actor at the team default, without
            # removing stochastic exploration during training.
            with torch.no_grad():
                model.policy.action_net.weight.zero_()
                model.policy.action_net.bias.zero_()
            model.save(out/'initial.zip')
        initial = model.num_timesteps
        model.set_logger(configure(str(out), ['csv']))
        started = time.perf_counter()
        callback = callback_class()(out, initial+args.steps, live_every=args.live_every,
            checkpoint_every=args.checkpoint_every, source=args.source,
            references=not args.skip_references)
        model.learn(total_timesteps=args.steps, callback=callback,
                    reset_num_timesteps=not bool(args.resume))
        model.save(out/'ppo.zip')
        elapsed = time.perf_counter()-started
        atomic_json(out/'training.json', dict(timesteps=model.num_timesteps,
            new_timesteps=model.num_timesteps-initial, seed=args.seed, workers=args.workers,
            wall_s=elapsed, steps_per_s=(model.num_timesteps-initial)/elapsed,
            resumed_from=str(args.resume) if args.resume else None,
            hyperparameters=dict(n_steps=1000, batch_size=250, n_epochs=10,
                learning_rate=3e-4, gamma=.995, gae_lambda=.95, clip_range=.2,
                ent_coef=args.ent_coef, vf_coef=.5, max_grad_norm=.5, log_std_init=-1.5)))
        print(f'TRAINING COMPLETE: {model.num_timesteps} steps, {elapsed:.3f} s', flush=True)
    finally:
        env.close()
    if not args.skip_selection:
        from rl.select import select_checkpoints
        select_checkpoints(out, args.workers)


def parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=3000000)
    parser.add_argument('--workers', type=int, default=2 if os.name == 'nt' else 8)
    parser.add_argument('--seed', type=int, default=TRAIN_SEED)
    parser.add_argument('--live-every', type=int, default=100000)
    parser.add_argument('--checkpoint-every', type=int, default=250000)
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--source', choices=['local', 'colab'], default='local')
    parser.add_argument('--skip-references', action='store_true')
    parser.add_argument('--skip-selection', action='store_true')
    parser.add_argument('--ent-coef', type=float, default=.005)
    return parser


if __name__ == '__main__':
    train(parser().parse_args())
