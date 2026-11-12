"""Frozen 22-case matrix; smoke subsets must never write result tables."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import multiprocessing as mp
import os
from pathlib import Path

import numpy as np

from rl.baselines import CONTROLLERS
from rl.env import Scenario
from rl.live import atomic_episode, atomic_json, episode, write_meta
from sim.resources import wait_for_simulation

TEST_SEEDS = tuple(range(420001, 420023))


def test_matrix():
    cases = {}
    for mu in (.11, .2, .45, .9):
        for behaviour in ('limit', 'trip'):
            for ground in ('terrain', 'plane'):
                name = f'turn_mu{mu}_{behaviour}_{ground}'
                cases[name] = Scenario(ground=ground, mu=mu, behaviour=behaviour)
    for mu in (.45, .9):
        cases[f'arc_mu{mu}_trip_terrain'] = Scenario(mu=mu, velocity=.2)
    cases['straight_mu0.9_trip_terrain'] = Scenario(velocity=.5, yaw=0.)
    cases['ood_mu1.1_trip_terrain'] = Scenario(mu=1.1)
    cases['ood_drops0.10_mu0.45_trip_terrain'] = Scenario(mu=.45, command_drop=.1, feedback_drop=.1)
    cases['ood_supply21_mu0.9_limit_terrain'] = Scenario(behaviour='limit', supply=21.)
    return {name: (case, seed) for (name, case), seed in zip(cases.items(), TEST_SEEDS)}


_controllers = {}


def evaluation_job(job):
    name, controller, scenario, seed, model_path, library, trace_dir, timesteps = job
    if controller in CONTROLLERS[:5]:
        policy = controller
    else:
        from rl.baselines import Learned, LearnedC
        key = (controller, model_path, library)
        if key not in _controllers:
            _controllers[key] = Learned(model_path) if controller == 'learned' else LearnedC(library)
        policy = _controllers[key]
    result, trace, observations, names = episode(policy, scenario, seed,
        timesteps=timesteps, trace=controller in ('learned', 'pi_backoff', 'wheel_guard'))
    if controller in ('learned', 'pi_backoff', 'wheel_guard'):
        atomic_episode(Path(trace_dir)/f'{controller}_{name}.npz', trace)
    result.update(name=name, controller=controller, scenario=asdict(scenario), seed=seed)
    print(f"{controller} {name}: yaw={result['yaw_deg_s']:.6f}, "
          f"faults={list(result['faults'])}", flush=True)
    return result, observations if controller == 'learned' else None, names


def plots_and_table(out, results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = list(test_matrix())
    controllers = list(CONTROLLERS)
    lines = ['R4 frozen test matrix; selected by disjoint validation returns.',
             'Mean yaw and yaw RMSE: 1-8 s. Electrical/drift: full 0-8 s.',
             'case controller yaw_deg_s yaw_rmse forward_error lateral_m bus_peak_A bus_mean_A energy_J Vmin faults']
    for result in results:
        forward = result['mean_forward_error_m_s']
        lines.append(f"{result['name']} {result['controller']} {result['yaw_deg_s']:.6f} "
            f"{result['yaw_rmse_deg_s']:.6f} {forward if forward is not None else '-'} "
            f"{result['lateral_drift_m']:.6f} {result['peak_bus_A']:.6f} "
            f"{result['mean_bus_A']:.6f} {result['energy_J']:.6f} "
            f"{result['minimum_voltage_V']:.6f} {result['faults']}")
    report = out/'report.txt'
    part = report.with_suffix('.txt.part')
    part.write_text('\n'.join(lines)+'\n', encoding='utf-8')
    os.replace(part, report)
    for metric, ylabel, filename in [('yaw_deg_s', 'Achieved heading yaw rate (deg/s)', 'yaw.png'),
            ('peak_wheel_current_A', 'Peak absolute wheel current, maximum of six (A)', 'current.png')]:
        fig, ax = plt.subplots(figsize=(16, 7), layout='constrained')
        for controller in controllers:
            by_name = {r['name']: r for r in results if r['controller'] == controller}
            values = [max(by_name[name][metric]) if metric == 'peak_wheel_current_A'
                      else by_name[name][metric] for name in names]
            ax.plot(range(len(names)), values, '-o', label=controller, markersize=3)
        ax.set(xticks=range(len(names)), xticklabels=names, ylabel=ylabel)
        ax.tick_params(axis='x', labelrotation=75, labelsize=7)
        ax.grid(alpha=.2)
        ax.legend(ncol=3)
        part = out/(filename+'.part')
        fig.savefig(part, format='png', dpi=150)
        plt.close(fig)
        os.replace(part, out/filename)


def evaluate(args):
    from rl.train import worker_limit
    worker_limit(args.workers)
    matrix = test_matrix()
    names = args.cases or list(matrix)
    if args.smoke:
        if args.out.resolve() == Path('results/r4').resolve():
            raise ValueError('Smoke output must be in scratch, not results/r4')
    elif set(names) != set(matrix) or set(args.controllers) != set(CONTROLLERS):
        raise ValueError('Result tables require all 22 cases and all seven controllers; use --smoke for a subset')
    if any(name not in matrix for name in names):
        raise ValueError('Unknown matrix case')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    wait_for_simulation()
    jobs = [(name, controller, matrix[name][0], matrix[name][1],
             str(args.model.resolve()), str(args.library.resolve()), str(out/'traces'), args.timesteps)
            for controller in args.controllers for name in names]
    with ProcessPoolExecutor(max_workers=args.workers, mp_context=mp.get_context('spawn')) as pool:
        outputs = list(pool.map(evaluation_job, jobs))
    results = [item[0] for item in outputs]
    trace_bytes = sum(path.stat().st_size for path in (out/'traces').glob('*.npz'))
    atomic_json(out/('smoke.json' if args.smoke else 'summary.json'),
                dict(smoke=args.smoke, results=results, trace_bytes=trace_bytes,
                     model=str(args.model), timesteps=args.timesteps))
    if not args.smoke:
        plots_and_table(out, results)
    observations = [item[1] for item in outputs if item[1] is not None]
    if observations:
        atomic_episode(out/'observations.npz', {'observations': np.concatenate(observations)})
    if outputs:
        write_meta(out/'traces', out.name, args.timesteps, outputs[0][2], args.source)
    print(f'Trace bytes: {trace_bytes}', flush=True)
    return results


def parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--library', type=Path, default=Path('rl/export/policy.dll' if os.name == 'nt' else 'rl/export/policy.so'))
    parser.add_argument('--out', type=Path, default=Path('results/r4'))
    parser.add_argument('--workers', type=int, default=2 if os.name == 'nt' else 8)
    parser.add_argument('--controllers', nargs='+', choices=CONTROLLERS, default=list(CONTROLLERS))
    parser.add_argument('--cases', nargs='+')
    parser.add_argument('--timesteps', type=int, default=0)
    parser.add_argument('--source', choices=['local', 'colab'], default='local')
    parser.add_argument('--smoke', action='store_true')
    return parser


if __name__ == '__main__':
    evaluate(parser().parse_args())
