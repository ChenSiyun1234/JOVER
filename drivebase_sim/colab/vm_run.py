"""Verify, install and run R4 on a Linux Colab CPU VM."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import zipfile


def run(command, cwd=None):
    print('RUN', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), cwd=cwd, check=True)


def main():
    drive_root = Path(os.environ.get('DRIVE_ROOT', '/content/drive'))
    base = drive_root/'MyDrive/RoverSim'
    run_id = os.environ['RUN_ID']
    budget = int(os.environ.get('TOTAL_TIMESTEPS', '3000000'))
    workers = int(os.environ.get('N_ENVS', '8'))
    bundle = Path(os.environ.get('BUNDLE', str(base/'code/rover-r4.zip')))
    expected = bundle.with_suffix('.zip.sha256').read_text().split()[0]
    if hashlib.sha256(bundle.read_bytes()).hexdigest() != expected:
        raise ValueError('Bundle SHA256 mismatch')
    code = Path('/content/rover-r4')
    code.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(bundle) as archive:
        archive.extractall(code)
    for name, digest in json.loads((code/'manifest.json').read_text()).items():
        if hashlib.sha256((code/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Payload SHA256 mismatch: '+name)
    # Installation is exclusively on the Colab VM, never the local machine.
    run([sys.executable, '-m', 'pip', 'install', 'torch==2.14.0',
         '--index-url', 'https://download.pytorch.org/whl/cpu'])
    run([sys.executable, '-m', 'pip', 'install', 'mujoco==4.0.0', 'gymnasium==1.4.0',
         'stable-baselines3==2.9.2', 'ziglang==0.17.1', 'numpy==2.5.2',
         'matplotlib==3.11.1', 'psutil==7.2.2', 'tzdata==2026.3'])
    run_dir = Path('/content/run')/run_id
    command = [sys.executable, '-m', 'rl.train', '--run-dir', run_dir,
        '--steps', budget, '--workers', workers, '--source', 'colab',
        '--live-every', os.environ.get('LIVE_EVERY', '100000'),
        '--checkpoint-every', os.environ.get('CHECKPOINT_EVERY', '250000')]
    if os.environ.get('RESUME'):
        command += ['--resume', os.environ['RESUME']]
    command += os.environ.get('TRAIN_ARGS', '').split()
    run(command, code)
    model = run_dir/'best.zip'
    export_dir = run_dir/'export'
    results = code/'results/r4'
    failure = None
    try:
        run([sys.executable, '-m', 'rl.export', '--model', model, '--out', export_dir], code)
        run([sys.executable, '-m', 'rl.evaluate', '--model', model,
             '--library', export_dir/'policy.so', '--workers', workers,
             '--out', results, '--source', 'colab', '--timesteps',
             json.loads((run_dir/'training.json').read_text())['timesteps']], code)
        run([sys.executable, '-m', 'rl.export', '--model', model, '--out', export_dir,
             '--observations', results/'observations.npz'], code)
        shutil.copytree(results, run_dir/'results/r4', dirs_exist_ok=True)
    except subprocess.CalledProcessError as error:
        # Archive the trained checkpoints even if a later step fails.
        failure = str(error)
    destination = base/'runs'/run_id
    destination.mkdir(parents=True, exist_ok=True)

    def read(path):
        return json.loads(path.read_text()) if path.exists() else None

    metrics = dict(run_id=run_id, bundle_sha256=expected, failure=failure,
        training=read(run_dir/'training.json'), validation=read(run_dir/'validation.json'),
        export=read(export_dir/'export.json'), evaluation=read(results/'summary.json'),
        mesh_proof=read(code/'mesh_proof.json'))
    part = destination/'metrics.json.part'
    part.write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    os.replace(part, destination/'metrics.json')
    part = destination/'run.tar.gz.part'
    with tarfile.open(part, 'w:gz') as archive:
        archive.add(run_dir, arcname=run_id)
    os.replace(part, destination/'run.tar.gz')
    print('VM RUN COMPLETE', destination, 'FAILED AFTER TRAINING: ' + failure if failure else '', flush=True)
    if failure:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
