# R4 replay interchange format

All poses are world-frame SI units. Quaternion order is **w, x, y, z**.
Every publication writes a sibling filename ending in `.part`, closes it,
then calls `os.replace(part, destination)`. Readers ignore `.part` files and
reopen replaced metadata/progress files. A trace file is complete when visible;
the entire directory is not one transactional snapshot.

`<run_dir>/live/meta.json` contains:

- `run_id`: string.
- `total_timesteps`: requested absolute total including resumed steps.
- `showcase_cases`: objects `hard` and `mid` with all Scenario settings.
- `controller_names`: open, governor, pi, pi_backoff, wheel_guard, learned, learned_c.
- `body_names`: fixed MuJoCo body order for every body owning a visual geom.
  The world body is named `world` if present. Collision-only tyre geoms in
  group 3 do not create additional body entries; wheel visual geoms do.
- `scene_file`: `viz/scene/rover_scene.xml`, relative to the bundle/repository.
- `source`: `local` or `colab`.
- `sample_hz`: 50; `quaternion_order`: `wxyz`.

Showcases use terrain, 30 deg/s turn in place, 25.2 V and no dropped frames.
`hard` uses mu 0.9 and trip; `mid` uses mu 0.45 and limit. Gyro noise is
0.01 rad/s; fixed seeds are 510001 and 510002. Every reset includes the 1.5 s
R3 settle, omitted from the saved 8 s window. Reference files are generated
once per run for all five hand controllers:
`ref_<controller>_<case>.npz`. Learned snapshots are
`ep_<timesteps:010d>_<case>.npz`, every 100k steps and after the final update.
The progress CSV is copied atomically from the SB3 logger to `live/progress.csv`.
SB3's original `<run_dir>/progress.csv` is its normal mutable log file.

Each NPZ can be loaded with `numpy.load(path, allow_pickle=False)`. Required keys:

| Key | Shape/type | Meaning |
| --- | --- | --- |
| `t` | (400,) float32 | 0, 0.02, ..., 7.98 seconds, start-of-step sample |
| `pos` | (400, nb, 3) float32 | body origins in world frame, metres |
| `quat` | (400, nb, 4) float32 | world body orientations, wxyz |
| `yaw_rate` | (400,) float32 | latest IMU-site gyro-z truth, deg/s |
| `yaw_cmd` | (400,) float32 | commanded yaw, deg/s |
| `current` | (400, 6) float32 | latest signed torque/Kt, A, LF LM LR RF RM RR |
| `error` | (400, 6) int32 | actual driver codes; 1 enabled, 9 UV, 10 overcurrent |
| `bus_v` | (400,) float32 | latest solved bus voltage, V |
| `timesteps` | scalar int64 | policy snapshot timestep; reference files use zero |
| `ep_return` | scalar float64 | complete episode return, including fault/tip penalties |
| `mean_yaw_deg_s` | scalar float64 | heading-based mean yaw over 1-7.998 s, as R3 |
| `drift_m` | scalar float64 | integral of absolute local lateral velocity over 8 s |
| `faults` | scalar Unicode string | JSON object: wheel -> first `time_s`, `error` |

Body transforms are recomputed using kinematics on separate data at the exact
sample qpos. The live rig's physics/sensor ordering is not changed. Electrical
and gyro fields are the latest completed samples (up to 2 ms old); they are
diagnostic truth and are not the policy observation. The policy uses noisy IMU
and delayed, quantized CAN feedback. No wheels are automatically re-enabled
after faults. Traces always contain 400 rows, even if drivers trip or the rover
tips; only training episodes terminate early.

Under `results/r4/traces/`, test files use
`<controller>_<matrix_case>.npz` for every `learned`, `pi_backoff` and `wheel_guard` case, with
the identical schema. Their `meta.json` supplies body order. Its showcase cases
describe the live convention; each test scenario/seed is authoritative in
`results/r4/summary.json`. The summary also records total NPZ trace bytes.
`results/r4/observations.npz` holds scaled host observations for the separate
pointwise C/PyTorch equivalence check; it is not a viewer trace. All these NPZ
files are generated locally and excluded from git.
