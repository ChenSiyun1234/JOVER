"""Run R1 in one process, without altering the source scene.

Usage: python -m experiments.run_all [--cases D1 D2_30 D2_60 D3 D4 D5]
"""

import argparse
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from sim.can_dm import CanBus, encode_velocity
from sim.drive import Drive, WHEELS
from sim.motor import Motor, KT, torque_limit
from sim.power import Battery
from sim.resources import wait_for_simulation

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "viz/scene/rover_scene.xml"
JOINTS = ("rocker_l", "rocker_r", "bogie_l", "bogie_r", "diff") + tuple("drive_" + w for w in WHEELS)
CASES = {"D1": (10, 0.5, 0), "D2_30": (10, 0, np.radians(30)),
         "D2_60": (10, 0, np.radians(60)), "D3": (12, 0.3, 0),
         "D4": (14, 0.3, 0), "D5": (2, 10, 0)}


def load_scene(stall=False):
    if not stall:
        return mujoco.MjModel.from_xml_path(str(SCENE))
    tree = ET.parse(SCENE)
    root = tree.getroot()
    root.find("compiler").set("meshdir", str(SCENE.parent))
    root.find("compiler").set("texturedir", str(SCENE.parent))
    for asset in root.find("asset"):
        if "file" in asset.attrib:
            asset.set("file", str(SCENE.parent / asset.get("file")))
    # Use welded wheel joints as the ideal rigid fixed-obstacle limit. A broad
    # front wall makes the obstacle explicit; wheel locks prevent soil spin.
    root.find("worldbody").append(ET.Element("geom", name="stall_wall", type="box",
        pos="0.625 0 0.6", size="0.10 1.2 0.6", friction="2 0.1 0.1"))
    eq = root.find("equality")
    for wheel in WHEELS:
        eq.append(ET.Element("joint", name="lock_" + wheel, joint1="drive_" + wheel,
                             polycoef="0 0 0 0 0", solref="0.002 1"))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def run_case(name):
    wait_for_simulation()
    wall_start = time.monotonic()
    model = load_scene(name == "D5")
    model.opt.timestep = 0.002
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if name == "D4":
        data.qpos[0] = 3.4
        hit = np.zeros(1, dtype=np.int32)
        height = 5 - mujoco.mj_ray(model, data, np.array([3.4, 0., 5.]),
                                 np.array([0., 0., -1.]), None, 1, -1, hit)
        data.qpos[2] = height + 0.40
    mujoco.mj_forward(model, data)
    drive = Drive.from_scene(model, data)
    velocity_indices = [model.joint("drive_" + w).dofadr[0] for w in WHEELS]
    angle_indices = [model.joint("drive_" + w).qposadr[0] for w in WHEELS]
    joint_indices = [model.joint(j).qposadr[0] for j in JOINTS]
    # Settle on the actual terrain with the same electronics active.
    motors, bus, battery = [Motor() for _ in WHEELS], CanBus(), Battery()
    torque = np.zeros(6)
    records, poses, angles, trace_time = [], [], [], []
    rock_contact_steps = np.zeros(6, dtype=int)
    rock_id = model.geom("rock00").id
    tyre_ids = [model.geom("tire_" + w).id for w in WHEELS]
    rock_crossing_wheel_height = np.full(6, np.nan)
    total, speed, yaw = CASES[name]
    start_pose = hold_pose = None
    for step in range(round((total + 1.5) / .002)):
        elapsed = step * .002 - 1.5
        if step == 750:
            start_pose = data.qpos[:7].copy()
        if name == "D4" and step == 2750:
            hold_pose = data.qpos[:7].copy()
        command = drive.speeds(speed, yaw) if elapsed >= 0 else np.zeros(6)
        if name == "D4" and elapsed >= 4:
            command[:] = 0
        tick_torques = []
        for substep in range(2):
            now = (step * 2 + substep) * .001
            if (step * 2 + substep) % 10 == 0:
                for mid, target in enumerate(command, 1):
                    bus.send(encode_velocity(mid, float(target)), now)
            states = {mid: (float((data.qpos[angle_indices[mid-1]] + np.pi) % (2*np.pi) - np.pi),
                            float(data.qvel[velocity_indices[mid-1]]), float(torque[mid-1]),
                            9 if battery.undervoltage else 1) for mid in range(1, 7)}
            bus.advance(now, states)
            speeds = data.qvel[velocity_indices].copy()
            for mid, motor in enumerate(motors, 1):
                motor.target = bus.targets[mid]
            demands = [motor.demand(w) for motor, w in zip(motors, speeds)]
            voltage, current, torque = battery.solve(demands, speeds)
            for motor, w, t in zip(motors, speeds, torque):
                motor.accept(w, t)
            tick_torques.append(torque.copy())
            if elapsed >= 0:
                limits = [torque_limit(w, voltage) for w in speeds]
                records.append([elapsed + substep * .001, voltage, current,
                                *torque, *(torque / KT), *speeds,
                                *[float(lim > 1e-6 and abs(t) >= lim - 1e-6)
                                  for t, lim in zip(torque, limits)]])
        data.ctrl[:] = np.mean(tick_torques, axis=0)
        mujoco.mj_step(model, data)
        if name == "D3" and elapsed >= 0:
            contacting = set()
            for contact in data.contact:
                if rock_id in contact.geom:
                    contacting.update(int(g) for g in contact.geom if g != rock_id)
            for i, gid in enumerate(tyre_ids):
                rock_contact_steps[i] += gid in contacting
                centre = data.geom_xpos[gid]
                if centre[0] >= model.geom_pos[rock_id, 0] and np.isnan(rock_crossing_wheel_height[i]):
                    rock_crossing_wheel_height[i] = centre[2]
        if elapsed >= 0 and (step - 750 + 1) % 10 == 0:
            trace_time.append(elapsed + .002)
            poses.append(data.qpos[:7].copy())
            angles.append(data.qpos[joint_indices].copy())
    arr, pose = np.array(records), np.array(poses)
    yaw_trace = np.unwrap(np.arctan2(2*(pose[:,3]*pose[:,6]+pose[:,4]*pose[:,5]),
                                    1-2*(pose[:,5]**2+pose[:,6]**2)))
    summary = dict(name=name, duration_s=total, command_m_s=speed, command_yaw_rad_s=yaw,
        wheel_order=WHEELS, separation_m=drive.separation, slip_multiplier=drive.slip_multiplier,
        mass_kg=float(model.body_mass.sum()), minimum_bus_voltage_V=float(arr[:,1].min()),
        maximum_bus_current_A=float(arr[:,2].max()), mean_bus_current_A=float(arr[:,2].mean()),
        undervoltage=bool(np.any(arr[:,1] < 20)),
        peak_winding_current_A=np.max(np.abs(arr[:,9:15]), axis=0).tolist(),
        peak_torque_Nm=np.max(np.abs(arr[:,3:9]), axis=0).tolist(),
        time_at_torque_limit_s=(arr[:,21:27].sum(axis=0)*.001).tolist(),
        final_wheel_speed_rad_s=arr[-100:,15:21].mean(axis=0).tolist(),
        displacement_m=(data.qpos[:3]-start_pose[:3]).tolist(),
        mean_yaw_rate_deg_s=float(np.degrees(yaw_trace[-1]-yaw_trace[0])/(trace_time[-1]-trace_time[0])),
        final_pose_wxyz=data.qpos[:7].tolist(), start_pose_wxyz=start_pose.tolist(),
        can_sent=bus.sent, can_dropped=bus.dropped, can_feedback_received=bus.received,
        scene_sha256=hashlib.sha256(SCENE.read_bytes()).hexdigest(),
        fixture="front wall and six rigid wheel locks" if name == "D5" else "unmodified generated scene",
        physics_dt_s=.002, electronics_dt_s=.001, command_period_s=.010,
        battery_open_circuit_V=battery.open_circuit, command_drop_rate=bus.drop_rate,
        numpy_version=np.__version__, mujoco_version=mujoco.__version__,
        wall_seconds=time.monotonic()-wall_start)
    if name == "D3":
        rock = model.geom_pos[model.geom("rock00").id]
        summary.update(rock_centre_m=rock.tolist(),
                       cleared_rock=bool(data.qpos[0] - .52 > rock[0]),
                       rock_contact_duration_per_wheel_s=(rock_contact_steps*.002).tolist(),
                       wheel_centre_height_at_rock_x_m=[None if np.isnan(x) else float(x) for x in rock_crossing_wheel_height],
                       maximum_chassis_height_gain_m=float(pose[:,2].max()-start_pose[2]))
    if name == "D4":
        hold = arr[:,0] >= 4
        summary.update(hold_mean_absolute_winding_current_A=np.abs(arr[hold,9:15]).mean(axis=0).tolist(),
                       hold_displacement_m=(data.qpos[:3]-hold_pose[:3]).tolist(),
                       hold_horizontal_slip_m=float(np.linalg.norm(data.qpos[:2]-hold_pose[:2])),
                       hold_mean_absolute_speed_rad_s=np.abs(arr[hold,15:21]).mean(axis=0).tolist())
    result = ROOT / "results"
    result.mkdir(exist_ok=True)
    payload = dict(summary=summary, columns=["time_s", "bus_voltage_V", "bus_current_A"] +
        [f"{quantity}_{wheel}" for quantity in ("torque_Nm", "winding_current_A", "speed_rad_s", "at_limit") for wheel in WHEELS],
        samples=arr.tolist())
    (result / f"{name}.json").write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    if name.startswith("D2") or name == "D3":
        np.savez_compressed(result / f"{name}_trace.npz", time=np.array(trace_time),
                            chassis_pose=pose, joint_angles=np.array(angles), joint_names=np.array(JOINTS),
                            quaternion_order="wxyz")
    fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    for ax, values, label in zip(axes[:3], (arr[:,3:9], arr[:,9:15], arr[:,15:21]),
                                ("Torque (N m)", "Winding current (A)", "Speed (rad/s)")):
        ax.plot(arr[::10,0], values[::10], linewidth=.8)
        ax.set_ylabel(label)
    axes[0].legend(WHEELS, ncol=6)
    axes[3].plot(arr[::10,0], arr[::10,2]); axes[3].set_ylabel("Bus current (A)")
    axes[4].plot(arr[::10,0], arr[::10,1]); axes[4].axhline(20, color="r", linestyle="--")
    axes[4].set_ylabel("Bus voltage (V)"); axes[4].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle(name); fig.tight_layout(); fig.savefig(result / f"{name}.png", dpi=130); plt.close(fig)
    print(json.dumps(summary, allow_nan=False), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    args = parser.parse_args()
    summaries = [run_case(name) for name in args.cases]
    (ROOT / "results/summary.json").write_text(json.dumps(summaries, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
