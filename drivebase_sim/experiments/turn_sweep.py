"""Serial R2 experiments; source scenes and R1 results are never written.

Run: python -m experiments.turn_sweep [--cases NAME ...] [--report-only]
"""

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import mujoco
import numpy as np

from experiments.run_all import SCENE, ROOT
from sim.can_dm import CanBus, encode_velocity
from sim.drive import Drive, WHEELS, YawRatePI
from sim.motor import Motor, torque_limit, KE
from sim.power import Battery
from sim.resources import wait_for_simulation
from sim.skid_model import steady_turn

OUT = ROOT / "results/turn"


def speed_clip_duration(electrical, supply):
    """Count held targets at the voltage limit used when they were sampled."""
    sample_indices = np.arange(len(electrical)) // 10 * 10
    sampled_voltage = np.r_[supply, electrical[:-1, 1]][sample_indices]
    return float(np.sum(np.any(np.abs(electrical[:, 27:33]) >=
                              sampled_voltage[:, None]/KE-1e-6, axis=1))*.001)


@dataclass(frozen=True)
class Case:
    ground: str = "terrain"
    mu: float = 0.9
    condim: int = 4
    shape: str = "cylinder"
    supply: float = 25.2
    slip: float = 1.25
    yaw: float = 30.0
    feedback: bool = False
    # Diagnostic only: change contact impedance, never baseline parameters.
    hard_contact: bool = False
    impratio: float = 1.0


def cases():
    out = {}
    for ground in ("terrain", "plane"):
        base = Case(ground=ground)
        for mu in (.3, .5, .7, .9):
            out[f"{ground}_mu{mu:.1f}"] = replace(base, mu=mu)
        out[f"{ground}_condim3"] = replace(base, condim=3)
        out[f"{ground}_sphere"] = replace(base, shape="sphere")
        out[f"{ground}_nominal"] = replace(base, supply=22.2)
        for slip in (1.0, 1.5, 2.0):
            out[f"{ground}_slip{slip:.2f}"] = replace(base, slip=slip)
        out[f"{ground}_sphere_c3"] = replace(base, shape="sphere", condim=3)
        out[f"{ground}_sphere_c3_mu0.5"] = replace(base, shape="sphere", condim=3, mu=.5)
    out["plane_sphere_c3_hard"] = Case(ground="plane", shape="sphere", condim=3,
                                                hard_contact=True)
    out["plane_sphere_c3_mu0.5_hard"] = Case(ground="plane", shape="sphere", condim=3,
                                                    mu=.5, hard_contact=True)
    for ratio in (10., 100.):
        out[f"plane_sphere_c3_mu0.5_imp{int(ratio)}"] = Case(
            ground="plane", shape="sphere", condim=3, mu=.5, impratio=ratio)
    out["plane_mu0.5_imp100"] = Case(ground="plane", mu=.5, impratio=100.)
    for ground, mu in (("terrain", .9), ("terrain", .5), ("plane", .9)):
        for yaw in (30., 60.):
            for feedback in (False, True):
                if yaw == 30 and not feedback:
                    continue  # Already in the one-factor sweep.
                name = f"{ground}_mu{mu:.1f}_{int(yaw)}_{'fb' if feedback else 'open'}"
                out[name] = Case(ground=ground, mu=mu, yaw=yaw, feedback=feedback)
    return out


def load_scene(case):
    root = ET.parse(SCENE).getroot()
    for key in ("meshdir", "texturedir"):
        root.find("compiler").set(key, str(SCENE.parent))
    for asset in root.find("asset"):
        if "file" in asset.attrib:
            asset.set("file", str(SCENE.parent / asset.get("file")))
    world = root.find("worldbody")
    if case.ground == "plane":
        for geom in list(world):
            if geom.tag == "geom" and geom.get("type") in ("hfield", "mesh"):
                world.remove(geom)
        world.insert(0, ET.Element("geom", name="terrain", type="plane",
            size="20 20 .1", pos="0 0 .311", condim="3", friction=".9 .01 .0001"))
    for wheel in WHEELS:
        geom = root.find(f".//geom[@name='tire_{wheel}']")
        geom.set("condim", str(case.condim))
        if case.shape == "sphere":
            geom.set("type", "sphere")
            geom.set("size", ".1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    model.opt.timestep = .002
    model.opt.impratio = case.impratio
    model.geom_friction[:, 0] = case.mu
    if case.hard_contact:
        model.geom_solimp[:, :2] = [.999, .9999]
    return model


def contacts(model, data, tyre_ids, chassis):
    """Contact-frame forces and material-point velocities at a physics tick."""
    rotation = data.xmat[chassis].reshape(3, 3)
    origin = data.xpos[chassis]
    loads, counts = np.zeros(6), np.zeros(6)
    slip, abs_slip, location, forces = [np.zeros((6, 3)) for _ in range(4)]
    torsion = np.zeros(6)
    penetration = np.zeros(6)
    utilisation = np.zeros(6)
    force_yaw = np.zeros(6)
    nonplanar_slip = np.zeros((6, 2))
    velocity = np.zeros((6, 6))
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for k, gid in enumerate(tyre_ids):
        # mj_step integrates qvel after computing cached cvel. Use the current
        # generalized velocity with the contact-configuration Jacobian rather
        # than mj_objectVelocity's pre-integration velocity cache.
        mujoco.mj_jac(model, data, jacp, jacr, data.geom_xpos[gid], model.geom_bodyid[gid])
        velocity[k, :3] = jacr @ data.qvel
        velocity[k, 3:] = jacp @ data.qvel
    wrench = np.zeros(6)
    for ci, contact in enumerate(data.contact):
        for k, gid in enumerate(tyre_ids):
            if gid not in contact.geom or contact.efc_address < 0:
                continue
            mujoco.mj_contactForce(model, data, ci, wrench)
            sign = 1 if contact.geom2 == gid else -1
            frame = contact.frame.reshape(3, 3)
            normal = frame[0] * sign
            # Wheel axle is body y; tangent x points in rolling direction.
            axis = data.xmat[model.geom_bodyid[gid]].reshape(3, 3)[:, 1]
            longitudinal = np.cross(axis, normal)
            longitudinal /= np.linalg.norm(longitudinal)
            lateral = np.cross(normal, longitudinal)
            vel = velocity[k, 3:] + np.cross(velocity[k, :3],
                                            contact.pos - data.geom_xpos[gid])
            components = np.array([vel @ longitudinal, vel @ lateral, vel @ normal])
            n = wrench[0]
            p_body = (contact.pos-origin) @ rotation
            v_body = data.qvel[:3] @ rotation
            wheel_joint = model.body_jntadr[model.geom_bodyid[gid]]
            omega = data.qvel[model.jnt_dofadr[wheel_joint]]
            planar = np.array([v_body[0]-data.qvel[5]*p_body[1]-omega*model.geom_size[gid, 0],
                               v_body[1]+data.qvel[5]*p_body[0], 0.0]) @ rotation.T
            nonplanar_slip[k] += n*np.abs(components[:2] -
                                          [planar @ longitudinal, planar @ lateral])
            loads[k] += n
            counts[k] += 1
            slip[k] += n * components
            abs_slip[k] += n * np.abs(components)
            location[k] += n * ((contact.pos - origin) @ rotation)
            force = sign * (wrench[:3] @ frame)
            forces[k] += force @ rotation
            force_yaw[k] += (np.cross(contact.pos - origin, force) @ rotation)[2]
            torsion[k] += (sign * (wrench[3:] @ frame) @ rotation)[2]
            penetration[k] += n * max(0, -contact.dist)
            utilisation[k] += np.linalg.norm(wrench[1:3]) / contact.friction[0]
    weight = np.maximum(loads, 1e-12)
    return dict(load_N=loads, count=counts, slip_m_s=slip / weight[:, None],
        absolute_slip_m_s=abs_slip / weight[:, None],
        location_body_m=location / weight[:, None], force_body_N=forces,
        torsional_yaw_Nm=torsion, penetration_m=penetration / weight,
        friction_utilisation=utilisation / weight, force_yaw_Nm=force_yaw,
        absolute_nonplanar_slip_m_s=nonplanar_slip/weight[:, None])


def run(name, case):
    wait_for_simulation()
    started = time.monotonic()
    model = load_scene(case)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    drive = Drive.from_scene(model, data)
    drive.slip_multiplier = case.slip
    controller = YawRatePI()
    chassis = model.body("chassis").id
    gyro = model.sensor("gyro").adr[0]
    vi = [model.joint("drive_" + w).dofadr[0] for w in WHEELS]
    qi = [model.joint("drive_" + w).qposadr[0] for w in WHEELS]
    tyres = [model.geom("tire_" + w).id for w in WHEELS]
    motors, battery, bus = [Motor() for _ in WHEELS], Battery(open_circuit=case.supply), CanBus()
    torque, command = np.zeros(6), np.zeros(6)
    voltage = case.supply
    electrical, trace, contact_rows, centres = [], [], [], []
    static_loads = None
    for step in range(4750):
        elapsed = step * .002 - 1.5
        if step == 750:
            start = data.qpos[:3].copy()
            static_loads = contacts(model, data, tyres, chassis)["load_N"].tolist()
        if step % 5 == 0:
            yaw = np.radians(case.yaw) if elapsed >= 0 else 0.0
            command = (controller.speeds(drive, 0, yaw, data.sensordata[gyro + 2], voltage)
                       if case.feedback and elapsed >= 0 else drive.speeds(0, yaw))
        tick_torques = []
        for substep in range(2):
            tick = step * 2 + substep
            now = tick * .001
            if tick % 10 == 0:
                for mid, target in enumerate(command, 1):
                    bus.send(encode_velocity(mid, float(target)), now)
            states = {mid: (float((data.qpos[qi[mid-1]] + np.pi) % (2*np.pi) - np.pi),
                float(data.qvel[vi[mid-1]]), float(torque[mid-1]),
                9 if battery.undervoltage else 1) for mid in range(1, 7)}
            bus.advance(now, states)
            speeds = data.qvel[vi].copy()
            for mid, motor in enumerate(motors, 1):
                motor.target = bus.targets[mid]
            voltage, current, torque = battery.solve([m.demand(w) for m, w in zip(motors, speeds)], speeds)
            for motor, w, t in zip(motors, speeds, torque):
                motor.accept(w, t)
            tick_torques.append(torque.copy())
            if elapsed >= 0:
                limits = np.array([torque_limit(w, voltage) for w in speeds])
                electrical.append([elapsed + substep * .001, voltage, current, *torque,
                    *speeds, *(np.abs(torque) >= limits - 1e-6), *limits, *command])
        if elapsed >= 0:
            rotation = data.xmat[chassis].reshape(3, 3)
            q = data.qpos[3:7]
            heading = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            trace.append([elapsed, data.sensordata[gyro+2], heading, *data.qpos[:3],
                          controller.measurement, *q, rotation[2, 2], *(data.qvel[:3] @ rotation)])
            contact_rows.append(contacts(model, data, tyres, chassis))
            centres.append((data.geom_xpos[tyres] - data.xpos[chassis]) @ rotation)
        data.ctrl[:] = np.mean(tick_torques, axis=0)
        mujoco.mj_step(model, data)
    arr, tr = np.asarray(electrical), np.asarray(trace)
    selected, cs = arr[:, 0] >= 1, tr[:, 0] >= 1
    contact_arrays = {key: np.array([row[key] for row in contact_rows]) for key in contact_rows[0]}
    means = {key: values[cs].mean(axis=0).tolist() for key, values in contact_arrays.items()}
    # Conditional contact quantities use normal-load weighting across both
    # contact points and time. Airborne samples must not bias locations to zero.
    weights = contact_arrays["load_N"][cs]
    for key in ("slip_m_s", "absolute_slip_m_s", "location_body_m",
                "penetration_m", "friction_utilisation", "absolute_nonplanar_slip_m_s"):
        values = contact_arrays[key][cs]
        w = weights[:, :, None] if values.ndim == 3 else weights
        means[key] = (np.sum(values*w, axis=0)/np.maximum(w.sum(axis=0), 1e-12)).tolist()
    a = arr[selected]
    heading_rate = float(np.degrees(np.unwrap(tr[:, 2])[cs][-1] -
                        np.unwrap(tr[:, 2])[cs][0])/(tr[cs, 0][-1]-tr[cs, 0][0]))
    summary = dict(name=name, case=asdict(case), wheel_order=WHEELS,
        duration_s=8, settle_s=1.5, averaging_window_s=[1, 8],
        mean_yaw_rate_deg_s=heading_rate,
        mean_body_gyro_z_deg_s=float(np.degrees(tr[cs, 1].mean())),
        heading_yaw_rate_deg_s=heading_rate,
        yaw_rmse_deg_s=float(np.sqrt(np.mean((np.degrees(tr[cs, 1])-case.yaw)**2))),
        yaw_ratio=heading_rate/case.yaw,
        start_position_m=start.tolist(),
        start_quaternion_wxyz=tr[0, 7:11].tolist(),
        minimum_body_up_dot_world_up=float(tr[:, 11].min()),
        upright=bool(np.all(tr[:, 11] > .5)),
        drift_m=float(np.linalg.norm(data.qpos[:2]-start[:2])),
        mean_torque_Nm=a[:, 3:9].mean(axis=0).tolist(),
        mean_absolute_torque_Nm=np.abs(a[:, 3:9]).mean(axis=0).tolist(),
        peak_torque_Nm=np.max(np.abs(arr[:, 3:9]), axis=0).tolist(),
        time_at_torque_envelope_s=(arr[:, 15:21].sum(axis=0)*.001).tolist(),
        steady_time_at_torque_envelope_s=(a[:, 15:21].sum(axis=0)*.001).tolist(),
        mean_wheel_speed_rad_s=a[:, 9:15].mean(axis=0).tolist(),
        mean_torque_limit_Nm=a[:, 21:27].mean(axis=0).tolist(),
        mean_command_rad_s=a[:, 27:33].mean(axis=0).tolist(),
        wheel_speed_clip_time_s=speed_clip_duration(arr, case.supply),
        mean_bus_current_A=float(a[:, 2].mean()), peak_bus_current_A=float(arr[:, 2].max()),
        minimum_voltage_V=float(arr[:, 1].min()), mean_voltage_V=float(a[:, 1].mean()),
        static_load_N=static_loads, contact_means=means,
        maximum_contact_count=contact_arrays["count"][cs].max(axis=0).tolist(),
        no_contact_fraction=(contact_arrays["count"][cs] == 0).mean(axis=0).tolist(),
        all_wheels_airborne_fraction=float(np.mean(weights.sum(axis=1) == 0)),
        minimum_normal_load_N=weights.min(axis=0).tolist(),
        maximum_normal_load_N=weights.max(axis=0).tolist(),
        mean_wheel_centres_body_m=np.array(centres)[cs].mean(axis=0).tolist(),
        mean_translation_velocity_body_m_s=tr[cs, 12:15].mean(axis=0).tolist(),
        can_sent=bus.sent, can_dropped=bus.dropped, can_received=bus.received,
        controller=dict(kp=controller.kp, ki=controller.ki, noise_std_rad_s=controller.noise_std,
                        sample_period_s=controller.period, seed=controller.seed),
        scene_sha256=hashlib.sha256(SCENE.read_bytes()).hexdigest(),
        physics_dt_s=.002, electronics_dt_s=.001, contact_sample_dt_s=.002,
        numpy_version=np.__version__, mujoco_version=mujoco.__version__,
        mass_kg=float(model.body_mass.sum()), wall_seconds=time.monotonic()-started)
    if case.ground == "plane":
        summary["reference"] = steady_turn(np.array(summary["mean_wheel_centres_body_m"])[:, :2],
            means["load_N"], case.mu, a[:, 9:15].mean(axis=0)*drive.radius,
            radius=drive.radius, torque_limits=a[:, 21:27].mean(axis=0))
        summary["command_reference"] = steady_turn(np.array(summary["mean_wheel_centres_body_m"])[:, :2],
            means["load_N"], case.mu, a[:, 27:33].mean(axis=0)*drive.radius,
            radius=drive.radius, torque_limits=a[:, 21:27].mean(axis=0))
        summary["translating_reference"] = steady_turn(np.array(summary["mean_wheel_centres_body_m"])[:, :2],
            means["load_N"], case.mu, a[:, 9:15].mean(axis=0)*drive.radius,
            radius=drive.radius, torque_limits=a[:, 21:27].mean(axis=0),
            translation_velocity=tr[cs, 12:14].mean(axis=0))
        reference_samples = []
        for j in np.flatnonzero(cs)[::10]:
            # A flight phase has no quasi-static contact equilibrium.
            if contact_arrays["load_N"][j].sum() == 0:
                continue
            speeds_j = arr[round(tr[j, 0]*1000), 9:15] * drive.radius
            kw = dict(positions=np.array(centres)[j, :, :2],
                      normal_loads=contact_arrays["load_N"][j], mu=case.mu,
                      surface_speeds=speeds_j, radius=drive.radius)
            fixed = steady_turn(**kw)["yaw_rate_rad_s"]
            moving = steady_turn(**kw, translation_velocity=tr[j, 12:14])["yaw_rate_rad_s"]
            reference_samples.append([tr[j, 0], fixed, moving])
        summary["samplewise_reference_mean_deg_s"] = np.degrees(np.mean(reference_samples, axis=0)[1:]).tolist()
        summary["samplewise_reference_eligible_fraction"] = len(reference_samples)/len(np.flatnonzero(cs)[::10])
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    np.savez_compressed(OUT / f"{name}.npz", electrical=arr, trace=tr,
        reference_samples=np.array(reference_samples if case.ground == "plane" else []),
        wheel_centres_body_m=np.array(centres), **contact_arrays,
        electrical_columns=np.array(["time_s", "voltage_V", "current_A"] +
            [f"{q}_{w}" for q in ("torque_Nm", "speed_rad_s", "at_limit", "limit_Nm", "target_rad_s") for w in WHEELS]),
        trace_columns=np.array(["time_s", "gyro_z_rad_s", "heading_rad", "x_m", "y_m", "z_m", "noisy_gyro_rad_s",
                                "qw", "qx", "qy", "qz", "body_up_dot_world_up", "vx_body_m_s", "vy_body_m_s", "vz_body_m_s"]))
    print(f"{name}: yaw={summary['mean_yaw_rate_deg_s']:.3f} deg/s "
          f"ratio={summary['yaw_ratio']:.3f} drift={summary['drift_m']:.3f} m "
          f"I={summary['mean_bus_current_A']:.2f} A wall={summary['wall_seconds']:.1f} s", flush=True)
    return summary


def report():
    summaries = [json.loads((OUT / f"{name}.json").read_text()) for name in cases()
                 if (OUT / f"{name}.json").exists()]
    (OUT / "summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    sweep = [s for s in summaries if not s["case"]["feedback"] and s["case"]["yaw"] == 30]
    fig, axes = plt.subplots(1, 2, figsize=(15, 9), layout="constrained")
    for ax, ground in zip(axes, ("terrain", "plane")):
        rows = [s for s in sweep if s["case"]["ground"] == ground]
        y = np.arange(len(rows))
        bars = ax.barh(y, [s["mean_yaw_rate_deg_s"] for s in rows], color="#28658C", label="Simulation")
        for bar, row in zip(bars, rows):
            if not row["upright"]:
                bar.set_color("#909090"); bar.set_edgecolor("#303030"); bar.set_hatch("///")
        if ground == "plane":
            ax.plot([np.degrees(s["reference"]["yaw_rate_rad_s"]) for s in rows], y,
                    "o", color="#C87821", label="Coulomb, measured speeds")
        ax.axvline(30, color="#303030", ls="--", label="Command")
        ax.set(yticks=y, yticklabels=[s["name"].removeprefix(ground+"_") for s in rows],
               xlabel="Mean heading rate, 1-8 s (deg/s)", title=ground.capitalize(), xlim=(0, 36))
        ax.invert_yaxis(); ax.grid(axis="x", alpha=.2)
        handles, labels = ax.get_legend_handles_labels()
        if any(not row["upright"] for row in rows):
            handles.append(Patch(facecolor="#909090", edgecolor="#303030", hatch="///"))
            labels.append("Overturned fixture")
        ax.legend(handles, labels, loc="lower right", fontsize=8)
    fig.savefig(OUT / "sweep.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True, layout="constrained")
    for r, (ground, mu) in enumerate((("terrain", .9), ("terrain", .5), ("plane", .9))):
        for col, yaw in enumerate((30, 60)):
            ax = axes[r, col]
            for fb, color, ls in ((False, "#28658C", "--"), (True, "#C87821", "-")):
                name = f"{ground}_mu{mu:.1f}" if yaw == 30 and not fb else f"{ground}_mu{mu:.1f}_{yaw}_{'fb' if fb else 'open'}"
                path = OUT / f"{name}.npz"
                if path.exists():
                    with np.load(path) as a:
                        tr = a["trace"]
                    ax.plot(tr[:, 0], np.degrees(tr[:, 1]), color=color, ls=ls,
                            label="Yaw PI" if fb else "Open loop")
            ax.axhline(yaw, color="#303030", ls=":", label="Command")
            ax.set(title=f"{ground}, mu={mu}, command {yaw} deg/s", ylabel="Gyro z (deg/s)", ylim=(-2, 70))
            ax.grid(alpha=.2)
    axes[0, 0].legend(); axes[-1, 0].set_xlabel("Time after turn start (s)"); axes[-1, 1].set_xlabel("Time after turn start (s)")
    fig.savefig(OUT / "feedback.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
    colors = ["#28658C", "#C87821", "#737D35"]
    for col, ground in enumerate(("terrain", "plane")):
        baseline_path = OUT / f"{ground}_mu0.9.npz"
        if not baseline_path.exists():
            axes[0, col].set_title(f"{ground} baseline not run")
            continue
        with np.load(baseline_path) as z:
            tr, arr, loads = z["trace"], z["electrical"], z["load_N"]
        load_time = tr[:, 0].reshape(-1, 50).mean(axis=1)
        load_means = loads.reshape(-1, 50, 6).mean(axis=1)
        for i, wheel in enumerate(WHEELS):
            style = "-" if i < 3 else "--"
            axes[0, col].plot(load_time, load_means[:, i], color=colors[i % 3], ls=style,
                              lw=.8, label=wheel.upper())
            axes[1, col].plot(arr[::20, 0], arr[::20, 9+i], color=colors[i % 3], ls=style, lw=.8)
        axes[0, col].set(title=f"{ground}, mu=0.9", ylabel="Normal load (N; 0.1 s means)", ylim=(0, 250))
        axes[1, col].set(ylabel="Wheel speed (rad/s)", xlabel="Time after turn start (s)")
        for value in (-2.3588, 2.3588):
            axes[1, col].axhline(value, color="#303030", ls=":", lw=1)
        axes[0, col].legend(ncol=3, fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.savefig(OUT / "loads_speeds.png", dpi=150); plt.close(fig)
    print(f"R2 report: {len(summaries)} cases, sweep.png, feedback.png, loads_speeds.png", flush=True)
    return summaries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", nargs="+")
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args()
    available = cases()
    if not args.report_only:
        for name in args.cases or available:
            run(name, available[name])
    report()


if __name__ == "__main__":
    main()
