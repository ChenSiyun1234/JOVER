"""Serial R3 batches; all modified fixtures are in memory, histories ignored.

Run python -m experiments.r3 [--cases NAME ...] [--report-only].
"""

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import time
import tempfile
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from experiments.run_all import ROOT, SCENE
from experiments.turn_sweep import Case as R2Case, load_scene as r2_scene, contacts
from sim.can_dm import CanBus, encode_velocity
from sim.drive import Drive, WHEELS, YawRatePI, TorqueGovernor
from sim.motor import DatasheetMotor, ProtectedMotor, IDEAL_KT, torque_limit
from sim.power import Battery, DatasheetBattery
from sim.resources import wait_for_simulation
from sim.rocker_bogie import predict_loads, geometry

OUT = ROOT / "results/r3"


@dataclass(frozen=True)
class Case:
    ground: str = "terrain"
    mu: float = .9
    slip: float = 1.0
    behaviour: str = "limit"
    kt: float = 1.0
    power: str = "copper"
    supply: float = 25.2
    yaw: float = 30.0
    velocity: float = 0.0
    lock: str = "none"
    controller: str = "open"
    stop: str = "none"
    driver_timeout: float | None = None
    stall: bool = False
    settle_limit: bool = False


def cases():
    result = {}
    for ground in ("terrain", "plane"):
        for mu in (.5, .9):
            for slip in (1., 1.25):
                for behaviour in ("limit", "trip"):
                    result[f"{ground}_mu{mu}_s{slip}_{behaviour}"] = Case(
                        ground=ground, mu=mu, slip=slip, behaviour=behaviour)
    for mu in (.11, .16, .2, .35, .45, .55):
        result[f"ground_mu{mu}"] = Case(mu=mu)
    for lock in ("rockers", "bogies", "both"):
        result[f"mechanism_{lock}"] = Case(ground="plane", lock=lock)
    result["mechanism_clockwise"] = Case(ground="plane", yaw=-30)
    result["mechanism_straight"] = Case(ground="plane", yaw=0, velocity=.2)
    for kt, label in ((1., "catalogueKt"), (IDEAL_KT, "idealKt")):
        for supply in (25.2, 22.2):
            for power in ("copper", "lumped"):
                result[f"D5_{supply}_{power}_{label}"] = Case(
                    kt=kt, supply=supply, power=power, stall=True, velocity=10, yaw=0)
        result[f"sensitivity_turn_{label}"] = Case(kt=kt)
    for mu in (.9, .45, .2):
        for behaviour in ("limit", "trip"):
            for controller in ("open", "governor", "pi", "arc"):
                result[f"fix_mu{mu}_{behaviour}_{controller}"] = Case(
                    mu=mu, behaviour=behaviour, controller=controller,
                    velocity=.2 if controller == "arc" else 0,
                    settle_limit=behaviour == "trip")
    for stop, timeout in (("cmd_vel", None), ("can", .5), ("can", None), ("cmd_vel", .5)):
        result[f"timeout_{stop}_{timeout}"] = Case(stop=stop, driver_timeout=timeout)
        result[f"timeout_mu0.45_{stop}_{timeout}"] = Case(mu=.45, stop=stop, driver_timeout=timeout)
    result["sensitivity_turn_lumped"] = Case(power="lumped")
    result["sensitivity_turn_22.2"] = Case(supply=22.2)
    return result


def load_scene(case):
    model = r2_scene(R2Case(ground=case.ground, mu=case.mu))
    if case.lock != "none" or case.stall:
        # Serialize only into scratch, never into viz. MuJoCo's saved XML keeps
        # absolute asset paths from the in-memory R2 loader.
        scratch = Path(tempfile.gettempdir()) / "rover_r3_fixture.xml"
        mujoco.mj_saveLastXML(str(scratch), model)
        root = ET.parse(scratch).getroot()
        eq = root.find("equality")
        # Remove suspension DOFs instead of using soft equality locks. This
        # actually holds the links rigidly at their zero-angle configuration.
        fixed = []
        if case.lock in ("rockers", "both"):
            fixed += ["rocker_l", "rocker_r", "diff"]
        if case.lock in ("bogies", "both"):
            fixed += ["bogie_l", "bogie_r"]
        for body in root.findall(".//body"):
            for joint in list(body.findall("joint")):
                if joint.get("name") in fixed:
                    body.remove(joint)
        for constraint in list(eq):
            if constraint.get("joint1") in fixed or constraint.get("joint2") in fixed:
                eq.remove(constraint)
        names = []
        if case.stall:
            names += ["drive_"+w for w in WHEELS]
            root.find("worldbody").append(ET.Element("geom", name="stall_wall", type="box",
                pos=".625 0 .6", size=".1 1.2 .6", friction="2 .1 .1"))
        for name in names:
            eq.append(ET.Element("joint", joint1=name, polycoef="0 0 0 0 0", solref=".002 1"))
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        scratch.unlink()
    # The trip bracket has a 13 A ceiling, not the old 12 Nm actuator clip.
    model.actuator_ctrlrange[:] = [-13*case.kt, 13*case.kt]
    return model


def run(name, case):
    wait_for_simulation()
    started = time.monotonic()
    model = load_scene(case)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    drive = Drive.from_scene(model, data)
    drive.slip_multiplier = case.slip
    motor_model = DatasheetMotor(kt=case.kt,
        current_limit=9.8 if case.behaviour == "limit" else 13, power_bound=case.power)
    motors = [ProtectedMotor(behaviour=case.behaviour, kt=case.kt, timeout=case.driver_timeout) for _ in WHEELS]
    if case.settle_limit:
        for m in motors:
            m.behaviour = "limit"
    battery, bus = DatasheetBattery(open_circuit=case.supply), CanBus()
    pi, governor = YawRatePI(), TorqueGovernor(limit_torque=case.kt*9.8)
    vi = [model.joint("drive_"+w).dofadr[0] for w in WHEELS]
    qi = [model.joint("drive_"+w).qposadr[0] for w in WHEELS]
    tyres = [model.geom("tire_"+w).id for w in WHEELS]
    chassis = model.body("chassis").id
    gyro = model.sensor("gyro").adr[0]
    torque, command = np.zeros(6), np.zeros(6)
    voltage = case.supply
    rows, traces, loads, forces, predictions, unloaded_predictions = [], [], [], [], [], []
    trips, feedback_errors = {}, {}
    duration = 2.0 if case.stall else 8.0
    geom = None
    for step in range(round((duration+1.5)/.002)):
        elapsed = step*.002-1.5
        if step == 750:
            start = data.qpos[:3].copy()
            static = contacts(model, data, tyres, chassis)["load_N"]
            geom = geometry(model, data)
            if case.settle_limit:
                for m in motors:
                    m.enable(1.5)
                    m.behaviour = case.behaviour
        if step % 5 == 0:
            yaw = np.radians(case.yaw) if elapsed >= 0 else 0.
            velocity = case.velocity if elapsed >= 0 else 0.
            if case.stop == "cmd_vel" and elapsed >= 3.5:
                yaw, velocity = 0., 0.
            if case.controller == "governor" and elapsed >= 0:
                yaw = governor.update(yaw, bus.feedback)
            command = (pi.speeds(drive, velocity, yaw, data.sensordata[gyro+2], voltage)
                       if case.controller == "pi" and elapsed >= 0 else drive.speeds(velocity, yaw))
        tick_torques = []
        for substep in range(2):
            tick = step*2+substep
            now = tick*.001
            if tick % 10 == 0 and not (case.stop == "can" and elapsed >= 3):
                for mid, target in enumerate(command, 1):
                    bus.send(encode_velocity(mid, float(target)), now)
            states = {mid: (float((data.qpos[qi[mid-1]]+np.pi)%(2*np.pi)-np.pi),
                float(data.qvel[vi[mid-1]]), float(torque[mid-1]), motors[mid-1].error) for mid in range(1, 7)}
            bus.advance(now, states)
            speeds = data.qvel[vi].copy()
            for mid, m in enumerate(motors, 1):
                if bus.last_command[mid] is not None and bus.last_command[mid] > m.last_command:
                    m.command(bus.targets[mid], bus.last_command[mid])
            demands = [m.check(w, voltage, now) for m, w in zip(motors, speeds)]
            voltage, current, torque = battery.solve(demands, speeds, motor_model)
            # UV protection responds to the solved loaded voltage, then latches;
            # save the triggering minimum even if the unloaded bus recovers.
            if voltage < 15:
                for m in motors:
                    if m.error == 1:
                        m.error = 9
                torque[:] = 0
            for i, (m, w, t) in enumerate(zip(motors, speeds, torque)):
                m.accept(w, t)
                if m.error != 1 and WHEELS[i] not in trips:
                    trips[WHEELS[i]] = dict(time_s=now-1.5, error=m.error,
                        demanded_current_A=float(abs(m.demand(w))/case.kt))
            for mid, feedback in bus.feedback.items():
                if feedback["error"] != 1:
                    feedback_errors[str(mid)] = feedback["error"]
            tick_torques.append(torque.copy())
            if elapsed >= 0:
                rows.append([now-1.5, voltage, current, *torque, *speeds,
                    *(np.abs(torque/case.kt) >= 9.8-1e-6), *[m.error for m in motors],
                    governor.scale, *command])
        if elapsed >= 0:
            q = data.qpos[3:7]
            heading = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            traces.append([elapsed, heading, data.sensordata[gyro+2], *data.qpos[:3], data.xmat[chassis, 8]])
            c = contacts(model, data, tyres, chassis)
            loads.append(c["load_N"])
            forces.append(c["force_body_N"])
            if case.ground == "plane" and case.lock == "none":
                rotation = data.xmat[chassis].reshape(3, 3)
                # Geometry-only nominal support points below the loaded axles.
                # Do not use normal-force-weighted contact locations: that
                # would leak the loads being predicted into the geometry.
                points = data.geom_xpos[tyres].copy()
                points[:, 2] = .311
                tangent = c["force_body_N"] @ rotation.T
                tangent[:, 2] = 0
                predicted, residual = predict_loads(model, data, points, tangent)
                no_tangent, _ = predict_loads(model, data, points, np.zeros((6,3)))
                unloaded_predictions.append([elapsed, *no_tangent])
                predictions.append([elapsed, *predicted, *c["force_body_N"][:, 2],
                                    float(np.max(np.abs(residual)))])
        data.ctrl[:] = np.mean(tick_torques, axis=0)
        mujoco.mj_step(model, data)
    a, tr, ld, ff, pred = map(np.asarray, (rows, traces, loads, forces, predictions))
    select, ts = a[:, 0] >= 1, tr[:, 0] >= 1
    heading = np.unwrap(tr[:, 1])
    summary = dict(name=name, case=asdict(case), wheel_order=WHEELS,
        settle_s=1.5, duration_s=duration, averaging_window_s=[1, duration],
        yaw_deg_s=float(np.degrees(heading[ts][-1]-heading[ts][0])/(tr[ts, 0][-1]-tr[ts, 0][0])),
        yaw_rmse_deg_s=float(np.sqrt(np.mean((np.degrees(tr[ts, 2])-case.yaw)**2))),
        final_second_yaw_deg_s=float(np.degrees(tr[tr[:, 0]>=duration-1, 2].mean())),
        peak_current_A=np.max(np.abs(a[:, 3:9]/case.kt), axis=0).tolist(),
        mean_current_A=(a[select,3:9]/case.kt).mean(axis=0).tolist(),
        mean_wheel_speed_rad_s=a[select,9:15].mean(axis=0).tolist(),
        mean_target_rad_s=a[select,28:34].mean(axis=0).tolist(),
        current_limit_time_s=(a[:, 15:21].sum(axis=0)*.001).tolist(),
        mean_bus_A=float(a[select, 2].mean()), peak_bus_A=float(a[:, 2].max()),
        minimum_voltage_V=float(a[:, 1].min()), margin_to_15V=float(a[:, 1].min()-15),
        static_load_N=static.tolist(), mean_load_N=ld[ts].mean(axis=0).tolist(),
        mean_contact_force_body_N=ff[ts].mean(axis=0).tolist(),
        drift_m=float(np.linalg.norm(data.qpos[:2]-start[:2])), trips=trips,
        feedback_faults=feedback_errors, minimum_upright=float(tr[:, 6].min()),
        upright=bool(np.all(tr[:,6]>.5)),
        final_governor_scale=float(governor.scale), geometry=geom,
        post_stop_yaw_deg_s=float(np.degrees(tr[tr[:, 0]>=4, 2].mean())) if duration>4 else None,
        post_stop_peak_current_A=float(np.max(np.abs(a[a[:, 0]>=4, 3:9]/case.kt))) if duration>4 else None,
        scene_sha256=hashlib.sha256(SCENE.read_bytes()).hexdigest(),
        physics_dt_s=.002, electronics_dt_s=.001, command_dt_s=.01,
        wall_s=time.monotonic()-started)
    if len(pred):
        p = pred[pred[:, 0]>=1]
        summary["analytic"] = dict(predicted_mean_N=p[:, 1:7].mean(axis=0).tolist(),
            sampled_actual_mean_N=p[:, 7:13].mean(axis=0).tolist(),
            per_wheel_rmse_N=np.sqrt(np.mean((p[:, 1:7]-p[:, 7:13])**2, axis=0)).tolist(),
            mean_error_N=(p[:, 1:7]-p[:, 7:13]).mean(axis=0).tolist(),
            negative_prediction_fraction=float(np.mean(np.any(p[:, 1:7]<0, axis=1))),
            maximum_solve_residual=float(p[:, 13].max()))
        no_tangent = np.asarray(unloaded_predictions)
        summary["analytic"]["without_tangential_force_mean_N"] = no_tangent[no_tangent[:,0]>=1,1:].mean(axis=0).tolist()
    if trips:
        first = min(v["time_s"] for v in trips.values())
        after = tr[:, 0] >= max(first+.5, 0)
        summary["yaw_after_first_trip_deg_s"] = float(np.degrees(tr[after, 2].mean())) if np.any(after) else None
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/f"{name}.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    np.savez_compressed(OUT/f"{name}.npz", electrical=a, trace=tr, load_N=ld, force_body_N=ff, analytic=pred,
        electrical_columns=np.array(["time_s", "voltage_V", "bus_A"]+
            [f"{v}_{w}" for v in ("torque_Nm", "speed_rad_s", "at_current_limit", "error") for w in WHEELS]+
            ["governor_scale"]+["target_"+w for w in WHEELS]),
        trace_columns=np.array(["time_s", "heading_rad", "gyro_z_rad_s", "x_m", "y_m", "z_m", "upright"]))
    print(f"{name}: yaw={summary['yaw_deg_s']:.3f}, Ibus={summary['mean_bus_A']:.3f}, "
          f"Vmin={summary['minimum_voltage_V']:.3f}, drift={summary['drift_m']:.3f}, trips={trips}, wall={summary['wall_s']:.1f}s", flush=True)
    return summary


def report():
    summaries = [json.loads((OUT/f"{n}.json").read_text()) for n in cases() if (OUT/f"{n}.json").exists()]
    (OUT/"summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    speeds = np.linspace(0, 29, 500)
    fig, ax = plt.subplots(figsize=(9, 5), layout="constrained")
    for voltage, color in ((22.2, "#3c6c93"), (24., "#a25f23"), (25.2, "#527b3e")):
        ax.plot(speeds*60/(2*np.pi), [torque_limit(w, voltage) for w in speeds], "--", color=color, label=f"Catalogue {voltage} V")
        ax.plot(speeds*60/(2*np.pi), DatasheetMotor().limit(speeds, voltage), color=color, label=f"Datasheet {voltage} V")
        ax.plot(speeds*60/(2*np.pi), DatasheetMotor(kt=IDEAL_KT).limit(speeds, voltage), ":", color=color, label=f"Ideal Kt {voltage} V")
    ax.set(xlabel="Output speed (rpm)", ylabel="Available torque (Nm)", ylim=(0, 13))
    ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.savefig(OUT/"envelopes.png", dpi=150); plt.close(fig)
    old = []
    for supply in (25.2, 22.2):
        v, i, t = Battery(open_circuit=supply).solve([100]*6, [0]*6)
        old.append(dict(supply_V=supply, voltage_V=v, bus_A=i, torque_Nm=t.tolist(), margin_to_15V=v-15))
    (OUT/"old_stall.json").write_text(json.dumps(old, indent=2), encoding="utf-8")
    by_name = {s["name"]: s for s in summaries}
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    for ax, ground in zip(axes, ("terrain", "plane")):
        for behaviour, style in (("limit", "-o"), ("trip", "--s")):
            for slip in (1., 1.25):
                rows = [by_name[f"{ground}_mu{mu}_s{slip}_{behaviour}"] for mu in (.5,.9)
                        if f"{ground}_mu{mu}_s{slip}_{behaviour}" in by_name]
                ax.plot([s["case"]["mu"] for s in rows], [s["yaw_deg_s"] for s in rows], style,
                        label=f"{behaviour}, multiplier {slip}")
        ax.axhline(30, color="black", ls=":")
        ax.set(title=ground, xlabel="Isotropic friction coefficient", ylabel="Heading rate (deg/s)")
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.savefig(OUT/"turns.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), layout="constrained")
    x = np.arange(6)
    for j, name in enumerate(("plane_mu0.9_s1.0_limit", "mechanism_clockwise", "mechanism_rockers", "mechanism_bogies", "mechanism_both")):
        if name in by_name:
            s = by_name[name]
            axes[0].plot(x, s["mean_load_N"], "-o", label=name)
    for name in ("mechanism_straight", "plane_mu0.9_s1.0_limit"):
        if name in by_name and "analytic" in by_name[name]:
            s = by_name[name]
            axes[1].plot(x, s["mean_load_N"], "-o", label=name+" simulated")
            axes[1].plot(x, s["analytic"]["predicted_mean_N"], "--s", label=name+" predicted")
    for ax in axes:
        ax.set(xticks=x, xticklabels=[w.upper() for w in WHEELS], ylabel="Mean wheel load (N)")
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.savefig(OUT/"mechanism.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True, layout="constrained")
    for row, mu in enumerate((.9,.45,.2)):
        for col, behaviour in enumerate(("limit", "trip")):
            ax = axes[row,col]
            for controller in ("open", "governor", "pi", "arc"):
                path = OUT/f"fix_mu{mu}_{behaviour}_{controller}.npz"
                if path.exists():
                    with np.load(path) as z:
                        tr = z["trace"]
                    # 0.1 s block means retain low-frequency control response.
                    smooth = tr[:, [0,2]].reshape(-1,50,2).mean(axis=1)
                    s = by_name.get(f"fix_mu{mu}_{behaviour}_{controller}")
                    label = controller + (" (overturned)" if s and not s["upright"] else "")
                    ax.plot(smooth[:,0], np.degrees(smooth[:,1]), label=label)
            ax.axhline(30, color="black", ls=":")
            ax.set(title=f"mu={mu}, {behaviour}", ylabel="Gyro z (deg/s)")
            ax.grid(alpha=.2); ax.legend(fontsize=8)
    axes[-1,0].set_xlabel("Time after turn start (s)"); axes[-1,1].set_xlabel("Time after turn start (s)")
    fig.savefig(OUT/"fixes.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    for name in ("timeout_cmd_vel_None", "timeout_can_0.5", "timeout_can_None", "timeout_cmd_vel_0.5"):
        path = OUT/f"{name}.npz"
        if path.exists():
            with np.load(path) as z:
                tr, a = z["trace"], z["electrical"]
            smooth = tr[:,[0,2]].reshape(-1,50,2).mean(axis=1)
            axes[0].plot(smooth[:,0], np.degrees(smooth[:,1]), label=name)
            axes[1].plot(a[::20,0], a[::20,2])
            axes[2].plot(a[::20,0], np.count_nonzero(a[::20,21:27] == 1,axis=1))
    for ax in axes:
        ax.axvline(3, color="black", ls=":"); ax.axvline(3.5, color="black", ls="--"); ax.grid(alpha=.2)
    axes[0].set_ylabel("Gyro z (deg/s)"); axes[0].legend(fontsize=8)
    axes[1].set_ylabel("Bus current (A)"); axes[2].set(ylabel="Enabled motors", xlabel="Time after turn start (s)")
    fig.savefig(OUT/"timeouts.png", dpi=150); plt.close(fig)
    print(f"R3 report: {len(summaries)} cases", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", nargs="+")
    parser.add_argument("--report-only", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if not args.report_only:
        for name in args.cases or cases():
            if args.resume and (OUT/f"{name}.json").exists():
                continue
            run(name, cases()[name])
    report()


if __name__ == "__main__":
    main()
