"""Physical sanity check and stills of the rover scene.

usage: python preview_rover.py [--speed 0.5] [--seconds 9]

Settles the rover for 1.5 s with the motors off, then drives straight with a plain per-wheel speed loop
(torque = k (w_ref - w), clipped to the 12 N m motor peak) over a rock under the left wheels. Prints body
roll/pitch, rocker and bogie angles and wheel torques once a second; writes stills to out/.
"""
import argparse
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
SCENE = HERE / "scene" / "rover_scene.xml"
OUT = HERE / "out"
WHEELS = [f"{s}{t}" for s in "lr" for t in "fmr"]


def euler(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.5, help="m/s")
    ap.add_argument("--seconds", type=float, default=9.0)
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, 1080, 1920)
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    jw = [m.joint(f"drive_{w}").dofadr[0] for w in WHEELS]
    ja = {n: m.joint(n).qposadr[0] for n in ("rocker_l", "rocker_r", "bogie_l", "bogie_r", "diff")}
    w_ref = args.speed / 0.10
    board = m.body("control_board").id
    wheel_lf = m.body("wheel_lf").id

    def shot(name, lookat, az, el, dist):
        cam.lookat[:] = lookat
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        r.update_scene(d, camera=cam, scene_option=opt)
        imageio.imwrite(OUT / f"rover_{name}.png", r.render())

    plan = [(1.4, lambda: shot("hero", d.xpos[1] + [0.05, 0, -0.05], 145, -13, 1.75)),
            (1.42, lambda: shot("wheel", d.xpos[wheel_lf], 115, -6, 0.55)),
            (1.44, lambda: shot("board", d.xpos[board], 250, -58, 0.45)),
            (1.46, lambda: shot("front", d.xpos[1] + [0, 0, 0.05], 180, -8, 1.6))]
    rock_shot = False
    peak = np.zeros(6)
    t_print = 0.0
    while d.time < args.seconds:
        if d.time > 1.5:
            d.ctrl[:] = np.clip(4.0 * (w_ref - d.qvel[jw]), -12, 12)
            peak = np.maximum(peak, np.abs(d.ctrl))
        mujoco.mj_step(m, d)
        while plan and d.time >= plan[0][0]:
            plan.pop(0)[1]()
        if not rock_shot and abs(np.degrees(d.qpos[ja["bogie_l"]])) > 8 and d.time > 2:
            rock_shot = True
            shot("rock_side", d.xpos[1] + [0, 0.2, -0.15], 270, -4, 1.55)
            shot("rock_quarter", d.xpos[1] + [0, 0.1, -0.1], 215, -10, 1.6)
        if d.time >= t_print:
            t_print += 1.0
            roll, pitch = euler(d.qpos[3:7])
            a = {k: np.degrees(d.qpos[v]) for k, v in ja.items()}
            print(f"t={d.time:4.1f}s x={d.qpos[0]:5.2f} roll={roll:6.2f} pitch={pitch:6.2f} "
                  f"rocker {a['rocker_l']:6.2f}/{a['rocker_r']:6.2f} bogie {a['bogie_l']:6.2f}/{a['bogie_r']:6.2f} "
                  f"diff {a['diff']:5.2f} torque {np.round(d.ctrl, 1)}", flush=True)
    shot("wide", d.xpos[1] + [0.8, 0, 0], 150, -18, 4.2)
    print("peak |torque| per wheel (N m):", np.round(peak, 2), " rock shot:", rock_shot)


if __name__ == "__main__":
    main()
