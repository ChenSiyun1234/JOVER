"""Quasi-static wheel load prediction by constrained virtual work.

All geometry and masses come from the loaded model. Unknowns are six world
vertical forces, not contact-normal magnitudes on sloping terrain. This model
is evaluated on the plane. Tangential forces are measured inputs; measured
vertical forces are never used on the right hand side.
"""

import mujoco
import numpy as np
from .drive import WHEELS


def solve_loads(vertical_virtual_work, known_virtual_work):
    """Solve A N + b = 0 without clipping infeasible negative predictions."""
    return np.linalg.solve(vertical_virtual_work, -np.asarray(known_virtual_work))


def predict_loads(model, data, points, tangential_forces):
    """Three chassis balances, two bogie moments, one differential moment.

    Allowed rocker displacement is dqL=1, dqR=-1, dqDiff=c, where c is
    read from the scene equality. Its virtual work cancels ideal reactions.
    Wheel spin is held fixed in these modes, so hub torque is internal.
    """
    basis = np.zeros((model.nv, 6))
    basis[2, 0], basis[3, 1], basis[4, 2] = 1, 1, 1
    for col, name in ((3, "bogie_l"), (4, "bogie_r"), (5, "rocker_l")):
        basis[model.joint(name).dofadr[0], col] = 1
    basis[model.joint("rocker_r").dofadr[0], 5] = -1
    diff_id = model.joint("diff").id
    eq = next(i for i in range(model.neq) if model.eq_obj1id[i] == diff_id)
    basis[model.joint("diff").dofadr[0], 5] = model.eq_data[eq, 1]
    a, b = np.zeros((6, 6)), np.zeros(6)
    jac, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for i, wheel in enumerate(WHEELS):
        mujoco.mj_jac(model, data, jac, jacr, points[i], model.body("wheel_"+wheel).id)
        virtual = jac @ basis
        a[:, i] = virtual[2]
        b += virtual.T @ tangential_forces[i]
    for body in range(1, model.nbody):
        mujoco.mj_jacBodyCom(model, data, jac, jacr, body)
        b += (jac @ basis).T @ (model.body_mass[body] * model.opt.gravity)
    loads = solve_loads(a, b)
    return loads, a @ loads + b


def geometry(model, data):
    """Expose the actual pivot, axle and centre-of-mass positions used."""
    names = ("rocker_l", "rocker_r", "bogie_l", "bogie_r")
    # These four scene hinges have zero local joint offsets, so body origins
    # retain the pivot locations even when a counterfactual removes the hinge.
    return dict(pivots_world_m={n: data.xpos[model.body(n).id].tolist() for n in names},
                axles_world_m={w: data.xpos[model.body("wheel_"+w).id].tolist() for w in WHEELS},
                total_mass_kg=float(model.body_mass.sum()))
