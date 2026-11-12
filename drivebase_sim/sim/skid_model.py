"""Planar, fixed-centre, isotropic Coulomb reference for a skid turn.

Speeds are signed along body x. Forces act on the chassis. This solves
zero yaw moment, not zero net translation force. A nonzero resultant is a
diagnostic that the fixed-centre assumption is inconsistent with free motion.
Torque limits flag infeasible prescribed speeds; they do not clip friction.
"""

import numpy as np
from scipy.optimize import brentq


def steady_turn(positions, normal_loads, mu, surface_speeds, radius=0.1,
                torque_limits=None, translation_velocity=(0.0, 0.0)):
    """Return yaw rate, contact forces, axle torques and feasibility flags.

    positions: (n, 2) wheel centres relative to the chosen body-frame origin.
    Sliding velocity is (-W*y-U, W*x). Zero slip uses a vanishing 1e-12
    m/s regularization; this is not a model of static friction or soil.
    """
    xy = np.asarray(positions, dtype=float)
    loads = np.asarray(normal_loads, dtype=float)
    speeds = np.asarray(surface_speeds, dtype=float)
    if xy.shape != (loads.size, 2) or speeds.shape != loads.shape:
        raise ValueError("Expected n positions, loads and signed speeds")
    if np.any(loads < 0) or mu <= 0 or radius <= 0:
        raise ValueError("Loads must be nonnegative; mu and radius positive")
    if not np.any(loads * np.sum(xy**2, axis=1)):
        raise ValueError("A loaded wheel must have a yaw moment arm")

    def forces(w):
        slip = np.column_stack((translation_velocity[0] - w * xy[:, 1] - speeds,
                                translation_velocity[1] + w * xy[:, 0]))
        return -mu * loads[:, None] * slip / np.maximum(
            np.linalg.norm(slip, axis=1)[:, None], 1e-12)

    def moment(w):
        f = forces(w)
        return float(np.sum(xy[:, 0] * f[:, 1] - xy[:, 1] * f[:, 0]))

    bound = max(1.0, 2 * np.max(np.abs(speeds)) /
                np.sqrt(np.max(np.sum(xy**2, axis=1))))
    while moment(-bound) * moment(bound) > 0:
        bound *= 2
    w = brentq(moment, -bound, bound, xtol=1e-12)
    f = forces(w)
    torque = f[:, 0] * radius
    exceeded = (np.zeros(loads.size, dtype=bool) if torque_limits is None else
                np.abs(torque) > np.asarray(torque_limits) + 1e-9)
    return dict(yaw_rate_rad_s=float(w), force_body_N=f.tolist(),
                longitudinal_force_N=f[:, 0].tolist(),
                axle_torque_Nm=torque.tolist(), exceeds_torque_limit=exceeded.tolist(),
                net_force_body_N=f.sum(axis=0).tolist(), yaw_moment_Nm=moment(w))
