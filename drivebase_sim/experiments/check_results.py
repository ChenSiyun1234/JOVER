"""Check saved electrical histories and 50 Hz replay traces without simulation."""
import json
from pathlib import Path
import numpy as np
from sim.motor import KE, KT, RESISTANCE
from .run_all import CASES, JOINTS


def main():
    result = Path(__file__).resolve().parents[1] / "results"
    for name, (duration, _, _) in CASES.items():
        payload = json.loads((result / f"{name}.json").read_text())
        summary, arr = payload["summary"], np.array(payload["samples"])
        assert arr.shape == (round(duration*1000), 27)
        assert np.isfinite(arr).all()
        np.testing.assert_allclose(np.diff(arr[:,0]), .001, atol=1e-12)
        np.testing.assert_allclose(arr[:,9:15], arr[:,3:9]/KT)
        limits = np.minimum(12, KT*np.maximum(0, arr[:,1,None] - KE*np.abs(arr[:,15:21]))/RESISTANCE)
        assert np.all(np.abs(arr[:,3:9]) <= limits+1e-8)
        power = np.maximum(0, arr[:,9:15]**2*RESISTANCE + arr[:,3:9]*arr[:,15:21]).sum(axis=1)
        np.testing.assert_allclose(arr[:,2], power/arr[:,1], atol=1e-8)
        np.testing.assert_allclose(arr[:,1], 25.2-.04*arr[:,2], atol=1e-6)
        assert arr[:,1].min() == summary["minimum_bus_voltage_V"]
        assert bool(np.any(arr[:,1] < 20)) == summary["undervoltage"]
        np.testing.assert_allclose(np.max(np.abs(arr[:,9:15]), axis=0), summary["peak_winding_current_A"])
        np.testing.assert_allclose(arr[:,21:27].sum(axis=0)*.001, summary["time_at_torque_limit_s"])
        if name.startswith("D2") or name == "D3":
            with np.load(result / f"{name}_trace.npz", allow_pickle=False) as trace:
                assert trace["chassis_pose"].shape == (round(duration*50), 7)
                assert trace["joint_angles"].shape == (round(duration*50), len(JOINTS))
                assert tuple(trace["joint_names"]) == JOINTS
                assert str(trace["quaternion_order"]) == "wxyz"
                assert np.isfinite(trace["chassis_pose"]).all()
                assert np.isfinite(trace["joint_angles"]).all()
                np.testing.assert_allclose(np.diff(trace["time"]), .02, atol=1e-12)
                np.testing.assert_allclose(np.linalg.norm(trace["chassis_pose"][:,3:], axis=1), 1, atol=1e-8)
        print(f"{name}: electrical histories, limits, sag and replay checks OK")
    print("6 experiment artifacts checked")


if __name__ == "__main__":
    main()
