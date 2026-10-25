"""Three filters on the same sensor data and seeds (v0.5.11, owner 2026-10-25): the 1D map-constrained
EKF (4 states, map grade), the 10-state map-constrained filter (no map grade) and the ESKF (16 + 6).

    python tools/compare_filters.py            # 50 seeds -> reports/<version>/compare_filters_50.json

Note10 and low-end (MPU9250 class); for the Note10 also the filter's mount pitched 2° wrong
(1D and 10-state), and the straight-on negative control for the 10-state filter.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.experiments import ekf1d_experiment, ekf10_experiment, eskf_experiment, simulate_run  # noqa: E402

ALL = ("S1", "S2", "S3", "S4", "S5", "S6", "S1-5km")


def pick(r):
    if "portal_horizontal_m" in r:
        return {"portal_horizontal_rmse": r["portal_horizontal_m"]["rmse"]}
    ck = r.get("callout_k", {}).get("1.0") or {}
    return {"diverge": r.get("at_diverge", {}).get("rmse"), "portal": r["real_exit"]["rmse"],
            "portal_p95": r["real_exit"]["p95_abs"], "nees": r["consistency_real_exit"]["mean_norm_sq"],
            "late_gt_1s": ck.get("late_gt_1s_frac", r.get("callout", {}).get("late_gt_1s_frac")),
            "exit_step_p95": r["recovery"]["max_step_jump_m"]["p95_abs"]}


def run(keys, seeds):
    sid = np.arange(seeds)
    res = {"version": tunnelsim.__version__, "seeds": seeds, "runs": {}}
    for key in keys:
        for imu, ov in (("note10", ()), ("lowend", ("imu_lowend",))):
            sim = simulate_run(key, sid, ov)
            row = {"1d": pick(ekf1d_experiment(key, sid, ov, sim=sim)),
                   "ekf10": pick(ekf10_experiment(key, sid, ov, sim=sim)),
                   "eskf": pick(eskf_experiment(key, sid, ov))}
            if imu == "note10":
                row["1d_mount2"] = pick(ekf1d_experiment(key, sid, ov, sim=sim, mount_err_deg=(0.0, 2.0, 0.0)))
                row["ekf10_mount2"] = pick(ekf10_experiment(key, sid, ov, sim=sim, mount_err_deg=(0.0, 2.0, 0.0)))
                row["ekf10_straight_on"] = pick(ekf10_experiment(key, sid, ov, branch=False))
            res["runs"][f"{key}/{imu}"] = row
            print(key, imu, json.dumps(row), flush=True)
    return res


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    seeds = int(next((x.split("=")[1] for x in sys.argv[1:] if x.startswith("--seeds=")), 50))
    res = run(a or ALL, seeds)
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    (d / f"compare_filters_{seeds}.json").write_text(json.dumps(res, indent=1) + "\n")
