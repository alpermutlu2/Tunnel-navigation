"""1D map-constrained EKF vs the ESKF, same sensors and seeds (v0.5.9, owner 2026-10-24).

    python tools/compare1d.py                 # 30 seeds -> reports/<version>/reference1d_30.json
    python tools/compare1d.py --seeds 10 S1 S4

Columns: 1D with the map grade, 1D with a map without grade, 1D with the filter's mount pitched
2° wrong, ESKF; the same for the low-end IMU (MPU9250 class). Along-track error rms at the diverge
and at the portal, (e/σ)² at the portal, callouts > 1 s late, exit step p95, run time.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.experiments import ekf1d_experiment, eskf_experiment, simulate_run  # noqa: E402

ALL = ("S1", "S2", "S3", "S4", "S5", "S6", "S1-5km")
VARIANTS = (("1d", {}), ("1d_nograde", {"use_grade": False}), ("1d_mount2", {"mount_err_deg": (0.0, 2.0, 0.0)}))


def pick(r):
    out = {"at_diverge_rmse": r.get("at_diverge", {}).get("rmse"), "portal_rmse": r["real_exit"]["rmse"],
           "portal_p95": r["real_exit"]["p95_abs"],
           "portal_nees": r["consistency_real_exit"]["mean_norm_sq"],
           "late_gt_1s": r.get("callout", {}).get("late_gt_1s_frac"),
           "exit_step_p95": r["recovery"]["max_step_jump_m"]["p95_abs"]}
    if "_time_s" in r:
        out["time_s"] = r["_time_s"]
    return out


def run(keys, seeds):
    sid = np.arange(seeds)
    res = {"version": tunnelsim.__version__, "seeds": seeds, "runs": {}}
    for key in keys:
        for imu, ov in (("note10", ()), ("lowend", ("imu_lowend",))):
            row = {}
            sim = simulate_run(key, sid, ov)
            for name, kw in VARIANTS:
                if imu == "lowend" and name != "1d":
                    continue
                t0 = time.perf_counter()
                r = ekf1d_experiment(key, sid, ov, sim=sim, **kw)
                r["_time_s"] = round(time.perf_counter() - t0, 1)
                row[name] = pick(r)
            t0 = time.perf_counter()
            r = eskf_experiment(key, sid, ov)
            r["_time_s"] = round(time.perf_counter() - t0, 1)
            row["eskf"] = pick(r)
            res["runs"][f"{key}/{imu}"] = row
            print(key, imu, {k: (v["at_diverge_rmse"], v["portal_rmse"], v["portal_nees"]) for k, v in row.items()}, flush=True)
    return res


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("scenarios", nargs="*", default=list(ALL))
    p.add_argument("--seeds", type=int, default=30)
    a = p.parse_args()
    res = run(a.scenarios, a.seeds)
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"reference1d_{a.seeds}.json"
    f.write_text(json.dumps(res, indent=1) + "\n")
    print(f"wrote {f}")
