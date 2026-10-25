"""Known failure cases under the map NIS gate and re-association (v0.5.11, owner 2026-10-25): does the
safety net fire where the geometry is wrong, or does re-association make a known failure look safe?

    python tools/failure_cases.py      -> reports/<version>/failure_cases_100.json

Pressure gradient 30 Pa/km (S1, S3; the ESKF may take the wrong branch) and twice the assumed map
error as a fixed bias (lateral 4 m, length +1 %; S1, S4, S5). ESKF variants base / gate /
gate+reassoc; the supervisor (class rule, majority monitor) with the gate+reassoc ESKF.
Seeds 0-99.
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402

V = {"base": {"map_nis_gate": False, "map_reassoc": False}, "gate": {"map_nis_gate": True, "map_reassoc": False},
     "gate+reassoc": {"map_nis_gate": True, "map_reassoc": True}}
CASES = [("S1", ("baro_tunnel_30pa",), None, "pressure30"), ("S3", ("baro_tunnel_30pa",), None, "pressure30"),
         ("S1", (), (4.0, 1.01), "map2x"), ("S4", (), (4.0, 1.01), "map2x"), ("S5", (), (4.0, 1.01), "map2x")]
SEEDS = np.arange(100)


def eskf_job(args):
    var, key, ov, me, name = args
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import eskf_experiment
    r = eskf_experiment(key, SEEDS, ov, eskf_cfg=EskfConfig(**V[var]), map_err=me, keep_tracks=True)
    T = r["_tracks"]
    kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
    e = T["e_s"][:, kx]
    return f"eskf/{var}/{key}/{name}", {
        "portal_rmse": r["real_exit"]["rmse"], "portal_p95": r["real_exit"]["p95_abs"],
        "failures_gt_100m": int((np.abs(e) > 100).sum()), "nees": r["consistency_real_exit"]["mean_norm_sq"],
        "branch": r["branch_confirm"]["branch_reported_frac"], "main": r["branch_confirm"]["main_reported_frac"],
        "late_gt_1s_k1": r["callout_k"]["1.0"]["late_gt_1s_frac"],
        "map_rejected": r["events"].get("map_rejected", 0), "map_reassoc": r["events"].get("map_reassoc", 0)}


def sup_job(args):
    key, ov, me, name = args
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import supervisor_experiment
    r = supervisor_experiment(key, SEEDS, ov, rules=("class",), eskf_cfg=EskfConfig(**V["gate+reassoc"]), map_err=me)
    c = r["rules"]["class"]
    return f"supervisor/{key}/{name}", {m: {k_: c[m][k_] for k_ in ("portal", "nees", "callout_mean_s", "late_gt_1s",
                                                                     "early_gt_5s", "alarm_time_frac", "alarm_runs_frac")
                                            if k_ in c[m]} for m in ("none", "majority")}


if __name__ == "__main__":
    res = {"version": tunnelsim.__version__, "seeds": [0, 99], "runs": {}}
    d = ROOT / "reports" / tunnelsim.__version__
    with ProcessPoolExecutor(2) as ex:
        futs = [ex.submit(eskf_job, (v, k, ov, me, n)) for v in V for k, ov, me, n in CASES] + \
               [ex.submit(sup_job, (k, ov, me, n)) for k, ov, me, n in CASES]
        for f in futs:
            tag, out = f.result()
            res["runs"][tag] = out
            print(tag, json.dumps(out), flush=True)
            (d / "failure_cases_100.json").write_text(json.dumps(res, indent=1) + "\n")
