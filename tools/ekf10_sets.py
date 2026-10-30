"""10-state filter on six seed sets (v0.5.12, owner 2026-10-26): (a) curve s-guard, (b) curve landmark.

    python tools/ekf10_sets.py        -> reports/<version>/ekf10_sets_300.json  (resumable)
    python tools/ekf10_sets.py --validation   -> validation seeds 300-399 (run once)

Standalone 10-state runs (no supervisor). v2 (owner 2026-10-26): the landmark is gated by the filter's
own branch decision; no-reverse constraint in two forms. ekf10_sets_300.json is v1 (variants base, a,
a+b; landmark ungated). Straight-on control: horizontal distance
between the displayed route position and the car at the portal.
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402

SETS = (0, 50, 100, 150, 200, 250)
OUT = "ekf10_sets_300_v3.json"
if "--validation" in sys.argv:
    SETS = (400, 450); OUT = "ekf10_validation_400-499.json"; sys.argv.remove("--validation")
KEYS = ("S1", "S2", "S3", "S4", "S5", "S6s", "S6", "S6L", "S1-1km", "S1-5km")
VARIANTS = {"v0.5.12": dict(lateral_speed=False, _mount="known"),            # v0.5.12 configuration
            "lateral": dict(_mount="known"),                                   # + lateral-force speed
            "v0.5.13": dict()}                                                 # + mount estimated in open sky
if "--only-final" in sys.argv:          # v0.5.13 option (a): the 10-state mount-yaw state changed only this variant
    VARIANTS = {"v0.5.13": dict()}; OUT = OUT.replace(".json", "_final.json"); sys.argv.remove("--only-final")
IMUS = {"note10": (), "lowend": ("imu_lowend",)}
FAIL_M = 100.0


def one(args):
    key, lo, imu, branch = args
    from tunnelsim.ekf10 import Ekf10Config
    from tunnelsim.experiments import ekf10_experiment, simulate_run
    seeds = np.arange(lo, lo + 50)
    sim = simulate_run(key, seeds, IMUS[imu], branch=branch)
    out = {}
    for var, kw in VARIANTS.items():
        cfg_kw = {k_: v_ for k_, v_ in kw.items() if not k_.startswith("_")}
        r = ekf10_experiment(key, seeds, IMUS[imu], cfg10=Ekf10Config(**cfg_kw), sim=sim, branch=branch, keep_tracks=branch,
                             mount=kw.get("_mount", "estimated"))
        ev = {k: v for k, v in r["events"].items() if k in ("landmark", "landmark_seen", "branch10", "main10", "no_reverse", "lateral_speed")}
        if not branch:
            out[var] = {"portal_horizontal_rmse": r["portal_horizontal_m"]["rmse"],
                        "portal_horizontal_p95": r["portal_horizontal_m"]["p95_abs"],
                        "branch10": r.get("branch10"), **ev}
            continue
        T = r["_tracks"]
        kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
        e = T["e_s"][:, kx]; ok = np.abs(e) <= FAIL_M
        out[var] = {"portal_rmse": r["real_exit"]["rmse"], "portal_p95": r["real_exit"]["p95_abs"],
                    "failures": int((~ok).sum()), "nees": r["consistency_real_exit"]["mean_norm_sq"],
                    "sigma_mean": r["consistency_real_exit"]["sigma_s_mean_m"],
                    "diverge_rmse": (r.get("at_diverge") or {}).get("rmse"), "errors": e.round(1).tolist(), **ev}
    return f"{imu}/{key}{'' if branch else '_straight_on'}/{lo}", out


if __name__ == "__main__":
    jobs = [(k, lo, imu, True) for imu in IMUS for lo in SETS for k in KEYS] + \
           [(k, lo, imu, False) for imu in IMUS for lo in SETS for k in ("S1", "S4")]
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    res = {"version": tunnelsim.__version__, "sets": list(SETS), "variants": VARIANTS, "fail_m": FAIL_M, "runs": {}}
    if (d / OUT).exists():
        res["runs"].update(json.loads((d / OUT).read_text())["runs"])
    jobs = [j for j in jobs if f"{j[2]}/{j[0]}{'' if j[3] else '_straight_on'}/{j[1]}" not in res["runs"]]
    with ProcessPoolExecutor(2) as ex:
        for tag, out in ex.map(one, jobs):
            res["runs"][tag] = out
            print(tag, {v: (o.get("portal_rmse", o.get("portal_horizontal_rmse")), o.get("failures"), o.get("nees")) for v, o in out.items()}, flush=True)
            (d / OUT).write_text(json.dumps(res, indent=1) + "\n")
