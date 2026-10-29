"""ESKF on independent seed sets of 50 (owner 2026-10-25): development 0-49, feature design 50-299
(200-299 since v0.5.12), validation 300-399 (run once). Variants are EskfConfig overrides.

    python tools/eval_sets.py default              -> reports/<version>/eval_sets_300_default.json (resumable)
    python tools/eval_sets.py default --validation -> reports/<version>/validation_300-399.json
    python tools/eval_sets.py win2 --only S4,S4_straight_on   -> a subset
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402

SETS = (0, 50, 100, 150, 200, 250)   # v0.5.12: 200-299 is a design tier (the branch window came from seed 247)
if "--validation" in sys.argv:          # untouched seeds, final configuration only, run once (owner 2026-10-25)
    SETS = (400, 450)
    sys.argv.remove("--validation")
ONLY = None
if "--only" in sys.argv:                # e.g. --only S1_straight_on,S4_straight_on
    i = sys.argv.index("--only"); ONLY = set(sys.argv[i + 1].split(",")); del sys.argv[i:i + 2]
KEYS = ("S1", "S2", "S3", "S4", "S5", "S6s", "S6", "S6L", "S1-1km", "S1-5km")
VARIANTS = {"base": {"map_nis_gate": False}, "gate": {"map_nis_gate": True},
            "gate+defer": {"map_nis_gate": True, "map_branch_defer": True},
            "gate+reassoc": {"map_nis_gate": True, "map_reassoc": True},
            "reassoc": {"map_nis_gate": False, "map_reassoc": True},
            "default": {},
            "win0": {"map_pre_sigma_k": 0.0},
            "win2": {"map_pre_sigma_k": 2.0},
            # v0.5.13: experiment options (keys starting with "_"): the mount the filters use, the true mount, the approach
            "known": {"_mount": "known"},
            "cup": {"_vehicle_mount": "cupholder"},
            "noexc": {"_approach": False},
            "cup-known": {"_vehicle_mount": "cupholder", "_mount": "known"},
            "noexc-known": {"_approach": False, "_mount": "known"},
            "win2+slip": {"map_pre_sigma_k": 2.0, "map_reassoc_slip_k": 7e-3}}
RUN = tuple(sys.argv[1].split(",")) if len(sys.argv) > 1 else tuple(VARIANTS)
FAIL_M = 100.0     # a run whose portal error exceeds this counts as a failure


def one(args):
    var, key, lo, branch = args
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import eskf_experiment
    kw = {k_: v_ for k_, v_ in VARIANTS[var].items() if not k_.startswith("_")}
    ex = {}
    if "_mount" in VARIANTS[var]:
        ex["mount"] = VARIANTS[var]["_mount"]
    if "_approach" in VARIANTS[var]:
        ex["approach_manoeuvre"] = VARIANTS[var]["_approach"]
    if VARIANTS[var].get("_vehicle_mount") == "cupholder":
        from dataclasses import replace as _rp
        from tunnelsim.experiments import CUPHOLDER_RPY_DEG, get_scenario
        ex["vehicle"] = _rp(get_scenario(key).vehicle, mount_rpy_deg=CUPHOLDER_RPY_DEG)
    r = eskf_experiment(key, np.arange(lo, lo + 50), eskf_cfg=EskfConfig(**kw), branch=branch, keep_tracks=True, **ex)
    T = r["_tracks"]
    kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
    e = T["e_s"][:, kx]
    ok = np.abs(e) <= FAIL_M
    out = {"diverge_rmse": r["at_diverge"]["rmse"], "diverge_p95": r["at_diverge"]["p95_abs"],
           "portal_rmse": r["real_exit"]["rmse"], "portal_p95": r["real_exit"]["p95_abs"],
           "portal_rmse_without_failures": round(float(np.sqrt(np.mean(e[ok] ** 2))), 3) if ok.any() else None,
           "failures": int((~ok).sum()), "nees": r["consistency_real_exit"]["mean_norm_sq"],
           "exit_step_p95": r["recovery"]["max_step_jump_m"]["p95_abs"],
           "branch": r["branch_confirm"]["branch_reported_frac"], "main": r["branch_confirm"]["main_reported_frac"],
           "map_rejected": r["events"].get("map_rejected", 0),
           "none": r["branch_confirm"]["none_frac"], "branch_delay_s": (r["branch_confirm"]["delay_s"] or {}).get("bias"),
           "errors": e.round(1).tolist()}
    if "mount" in r:
        out["mount_err_rms_deg"] = r["mount"]["err_rms_deg"]; out["mount_yaw_from"] = r["mount"]["yaw_from"]
    if "recovery_time" in r:                     # v0.5.13
        out.update({f"rec_{k_}": v_ for k_, v_ in r["recovery_time"].items()})
    if "prompts" in r:
        for pn, pv in r["prompts"].items():
            out.update({f"prompt_{pn.replace(' ', '')}_{k_}": v_ for k_, v_ in pv.items()})
    if "undecided" in r:
        u = r["undecided"]
        out.update({"undecided_s_mean": u["after_diverge_s"]["bias"], "undecided_s_p95": u["after_diverge_s"]["p95_abs"],
                    "undecided_at_portal": u["at_portal_frac"],
                    "display_off_road_mean_m": u["display_off_road_max_m"]["bias"],
                    "display_off_road_p95_m": u["display_off_road_max_m"]["p95_abs"],
                    "display_off_road_gt_10m": u["display_off_road_gt_10m"]})
    for k_, v in r.get("callout_k", {}).items():
        out[f"late_gt_1s_k{k_}"] = v["late_gt_1s_frac"]
        out[f"mean_s_k{k_}"] = v["mean_s"]
    return f"{var}/{key}{'' if branch else '_straight_on'}/{lo}", out


if __name__ == "__main__":
    jobs = [(v, k, lo, True) for v in RUN for lo in SETS for k in KEYS] + \
           [(v, k, lo, False) for v in RUN for lo in SETS for k in ("S1", "S4")]
    if ONLY:
        jobs = [j for j in jobs if f"{j[1]}{'' if j[3] else '_straight_on'}" in ONLY]
    n_ = 50 * len(SETS)
    out_name = f"eval_sets_{n_}.json" if RUN == tuple(VARIANTS)[:3] else f"eval_sets_{n_}_{'_'.join(RUN).replace('+', '-')}.json"
    if ONLY:
        out_name = out_name.replace(".json", "_" + "_".join(sorted(ONLY)) + ".json")
    if SETS[0] >= 400:
        out_name = "validation_400-499.json"
    res = {"version": tunnelsim.__version__, "sets": list(SETS), "variants": {v: VARIANTS[v] for v in RUN}, "seeds_per_set": 50, "fail_m": FAIL_M, "runs": {}}
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    if (d / out_name).exists() and "--fresh" not in sys.argv:     # resume: keep the runs already saved
        old = json.loads((d / out_name).read_text())
        res["runs"].update(old.get("runs", {}))
        jobs = [j for j in jobs if f"{j[0]}/{j[1]}{'' if j[3] else '_straight_on'}/{j[2]}" not in res["runs"]]
    with ProcessPoolExecutor(2) as ex:
        for tag, out in ex.map(one, jobs):
            res["runs"][tag] = out
            print(tag, out["diverge_rmse"], out["portal_rmse"], out["failures"], out["nees"], flush=True)
            (d / out_name).write_text(json.dumps(res, indent=1) + "\n")
