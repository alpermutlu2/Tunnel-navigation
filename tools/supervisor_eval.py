"""Supervisor evaluation (v0.5.11, owner 2026-10-25): three filters, open-sky rehearsals, selection rules
and the disagreement monitor, on development seeds (0-49) and held-out seeds (50-99).

    python tools/supervisor_eval.py dev        -> reports/<version>/supervisor_dev.json
    python tools/supervisor_eval.py heldout    -> reports/<version>/supervisor_heldout.json

Thresholds are fixed in tunnelsim/supervisor.py before the runs. Nominal scenarios for both IMU
classes, plus known failures: tunnel pressure gradient 30 Pa/km (S3, S1).
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402

CUSTOM = "custom:L=5000,g=3,v=110"       # v0.5.14: 5 km, 3 % grade, 110 km/h (the hybrid's case)
CASES = [(k, imu) for k in ("S1", "S2", "S3", "S4", "S5", "S6", "S1-5km") for imu in ("note10", "lowend")] + \
        [("S3", "pressure30"), ("S1", "pressure30")] + \
        [("S3", "lowend_pressure30"), ("S1", "lowend_pressure30")] + \
        [(CUSTOM, i) for i in ("note10", "lowend", "pressure25", "lowend_pressure25", "pressure-25", "lowend_pressure-25")] + \
        [(k, i) for k in ("S1", "S4", "S1-5km") for i in ("note10_straight", "lowend_straight")]   # v0.5.15: K1
OV = {"note10": (), "lowend": ("imu_lowend",), "pressure30": ("baro_tunnel_30pa",),
      "lowend_pressure30": ("imu_lowend", "baro_tunnel_30pa"),        # v0.5.14: the finding that led to the hybrid
      "pressure25": ("set:baro.tunnel_grad_pa_per_km=25",), "lowend_pressure25": ("imu_lowend", "set:baro.tunnel_grad_pa_per_km=25"),
      "pressure-25": ("set:baro.tunnel_grad_pa_per_km=-25",), "lowend_pressure-25": ("imu_lowend", "set:baro.tunnel_grad_pa_per_km=-25"),
      "note10_straight": (), "lowend_straight": ("imu_lowend",)}          # the car goes straight on at the diverge


def one(args):
    key, imu, lo, hi, rules = args
    from tunnelsim.experiments import supervisor_experiment
    r = supervisor_experiment(key, np.arange(lo, hi), OV[imu], rules=rules, branch=not imu.endswith("_straight"))
    return f"{key}/{imu}/{lo}", r


if __name__ == "__main__":
    part = sys.argv[1] if len(sys.argv) > 1 else "dev"
    if ":" in part:                 # e.g. "sets:0-100": the class rule on seeds 0-99, in 50-seed sets
        name_, rng = part.split(":")
        lo_, hi_ = map(int, rng.split("-"))
        globals()["_SPANS"] = [(a_, a_ + 50) for a_ in range(lo_, hi_, 50)]
        part = f"{name_}_{lo_}-{hi_ - 1}"
    # "sets": the class rule only (no rehearsals) on four independent sets of 50 seeds
    spans = {"dev": [(0, 50)], "heldout": [(50, 100)], "smoke": [(0, 2)],
             "sets": [(0, 50), (50, 100), (100, 150), (150, 200)],
             "validation": [(600, 650), (650, 700)]}.get(part) or globals()["_SPANS"]     # v0.5.15: 600-699
    rules = ("class",) if part.startswith(("sets", "validation")) else ("class", "rehearsal", "simplest")
    res = {"version": tunnelsim.__version__, "spans": spans, "rules": rules, "runs": {}}
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    d.mkdir(parents=True, exist_ok=True)
    jobs = [(k, i, lo, hi, rules) for lo, hi in spans for k, i in CASES]
    if (d / f"supervisor_{part}.json").exists():           # resume
        res["runs"].update(json.loads((d / f"supervisor_{part}.json").read_text())["runs"])
        jobs = [j for j in jobs if f"{j[0]}/{j[1]}/{j[2]}" not in res["runs"]]
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, jobs):
            res["runs"][tag] = r
            (d / f"supervisor_{part}.json").write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(tag, json.dumps({n: {m: (v.get("portal", v) if isinstance(v, dict) else v)
                                       for m, v in r["rules"][n].items()} for n in r["rules"]}), flush=True)
    (d / f"supervisor_{part}.json").write_text(json.dumps(res, indent=1, default=str) + "\n")
