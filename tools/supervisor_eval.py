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

CASES = [(k, imu) for k in ("S1", "S2", "S3", "S4", "S5", "S6", "S1-5km") for imu in ("note10", "lowend")] + \
        [("S3", "pressure30"), ("S1", "pressure30")]
OV = {"note10": (), "lowend": ("imu_lowend",), "pressure30": ("baro_tunnel_30pa",)}


def one(args):
    key, imu, lo, hi, rules = args
    from tunnelsim.experiments import supervisor_experiment
    r = supervisor_experiment(key, np.arange(lo, hi), OV[imu], rules=rules)
    return f"{key}/{imu}/{lo}", r


if __name__ == "__main__":
    part = sys.argv[1] if len(sys.argv) > 1 else "dev"
    # "sets": the class rule only (no rehearsals) on four independent sets of 50 seeds
    spans = {"dev": [(0, 50)], "heldout": [(50, 100)], "smoke": [(0, 2)],
             "sets": [(0, 50), (50, 100), (100, 150), (150, 200)]}[part]
    rules = ("class",) if part == "sets" else ("class", "rehearsal", "simplest")
    res = {"version": tunnelsim.__version__, "spans": spans, "rules": rules, "runs": {}}
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, [(k, i, lo, hi, rules) for lo, hi in spans for k, i in CASES]):
            res["runs"][tag] = r
            (d / f"supervisor_{part}.json").write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(tag, json.dumps({n: {m: (v.get("portal", v) if isinstance(v, dict) else v)
                                       for m, v in r["rules"][n].items()} for n in r["rules"]}), flush=True)
    (d / f"supervisor_{part}.json").write_text(json.dumps(res, indent=1, default=str) + "\n")
