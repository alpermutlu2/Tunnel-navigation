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
    key, imu, lo, hi = args
    from tunnelsim.experiments import supervisor_experiment
    r = supervisor_experiment(key, np.arange(lo, hi), OV[imu])
    return f"{key}/{imu}", r


if __name__ == "__main__":
    part = sys.argv[1] if len(sys.argv) > 1 else "dev"
    lo, hi = (0, 50) if part == "dev" else (50, 100)
    res = {"version": tunnelsim.__version__, "seeds": [lo, hi - 1], "runs": {}}
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, [(k, i, lo, hi) for k, i in CASES]):
            res["runs"][tag] = r
            print(tag, json.dumps({n: {m: r["rules"][n][m]["portal"] if isinstance(r["rules"][n][m], dict) else r["rules"][n][m]
                                       for m in r["rules"][n]} for n in r["rules"]}), flush=True)
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    (d / f"supervisor_{part}.json").write_text(json.dumps(res, indent=1, default=str) + "\n")
