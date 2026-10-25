"""Per-seed change between two ESKF configurations on 200 seeds (v0.5.11, owner 2026-10-25): the rms
hides both ends, so list the most-worsened and most-improved seeds per scenario.

    python tools/perseed.py      -> reports/<version>/perseed_base_vs_gate-reassoc.json
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402

A = {"map_nis_gate": False, "map_reassoc": False}
B = {"map_nis_gate": True, "map_reassoc": True}
KEYS = ("S1", "S2", "S3", "S4", "S5", "S6s", "S6", "S6L", "S1-1km", "S1-5km")


def job(args):
    key, lo = args
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import eskf_experiment
    out = {}
    for name, cfg in (("a", A), ("b", B)):
        r = eskf_experiment(key, np.arange(lo, lo + 50), eskf_cfg=EskfConfig(**cfg), keep_tracks=True)
        T = r["_tracks"]
        kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
        out[name] = {"portal": T["e_s"][:, kx].tolist(), "sig": T["sig_s"][:, kx].tolist()}
    return key, lo, out


if __name__ == "__main__":
    res = {"version": tunnelsim.__version__, "a": A, "b": B, "runs": {}}
    with ProcessPoolExecutor(2) as ex:
        for key, lo, out in ex.map(job, [(k, lo) for k in KEYS for lo in (0, 50, 100, 150)]):
            r = res["runs"].setdefault(key, {"seeds": [], "a": [], "b": [], "sig_b": []})
            r["seeds"] += list(range(lo, lo + 50)); r["a"] += out["a"]["portal"]; r["b"] += out["b"]["portal"]
            r["sig_b"] += out["b"]["sig"]
            print(key, lo, flush=True)
    for key, r in res["runs"].items():
        a, b = np.abs(np.array(r["a"])), np.abs(np.array(r["b"]))
        d = b - a
        i, j = int(np.argmax(d)), int(np.argmin(d))
        r["summary"] = {"worst_seed": r["seeds"][i], "worst_a_m": round(float(a[i]), 1), "worst_b_m": round(float(b[i]), 1),
                        "worst_b_e_over_sigma": round(float(np.array(r["b"])[i] / r["sig_b"][i]), 2),
                        "best_seed": r["seeds"][j], "best_a_m": round(float(a[j]), 1), "best_b_m": round(float(b[j]), 1),
                        "worse_by_gt_5m": int((d > 5).sum()), "better_by_gt_5m": int((d < -5).sum())}
    d = ROOT / "reports" / tunnelsim.__version__
    (d / "perseed_base_vs_gate-reassoc.json").write_text(json.dumps(res, indent=1) + "\n")
