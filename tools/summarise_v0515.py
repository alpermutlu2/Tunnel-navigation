"""v0.5.15 supervisor results: K1 (drawn point near the diverge), K3 (callout k by class, the sweep and the
acceptance rule), against v0.5.14.

    python tools/summarise_v0515.py [sets|validation]
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PART = sys.argv[1] if len(sys.argv) > 1 else "sets"
F15 = ROOT / "reports" / "0.5.15" / ("supervisor_sets_0-299.json" if PART == "sets" else "supervisor_validation.json")
F14 = ROOT / "reports" / "0.5.14" / ("supervisor_sets_0-299.json" if PART == "sets" else "supervisor_validation.json")


def rng(v, f="{:.1f}"):
    v = [x for x in v if x is not None]
    if not v:
        return "–"
    a, b = min(v), max(v)
    return f.format(a) if f.format(a) == f.format(b) else f"{f.format(a)}–{f.format(b)}"


b = json.loads(F15.read_text())["runs"]
a = json.loads(F14.read_text())["runs"] if F14.exists() else {}
cases = sorted({k.rsplit("/", 1)[0] for k in b})
print("case | portal v0.5.14 -> v0.5.15 | (e/σ)² | late>1s (class k) | k sweep 0.5/0.7/1: late, >5s early, mean s, now m")
for c in cases:
    S = [v["rules"]["class"] for k, v in b.items() if k.rsplit("/", 1)[0] == c]
    A = [v["rules"]["class"] for k, v in a.items() if k.rsplit("/", 1)[0] == c]
    m = [s["majority"] for s in S]
    n = 50 * len(S)
    late = sum(round(50 * x["late_gt_1s"]) for x in m)
    sw = {}
    for kk in ("0.5", "0.7", "1.0"):
        q = [x["callout_k_sweep"][kk] for x in m]
        sw[kk] = (sum(x["late_gt_1s"] for x in q), sum(x["early_gt_5s"] for x in q), rng([x["mean_s"] for x in q]),
                  rng([x["now_err_mean_m"] for x in q], "{:.0f}"))
    old = rng([x["majority"]["portal"] for x in A]) if A else "–"
    print(f"{c:42s} | {old} -> {rng([x['portal'] for x in m])} | {max(x['nees'] for x in m):.2f} | {late}/{n} | " +
          " ; ".join(f"k{kk}: {v[0]}, {v[1]}, {v[2]}, {v[3]}" for kk, v in sw.items()))
print("\nK1 display (product): route -> main_first (limit) | main_first no limit; off-road max mean (p95) [runs>10 m]; transition step max (p95); not-yet s; decision at portal")
for c in cases:
    S = [v["rules"]["class"].get("display") for k, v in b.items() if k.rsplit("/", 1)[0] == c]
    if not S or S[0] is None:
        continue
    f = lambda p, key: [s[p][key] for s in S]
    dec = {kk: sum(s["decision_at_portal"][kk] for s in S) for kk in ("branch", "main", "none")}
    print(f"{c:42s} | route {rng(f('route','offroad_max_mean_m'))} ({max(f('route','offroad_max_p95_m')):.1f}) [{sum(f('route','runs_offroad_gt_10m'))}] step {max(f('route','transition_step_max_m')):.1f}"
          f" | main {rng(f('main_first','offroad_max_mean_m'))} ({max(f('main_first','offroad_max_p95_m')):.1f}) [{sum(f('main_first','runs_offroad_gt_10m'))}] step {max(f('main_first','transition_step_max_m')):.1f} (p95 {max(f('main_first','transition_step_p95_m')):.1f})"
          f" | no-limit step {max(f('main_first_no_limit','transition_step_max_m')):.1f} (p95 {max(f('main_first_no_limit','transition_step_p95_m')):.1f}) off {rng(f('main_first_no_limit','offroad_max_mean_m'))}"
          f" | err max {rng(f('main_first','err_max_mean_m'))} | not yet {rng([s['not_yet_s_mean'] for s in S])} s | {dec}")
