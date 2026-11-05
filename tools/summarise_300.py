"""v0.5.14: the 300-seed results against v0.5.13's, as ranges over the six sets of 50 (0-299).

    python tools/summarise_300.py [eskf|ekf10|supervisor|all]
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
R13, R14 = ROOT / "reports" / "0.5.13", ROOT / "reports" / "0.5.14"
WHAT = sys.argv[1] if len(sys.argv) > 1 else "all"


def rng(vals, f="{:.1f}"):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "–"
    return f.format(min(vals)) if min(vals) == max(vals) else f"{f.format(min(vals))}–{f.format(max(vals))}"


def load(p):
    return json.loads(p.read_text())["runs"] if p.exists() else {}


def eskf():
    a, b = load(R13 / "eval_sets_300_default.json"), load(R14 / "eval_sets_300_default.json")
    keys = sorted({k.split("/")[1] for k in b})
    print("ESKF Note10 (v0.5.13 -> v0.5.14): diverge | portal | failures | worst (e/σ)² | branch | late k=1 | in 2σ portal")
    for k in keys:
        rows = []
        for d in (a, b):
            sets = [v for kk, v in d.items() if kk.split("/")[1] == k]
            if not sets:
                rows.append("–"); continue
            cov = [s.get("coverage", {}).get("portal", {}).get("within_2sigma") for s in sets]
            late = sum(int(round((s.get("late_gt_1s_k1.0") or 0) * 50)) for s in sets)
            br = sum(int(round((s.get("branch") or 0) * 50)) for s in sets)
            rows.append(f"{rng([s.get('diverge_rmse') for s in sets])} | {rng([s['portal_rmse'] for s in sets])} | "
                        f"{sum(s['failures'] for s in sets)} | {max(s['nees'] for s in sets):.2f} | {br}/{50 * len(sets)} | {late} | {rng(cov, '{:.2f}')}")
        print(f"  {k:18s} {rows[0]}\n  {'':18s} {rows[1]}")


def ekf10():
    a = load(R13 / "ekf10_sets_300_v3_final.json"); b = load(R14 / "ekf10_sets_300.json")
    print("10-state (v0.5.13 | v0.5.14 mean | soft): portal rms range, failures, worst (e/σ)², diverge range")
    keys = sorted({k.rsplit("/", 1)[0] for k in b})
    for k in keys:
        out = []
        for d, var in ((a, "v0.5.13"), (b, "v0.5.14"), (b, "soft")):
            sets = [v[var] for kk, v in d.items() if kk.rsplit("/", 1)[0] == k and var in v]
            if not sets:
                out.append("–"); continue
            if "portal_rmse" in sets[0]:
                out.append(f"{rng([s['portal_rmse'] for s in sets])} m, {sum(s['failures'] for s in sets)}, "
                           f"{max(s['nees'] for s in sets):.2f}, div {rng([s.get('diverge_rmse') for s in sets])}")
            else:
                out.append(f"horiz {rng([s['portal_horizontal_rmse'] for s in sets])} m")
        print(f"  {k:26s} " + "  |  ".join(out))
    # the seeds the owner asked about (low-end S4)
    for var_d, var in ((a, "v0.5.13"), (b, "v0.5.14"), (b, "soft")):
        line = []
        for sd in (113, 125, 200, 232, 255):
            lo = sd // 50 * 50
            run = var_d.get(f"lowend/S4/{lo}", {}).get(var)
            line.append(f"{sd}: {run['errors'][sd - lo]:.0f}" if run else f"{sd}: –")
        print(f"  low-end S4 portal error, {var:8s}: " + ", ".join(line))


def supervisor():
    a = {}
    for p in ("supervisor_sets_0-99.json", "supervisor_sets_100-199.json", "supervisor_sets_200-299.json"):
        a.update(load(R13 / p))
    b = load(R14 / "supervisor_sets_0-299.json")
    print("Supervisor, class rule with the majority monitor (v0.5.13 -> v0.5.14): portal range, worst (e/σ)², branch")
    cases = sorted({k.rsplit("/", 1)[0] for k in b})
    for c in cases:
        out = []
        for d in (a, b):
            sets = [v for kk, v in d.items() if kk.rsplit("/", 1)[0] == c]
            if not sets:
                out.append("–"); continue
            m = [s["rules"]["class"].get("majority") or s["rules"]["class"]["none"] for s in sets]
            bc = [s["rules"]["class"].get("branch_shown", {}) for s in sets]
            out.append(f"{rng([x['portal'] for x in m])} m, {max(x['nees'] for x in m):.2f}, "
                       f"branch {rng([x.get('branch') for x in bc], '{:.2f}')}")
        print(f"  {c:40s} {out[0]:40s} -> {out[1]}")


if WHAT in ("eskf", "all"):
    eskf()
if WHAT in ("ekf10", "all"):
    ekf10()
if WHAT in ("supervisor", "all"):
    supervisor()
