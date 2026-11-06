"""v0.5.17: the 300-seed results (sets 0-299) against the last full ESKF run (v0.5.14; the ESKF did not change
in v0.5.15-16) and the supervisor's against v0.5.15, as ranges over the six sets of 50.

    python tools/summarise_v0517.py [eskf|supervisor|all|validation]

validation: the untouched tier 700-799 (run once) against the v0.5.17 ranges over sets 0-299.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / "reports"
WHAT = sys.argv[1] if len(sys.argv) > 1 else "all"


def rng(v, f="{:.1f}"):
    v = [x for x in v if x is not None]
    if not v:
        return "–"
    a, b = min(v), max(v)
    return f.format(a) if f.format(a) == f.format(b) else f"{f.format(a)}–{f.format(b)}"


def load(p):
    return json.loads(p.read_text())["runs"] if p.exists() else {}


ROWS = [("diverge rms", "diverge_rmse", "rng"), ("portal rms", "portal_rmse", "rng"),
        ("portal p95 worst", "portal_p95", "max"), ("failures", "failures", "sum"),
        ("(e/σ)² worst", "nees", "max2"), ("exit step p95 worst", "exit_step_p95", "max"),
        ("<5 m median s", "rec_to_5m_median_s", "rng"), ("not <5 m in 60 s", "rec_not_5m_in_60s", "sum"),
        ("<10 m p95 s", "rec_to_10m_p95_s", "rng"), ("worst shown p95", "rec_worst_shown_m_p95", "max"),
        ("late>1s k=1 (n)", "late_gt_1s_k1.0", "n50"), ("branch (n)", "branch", "n50"), ("main (n)", "main", "n50")]


def agg(sets, key, how):
    v = [s.get(key) for s in sets if s.get(key) is not None]
    if not v:
        return "–"
    if how == "rng":
        return rng(v)
    if how == "max":
        return f"{max(v):.1f}"
    if how == "max2":
        return f"{max(v):.2f}"
    if how == "sum":
        return str(int(sum(v)))
    return str(int(sum(round(x * 50) for x in v)))


def eskf():
    a, b = load(R / "0.5.14" / "eval_sets_300_default.json"), load(R / "0.5.17" / "eval_sets_300_default.json")
    keys = sorted({k.split("/")[1] for k in b})
    print("ESKF Note10, sets 0-299: v0.5.14 -> v0.5.17 (changed values marked *)")
    for k in keys:
        A = [v for kk, v in a.items() if kk.split("/")[1] == k]
        B = [v for kk, v in b.items() if kk.split("/")[1] == k]
        cells = []
        for name, key, how in ROWS:
            x, y = agg(A, key, how), agg(B, key, how)
            cells.append(f"{name}: {x}" if x == y else f"{name}: {x} -> {y} *")
        print(f"  {k}\n    " + "\n    ".join(cells))


def supervisor():
    a, b = load(R / "0.5.15" / "supervisor_sets_0-299.json"), load(R / "0.5.17" / "supervisor_sets_0-299.json")
    cases = sorted({k.rsplit("/", 1)[0] for k in b})
    print("Supervisor (class rule, majority) and its ESKF, sets 0-299: v0.5.15 -> v0.5.17")
    for c in cases:
        out = []
        for path, label in ((("rules", "class", "majority"), "sup"), (("filters", "eskf"), "eskf")):
            def g(d):
                S = [v for kk, v in d.items() if kk.rsplit("/", 1)[0] == c]
                for p in path:
                    S = [s.get(p, {}) for s in S]
                return S
            A, B = g(a), g(b)
            for name, key, how in (("portal", "portal", "rng"), ("(e/σ)²", "nees", "max2"),
                                   ("exit step p95", "exit_step_p95", "max")):
                x, y = agg(A, key, how), agg(B, key, how)
                out.append(f"{label} {name} {x}" if x == y else f"{label} {name} {x} -> {y} *")
        print(f"  {c:28s} " + " | ".join(out))


def validation():
    """Validation 700-799 (run once), each set of 50 against the v0.5.17 300-seed range; outside marked !"""
    a, b = load(R / "0.5.17" / "eval_sets_300_default.json"), load(R / "0.5.17" / "validation_700-799.json")
    print("ESKF Note10, validation 700-799 (two sets) against sets 0-299 (v0.5.17); ! = outside the range")
    for k in sorted({kk.split("/")[1] for kk in b}):
        A = [v for kk, v in a.items() if kk.split("/")[1] == k]
        B = [v for kk, v in b.items() if kk.split("/")[1] == k]
        cells = []
        for name, key, how in ROWS:
            va, vb = [s.get(key) for s in A if s.get(key) is not None], [s.get(key) for s in B if s.get(key) is not None]
            if not vb:
                continue
            if how in ("sum", "n50"):
                cells.append(f"{name}: {agg(B, key, how)} of {50 * len(B)} (0-299: {agg(A, key, how)} of {50 * len(A)})")
                continue
            out = va and (max(vb) > max(va) + 1e-9 or (how == "rng" and min(vb) < min(va) - 1e-9))
            cells.append(f"{name}: {agg(B, key, how)} (0-299 {agg(A, key, how)}){' !' if out else ''}")
        print(f"  {k}\n    " + "\n    ".join(cells))
    a, b = load(R / "0.5.17" / "supervisor_sets_0-299.json"), load(R / "0.5.17" / "supervisor_validation.json")
    print("Supervisor (class rule, majority), validation 700-799 against sets 0-299 (v0.5.17); ! = above the range")
    for c in sorted({k.rsplit("/", 1)[0] for k in b}):
        out = []
        for path, label in ((("rules", "class", "majority"), "sup"), (("filters", "eskf"), "eskf")):
            def g(d):
                S = [v for kk, v in d.items() if kk.rsplit("/", 1)[0] == c]
                for p in path:
                    S = [s.get(p, {}) for s in S]
                return S
            A, B = g(a), g(b)
            for name, key, how in (("portal", "portal", "rng"), ("(e/σ)²", "nees", "max2"),
                                   ("exit step p95", "exit_step_p95", "max")):
                va, vb = [s.get(key) for s in A if s.get(key) is not None], [s.get(key) for s in B if s.get(key) is not None]
                flag = " !" if va and vb and max(vb) > max(va) + 1e-9 else ""
                out.append(f"{label} {name} {agg(B, key, how)} ({agg(A, key, how)}){flag}")
        print(f"  {c:28s} " + " | ".join(out))


if WHAT in ("eskf", "all"):
    eskf()
if WHAT in ("supervisor", "all"):
    supervisor()
if WHAT == "validation":
    validation()
