"""v0.5.17.1 against v0.5.17 on sets 0-299: are the ESKF and supervisor results identical, and how
many position fixes did the gate reject inside the re-acquisition phase (the only case where the
settle-counter fix can change anything)?

    python tools/compare_v05171.py
"""
import json
from pathlib import Path

R = Path(__file__).resolve().parents[1] / "reports"


def load(p):
    return json.loads(p.read_text())["runs"]


def diff(a, b, skip=("gnss_pos_rej_reacq",)):
    keys = sorted(set(a) | set(b))
    bad = []
    for k in keys:
        x, y = a.get(k), b.get(k)
        if isinstance(x, dict) and isinstance(y, dict):
            x = {kk: vv for kk, vv in x.items() if kk not in skip}
            y = {kk: vv for kk, vv in y.items() if kk not in skip}
        if x != y:
            bad.append(k)
    return keys, bad


a, b = load(R / "0.5.17" / "eval_sets_300_default.json"), load(R / "0.5.17.1" / "eval_sets_300_default.json")
keys, bad = diff(a, b)
rej = sum(v.get("gnss_pos_rej_reacq") or 0 for v in b.values())
print(f"ESKF Note10 sets 0-299: {len(keys) - len(bad)} of {len(keys)} runs of 50 identical; "
      f"rejected fixes in the phase: {rej}")
for k in bad:
    print("  differs:", k)
pa, pb = R / "0.5.17" / "supervisor_sets_0-299.json", R / "0.5.17.1" / "supervisor_sets_0-299.json"
if pb.exists():
    keys, bad = diff(load(pa), load(pb))
    print(f"Supervisor sets 0-299: {len(keys) - len(bad)} of {len(keys)} identical")
    for k in bad:
        print("  differs:", k)
