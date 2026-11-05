"""v0.5.14: ESKF Note10 S4 seed 247 with both one-way forms, with and without the hybrid (the reason the ESKF
keeps the soft form). Portal along-track error and σ, diverge rms.

    python tools/seed247_forms.py   -> reports/<version>/seed247_forms.txt
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tunnelsim  # noqa: E402
from tunnelsim.eskf import EskfConfig  # noqa: E402
from tunnelsim.experiments import eskf_experiment  # noqa: E402

OFF = dict(grade_update=False, baro_grad=False)
CASES = [("no hybrid, mean-only", dict(OFF, no_reverse_mode="mean")), ("no hybrid, soft", dict(OFF, no_reverse_mode="soft")),
         ("hybrid, mean-only", dict(no_reverse_mode="mean")), ("hybrid, soft (shipped)", dict(no_reverse_mode="soft"))]
lines = []
for name, kw in CASES:
    r = eskf_experiment("S4", np.array([247]), eskf_cfg=EskfConfig(**kw), keep_tracks=True)
    T = r["_tracks"]; kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
    lines.append(f"{name:24s} portal e {T['e_s'][0, kx]:7.1f} m  σ {T['sig_s'][0, kx]:5.1f} m  diverge {r['at_diverge']['rmse']:6.1f} m")
    print(lines[-1], flush=True)
out = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0] / "seed247_forms.txt"
out.write_text("\n".join(lines) + "\n")
