"""v0.5.17 validation, diagnosis only (no tuning): on the validation set whose recovery times fall
outside the 0-299 range (S1 family, seeds 750-799), the same seeds with the v0.5.16 cap (whole
position correction) against v0.5.17's (along-track only). Says whether the change or the seeds
cause the longer recovery.

    python tools/validation_attrib_v0517.py
"""
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tunnelsim.eskf import EskfConfig                  # noqa: E402
from tunnelsim.experiments import eskf_experiment      # noqa: E402

SEEDS = np.arange(750, 800)
out = {}
for case in ("S1", "S6s", "S4"):
    for name, along in (("v0.5.16 cap all", False), ("v0.5.17 cap along", True)):
        r = eskf_experiment(case, SEEDS, eskf_cfg=replace(EskfConfig(), reacq_cap_along_only=along), branch=True)
        rt = r["recovery_time"]
        m = {k: rt.get(k) for k in ("to_5m_median_s", "to_10m_p95_s", "not_5m_in_60s")}
        m["portal_rms_m"] = r["real_exit"]["rmse"]
        out[f"{case}/{name}"] = m
        print(case, name, m, flush=True)
(ROOT / "reports" / "0.5.17" / "validation_attrib_750-799.json").write_text(json.dumps(out, indent=1))
