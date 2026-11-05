"""K1 (v0.5.15): the rate at which the drawn point crosses from the main tunnel to the branch once "branch"
is decided. Off-road distance against the largest step across, for rates 2 / 5 / 10 m/s and none, on the
low-end cases where the crossing is longest. Seeds 0-49.

    python tools/k1_rate_sweep.py  -> reports/<version>/k1_rate_sweep_0-49.json
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tunnelsim  # noqa: E402
from tunnelsim.display import display_metrics, shown_xy  # noqa: E402
from tunnelsim.experiments import get_scenario, supervisor_experiment  # noqa: E402
from tunnelsim.mapview import MapView  # noqa: E402

CASES = [("S4", ("imu_lowend",)), ("S1", ("imu_lowend", "baro_tunnel_30pa")), ("S1-5km", ("imu_lowend",)), ("S4", ())]
res = {}
for key, ov in CASES:
    r = supervisor_experiment(key, np.arange(0, 50), ov, rules=("class",), keep_tracks=True)
    T = r["_tracks"]; oe = T["oe"]; s_disp = T["rules"]["class/majority"]["s_disp"]; sel = T["rules"]["class/majority"]["sel"]
    mv = MapView(get_scenario(key, branch=True).road)
    st_e = oe.aux["branch_state"]; t10 = oe.aux["branch10_time"]; d10 = oe.aux["branch10"]
    st_10 = np.where(oe.t[None] >= np.where(np.isfinite(t10), t10, np.inf)[:, None], d10[:, None], 0)
    state = np.where((sel == "ekf10")[:, None], st_10, st_e)
    s_true = T["s_true"]
    from tunnelsim.experiments import simulate_run
    sc, tr, *_ = simulate_run(key, np.arange(0, 1), ov)
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1])); t_x = tr.t[min(i_x, len(tr.t) - 1)]
    win = (s_true >= mv.s_div - 50.0) & (oe.t <= t_x)
    ks = np.nonzero(win)[0]
    ks = ks[np.unique(np.minimum(np.searchsorted(oe.t[ks], np.arange(oe.t[ks[0]], oe.t[ks[-1]], 0.1)), len(ks) - 1))]
    idx = np.clip(np.round((oe.t[ks] - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    j0 = int(np.argmax(tr.s >= mv.s_div - 100.0)); road_xy = tr.pos_n[j0::max(1, int(round(0.05 / tr.dt))), :2]
    out = {}
    for rate in (2.0, 5.0, 10.0, None):
        xy, _ = shown_xy(mv, oe.t[ks], s_disp[:, ks], state[:, ks], "main_first", lat_rate=rate)
        out[str(rate)] = display_metrics(oe.t[ks], xy, s_disp[:, ks], road_xy, tr.pos_n[idx, :2], np.ones(len(ks), bool))
    res[f"{key}/{'+'.join(ov) or 'note10'}"] = out
    print(key, ov, json.dumps(out), flush=True)
d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
(d / "k1_rate_sweep_0-49.json").write_text(json.dumps(res, indent=1) + "\n")
