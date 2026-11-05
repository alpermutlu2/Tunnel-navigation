"""Low-end phone + tunnel pressure gradient through the supervisor (v0.5.14, owner 2026-11-05).

Found in the live simulator: a custom 5 km tunnel, 3 % grade, 110 km/h, 25 Pa/km; the 10-state filter
ended +360 m off with sigma 13 m. The supervisor results of v0.5.11-13 covered the pressure gradient on the
Note10 only. A first look, 20 seeds of the feature-design tier (50-69):

    python tools/supervisor_pressure_lowend.py reports/<version>/supervisor_lowend_pressure_50-69.json
"""
import sys, json, numpy as np
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]))
from concurrent.futures import ProcessPoolExecutor
SEEDS = np.arange(50, 70)          # feature-design tier, a first look (20 seeds)
CASES = [("S1", "lowend", 30), ("S3", "lowend", 30), ("S1-5km", "lowend", 30),
         ("custom:L=5000,g=3,v=110", "lowend", 25), ("custom:L=5000,g=3,v=110", "note10", 25),
         ("S1", "lowend", 0), ("S1-5km", "lowend", 0)]
def one(c):
    key, imu, pg = c
    from tunnelsim.experiments import supervisor_experiment
    ov = (("imu_lowend",) if imu == "lowend" else ()) + ((f"set:baro.tunnel_grad_pa_per_km={pg}",) if pg else ())
    r = supervisor_experiment(key, SEEDS, ov, rules=("class",))
    return f"{key}/{imu}/{pg}Pa", r
if __name__ == "__main__":
    out = {}
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, CASES):
            out[tag] = r
            json.dump(out, open(sys.argv[1], "w"), default=str, indent=1)
            f = r["filters"]; c = r["rules"]["class"]
            print(tag, "| sel", c["selected"], "| filters portal/p95/nees:",
                  {k: (f[k]["portal"], f[k]["portal_p95"], f[k]["nees"]) for k in f},
                  "| app none:", (c["none"]["portal"], c["none"]["portal_p95"], c["none"]["nees"]),
                  "majority:", (c["majority"]["portal"], c["majority"]["nees"], c["majority"].get("alarm_runs_frac")),
                  "branch:", c.get("branch_shown"), flush=True)
