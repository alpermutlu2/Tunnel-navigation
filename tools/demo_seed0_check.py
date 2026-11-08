# v0.5.17.1 (2026-11-08): the DEMO.md seed-0 numbers and the docs/media recording against the v0.5.16 re-acquisition.
# python tools/demo_seed0_check.py
"""Seed 0 cases of DEMO.md and the S5 recording: v0.5.17.1 against the v0.5.16 re-acquisition
(cap on the whole correction, old settle counting). Drawn point difference and the DEMO numbers."""
import sys, json
import numpy as np
sys.path.insert(0, 'tools'); sys.path.insert(0, '.')
import serve, simulate
OLD = ["reacq_cap_along_only=False", "reacq_settle_accepted_only=False", "reacq_rej_exit_n=0"]
orig = simulate.run
def run_old(key, seed, ov, z, eskf, *a):
    return orig(key, seed, ov, z, list(eskf) + OLD, *a)
cases = [("S5 (recording)", {'scenario': 'S5', 'seed': '0'}),
         ("S1", {'scenario': 'S1', 'seed': '0'}),
         ("S1 map off", {'scenario': 'S1', 'seed': '0', 'map_in_tunnel': '0'}),
         ("S4", {'scenario': 'S4', 'seed': '0'}),
         ("S3 p30 hybrid off", {'scenario': 'S3', 'seed': '0', 'baro_tunnel': 'baro_tunnel_30pa', 'hybrid': 'off'}),
         ("S3 p30 hybrid on", {'scenario': 'S3', 'seed': '0', 'baro_tunnel': 'baro_tunnel_30pa'}),
         ("S4 low-end", {'scenario': 'S4', 'seed': '0', 'imu': 'imu_lowend'})]
for name, q in cases:
    simulate.run = orig; serve.simulate.run = orig
    a = serve.run_data(dict(q))
    simulate.run = run_old; serve.simulate.run = run_old
    b = serve.run_data(dict(q))
    T = np.array(a['t']); te = a['t_exit']
    s = np.array(a['s_true']); xs = a['exits'][0]['s']
    i = int(np.argmax(s >= xs)); p = int(np.max(np.where(T < te)[0]))
    def num(d, j):
        return {k: round(float(d[k][j]), 1) for k in ('e_s', 'sig_s', 'e_app', 'sig_app', 'e_10', 'sig_10') if d.get(k) is not None and d[k][j] is not None}
    out = {}
    for key in ('app_xy', 'est_xy'):
        A, B = np.array(a[key], float), np.array(b[key], float)
        dd = np.linalg.norm(A - B, axis=1)
        out[key] = (round(float(np.nanmax(dd)), 2), round(float(np.nanmax(dd[T >= te])) if (T >= te).any() else 0, 2))
    print(name, '| max |Δ| drawn (app_xy) all/after exit', out['app_xy'], '| raw 16-state (est_xy)', out['est_xy'])
    print('   diverge', num(a, i), ' portal', num(a, p))
    print('   decisions', [e for e in a['events'] if 'branch' in json.dumps(e).lower() or 'decid' in json.dumps(e).lower()][:3])
    sys.stdout.flush()
