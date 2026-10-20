"""Structural observability rank check of the ESKF error model (v0.5; focus decision b).

Analysis harness: linearises the filter's own 16-state error model (eskf.py) about the TRUE
trajectory and stacks the observability matrix O = [H_k Φ_k,0] over a window. States are scaled
by representative magnitudes before the SVD so that unit choices do not decide the rank.
Rank and the weakest right-singular directions show which state combinations the measurements
cannot separate. Priors are excluded on purpose (a small posterior covariance can come from a
prior, which is not observability).
"""
from __future__ import annotations

import numpy as np

from .eskf import NX, skew_b
from .frames import GRAVITY_N

NAMES = ["dp_e", "dp_n", "dp_u", "dv_e", "dv_n", "dv_u", "dth_e", "dth_n", "dth_u",
         "dba_x", "dba_y", "dba_z", "dbg_x", "dbg_y", "dbg_z", "dbh"]
SCALE = np.r_[[1.0] * 3, [0.1] * 3, [np.deg2rad(0.1)] * 3, [1e-2] * 3, [1e-4] * 3, 1.0]


def observability(tr, t0, t1, gnss=True, nhc=True, baro=True, dt=1.0, tol=1e-6):
    C_np = tr.C_nb @ tr.C_bp
    C_bp = tr.C_bp
    ks = np.arange(int(round((t0 - tr.t[0]) / tr.dt)), int(round((t1 - tr.t[0]) / tr.dt)), int(round(dt / tr.dt)))
    Phi = np.eye(NX)
    rows = []
    for j, k in enumerate(ks):
        C = C_np[k]
        f_n = C @ tr.f_p[k]
        v = tr.vel_phone_n[k]
        Hs = []
        if gnss:
            H = np.zeros((6, NX)); H[0:3, 0:3] = np.eye(3); H[3:6, 3:6] = np.eye(3); Hs.append(H)
        if nhc and np.linalg.norm(v) > 1.0:
            C_bn = C_bp @ C.T
            H = np.zeros((2, NX)); H[:, 3:6] = C_bn[1:3]; H[:, 6:9] = (C_bn @ skew_b(v[None])[0])[1:3]
            Hs.append(H)
        if baro:
            H = np.zeros((1, NX)); H[0, 2] = 1.0; H[0, 15] = 1.0; Hs.append(H)
        for H in Hs:
            rows.append(H @ Phi)
        F = np.zeros((NX, NX))
        F[0:3, 3:6] = np.eye(3); F[3:6, 6:9] = -skew_b(f_n[None])[0]; F[3:6, 9:12] = -C; F[6:9, 12:15] = -C
        Phi = (np.eye(NX) + F * dt + 0.5 * F @ F * dt * dt) @ Phi
    O = np.vstack(rows) * SCALE[None, :]
    U, s, Vt = np.linalg.svd(O, full_matrices=False)
    s = s / s[0]
    rank = int(np.sum(s > tol))
    weak = []
    for i in range(1, 4):
        v = Vt[-i]
        top = np.argsort(-np.abs(v))[:4]
        weak.append({"rel_singular_value": float(f"{s[-i]:.2e}"),
                     "direction": {NAMES[t]: round(float(v[t]), 3) for t in top}})
    return {"window_s": [round(t0, 1), round(t1, 1)], "measurements": {"gnss": gnss, "nhc": nhc, "baro": baro},
            "rank": rank, "of": NX, "weakest": weak}


def rank_report(key="S1"):
    from .scenario import build_scenarios
    from .truth import generate_truth
    tr = generate_truth(build_scenarios(branch=False)[key])
    t_e = tr.t[int(np.argmax(tr.in_tunnel))]
    return {
        "approach_with_maneuvers_gnss_nhc_baro": observability(tr, 10.0, 90.0),
        "steady_approach_gnss_nhc_baro": observability(tr, 100.0, t_e - 5.0),
        "tunnel_nhc_baro_only": observability(tr, t_e, t_e + 120.0, gnss=False),
    }
