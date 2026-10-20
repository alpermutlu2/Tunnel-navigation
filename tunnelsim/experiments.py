"""v0.4 step 4 experiments (analysis harness; reads the truth for scoring and oracle flags).

Calibration + reference algorithms, ideal attitude (oracle upper bound for accel-based speed),
everything else from simulated measurements: IMU with all IMU error terms (no timing errors),
nominal GNSS. Entry state from GNSS: last fix before the degradation window (t_entry − 5 s),
velocity averaged over the 10 s before it; both CV and INS start from it.
"""
from __future__ import annotations

import numpy as np

from .budget import IMU_TERMS, _tangent, stats
from .profiles import NOMINAL, configure
from .reference import gm_conditional_shape, gnss_entry_state, ins_from_entry, ls_accel_bias
from .scenario import build_scenarios
from .sensors import GnssTruthSource, ImuTruthSource, Switches, bi_components, simulate_gnss, simulate_imu
from .truth import generate_truth

WINDOWS = (60.0, 120.0, 180.0)
MODES = ("hold", "condgm", "none")


def condgm_shape(prof, include_constant=True):
    """Shape for 'condgm' from the ASSUMED sensor model. Constant part: turn-on bias plus scale
    and misalignment acting on gravity (sigma * g)."""
    from .frames import G0
    N, B = prof.accel_vrw.a, prof.accel_bias_instab.a
    comps = [bi_components(N[i], B[i], prof) for i in range(3)]
    cv = 0.0
    if include_constant:
        cv = (float(prof.accel_turn_on_sigma.v) ** 2 + (float(prof.misalign_sigma.v) * G0) ** 2
              + (float(prof.accel_scale_sigma.v) * G0) ** 2)
    return lambda t: gm_conditional_shape(t, [c[0] for c in comps], [c[1] for c in comps], cv)


def calibration_experiment(key, seeds=100, imu_overrides=(), zupt_oracle=False) -> dict:
    sc = build_scenarios()[key]
    tr = generate_truth(sc)
    cfg = configure("no_gaps", *imu_overrides)
    isrc, gsrc = ImuTruthSource.from_truth(tr), GnssTruthSource.from_truth(tr, sc)
    i_e = int(np.argmax(tr.in_tunnel))
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_e, t_x = tr.t[i_e], tr.t[min(i_x, len(tr.t) - 1)]
    t_ref = t_e - float(cfg.gnss.degrade_s.v) - 1.0
    C_np = tr.C_nb @ tr.C_bp
    shape = condgm_shape(cfg.imu)
    eval_t = {"fixed_180s_ins_only": t_e + 180.0, "real_exit": t_x}
    k_eval = {n: int(round((tt - tr.t[0]) / tr.dt)) for n, tt in eval_t.items()}
    F, W, B, P0, V0, T0 = [], [], {T: [] for T in WINDOWS}, [], [], []
    for s in range(seeds):
        m, _ = simulate_imu(isrc, cfg, Switches.only(*IMU_TERMS), seed=s)
        g, _ = simulate_gnss(gsrc, cfg, Switches.all_on(), seed=s)
        t_fix, p0, v0 = gnss_entry_state(g.t, g.pos, g.vel, t_ref)
        for T in WINDOWS:
            b, _ = ls_accel_bias(m.t, m.f, C_np, g.t, g.vel, t_fix - T, t_fix)
            B[T].append(b)
        F.append(m.f), W.append(m.w), P0.append(p0), V0.append(v0), T0.append(t_fix)
    assert len(set(T0)) == 1, "entry fix time must be common (1 Hz grid, no GNSS time offset)"
    t_fix = T0[0]
    F, W, P0, V0 = np.stack(F), np.stack(W), np.stack(P0), np.stack(V0)
    k_fix = int(round((t_fix - tr.t[0]) / tr.dt))
    out = {"scenario": key, "seeds": seeds, "overrides": list(imu_overrides),
           "entry_fix_before_portal_s": round(t_e - t_fix, 2),
           "exit_after_portal_s": round(t_x - t_e, 2), "results": {}}

    def score(res, i0):
        r = {}
        for name, k in k_eval.items():
            kk = k - i0
            pP = res.p_n[:, kk]
            pR = pP - (C_np[k] @ tr.C_bp.T @ tr.lever_arm_b)
            r[name] = stats((pR - tr.pos_n[k]) @ _tangent(tr, np.array([k]))[0])
        return r

    zmask = None
    for T in WINDOWS:
        b = np.stack(B[T])
        for mode in MODES:
            if mode == "none" and T != WINDOWS[0]:
                continue
            res, i0 = ins_from_entry(tr.t, F, W, C_np, t_fix, P0, V0, b, mode, shape)
            out["results"][f"ins_{mode}" + ("" if mode == "none" else f"_{int(T)}s")] = score(res, i0)
            if zupt_oracle and mode in ("hold", "condgm") and T == WINDOWS[-1]:
                from .ins import ImuCalibration, InsInit, strapdown
                zmask = tr.v[i0:] < 1e-3
                cal = ImuCalibration(accel_bias=b) if mode == "hold" else \
                    ImuCalibration(accel_bias=b, accel_bias_shape=shape(tr.t[i0:] - tr.t[i0]))
                rz = strapdown(tr.t[i0:], F[:, i0:], W[:, i0:], InsInit(P0, V0, C_np[i0]), cal,
                               attitude=C_np[i0:], zero_velocity=zmask)
                out["results"][f"ins_{mode}_{int(T)}s_zupt_oracle"] = score(rz, i0)
    # constant velocity from the same entry state, along the road
    vh = np.linalg.norm(V0[:, :2], axis=1)
    tn_fix = _tangent(tr, np.array([k_fix]))[0]
    ds0 = (P0 - tr.pos_phone_n[k_fix]) @ tn_fix
    cv = {}
    for name, k in k_eval.items():
        cv[name] = stats(ds0 + vh * (tr.t[k] - t_fix) - (tr.s[k] - tr.s[k_fix]))
    out["results"]["cv"] = cv
    out["entry_speed_error_mps"] = stats(vh - tr.v[k_fix])
    return out
