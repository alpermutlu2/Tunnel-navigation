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


# ---------------------------------------------------------------------------------------------
# v0.5: ESKF experiments
# ---------------------------------------------------------------------------------------------
CALLOUT_LEAD_M = 300.0      # [ASSUMED] the app announces the exit 300 m before the ramp
CALLOUT_LATE_S = 1.0        # [ASSUMED] a callout more than 1 s late counts as a late warning (review 6)
RECOVERY_M = 10.0           # recovered = horizontal error stays below this; ~2x the open-sky GNSS
                            # horizontal error (3 m per axis -> 4.2 m rms), so GNSS noise alone
                            # does not count as "not recovered"


def simulate_run(key, seeds, overrides=(), imu_switches=None, branch=True, vehicle=None):
    """Sensor data for a batch of seeds: IMU (no gaps; jitter off so the batch shares a time
    axis), GNSS and barometer at nominal settings plus overrides. branch=False: the car goes
    straight on at the diverge (v0.4 geometry); vehicle: replaces the scenario's Vehicle (e.g. no
    sideslip)."""
    from .sensors import BaroTruthSource, simulate_baro
    sc = build_scenarios(branch=branch)[key]
    if vehicle is not None:
        sc.vehicle = vehicle
    tr = generate_truth(sc)
    cfg = configure("no_gaps", *overrides)
    isrc, gsrc, bsrc = ImuTruthSource.from_truth(tr), GnssTruthSource.from_truth(tr, sc), BaroTruthSource.from_truth(tr, sc)
    sw = imu_switches or Switches.all_on().without("ts_jitter", "ts_gaps")
    F, W, G, H = [], [], [], []
    for s in seeds:
        m, _ = simulate_imu(isrc, cfg, sw, seed=int(s))
        g, _ = simulate_gnss(gsrc, cfg, Switches.all_on(), seed=int(s))
        b, _ = simulate_baro(bsrc, cfg, Switches.all_on(), seed=int(s))
        F.append(m.f), W.append(m.w), G.append(g), H.append(b.h)
    gnss = {"t": G[0].t, "pos": np.stack([g.pos for g in G]), "vel": np.stack([g.vel for g in G]),
            "acc_pos_h": np.stack([g.acc_pos_h for g in G]), "acc_pos_v": np.stack([g.acc_pos_v for g in G]),
            "acc_vel": np.stack([g.acc_vel for g in G])}
    return sc, tr, m.t, np.stack(F), np.stack(W), gnss, b.t, np.stack(H)


def eskf_experiment(key, seeds=50, overrides=(), zupt_oracle=False, eskf_cfg=None, keep_tracks=False,
                    branch=True, vehicle=None):
    """branch=False: negative control, the car goes straight on at the diverge while the map (and
    the branch check) still has the branch on the route (review 7)."""
    from .eskf import EskfConfig, run_eskf
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, np.arange(seeds), overrides, branch=branch, vehicle=vehicle)
    map_road = build_scenarios(branch=True)[key].road
    zmask = (tr.v < 0.05) if zupt_oracle else None
    out = run_eskf(t, F, W, gnss, bt, bh, tr.C_bp, tr.lever_arm_b, eskf_cfg or EskfConfig(), zupt_mask=zmask)
    i_e = int(np.argmax(tr.in_tunnel))
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_e, t_x = tr.t[i_e], tr.t[min(i_x, len(tr.t) - 1)]
    idx = np.clip(np.round((out.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    tn = _tangent(tr, idx)
    pR_hat = out.p - np.einsum("bkij,j->bki", out.C @ tr.C_bp.T, tr.lever_arm_b)
    d = pR_hat - tr.pos_n[idx][None]
    e_s = np.einsum("bki,ki->bk", d, tn)
    e_h = np.linalg.norm(d[..., :2], axis=-1)
    sig_s = np.sqrt(np.einsum("ki,bkij,kj->bk", tn, out.P_pos, tn))
    s_true = tr.s[idx]
    s_hat = s_true[None] + e_s

    def at(tt):
        return int(np.argmin(np.abs(out.t - tt)))

    k180, kx = at(t_e + 180.0), at(t_x - 0.05)
    res = {"scenario": key, "seeds": seeds, "overrides": list(overrides), "zupt": "oracle" if zupt_oracle else "detector",
           "exit_after_portal_s": round(t_x - t_e, 2), "events": out.events}
    res["fixed_180s"] = stats(e_s[:, k180])
    res["real_exit"] = stats(e_s[:, kx])
    z = e_s[:, kx] / sig_s[:, kx]
    res["consistency_real_exit"] = {"frac_within_2sigma": round(float(np.mean(np.abs(z) < 2)), 3),
                                    "mean_norm_sq": round(float(np.mean(z * z)), 3),
                                    "sigma_s_mean_m": round(float(sig_s[:, kx].mean()), 2)}
    # exit callout: announce when ŝ crosses s_ramp − lead
    if sc.road.exits:
        s_call = sc.road.exits[0].s - CALLOUT_LEAD_M
        k_true = int(np.argmax(s_true >= s_call))
        t_true = out.t[k_true]
        k_hat = np.argmax(s_hat >= s_call, axis=1)
        dt_call = out.t[k_hat] - t_true                      # > 0: the callout comes late
        v_call = float(np.linalg.norm(tr.vel_n[int(round((t_true - tr.t[0]) / tr.dt))]))
        sig_t = sig_s[:, k_true] / v_call                    # timing uncertainty the filter claims
        res["callout"] = {"lead_m": CALLOUT_LEAD_M, "timing_error_s": stats(dt_call),
                          "late_frac": round(float(np.mean(dt_call > 0)), 3),
                          "late_gt_1s_frac": round(float(np.mean(dt_call > CALLOUT_LATE_S)), 3),
                          "sigma_t_mean_s": round(float(sig_t.mean()), 3),
                          "within_2sigma_t": round(float(np.mean(np.abs(dt_call) < 2 * sig_t)), 3)}
    # exit branch: error at the true diverge (before/after the branch reported separately) and the
    # gyro branch confirmation (tunnelsim/branch.py; estimator side, no truth)
    if map_road.exits and map_road.exits[0].on_route:
        from .branch import branch_check
        ex = map_road.exits[0]
        k_div = int(np.argmax(s_true >= ex.s))
        t_div = out.t[k_div]
        res["at_diverge"] = stats(e_s[:, k_div])
        t_conf, choice = branch_check(t, W, out.t, out.C, out.p, out.v, map_road)
        delay = t_conf - t_div
        took = 1 if branch else -1                          # what the car really did
        right = choice == took
        res["branch_confirm"] = {"car": "branch" if branch else "main",
                                 "branch_reported_frac": round(float(np.mean(choice == 1)), 3),
                                 "main_reported_frac": round(float(np.mean(choice == -1)), 3),
                                 "none_frac": round(float(np.mean(choice == 0)), 3),
                                 "confirmed_frac": round(float(np.mean(right)), 3),
                                 "before_diverge_frac": round(float(np.mean(delay[choice != 0] < 0)) if (choice != 0).any() else 0.0, 3),
                                 "delay_s": stats(delay[right]) if right.any() else None}
    # recovery after the exit
    after = out.t >= t_x
    win = after & (out.t <= t_x + 60.0)
    jumps = np.abs(np.diff(s_hat[:, win], axis=1) - np.diff(s_true[win])[None])
    conv = []
    for b in range(seeds):
        kk = np.nonzero(win)[0]
        good = e_h[b, kk] < RECOVERY_M
        idx_ok = next((i for i in range(len(kk)) if good[i:].all()), None)
        conv.append(np.nan if idx_ok is None else out.t[kk[idx_ok]] - t_x)
    conv = np.array(conv)
    res["recovery"] = {"max_step_jump_m": stats(jumps.max(axis=1)),
                       "threshold_m": RECOVERY_M,
                       "time_to_threshold_s_median": float(np.nanmedian(conv)) if np.isfinite(conv).any() else None,
                       "not_converged_in_60s": int(np.isnan(conv).sum())}
    if keep_tracks:
        res["_tracks"] = {"t": out.t, "e_s": e_s, "sig_s": sig_s, "s_true": s_true, "t_entry": t_e, "t_exit": t_x}
    return res
