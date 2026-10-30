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
from .scenario import build_scenarios, get_scenario
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
CALLOUT_K_SWEEP = (0.0, 1.0, 1.3, 1.65, 2.0)
CALLOUT_K = 1.0             # owner 2026-10-21: announce when ŝ + k·σ_s reaches the callout point (errs early);
                            # σ_s is ~2× conservative ((e/σ)² 0.1–0.3), so k = 1 is in effect ~2σ; re-select k on real data
CALLOUT_LATE_S = 1.0        # [ASSUMED] a callout more than 1 s late counts as a late warning (review 6)
RECOVERY_M = 10.0           # recovered = horizontal error stays below this; ~2x the open-sky GNSS
                            # horizontal error (3 m per axis -> 4.2 m rms), so GNSS noise alone
                            # does not count as "not recovered"


CUPHOLDER_RPY_DEG = (10.0, -20.0, 25.0)    # [ASSUMED] phone in a cup holder: offset from the dash mount


def filter_mount(tr, t, F, W, gnss, seed_ids, mount="known", mount_err_deg=None):
    """The mount the filters use, and the IMU data in the frame they expect (v0.5.13).
    mount "known": the true mount (or it offset by mount_err_deg, the v0.5.0 test).
    mount "estimated": tunnelsim/mount_init.py on the open-sky approach (gravity tilt, magnetometer
    yaw, velocity-direction yaw); the data of each member is rotated into a virtual phone frame so
    one nominal C_bp serves the batch: f_v = C_nomᵀ Ĉ_bp f_p, hence C_nom f_v = Ĉ_bp f_p.
    Returns F, W, C_bp for the filters and a dict of diagnostics (mount errors in degrees)."""
    from .frames import C_BP_NOMINAL, mount_rotation
    if mount == "known" or mount_err_deg is not None:
        C = tr.C_bp if mount_err_deg is None else mount_rotation(*mount_err_deg) @ C_BP_NOMINAL.T @ tr.C_bp
        return F, W, C, None
    from .magnetometer import simulate_mag
    from .mount_init import estimate_mount, mount_error_deg
    mt, M = simulate_mag(tr, seed_ids)
    use_mag, use_acc = {"estimated": (True, True), "estimated_mag": (True, False)}[mount]
    C_hat, info = estimate_mount(t, F, W, mt, M, gnss, use_mag=use_mag, use_accel=use_acc)
    R = np.einsum("ji,bjk->bik", C_BP_NOMINAL, C_hat)                 # C_nomᵀ Ĉ_bp
    err = mount_error_deg(C_hat, tr.C_bp)
    diag = {"mode": mount, "err_rms_deg": np.sqrt((err ** 2).mean(0)).round(2).tolist(),
            "yaw_err_p95_deg": round(float(np.percentile(np.abs(err[:, 2]), 95)), 2),
            "yaw_from": {k_: sum(1 for i in info if i["yaw_from"] == k_) for k_ in ("mag", "mag+accel", "accel", "none")}}
    # the ESKF's mount-yaw state prior: magnetometer only ~6° rms (measured, S1 10 seeds), with the
    # velocity-direction step 0.5° rms, taken as 1° [ASSUMED]; the batch shares one scenario, so one value
    diag["yaw_sigma_deg"] = 6.0 if diag["yaw_from"]["mag"] > len(info) / 2 else 1.0
    return np.einsum("bij,bnj->bni", R, F), np.einsum("bij,bnj->bni", R, W), C_BP_NOMINAL, diag


def simulate_run(key, seeds, overrides=(), imu_switches=None, branch=True, vehicle=None, speed_scale=1.0,
                 speed_change=None, approach_manoeuvre=True):
    """Sensor data for a batch of seeds: IMU (no gaps; jitter off so the batch shares a time
    axis), GNSS and barometer at nominal settings plus overrides. branch=False: the car goes
    straight on at the diverge (v0.4 geometry); vehicle: replaces the scenario's Vehicle (e.g. no
    sideslip)."""
    from .sensors import BaroTruthSource, simulate_baro
    sc = get_scenario(key, branch=branch)
    if speed_scale != 1.0:
        # live slider (owner 2026-10-23): every speed of the scenario × factor; positions unchanged
        from dataclasses import replace as _rp
        sc.speed = type(sc.speed)(v0=sc.speed.v0 * speed_scale,
                                  events=[_rp(e, v_target=e.v_target * speed_scale) for e in sc.speed.events])
    if speed_change and speed_change[0]:
        # live slider (owner 2026-10-23): inside the tunnel at fraction `at` of its length, change
        # speed by dv km/h from the scenario's cruise speed over 8 s, hold 20 s, return over 8 s
        from dataclasses import replace as _rp
        from .scenario import SpeedEvent, kmh
        dv, at = float(speed_change[0]), float(speed_change[1])
        s_at = sc.road.tunnel.s_entry + at * (sc.road.tunnel.s_exit - sc.road.tunnel.s_entry)
        v_c = sc.speed.v0
        evs = list(sc.speed.events) + [SpeedEvent(s_at, max(v_c + kmh(dv), kmh(10)), 8.0, dwell=20.0),
                                       SpeedEvent(s_at, v_c, 8.0)]
        evs.sort(key=lambda e: e.at_s)     # stable: the change and its return keep their order
        sc.speed = type(sc.speed)(v0=sc.speed.v0, events=evs)
    if vehicle is not None:
        sc.vehicle = vehicle
    if not approach_manoeuvre:
        # v0.5.13: no speed change before the tunnel (the approach is straight), so nothing excites the
        # mount yaw in open sky
        sc.speed = type(sc.speed)(v0=sc.speed.v0, events=[e for e in sc.speed.events if e.at_s >= sc.road.tunnel.s_entry])
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
                    branch=True, vehicle=None, mount_err_deg=None, map_err=None, mount="estimated", approach_manoeuvre=True):
    """branch=False: negative control, the car goes straight on at the diverge while the map (and
    the branch check) still has the branch on the route (review 7)."""
    from .eskf import EskfConfig, run_eskf
    seed_ids = np.arange(seeds) if np.isscalar(seeds) else np.asarray(seeds)
    seeds = len(seed_ids)
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, seed_ids, overrides, branch=branch, vehicle=vehicle,
                                                 approach_manoeuvre=approach_manoeuvre)
    map_road = get_scenario(key, branch=True).road
    zmask = (tr.v < 0.05) if zupt_oracle else None
    # assumption removal (C): mount_err_deg offsets the filter's mount; v0.5.13: mount="estimated"
    F, W, C_bp_filter, mount_diag = filter_mount(tr, t, F, W, gnss, seed_ids, mount, mount_err_deg)
    C_bp_eval = tr.C_bp if mount == "known" else C_bp_filter     # the body frame as the filter holds it
    from .mapview import MapView
    mv = MapView(map_road, *(map_err or (0.0, 1.0)))     # map_err: (lateral offset m, length scale)
    cfg_e = eskf_cfg or EskfConfig()
    if mount_diag is not None:
        from dataclasses import replace as _rp
        cfg_e = _rp(cfg_e, mount_yaw_sigma=np.deg2rad(mount_diag["yaw_sigma_deg"]))
    out = run_eskf(t, F, W, gnss, bt, bh, C_bp_filter, tr.lever_arm_b, cfg_e, zupt_mask=zmask,
                   road_map=mv)
    i_e = int(np.argmax(tr.in_tunnel))
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_e, t_x = tr.t[i_e], tr.t[min(i_x, len(tr.t) - 1)]
    idx = np.clip(np.round((out.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    tn = _tangent(tr, idx)
    pR_hat = out.p - np.einsum("bkij,j->bki", out.C @ C_bp_eval.T, tr.lever_arm_b)
    d = pR_hat - tr.pos_n[idx][None]
    e_s = np.einsum("bki,ki->bk", d, tn)
    e_h = np.linalg.norm(d[..., :2], axis=-1)
    # cross-track (left of the road +) and heading error of the body x axis vs the road (review 9)
    nrm = np.stack([-tn[:, 1], tn[:, 0], np.zeros(len(tn))], 1)
    e_lat = np.einsum("bki,ki->bk", d, nrm)
    fwd = np.einsum("bkij,j->bki", out.C @ C_bp_eval.T, np.array([1.0, 0.0, 0.0]))
    psi_hat = np.arctan2(fwd[..., 1], fwd[..., 0])
    fwd_t = np.einsum("kij,j->ki", tr.C_nb[idx], np.array([1.0, 0.0, 0.0]))
    e_psi = np.rad2deg(np.angle(np.exp(1j * (psi_hat - np.arctan2(fwd_t[:, 1], fwd_t[:, 0])[None]))))
    sig_s = np.sqrt(np.einsum("ki,bkij,kj->bk", tn, out.P_pos, tn))
    s_true = tr.s[idx]
    s_hat = s_true[None] + e_s

    def at(tt):
        return int(np.argmin(np.abs(out.t - tt)))

    k180, kx = at(t_e + 180.0), at(t_x - 0.05)
    res = {"scenario": key, "seeds": seeds, "overrides": list(overrides), "zupt": "oracle" if zupt_oracle else "detector",
           "exit_after_portal_s": round(t_x - t_e, 2), "events": out.events}
    if mount_diag is not None:
        res["mount"] = mount_diag
    res["fixed_180s"] = stats(e_s[:, k180])
    res["real_exit"] = stats(e_s[:, kx])
    res["real_exit_lateral"] = stats(e_lat[:, kx])
    res["real_exit_heading_deg"] = stats(e_psi[:, kx])
    z = e_s[:, kx] / sig_s[:, kx]
    res["consistency_real_exit"] = {"frac_within_2sigma": round(float(np.mean(np.abs(z) < 2)), 3),
                                    "mean_norm_sq": round(float(np.mean(z * z)), 3),
                                    "sigma_s_mean_m": round(float(sig_s[:, kx].mean()), 2)}
    # exit callout: announce when ŝ crosses s_ramp − lead
    if sc.road.exits:
        s_call = sc.road.exits[0].s - CALLOUT_LEAD_M
        k_true = int(np.argmax(s_true >= s_call))
        t_true = out.t[k_true]
        k_hat = np.argmax(s_hat + CALLOUT_K * sig_s >= s_call, axis=1)
        dt_call = out.t[k_hat] - t_true                      # > 0: the callout comes late
        v_call = float(np.linalg.norm(tr.vel_n[int(round((t_true - tr.t[0]) / tr.dt))]))
        sig_t = sig_s[:, k_true] / v_call                    # timing uncertainty the filter claims
        res["callout"] = {"lead_m": CALLOUT_LEAD_M, "k_sigma": CALLOUT_K, "timing_error_s": stats(dt_call),
                          "late_frac": round(float(np.mean(dt_call > 0)), 3),
                          "late_gt_1s_frac": round(float(np.mean(dt_call > CALLOUT_LATE_S)), 3),
                          "sigma_t_mean_s": round(float(sig_t.mean()), 3),
                          "within_2sigma_t": round(float(np.mean(np.abs(dt_call) < 2 * sig_t)), 3)}
        # uncertainty-aware callout (owner, 2026-10-21): announce when ŝ + k·σ_s reaches the
        # callout point, so the callout errs early; k = 0 is the plain callout
        res["callout_k"] = {}
        for kk_ in CALLOUT_K_SWEEP:
            k_h = np.argmax(s_hat + kk_ * sig_s >= s_call, axis=1)
            dtk = out.t[k_h] - t_true
            res["callout_k"][str(kk_)] = {"rmse_s": round(float(np.sqrt(np.mean(dtk ** 2))), 2),
                                          "mean_s": round(float(dtk.mean()), 2),
                                          "late_frac": round(float(np.mean(dtk > 0)), 3),
                                          "late_gt_1s_frac": round(float(np.mean(dtk > CALLOUT_LATE_S)), 3),
                                          "early_p95_s": round(float(np.percentile(-dtk, 95)), 2)}
    # exit branch: error at the true diverge (before/after the branch reported separately) and the
    # gyro branch confirmation (tunnelsim/branch.py; estimator side, no truth)
    if map_road.exits and map_road.exits[0].on_route:
        from .branch import branch_check
        ex = map_road.exits[0]
        k_div = int(np.argmax(s_true >= ex.s))
        t_div = out.t[k_div]
        res["at_diverge"] = stats(e_s[:, k_div])
        res["at_diverge_lateral"] = stats(e_lat[:, k_div])
        took = 1 if branch else -1                          # what the car really did

        def summary(t_conf, choice):
            delay = t_conf - t_div
            right = choice == took
            return {"car": "branch" if branch else "main",
                    "branch_reported_frac": round(float(np.mean(choice == 1)), 3),
                    "main_reported_frac": round(float(np.mean(choice == -1)), 3),
                    "none_frac": round(float(np.mean(choice == 0)), 3),
                    "confirmed_frac": round(float(np.mean(right)), 3),
                    "before_diverge_frac": round(float(np.mean(delay[choice != 0] < 0)) if (choice != 0).any() else 0.0, 3),
                    "delay_s": stats(delay[right]) if right.any() else None}
        # v0.5.12 (owner): how long the decision stays "not yet" after the car passes the diverge,
        # and what the screen shows meanwhile: the map point nearest the estimate on the route
        # polyline (the branch is on the route), scored horizontally against the true position
        bs = out.aux.get("branch_state")
        if bs is not None:
            after_div = (out.t >= t_div) & (out.t <= t_x)
            und = (bs == 0) & after_div[None]
            dur = und.sum(axis=1) * (out.t[1] - out.t[0])
            derr = np.zeros(len(seed_ids)); doff = np.zeros(len(seed_ids))
            road_xy = tr.pos_n[::max(1, int(round(0.05 / tr.dt))), :2]     # the road the car drove, 0.05 s apart
            for b in np.nonzero(und.any(axis=1))[0]:
                kk_ = np.nonzero(und[b])[0]
                s_d = mv.nearest(pR_hat[b, kk_, :2], np.full(len(kk_), np.nan))
                c_d, _, _ = mv.at(s_d)
                derr[b] = float(np.linalg.norm(c_d - tr.pos_n[idx[kk_], :2], axis=1).max())
                doff[b] = float(np.sqrt(((c_d[:, None] - road_xy[None]) ** 2).sum(-1).min(axis=1)).max())
            res["undecided"] = {"after_diverge_s": stats(dur), "any_frac": round(float(np.mean(dur > 0)), 3),
                                "at_portal_frac": round(float(np.mean(bs[:, -1] == 0)), 3),
                                "display_err_max_m": stats(derr),
                                "display_err_gt_20m": int((derr > 20.0).sum()),
                                "display_off_road_max_m": stats(doff),     # distance to the road the car is on
                                "display_off_road_gt_10m": int((doff > 10.0).sum())}
        # offline check (branch.py, run after the fact on the filter output; independent reference)
        res["branch_offline"] = summary(*branch_check(t, W, out.t, out.C, out.p, out.v, map_road, out_P_pos=out.P_pos))
        # in-loop decision of the map layer (the one the app acts on); offline when the layer is off
        dec = out.aux.get("branch_decision")
        if dec is not None and (dec != 0).any():
            res["branch_confirm"] = summary(out.aux["branch_time"], dec)
            res["branch_confirm"]["source"] = "in-loop (map layer)"
        else:
            res["branch_confirm"] = dict(res["branch_offline"], source="offline (branch.py)")
    # v0.5.13: recovery time and prompt timing (tunnelsim/navmetrics.py), from the first fix of any kind
    from . import navmetrics as NM
    t_back = np.full(len(seed_ids), np.inf)
    for b_ in range(len(seed_ids)):
        anyf = gnss["t"][(np.isfinite(gnss["pos"][b_, :, 0]) | np.isfinite(gnss["vel"][b_, :, 0])) & (gnss["t"] > t_x - 5.0)]
        if len(anyf):
            t_back[b_] = anyf[0]
    rec, worst = NM.recovery(out.t, e_s, t_back)
    res["recovery_time"] = NM.summarise_recovery(rec, worst)
    if sc.road.exits:
        res["prompts"] = NM.summarise_prompts(NM.prompts(out.t, s_hat, sig_s, s_true, np.linalg.norm(out.v, axis=-1),
                                                         sc.road.exits[0].s, CALLOUT_K))
    # recovery after the exit
    after = out.t >= t_x
    win = after & (out.t <= t_x + 60.0)
    jumps = np.abs(np.diff(s_hat[:, win], axis=1) - np.diff(s_true[win])[None])
    conv = []
    for b in range(len(seed_ids)):
        kk = np.nonzero(win)[0]
        good = e_h[b, kk] < RECOVERY_M
        idx_ok = next((i for i in range(len(kk)) if good[i:].all()), None)
        conv.append(np.nan if idx_ok is None else out.t[kk[idx_ok]] - t_x)
    conv = np.array(conv)
    res["recovery"] = {"max_step_jump_m": stats(jumps.max(axis=1)),
                       "threshold_m": RECOVERY_M,
                       "time_to_threshold_s_median": float(np.nanmedian(conv)) if np.isfinite(conv).any() else None,
                       "not_converged_in_60s": int(np.isnan(conv).sum())}
    if out.aux.get("imu_class") is not None:
        cls, cnt = np.unique(out.aux["imu_class"], return_counts=True)
        res["imu_class_detected"] = {str(c): int(n) for c, n in zip(cls, cnt)}
    if keep_tracks:
        res["_tracks"] = {"t": out.t, "e_s": e_s, "e_lat": e_lat, "e_psi": e_psi, "sig_s": sig_s, "s_true": s_true, "t_entry": t_e, "t_exit": t_x}
    return res


# ---------------------------------------------------------------------------------------------
# v0.5.9: 1D map-constrained EKF baseline (tunnelsim/reference1d.py), scored like the ESKF
# ---------------------------------------------------------------------------------------------
def ekf1d_experiment(key, seeds=50, overrides=(), use_grade=True, mount_err_deg=None, keep_tracks=False,
                     eskf_cfg=None, sim=None, grade_scale=1.0):
    """Same sensors and seeds as eskf_experiment (simulate_run); along-track metrics only (the 1D
    filter has no cross-track). use_grade=False: the map has no grade. sim: a simulate_run tuple
    to reuse (the simulator page runs both filters on one batch)."""
    from .eskf import EskfConfig
    from .reference1d import Ekf1dConfig, run_ekf1d
    seed_ids = np.arange(seeds) if np.isscalar(seeds) else np.asarray(seeds)
    sc, tr, t, F, W, gnss, bt, bh = sim if sim is not None else simulate_run(key, seed_ids, overrides)
    map_road = get_scenario(key, branch=True).road if sim is None else sc.road
    C_bp_filter = tr.C_bp
    if mount_err_deg is not None:
        from .frames import C_BP_NOMINAL, mount_rotation
        C_bp_filter = mount_rotation(*mount_err_deg) @ C_BP_NOMINAL.T @ tr.C_bp
    out = run_ekf1d(t, F, W, gnss, C_bp_filter, tr.lever_arm_b, map_road, Ekf1dConfig(use_grade=use_grade, grade_scale=grade_scale),
                    eskf_cfg or EskfConfig())
    i_e = int(np.argmax(tr.in_tunnel))
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_e, t_x = tr.t[i_e], tr.t[min(i_x, len(tr.t) - 1)]
    idx = np.clip(np.round((out.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    s_true = tr.s[idx]
    e_s = out.s - s_true[None]
    sig_s = out.sig_s

    def at(tt):
        return int(np.argmin(np.abs(out.t - tt)))

    kx = at(t_x - 0.05)
    res = {"scenario": key, "filter": "1D map-constrained EKF", "seeds": len(seed_ids), "overrides": list(overrides),
           "use_grade": use_grade, "events": out.events,
           "real_exit": stats(e_s[:, kx]), "fixed_180s": stats(e_s[:, at(t_e + 180.0)])}
    z = e_s[:, kx] / sig_s[:, kx]
    res["consistency_real_exit"] = {"frac_within_2sigma": round(float(np.mean(np.abs(z) < 2)), 3),
                                    "mean_norm_sq": round(float(np.mean(z * z)), 3),
                                    "sigma_s_mean_m": round(float(sig_s[:, kx].mean()), 2)}
    if sc.road.exits:
        ex = sc.road.exits[0]
        res["at_diverge"] = stats(e_s[:, int(np.argmax(s_true >= ex.s))])
        s_call = ex.s - CALLOUT_LEAD_M
        k_true = int(np.argmax(s_true >= s_call))
        k_hat = np.argmax(out.s + CALLOUT_K * sig_s >= s_call, axis=1)
        dt_call = out.t[k_hat] - out.t[k_true]
        res["callout"] = {"k_sigma": CALLOUT_K, "timing_error_s": stats(dt_call),
                          "late_gt_1s_frac": round(float(np.mean(dt_call > CALLOUT_LATE_S)), 3)}
    win = (out.t >= t_x) & (out.t <= t_x + 60.0)
    jumps = np.abs(np.diff(out.s[:, win], axis=1) - np.diff(s_true[win])[None])
    res["recovery"] = {"max_step_jump_m": stats(jumps.max(axis=1))}
    cls, cnt = np.unique(out.imu_class, return_counts=True)
    res["imu_class_detected"] = {str(c): int(n) for c, n in zip(cls, cnt)}
    if keep_tracks:
        res["_tracks"] = {"t": out.t, "e_s": e_s, "sig_s": sig_s, "s_hat": out.s, "v_hat": out.v,
                          "t_entry": t_e, "t_exit": t_x}
    return res


# ---------------------------------------------------------------------------------------------
# v0.5.11: 10-state map-constrained filter (tunnelsim/ekf10.py), scored like the 1D baseline
# ---------------------------------------------------------------------------------------------
def ekf10_experiment(key, seeds=50, overrides=(), mount_err_deg=None, cfg10=None, eskf_cfg=None, sim=None,
                     branch=True, keep_tracks=False, mount="estimated", vehicle=None, approach_manoeuvre=True):
    """Along-track metrics at the diverge and the portal, consistency, callout timing (k = 0, 1),
    exit step. branch=False: negative control (the car goes straight on, the filter's route takes
    the branch; the 10-state filter has no second hypothesis), scored as the horizontal distance
    between the displayed position on the route and the car at the portal."""
    from .eskf import EskfConfig
    from .ekf10 import Ekf10Config, run_ekf10
    from .frames import C_BP_NOMINAL, mount_rotation
    seed_ids = np.arange(seeds) if np.isscalar(seeds) else np.asarray(seeds)
    sc, tr, t, F, W, gnss, bt, bh = sim if sim is not None else simulate_run(key, seed_ids, overrides, branch=branch,
                                                                             vehicle=vehicle, approach_manoeuvre=approach_manoeuvre)
    road = get_scenario(key, branch=True).road
    F, W, C_bp_f, mount_diag = filter_mount(tr, t, F, W, gnss, seed_ids, mount, mount_err_deg)
    if mount_diag is not None:
        from dataclasses import replace as _rp
        cfg10 = _rp(cfg10 or Ekf10Config(), mount_yaw_sigma=np.deg2rad(mount_diag["yaw_sigma_deg"]))
    o = run_ekf10(t, F, W, gnss, bt, bh, C_bp_f, tr.lever_arm_b, road, cfg10 or Ekf10Config(), eskf_cfg or EskfConfig())
    kt = np.clip(np.round((o.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    s_true = tr.s[kt]
    e = o.s - s_true[None]
    i_e = int(np.argmax(tr.in_tunnel))
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_x = tr.t[min(i_x, len(tr.t) - 1)]
    k_x = int(np.argmin(np.abs(o.t - (t_x - 0.05))))
    res = {"scenario": key, "filter": "10-state map-constrained", "seeds": int(len(seed_ids)),
           "overrides": list(overrides), "events": o.events,
           "imu_class_detected": {str(c): int(n) for c, n in zip(*np.unique(o.imu_class, return_counts=True))}}
    if mount_diag is not None:
        res["mount"] = mount_diag
    if o.branch_decision is not None:          # v0.5.12: the 10-state filter's own branch decision
        bd = o.branch_decision
        res["branch10"] = {"branch": round(float(np.mean(bd == 1)), 3), "main": round(float(np.mean(bd == -1)), 3),
                           "none": round(float(np.mean(bd == 0)), 3)}
    if not branch:
        disp = road.position(o.s[:, k_x])[:, :2]
        res["portal_horizontal_m"] = stats(np.linalg.norm(disp - tr.pos_n[kt[k_x], :2][None], axis=1))
        return res
    z = e[:, k_x] / o.sig_s[:, k_x]
    res["real_exit"] = stats(e[:, k_x])
    res["consistency_real_exit"] = {"frac_within_2sigma": round(float(np.mean(np.abs(z) < 2)), 3),
                                    "mean_norm_sq": round(float(np.mean(z * z)), 3),
                                    "sigma_s_mean_m": round(float(o.sig_s[:, k_x].mean()), 2)}
    ex = sc.road.exits[0]
    s_call = ex.s - CALLOUT_LEAD_M
    k_div = int(np.argmax(s_true >= ex.s)); k_ct = int(np.argmax(s_true >= s_call))
    res["at_diverge"] = stats(e[:, k_div])
    res["callout_k"] = {}
    for kk_ in (0.0, CALLOUT_K):
        m = (o.s + kk_ * o.sig_s) >= s_call
        k_hat = np.where(m.any(1), np.argmax(m, 1), m.shape[1] - 1)
        d = o.t[k_hat] - o.t[k_ct]
        res["callout_k"][str(kk_)] = {"rmse_s": round(float(np.sqrt(np.mean(d ** 2))), 2), "mean_s": round(float(d.mean()), 2),
                                      "late_frac": round(float(np.mean(d > 0)), 3),
                                      "late_gt_1s_frac": round(float(np.mean(d > CALLOUT_LATE_S)), 3)}
    win = (o.t >= t_x) & (o.t <= t_x + 60.0)
    jumps = np.abs(np.diff(o.s[:, win], axis=1) - np.diff(s_true[win])[None])
    res["recovery"] = {"max_step_jump_m": stats(jumps.max(axis=1))}
    if keep_tracks:
        res["_tracks"] = {"t": o.t, "e_s": e, "sig_s": o.sig_s, "s_hat": o.s, "v_hat": o.v, "t_entry": tr.t[i_e], "t_exit": t_x}
    return res


# ---------------------------------------------------------------------------------------------
# v0.5.11: supervisor over the three filters (tunnelsim/supervisor.py)
# ---------------------------------------------------------------------------------------------
def _eskf_along(mv, t_out, p, C, P_pos, C_bp, r_b):
    """Estimator-side along-track position of the reference point and its σ from an ESKF output:
    ŝ by a tangent-plane step from the previous epoch (one Newton step on the map polyline)."""
    pR = p - np.einsum("bkij,j->bki", C @ C_bp.T, r_b)
    B, K = pR.shape[:2]
    s = np.empty((B, K)); g = np.empty((B, K))
    s_prev = mv.nearest(pR[:, 0, :2], np.full(B, np.nan))
    for k in range(K):
        c, psi, _ = mv.at(s_prev)
        tn = np.stack([np.cos(psi), np.sin(psi)], 1)
        s_k = s_prev + np.einsum("bi,bi->b", pR[:, k, :2] - c, tn)
        c, psi, _ = mv.at(s_k)
        tn = np.stack([np.cos(psi), np.sin(psi)], 1)
        s_k = s_k + np.einsum("bi,bi->b", pR[:, k, :2] - c, tn)
        s[:, k] = s_k
        g[:, k] = np.sqrt(np.einsum("bi,bij,bj->b", tn, P_pos[:, k, :2, :2], tn))
        s_prev = s_k
    return s, g


def _three_filters(t, F, W, gnss, bt, bh, C_bp, r_b, road, mv, k_end=None, eskf_cfg=None):
    """Run the 1D, 10-state and ESKF filters on one batch; along-track outputs on the ESKF time axis."""
    from .eskf import EskfConfig, run_eskf
    from .ekf10 import run_ekf10
    from .reference1d import run_ekf1d
    if k_end is not None:
        t, F, W = t[:k_end], F[:, :k_end], W[:, :k_end]
        m = bt <= t[-1]
        bt, bh = bt[m], bh[:, m]
    oe = run_eskf(t, F, W, gnss, bt, bh, C_bp, r_b, eskf_cfg or EskfConfig(), road_map=mv)
    s_e, g_e = _eskf_along(mv, oe.t, oe.p, oe.C, oe.P_pos, C_bp, r_b)
    from .ekf10 import Ekf10Config
    o10 = run_ekf10(t, F, W, gnss, bt, bh, C_bp, r_b, road,            # its own branch decision (v0.5.12)
                    Ekf10Config(mount_yaw_sigma=(eskf_cfg.mount_yaw_sigma if eskf_cfg is not None else 0.0)))
    oe.aux["branch10"] = o10.branch_decision
    o1 = run_ekf1d(t, F, W, gnss, C_bp, r_b, road)
    ip = lambda tt, x: np.stack([np.interp(oe.t, tt, xb) for xb in x])
    s = {"1d": ip(o1.t, o1.s), "ekf10": ip(o10.t, o10.s), "eskf": s_e}
    g = {"1d": ip(o1.t, o1.sig_s), "ekf10": ip(o10.t, o10.sig_s), "eskf": g_e}
    v = {"1d": ip(o1.t, o1.v), "ekf10": ip(o10.t, o10.v), "eskf": np.linalg.norm(oe.v, axis=-1)}
    cls = oe.aux.get("imu_class")
    return oe, s, g, cls, v


def supervisor_experiment(key, seeds=50, overrides=(), rules=("class", "rehearsal", "simplest"), sim=None,
                          keep_tracks=False, eskf_cfg=None, map_err=None, mount="estimated", vehicle=None, approach_manoeuvre=True):
    """The three filters, their open-sky rehearsals and the supervisor, scored against the truth.
    Metrics per filter and per selection rule (displayed track: selected filter in the tunnel,
    disagreement-inflated σ, ESKF handover after the exit)."""
    from . import supervisor as SV
    from .mapview import MapView
    seed_ids = np.arange(seeds) if np.isscalar(seeds) else np.asarray(seeds)
    sc, tr, t, F, W, gnss, bt, bh = sim if sim is not None else simulate_run(key, seed_ids, overrides, vehicle=vehicle,
                                                                             approach_manoeuvre=approach_manoeuvre)
    road = get_scenario(key, branch=True).road if sim is None else sc.road
    mv = MapView(road, *(map_err or (0.0, 1.0)))
    F, W, C_bp, mount_diag = filter_mount(tr, t, F, W, gnss, seed_ids, mount)     # v0.5.13
    r_b = tr.lever_arm_b
    if mount_diag is not None:
        from dataclasses import replace as _rp
        from .eskf import EskfConfig
        eskf_cfg = _rp(eskf_cfg or EskfConfig(), mount_yaw_sigma=np.deg2rad(mount_diag["yaw_sigma_deg"]))
    oe, s, g, cls, vv = _three_filters(t, F, W, gnss, bt, bh, C_bp, r_b, road, mv, eskf_cfg=eskf_cfg)
    B = len(seed_ids)
    # estimator side: outage start / GNSS back, from the fixes alone
    ok = np.isfinite(gnss["pos"][:, :, 0]) & (np.nan_to_num(gnss["acc_pos_h"], nan=1e9) <= 10.0)
    gt = gnss["t"]
    t_out = np.empty(B); t_back = np.empty(B)
    for b in range(B):
        ft = gt[ok[b]]
        gap = np.nonzero(np.diff(ft) > 5.0)[0]
        t_out[b] = ft[gap[0]] if len(gap) else ft[-1]
        # GNSS is back at the first fix of any kind after the outage: Doppler velocity arrives before
        # position at the exit, and the smaller filters re-lock on it (a 25 m step on low-end S3)
        anyf = gt[(np.isfinite(gnss["pos"][b, :, 0]) | np.isfinite(gnss["vel"][b, :, 0])) & (gt > t_out[b] + 5.0)]
        t_back[b] = anyf[0] if len(anyf) else np.inf
    # rehearsals (shadow tunnels in open sky, before the real outage)
    win = SV.rehearsal_windows(gt[np.nonzero(ok.all(0))[0][0]], float(t_out.min()))
    scores = {}
    if win and any(r_ != "class" for r_ in rules):
        gm = SV.mask_gnss(gnss, win)
        k_end = int(np.searchsorted(t, t_out.min() + 1.0))
        _, s_sh, g_sh, _, _ = _three_filters(t, F, W, gm, bt, bh, C_bp, r_b, road, mv, k_end=k_end, eskf_cfg=eskf_cfg)
        t_sh = oe.t[: s_sh["eskf"].shape[1]]
        s_fix = np.full(ok.shape, np.nan); sig_fix = np.full(ok.shape, np.nan)
        for j in range(len(gt)):
            if gt[j] > t_out.min():
                break
            okj = ok[:, j]
            if okj.any():
                s_fix[okj, j] = mv.nearest(gnss["pos"][okj, j, :2], np.full(int(okj.sum()), np.nan)) - r_b[0]
                sig_fix[okj, j] = gnss["acc_pos_h"][okj, j]
        for n in SV.FILTERS:
            # the shadow ESKF output grid is a prefix of the full one (same start, same rate)
            K = s_sh[n].shape[1]
            scores[n] = SV.rehearsal_scores(t_sh[:K], s_sh[n], g_sh[n], win, gt, s_fix, sig_fix)
    idx = np.clip(np.round((oe.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    s_true = tr.s[idx]
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_x = tr.t[min(i_x, len(tr.t) - 1)]
    kx = int(np.argmin(np.abs(oe.t - (t_x - 0.05))))
    ex = sc.road.exits[0]
    k_div = int(np.argmax(s_true >= ex.s))
    s_call = ex.s - CALLOUT_LEAD_M
    k_ct = int(np.argmax(s_true >= s_call))
    in_t = (oe.t >= tr.t[int(np.argmax(tr.in_tunnel))]) & (oe.t <= t_x)

    from . import navmetrics as NM

    def metrics(s_d, g_d, dmax=None, v_d=None):
        e = s_d - s_true[None]
        m = (s_d + CALLOUT_K * g_d) >= s_call
        k_hat = np.where(m.any(1), np.argmax(m, 1), m.shape[1] - 1)
        dc = oe.t[k_hat] - oe.t[k_ct]
        w = (oe.t >= t_x) & (oe.t <= t_x + 60.0)
        jumps = np.abs(np.diff(s_d[:, w], axis=1) - np.diff(s_true[w])[None]).max(axis=1)
        r = {"diverge": stats(e[:, k_div])["rmse"], "portal": stats(e[:, kx])["rmse"],
             "portal_p95": stats(e[:, kx])["p95_abs"],
             "nees": round(float(np.mean((e[:, kx] / g_d[:, kx]) ** 2)), 3),
             "callout_mean_s": round(float(dc.mean()), 2), "late_gt_1s": round(float(np.mean(dc > CALLOUT_LATE_S)), 3),
             "early_gt_5s": round(float(np.mean(dc < -5.0)), 3),
             "exit_step_p95": stats(jumps)["p95_abs"]}
        # v0.5.13: recovery time of the shown track and prompt timing (tunnelsim/navmetrics.py)
        rec, worst = NM.recovery(oe.t, e, t_back)
        r["recovery"] = NM.summarise_recovery(rec, worst)
        if v_d is not None:
            r["prompts"] = NM.summarise_prompts(NM.prompts(oe.t, s_d, g_d, s_true, v_d, ex.s, CALLOUT_K))
        if dmax is not None:
            r["alarm_time_frac"] = round(float(np.mean(dmax[:, in_t] > SV.ALARM_D)), 3)
            r["alarm_runs_frac"] = round(float(np.mean((dmax[:, in_t] > SV.ALARM_D).any(axis=1))), 3)
        return r

    res = {"scenario": key, "seeds": [int(seed_ids[0]), int(seed_ids[-1])], "overrides": list(overrides),
           "rehearsal_windows": [list(map(float, w_)) for w_ in win], "filters": {}, "rules": {}}
    if mount_diag is not None:
        res["mount"] = mount_diag
    for n in SV.FILTERS:
        res["filters"][n] = metrics(s[n], g[n], v_d=vv[n])
        if n in scores:
            res["filters"][n]["rehearsal_rms_median"] = round(float(np.median(scores[n][0])), 2)
            res["filters"][n]["rehearsal_nees_median"] = round(float(np.median(scores[n][1])), 2)
    tracks = {}
    for rule in rules:
        sel = SV.select(rule, cls if cls is not None else np.full(B, "note10"), scores or None)
        g_i = np.stack([g[n][b] for b, n in enumerate(sel)])
        r = {"selected": {n: int((sel == n).sum()) for n in SV.FILTERS}}
        for mode in ("none", "any", "majority"):
            s_i, g_disp, dmax = SV.monitor(sel, s, g, "any" if mode == "none" else mode)
            v_i = np.stack([vv[n][b] for b, n in enumerate(sel)])
            s_disp = SV.display(oe.t, s_i, v_i, t_back)
            r[mode] = metrics(s_disp, g_i, dmax, v_i) if mode == "none" else metrics(s_disp, g_disp, dmax, v_i)
            tracks[f"{rule}/{mode}"] = {"sel": sel, "s_disp": s_disp, "sig_disp": g_i if mode == "none" else g_disp,
                                        "dmax": dmax}
        # v0.5.12: the branch decision shown comes from the selected filter (ESKF or 10-state, both
        # have one); the 1D filter has none, so it falls back to the ESKF's
        d_e, d_10 = oe.aux.get("branch_decision"), oe.aux.get("branch10")
        if d_e is not None and d_10 is not None and getattr(mv, "s_div", None) is not None:
            shown = np.where(sel == "ekf10", d_10, d_e)
            r["branch_shown"] = {"branch": round(float(np.mean(shown == 1)), 3), "main": round(float(np.mean(shown == -1)), 3),
                                 "none": round(float(np.mean(shown == 0)), 3),
                                 "eskf_branch": round(float(np.mean(d_e == 1)), 3), "ekf10_branch": round(float(np.mean(d_10 == 1)), 3)}
        res["rules"][rule] = r
    if keep_tracks:
        res["_tracks"] = {"t": oe.t, "s_true": s_true, "s": s, "sig": g, "rules": tracks, "t_back": t_back,
                          "scores": scores, "imu_class": cls, "oe": oe}
    return res
