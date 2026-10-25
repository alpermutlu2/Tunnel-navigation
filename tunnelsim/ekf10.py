"""10-state map-constrained filter with attitude (the 'right-sized' filter between the 4-state 1D EKF
and the 16-state ESKF). Prototype (owner 2026-10-25, v0.5.11); scored by `experiments.ekf10_experiment`.

Estimator side: imports only numpy, frames and eskf (shared stop detector, IMU-class detection and
tuning profiles). The map is an estimator-side road object with `position(s)`, `heading(s)`,
`curvature(s)`, `curvature_rate(s)` and `length`; the map grade is NOT used (DESIGN A3).

State x = [s, v, h, theta, psi, b_a, b_gy, b_gz, b_h, s_g]
    s      arc length of the reference point R along the route [m]
    v      speed along the body x axis (= along the road) [m/s]
    h      height of the IMU [m]
    theta  body pitch, nose up > 0 [rad]
    psi    body yaw, heading from east, counter-clockwise [rad]
    b_a    forward accelerometer bias [m/s^2]
    b_gy   pitch gyro bias [rad/s]        b_gz   yaw gyro bias [rad/s]
    b_h    barometer offset [m]
    s_g    yaw gyro scale error [-] (random constant)
Dynamics (body frame FLU; the mount C_bp is an input, as in the ESKF baseline E2):
    s' = v                              h' = v sin(theta)
    v' = f_x - b_a - g sin(theta)       theta' = -(w_y - b_gy)       psi' = w_z - b_gz
    biases: random walks (Q from the ESKF profile of the detected IMU class); s_g: constant
Measurements:
    GNSS, open sky, reported sigma <= 10 m, chi2(1) 99.9 % gates; ungated re-lock after an outage:
        s   = nearest route point to the fix (within +-300 m of the last s), minus the lever arm
        v   = fix velocity along the map tangent
        h   = fix height
        v sin(theta) = fix vertical velocity                   (pitch and b_gy calibrate before the portal)
    barometer, 1 s mean at the window centre:  h - lag*v*sin(theta) + b_h
    map heading, 1 Hz:  psi - psi_map(s) = 0,  H_s = -kappa(s);  R = sigma_psi^2 + (k_slip*v^2*kappa)^2
        (global along the arc: corrects s in both directions; the ESKF map layer's heading row in 1D)
    curvature-gyro, 1 Hz mean yaw rate:  w = (1+s_g)*v*kappa(s)*cos(theta) + b_gz
        R = gyro_noise^2/1s + sigma_BI^2 + (k_slip*v^3*kappa')^2 + sigma_wander^2
        (speed from the curve; sideslip transient and lane wander in R, scale error as a state)
    ZUPT: v = 0 when the shared stop detector fires and v_hat < zupt_speed_max

What this filter does not model: roll, lateral motion and lane position (the map is a hard
constraint), the lever arm beyond its constant part (b_h, lumped bias), the branch decision (the
route is given; a car going straight on at the diverge needs a second hypothesis), map errors as
states, a step cap at the exit. These are what the 16-state ESKF still buys.

How the design got to 10 states (each step from a Monte Carlo trace on this simulator):
  8 states, curvature-gyro update on single samples -> over-confident ((e/s)^2 4-13):
    1. sideslip transient at the clothoids read as position: k_slip*v^3*kappa' in R      (S1 portal 8.5 -> 3.4 m)
    2. yaw gyro scale error enters the curve speed directly: s_g state                    (9 states)
    3. a real car wanders in its lane: sigma_wander in R                                   (5 km portal 23 -> 7.5 m)
    4. a filter 35 m *ahead* leaves the clothoid before the car, kappa'(s_hat) = 0, the position
       information is lost one-sided: yaw state + global map-heading update              (S4 portal 18.6 -> 5.7 m, 4 seeds)
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .eskf import FILTER_PROFILES, EskfConfig, detect_imu_class, zupt_detector
from .frames import G0

CHI2_1_999 = 10.83
NX = 10
I_S, I_V, I_H, I_T, I_PSI, I_BA, I_BGY, I_BGZ, I_BH, I_SG = range(NX)


@dataclass(frozen=True)
class Ekf10Config:
    """Everything here is [ASSUMED] unless noted; the IMU noise, bias and GNSS/baro R come from EskfConfig."""
    heading_update: bool = True
    heading_sigma_deg: float = 1.0        # map polyline heading (~0.5 deg) + driver in lane (~0.5 deg), as the ESKF map layer
    curvature_update: bool = True
    slip_gain: float = 7e-3               # rad/(m/s^2): sideslip beta = -k*a_lat, k unknown of this size (truth: 0.4 deg/(m/s^2))
    lane_wander_rate_sigma: float = 4e-3  # rad/s: 0.5 m lateral wander with a ~15 s period at 22 m/s
    gyro_bi_note10: float = 2.0e-4        # rad/s: gyro bias instability in the curve-update R (Note10 ~1.6e-4 MEAS)
    gyro_bi_ratio_mpu9250: float = 30.7   # the measured BI ratio of the two profiles
    gyro_scale_sigma: float = 3e-3        # yaw gyro scale error prior (0.3 %, as the truth's [ASSUMED] gyro scale)
    p0_h: float = 10.0                    # m
    map_window_m: float = 300.0
    cov_rate_hz: float = 10.0
    # v0.5.12 (owner 2026-10-25): re-association along the arc during an outage, as the ESKF map layer
    # (v0.5.11) but on the yaw RATE: the 10-state yaw is pulled to ψ_map(ŝ) by the map-heading row
    # every second, so it carries no position, and a gyro-only heading drifts ~90° in 300 s on the
    # low-end IMU (measured, S4). Candidates s_c in s ± 3σ_s, J = (w̄ − (1+s_g)·v·κ(s_c)·cosθ − b_gz)²/σ_w² + u²;
    # if J(s) − min J > χ²₁ 99.9 %, a position pseudo-measurement s = s*, R = max(5 m, σ_w/|v·κ'(s*)|)².
    # The route is given (no branch decision here), so it runs on the whole route.
    map_reassoc: bool = False
    map_reassoc_n: int = 121
    outage_s: float = 2.0                 # no fix for this long = outage (the ESKF's cruise_no_gnss_s)


@dataclass
class Ekf10Output:
    t: np.ndarray        # (K,)
    s: np.ndarray        # (B,K) reference-point arc length
    v: np.ndarray        # (B,K)
    sig_s: np.ndarray    # (B,K)
    theta: np.ndarray    # (B,K)
    psi: np.ndarray      # (B,K)
    b_a: np.ndarray      # (B,K)
    events: dict
    imu_class: np.ndarray


class _RouteGrid:
    """Route sampled on a 1 m grid for the GNSS projection."""

    def __init__(self, road, step: float = 1.0):
        self.s = np.arange(0.0, road.length, step)
        self.xy = road.position(self.s)[:, :2]
        self.step = step

    def nearest(self, p_xy, s_prev, window):
        out = np.empty(len(p_xy))
        for b in range(len(p_xy)):
            if np.isfinite(s_prev[b]):
                a, e = np.searchsorted(self.s, [s_prev[b] - window, s_prev[b] + window + self.step])
                a = min(a, len(self.s) - 1); e = max(e, a + 1)        # estimate past an end of the route
            else:
                a, e = 0, len(self.s)
            d2 = ((self.xy[a:e] - p_xy[b]) ** 2).sum(1)
            out[b] = self.s[a + int(np.argmin(d2))]
        return out


def run_ekf10(imu_t, f, w, gnss, baro_t, baro_h, C_bp, r_b, road_map, cfg10: Ekf10Config = Ekf10Config(),
              cfg: EskfConfig = EskfConfig()) -> Ekf10Output:
    """imu_t (N,); f, w (B,N,3) phone frame; gnss as for run_eskf (t (M,), pos/vel (B,M,3), acc_pos_h,
    acc_pos_v, acc_vel (B,M)); baro_t (L,), baro_h (B,L); C_bp the filter's mount (body <- phone);
    r_b the lever arm of the IMU from R in the body frame; road_map the route as the filter knows it."""
    B, N = f.shape[0], imu_t.size
    fs = 1.0 / np.median(np.diff(imu_t))
    n1 = int(round(fs))
    g_t = gnss["t"]
    k0 = int(np.nonzero(np.all(np.isfinite(gnss["pos"][:, :, 0]), axis=0)
                        & np.all(np.isfinite(gnss["vel"][:, :, 0]), axis=0))[0][0])
    i0 = int(np.searchsorted(imu_t, g_t[k0]))
    # IMU class and tuning profile (shared with the ESKF; one batch = one phone model)
    cls = np.full(B, "note10")
    name = cfg.imu_class
    if cfg.imu_class == "auto":
        cls, _ = detect_imu_class(imu_t, w, cfg, i0)
        vals, cnt = np.unique(cls, return_counts=True)
        name = str(vals[np.argmax(cnt)])
    if name != "note10":
        cfg = replace(cfg, **FILTER_PROFILES[name])
    sig_bi = cfg10.gyro_bi_note10 * (cfg10.gyro_bi_ratio_mpu9250 if name == "mpu9250" else 1.0)
    # body-frame specific force (forward) and rates (pitch, yaw) through the known mount
    fx = np.einsum("j,bnj->bn", C_bp[0], f)
    wy = np.einsum("j,bnj->bn", C_bp[1], w)
    wz = np.einsum("j,bnj->bn", C_bp[2], w)
    grid = _RouteGrid(road_map)
    lever = float(r_b[0])
    # --- initialisation at the first fix with position and velocity
    x = np.zeros((B, NX))
    x[:, I_S] = grid.nearest(gnss["pos"][:, k0, :2], np.full(B, np.nan), 0.0) - lever
    psi0 = road_map.heading(x[:, I_S] + lever)
    x[:, I_V] = gnss["vel"][:, k0, 0] * np.cos(psi0) + gnss["vel"][:, k0, 1] * np.sin(psi0)
    x[:, I_H] = gnss["pos"][:, k0, 2]
    x[:, I_T] = np.arcsin(np.clip(gnss["vel"][:, k0, 2] / np.maximum(x[:, I_V], 1.0), -0.3, 0.3))
    x[:, I_PSI] = psi0
    bwin = (baro_t >= g_t[k0] - 0.5) & (baro_t <= g_t[k0] + 0.5)
    x[:, I_BH] = np.nanmean(baro_h[:, bwin], axis=1) - x[:, I_H]
    P = np.zeros((B, NX, NX))
    for i, s0 in ((I_S, cfg.p0_pos), (I_V, cfg.p0_vel), (I_H, cfg10.p0_h), (I_T, cfg.p0_att_rp), (I_PSI, cfg.p0_att_yaw),
                  (I_BA, cfg.p0_ba), (I_BGY, cfg.p0_bg), (I_BGZ, cfg.p0_bg), (I_BH, cfg.p0_bh), (I_SG, cfg10.gyro_scale_sigma)):
        P[:, i, i] = s0 ** 2
    q = np.zeros(NX)
    q[I_V] = cfg.accel_noise ** 2
    q[I_T] = q[I_PSI] = cfg.gyro_noise ** 2
    q[I_BA] = cfg.accel_bias_rw ** 2
    q[I_BGY] = q[I_BGZ] = cfg.gyro_bias_rw ** 2
    q[I_BH] = cfg.baro_bias_rw ** 2
    # --- schedules
    still = zupt_detector(imu_t, f, cfg, fs)
    n_cov = max(1, int(round(fs / cfg10.cov_rate_hz)))
    g_idx = np.searchsorted(imu_t, g_t)
    gi = int(np.searchsorted(g_idx, i0, side="right"))
    b1_t = np.arange(np.ceil(g_t[k0]), imu_t[-1], 1.0)
    b_idx = np.searchsorted(imu_t, b1_t)
    b_avg = np.stack([np.nanmean(baro_h[:, (baro_t > tt - 1.0) & (baro_t <= tt)], axis=1) for tt in b1_t], 1)
    b_lag = np.array([tt - baro_t[(baro_t > tt - 1.0) & (baro_t <= tt)].mean() for tt in b1_t])
    bi = int(np.searchsorted(b_idx, i0, side="right"))
    last_fix = np.full(B, imu_t[i0]); relock = np.zeros(B, bool); settle = np.zeros(B, int)
    wsum = np.zeros(B); wn = 0
    Fsum = np.zeros((B, NX, NX)); dt_sum = 0.0
    ev = {"gnss_pos_acc": 0, "gnss_pos_rej": 0, "heading": 0, "curv": 0, "zupt": 0, "baro": 0}
    out = {k_: [] for k_ in ("t", "s", "v", "sig_s", "theta", "psi", "b_a")}

    def update(sel, z, H, R, gate):
        """Scalar update on the batch members in sel; z = measurement - h(x)."""
        Hs, Ps = H[sel], P[sel]
        S = np.einsum("bi,bij,bj->b", Hs, Ps, Hs) + R[sel]
        ok = (z[sel] ** 2 / S < gate) if gate is not None else np.ones(int(sel.sum()), bool)
        K = np.einsum("bij,bj->bi", Ps, Hs) / S[:, None]
        K[~ok] = 0.0
        idx = np.nonzero(sel)[0]
        x[idx] += K * z[sel][:, None]
        IKH = np.eye(NX) - K[:, :, None] * Hs[:, None, :]
        P[idx] = IKH @ Ps @ np.swapaxes(IKH, 1, 2) + R[sel][:, None, None] * K[:, :, None] * K[:, None, :]
        return ok

    for k in range(i0, N - 1):
        dt = imu_t[k + 1] - imu_t[k]
        th, v = x[:, I_T], x[:, I_V]
        a = 0.5 * (fx[:, k] + fx[:, k + 1]) - x[:, I_BA] - G0 * np.sin(th)
        th_rate = -(0.5 * (wy[:, k] + wy[:, k + 1]) - x[:, I_BGY])
        psi_rate = 0.5 * (wz[:, k] + wz[:, k + 1]) - x[:, I_BGZ]
        x[:, I_S] += v * dt + 0.5 * a * dt * dt
        x[:, I_H] += v * np.sin(th) * dt
        x[:, I_V] += a * dt
        x[:, I_T] += th_rate * dt
        x[:, I_PSI] += psi_rate * dt
        F = np.zeros((B, NX, NX))
        F[:, I_S, I_V] = 1.0
        F[:, I_V, I_T] = -G0 * np.cos(th); F[:, I_V, I_BA] = -1.0
        F[:, I_H, I_V] = np.sin(th); F[:, I_H, I_T] = v * np.cos(th)
        F[:, I_T, I_BGY] = 1.0
        F[:, I_PSI, I_BGZ] = -1.0
        Fsum += F * dt; dt_sum += dt
        wsum += 0.5 * (wz[:, k] + wz[:, k + 1]); wn += 1
        kk = k + 1
        gnss_due = gi < len(g_idx) and g_idx[gi] == kk
        baro_due = bi < len(b_idx) and b_idx[bi] == kk
        map_due = (kk - i0) % n1 == 0
        do_cov = (kk - i0) % n_cov == 0
        st = still[:, kk] & (x[:, I_V] < cfg.zupt_speed_max)
        if not (do_cov or gnss_due or baro_due or map_due or st.any()):
            continue
        if dt_sum > 0:
            Phi = np.eye(NX) + Fsum + 0.5 * Fsum @ Fsum
            P[:] = Phi @ P @ np.swapaxes(Phi, 1, 2) + np.diag(q) * dt_sum
            Fsum[:] = 0.0; dt_sum = 0.0
        # --- GNSS
        while gi < len(g_idx) and g_idx[gi] == kk:
            pz, vz, tf = gnss["pos"][:, gi], gnss["vel"][:, gi], g_t[gi]
            sh = np.nan_to_num(gnss["acc_pos_h"][:, gi], nan=1e9)
            sv = np.nan_to_num(gnss["acc_pos_v"][:, gi], nan=1e9)
            okp = np.isfinite(pz[:, 0]) & (sh <= cfg.gnss_max_reported_sigma_m)
            okv = np.isfinite(vz[:, 0])
            new_phase = (okp | okv) & (tf - last_fix > cfg.reacq_gap_s)
            relock |= new_phase
            settle = np.where(new_phase, 0, settle)
            last_fix = np.where(okp | okv, tf, last_fix)
            if okp.any():
                sm = np.full(B, np.nan)
                sm[okp] = grid.nearest(pz[okp, :2], x[okp, I_S] + lever, cfg10.map_window_m) - lever
                H = np.zeros((B, NX)); H[:, I_S] = 1.0
                R = (cfg.gnss_pos_factor * sh) ** 2
                z = np.where(okp, np.nan_to_num(sm) - x[:, I_S], 0.0)
                passes = z ** 2 / (P[:, I_S, I_S] + R) < CHI2_1_999
                ung = okp & relock                     # re-lock after an outage: ungated until 5 fixes in a row pass
                if ung.any():
                    update(ung, z, H, R, None)
                    settle = np.where(ung, np.where(passes, settle + 1, 0), settle)
                    relock &= ~(ung & (settle >= cfg.reacq_settle_fixes))
                    ev["gnss_pos_acc"] += int(ung.sum())
                gat = okp & ~ung
                if gat.any():
                    ok = update(gat, z, H, R, CHI2_1_999)
                    ev["gnss_pos_acc"] += int(ok.sum()); ev["gnss_pos_rej"] += int((~ok).sum())
                Hh = np.zeros((B, NX)); Hh[:, I_H] = 1.0
                update(okp, np.where(okp, pz[:, 2] - x[:, I_H], 0.0), Hh, (cfg.gnss_pos_factor * sv) ** 2,
                       None if relock.any() else CHI2_1_999)
            if okv.any():
                psi_m = road_map.heading(x[:, I_S] + lever)
                vm = vz[:, 0] * np.cos(psi_m) + vz[:, 1] * np.sin(psi_m)
                sve = cfg.gnss_vel_factor * np.nan_to_num(gnss["acc_vel"][:, gi], nan=1e3)
                H = np.zeros((B, NX)); H[:, I_V] = 1.0
                update(okv, np.where(okv, np.nan_to_num(vm) - x[:, I_V], 0.0), H, sve ** 2, CHI2_1_999)
                th, v = x[:, I_T], x[:, I_V]
                Hz = np.zeros((B, NX)); Hz[:, I_V] = np.sin(th); Hz[:, I_T] = v * np.cos(th)
                update(okv, np.where(okv, np.nan_to_num(vz[:, 2]) - v * np.sin(th), 0.0), Hz, (1.5 * sve) ** 2, CHI2_1_999)
            gi += 1
        # --- barometer (1 s mean, window centre)
        while bi < len(b_idx) and b_idx[bi] == kk:
            hz, lag = b_avg[:, bi], b_lag[bi]
            okb = np.isfinite(hz)
            th, v = x[:, I_T], x[:, I_V]
            H = np.zeros((B, NX))
            H[:, I_H] = 1.0; H[:, I_V] = -lag * np.sin(th); H[:, I_T] = -lag * v * np.cos(th); H[:, I_BH] = 1.0
            z = np.where(okb, np.nan_to_num(hz) - (x[:, I_H] - lag * v * np.sin(th) + x[:, I_BH]), 0.0)
            update(okb, z, H, np.full(B, cfg.baro_sigma ** 2), None)
            ev["baro"] += int(okb.sum()); bi += 1
        # --- re-association along the arc (outage only)
        if map_due and cfg10.map_reassoc and wn > 0:
            out_ = imu_t[kk] - last_fix > cfg10.outage_s
            ra = out_ & (x[:, I_V] > 1.0)
            if ra.any():
                sg_s = np.sqrt(P[:, I_S, I_S])
                u_ = np.linspace(-3.0, 3.0, cfg10.map_reassoc_n)
                th, v = x[:, I_T], x[:, I_V]
                cand = np.clip(x[:, I_S, None] + lever + sg_s[:, None] * u_[None], 0.0, road_map.length - 1.0)
                cc = cand - 0.5 * v[:, None] * (wn / fs)                          # window centre
                kc = road_map.curvature(cc.ravel()).reshape(cc.shape)
                krc = road_map.curvature_rate(cc.ravel()).reshape(cc.shape)
                sg = 1.0 + x[:, I_SG]; cth = np.cos(th)
                wm = wsum / wn
                r_ = wm[:, None] - (sg[:, None] * v[:, None] * kc * cth[:, None] + x[:, I_BGZ, None])
                var_w = (cfg.gyro_noise ** 2 / (wn / fs) + sig_bi ** 2 + cfg10.lane_wander_rate_sigma ** 2
                         + (cfg10.slip_gain * np.abs(v[:, None]) ** 3 * np.abs(krc)) ** 2
                         + P[:, I_BGZ, I_BGZ, None] + (sg[:, None] * kc) ** 2 * P[:, I_V, I_V, None]
                         + (v[:, None] * kc) ** 2 * P[:, I_SG, I_SG, None])
                J = r_ ** 2 / var_w + u_[None] ** 2
                jb = np.argmin(J, axis=1); ar = np.arange(B)
                go = ra & (J[:, cfg10.map_reassoc_n // 2] - J[ar, jb] > CHI2_1_999)
                if go.any():
                    s_star = cand[ar, jb] - lever
                    # along-track precision of the match: the rate's σ over its slope v·κ' there
                    sig_a = np.maximum(5.0, np.sqrt(var_w[ar, jb]) / np.maximum(np.abs(v * krc[ar, jb]), 1e-9))
                    H = np.zeros((B, NX)); H[:, I_S] = 1.0
                    update(go, np.where(go, s_star - x[:, I_S], 0.0), H, sig_a ** 2, None)
                    ev["reassoc"] = ev.get("reassoc", 0) + int(go.sum())
        # --- map heading (global along the route)
        if map_due and cfg10.heading_update:
            sp = x[:, I_S] + lever
            psi_m, kap = road_map.heading(sp), road_map.curvature(sp)
            v = x[:, I_V]
            z = np.angle(np.exp(1j * (psi_m - x[:, I_PSI])))          # h = psi - psi_map(s); z = 0 - h
            H = np.zeros((B, NX)); H[:, I_S] = -kap; H[:, I_PSI] = 1.0
            R = np.deg2rad(cfg10.heading_sigma_deg) ** 2 + (cfg10.slip_gain * v * v * kap) ** 2   # sideslip beta = -k a_lat
            update(np.ones(B, bool), z, H, R, CHI2_1_999)
            ev["heading"] += B
        # --- curvature-gyro (speed from the curve), 1 s mean rate at the window centre
        if map_due and wn > 0 and cfg10.curvature_update:
            wm = wsum / wn
            th, v = x[:, I_T], x[:, I_V]
            sc = x[:, I_S] + lever - 0.5 * v * (wn / fs)
            kap, kap_r, cth = road_map.curvature(sc), road_map.curvature_rate(sc), np.cos(th)
            sg = 1.0 + x[:, I_SG]
            H = np.zeros((B, NX))
            H[:, I_S] = sg * v * kap_r * cth; H[:, I_V] = sg * kap * cth; H[:, I_T] = -sg * v * kap * np.sin(th)
            H[:, I_BGZ] = 1.0; H[:, I_SG] = v * kap * cth
            z = wm - (sg * v * kap * cth + x[:, I_BGZ])
            R = (cfg.gyro_noise ** 2 / (wn / fs) + sig_bi ** 2
                 + (cfg10.slip_gain * np.abs(v) ** 3 * np.abs(kap_r)) ** 2      # sideslip rate where kappa changes
                 + cfg10.lane_wander_rate_sigma ** 2)                           # lane keeping
            update(np.ones(B, bool), z, H, np.broadcast_to(R, (B,)).copy() if np.ndim(R) == 0 else R, CHI2_1_999)
            ev["curv"] += B
            wsum[:] = 0.0; wn = 0
        # --- ZUPT
        if st.any():
            H = np.zeros((B, NX)); H[:, I_V] = 1.0
            update(st, -x[:, I_V], H, np.full(B, cfg.zupt_sigma ** 2), None)
            ev["zupt"] += int(st.sum())
        if do_cov:
            out["t"].append(imu_t[kk]); out["s"].append(x[:, I_S].copy()); out["v"].append(x[:, I_V].copy())
            out["sig_s"].append(np.sqrt(P[:, I_S, I_S])); out["theta"].append(x[:, I_T].copy())
            out["psi"].append(x[:, I_PSI].copy()); out["b_a"].append(x[:, I_BA].copy())
    st_ = lambda L: np.stack(L, 1)
    return Ekf10Output(np.array(out["t"]), st_(out["s"]), st_(out["v"]), st_(out["sig_s"]), st_(out["theta"]),
                       st_(out["psi"]), st_(out["b_a"]), ev, cls)
