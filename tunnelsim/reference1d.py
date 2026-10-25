"""1D map-constrained EKF: a simple baseline for the ESKF (owner 2026-10-24, v0.5.9).

A common design for in-tunnel navigation, re-implemented here from its description and adapted
to this simulator's sensors (two changes, below). It is a reference, not a candidate: it shows
what the full ESKF buys and what a simpler model does not see.

Estimator side: imports only numpy, frames and eskf (for the shared ZUPT detector, IMU-class
detection and tuning profiles). The map is an estimator-side road object with `position(s)`,
`heading(s)`, `curvature(s)`, `curvature_rate(s)` and optionally `grade(s)`, `grade_rate(s)`.

State x = [s, v, b_a, b_g]: arc length of the reference point along the route [m], speed along the
road [m/s], forward accelerometer bias [m/s²], body yaw-rate bias [rad/s].

    ṡ = v
    v̇ = f_x − b_a − g·sin γ(s)            f_x: forward specific force (phone IMU through the mount)
    ḃ_a, ḃ_g random walk                  γ(s): map grade (0 when the map has no grade)

Measurements:
    GNSS position, projected on the route: s_meas = nearest map point within ±300 m, minus the
        lever arm's forward component; R = (gnss_pos_factor · reported σ_h)²; NIS gate χ²₁ 99.9 %,
        except in the re-lock phase after an outage (ungated until 5 fixes in a row pass the gate)
    GNSS velocity along the map tangent: R = (gnss_vel_factor · reported σ_v)²
    curvature-gyro, 1 Hz: mean body yaw rate over 1 s = v·κ(s)·cos γ + b_g,
        R = gyro_noise² / 1 s + σ_BI²
    ZUPT: v = 0 when the shared stop detector fires and v̂ < 2 m/s

Adaptations to this simulator (both against physics the original model does not have):
  1. the curvature-gyro update runs at 1 Hz on the 1 s mean rate with R including the gyro bias
     instability, not at 100 Hz on every sample with white R: per sample, vibration and the
     correlated bias error made the filter over-confident ((e/σ)² ≈ 23); the same lesson as the
     ESKF's GNSS R inflation;
  2. κ(s) is the map's analytic curvature: a numerical gradient of the heading produced spurious
     speed information on straight roads.
Every aid the ESKF gets (pre-tunnel GNSS, the same ZUPT detector, R inflation, P0, IMU class
profile) is given to this filter too.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .eskf import FILTER_PROFILES, EskfConfig, detect_imu_class, zupt_detector
from .frames import G0

CHI2_1_999 = 10.83


@dataclass(frozen=True)
class Ekf1dConfig:
    use_grade: bool = True            # map carries the grade γ(s); False: γ = 0 (map without grade)
    grade_scale: float = 1.0          # map grade = scale × road grade (1.1: the map's grade 10 % too steep) [stress]
    curv_rate_hz: float = 1.0         # curvature-gyro update rate (adaptation 1)
    gyro_bi_note10: float = 2.0e-4    # rad/s, σ_BI in the curvature-gyro R [ASSUMED, ~Note10 gyro BI 1.6e-4 MEAS]
    gyro_bi_ratio: dict = None        # per class multiplier of gyro_bi (default: mpu9250 × 30.7, the BI ratio)
    reacq_gap_s: float = 10.0         # an outage longer than this starts a re-lock phase: fixes not gated ...
    reacq_settle_fixes: int = 5       # ... until this many in a row pass the gate (as the ESKF's settle count)
    map_window_m: float = 300.0
    cov_rate_hz: float = 10.0


@dataclass
class Ekf1dOutput:
    t: np.ndarray        # (K,)
    s: np.ndarray        # (B,K) reference-point arc length
    v: np.ndarray        # (B,K)
    sig_s: np.ndarray    # (B,K)
    b_a: np.ndarray      # (B,K)
    b_g: np.ndarray      # (B,K)
    events: dict
    imu_class: np.ndarray


class _MapGrid:
    """Route sampled on a 1 m grid for the GNSS projection."""

    def __init__(self, road, step=1.0):
        self.road = road
        self.s = np.arange(0.0, road.length, step)
        self.xy = road.position(self.s)[:, :2]
        self.step = step

    def nearest(self, p_xy, s_prev, window):
        out = np.empty(len(p_xy))
        for b in range(len(p_xy)):
            if np.isfinite(s_prev[b]):
                a, e = np.searchsorted(self.s, [s_prev[b] - window, s_prev[b] + window + self.step])
            else:
                a, e = 0, len(self.s)
            d2 = ((self.xy[a:e] - p_xy[b]) ** 2).sum(1)
            out[b] = self.s[a + int(np.argmin(d2))]
        return out


def run_ekf1d(imu_t, f, w, gnss, C_bp, r_b, road_map, cfg1d: Ekf1dConfig = Ekf1dConfig(),
              cfg: EskfConfig = EskfConfig(), zupt_mask=None):
    """imu_t (N,), f/w (B,N,3) phone frame; gnss as for run_eskf; C_bp the filter's mount, r_b the
    lever arm (body); road_map the route as the filter knows it."""
    B, N = f.shape[0], imu_t.size
    fs = 1.0 / np.median(np.diff(imu_t))
    g_t = gnss["t"]
    k0 = int(np.nonzero(np.all(np.isfinite(gnss["pos"][:, :, 0]), axis=0)
                        & np.all(np.isfinite(gnss["vel"][:, :, 0]), axis=0))[0][0])
    i0 = int(np.searchsorted(imu_t, g_t[k0]))
    cls = np.full(B, "note10")
    if cfg.imu_class == "auto":
        cls, _ = detect_imu_class(imu_t, w, cfg, i0)
        vals, cnt = np.unique(cls, return_counts=True)
        cfg = replace(cfg, **FILTER_PROFILES[str(vals[np.argmax(cnt)])])
        name = str(vals[np.argmax(cnt)])
    else:
        name = cfg.imu_class
        if name != "note10":
            cfg = replace(cfg, **FILTER_PROFILES[name])
    ratio = cfg1d.gyro_bi_ratio or {"note10": 1.0, "mpu9250": 30.7}
    sig_bi = cfg1d.gyro_bi_note10 * ratio.get(name, 1.0)
    # body-frame forward specific force and yaw rate
    fx = np.einsum("j,bnj->bn", C_bp[0], f)
    wz = np.einsum("j,bnj->bn", C_bp[2], w)
    grid = _MapGrid(road_map)
    have_grade = cfg1d.use_grade and hasattr(road_map, "grade")

    def grade(s):
        if not have_grade:
            return np.zeros_like(s), np.zeros_like(s)
        return cfg1d.grade_scale * road_map.grade(s), cfg1d.grade_scale * road_map.grade_rate(s)

    # init at the first fix with position and velocity
    s = grid.nearest(gnss["pos"][:, k0, :2], np.full(B, np.nan), 0.0) - r_b[0]
    t_hat = np.stack([np.cos(road_map.heading(s)), np.sin(road_map.heading(s))], 1)
    v = np.einsum("bi,bi->b", gnss["vel"][:, k0, :2], t_hat)
    b_a = np.zeros(B); b_g = np.zeros(B)
    P = np.zeros((B, 4, 4))
    P[:, 0, 0] = cfg.p0_pos ** 2; P[:, 1, 1] = cfg.p0_vel ** 2
    P[:, 2, 2] = cfg.p0_ba ** 2; P[:, 3, 3] = cfg.p0_bg ** 2
    q = np.array([0.0, cfg.accel_noise ** 2, cfg.accel_bias_rw ** 2, cfg.gyro_bias_rw ** 2])
    still = zupt_detector(imu_t, f, cfg, fs) if zupt_mask is None else np.broadcast_to(zupt_mask, (B, N))
    n_cov = max(1, int(round(fs / cfg1d.cov_rate_hz)))
    n_curv = max(1, int(round(fs / cfg1d.curv_rate_hz)))
    g_idx = np.searchsorted(imu_t, g_t)
    gi = int(np.searchsorted(g_idx, i0, side="right"))
    last_fix = np.full(B, imu_t[i0]); relock_on = np.zeros(B, bool); settle = np.zeros(B, int)
    wsum = np.zeros(B); wn = 0
    Fsum = np.zeros((B, 4, 4)); dt_sum = 0.0
    ev = {"gnss_pos_acc": 0, "gnss_pos_rej": 0, "gnss_vel_acc": 0, "curv_updates": 0, "zupt": 0}
    out_t, out_s, out_v, out_sig, out_ba, out_bg = [], [], [], [], [], []

    def update(sel, z, H, R, gate):
        """Scalar update on the members in sel; z = h(x̂) − measurement."""
        nonlocal s, v, b_a, b_g
        Hs = H[sel]; Ps = P[sel]
        S = np.einsum("bi,bij,bj->b", Hs, Ps, Hs) + R[sel]
        nis = z[sel] ** 2 / S
        ok = nis < gate if gate is not None else np.ones(len(S), bool)
        K = np.einsum("bij,bj->bi", Ps, Hs) / S[:, None]
        K[~ok] = 0.0
        dx = -K * z[sel][:, None]
        idx = np.nonzero(sel)[0]
        s[idx] += dx[:, 0]; v[idx] += dx[:, 1]; b_a[idx] += dx[:, 2]; b_g[idx] += dx[:, 3]
        IKH = np.eye(4) - K[:, :, None] * Hs[:, None, :]
        P[idx] = IKH @ Ps @ np.swapaxes(IKH, 1, 2) + R[sel][:, None, None] * K[:, :, None] * K[:, None, :]
        return ok

    for k in range(i0, N - 1):
        dt = imu_t[k + 1] - imu_t[k]
        gam, gam_r = grade(s)
        a = 0.5 * (fx[:, k] + fx[:, k + 1]) - b_a - G0 * np.sin(gam)
        s = s + v * dt + 0.5 * a * dt * dt
        v = v + a * dt
        F = np.zeros((B, 4, 4))
        F[:, 0, 1] = 1.0
        F[:, 1, 0] = -G0 * np.cos(gam) * gam_r
        F[:, 1, 2] = -1.0
        Fsum += F * dt; dt_sum += dt
        wsum += 0.5 * (wz[:, k] + wz[:, k + 1]); wn += 1
        kk = k + 1
        gnss_due = gi < len(g_idx) and g_idx[gi] == kk
        curv_due = (kk - i0) % n_curv == 0
        do_cov = (kk - i0) % n_cov == 0
        st = still[:, kk] & (v < cfg.zupt_speed_max)
        if not (do_cov or gnss_due or curv_due or st.any()):
            continue
        if dt_sum > 0:
            Phi = np.eye(4) + Fsum
            P = Phi @ P @ np.swapaxes(Phi, 1, 2) + np.diag(q) * dt_sum
            Fsum[:] = 0.0; dt_sum = 0.0
        while gi < len(g_idx) and g_idx[gi] == kk:
            pz, vz = gnss["pos"][:, gi], gnss["vel"][:, gi]
            sh = np.nan_to_num(gnss["acc_pos_h"][:, gi], nan=1e9)
            okp = np.isfinite(pz[:, 0]) & (sh <= cfg.gnss_max_reported_sigma_m)
            tf = g_t[gi]
            new_phase = (okp | np.isfinite(vz[:, 0])) & (tf - last_fix > cfg1d.reacq_gap_s)
            relock_on = relock_on | new_phase
            settle = np.where(new_phase, 0, settle)
            last_fix = np.where(okp | np.isfinite(vz[:, 0]), tf, last_fix)
            if okp.any():
                sm = np.full(B, np.nan)
                sm[okp] = grid.nearest(pz[okp, :2], s[okp] + r_b[0], cfg1d.map_window_m) - r_b[0]
                H = np.zeros((B, 4)); H[:, 0] = 1.0
                R = (cfg.gnss_pos_factor * sh) ** 2
                z = np.where(okp, s - np.nan_to_num(sm), 0.0)
                S_ = P[:, 0, 0] + R
                passes = z ** 2 / S_ < CHI2_1_999
                ung = okp & relock_on
                if ung.any():
                    update(ung, z, H, R, None); ev["gnss_pos_acc"] += int(ung.sum())
                    settle = np.where(ung, np.where(passes, settle + 1, 0), settle)
                    relock_on = relock_on & ~(ung & (settle >= cfg1d.reacq_settle_fixes))
                gat = okp & ~ung
                if gat.any():
                    ok = update(gat, z, H, R, CHI2_1_999)
                    ev["gnss_pos_acc"] += int(ok.sum()); ev["gnss_pos_rej"] += int((~ok).sum())
            okv = np.isfinite(vz[:, 0])
            if okv.any():
                psi = road_map.heading(s)
                vm = vz[:, 0] * np.cos(psi) + vz[:, 1] * np.sin(psi)
                H = np.zeros((B, 4)); H[:, 1] = 1.0
                R = (cfg.gnss_vel_factor * np.nan_to_num(gnss["acc_vel"][:, gi], nan=1e3)) ** 2
                z = np.where(okv, v - np.nan_to_num(vm), 0.0)
                ok = update(okv, z, H, R, CHI2_1_999)
                ev["gnss_vel_acc"] += int(ok.sum())
            gi += 1
        if curv_due and wn > 0:
            wm = wsum / wn
            # the window's mean rate belongs to its centre, half a window behind
            sc = s - 0.5 * v * (wn / fs)
            kap = road_map.curvature(sc); kap_r = road_map.curvature_rate(sc)
            cg = np.cos(grade(sc)[0])
            h = v * kap * cg + b_g
            H = np.zeros((B, 4)); H[:, 0] = v * kap_r * cg; H[:, 1] = kap * cg; H[:, 3] = 1.0
            R = np.full(B, cfg.gyro_noise ** 2 / (wn / fs) + sig_bi ** 2)
            update(np.ones(B, bool), h - wm, H, R, None)
            ev["curv_updates"] += B
            wsum[:] = 0.0; wn = 0
        if st.any():
            H = np.zeros((B, 4)); H[:, 1] = 1.0
            update(st, v.copy(), H, np.full(B, cfg.zupt_sigma ** 2), None)
            ev["zupt"] += int(st.sum())
        if do_cov:
            out_t.append(imu_t[kk]); out_s.append(s.copy()); out_v.append(v.copy())
            out_sig.append(np.sqrt(P[:, 0, 0])); out_ba.append(b_a.copy()); out_bg.append(b_g.copy())
    st_ = lambda L: np.stack(L, 1)
    return Ekf1dOutput(np.array(out_t), st_(out_s), st_(out_v), st_(out_sig), st_(out_ba), st_(out_bg), ev, cls)
