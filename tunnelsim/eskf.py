"""Error-state Kalman filter (v0.5; design E1-E6 approved 2026-10-20).

Estimator side: imports only numpy/scipy, frames, meas, ins (docs/contracts.md 6). Batched over a
leading dimension B (Monte Carlo seeds); every seed is an independent filter.

States (16, nav ENU, phone point P):
    0:3 δp   3:6 δv   6:9 δθ (nav-frame, Ĉ_np = Exp([δθ]×) C_np)
    9:12 δb_a (phone frame)   12:15 δb_g (phone frame)   15 δb_h (barometer offset)
Error dynamics (error = estimate − truth):
    δṗ = δv,  δv̇ = −[f_n]× δθ − C δb_a,  δθ̇ = −C δb_g,  biases random walk.
Only the bias states get deliberately small process noise (robustness over optimality, DESIGN 5.0).

Measurements:
    GNSS position / velocity (R inflated for correlated errors, NIS gate),
    NHC: body lateral and vertical velocity at R = 0,
    barometer altitude (1 Hz average) with the offset state,
    ZUPT: v = 0 when a stop is detected (detector or a supplied mask).
Mount C_bp and lever arm r_b are inputs (assumed known in the baseline, E2).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import butter, sosfiltfilt

from .frames import GRAVITY_N
from .ins import so3_exp_batch

NX = 16
CHI2_3_999 = 16.27
CHI2_2_999 = 13.82


@dataclass(frozen=True)
class EskfConfig:
    """Filter tuning. Every number is a design choice [ASSUMED] unless noted."""
    accel_noise: float = 0.02         # m/s/√s, effective (VRW 1e-3 + vibration)
    gyro_noise: float = 1.0e-3        # rad/√s, effective (ARW 7e-4 + vibration)
    accel_bias_rw: float = 5e-4 / np.sqrt(180.0)   # m/s²/√s: 5e-4 m/s² allowed over 180 s (E1)
    gyro_bias_rw: float = 1e-4 / np.sqrt(180.0)    # rad/s/√s: 1e-4 rad/s over 180 s (E1)
    baro_bias_rw: float = 0.026       # m/√s: GM(0.1 m, 60 s) as a random walk, √(2σ²/τ)
    gnss_pos_factor: float = 2.0      # R_pos = (factor · reported σ)²  (E3)
    gnss_vel_factor: float = 3.2      # R_vel = (factor · reported σ)²: 0.10 -> 0.32 m/s eq. white (E3)
    nhc_sigma: float = 0.1            # m/s (E4)
    nhc_rate_hz: float = 5.0
    baro_sigma: float = 0.15          # m, 1 Hz average (E4)
    zupt_sigma: float = 0.02          # m/s (E5)
    zupt_band_hz: tuple = (2.0, 18.0)  # road-vibration band, below the engine tone (E5)
    zupt_band_rms: float = 0.01       # m/s² threshold on the 1 s band RMS of |f|
    zupt_hold_s: float = 0.3          # s: debounce
    zupt_lowf_std: float = 0.05       # m/s²: 1 s std of the < 2 Hz specific force vector (a launch
                                      # or a brake moves it by ~1 m/s²; a stopped car does not)
    zupt_speed_max: float = 2.0       # m/s: filter speed must be below this
    cov_rate_hz: float = 10.0         # covariance propagation rate
    p0_pos: float = 5.0
    p0_vel: float = 0.3
    p0_att_rp: float = np.deg2rad(1.0)
    p0_att_yaw: float = np.deg2rad(5.0)
    p0_ba: float = 0.15               # turn-on 10 mg ≈ 0.1 m/s² plus scale/misalignment through g
    p0_bg: float = np.deg2rad(0.1)
    p0_bh: float = 10.0


@dataclass
class EskfOutput:
    t: np.ndarray              # (K,) output epochs (cov rate)
    p: np.ndarray              # (B,K,3) phone position
    v: np.ndarray              # (B,K,3)
    C: np.ndarray              # (B,K,3,3)
    P_pos: np.ndarray          # (B,K,3,3)
    b_a: np.ndarray            # (B,K,3)
    zupt: np.ndarray           # (B,K) bool, ZUPT applied
    events: dict = field(default_factory=dict)   # counts of accepted / rejected updates


def skew_b(v):
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    o = np.zeros_like(x)
    return np.stack([np.stack([o, -z, y], -1), np.stack([z, o, -x], -1), np.stack([-y, x, o], -1)], -2)


def triad(f_p, fwd_p, v_n):
    """Initial phone attitude: phone-frame up (mean specific force) and forward (mount) matched
    to nav up and the GNSS course."""
    def basis(a, b):
        e1 = a / np.linalg.norm(a)
        e2 = np.cross(a, b); e2 /= np.linalg.norm(e2)
        return np.column_stack([e1, e2, np.cross(e1, e2)])
    Bp = basis(f_p, fwd_p)
    vh = np.array([v_n[0], v_n[1], 0.0])
    Bn = basis(np.array([0.0, 0.0, 1.0]), vh)
    return Bn @ Bp.T


def zupt_detector(t, f, cfg, fs, win_s=1.0):
    """Stop detector, (B,N) bool. Both must hold over the trailing 1 s window:
      1. road-band (2-18 Hz) RMS of |f| below a threshold: no road vibration (the engine tone at
         ~25 Hz is outside the band), and
      2. the low-frequency (< 2 Hz) specific-force vector is steady: no launch or braking.
    Condition 1 alone cannot tell a stopped car from one creeping below ~2 m/s, because road
    vibration scales with speed (found on S4: a launch after a stop was taken as standing).
    Relies on the ASSUMED vibration model (DESIGN 5.2 honesty note)."""
    n = int(win_s * fs)

    def mov(x):                                    # trailing moving mean over n samples
        c = np.cumsum(np.concatenate([np.zeros(x.shape[:-1] + (1,)), x], axis=-1), axis=-1)
        m = (c[..., n:] - c[..., :-n]) / n
        return np.concatenate([np.repeat(m[..., :1], n - 1, axis=-1), m], axis=-1)   # length N

    sos = butter(4, cfg.zupt_band_hz, btype="bandpass", fs=fs, output="sos")
    xb = sosfiltfilt(sos, np.linalg.norm(f, axis=-1), axis=-1)
    band_rms = np.sqrt(np.maximum(mov(xb * xb), 0.0))
    lp = butter(2, 2.0, btype="lowpass", fs=fs, output="sos")
    fl = sosfiltfilt(lp, f, axis=-2)                                  # (B,N,3)
    var = sum(np.maximum(mov(fl[..., i] ** 2) - mov(fl[..., i]) ** 2, 0.0) for i in range(3))
    raw = (band_rms < cfg.zupt_band_rms) & (np.sqrt(var) < cfg.zupt_lowf_std)
    # debounce: both conditions held without a break for the last zupt_hold_s
    n = max(1, int(cfg.zupt_hold_s * fs))
    return mov(raw.astype(float)) > 1.0 - 1e-9


def _update(x_corr, P, z, H, R, gate):
    """Batched update. z = h(x̂) − measurement − H·x_corr (innovation in the estimate − truth
    convention, net of corrections already accumulated this epoch); the filter then estimates the
    error δ and the caller injects x̂ ← x̂ ⊖ δ. Returns the accepted mask and the NIS."""
    S = H @ P @ np.swapaxes(H, 1, 2) + R
    Sinv = np.linalg.inv(S)
    nis = np.einsum("bi,bij,bj->b", z, Sinv, z)
    ok = nis < gate if gate is not None else np.ones(len(z), bool)
    K = P @ np.swapaxes(H, 1, 2) @ Sinv
    K[~ok] = 0.0
    x_corr += np.einsum("bij,bj->bi", K, z)
    IKH = np.eye(NX) - K @ H
    P[:] = IKH @ P @ np.swapaxes(IKH, 1, 2) + K @ R @ np.swapaxes(K, 1, 2)
    return ok, nis


def run_eskf(imu_t, f, w, gnss, baro_t, baro_h, C_bp, r_b, cfg: EskfConfig = EskfConfig(),
             zupt_mask=None, t_start=None):
    """Run the filter.

    imu_t (N,) shared time axis; f, w (B,N,3); gnss: dict of (B,M,...) arrays t (M,), pos, vel,
    acc_pos_h, acc_pos_v, acc_vel (NaN where missing); baro_t (L,), baro_h (B,L).
    zupt_mask: optional (N,) or (B,N) bool overriding the detector (oracle stops).
    """
    B, N = f.shape[0], imu_t.size
    fs = 1.0 / np.median(np.diff(imu_t))
    C_pb = C_bp.T
    fwd_p = C_pb[:, 0]
    g_t = gnss["t"]
    # --- initialisation at the first GNSS fix with position and velocity
    k0 = int(np.nonzero(np.all(np.isfinite(gnss["pos"][:, :, 0]), axis=0)
                        & np.all(np.isfinite(gnss["vel"][:, :, 0]), axis=0))[0][0])
    t0 = g_t[k0] if t_start is None else t_start
    i0 = int(np.searchsorted(imu_t, t0))
    p = gnss["pos"][:, k0].copy()
    v = gnss["vel"][:, k0].copy()
    f_mean = f[:, max(0, i0 - int(fs)):i0 + 1].mean(1)
    C = np.stack([triad(f_mean[b], fwd_p, v[b]) for b in range(B)])
    b_a = np.zeros((B, 3)); b_g = np.zeros((B, 3))
    b_h = np.nanmean(baro_h[:, (baro_t >= t0 - 0.5) & (baro_t <= t0 + 0.5)], axis=1) - p[:, 2]
    P = np.zeros((B, NX, NX))
    d = np.r_[[cfg.p0_pos] * 3, [cfg.p0_vel] * 3, cfg.p0_att_rp, cfg.p0_att_rp, cfg.p0_att_yaw,
              [cfg.p0_ba] * 3, [cfg.p0_bg] * 3, cfg.p0_bh] ** 2
    P[:] = np.diag(d)
    # --- ZUPT detector
    if zupt_mask is None:
        still = zupt_detector(imu_t, f, cfg, fs)
    else:
        still = np.broadcast_to(zupt_mask, (B, N))
    # --- schedules
    n_cov = max(1, int(round(fs / cfg.cov_rate_hz)))
    n_nhc = max(1, int(round(fs / cfg.nhc_rate_hz)))
    g_idx = np.searchsorted(imu_t, g_t)                  # IMU sample at/after each fix stamp
    baro_1hz_t = np.arange(np.ceil(t0), imu_t[-1], 1.0)
    baro_idx = np.searchsorted(imu_t, baro_1hz_t)
    baro_avg = np.stack([np.nanmean(baro_h[:, (baro_t > tt - 1.0) & (baro_t <= tt)], axis=1) for tt in baro_1hz_t], 1)
    out_t, out_p, out_v, out_C, out_P, out_ba, out_z = [], [], [], [], [], [], []
    ev = {"gnss_pos_acc": 0, "gnss_pos_rej": 0, "gnss_vel_acc": 0, "gnss_vel_rej": 0, "zupt": 0}
    Fsum = np.zeros((B, NX, NX)); dt_sum = 0.0
    Qc = np.zeros(NX)
    Qc[3:6] = cfg.accel_noise ** 2; Qc[6:9] = cfg.gyro_noise ** 2
    Qc[9:12] = cfg.accel_bias_rw ** 2; Qc[12:15] = cfg.gyro_bias_rw ** 2; Qc[15] = cfg.baro_bias_rw ** 2
    fc = f[:, i0] - b_a
    a_k = np.einsum("bij,bj->bi", C, fc) + GRAVITY_N
    gi = int(np.searchsorted(g_idx, i0, side="right"))
    bi = int(np.searchsorted(baro_idx, i0, side="right"))     # first baro epoch after the start
    for k in range(i0, N - 1):
        dt = imu_t[k + 1] - imu_t[k]
        wm = 0.5 * (w[:, k] + w[:, k + 1]) - b_g
        C_next = C @ so3_exp_batch(wm * dt)
        f_next = f[:, k + 1] - b_a
        a_next = np.einsum("bij,bj->bi", C_next, f_next) + GRAVITY_N
        v_next = v + 0.5 * (a_k + a_next) * dt
        p = p + 0.5 * (v + v_next) * dt
        v, C, a_k = v_next, C_next, a_next
        # error-state Jacobian, accumulated
        f_n = a_k - GRAVITY_N
        F = np.zeros((B, NX, NX))
        F[:, 0:3, 3:6] = np.eye(3)
        F[:, 3:6, 6:9] = -skew_b(f_n)
        F[:, 3:6, 9:12] = -C
        F[:, 6:9, 12:15] = -C
        Fsum += F * dt; dt_sum += dt
        kk = k + 1
        do_cov = (kk - i0) % n_cov == 0
        upd_due = (gi < len(g_idx) and g_idx[gi] == kk) or ((kk - i0) % n_nhc == 0) or \
                  (bi < len(baro_idx) and baro_idx[bi] == kk) or still[:, kk].any()
        if not (do_cov or upd_due):
            continue
        if dt_sum > 0:
            Phi = np.eye(NX) + Fsum + 0.5 * Fsum @ Fsum
            P = Phi @ P @ np.swapaxes(Phi, 1, 2) + np.diag(Qc) * dt_sum
            Fsum[:] = 0.0; dt_sum = 0.0
        x = np.zeros((B, NX))
        # GNSS
        while gi < len(g_idx) and g_idx[gi] == kk:
            pz = gnss["pos"][:, gi]; vz = gnss["vel"][:, gi]
            okp = np.isfinite(pz[:, 0])
            if okp.any():
                sh, sv = gnss["acc_pos_h"][:, gi], gnss["acc_pos_v"][:, gi]
                R = np.zeros((B, 3, 3))
                R[:, 0, 0] = R[:, 1, 1] = (cfg.gnss_pos_factor * np.nan_to_num(sh, nan=1e3)) ** 2
                R[:, 2, 2] = (cfg.gnss_pos_factor * np.nan_to_num(sv, nan=1e3)) ** 2
                H = np.zeros((B, 3, NX)); H[:, :, 0:3] = np.eye(3)
                z = np.where(okp[:, None], p - pz, 0.0) - np.einsum("bij,bj->bi", H, x)
                Psub, xs = P[okp], x[okp]
                acc, _ = _update(xs, Psub, z[okp], H[okp], R[okp], CHI2_3_999)
                P[okp], x[okp] = Psub, xs
                ev["gnss_pos_acc"] += int(acc.sum()); ev["gnss_pos_rej"] += int((~acc).sum())
            okv = np.isfinite(vz[:, 0])
            if okv.any():
                sv_ = cfg.gnss_vel_factor * np.nan_to_num(gnss["acc_vel"][:, gi], nan=1e3)
                R = np.zeros((B, 3, 3))
                R[:, 0, 0] = R[:, 1, 1] = sv_ ** 2; R[:, 2, 2] = (1.5 * sv_) ** 2
                H = np.zeros((B, 3, NX)); H[:, :, 3:6] = np.eye(3)
                z = np.where(okv[:, None], v - vz, 0.0) - np.einsum("bij,bj->bi", H, x)
                Psub, xs = P[okv], x[okv]
                acc, _ = _update(xs, Psub, z[okv], H[okv], R[okv], CHI2_3_999)
                P[okv], x[okv] = Psub, xs
                ev["gnss_vel_acc"] += int(acc.sum()); ev["gnss_vel_rej"] += int((~acc).sum())
            gi += 1
        # barometer (1 Hz average)
        while bi < len(baro_idx) and baro_idx[bi] == kk:
            hz = baro_avg[:, bi]
            ok = np.isfinite(hz)
            H = np.zeros((B, 1, NX)); H[:, 0, 2] = 1.0; H[:, 0, 15] = 1.0
            z = ((p[:, 2] + b_h) - hz)[:, None] - np.einsum("bij,bj->bi", H, x)
            R = np.full((B, 1, 1), cfg.baro_sigma ** 2)
            Psub, xs = P[ok], x[ok]
            _update(xs, Psub, z[ok], H[ok], R[ok], None)
            P[ok], x[ok] = Psub, xs
            bi += 1
        # ZUPT
        st = still[:, kk] & (np.linalg.norm(v, axis=1) < cfg.zupt_speed_max)
        if st.any():
            H = np.zeros((B, 3, NX)); H[:, :, 3:6] = np.eye(3)
            z = v - np.einsum("bij,bj->bi", H, x)
            R = np.broadcast_to(np.eye(3) * cfg.zupt_sigma ** 2, (B, 3, 3)).copy()
            Psub, xs = P[st], x[st]
            _update(xs, Psub, z[st], H[st], R[st], None)
            P[st], x[st] = Psub, xs
            ev["zupt"] += int(st.sum())
        # NHC (body y, z velocity at R = 0)
        if (kk - i0) % n_nhc == 0:
            C_bn = C_bp @ np.swapaxes(C, 1, 2)                       # body <- nav
            w_b = np.einsum("ij,bj->bi", C_bp, w[:, kk] - b_g)
            v_R = v - np.einsum("bij,bj->bi", C @ C_pb, np.cross(w_b, r_b))
            v_b = np.einsum("bij,bj->bi", C_bn, v_R)
            H = np.zeros((B, 2, NX))
            H[:, :, 3:6] = C_bn[:, 1:3]
            H[:, :, 6:9] = (C_bn @ skew_b(v_R))[:, 1:3]
            z = v_b[:, 1:3] - np.einsum("bij,bj->bi", H, x)
            R = np.broadcast_to(np.eye(2) * cfg.nhc_sigma ** 2, (B, 2, 2)).copy()
            moving = np.linalg.norm(v, axis=1) > 1.0
            Psub, xs = P[moving], x[moving]
            if moving.any():
                _update(xs, Psub, z[moving], H[moving], R[moving], None)
                P[moving], x[moving] = Psub, xs
        # inject
        if np.any(x):
            p = p - x[:, 0:3]
            v = v - x[:, 3:6]
            C = so3_exp_batch(-x[:, 6:9]) @ C
            b_a = b_a - x[:, 9:12]
            b_g = b_g - x[:, 12:15]
            b_h = b_h - x[:, 15]
            fc = f[:, kk] - b_a
            a_k = np.einsum("bij,bj->bi", C, fc) + GRAVITY_N
        if do_cov:
            out_t.append(imu_t[kk]); out_p.append(p.copy()); out_v.append(v.copy()); out_C.append(C.copy())
            out_P.append(P[:, 0:3, 0:3].copy()); out_ba.append(b_a.copy()); out_z.append(st.copy())
    return EskfOutput(t=np.array(out_t), p=np.stack(out_p, 1), v=np.stack(out_v, 1), C=np.stack(out_C, 1),
                      P_pos=np.stack(out_P, 1), b_a=np.stack(out_ba, 1), zupt=np.stack(out_z, 1), events=ev)
