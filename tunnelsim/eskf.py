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
from scipy.signal import butter, sosfiltfilt, sosfilt

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
    nhc_turn_k: float = 0.0           # rad/(m/s²); 0 = off. Lateral NHC inflation in turns (sideslip)
    nhc_turn_gate: float = 0.3        # m/s² [ASSUMED, review 7]; 0 = off. Lateral NHC dropped while |ω_z|·v exceeds this
    nhc_rate_hz: float = 5.0
    baro_sigma: float = 0.15          # m, 1 Hz average (E4)
    zupt_sigma: float = 0.02          # m/s (E5)
    zupt_band_hz: tuple = (2.0, 18.0)  # road-vibration band, below the engine tone (E5)
    zupt_band_rms: float = 0.01       # m/s² threshold on the 1 s band RMS of |f|
    zupt_hold_s: float = 0.3          # s: debounce
    zupt_lowf_std: float = 0.05       # m/s²: 1 s std of the < 2 Hz specific force vector (a launch
                                      # or a brake moves it by ~1 m/s²; a stopped car does not)
    zupt_causal: bool = False         # True: causal filters (real time); False: forward-backward (offline, optimistic)
    zupt_speed_max: float = 2.0       # m/s: filter speed must be below this
    cov_rate_hz: float = 10.0         # covariance propagation rate
    # GNSS re-acquisition after an outage (v0.5 A, accepted 2026-10-20): robust over fast
    reacq_gap_s: float = 10.0         # an outage longer than this starts a re-acquisition phase
    reacq_min_s: float = 10.0         # the phase lasts at least this long ...
    reacq_settle_fixes: int = 5       # ... and ends after this many uncapped position updates in a row
    reacq_vel_only_s: float = 3.0     # first seconds: velocity (Doppler) only, no position
    reacq_r_factor: float = 10.0      # position sigma inflation at the start of the phase ...
    reacq_r_tau_s: float = 8.0        # ... decaying to 1 with this time constant
    reacq_max_step_m: float = 2.0     # largest position correction per fix in the phase (gain scaled)
    gnss_max_reported_sigma_m: float = 10.0   # position fixes reporting more than this are not used
    reacq: bool = True                # switch for A/B comparison
    # B. Cruise (speed-hold) mode (accepted 2026-10-20). Without GNSS velocity for > 5 s, moving
    # and not stopped: if the 5 s mean of the filter's along-track acceleration is below the
    # threshold for 5 s, hold the speed with a pseudo-measurement whose variance grows like an
    # undetectable sub-threshold acceleration: R = σ0² + (a_th Δt)²/3.
    cruise: bool = True              # B accepted 2026-10-20; GM speed-wander form by default (review 6)
    cruise_acc_th: float = 0.02       # m/s²
    cruise_win_s: float = 5.0
    cruise_sigma0: float = 0.05       # m/s
    cruise_no_gnss_s: float = 5.0
    cruise_min_speed: float = 3.0     # m/s
    cruise_mode: str = "gm"           # "gm" (default since review 6), "increment" (b) or "hold" (a)
    cruise_hold_kP: float = 0.0       # (a) R inflation by k·P_hold
    cruise_inc_sigma: float = 0.0     # (b) extra white speed noise per increment [ASSUMED 0: truth has no jitter]
    # "gm" (review 6): speed = v_ref + d, v_ref a clone of the speed at the start of the mode,
    # d a Gauss-Markov driver speed wander; a statistical prior on the driver, not a sensor.
    cruise_gm_sigma: float = 0.5      # m/s, σ_d [ASSUMED, review 6 nominal]
    cruise_gm_tau: float = 60.0       # s, τ_d [ASSUMED, review 6 nominal]
    cruise_gm_r: float = 0.02         # m/s, residual of the identity 0 = v∥ − v_ref − d [ASSUMED]
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
    filt = (lambda so, x, axis: sosfilt(so, x, axis=axis)) if cfg.zupt_causal else sosfiltfilt
    xb = filt(sos, np.linalg.norm(f, axis=-1), axis=-1)
    band_rms = np.sqrt(np.maximum(mov(xb * xb), 0.0))
    lp = butter(2, 2.0, btype="lowpass", fs=fs, output="sos")
    fl = filt(lp, f, axis=-2)                                         # (B,N,3)
    var = sum(np.maximum(mov(fl[..., i] ** 2) - mov(fl[..., i]) ** 2, 0.0) for i in range(3))
    raw = (band_rms < cfg.zupt_band_rms) & (np.sqrt(var) < cfg.zupt_lowf_std)
    # debounce: both conditions held without a break for the last zupt_hold_s
    n = max(1, int(cfg.zupt_hold_s * fs))
    return mov(raw.astype(float)) > 1.0 - 1e-9


def _update_capped(x_corr, P, z, H, R, gate, max_step):
    """As _update, but the gain is scaled by α ≤ 1 so the position correction is at most
    max_step (per seed). Joseph form keeps the covariance valid for any gain."""
    S = H @ P @ np.swapaxes(H, 1, 2) + R
    Sinv = np.linalg.inv(S)
    nis = np.einsum("bi,bij,bj->b", z, Sinv, z)
    ok = nis < gate if gate is not None else np.ones(len(z), bool)
    K = P @ np.swapaxes(H, 1, 2) @ Sinv
    K[~ok] = 0.0
    dx = np.einsum("bij,bj->bi", K, z)
    step = np.linalg.norm(dx[:, 0:3], axis=1)
    alpha = np.where(step > max_step, max_step / np.maximum(step, 1e-12), 1.0)
    K = K * alpha[:, None, None]
    x_corr += dx * alpha[:, None]
    IKH = np.eye(P.shape[-1]) - K @ H
    P[:] = IKH @ P @ np.swapaxes(IKH, 1, 2) + K @ R @ np.swapaxes(K, 1, 2)
    return ok, alpha


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
    IKH = np.eye(P.shape[-1]) - K @ H
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
    NA = NX + 2          # + speed clone / v_ref (index NX) and driver wander d (NX + 1), cruise only
    ID = NX + 1
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
    P = np.zeros((B, NA, NA))
    d = np.r_[[cfg.p0_pos] * 3, [cfg.p0_vel] * 3, cfg.p0_att_rp, cfg.p0_att_rp, cfg.p0_att_yaw,
              [cfg.p0_ba] * 3, [cfg.p0_bg] * 3, cfg.p0_bh] ** 2
    P[:, :NX, :NX] = np.diag(d)
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
    # the 1 s average represents the height at the window's mean sample time, ~0.5 s before the
    # epoch: predicted h = p_z − lag·v_z + b_h (on a 5 % grade at 70 km/h the uncorrected lag is
    # ~0.5 m and was read as a pitch error at every grade change; found on S3)
    baro_lag = np.array([tt - baro_t[(baro_t > tt - 1.0) & (baro_t <= tt)].mean() for tt in baro_1hz_t])
    out_t, out_p, out_v, out_C, out_P, out_ba, out_z = [], [], [], [], [], [], []
    ev = {"gnss_pos_acc": 0, "gnss_pos_rej": 0, "gnss_vel_acc": 0, "gnss_vel_rej": 0, "zupt": 0,
          "gnss_pos_skipped_reacq": 0, "gnss_pos_capped": 0}
    last_fix = np.full(B, t0)              # time of the last position fix used
    last_vel = np.full(B, t0)              # time of the last GNSS velocity
    n_win = max(1, int(round(cfg.cruise_win_s * cfg.cov_rate_hz)))
    a_buf = np.zeros((B, n_win)); a_cnt = 0  # ring buffer of along-track accel at cov rate
    cruise_on = np.zeros(B, bool); calm_since = np.full(B, t0); v_hold = np.zeros(B); t_hold = np.zeros(B)
    n_upd = np.zeros(B); P_hold = np.zeros(B)
    s_clone = np.zeros(B); t_clone = np.zeros(B); clone_ok = np.zeros(B, bool)
    d_hat = np.zeros(B)                     # driver wander estimate (gm mode)
    gm_mode = cfg.cruise and cfg.cruise_mode == "gm"
    ev["cruise_updates"] = 0
    reacq_start = np.full(B, -np.inf)       # start of the current re-acquisition phase
    reacq_on = np.zeros(B, bool)            # phase active (ends on convergence, not on a timer)
    settle = np.zeros(B, int)               # uncapped position updates in a row within the phase
    Fsum = np.zeros((B, NA, NA)); dt_sum = 0.0
    Qc = np.zeros(NA)
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
        F = np.zeros((B, NA, NA))
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
            Phi = np.eye(NA) + Fsum + 0.5 * Fsum @ Fsum
            if gm_mode:
                # d: discrete GM, φ = exp(−Δt/τ_d), Q_d = σ_d²(1 − φ²) (review 6)
                phi_d = np.exp(-dt_sum / cfg.cruise_gm_tau)
                Phi[:, ID, ID] = phi_d
                d_hat = d_hat * phi_d
            P = Phi @ P @ np.swapaxes(Phi, 1, 2) + np.diag(Qc) * dt_sum
            if gm_mode:
                P[:, ID, ID] += cfg.cruise_gm_sigma ** 2 * (1.0 - phi_d ** 2)
            Fsum[:] = 0.0; dt_sum = 0.0
        x = np.zeros((B, NA))
        reclone = np.zeros(B, bool); gm_init = np.zeros(B, bool)
        # GNSS
        while gi < len(g_idx) and g_idx[gi] == kk:
            pz = gnss["pos"][:, gi]; vz = gnss["vel"][:, gi]
            tf = g_t[gi]
            sh, sv = gnss["acc_pos_h"][:, gi], gnss["acc_pos_v"][:, gi]
            okp = np.isfinite(pz[:, 0]) & (np.nan_to_num(sh, nan=1e9) <= cfg.gnss_max_reported_sigma_m)
            fac = np.ones(B)
            in_reacq = np.zeros(B, bool)
            if cfg.reacq:
                any_fix = okp | np.isfinite(vz[:, 0])
                new_phase = any_fix & (tf - last_fix > cfg.reacq_gap_s)
                reacq_start = np.where(new_phase, tf, reacq_start)
                reacq_on = reacq_on | new_phase
                settle = np.where(new_phase, 0, settle)
                age = tf - reacq_start
                done = reacq_on & (age >= cfg.reacq_min_s) & (settle >= cfg.reacq_settle_fixes)
                reacq_on = reacq_on & ~done
                in_reacq = reacq_on
                skip = okp & in_reacq & (age < cfg.reacq_vel_only_s)
                ev["gnss_pos_skipped_reacq"] += int(skip.sum())
                last_fix = np.where(any_fix, tf, last_fix)  # the outage is over once fixes return
                okp = okp & ~skip
                fac = np.where(in_reacq, 1.0 + (cfg.reacq_r_factor - 1.0) * np.exp(-np.maximum(age, 0) / cfg.reacq_r_tau_s), 1.0)
            if okp.any():
                R = np.zeros((B, 3, 3))
                R[:, 0, 0] = R[:, 1, 1] = (fac * cfg.gnss_pos_factor * np.nan_to_num(sh, nan=1e3)) ** 2
                R[:, 2, 2] = (fac * cfg.gnss_pos_factor * np.nan_to_num(sv, nan=1e3)) ** 2
                H = np.zeros((B, 3, NA)); H[:, :, 0:3] = np.eye(3)
                z = np.where(okp[:, None], p - pz, 0.0) - np.einsum("bij,bj->bi", H, x)
                cap = okp & in_reacq
                for sel, capped in ((okp & ~cap, False), (cap, True)):
                    if not sel.any():
                        continue
                    Psub, xs = P[sel], x[sel]
                    if capped:
                        acc, alpha = _update_capped(xs, Psub, z[sel], H[sel], R[sel], CHI2_3_999, cfg.reacq_max_step_m)
                        ev["gnss_pos_capped"] += int((alpha < 1).sum())
                        sidx = np.nonzero(sel)[0]
                        settle[sidx] = np.where(alpha < 1, 0, settle[sidx] + 1)
                    else:
                        acc, _ = _update(xs, Psub, z[sel], H[sel], R[sel], CHI2_3_999)
                    P[sel], x[sel] = Psub, xs
                    ev["gnss_pos_acc"] += int(acc.sum()); ev["gnss_pos_rej"] += int((~acc).sum())
            okv = np.isfinite(vz[:, 0])
            last_vel = np.where(okv, g_t[gi], last_vel)
            if okv.any():
                sv_ = cfg.gnss_vel_factor * np.nan_to_num(gnss["acc_vel"][:, gi], nan=1e3)
                R = np.zeros((B, 3, 3))
                R[:, 0, 0] = R[:, 1, 1] = sv_ ** 2; R[:, 2, 2] = (1.5 * sv_) ** 2
                H = np.zeros((B, 3, NA)); H[:, :, 3:6] = np.eye(3)
                z = np.where(okv[:, None], v - vz, 0.0) - np.einsum("bij,bj->bi", H, x)
                # in the re-acquisition phase the position part of a velocity update is capped
                # too: at the end of a long tunnel P_pv is large, so a velocity fix alone can
                # move the position by tens of metres
                capv = okv & in_reacq
                for sel, capped in ((okv & ~capv, False), (capv, True)):
                    if not sel.any():
                        continue
                    Psub, xs = P[sel], x[sel]
                    if capped:
                        acc, alpha = _update_capped(xs, Psub, z[sel], H[sel], R[sel], CHI2_3_999, cfg.reacq_max_step_m)
                        ev["gnss_pos_capped"] += int((alpha < 1).sum())
                    else:
                        acc, _ = _update(xs, Psub, z[sel], H[sel], R[sel], CHI2_3_999)
                    P[sel], x[sel] = Psub, xs
                    ev["gnss_vel_acc"] += int(acc.sum()); ev["gnss_vel_rej"] += int((~acc).sum())
            gi += 1
        # barometer (1 Hz average)
        while bi < len(baro_idx) and baro_idx[bi] == kk:
            hz = baro_avg[:, bi]
            ok = np.isfinite(hz)
            lag = baro_lag[bi]
            H = np.zeros((B, 1, NA)); H[:, 0, 2] = 1.0; H[:, 0, 5] = -lag; H[:, 0, 15] = 1.0
            z = ((p[:, 2] - lag * v[:, 2] + b_h) - hz)[:, None] - np.einsum("bij,bj->bi", H, x)
            R = np.full((B, 1, 1), cfg.baro_sigma ** 2)
            Psub, xs = P[ok], x[ok]
            _update(xs, Psub, z[ok], H[ok], R[ok], None)
            P[ok], x[ok] = Psub, xs
            bi += 1
        # ZUPT
        st = still[:, kk] & (np.linalg.norm(v, axis=1) < cfg.zupt_speed_max)
        if st.any():
            H = np.zeros((B, 3, NA)); H[:, :, 3:6] = np.eye(3)
            z = v - np.einsum("bij,bj->bi", H, x)
            R = np.broadcast_to(np.eye(3) * cfg.zupt_sigma ** 2, (B, 3, 3)).copy()
            Psub, xs = P[st], x[st]
            _update(xs, Psub, z[st], H[st], R[st], None)
            P[st], x[st] = Psub, xs
            ev["zupt"] += int(st.sum())
        # B. cruise mode (decided at the covariance rate, pseudo-measurement at 1 Hz)
        if cfg.cruise and do_cov:
            spd = np.linalg.norm(v, axis=1)
            u = v / np.maximum(spd, 1e-6)[:, None]
            a_buf[:, a_cnt % n_win] = np.einsum("bi,bi->b", a_k, u)
            a_cnt += 1
            a_mean = a_buf.mean(1) if a_cnt >= n_win else np.full(B, np.inf)
            tk = imu_t[kk]
            calm = np.abs(a_mean) < cfg.cruise_acc_th
            eligible = (tk - last_vel > cfg.cruise_no_gnss_s) & (spd > cfg.cruise_min_speed) & ~st
            calm_since = np.where(calm, calm_since, tk)
            cruise_on = cruise_on & calm & eligible
            start = ~cruise_on & calm & eligible & (tk - calm_since >= cfg.cruise_win_s)
            v_hold = np.where(start, spd, v_hold); t_hold = np.where(start, tk, t_hold)
            P_hold = np.where(start, np.einsum("bi,bij,bj->b", u, P[:, 3:6, 3:6], u), P_hold)
            n_upd = np.where(start, 0, n_upd)
            clone_ok = clone_ok & cruise_on
            cruise_on = cruise_on | start
            if cruise_on.any() and (kk - i0) % int(round(fs)) == 0:
                H = np.zeros((B, 1, NA)); H[:, 0, 3:6] = u
                if cfg.cruise_mode == "hold":
                    # (a) absolute hold, v_hold from the filter's own state; k-th update R×k and
                    # R inflated by k_P·P_hold (the hold's own variance) -- known to be
                    # overconfident for any finite k_P (decision note B)
                    z = (spd - v_hold)[:, None] - np.einsum("bij,bj->bi", H, x)
                    n_upd = n_upd + cruise_on
                    R = ((cfg.cruise_sigma0 ** 2 + (cfg.cruise_acc_th * (tk - t_hold)) ** 2 / 3.0
                          + cfg.cruise_hold_kP * P_hold) * np.maximum(n_upd, 1))[:, None, None]
                    sel = cruise_on
                else:
                    # (b) speed increment over the last second against a stochastic clone of
                    # the speed: z = speed_k − clone, H = [u on δv, −1 on the clone]. Consecutive
                    # increments are independent, and the clone carries the correlation with
                    # the state, so only the along-track drift (accel bias + pitch·g) is observed.
                    H[:, 0, NX] = -1.0
                    z = (spd - s_clone)[:, None] - np.einsum("bij,bj->bi", H, x)
                    dtc = tk - t_clone
                    R = (cfg.cruise_inc_sigma ** 2 + (cfg.cruise_acc_th * dtc) ** 2 / 3.0)[:, None, None]
                    sel = cruise_on & clone_ok
                if cfg.cruise_mode == "gm":
                    # identity 0 = v∥ − v_ref − d; the speed-change uncertainty is in d's process
                    H[:, 0, NX] = -1.0; H[:, 0, ID] = -1.0
                    z = (spd - s_clone - d_hat)[:, None] - np.einsum("bij,bj->bi", H, x)
                    R = np.full((B, 1, 1), cfg.cruise_gm_r ** 2)
                    sel = cruise_on & clone_ok
                if sel.any():
                    Psub, xs = P[sel], x[sel]
                    _update(xs, Psub, z[sel], H[sel], R[sel], None)
                    P[sel], x[sel] = Psub, xs
                    ev["cruise_updates"] += int(sel.sum())
                if cfg.cruise_mode == "increment":
                    reclone = cruise_on
            if cfg.cruise_mode == "gm":
                gm_init = gm_init | start
        # NHC (body y, z velocity at R = 0)
        if (kk - i0) % n_nhc == 0:
            C_bn = C_bp @ np.swapaxes(C, 1, 2)                       # body <- nav
            w_b = np.einsum("ij,bj->bi", C_bp, w[:, kk] - b_g)
            v_R = v - np.einsum("bij,bj->bi", C @ C_pb, np.cross(w_b, r_b))
            v_b = np.einsum("bij,bj->bi", C_bn, v_R)
            H = np.zeros((B, 2, NA))
            H[:, :, 3:6] = C_bn[:, 1:3]
            H[:, :, 6:9] = (C_bn @ skew_b(v_R))[:, 1:3]
            z = v_b[:, 1:3] - np.einsum("bij,bj->bi", H, x)
            R = np.broadcast_to(np.eye(2) * cfg.nhc_sigma ** 2, (B, 2, 2)).copy()
            if cfg.nhc_turn_k > 0.0:
                # sideslip in turns violates the lateral NHC: inflate it by the sideslip a car of
                # gain k could have, β ≈ k·a_lat with a_lat ≈ ω_z·v (gyro, estimator side)
                spd_ = np.linalg.norm(v, axis=1)
                a_lat = np.abs(w_b[:, 2]) * spd_
                R[:, 0, 0] += (cfg.nhc_turn_k * a_lat * spd_) ** 2
            if cfg.nhc_turn_gate > 0.0:
                # the sideslip error is the same sign for the whole curve (correlated), so R
                # inflation only slows it down; drop the lateral row while turning instead
                spd_ = np.linalg.norm(v, axis=1)
                a_lat = np.abs(w_b[:, 2]) * spd_
                R[:, 0, 0] = np.where(a_lat > cfg.nhc_turn_gate, 1e6, R[:, 0, 0])
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
            s_clone = s_clone - x[:, NX]
            d_hat = d_hat - x[:, ID]
            fc = f[:, kk] - b_a
            a_k = np.einsum("bij,bj->bi", C, fc) + GRAVITY_N
        if reclone.any():
            # new clone of the (corrected) speed: δc = uᵀδv, so the clone row/column copy the
            # speed's covariance with every state
            spd = np.linalg.norm(v, axis=1)
            u = v / np.maximum(spd, 1e-6)[:, None]
            J = np.zeros((B, NA)); J[:, 3:6] = u
            row = np.einsum("bi,bij->bj", J, P)
            Pn = P.copy()
            Pn[:, NX, :] = row; Pn[:, :, NX] = row
            Pn[:, NX, NX] = np.einsum("bj,bj->b", row, J)
            P = np.where(reclone[:, None, None], Pn, P)
            s_clone = np.where(reclone, spd, s_clone); t_clone = np.where(reclone, imu_t[kk], t_clone)
            clone_ok = clone_ok | reclone
        if gm_init.any():
            # start of the gm mode: v̂_ref = speed, d̂ = 0. With δ = estimate − truth,
            # δv_ref = δs − δd, d ~ N(0, σ_d²) independent of the state: P_ref,ref = P_ss + σ_d²,
            # P_ref,d = −σ_d², P_d,d = σ_d², P_ref,x = P_s,x (cross-covariance kept, review 6)
            spd = np.linalg.norm(v, axis=1)
            u = v / np.maximum(spd, 1e-6)[:, None]
            J = np.zeros((B, NA)); J[:, 3:6] = u
            row = np.einsum("bi,bij->bj", J, P)
            Pn = P.copy()
            Pn[:, ID, :] = 0.0; Pn[:, :, ID] = 0.0
            row[:, ID] = 0.0
            Pn[:, NX, :] = row; Pn[:, :, NX] = row
            sd2 = cfg.cruise_gm_sigma ** 2
            Pn[:, NX, NX] = np.einsum("bj,bj->b", row, J) + sd2
            Pn[:, NX, ID] = Pn[:, ID, NX] = -sd2
            Pn[:, ID, ID] = sd2
            P = np.where(gm_init[:, None, None], Pn, P)
            s_clone = np.where(gm_init, spd, s_clone); d_hat = np.where(gm_init, 0.0, d_hat)
            t_clone = np.where(gm_init, imu_t[kk], t_clone)
            clone_ok = clone_ok | gm_init
        if do_cov:
            out_t.append(imu_t[kk]); out_p.append(p.copy()); out_v.append(v.copy()); out_C.append(C.copy())
            out_P.append(P[:, 0:3, 0:3].copy()); out_ba.append(b_a.copy()); out_z.append(st.copy())
    return EskfOutput(t=np.array(out_t), p=np.stack(out_p, 1), v=np.stack(out_v, 1), C=np.stack(out_C, 1),
                      P_pos=np.stack(out_P, 1), b_a=np.stack(out_ba, 1), zupt=np.stack(out_z, 1), events=ev)
