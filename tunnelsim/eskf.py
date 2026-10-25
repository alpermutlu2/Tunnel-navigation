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

from dataclasses import dataclass, field, replace

import numpy as np
from scipy.signal import butter, sosfiltfilt, sosfilt, sosfilt_zi

from .frames import GRAVITY_N
from .ins import so3_exp_batch

NX = 16
CHI2_3_999 = 16.27
CHI2_2_999 = 13.82
CHI2_1_999 = 10.83


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
    # Tunnel pressure gradient (Fable review, accepted 2026-10-20): during an outage the barometer
    # offset drifts with distance, ḃ_h = g_h·|v|; g_h restarts at 0 with σ_g at each outage.
    baro_grad: bool = False            # experimental: fixes 10-30 Pa/km, costs S3/S5 nominal (decision note)
    baro_grad_sigma: float = 3e-3      # m/m ≈ 35 Pa/km [ASSUMED]
    # GNSS gate lock recovery (accepted 2026-10-20): (i) after N consecutive rejections in open
    # sky, inflate P (non-position states) so the current innovation fits the gate ×1.5;
    # (ii) adaptive accel/gyro noise from the GNSS NIS (rises with a 20 s window, decays with 60 s).
    gate_recover: bool = True
    gate_recover_n: int = 5            # [ASSUMED]
    adaptive_q: bool = True
    adaptive_q_max: float = 30.0       # [ASSUMED]
    adaptive_q_rise_s: float = 20.0
    adaptive_q_fall_s: float = 60.0
    # Map in the tunnel (review 9, owner's design 2026-10-20): during an outage, 1 Hz heading and
    # cross-track pseudo-measurements against the route polyline; paused from 100 m before the
    # diverge until the in-loop branch decision, then the decided geometry (route or main).
    map_in_tunnel: bool = True
    map_heading_sigma: float = np.deg2rad(1.0)   # polyline heading (~0.5°) + driver in lane (~0.5°) [ASSUMED]
    map_lateral_sigma: float = 1.5               # m: lane-centre deviation + polyline error [ASSUMED]
    map_window_m: float = 300.0                  # ŝ searched within ±window of the last ŝ
    map_pre_m: float = 100.0                     # pause starts this far before the diverge
    map_branch_deg: float = 5.0                  # branch decision threshold (as branch.py)
    # Map-row innovation gate and branch deferral (owner 2026-10-25, v0.5.11 A/B; thresholds chosen
    # before the runs): reject the heading/cross-track update when its NIS exceeds χ² 99.9 % (2 dof,
    # 1 dof for the cross-track row alone), so a filter far along-track from the car cannot write a
    # wrong curve into yaw and position (S4, 2 runs in 200: −478 m); defer the branch decision while
    # σ_s exceeds the branch's clothoid length, read from the map's curvature profile.
    map_nis_gate: bool = True              # default since v0.5.11 (owner): fixes the two −500 m runs, costs three +110 m overshoots in 200 (S4)
    map_branch_defer: bool = False
    # Re-association along the arc (owner 2026-10-25, v0.5.11 A/B, threshold fixed before the runs): at
    # 1 Hz, candidates s_c in ŝ ± 3σ_s, J(s_c) = wrap(ψ̂ − ψ_map(s_c))²/(σ_ψ² + P_ψψ) + (s_c − ŝ)²/σ_s²;
    # if J(ŝ) − min J > χ²₁ 99.9 %, an along-track position pseudo-measurement pulls ŝ to s* = argmin J,
    # z = tᵀ(p̂_R − c(s*)), R = max(5 m, σ_ψ,eff/|κ(s*)|)². In a curve the yaw (gyro, accurate over
    # seconds) says where in the curve the car is; a point linearisation at a wrong ŝ cannot.
    map_reassoc: bool = True               # default since v0.5.11 (owner), with the gate as a safety net
    map_reassoc_n: int = 121
    map_reassoc_scale: bool = True     # the map length-error state in the re-association row (owner's map-error states)
    # map errors as states, constant per tunnel, reset at each outage (owner, 2026-10-20): lateral
    # offset of the polyline and a length (scale) error; the curve-matching gain is then bounded
    # by the map's own uncertainty instead of being taken as exact
    map_offset_sigma: float = 2.0                # m [ASSUMED]
    map_scale_sigma: float = 0.005               # 0.5 % [ASSUMED]
    # review 10 (optional): deterministic lateral effect of the scale error in the cross-track row,
    # δz += nᵀ(c − c_entry)·δε, instead of the turn-driven offset process noise
    map_scale_lateral: bool = True               # default since v0.5.3 (random map-error stress)
    nhc_turn_gate: float = 0.3        # m/s² [ASSUMED, review 7]; 0 = off. Lateral NHC dropped while |ω_z|·v exceeds this
    nhc_rate_hz: float = 5.0
    baro_sigma: float = 0.15          # m, 1 Hz average (E4)
    zupt_sigma: float = 0.02          # m/s (E5)
    zupt_band_hz: tuple = (2.0, 18.0)  # road-vibration band, below the engine tone (E5)
    zupt_band_rms: float = 0.01       # m/s² threshold on the 1 s band RMS of |f|
    zupt_hold_s: float = 0.3          # s: debounce
    zupt_lowf_std: float = 0.05       # m/s²: 1 s std of the < 2 Hz specific force vector (a launch
                                      # or a brake moves it by ~1 m/s²; a stopped car does not)
    zupt_causal: bool = True          # owner 2026-10-25 (v0.5.10): causal filters by default (real time); False: forward-backward
                                      # (offline, optimistic). 30 seeds: S4 diverge 30.3 -> 33.7 m, portal 4.8 -> 4.7 m, no other scenario changes
    zupt_speed_max: float = 2.0       # m/s: filter speed must be below this
    zupt_band_order: int = 4          # Butterworth order of the road band-pass [ASSUMED]
    zupt_lp_order: int = 2            # Butterworth order of the < 2 Hz low-pass [ASSUMED]
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
    # IMU class (review 11, owner 2026-10-21): "auto" detects the class from the gyro white noise in
    # the first IMU window and applies FILTER_PROFILES[class]; "note10" keeps the values above.
    # Levelling (gravity-direction) pseudo-measurement (review 11 option 3, owner 2026-10-21; A/B,
    # off by default): at 1 Hz while the car is in cruise mode or stopped and not turning or pitching,
    # the 1 s mean specific force is gravity plus the accelerometer bias, f̄ = −Cᵀg + b_a; it ties
    # tilt and the lumped bias directly instead of through the integrated speed.
    level: bool = False
    level_sigma: float = 0.02          # m/s²: unmodelled acceleration in "steady" (the cruise threshold) [ASSUMED]
    level_mode: str = "cruise"         # "cruise": in cruise mode and at stops (review 11 form); "stops": at stops only
                                       # (30 seeds: no effect, the ZUPT already holds there; reports/0.5.5)
    level_vmax: float = 0.0            # m/s; > 0: levelling only below this speed (crawl), 0 = any speed
    level_rate_gate: float = 0.3       # m/s²: skipped while the 1 s mean |ω_y,z|·v exceeds this (curves, grade changes)
    # Pitch prior from the road geometry (owner 2026-10-25, v0.5.11 A/B): during an outage and below
    # pitch_prior_vmax, the body pitch is tied to the map's road grade through a constant offset
    # state δγ (grade error, mount pitch, mean suspension pitch): z = θ̂_b − γ(ŝ) − δγ̂. The offset
    # restarts at each outage with pitch_prior_offset_sigma, so the prior constrains the change of
    # pitch (the gyro drift), not its absolute value. A profile field: sigma 0 = off.
    # Why: folding the fast gyro-bias terms into white noise (v0.5.7) kept the filter consistent but
    # made the attitude process noise so large (pitch spreads ~3.6°/√s on the MPU9250 profile)
    # that at low speed, where NHC and barometer see pitch only through v·δθ, an outside source
    # of pitch is needed; the road geometry is that source.
    pitch_prior_sigma: float = 0.0                  # rad; 0 = off [ASSUMED when on]
    pitch_prior_offset_sigma: float = np.deg2rad(1.0)   # δγ at the start of each outage [ASSUMED]
    pitch_prior_offset_rw: float = np.deg2rad(0.2) / np.sqrt(300.0)   # rad/√s: 0.2° over 300 s [ASSUMED]
    pitch_prior_vmax: float = 10.0                  # m/s [ASSUMED]: S4 crawls at 8.3 m/s
    pitch_prior_every_s: float = 0.1
    pitch_prior_map_grade: bool = True              # False: γ = 0 (a map without grade)
    # ZUPT speed gate source (owner 2026-10-25, A/B): a stop the detector has held without a break for
    # at least zupt_trust_s is applied whatever the filter speed; shorter claims keep the
    # zupt_speed_max gate. 0 = off (gate always on).
    zupt_trust_s: float = 0.0
    imu_class: str = "auto"
    imu_detect_s: float = 10.0
    imu_detect_th: float = 0.027      # rad/s per sample at 100 Hz ≈ ARW 2.7e-3 rad/√s: geometric mean of
                                      # Note10 (≤ 0.013 with noisy vibration) and MPU9250 (≥ 0.059), 10 seeds


# Filter-side tuning per IMU class (review 11, owner 2026-10-21). "note10" is the EskfConfig default.
# "mpu9250": the Note10 tuning rules applied to the MPU9250 Allan terms [MEAS, Suvorkin 2024 Table 5]:
#   gyro_noise    √(ARW_max² + vibration²), vibration 7.1e-4 from the Note10 value (√(1e-3² − 7e-4²)); v0.5.7: plus
#                 the τ ≤ 50 s bias-instability terms as white noise (see below)
#   gyro_bias_rw  Note10 × mean-gyro-BI ratio (4.79e-3 / 1.56e-4 = 30.7)
#   accel_bias_rw Note10 × mean-accel-BI ratio (1.07e-3 / 0.99e-3 = 1.08)
#   p0_bg         2 × turn-on (0.2° [ASSUMED]), the Note10 ratio
#   p0_ba         1.44 × rss(turn-on 30 mg, scale 1 %·g, misalignment 0.5°·g) = 1.44 × 0.32, the Note10 ratio
FILTER_PROFILES = {
    "note10": {},
    # v0.5.7 (owner 2026-10-24): gyro_noise also folds the bias-instability GM terms with τ ≤ 50 s into
    # white noise, √(ARW² + vib² + Σ_{τ≤50 s} 2τσᵢ²) on the worst axis = 0.063 rad/√s: those terms decorrelate
    # within the filter's update intervals and a random walk cannot follow them (A/B: reports/0.5.7/)
    # v0.5.8 (owner 2026-10-24): cruise_acc_th 0.05 m/s²: with this gyro the filter's along-track
    # acceleration is noisier, so the 0.02 threshold rarely let the cruise prior engage (A/B 30 seeds)
    "mpu9250": {"cruise_acc_th": 0.05, "gyro_noise": 0.0634, "gyro_bias_rw": 3.07e-3 / np.sqrt(180.0),
                "accel_bias_rw": 5.4e-4 / np.sqrt(180.0), "p0_bg": np.deg2rad(0.4), "p0_ba": 0.45},
}


def detect_imu_class(imu_t, w, cfg: EskfConfig, i0: int):
    """IMU class from the gyro white noise: σ̂ = std(Δω)/√2 per axis (differencing removes the real
    rotation), median over axes, over the imu_detect_s window ending at the filter start (or the
    first window if the run starts earlier). Returns (class per batch member, σ̂)."""
    fs = 1.0 / np.median(np.diff(imu_t))
    n = int(round(cfg.imu_detect_s * fs))
    a = max(0, i0 - n)
    sig = np.median(np.diff(w[:, a:a + n], axis=1).std(axis=1) / np.sqrt(2.0), axis=1)
    sig_100 = sig * np.sqrt(fs / 100.0)          # threshold is stated at 100 Hz
    return np.where(sig_100 > cfg.imu_detect_th, "mpu9250", "note10"), sig


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
    aux: dict = field(default_factory=dict)      # diagnostics: b_h, g_h, cruise flag (B,K)


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

    # causal (v0.5.10, owner 2026-10-25): one pass, starting from the filter's steady state for the
    # first sample. Deliberately conservative: one pass rejects the ~25 Hz engine tone less than
    # the forward-backward pass, which keeps idle and crawling vibration slightly above the band
    # threshold; the detector then misses part of each stop (a missed stop costs little) but does
    # not fire while the car crawls (a false stop at 3 m/s cost hundreds of metres with twice the
    # order, which restores the offline rejection: reports/0.5.10/zupt_order_ab.json).

    def _causal(so, x, axis):
        x = np.moveaxis(x, axis, -1)
        zi = np.einsum("sk,...->s...k", sosfilt_zi(so), x[..., 0])
        return np.moveaxis(sosfilt(so, x, axis=-1, zi=zi)[0], -1, axis)

    filt = _causal if cfg.zupt_causal else sosfiltfilt
    sos = butter(cfg.zupt_band_order, cfg.zupt_band_hz, btype="bandpass", fs=fs, output="sos")
    xb = filt(sos, np.linalg.norm(f, axis=-1), axis=-1)
    band_rms = np.sqrt(np.maximum(mov(xb * xb), 0.0))
    lp = butter(cfg.zupt_lp_order, 2.0, btype="lowpass", fs=fs, output="sos")
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
             zupt_mask=None, t_start=None, road_map=None):
    """Run the filter.

    imu_t (N,) shared time axis; f, w (B,N,3); gnss: dict of (B,M,...) arrays t (M,), pos, vel,
    acc_pos_h, acc_pos_v, acc_vel (NaN where missing); baro_t (L,), baro_h (B,L).
    zupt_mask: optional (N,) or (B,N) bool overriding the detector (oracle stops).
    road_map: optional `MapView` of the route (estimator-side map) for the map-in-tunnel layer.
    """
    B, N = f.shape[0], imu_t.size
    imu_classes, imu_sig = None, None
    cls_name = cfg.imu_class
    if cfg.imu_class == "auto":
        g0 = np.nonzero(np.all(np.isfinite(gnss["pos"][:, :, 0]), axis=0) & np.all(np.isfinite(gnss["vel"][:, :, 0]), axis=0))[0][0]
        i_det = int(np.searchsorted(imu_t, gnss["t"][g0] if t_start is None else t_start))
        imu_classes, imu_sig = detect_imu_class(imu_t, w, cfg, i_det)
        vals, cnt = np.unique(imu_classes, return_counts=True)   # one batch = one phone model
        cls_name = str(vals[np.argmax(cnt)])
        cfg = replace(cfg, **FILTER_PROFILES[cls_name])
    elif cfg.imu_class != "note10":
        cfg = replace(cfg, **FILTER_PROFILES[cfg.imu_class])
    NA = NX + 6          # (+ pitch-prior offset δγ, NX + 5)
    IP = NX + 5
    # + speed clone / v_ref (NX), driver wander d (NX + 1), baro gradient g_h (NX + 2),
    ID = NX + 1          #   map lateral offset (NX + 3), map scale error (NX + 4)
    IG = NX + 2
    IM, IE = NX + 3, NX + 4
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
    # time the detector has held "stopped" without a break, per sample [s]
    _idx = np.arange(N)[None, :]
    _last_off = np.maximum.accumulate(np.where(still, -1, _idx), axis=1)
    still_run = (_idx - _last_off) / fs
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
    out_t, out_p, out_v, out_C, out_P, out_ba, out_z, out_aux = [], [], [], [], [], [], [], []
    g_hat = np.zeros(B)                     # tunnel pressure gradient as height per metre [m/m]
    use_map = cfg.map_in_tunnel and road_map is not None
    s_map = np.full(B, np.nan)              # map-matched along-track position (outage only)
    br_dec = np.zeros(B, int)               # branch decision: 0 none, +1 route (branch), -1 main
    psi_start = np.full(B, np.nan); s_start = np.full(B, np.nan); map_prev_out = np.zeros(B, bool)
    ra_flag = np.zeros(B, bool)             # re-association applied since the last output epoch
    m_hat = np.zeros(B); e_hat = np.zeros(B); off_hat = np.zeros(B)   # + pitch-prior offset [rad]  # map lateral offset [m], map scale error [-]
    br_time = np.full(B, np.nan)              # time of the in-loop branch decision
    if use_map and road_map.s_div is not None:
        sg = np.arange(road_map.s_div, road_map.s_div + road_map.curve_len + 1.0, 0.5)
        _, psg, _ = road_map.at(sg)
        s_thr5 = sg[int(np.argmax(np.abs(psg - psg[0]) >= np.deg2rad(cfg.map_branch_deg)))]
        _, _, _, kg = road_map.at(sg, with_kappa=True)
        L_clothoid = float(sg[int(np.argmax(np.abs(kg) >= 0.99 * np.abs(kg).max()))] - sg[0])   # 0 -> |κ| max
    outage_prev = np.zeros(B, bool)
    d_acc = np.zeros(B)                     # distance travelled in the current outage (gradient lever)
    n_rej = np.zeros(B, int)                # consecutive open-sky GNSS rejections (pos or vel)
    q_scale = np.ones(B)                    # adaptive accel/gyro noise scale (α)
    ev = {"map_updates": 0, "map_branch": 0, "map_main": 0, "gate_inflations": 0, "gnss_pos_acc": 0, "gnss_pos_rej": 0, "gnss_vel_acc": 0, "gnss_vel_rej": 0, "zupt": 0,
          "gnss_pos_skipped_reacq": 0, "gnss_pos_capped": 0}
    def _gate_monitor(sel, acc, nis, open_sky):
        """(i) gate-lock recovery and (ii) adaptive noise from the GNSS NIS (3-dof updates)."""
        nonlocal P, q_scale
        idx = np.nonzero(sel)[0]
        if cfg.adaptive_q:
            r = nis / 3.0
            up = r > q_scale[idx]
            a = np.where(up, 1.0 / cfg.adaptive_q_rise_s, 1.0 / cfg.adaptive_q_fall_s)
            q_scale[idx] = np.clip(q_scale[idx] + a * (r - q_scale[idx]), 1.0, cfg.adaptive_q_max)
        if cfg.gate_recover:
            cnt = np.where(~acc & open_sky, n_rej[idx] + 1, 0)
            fire = cnt >= cfg.gate_recover_n
            if fire.any():
                lam = np.maximum(1.0, nis[fire] / CHI2_3_999) * 1.5
                S = np.ones((int(fire.sum()), NA)); S[:, 3:] = np.sqrt(lam)[:, None]
                fi = idx[fire]
                P[fi] = P[fi] * S[:, :, None] * S[:, None, :]      # S P S: stays PSD
                ev["gate_inflations"] += int(fire.sum())
                cnt = np.where(fire, 0, cnt)
            n_rej[idx] = cnt

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
    Qc[IP] = cfg.pitch_prior_offset_rw ** 2
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
        if cfg.baro_grad:
            spd_k = np.linalg.norm(v, axis=1)
            out_k = imu_t[k + 1] - last_vel > cfg.cruise_no_gnss_s
            F[:, 15, IG] = np.where(out_k, spd_k, 0.0)
            b_h = b_h + np.where(out_k, g_hat * spd_k * dt, 0.0)
            d_acc = d_acc + np.where(out_k, spd_k * dt, 0.0)
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
            Qk = np.broadcast_to(np.diag(Qc), (B, NA, NA)).copy()
            if cfg.adaptive_q:
                Qk[:, 3:9, 3:9] *= q_scale[:, None, None]
            P = Phi @ P @ np.swapaxes(Phi, 1, 2) + Qk * dt_sum
            if gm_mode:
                P[:, ID, ID] += cfg.cruise_gm_sigma ** 2 * (1.0 - phi_d ** 2)
            Fsum[:] = 0.0; dt_sum = 0.0
        x = np.zeros((B, NA))
        reclone = np.zeros(B, bool); gm_init = np.zeros(B, bool)
        if cfg.baro_grad:
            outage = imu_t[kk] - last_vel > cfg.cruise_no_gnss_s
            new_out = outage & ~outage_prev
            if new_out.any():
                # a new tunnel: its gradient is unknown and independent of the last one
                P[new_out, IG, :] = 0.0; P[new_out, :, IG] = 0.0
                P[new_out, IG, IG] = cfg.baro_grad_sigma ** 2
                g_hat = np.where(new_out, 0.0, g_hat)
                d_acc = np.where(new_out, 0.0, d_acc)
            ended = ~outage & outage_prev
            if ended.any():
                # out of the tunnel the gradient's pressure offset is gone: remove the accumulated
                # part c = g_h·d from b_h, δb_h' = δb_h − d·δg_h (exact transform of P)
                b_h = np.where(ended, b_h - g_hat * d_acc, b_h)
                T = np.broadcast_to(np.eye(NA), (B, NA, NA)).copy()
                T[:, 15, IG] = -np.where(ended, d_acc, 0.0)
                P = np.where(ended[:, None, None], T @ P @ np.swapaxes(T, 1, 2), P)
                d_acc = np.where(ended, 0.0, d_acc)
            outage_prev = outage
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
                        acc, nis = _update(xs, Psub, z[sel], H[sel], R[sel], CHI2_3_999)
                    P[sel], x[sel] = Psub, xs
                    if not capped:
                        _gate_monitor(sel, acc, nis, open_sky=(np.nan_to_num(sh, nan=1e9)[sel] <= cfg.gnss_max_reported_sigma_m))
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
                        acc, nis = _update(xs, Psub, z[sel], H[sel], R[sel], CHI2_3_999)
                    P[sel], x[sel] = Psub, xs
                    if not capped:
                        _gate_monitor(sel, acc, nis, open_sky=(np.nan_to_num(sh, nan=1e9)[sel] <= cfg.gnss_max_reported_sigma_m))
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
        st = still[:, kk] & ((np.linalg.norm(v, axis=1) < cfg.zupt_speed_max)
                             | ((cfg.zupt_trust_s > 0) & (still_run[:, kk] >= cfg.zupt_trust_s)))
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
        # map in the tunnel: heading + cross-track at 1 Hz during the outage (review 9)
        if use_map and (kk - i0) % int(round(fs)) == 0:
            tk = imu_t[kk]
            out_m = (tk - last_vel > cfg.cruise_no_gnss_s) & (np.linalg.norm(v, axis=1) > 1.0)
            new_m = out_m & ~map_prev_out
            br_dec = np.where(new_m, 0, br_dec); psi_start = np.where(new_m, np.nan, psi_start)
            if new_m.any():                       # a new tunnel: its map errors are unknown, independent
                for j, sg_ in ((IM, cfg.map_offset_sigma), (IE, cfg.map_scale_sigma), (IP, cfg.pitch_prior_offset_sigma)):
                    P[new_m, j, :] = 0.0; P[new_m, :, j] = 0.0; P[new_m, j, j] = sg_ ** 2
                m_hat = np.where(new_m, 0.0, m_hat); e_hat = np.where(new_m, 0.0, e_hat); off_hat = np.where(new_m, 0.0, off_hat)
            s_map = np.where(out_m, s_map, np.nan)
            map_prev_out = out_m
            if out_m.any():
                l_n = np.einsum("bij,j->bi", C, C_bp.T @ r_b)                     # phone -> R lever, nav
                pR = p - l_n
                fwd = np.einsum("bij,j->bi", C, C_bp.T @ np.array([1.0, 0.0, 0.0]))
                psi_hat = np.arctan2(fwd[:, 1], fwd[:, 0])
                s_new = road_map.nearest(pR[:, :2], s_map)
                if road_map.s_div is not None:
                    tm = np.array([np.cos(road_map.main_psi), np.sin(road_map.main_psi)])
                    s_main = road_map.s_div + (pR[:, :2] - road_map.main_xy0) @ tm
                    s_new = np.where(br_dec == -1, np.maximum(s_main, road_map.s_div), s_new)
                s_map = np.where(out_m, s_new, np.nan)
                if cfg.map_reassoc:
                    # only on a decided branch geometry: before the decision the route's branch is a
                    # hypothesis, and a car going straight on matched to it is pulled back to the
                    # straight before the diverge (negative control: 92-170 m in v0.5.11 A/B)
                    ra = out_m & ((br_dec == 1) if road_map.s_div is not None else np.ones(B, bool))
                    if ra.any():
                        sm0 = np.nan_to_num(s_map)
                        _, ps0_, _ = road_map.at(sm0)
                        t0_ = np.stack([np.cos(ps0_), np.sin(ps0_)], 1)
                        sg_s = np.sqrt(np.einsum("bi,bij,bj->b", t0_, P[:, 0:2, 0:2], t0_))
                        u_ = np.linspace(-3.0, 3.0, cfg.map_reassoc_n)
                        cand = sm0[:, None] + sg_s[:, None] * u_[None]
                        _, psc, _ = road_map.at(cand.ravel())
                        psc = psc.reshape(cand.shape)
                        var_psi = cfg.map_heading_sigma ** 2 + P[:, 8, 8]
                        J = np.angle(np.exp(1j * (psi_hat[:, None] - psc))) ** 2 / var_psi[:, None] + u_[None] ** 2
                        jbest = np.argmin(J, axis=1)
                        j0 = J[:, cfg.map_reassoc_n // 2]
                        go = ra & (j0 - J[np.arange(B), jbest] > CHI2_1_999)
                        if go.any():
                            s_star = cand[np.arange(B), jbest]
                            c_s, ps_s, _, k_s = road_map.at(s_star, with_kappa=True)
                            t_s = np.stack([np.cos(ps_s), np.sin(ps_s)], 1)
                            Hr = np.zeros((B, 1, NA)); Hr[:, 0, 0:2] = t_s
                            Hr[:, 0, 6:9] = np.einsum("bi,bij->bj", np.c_[t_s, np.zeros(B)], skew_b(l_n))
                            if cfg.map_reassoc_scale:
                                # the candidate is a map point: with the map's length error ε the true point
                                # lies tᵀ(c − c_entry)·ε further on (the lever the other map rows use)
                                lev_s = np.einsum("bi,bi->b", t_s, c_s - road_map.xy_entry)
                                Hr[:, 0, IE] = lev_s            # as the cross-track row: h = tᵀ(p̂_R − c) + lever·ε̂
                            else:
                                lev_s = np.zeros(B)
                            zr = (np.einsum("bi,bi->b", t_s, pR[:, :2] - c_s) + lev_s * e_hat)[:, None] - np.einsum("bij,bj->bi", Hr, x)
                            sig_a = np.maximum(5.0, np.sqrt(var_psi) / np.maximum(np.abs(k_s), 1e-6))
                            Rr = (sig_a ** 2)[:, None, None]
                            Psub, xs = P[go], x[go]
                            _update(xs, Psub, zr[go], Hr[go], Rr[go], None)
                            P[go], x[go] = Psub, xs
                            s_map = np.where(go, s_star, s_map)
                            ra_flag |= go
                            ev["map_reassoc"] = ev.get("map_reassoc", 0) + int(go.sum())
                paused = np.zeros(B, bool)
                if road_map.s_div is not None:
                    inwin = out_m & (br_dec == 0) & (s_map >= road_map.s_div - cfg.map_pre_m)
                    first = inwin & ~np.isfinite(psi_start)
                    psi_start = np.where(first, psi_hat, psi_start); s_start = np.where(first, s_map, s_start)
                    if inwin.any():
                        _, ps0, _ = road_map.at(np.nan_to_num(s_start))
                        _, ps1, _ = road_map.at(np.nan_to_num(s_map))
                        dpsi = np.angle(np.exp(1j * (psi_hat - psi_start)))
                        thr = np.deg2rad(cfg.map_branch_deg)
                        r_b_ = dpsi - (ps1 - ps0); r_m = dpsi
                        t2 = np.stack([np.cos(ps1), np.sin(ps1)], 1)
                        sig_s = np.sqrt(np.einsum("bi,bij,bj->b", t2, P[:, 0:2, 0:2], t2))
                        ready = (sig_s <= L_clothoid) if cfg.map_branch_defer else np.ones(B, bool)
                        go_b = inwin & ready & (np.abs(r_m) > thr) & (np.abs(r_b_) < np.abs(r_m))
                        go_m = inwin & ready & ~go_b & (np.abs(r_b_) > thr) & (np.abs(r_m) < np.abs(r_b_)) & (s_map - 2 * sig_s >= s_thr5)
                        br_dec = np.where(go_b, 1, np.where(go_m, -1, br_dec))
                        br_time = np.where(go_b | go_m, tk, br_time)
                        ev["map_branch"] += int(go_b.sum()); ev["map_main"] += int(go_m.sum())
                    paused = (br_dec == 0) & (s_map >= road_map.s_div - cfg.map_pre_m)
                sel = out_m & ~paused
                if sel.any():
                    c, psi_m, n, kap = road_map.at(np.nan_to_num(s_map), main=(br_dec == -1), with_kappa=True)
                    # a length error also moves the polyline sideways after every turn (the map is a
                    # scaled copy): the offset state may change by σ_ε·d·|Δψ| over a curve
                    dpsi_step = np.abs(kap) * np.linalg.norm(v, axis=1) * 1.0
                    if not cfg.map_scale_lateral:
                        P[:, IM, IM] += np.where(out_m, (cfg.map_scale_sigma * np.maximum(np.nan_to_num(s_map) - road_map.s_entry, 0.0) * dpsi_step) ** 2, 0.0)
                    t2 = np.stack([np.cos(psi_m), np.sin(psi_m)], 1)
                    dist = np.nan_to_num(s_map) - road_map.s_entry                    # lever of the scale error
                    H = np.zeros((B, 2, NA))
                    # heading: ψ̂ − (ψ_map(ŝ) + κ·d·ε̂); δz = δθ_z − κ tᵀδp − κ d δε (curve matching)
                    H[:, 0, 8] = 1.0
                    H[:, 0, 0:2] = -kap[:, None] * t2
                    H[:, 0, IE] = -kap * dist
                    # cross-track: nᵀ(p̂_R − c_map(ŝ)) + m̂; δz = nᵀδp + nᵀ[l×]δθ + δm
                    H[:, 1, 0:2] = n
                    H[:, 1, 6:9] = np.einsum("bi,bij->bj", np.c_[n, np.zeros(B)], skew_b(l_n))
                    H[:, 1, IM] = 1.0
                    lever = np.einsum("bi,bi->b", n, c - road_map.xy_entry) if cfg.map_scale_lateral else np.zeros(B)
                    H[:, 1, IE] = lever
                    z = np.stack([np.angle(np.exp(1j * (psi_hat - psi_m - kap * dist * e_hat))),
                                  np.einsum("bi,bi->b", n, pR[:, :2] - c) + m_hat + lever * e_hat], 1) - np.einsum("bij,bj->bi", H, x)
                    R = np.zeros((B, 2, 2)); R[:, 0, 0] = cfg.map_heading_sigma ** 2; R[:, 1, 1] = cfg.map_lateral_sigma ** 2
                    w_b = np.einsum("ij,bj->bi", C_bp, w[:, kk] - b_g)
                    turning = np.abs(w_b[:, 2]) * np.linalg.norm(v, axis=1) > cfg.nhc_turn_gate if cfg.nhc_turn_gate > 0 else np.zeros(B, bool)
                    for ss, rows, gate in ((sel & ~turning, slice(0, 2), CHI2_2_999), (sel & turning, slice(1, 2), CHI2_1_999)):   # heading off in turns (sideslip)
                        if ss.any():
                            Psub, xs = P[ss], x[ss]
                            acc_m, _ = _update(xs, Psub, z[ss][:, rows], H[ss][:, rows], R[ss][:, rows, rows],
                                               gate if cfg.map_nis_gate else None)
                            P[ss], x[ss] = Psub, xs
                            ev["map_rejected"] = ev.get("map_rejected", 0) + int((~acc_m).sum())
                    ev["map_updates"] += int(sel.sum())
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
            moving = np.linalg.norm(v, axis=1) > 1.0
            turning = np.zeros(B, bool)
            if cfg.nhc_turn_gate > 0.0:
                # the sideslip error is the same sign for the whole curve (correlated), so R
                # inflation only slows it down; skip the lateral row while turning instead
                a_lat = np.abs(w_b[:, 2]) * np.linalg.norm(v, axis=1)
                turning = a_lat > cfg.nhc_turn_gate
            for sel, rows in ((moving & ~turning, slice(0, 2)), (moving & turning, slice(1, 2))):
                if sel.any():
                    Psub, xs = P[sel], x[sel]
                    _update(xs, Psub, z[sel][:, rows], H[sel][:, rows], R[sel][:, rows, rows], None)
                    P[sel], x[sel] = Psub, xs
        # pitch prior from the road (v0.5.11 A/B): θ_b = asin(u_z), u = Ĉ C_pb e_x (body forward, nav);
        # z = θ̂_b − γ(ŝ) − δγ̂; ∂/∂δθ = [u_y, −u_x, 0]/cos θ_b (Ĉ = Exp(δθ)C), ∂/∂δp = −γ'(ŝ)·tᵀ, ∂/∂δγ = −1
        if (cfg.pitch_prior_sigma > 0 and use_map
                and (kk - i0) % max(1, int(round(cfg.pitch_prior_every_s * fs))) == 0):
            spd_p = np.linalg.norm(v, axis=1)
            sel = np.isfinite(s_map) & (spd_p < cfg.pitch_prior_vmax) & (imu_t[kk] - last_vel > cfg.cruise_no_gnss_s)
            if sel.any():
                u = np.einsum("bij,j->bi", C, C_bp.T @ np.array([1.0, 0.0, 0.0]))
                th = np.arcsin(np.clip(u[:, 2], -1.0, 1.0))
                sm = np.nan_to_num(s_map)
                gam, gam_r = road_map.grade_at(sm) if cfg.pitch_prior_map_grade else (np.zeros(B), np.zeros(B))
                _, psi_g, _ = road_map.at(sm)
                H = np.zeros((B, 1, NA))
                H[:, 0, 6] = u[:, 1] / np.cos(th); H[:, 0, 7] = -u[:, 0] / np.cos(th)
                H[:, 0, 0] = -gam_r * np.cos(psi_g); H[:, 0, 1] = -gam_r * np.sin(psi_g)
                H[:, 0, IP] = -1.0
                z = (th - gam - off_hat)[:, None] - np.einsum("bij,bj->bi", H, x)
                R = np.full((B, 1, 1), cfg.pitch_prior_sigma ** 2)
                Psub, xs = P[sel], x[sel]
                _update(xs, Psub, z[sel], H[sel], R[sel], None)
                P[sel], x[sel] = Psub, xs
                ev["pitch_prior_updates"] = ev.get("pitch_prior_updates", 0) + int(sel.sum())
        # levelling: f̄ = −Ĉᵀg + b̂_a; δz = −Ĉᵀ[g×]δθ + δb_a  (Ĉ = Exp(δθ)C, δ = estimate − truth)
        if cfg.level and (kk - i0) % int(round(fs)) == 0 and kk - i0 >= int(round(fs)):
            n1 = int(round(fs))
            w_win = np.einsum("ij,bkj->bki", C_bp, w[:, kk - n1 + 1:kk + 1] - b_g[:, None])
            wm = w_win.mean(axis=1)                    # 1 s mean: the per-sample max would be gyro noise
            rate = np.hypot(wm[:, 1], wm[:, 2]) * np.linalg.norm(v, axis=1)
            stopped = still[:, kk - n1 + 1:kk + 1].all(axis=1)
            sel = (stopped | (cruise_on if cfg.level_mode == "cruise" else False)) & (rate < cfg.level_rate_gate)
            if cfg.level_vmax > 0:
                sel = sel & (np.linalg.norm(v, axis=1) < cfg.level_vmax)
            if sel.any():
                fbar = f[:, kk - n1 + 1:kk + 1].mean(axis=1)
                Ct = np.swapaxes(C, 1, 2)
                H = np.zeros((B, 3, NA))
                H[:, :, 6:9] = -Ct @ skew_b(np.broadcast_to(GRAVITY_N, (B, 3)))
                H[:, :, 9:12] = np.eye(3)
                z = (-np.einsum("bij,j->bi", Ct, GRAVITY_N) + b_a - fbar) - np.einsum("bij,bj->bi", H, x)
                R = np.broadcast_to(np.eye(3) * cfg.level_sigma ** 2, (B, 3, 3)).copy()
                Psub, xs = P[sel], x[sel]
                _update(xs, Psub, z[sel], H[sel], R[sel], None)
                P[sel], x[sel] = Psub, xs
                ev["level_updates"] = ev.get("level_updates", 0) + int(sel.sum())
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
            g_hat = g_hat - x[:, IG]
            m_hat = m_hat - x[:, IM]; e_hat = e_hat - x[:, IE]; off_hat = off_hat - x[:, IP]
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
            out_aux.append(np.stack([b_h, g_hat, cruise_on.astype(float), np.sqrt(P[:, IG, IG]), ra_flag.astype(float)], 1))
            ra_flag[:] = False
    return EskfOutput(t=np.array(out_t), p=np.stack(out_p, 1), v=np.stack(out_v, 1), C=np.stack(out_C, 1),
                      P_pos=np.stack(out_P, 1), b_a=np.stack(out_ba, 1), zupt=np.stack(out_z, 1), events=ev,
                      aux={**dict(zip(("b_h", "g_h", "cruise", "sigma_g", "reassoc"), np.moveaxis(np.stack(out_aux, 1), -1, 0))),
                           "branch_decision": br_dec.copy(), "branch_time": br_time.copy(),
                           "imu_class": imu_classes, "imu_sigma": imu_sig})
