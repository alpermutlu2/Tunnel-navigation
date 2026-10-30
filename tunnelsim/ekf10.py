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
# Short-term yaw-gyro noise for the curve landmark (estimator-side copy of the sensor spec, worst axis,
# [MEAS] Suvorkin et al. 2024 Table 5 via tunnelsim.sensors.bi_components): ARW [rad/√s] and the
# bias-instability GM terms with τ ≤ 50 s as (σ [rad/s], τ [s]); the τ = 500 s term is the bias state.
LANDMARK_GYRO = {"note10": (7.0e-4, ((2.2e-4, 5.0), (1.8e-4, 50.0))),
                 "mpu9250": (6.25e-3, ((6.64e-3, 5.0), (5.61e-3, 50.0)))}
NX = 11
I_S, I_V, I_H, I_T, I_PSI, I_BA, I_BGY, I_BGZ, I_BH, I_SG, I_MY = range(NX)   # + mount yaw (v0.5.13)


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
    # v0.5.12 (owner 2026-10-26), in this order:
    # (a) curve s-guard: while σ_s exceeds the route's shortest clothoid, the map-heading and
    #     curvature-gyro rows do not correct s (H_s = 0): linearised over more than the curve they lock
    #     s to a wrong point (low-end S4: σ_s 350 → 20 m at −160 m error). Same criterion as the
    #     ESKF's branch deferral (L_clothoid).
    # (b) curve landmark: the yaw CHANGE over the last landmark_T s, Δψ_g(t_j) = ∫(w_z − b̂_gz)dt/(1+ŝ_g),
    #     matched against ψ_map(c − d_T + d_j) − ψ_map(c − d_T) for candidates c in s ± 3σ_s
    #     (d_j = ∫v̂ dt). The bias is nearly constant over T, so only the short-term gyro noise counts
    #     (the profile's 0.063 rad/√s folds τ ≤ 50 s bias terms into white noise for 300 s consistency,
    #     which is the wrong model over 12 s). C = ARW²·min(t_i,t_j) + Σ_GM(τ≤50 s) + P_bgz·t_i t_j
    #     + diag(sideslip k·v·|w| and map heading)². Fires only when |Δψ_g| > landmark_min_deg (a turn was
    #     seen) and after this filter's own "branch" decision (below); one update per T.
    # (a) and (b) are one switch: (a) without (b) is consistent but worse (S1-5km Note10: 39 runs
    # > 100 m in 300), so that configuration cannot be built.
    # Own branch decision (owner 2026-10-26, the batch form of the ESKF's r_b / r_m test): the same
    # yaw-change profile against the same candidates, once with the route (branch) geometry, J_kol, once
    # with the main tunnel (after the diverge: straight along the diverge heading, as the ESKF's map
    # object), J_ana. min J_ana − min J_kol > χ²₁ → "branch", the reverse → "main" (the map rows then
    # use the main geometry and the landmark stays off), otherwise "not yet". Decided once per outage.
    curve_landmark: bool = True           # default since v0.5.12 (owner): (a) + (b)
    landmark_margin: bool = False         # v0.5.12 A/B: require the best candidate to beat the best one
                                          # outside its own peak by χ²₁ (seed 219, a wrong lock)
    # One-way tunnel: the car does not reverse (owner 2026-10-26, [ASSUMED]). While v̂ < 0 in an
    # outage, z = 0 − v̂ with R = zupt_sigma² (the simplest inequality constraint); nothing for v̂ ≥ 0.
    # A limit, not a cure: the crawl pitch error that drives v̂ negative is left (pitch prior, §5).
    no_reverse: bool = True
    # v0.5.13 (owner 2026-10-29, A/B): speed from the lateral force in a turn. In a turn of rate ω the
    # car's centripetal acceleration is v·ω, so the 1 s mean lateral specific force f_y = v·ω: κ cancels,
    # a measurement of speed that needs neither the map nor s: z = f̄_y − b̂_y − v̂·(ω̄ − b̂_gz), applied
    # when |ω̄| > lateral_omega_min. b̂_y, the lateral
    # accelerometer bias plus the mount's residual roll, is the mean f_y over straight driving (|ω| < 0.005
    # rad/s, 30 s memory). R: accelerometer noise over 1 s, body roll leaking g·sin φ (10 % of v·ω), the
    # gyro bias instability times v, lane keeping (v·σ_wander) and the b̂_y error (0.02 m/s²) [ASSUMED].
    lateral_speed: bool = True            # default since v0.5.13 (owner)
    # v0.5.13 (owner, option a): the mount's yaw error ψ_m as a random constant, as in the ESKF. With the
    # mount rotated by ψ_m about the body z axis the forward and lateral forces mix: f_x = f'_x − ψ_m f'_y,
    # f_y = f'_y + ψ_m f'_x (f' as read through the estimated mount). Prior σ from how the mount was found
    # (magnetometer only ~6°, with the velocity-direction step ~1°); 0 = known mount, the state is frozen.
    mount_yaw_sigma: float = 0.0
    lateral_omega_min: float = 0.02       # rad/s [ASSUMED]
    no_reverse_k: float = 0.0             # A/B: act only while v̂ < −k·σ_v (0 = owner's form)
    no_reverse_sigma: float = 0.5         # m/s [ASSUMED]; soft form, default since v0.5.12 (owner): R = (0.02 m/s)² gave 12 runs > 100 m in 300 (low-end S4), this 2
    landmark_gate: str = "own"            # "own": this filter's branch decision; "turn": a turn seen only
                                          # (v0.5.12 A/B: on a straight main tunnel the two must agree)
    landmark_T: float = 12.0              # s [ASSUMED] (owner: 10-15 s, one curve)
    landmark_min_deg: float = 5.0         # [ASSUMED] (owner)
    landmark_n: int = 121
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
    branch_decision: np.ndarray = None   # (B,) final: +1 branch, −1 main, 0 not yet
    branch_time: np.ndarray = None       # (B,) time of the decision (NaN: none)


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


def _landmark_cov(n, arw, gm, dt_fine=0.1):
    """Covariance (n,n) of the integrated short-term gyro noise at t = 1..n s: ARW and GM terms."""
    t = np.arange(1, n + 1, dtype=float)
    C = arw ** 2 * np.minimum(t[:, None], t[None])
    u = np.arange(0.0, n, dt_fine) + 0.5 * dt_fine
    L = (u[None] < t[:, None]).astype(float) * dt_fine                  # ∫_0^t_i as a sum
    for sg, tau in gm:
        K = sg ** 2 * np.exp(-np.abs(u[:, None] - u[None]) / tau)
        C += L @ K @ L.T
    return C


def _min_clothoid(road, step=0.5):
    """Shortest stretch of the route over which |κ| changes (a clothoid), m; inf if none."""
    sg = np.arange(0.0, road.length, step)
    m = np.abs(road.curvature_rate(sg)) > 1e-9
    if not m.any():
        return np.inf
    d = np.diff(np.r_[0, m.astype(int), 0]); a, b = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]
    L = (b - a) * step
    return float(L[L > 1.0].min()) if (L > 1.0).any() else np.inf


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
    fy = np.einsum("j,bnj->bn", C_bp[1], f)
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
    P[:, I_MY, I_MY] = max(cfg10.mount_yaw_sigma, 1e-9) ** 2
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
    fysum = np.zeros(B); fxsum = np.zeros(B); by_hat = np.full(B, np.nan)       # lateral force sum, its straight-road mean
    L_clo = _min_clothoid(road_map)
    nT = int(round(cfg10.landmark_T))
    ex0 = road_map.exits[0] if getattr(road_map, "exits", None) else None
    s_div = float(ex0.s) if ex0 is not None and ex0.on_route else None
    psi_div = float(road_map.heading(np.array([s_div]))[0]) if s_div is not None else 0.0

    def heading_g(sq, main):
        """Map heading at sq (B,...) on the route, or on the main tunnel where main (B,) is set."""
        h = road_map.heading(np.clip(sq, 0.0, road_map.length - 1.0).ravel()).reshape(np.shape(sq))
        if s_div is None:
            return h
        m = np.broadcast_to(np.reshape(main, (-1,) + (1,) * (np.ndim(sq) - 1)), np.shape(sq)) & (sq >= s_div)
        return np.where(m, psi_div, h)

    br10 = np.zeros(B, int); br10_t = np.full(B, np.nan); prev_out = np.zeros(B, bool)
    if cfg10.curve_landmark:
        arw_, gm_ = LANDMARK_GYRO.get(name, LANDMARK_GYRO["note10"])
        C_lm0 = _landmark_cov(nT, arw_, gm_)
        tT = np.arange(1, nT + 1, dtype=float)
        lm_psi = np.zeros(B); lm_d = np.zeros(B)                          # integrated Δψ_g and ∫v dt
        lm_buf_psi, lm_buf_d, lm_buf_a = [], [], []                        # 1 Hz samples (newest last)
        lm_last = np.full(B, -np.inf)
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
        fy_k = 0.5 * (fy[:, k] + fy[:, k + 1])
        a = 0.5 * (fx[:, k] + fx[:, k + 1]) - x[:, I_MY] * fy_k - x[:, I_BA] - G0 * np.sin(th)
        th_rate = -(0.5 * (wy[:, k] + wy[:, k + 1]) - x[:, I_BGY])
        psi_rate = 0.5 * (wz[:, k] + wz[:, k + 1]) - x[:, I_BGZ]
        x[:, I_S] += v * dt + 0.5 * a * dt * dt
        x[:, I_H] += v * np.sin(th) * dt
        x[:, I_V] += a * dt
        x[:, I_T] += th_rate * dt
        x[:, I_PSI] += psi_rate * dt
        F = np.zeros((B, NX, NX))
        F[:, I_S, I_V] = 1.0
        F[:, I_V, I_T] = -G0 * np.cos(th); F[:, I_V, I_BA] = -1.0; F[:, I_V, I_MY] = -fy_k
        F[:, I_H, I_V] = np.sin(th); F[:, I_H, I_T] = v * np.cos(th)
        F[:, I_T, I_BGY] = 1.0
        F[:, I_PSI, I_BGZ] = -1.0
        Fsum += F * dt; dt_sum += dt
        wsum += 0.5 * (wz[:, k] + wz[:, k + 1]); wn += 1
        fysum += 0.5 * (fy[:, k] + fy[:, k + 1]); fxsum += 0.5 * (fx[:, k] + fx[:, k + 1])
        if cfg10.curve_landmark:
            lm_psi += psi_rate * dt / (1.0 + x[:, I_SG]); lm_d += v * dt
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
        # --- curve landmark and the branch decision (outage only; see Ekf10Config)
        if map_due and cfg10.curve_landmark:
            out_ = imu_t[kk] - last_fix > 2.0
            new_o = out_ & ~prev_out; prev_out = out_
            br10 = np.where(new_o, 0, br10); br10_t = np.where(new_o, np.nan, br10_t)
            lm_buf_psi.append(np.where(out_, lm_psi, np.nan)); lm_buf_d.append(lm_d.copy())
            lm_buf_a.append(np.abs(x[:, I_V] * (wsum / max(wn, 1))))             # |a_lat| ≈ v·|w|
            if len(lm_buf_psi) > nT + 1:
                lm_buf_psi.pop(0); lm_buf_d.pop(0); lm_buf_a.pop(0)
            if len(lm_buf_psi) == nT + 1:
                Pg = np.stack(lm_buf_psi, 1); Dg = np.stack(lm_buf_d, 1); Ag = np.stack(lm_buf_a, 1)
                dpsi = Pg[:, 1:] - Pg[:, :1]; dd = Dg - Dg[:, :1]                   # (B,nT), (B,nT+1)
                full = np.isfinite(dpsi).all(axis=1)
                seen = full & (np.abs(np.nan_to_num(dpsi[:, -1])) > np.deg2rad(cfg10.landmark_min_deg))
                sg_s = np.sqrt(P[:, I_S, I_S])
                u_ = np.linspace(-3.0, 3.0, cfg10.landmark_n)
                cur = x[:, I_S, None] + lever + sg_s[:, None] * u_[None]              # candidate current position
                st0 = cur - dd[:, -1:]
                pos = st0[:, :, None] + dd[:, None, 1:]                               # (B,n_c,nT)
                undec = full & (br10 == 0) & (s_div is not None) & (cur[:, -1] >= (s_div or 0.0))
                lm_go0 = seen & (br10 == 1) & (imu_t[kk] - lm_last >= cfg10.landmark_T)
                if s_div is None or cfg10.landmark_gate == "turn":
                    lm_go0 = seen & (imu_t[kk] - lm_last >= cfg10.landmark_T)
                if (undec | lm_go0).any():
                    C_ = (C_lm0[None] + P[:, I_BGZ, I_BGZ, None, None] * tT[None, :, None] * tT[None, None, :])
                    slip = cfg10.slip_gain * np.abs(Ag[:, 1:] - Ag[:, :1])
                    C_ = C_ + np.einsum("bi,ij->bij", slip ** 2 + 2.0 * np.deg2rad(cfg10.heading_sigma_deg) ** 2, np.eye(nT))
                    Ci = np.linalg.inv(C_)
                    dp_ = np.nan_to_num(dpsi)[:, None, :]

                    def cost(main):
                        r_ = dp_ - np.angle(np.exp(1j * (heading_g(pos, main) - heading_g(st0, main)[:, :, None])))
                        return np.einsum("bci,bij,bcj->bc", r_, Ci, r_)
                    Jk = cost(np.zeros(B, bool))
                    ar = np.arange(B); mid = cfg10.landmark_n // 2
                    if undec.any():
                        Ja = cost(np.ones(B, bool))
                        mk, ma = (Jk + u_[None] ** 2).min(axis=1), (Ja + u_[None] ** 2).min(axis=1)
                        to_b = undec & (ma - mk > CHI2_1_999); to_m = undec & (mk - ma > CHI2_1_999)
                        br10 = np.where(to_b, 1, np.where(to_m, -1, br10))
                        br10_t = np.where(to_b | to_m, imu_t[kk], br10_t)
                        ev["branch10"] = ev.get("branch10", 0) + int(to_b.sum()); ev["main10"] = ev.get("main10", 0) + int(to_m.sum())
                        if cfg10.landmark_gate == "own":
                            lm_go0 = seen & (br10 == 1) & (imu_t[kk] - lm_last >= cfg10.landmark_T)
                    J = Jk + u_[None] ** 2
                    jb = np.argmin(J, axis=1)
                    jb_ = np.clip(jb, 1, cfg10.landmark_n - 2)
                    h = sg_s * (u_[1] - u_[0])
                    d2 = (Jk[ar, jb_ + 1] - 2 * Jk[ar, jb_] + Jk[ar, jb_ - 1]) / np.maximum(h, 1e-9) ** 2
                    sig_a = np.maximum(5.0, np.sqrt(2.0 / np.maximum(d2, 1e-12)))
                    chi_n = nT * (1 - 2 / (9 * nT) + 3.09 * np.sqrt(2 / (9 * nT))) ** 3   # χ²_nT 99.9 % (Wilson-Hilferty)
                    plaus = Jk[ar, jb] < chi_n                                         # the matched profile fits
                    go = lm_go0 & plaus & (J[:, mid] - J[ar, jb] > CHI2_1_999) & (jb > 0) & (jb < cfg10.landmark_n - 1)
                    if cfg10.landmark_margin:
                        # best outside the best one's own peak (± max(3σ_a, 20 m)) must be χ²₁ worse
                        far = np.abs(cur - cur[ar, jb][:, None]) > np.maximum(3.0 * sig_a, 20.0)[:, None]
                        J2 = np.where(far, J, np.inf).min(axis=1)
                        go &= (J2 - J[ar, jb] > CHI2_1_999)
                    if go.any():
                        Hl = np.zeros((B, NX)); Hl[:, I_S] = 1.0
                        update(go, np.where(go, cur[ar, jb] - lever - x[:, I_S], 0.0), Hl, sig_a ** 2, None)
                        lm_last = np.where(go, imu_t[kk], lm_last)
                        ev["landmark"] = ev.get("landmark", 0) + int(go.sum())
                    ev["landmark_seen"] = ev.get("landmark_seen", 0) + int(lm_go0.sum())
        main_ = br10 == -1
        guard = (np.sqrt(P[:, I_S, I_S]) > L_clo) if cfg10.curve_landmark else np.zeros(B, bool)
        # --- map heading (global along the route)
        if map_due and cfg10.heading_update:
            sp = x[:, I_S] + lever
            psi_m, kap = heading_g(sp, main_), road_map.curvature(sp)
            if s_div is not None:
                kap = np.where(main_ & (sp >= s_div), 0.0, kap)              # main tunnel: straight
            v = x[:, I_V]
            z = np.angle(np.exp(1j * (psi_m - x[:, I_PSI])))          # h = psi - psi_map(s); z = 0 - h
            H = np.zeros((B, NX)); H[:, I_S] = np.where(guard, 0.0, -kap); H[:, I_PSI] = 1.0
            R = np.deg2rad(cfg10.heading_sigma_deg) ** 2 + (cfg10.slip_gain * v * v * kap) ** 2   # sideslip beta = -k a_lat
            update(np.ones(B, bool), z, H, R, CHI2_1_999)
            ev["heading"] += B
        # --- curvature-gyro (speed from the curve), 1 s mean rate at the window centre
        if map_due and wn > 0 and cfg10.curvature_update:
            wm = wsum / wn
            th, v = x[:, I_T], x[:, I_V]
            sc = x[:, I_S] + lever - 0.5 * v * (wn / fs)
            kap, kap_r, cth = road_map.curvature(sc), road_map.curvature_rate(sc), np.cos(th)
            if s_div is not None:
                on_main = main_ & (sc >= s_div)
                kap = np.where(on_main, 0.0, kap); kap_r = np.where(on_main, 0.0, kap_r)
            sg = 1.0 + x[:, I_SG]
            H = np.zeros((B, NX))
            H[:, I_S] = np.where(guard, 0.0, sg * v * kap_r * cth); H[:, I_V] = sg * kap * cth; H[:, I_T] = -sg * v * kap * np.sin(th)
            H[:, I_BGZ] = 1.0; H[:, I_SG] = v * kap * cth
            z = wm - (sg * v * kap * cth + x[:, I_BGZ])
            R = (cfg.gyro_noise ** 2 / (wn / fs) + sig_bi ** 2
                 + (cfg10.slip_gain * np.abs(v) ** 3 * np.abs(kap_r)) ** 2      # sideslip rate where kappa changes
                 + cfg10.lane_wander_rate_sigma ** 2)                           # lane keeping
            update(np.ones(B, bool), z, H, np.broadcast_to(R, (B,)).copy() if np.ndim(R) == 0 else R, CHI2_1_999)
            ev["curv"] += B
        if map_due and wn > 0:
            wm_ = wsum / wn; fym = fysum / wn; fxm = fxsum / wn
            wc = wm_ - x[:, I_BGZ]
            straight = np.abs(wc) < 0.005
            a_ = 1.0 / 30.0
            by_hat = np.where(straight, np.where(np.isfinite(by_hat), (1 - a_) * by_hat + a_ * fym, fym), by_hat)
            if cfg10.lateral_speed:
                v = x[:, I_V]
                # the 1 s mean rate's own short-term noise (sensor ARW and τ ≤ 50 s bias terms, as the
                # landmark): the row needs a turn well above it, else noise peaks are selected as turns
                # (low-end S4 at 0.02 rad/s: 12 runs > 100 m in 20) and v̂·ω̄ carries that noise into z
                arw_l, gm_l = LANDMARK_GYRO.get(name, LANDMARK_GYRO["note10"])
                sig_w1 = np.sqrt(arw_l ** 2 / (wn / fs) + sum(sg_ ** 2 for sg_, _ in gm_l))
                sel = (np.abs(wc) > max(cfg10.lateral_omega_min, 4.0 * sig_w1)) & np.isfinite(by_hat) & (v > 1.0)
                if sel.any():
                    # f'_y = v·ω − ψ_m f'_x (ψ_m from the mount-yaw state)
                    H = np.zeros((B, NX)); H[:, I_V] = wc; H[:, I_BGZ] = -v; H[:, I_MY] = -fxm
                    z = np.where(sel, fym - np.nan_to_num(by_hat) - (v * wc - x[:, I_MY] * fxm), 0.0)
                    R = (cfg.accel_noise ** 2 / (wn / fs) + (0.1 * v * wc) ** 2 + (v * sig_bi) ** 2 + (v * sig_w1) ** 2
                         + (v * cfg10.lane_wander_rate_sigma) ** 2 + 0.02 ** 2)
                    ok = update(sel, z, H, R, CHI2_1_999)
                    ev["lateral_speed"] = ev.get("lateral_speed", 0) + int(ok.sum())
        if map_due and wn > 0:
            wsum[:] = 0.0; wn = 0; fysum[:] = 0.0; fxsum[:] = 0.0
        # --- ZUPT
        if st.any():
            H = np.zeros((B, NX)); H[:, I_V] = 1.0
            update(st, -x[:, I_V], H, np.full(B, cfg.zupt_sigma ** 2), None)
            ev["zupt"] += int(st.sum())
        if cfg10.no_reverse and do_cov:
            rev = (imu_t[kk] - last_fix > 2.0) & (x[:, I_V] < -cfg10.no_reverse_k * np.sqrt(P[:, I_V, I_V]))
            if rev.any():
                H = np.zeros((B, NX)); H[:, I_V] = 1.0
                update(rev, -x[:, I_V], H, np.full(B, max(cfg.zupt_sigma, cfg10.no_reverse_sigma) ** 2), None)
                ev["no_reverse"] = ev.get("no_reverse", 0) + int(rev.sum())
        if do_cov:
            out["t"].append(imu_t[kk]); out["s"].append(x[:, I_S].copy()); out["v"].append(x[:, I_V].copy())
            out["sig_s"].append(np.sqrt(P[:, I_S, I_S])); out["theta"].append(x[:, I_T].copy())
            out["psi"].append(x[:, I_PSI].copy()); out["b_a"].append(x[:, I_BA].copy())
    st_ = lambda L: np.stack(L, 1)
    return Ekf10Output(np.array(out["t"]), st_(out["s"]), st_(out["v"]), st_(out["sig_s"]), st_(out["theta"]),
                       st_(out["psi"]), st_(out["b_a"]), ev, cls, br10.copy(), br10_t.copy())
