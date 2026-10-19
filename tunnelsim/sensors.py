"""Sensor generators (DESIGN.md section 3, decisions K1 to K9 and R1 to R4).

Separation (DESIGN.md 7.2): each generator returns a measurement object from `meas.py`
(the only thing an estimator may see) and a separate error object holding every component,
for analysis only.

Error terms can be switched off one by one. Each term draws from its own random stream,
keyed by a fixed id (`TERM_IDS`), so switching one term off, or adding a new term later,
never changes the samples of any other term.

IMU model, per phone axis (R1: vibration passes through the sensor errors):

    f~ = (I + S_a + M_a)(f_p + vib_a) + b_a,on + b_a,BI(t) + n_a
    w~ = (I + S_g + M_g)(w_p + vib_g) + b_g,on + b_g,BI(t) + n_g

Temperature drift is not modelled (R4).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.signal import butter, lfilter, sosfilt

from .allan import FLICKER_K, avar_gm, fit_gm_sigmas
from .frames import G0, mount_rotation
from .meas import BaroMeas, GnssMeas, ImuMeas
from .profiles import NOMINAL, ImuProfile, SensorConfig

# Fixed ids, never reused or renumbered. 1-99 IMU, 100-199 barometer, 200-299 GNSS.
TERM_IDS = {
    "a_white": 1, "g_white": 2,
    "a_bi": 3, "g_bi": 4,
    "a_turnon": 5, "g_turnon": 6,
    "a_scale": 7, "g_scale": 8,
    "a_misalign": 9, "g_misalign": 10,
    "vib_road": 11, "vib_engine": 12,
    "ts_jitter": 13, "ts_gaps": 14,
    "b_white": 101, "b_gm": 102, "b_offset": 103, "b_cabin": 104, "b_tunnel": 105,
    "gnss_pos": 201, "gnss_vel_white": 202, "gnss_vel_gm": 203, "gnss_exit": 204, "gnss_degrade": 205,
}

# Allan peak of a first-order GM: max_T ADEV / sigma (0.617 at T = 1.89 tau)
_T = np.logspace(-2, 3, 20001)
GM_ADEV_PEAK = float(np.sqrt(avar_gm(_T, 1.0, 1.0)).max())
GM1_SIGMA_PER_B = float(np.sqrt(FLICKER_K) / GM_ADEV_PEAK)          # 1.075


def term_rng(seed: int, term: str) -> np.random.Generator:
    return np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(TERM_IDS[term],)))


@dataclass(frozen=True)
class Switches:
    """Which error terms are on. Default: all on."""
    off: frozenset = frozenset()

    def __post_init__(self):
        bad = set(self.off) - set(TERM_IDS)
        if bad:
            raise KeyError(f"unknown terms {sorted(bad)}")

    @classmethod
    def all_on(cls) -> "Switches":
        return cls()

    @classmethod
    def only(cls, *terms: str) -> "Switches":
        bad = set(terms) - set(TERM_IDS)
        if bad:
            raise KeyError(f"unknown terms {sorted(bad)}")
        return cls(frozenset(set(TERM_IDS) - set(terms)))

    def without(self, *terms: str) -> "Switches":
        return Switches(frozenset(self.off | set(terms)))

    def __call__(self, term: str) -> bool:
        if term not in TERM_IDS:
            raise KeyError(term)
        return term not in self.off


# ---------------------------------------------------------------------------------------------
# Noise building blocks (also used directly by the long static Allan test)
# ---------------------------------------------------------------------------------------------
def white(rng, n: int, dt: float, density) -> np.ndarray:
    """White noise with density N per axis: sample std N / sqrt(dt). Shape (n, len(N))."""
    density = np.atleast_1d(np.asarray(density, dtype=float))
    return rng.standard_normal((n, density.size)) * (density / np.sqrt(dt))


def gm_sum(rng, n: int, dt: float, sigmas, taus) -> np.ndarray:
    """Sum of independent first-order GM processes, stationary from the first sample."""
    out = np.zeros(n)
    for s, tau in zip(sigmas, taus):
        if s == 0.0:
            rng.standard_normal(n + 1)          # keep the stream layout independent of weights
            continue
        a = np.exp(-dt / tau)
        x0 = s * rng.standard_normal()
        w = rng.standard_normal(n)
        out += lfilter([s * np.sqrt(1.0 - a * a)], [1.0, -a], w, zi=[a * x0])[0]
    return out


def bi_components(N: float, B: float, prof: ImuProfile) -> tuple[tuple, tuple]:
    """(sigmas, taus) of the bias-instability model for one axis."""
    if prof.bi_model == "gm3":
        taus = tuple(float(x) for x in prof.bi_taus_s.v)
        return fit_gm_sigmas(float(N), float(B), taus, tuple(prof.bi_fit_range_s.v)), taus
    if prof.bi_model == "gm1":
        return (GM1_SIGMA_PER_B * float(B),), (float(prof.bi_gm1_tau_s.v),)
    raise ValueError(f"unknown bi_model {prof.bi_model!r}")


def bias_instability(rng, n: int, dt: float, N, B, prof: ImuProfile) -> np.ndarray:
    """(n, 3) bias-instability process, axes drawn one after another from one stream."""
    out = np.empty((n, 3))
    for i in range(3):
        sig, taus = bi_components(N[i], B[i], prof)
        out[:, i] = gm_sum(rng, n, dt, sig, taus)
    return out


def bandlimited(rng, n: int, fs: float, band, order: int, ncols: int) -> np.ndarray:
    """Unit-rms band-pass Gaussian noise, (n, ncols). A warm-up removes the filter transient."""
    sos = butter(int(order), band, btype="bandpass", fs=fs, output="sos")
    warm = int(np.ceil(10.0 * fs / band[0]))
    x = sosfilt(sos, rng.standard_normal((n + warm, ncols)), axis=0)[warm:]
    imp = np.zeros(int(50 * fs / band[0]))
    imp[0] = 1.0
    gain = np.sqrt(np.sum(sosfilt(sos, imp) ** 2))
    return x / gain


def _scale(rng, sigma: float) -> np.ndarray:
    return np.diag(rng.standard_normal(3) * sigma)


def _misalign(rng, sigma: float) -> np.ndarray:
    M = rng.standard_normal((3, 3)) * sigma
    np.fill_diagonal(M, 0.0)
    return M


# ---------------------------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class ImuTruthSource:
    """What the IMU generator needs from the truth: ideal phone signals, speed, mount."""
    t: np.ndarray        # (M,)
    f_p: np.ndarray      # (M,3)
    w_p: np.ndarray      # (M,3)
    v: np.ndarray        # (M,)  speed, scales road vibration
    C_pb: np.ndarray     # (3,3) body -> phone (constant mount)

    @classmethod
    def from_truth(cls, tr) -> "ImuTruthSource":
        return cls(t=tr.t, f_p=tr.f_p, w_p=tr.w_p, v=tr.v, C_pb=tr.C_bp.T)

    @classmethod
    def static(cls, duration_s: float, rate_hz: float = 100.0,
               mount_rpy_deg=(1.0, 12.0, 3.0)) -> "ImuTruthSource":
        """Level, stationary car (engine running), phone in the default mount."""
        t = np.arange(int(duration_s * rate_hz) + 1) / rate_hz
        C_pb = mount_rotation(*mount_rpy_deg).T
        f_p = np.tile(C_pb @ np.array([0.0, 0.0, G0]), (t.size, 1))
        return cls(t=t, f_p=f_p, w_p=np.zeros((t.size, 3)), v=np.zeros(t.size), C_pb=C_pb)


@dataclass
class ImuErrors:
    """Everything the generator did, for analysis. Never passed to an estimator."""
    t_true: np.ndarray                  # (N,) true sample times
    t_reported: np.ndarray              # (N,) = ImuMeas.t
    f_true: np.ndarray                  # (N,3) ideal specific force at t_true, phone frame
    w_true: np.ndarray                  # (N,3)
    components: dict = field(default_factory=dict)   # name -> (N,3) additive or vibration term
    constants: dict = field(default_factory=dict)    # name -> realised constant (3,) or (3,3)
    bi_model: dict = field(default_factory=dict)     # per sensor: list of (sigmas, taus) per axis
    n_grid: int = 0                     # samples before gaps
    kept: np.ndarray | None = None      # (n_grid,) bool


def simulate_imu(src: ImuTruthSource, cfg: SensorConfig = NOMINAL,
                 switches: Switches = Switches(), seed: int = 0) -> tuple[ImuMeas, ImuErrors]:
    prof, vib, tim = cfg.imu, cfg.vib, cfg.timing
    on = switches
    fs = float(prof.rate_hz.v) * (1.0 + float(tim.rate_offset.v))
    dt = 1.0 / fs
    t0, t1 = float(src.t[0]), float(src.t[-1])
    n = int(np.floor((t1 - t0) / dt)) + 1

    # sample instants (true) and dropped samples; all terms are drawn on the full grid
    t_true = t0 + np.arange(n) * dt
    if on("ts_jitter"):
        sj = float(tim.jitter_sigma_s.v)
        j = np.clip(term_rng(seed, "ts_jitter").standard_normal(n) * sj, -3 * sj, 3 * sj)
        t_true = np.clip(t_true + j, t0, t1)
    kept = np.ones(n, dtype=bool)
    if on("ts_gaps"):
        r = term_rng(seed, "ts_gaps")
        n_ev = r.poisson(float(tim.gap_rate_hz.v) * (t1 - t0))
        for s in r.integers(0, n, n_ev):
            kept[s:s + int(tim.gap_len_samples.v)] = False

    # ideal signals at the true sample instants
    cs = CubicSpline(src.t, np.column_stack([src.f_p, src.w_p, src.v]), axis=0)
    x = cs(t_true)
    f_true, w_true, v = x[:, 0:3], x[:, 3:6], np.maximum(x[:, 6], 0.0)

    comps: dict[str, np.ndarray] = {}
    consts: dict[str, np.ndarray] = {}
    f_in, w_in = f_true.copy(), w_true.copy()

    # vibration: body frame -> phone frame, added before the sensor errors (R1)
    lvl = float(vib.level.v)
    order = int(vib.filter_order.v)
    if on("vib_road"):
        u = bandlimited(term_rng(seed, "vib_road"), n, fs, vib.road_band_hz.v, order, 6)
        k = (lvl * v / float(vib.road_v_ref_mps.v))[:, None]
        va = k * u[:, :3] * vib.road_accel_rms_b.a @ src.C_pb.T
        vg = k * u[:, 3:] * vib.road_rate_rms_b.a @ src.C_pb.T
        comps["vib_road_a"], comps["vib_road_g"] = va, vg
        f_in += va
        w_in += vg
    if on("vib_engine"):
        fc, bw = float(vib.engine_freq_hz.v), float(vib.engine_bw_hz.v)
        u = bandlimited(term_rng(seed, "vib_engine"), n, fs, (fc - bw / 2, fc + bw / 2), order, 6)
        va = lvl * u[:, :3] * vib.engine_accel_rms_b.a @ src.C_pb.T
        vg = lvl * u[:, 3:] * vib.engine_rate_rms_b.a @ src.C_pb.T
        comps["vib_engine_a"], comps["vib_engine_g"] = va, vg
        f_in += va
        w_in += vg

    # scale factor and axis misalignment
    I3 = np.eye(3)
    S_a = _scale(term_rng(seed, "a_scale"), float(prof.accel_scale_sigma.v)) if on("a_scale") else 0 * I3
    S_g = _scale(term_rng(seed, "g_scale"), float(prof.gyro_scale_sigma.v)) if on("g_scale") else 0 * I3
    M_a = _misalign(term_rng(seed, "a_misalign"), float(prof.misalign_sigma.v)) if on("a_misalign") else 0 * I3
    M_g = _misalign(term_rng(seed, "g_misalign"), float(prof.misalign_sigma.v)) if on("g_misalign") else 0 * I3
    consts.update(S_a=S_a, S_g=S_g, M_a=M_a, M_g=M_g)
    f = f_in @ (I3 + S_a + M_a).T
    w = w_in @ (I3 + S_g + M_g).T

    # biases and white noise
    if on("a_turnon"):
        consts["b_a_on"] = term_rng(seed, "a_turnon").standard_normal(3) * float(prof.accel_turn_on_sigma.v)
        f += consts["b_a_on"]
    if on("g_turnon"):
        consts["b_g_on"] = term_rng(seed, "g_turnon").standard_normal(3) * float(prof.gyro_turn_on_sigma.v)
        w += consts["b_g_on"]
    bi_info = {}
    for key, term, N, B, target in (("a", "a_bi", prof.accel_vrw.a, prof.accel_bias_instab.a, f),
                                    ("g", "g_bi", prof.gyro_arw.a, prof.gyro_bias_instab.a, w)):
        bi_info[key] = [bi_components(N[i], B[i], prof) for i in range(3)]
        if on(term):
            comps[term] = bias_instability(term_rng(seed, term), n, dt, N, B, prof)
            target += comps[term]
    if on("a_white"):
        comps["a_white"] = white(term_rng(seed, "a_white"), n, dt, prof.accel_vrw.a)
        f += comps["a_white"]
    if on("g_white"):
        comps["g_white"] = white(term_rng(seed, "g_white"), n, dt, prof.gyro_arw.a)
        w += comps["g_white"]

    t_rep = t0 + (t_true - t0) * (1.0 + float(tim.ts_scale_error.v))

    meas = ImuMeas(t=t_rep[kept], f=f[kept], w=w[kept])
    err = ImuErrors(t_true=t_true[kept], t_reported=t_rep[kept], f_true=f_true[kept],
                    w_true=w_true[kept], components={k_: c[kept] for k_, c in comps.items()},
                    constants=consts, bi_model=bi_info, n_grid=n, kept=kept)
    return meas, err


# ---------------------------------------------------------------------------------------------
# Barometer (K7)
#
#   h~ = h + b_h0 + b_h,GM(t) + n_h + d_cabin(t) + d_tunnel(s)
#
#   d_tunnel = -(k_p / rho_g) * (s - s_entry) inside the tunnel: a pressure gradient along travel
#   (ventilation) read as altitude, zero at the entry portal. After the exit it decays over
#   distance, d(s_exit) * exp(-(s - s_exit) / L_p) (review 2, 4.2); L_p = 0 gives the step
#   variant (stress case). L_p is a sensitivity parameter, not a validated physical value.
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class BaroTruthSource:
    t: np.ndarray        # (M,)
    h: np.ndarray        # (M,) true phone altitude (ENU up)
    s: np.ndarray        # (M,) along-track position of the reference point
    s_entry: float
    s_exit: float

    @classmethod
    def from_truth(cls, tr, sc) -> "BaroTruthSource":
        return cls(t=tr.t, h=tr.pos_phone_n[:, 2], s=tr.s,
                   s_entry=float(sc.road.tunnel.s_entry), s_exit=float(sc.road.tunnel.s_exit))


@dataclass
class BaroErrors:
    t: np.ndarray
    h_true: np.ndarray
    components: dict = field(default_factory=dict)   # name -> (N,)
    constants: dict = field(default_factory=dict)


def tunnel_pressure_altitude(s, s_entry, s_exit, grad_pa_per_km, rho_g_pa_per_m, exit_decay_m=0.0):
    """d_tunnel(s) [m]: pressure rising along travel reads as falling altitude."""
    s = np.asarray(s, dtype=float)
    k = -(grad_pa_per_km / 1000.0) / rho_g_pa_per_m
    inside = (s >= s_entry) & (s <= s_exit)
    d = np.where(inside, k * (s - s_entry), 0.0)
    if exit_decay_m > 0.0:
        after = s > s_exit
        d = np.where(after, k * (s_exit - s_entry) * np.exp(-(s - s_exit) / exit_decay_m), d)
    return d


def simulate_baro(src: BaroTruthSource, cfg: SensorConfig = NOMINAL,
                  switches: Switches = Switches(), seed: int = 0) -> tuple[BaroMeas, BaroErrors]:
    p, on = cfg.baro, switches
    fs = float(p.rate_hz.v)
    dt = 1.0 / fs
    t0, t1 = float(src.t[0]), float(src.t[-1])
    n = int(np.floor((t1 - t0) / dt)) + 1
    t = t0 + np.arange(n) * dt
    h_true = CubicSpline(src.t, src.h)(t)
    s = np.interp(t, src.t, src.s)

    comps, consts = {}, {}
    if on("b_white"):
        comps["b_white"] = white(term_rng(seed, "b_white"), n, dt, float(p.white_sigma_m.v) * np.sqrt(dt))[:, 0]
    if on("b_gm"):
        comps["b_gm"] = gm_sum(term_rng(seed, "b_gm"), n, dt, (float(p.gm_sigma_m.v),), (float(p.gm_tau_s.v),))
    if on("b_offset"):
        consts["b_h0"] = float(term_rng(seed, "b_offset").standard_normal() * float(p.offset_sigma_m.v))
        comps["b_offset"] = np.full(n, consts["b_h0"])
    if on("b_cabin"):
        comps["b_cabin"] = gm_sum(term_rng(seed, "b_cabin"), n, dt,
                                  (float(p.cabin_sigma_m.v),), (float(p.cabin_tau_s.v),))
    if on("b_tunnel"):
        comps["b_tunnel"] = tunnel_pressure_altitude(s, src.s_entry, src.s_exit,
                                                     float(p.tunnel_grad_pa_per_km.v),
                                                     float(p.rho_g_pa_per_m.v),
                                                     float(p.tunnel_exit_decay_m.v))
    h = h_true + sum(comps.values(), np.zeros(n))
    return BaroMeas(t=t, h=h), BaroErrors(t=t, h_true=h_true, components=comps, constants=consts)


# ---------------------------------------------------------------------------------------------
# GNSS (K8, K9)
#
# Open sky:  pos~ = p + e_p,   e_p GM per axis (sigma_h, sigma_h, sigma_v; tau_p)
#            vel~ = v + n_v + b_v,GM   (white + GM per axis, vertical x vel_vert_factor)
# Entry:     sigmas ramp linearly from 1 to degrade_factor over degrade_s before the portal;
#            at the portal the fixes stop (nominal) or the receiver coasts: fake fixes
#            propagated from its own last fix (position + velocity * dt), reported accuracy
#            growing, for coast_s seconds.
# Exit:      velocity from vel_reacq_s, position from pos_reacq_s after the exit portal.
#            Exit bias on position, decaying with exit_decay_s from the first position fix:
#              "multipath": random horizontal direction, magnitude exit_multipath_m
#              "lagging":   -v_true * lag_s (receiver output behind along track)
#            Velocity noise is multiplied by exit_vel_factor at the first velocity fix, same decay.
# Reported:  reported_ratio * model sigma (incl. degradation), independent of the realised
#            error; the exit bias is not reported (receivers underestimate it).
# Time:      the fix stamped t describes the antenna at t - imu_gnss_offset_s.
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class GnssTruthSource:
    t: np.ndarray        # (M,)
    pos: np.ndarray      # (M,3) phone antenna, ENU
    vel: np.ndarray      # (M,3)
    s: np.ndarray        # (M,) along-track position of the reference point
    s_entry: float
    s_exit: float

    @classmethod
    def from_truth(cls, tr, sc) -> "GnssTruthSource":
        return cls(t=tr.t, pos=tr.pos_phone_n, vel=tr.vel_phone_n, s=tr.s,
                   s_entry=float(sc.road.tunnel.s_entry), s_exit=float(sc.road.tunnel.s_exit))


@dataclass
class GnssErrors:
    t: np.ndarray                 # (N,) stamped fix times
    t_true: np.ndarray            # (N,) times the fixes describe (t - offset)
    pos_true: np.ndarray          # (N,3)
    vel_true: np.ndarray          # (N,3)
    components: dict = field(default_factory=dict)
    events: dict = field(default_factory=dict)     # t_entry, t_exit, t_vel0, t_pos0, ...
    masks: dict = field(default_factory=dict)      # pos_valid, vel_valid, coasting


def _crossing_time(t, s, s_target):
    i = int(np.searchsorted(s, s_target))
    if i == 0:
        return float(t[0])
    if i >= len(s):
        return np.inf
    return float(np.interp(s_target, s[i - 1:i + 1], t[i - 1:i + 1]))


def simulate_gnss(src: GnssTruthSource, cfg: SensorConfig = NOMINAL,
                  switches: Switches = Switches(), seed: int = 0) -> tuple[GnssMeas, GnssErrors]:
    g, on = cfg.gnss, switches
    off = float(cfg.timing.imu_gnss_offset_s.v)
    dt = 1.0 / float(g.rate_hz.v)
    t0, t1 = float(src.t[0]), float(src.t[-1])
    t = t0 + off + np.arange(int(np.floor((t1 - t0 - off) / dt)) + 1) * dt   # stamped times
    tm = t - off                                                           # described times
    n = t.size
    cs = CubicSpline(src.t, np.column_stack([src.pos, src.vel]), axis=0)
    x = cs(tm)
    p_true, v_true = x[:, :3], x[:, 3:]

    t_entry = _crossing_time(src.t, src.s, src.s_entry)
    t_exit = _crossing_time(src.t, src.s, src.s_exit)

    # sigma multiplier before the portal (only before it; after the exit the exit model applies)
    before = tm < t_entry
    fac = np.ones(n)
    if on("gnss_degrade") and float(g.degrade_s.v) > 0:
        u = np.clip((tm - (t_entry - float(g.degrade_s.v))) / float(g.degrade_s.v), 0.0, 1.0)
        fac = np.where(before, 1.0 + (float(g.degrade_factor.v) - 1.0) * u, 1.0)

    comps = {}
    sig_p = np.array([float(g.pos_sigma_h_m.v)] * 2 + [float(g.pos_sigma_v_m.v)])
    vf = np.array([1.0, 1.0, float(g.vel_vert_factor.v)])
    e_p = np.zeros((n, 3))
    if on("gnss_pos"):
        r = term_rng(seed, "gnss_pos")
        e_p = np.column_stack([gm_sum(r, n, dt, (1.0,), (float(g.pos_tau_s.v),)) for _ in range(3)]) * sig_p
    n_vw = np.zeros((n, 3))
    if on("gnss_vel_white"):
        n_vw = term_rng(seed, "gnss_vel_white").standard_normal((n, 3)) * float(g.vel_white_h_mps.v) * vf
    n_vg = np.zeros((n, 3))
    if on("gnss_vel_gm"):
        r = term_rng(seed, "gnss_vel_gm")
        n_vg = np.column_stack([gm_sum(r, n, dt, (1.0,), (float(g.vel_gm_tau_s.v),)) for _ in range(3)]) \
            * float(g.vel_gm_h_mps.v) * vf

    # availability
    t_vel0 = t_exit + float(g.vel_reacq_s.v)
    t_pos0 = t_exit + float(g.pos_reacq_s.v)
    vel_ok = before | (tm >= t_vel0)
    pos_ok = before | (tm >= t_pos0)
    if np.any(tm >= t_pos0):
        t_pos0 = float(tm[tm >= t_pos0][0])
    if np.any(tm >= t_vel0):
        t_vel0 = float(tm[tm >= t_vel0][0])

    # exit effects
    exit_bias = np.zeros((n, 3))
    vel_exit_fac = np.ones(n)
    tau_x = float(g.exit_decay_s.v)
    if on("gnss_exit") and np.isfinite(t_exit):
        after_p = tm >= t_pos0
        decay = np.exp(-(tm[after_p] - t_pos0) / tau_x)
        if g.exit_mode == "multipath":
            ang = term_rng(seed, "gnss_exit").uniform(0.0, 2 * np.pi)
            u = np.array([np.cos(ang), np.sin(ang), 0.0])
            exit_bias[after_p] = float(g.exit_multipath_m.v) * decay[:, None] * u
        elif g.exit_mode == "lagging":
            exit_bias[after_p] = -v_true[after_p] * float(g.lag_s.v) * decay[:, None]
        else:
            raise ValueError(f"unknown exit_mode {g.exit_mode!r}")
        after_v = tm >= t_vel0
        vel_exit_fac[after_v] = 1.0 + (float(g.exit_vel_factor.v) - 1.0) * np.exp(-(tm[after_v] - t_vel0) / tau_x)

    pos = p_true + fac[:, None] * e_p + exit_bias
    vel = v_true + (fac * vel_exit_fac)[:, None] * (n_vw + n_vg)
    ratio = float(g.reported_ratio.v)
    sig_v = float(np.hypot(float(g.vel_white_h_mps.v), float(g.vel_gm_h_mps.v)))
    acc_h = ratio * sig_p[0] * fac
    acc_v = ratio * sig_p[2] * fac
    acc_vel = ratio * sig_v * fac * vel_exit_fac

    # coasting: fake fixes from the receiver's own last fix (not from the truth)
    coasting = np.zeros(n, dtype=bool)
    coast_s = float(g.coast_s.v)
    if coast_s > 0 and np.any(before) and np.isfinite(t_entry):
        k = int(np.nonzero(before)[0][-1])
        coasting = (~before) & (tm < t_entry + coast_s) & (tm < t_exit)
        dtc = tm[coasting] - tm[k]
        pos[coasting] = pos[k] + vel[k] * dtc[:, None]
        vel[coasting] = vel[k]
        acc_h[coasting] = acc_h[k] + float(g.coast_acc_growth_mps.v) * dtc
        acc_v[coasting] = acc_v[k] + float(g.coast_acc_growth_mps.v) * dtc
        acc_vel[coasting] = acc_vel[k]
        pos_ok |= coasting
        vel_ok |= coasting

    keep = pos_ok | vel_ok
    pos = np.where(pos_ok[:, None], pos, np.nan)
    vel = np.where(vel_ok[:, None], vel, np.nan)
    acc_h, acc_v = np.where(pos_ok, acc_h, np.nan), np.where(pos_ok, acc_v, np.nan)
    acc_vel = np.where(vel_ok, acc_vel, np.nan)

    comps.update(pos_noise=fac[:, None] * e_p, vel_white=n_vw, vel_gm=n_vg, exit_bias=exit_bias,
                 sigma_factor=fac, vel_exit_factor=vel_exit_fac)
    meas = GnssMeas(t=t[keep], pos=pos[keep], vel=vel[keep], acc_pos_h=acc_h[keep],
                    acc_pos_v=acc_v[keep], acc_vel=acc_vel[keep])
    err = GnssErrors(t=t[keep], t_true=tm[keep], pos_true=p_true[keep], vel_true=v_true[keep],
                     components={k_: c[keep] for k_, c in comps.items()},
                     events={"t_entry": t_entry, "t_exit": t_exit, "t_vel0": t_vel0, "t_pos0": t_pos0},
                     masks={"pos_valid": pos_ok[keep], "vel_valid": vel_ok[keep], "coasting": coasting[keep]})
    return meas, err
