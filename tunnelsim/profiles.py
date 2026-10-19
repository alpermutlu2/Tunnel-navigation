"""Sensor error profiles, nominal configuration and named sensitivity overrides.

Every number is a `Param` carrying its origin (DESIGN.md section 3):
  [MEAS]    measured in a real experiment, cited
  [SPEC]    datasheet / manufacturer specification
  [ASSUMED] chosen for the simulation, not validated; swept rather than pinned

Nominal and sensitivity are kept apart: `NOMINAL` is the only default; every sensitivity
case is a named entry in `OVERRIDES`, applied with `configure(*names)`. An override must
itself be a `Param`, so a swept value cannot lose its tag.

Units are SI throughout: m, s, rad. Per-axis tuples are (x, y, z) in the frame named by the
field (phone frame unless stated).

The GNSS profile is still the v0.2 placeholder; it is rebuilt (tags, K8, K9) in its own commit.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields, replace

import numpy as np

TAGS = ("MEAS", "SPEC", "ASSUMED")


@dataclass(frozen=True)
class Param:
    v: object          # float or tuple of floats
    tag: str
    src: str = ""

    def __post_init__(self):
        if self.tag not in TAGS:
            raise ValueError(f"unknown tag {self.tag!r}")

    @property
    def a(self) -> np.ndarray:
        return np.asarray(self.v, dtype=float)


def MEAS(v, src=""):
    return Param(v, "MEAS", src)


def SPEC(v, src=""):
    return Param(v, "SPEC", src)


def ASSUMED(v, src=""):
    return Param(v, "ASSUMED", src)


MG = 9.80665e-3          # 1 milli-g in m/s^2
DEG = np.pi / 180.0

_SUV4 = "Suvorkin et al. 2024, Table 4 (Note10, LSM6DSR)"
_SUV5 = "Suvorkin et al. 2024, Table 5 (MPU9250)"


# ---------------------------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class ImuProfile:
    name: str
    rate_hz: Param
    # Allan terms
    accel_vrw: Param               # m/s/sqrt(s)
    accel_bias_instab: Param       # m/s^2   (B)
    gyro_arw: Param                # rad/sqrt(s)
    gyro_bias_instab: Param        # rad/s   (B)
    # Bias-instability realisation (K1, R2, R3)
    bi_taus_s: Param               # GM correlation times of the baseline (gm3) model
    bi_fit_range_s: Param          # range over which white + GM sum matches the target
    bi_gm1_tau_s: Param            # reference single-GM model
    # Constant per run
    accel_turn_on_sigma: Param     # m/s^2, residual turn-on bias (1 sigma)
    gyro_turn_on_sigma: Param      # rad/s
    accel_scale_sigma: Param       # dimensionless
    gyro_scale_sigma: Param
    misalign_sigma: Param          # rad, sensor axis non-orthogonality (MEMS -> phone axes)
    bi_model: str = "gm3"          # "gm3" baseline, "gm1" reference
    source: str = ""


NOTE10_LSM6DSR = ImuProfile(
    name="Note10 (LSM6DSR)",
    rate_hz=MEAS(100.0, "Suvorkin et al. 2024, Table 1 (from output timestamps)"),
    accel_vrw=MEAS((9.99e-4, 9.20e-4, 8.58e-4), _SUV4),
    accel_bias_instab=MEAS((6.63e-4, 6.79e-4, 1.62e-3), _SUV4),
    gyro_arw=MEAS((7.00e-4, 6.01e-4, 6.70e-4), _SUV4),
    gyro_bias_instab=MEAS((8.84e-5, 1.54e-4, 2.26e-4), _SUV4),
    bi_taus_s=ASSUMED((5.0, 50.0, 500.0), "R3: derived from the target range (tau ~ T/2)"),
    bi_fit_range_s=ASSUMED((10.0, 1000.0), "R3: outage horizon with margin"),
    bi_gm1_tau_s=ASSUMED(300.0, "K1 reference model"),
    accel_turn_on_sigma=SPEC(10 * MG, "LSM6DSR datasheet LA_TyOff +-10 mg, used as 1 sigma"),
    gyro_turn_on_sigma=ASSUMED(0.05 * DEG, "K2 nominal, uncalibrated stream residual; "
                                           "SPEC raw G_TyOff +-1 dps is an upper bound"),
    accel_scale_sigma=ASSUMED(3e-3, "K4"),
    gyro_scale_sigma=ASSUMED(3e-3, "K4 nominal 0.3 %; SPEC G_So% +-1 % is the bound"),
    misalign_sigma=ASSUMED(0.1 * DEG, "K4"),
    source="Allan terms Suvorkin et al. 2024 Table 4; bounds LSM6DSR datasheet Table 2",
)

MPU9250_LOWEND = ImuProfile(
    name="Low-end (MPU9250)",
    rate_hz=MEAS(100.0, "Suvorkin et al. 2024, Table 1"),
    accel_vrw=MEAS((9.61e-4, 9.62e-4, 1.60e-3), _SUV5),
    accel_bias_instab=MEAS((7.64e-4, 1.16e-3, 1.27e-3), _SUV5),
    gyro_arw=MEAS((6.07e-3, 5.64e-3, 6.25e-3), _SUV5),
    gyro_bias_instab=MEAS((4.45e-3, 6.87e-3, 3.04e-3), _SUV5),
    bi_taus_s=ASSUMED((5.0, 50.0, 500.0), "R3"),
    bi_fit_range_s=ASSUMED((10.0, 1000.0), "R3"),
    bi_gm1_tau_s=ASSUMED(300.0, "K1 reference model"),
    accel_turn_on_sigma=ASSUMED(30 * MG, "datasheet not checked"),
    gyro_turn_on_sigma=ASSUMED(0.2 * DEG, "datasheet not checked"),
    accel_scale_sigma=ASSUMED(1e-2, ""),
    gyro_scale_sigma=ASSUMED(1e-2, ""),
    misalign_sigma=ASSUMED(0.5 * DEG, ""),
    source="Allan terms Suvorkin et al. 2024 Table 5; other terms assumed",
)


# ---------------------------------------------------------------------------------------------
# Vibration (K5, R1). Generated in the body frame, rotated into the phone frame by the mount.
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class VibrationProfile:
    level: Param                   # overall scale: 1 = "calm" (nominal), 3 = "noisy"
    road_band_hz: Param            # band-pass of the road component
    road_v_ref_mps: Param          # speed at which the road rms values apply (rms ~ v / v_ref)
    road_accel_rms_b: Param        # m/s^2 at v_ref, body FLU
    road_rate_rms_b: Param         # rad/s at v_ref, body (roll, pitch, yaw)
    engine_freq_hz: Param          # centre of the narrow engine band (idle firing frequency)
    engine_bw_hz: Param
    engine_accel_rms_b: Param      # m/s^2, constant while the engine runs
    engine_rate_rms_b: Param       # rad/s
    filter_order: Param            # Butterworth band-pass order (band shape)


VIB_CALM = VibrationProfile(
    level=ASSUMED(1.0, "K5 'calm'"),
    road_band_hz=ASSUMED((2.0, 30.0), "K5"),
    road_v_ref_mps=ASSUMED(25.0, "K5, rms linear in speed"),
    road_accel_rms_b=ASSUMED((0.05, 0.05, 0.15), "K5"),
    road_rate_rms_b=ASSUMED((0.005, 0.005, 0.002), "K5"),
    engine_freq_hz=ASSUMED(25.0, "K5: 4-cyl idle ~750 rpm, 2 firings/rev"),
    engine_bw_hz=ASSUMED(2.0, "K5"),
    engine_accel_rms_b=ASSUMED((0.02, 0.02, 0.05), "K5"),
    engine_rate_rms_b=ASSUMED((0.001, 0.001, 0.0005), "K5"),
    filter_order=ASSUMED(4, "band shape"),
)


# ---------------------------------------------------------------------------------------------
# Timing (K6). Timestamps carry the true measurement time unless ts_scale_error != 0.
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class TimingProfile:
    rate_offset: Param             # actual rate = nominal * (1 + rate_offset)   (K6 case 1)
    jitter_sigma_s: Param          # sampling-instant jitter, timestamps follow it
    gap_rate_hz: Param             # mean rate of dropped-sample events
    gap_len_samples: Param         # samples lost per event
    ts_scale_error: Param          # reported = t0 + (t - t0)(1 + e)   (K6 case 2)
    imu_gnss_offset_s: Param       # used by the GNSS generator


TIMING = TimingProfile(
    rate_offset=ASSUMED(0.0, "K6 case 1: nominal 0, sweep +-1..2 %"),
    jitter_sigma_s=ASSUMED(0.5e-3, "K6; Android pipeline irregularity, Bochkati & Pany 2021"),
    gap_rate_hz=ASSUMED(1.0 / 120.0, "K6 occasional gaps"),
    gap_len_samples=ASSUMED(3, "K6"),
    ts_scale_error=ASSUMED(0.0, "K6 case 2: nominal 0, sweep 1e-2"),
    imu_gnss_offset_s=ASSUMED(0.0, "K6: nominal 0, sweep 0..0.2 s"),
)


# ---------------------------------------------------------------------------------------------
# Barometer (K7): h~ = h + b_h0 + b_h,GM(t) + n_h + d_cabin(t) + d_tunnel(s), altitude units.
# The Note10 barometer part could not be confirmed; LPS22HB (ST) is quoted as a representative
# phone barometer only, so values derived from it stay [ASSUMED].
# ---------------------------------------------------------------------------------------------
_LPS22HB = "LPS22HB datasheet Table 3 (representative phone part; Note10 part not confirmed)"


@dataclass(frozen=True)
class BaroProfile:
    rate_hz: Param
    white_sigma_m: Param           # n_h, per sample
    gm_sigma_m: Param              # b_h,GM steady-state std
    gm_tau_s: Param
    offset_sigma_m: Param          # b_h0, constant within a drive
    cabin_sigma_m: Param           # d_cabin, GM; 0 = off (A6 baseline)
    cabin_tau_s: Param
    tunnel_grad_pa_per_km: Param   # d_tunnel: pressure gradient along travel inside the tunnel
    rho_g_pa_per_m: Param          # pressure-altitude conversion


BARO = BaroProfile(
    rate_hz=ASSUMED(25.0, "K7; LPS22HB offers 25 Hz ODR"),
    white_sigma_m=ASSUMED(0.10, "K7; cf. 0.0075 hPa rms = 0.064 m, " + _LPS22HB),
    gm_sigma_m=ASSUMED(0.10, "K7 nominal = mildest correlated row; white-only is an override"),
    gm_tau_s=ASSUMED(60.0, "K7 nominal"),
    offset_sigma_m=ASSUMED(0.85, "K7; +-0.1 hPa relative accuracy, " + _LPS22HB),
    cabin_sigma_m=ASSUMED(0.0, "A6 baseline: off"),
    cabin_tau_s=ASSUMED(5.0, "A6 sensitivity"),
    tunnel_grad_pa_per_km=ASSUMED(0.0, "K7: nominal off, swept; magnitude not sourced yet"),
    rho_g_pa_per_m=ASSUMED(11.8, "standard air rho = 1.2 kg/m^3"),
)


# ---------------------------------------------------------------------------------------------
# GNSS: v0.2 placeholder (plain floats, all assumed), rebuilt in its own commit.
# ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class GnssProfile:
    rate_hz: float = 1.0
    pos_sigma_m: float = 3.0
    vert_sigma_m: float = 6.0
    pos_corr_tau_s: float = 30.0
    vel_sigma_mps: float = 0.10
    degrade_before_portal_s: float = 4.0
    exit_multipath_m: float = 30.0
    exit_multipath_decay_s: float = 8.0
    exit_reacq_delay_s: float = 2.0


# ---------------------------------------------------------------------------------------------
# Nominal configuration and named overrides
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class SensorConfig:
    imu: ImuProfile
    vib: VibrationProfile
    timing: TimingProfile
    baro: BaroProfile
    gnss: GnssProfile = field(default_factory=GnssProfile)


NOMINAL = SensorConfig(imu=NOTE10_LSM6DSR, vib=VIB_CALM, timing=TIMING, baro=BARO)

OVERRIDES: dict[str, dict[str, object]] = {
    "imu_lowend":        {"imu": MPU9250_LOWEND},
    "imu_bi_gm1":        {"imu.bi_model": "gm1"},
    "gyro_turnon_0p2":   {"imu.gyro_turn_on_sigma": ASSUMED(0.2 * DEG, "K2 sweep")},
    "gyro_turnon_1":     {"imu.gyro_turn_on_sigma": ASSUMED(1.0 * DEG, "K2 sweep (SPEC bound)")},
    "gyro_scale_1pct":   {"imu.gyro_scale_sigma": SPEC(1e-2, "LSM6DSR G_So% bound")},
    "vib_noisy":         {"vib.level": ASSUMED(3.0, "K5 'noisy'")},
    "rate_offset_+1pct": {"timing.rate_offset": ASSUMED(0.01, "K6 case 1 sweep")},
    "rate_offset_+2pct": {"timing.rate_offset": ASSUMED(0.02, "K6 case 1 sweep")},
    "rate_offset_-2pct": {"timing.rate_offset": ASSUMED(-0.02, "K6 case 1 sweep")},
    "ts_scale_1pct":     {"timing.ts_scale_error": ASSUMED(0.01, "K6 case 2")},
    "no_gaps":           {"timing.gap_rate_hz": ASSUMED(0.0, "isolation")},
    # barometer (K7): GM rows of the grade-floor table, cabin (A6), tunnel gradient
    "baro_white_only":   {"baro.gm_sigma_m": ASSUMED(0.0, "K7 table row 1")},
    "baro_gm_0p3_300":   {"baro.gm_sigma_m": ASSUMED(0.3, "K7 table row 3"),
                          "baro.gm_tau_s": ASSUMED(300.0, "K7 table row 3")},
    "baro_gm_0p3_60":    {"baro.gm_sigma_m": ASSUMED(0.3, "K7 table row 4")},
    "baro_cabin":        {"baro.cabin_sigma_m": ASSUMED(0.3, "A6 sensitivity")},
    "baro_tunnel_10pa":  {"baro.tunnel_grad_pa_per_km": ASSUMED(10.0, "K7 sweep, illustrative")},
    "baro_tunnel_30pa":  {"baro.tunnel_grad_pa_per_km": ASSUMED(30.0, "K7 sweep, illustrative")},
}


def configure(*names: str, base: SensorConfig = NOMINAL) -> SensorConfig:
    """Apply named overrides, in order, to a copy of `base`."""
    cfg = base
    for name in names:
        if name not in OVERRIDES:
            raise KeyError(f"unknown override {name!r}; known: {sorted(OVERRIDES)}")
        for path, value in OVERRIDES[name].items():
            cfg = _set(cfg, path.split("."), value)
    return cfg


def _set(obj, path, value):
    head, rest = path[0], path[1:]
    if not hasattr(obj, head):
        raise KeyError(f"{type(obj).__name__} has no field {head!r}")
    if rest:
        return replace(obj, **{head: _set(getattr(obj, head), rest, value)})
    old = getattr(obj, head)
    if isinstance(old, Param) and not isinstance(value, Param):
        raise TypeError(f"override of {head!r} must be a tagged Param")
    return replace(obj, **{head: value})


def param_table(obj, prefix: str = "") -> dict[str, dict]:
    """Flat {path: {value, tag, src}} of every Param in a profile (for reports)."""
    out = {}
    for f in fields(obj):
        val = getattr(obj, f.name)
        if isinstance(val, Param):
            v = val.v if not isinstance(val.v, tuple) else list(val.v)
            out[prefix + f.name] = {"value": v, "tag": val.tag, "src": val.src}
    return out
