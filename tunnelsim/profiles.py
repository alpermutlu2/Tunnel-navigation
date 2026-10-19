"""Sensor error profiles.

Every number carries its origin:
  [MEAS]    measured value from a cited source
  [ASSUMED] our assumption, to be stated in the write-up and varied in sensitivity runs

Units are SI throughout: m, s, rad. Per-axis arrays are ordered (x, y, z) in the
sensor's own (phone) frame.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ImuProfile:
    name: str
    rate_hz: float

    # Allan deviation terms (white noise and bias instability)
    accel_vrw: np.ndarray          # m/s/sqrt(s)   (= m/s^2/sqrt(Hz))
    accel_bias_instab: np.ndarray  # m/s^2
    gyro_arw: np.ndarray           # rad/sqrt(s)   (= rad/s/sqrt(Hz))
    gyro_bias_instab: np.ndarray   # rad/s

    # Bias instability is flicker noise; we realise it as a first-order Gauss-Markov
    # process with this correlation time and steady-state sigma = bias_instab.
    bias_gm_tau_s: float

    # Terms not captured by a static, temperature-stable lab test
    accel_turn_on_sigma: float     # m/s^2, residual turn-on bias, random per run
    gyro_turn_on_sigma: float      # rad/s, residual after OS gyro calibration
    accel_temp_drift: float        # m/s^2 per minute, slow ramp (phone heating up)
    gyro_temp_drift: float         # rad/s per minute
    accel_scale_sigma: float       # dimensionless (e.g. 1e-3 = 1000 ppm)
    gyro_scale_sigma: float
    misalign_sigma_rad: float      # axis non-orthogonality
    timestamp_jitter_s: float      # std of sample time jitter

    source: str = ""


def _a(x, y, z) -> np.ndarray:
    return np.array([x, y, z], dtype=float)


MG = 9.80665e-3          # 1 milli-g in m/s^2
DEG = np.pi / 180.0

# Samsung Galaxy Note10, STMicroelectronics LSM6DSR.
# Allan terms [MEAS]: Suvorkin et al., Sensors 24(6):1953, 2024, Table 4
# (week-long static OADEV, temperature-stable room, foam pad).
NOTE10_LSM6DSR = ImuProfile(
    name="Note10 (LSM6DSR)",
    rate_hz=100.0,
    accel_vrw=_a(9.99e-4, 9.20e-4, 8.58e-4),
    accel_bias_instab=_a(6.63e-4, 6.79e-4, 1.62e-3),
    gyro_arw=_a(7.00e-4, 6.01e-4, 6.70e-4),
    gyro_bias_instab=_a(8.84e-5, 1.54e-4, 2.26e-4),
    bias_gm_tau_s=300.0,                 # [ASSUMED]
    accel_turn_on_sigma=10 * MG,         # [ASSUMED] order of a consumer zero-g offset
    gyro_turn_on_sigma=0.05 * DEG,       # [ASSUMED] residual after OS stationary calibration
    accel_temp_drift=0.05 * MG,          # [ASSUMED] per minute
    gyro_temp_drift=0.002 * DEG,         # [ASSUMED] per minute
    accel_scale_sigma=3e-3,              # [ASSUMED]
    gyro_scale_sigma=3e-3,               # [ASSUMED]
    misalign_sigma_rad=0.1 * DEG,        # [ASSUMED]
    timestamp_jitter_s=0.5e-3,           # [ASSUMED] Android pipeline (Bochkati & Pany 2021)
    source="Suvorkin et al. 2024, Table 4; added terms assumed",
)

# InvenSense TDK MPU9250, low-end profile.
# Allan terms [MEAS]: same paper and method, Table 5.
MPU9250_LOWEND = ImuProfile(
    name="Low-end (MPU9250)",
    rate_hz=100.0,
    accel_vrw=_a(9.61e-4, 9.62e-4, 1.60e-3),
    accel_bias_instab=_a(7.64e-4, 1.16e-3, 1.27e-3),
    gyro_arw=_a(6.07e-3, 5.64e-3, 6.25e-3),
    gyro_bias_instab=_a(4.45e-3, 6.87e-3, 3.04e-3),
    bias_gm_tau_s=300.0,                 # [ASSUMED]
    accel_turn_on_sigma=30 * MG,         # [ASSUMED]
    gyro_turn_on_sigma=0.2 * DEG,        # [ASSUMED]
    accel_temp_drift=0.2 * MG,           # [ASSUMED]
    gyro_temp_drift=0.01 * DEG,          # [ASSUMED]
    accel_scale_sigma=1e-2,              # [ASSUMED]
    gyro_scale_sigma=1e-2,               # [ASSUMED]
    misalign_sigma_rad=0.5 * DEG,        # [ASSUMED]
    timestamp_jitter_s=1.0e-3,           # [ASSUMED]
    source="Suvorkin et al. 2024, Table 5; added terms assumed",
)


@dataclass(frozen=True)
class BaroProfile:
    rate_hz: float = 25.0
    noise_sigma_m: float = 0.10          # [ASSUMED] white altitude noise after on-chip filtering
    drift_m_per_min: float = 0.02        # [ASSUMED] weather drift, negligible over minutes
    cabin_disturbance_m: float = 0.0     # [ASSUMED] off by default (A6); sensitivity case > 0
    cabin_disturbance_tau_s: float = 5.0


@dataclass(frozen=True)
class GnssProfile:
    rate_hz: float = 1.0
    pos_sigma_m: float = 3.0             # [ASSUMED] open-sky phone, horizontal per axis
    vert_sigma_m: float = 6.0            # [ASSUMED]
    pos_corr_tau_s: float = 30.0         # [ASSUMED] GNSS position error is time-correlated
    vel_sigma_mps: float = 0.10          # [ASSUMED] Doppler velocity per axis
    degrade_before_portal_s: float = 4.0 # [ASSUMED] SNR drop before full loss at entry
    exit_multipath_m: float = 30.0       # [ASSUMED] first-fix bias magnitude at exit (10..50 m)
    exit_multipath_decay_s: float = 8.0  # [ASSUMED]
    exit_reacq_delay_s: float = 2.0      # [ASSUMED] time to first fix after leaving the tunnel


IMU_PROFILES = {
    "note10": NOTE10_LSM6DSR,
    "lowend": MPU9250_LOWEND,
}
