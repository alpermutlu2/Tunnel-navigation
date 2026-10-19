"""Measured data types: the only sensor objects an estimator may see (DESIGN.md 7.2).

This module must not import anything from the truth side (truth, scenario, sensors).
A test checks that.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ImuMeas:
    """Phone IMU stream as an app receives it (uncalibrated accel and gyro)."""
    t: np.ndarray   # (N,)   reported timestamps [s]
    f: np.ndarray   # (N,3)  specific force, phone frame [m/s^2]
    w: np.ndarray   # (N,3)  angular rate, phone frame [rad/s]


@dataclass(frozen=True)
class BaroMeas:
    """Barometric altitude (pressure already converted by the app), arbitrary datum."""
    t: np.ndarray   # (N,) [s]
    h: np.ndarray   # (N,) [m]


@dataclass(frozen=True)
class GnssMeas:
    """GNSS fixes of the phone antenna in local ENU (flat earth). NaN where a quantity is not
    available (e.g. velocity re-acquired before position at the tunnel exit). Accuracies are
    the receiver's reported 1-sigma per axis, not the true error."""
    t: np.ndarray          # (N,) [s]
    pos: np.ndarray        # (N,3) [m]
    vel: np.ndarray        # (N,3) [m/s]
    acc_pos_h: np.ndarray  # (N,) [m]
    acc_pos_v: np.ndarray  # (N,) [m]
    acc_vel: np.ndarray    # (N,) [m/s], horizontal axes
