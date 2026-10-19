"""Strapdown INS mechanisation (v0.4 step 2; docs/decisions/2026-10-19-v0.4-plan.md section 1).

Flat earth, ENU, no earth rotation (same assumptions as the truth). Phone point P.

    C_k+1 = C_k Exp([w_bar_k]x dt_k),   w_bar_k = (w_k + w_k+1) / 2
    a_k   = C_k f_k + g_n                (C_k+1 first, then a_k+1)
    v_k+1 = v_k + (a_k + a_k+1)/2 dt_k
    p_k+1 = p_k + (v_k + v_k+1)/2 dt_k

dt_k comes from the reported timestamps. Midpoint rate integration ignores coning; the
error-free test checks the error falls at second order with dt.

Separation (docs/contracts.md 6): this module imports only numpy, `frames` and `meas`. It never
knows where its inputs come from: an "ideal attitude" is just an attitude sequence passed in,
calibration values are parameters, and corrections are applied here, not by the caller.

Batch form: leading dimension B (e.g. Monte Carlo seeds) on every per-run array; all runs in a
batch must have the same number of samples.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frames import GRAVITY_N
from .meas import ImuMeas


@dataclass(frozen=True)
class InsInit:
    """Initial state at the first INS sample (phone point P)."""
    p_n: np.ndarray        # (3,) or (B,3)
    v_n: np.ndarray        # (3,) or (B,3)
    C_np: np.ndarray       # (3,3) or (B,3,3)


@dataclass(frozen=True)
class ImuCalibration:
    """Calibration the INS applies: f = K_a^-1 (f~ - b_a), w = K_g^-1 (w~ - b_g),
    K = I + S + M. Defaults: no correction."""
    accel_bias: np.ndarray | None = None     # (3,) or (B,3)
    gyro_bias: np.ndarray | None = None
    accel_matrix: np.ndarray | None = None   # (3,3) or (B,3,3) = I + S + M
    gyro_matrix: np.ndarray | None = None


@dataclass
class InsResult:
    t: np.ndarray          # (B,N) reported times
    p_n: np.ndarray        # (B,N,3)
    v_n: np.ndarray        # (B,N,3)
    C_np: np.ndarray       # (B,N,3,3)


def so3_exp_batch(phi: np.ndarray) -> np.ndarray:
    """Rodrigues for (B,3) rotation vectors -> (B,3,3)."""
    ang = np.linalg.norm(phi, axis=-1)
    x, y, z = phi[:, 0], phi[:, 1], phi[:, 2]
    zero = np.zeros_like(x)
    K = np.stack([np.stack([zero, -z, y], -1), np.stack([z, zero, -x], -1),
                  np.stack([-y, x, zero], -1)], -2)
    small = ang < 1e-8
    a2 = np.where(small, 1.0, ang * ang)
    A = np.where(small, 1.0 - ang**2 / 6.0, np.sin(ang) / np.where(small, 1.0, ang))
    Bc = np.where(small, 0.5 - ang**2 / 24.0, (1.0 - np.cos(ang)) / a2)
    return np.eye(3) + A[:, None, None] * K + Bc[:, None, None] * (K @ K)


def _apply_cal(x, bias, matrix):
    if bias is not None:
        x = x - np.asarray(bias, dtype=float)[..., None, :]
    if matrix is not None:
        Minv = np.linalg.inv(np.asarray(matrix, dtype=float))
        if Minv.ndim == 2:
            x = x @ Minv.T
        else:
            x = np.einsum("bij,bnj->bni", Minv, x)
    return x


def _batch(a, nd):
    a = np.asarray(a, dtype=float)
    return a[None] if a.ndim == nd else a


def strapdown(t, f, w, init: InsInit, cal: ImuCalibration = ImuCalibration(),
              attitude=None) -> InsResult:
    """Mechanise IMU samples.

    t: (N,) or (B,N) reported times; f, w: (N,3) or (B,N,3).
    attitude: optional (N,3,3) or (B,N,3,3) phone attitude sequence; if given it replaces the
    gyro integration (ideal-attitude experiments) and w is ignored.
    """
    t, f, w = _batch(t, 1), _batch(f, 2), _batch(w, 2)
    B, N = max(t.shape[0], f.shape[0]), t.shape[1]
    t = np.broadcast_to(t, (B, N))                 # a shared time axis is allowed
    w = np.broadcast_to(w, (B, N, 3))
    f = _apply_cal(f, cal.accel_bias, cal.accel_matrix)
    w = _apply_cal(w, cal.gyro_bias, cal.gyro_matrix)
    p = np.broadcast_to(_batch(init.p_n, 1), (B, 3)).copy()
    v = np.broadcast_to(_batch(init.v_n, 1), (B, 3)).copy()
    C = np.broadcast_to(_batch(init.C_np, 2), (B, 3, 3)).copy()
    att = None if attitude is None else np.broadcast_to(_batch(attitude, 3), (B, N, 3, 3))
    if att is not None:
        C = att[:, 0].copy()

    P = np.empty((B, N, 3)); V = np.empty((B, N, 3)); A = np.empty((B, N, 3, 3))
    P[:, 0], V[:, 0], A[:, 0] = p, v, C
    a_k = np.einsum("bij,bj->bi", C, f[:, 0]) + GRAVITY_N
    for k in range(N - 1):
        dt = t[:, k + 1] - t[:, k]
        if att is None:
            C_next = C @ so3_exp_batch(0.5 * (w[:, k] + w[:, k + 1]) * dt[:, None])
        else:
            C_next = att[:, k + 1]
        a_next = np.einsum("bij,bj->bi", C_next, f[:, k + 1]) + GRAVITY_N
        v_next = v + 0.5 * (a_k + a_next) * dt[:, None]
        p = p + 0.5 * (v + v_next) * dt[:, None]
        v, C, a_k = v_next, C_next, a_next
        P[:, k + 1], V[:, k + 1], A[:, k + 1] = p, v, C
    return InsResult(t=t, p_n=P, v_n=V, C_np=A)


def strapdown_meas(imu: ImuMeas, init: InsInit, cal: ImuCalibration = ImuCalibration(),
                   attitude=None) -> InsResult:
    """Single run from an ImuMeas."""
    return strapdown(imu.t, imu.f, imu.w, init, cal, attitude)


def reference_point(p_P, C_np, C_bp, r_b):
    """p_R = p_P - C_np C_bp^T r_b (docs/contracts.md 2). Shapes broadcast over (..., 3)."""
    C_nb = np.asarray(C_np) @ np.asarray(C_bp).T
    return np.asarray(p_P) - C_nb @ np.asarray(r_b)
