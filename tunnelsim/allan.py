"""Allan variance tools: analytic AVAR of the noise models, bias-instability fit, OADEV.

The bias-instability target follows the parametric model of Suvorkin et al. 2024, Table 2:

    AVAR_target(T) = N^2 / T + (2 ln2 / pi) B^2

N (VRW/ARW) and B (bias instability) are [MEAS]. Flicker noise is realised as a sum of
first-order Gauss-Markov processes whose weights are fitted (NNLS) so that white + GM sum
matches the target over a finite range (decision R3, docs/decisions/2026-10-19-v0.3-sensors.md).
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np
from scipy.optimize import nnls

FLICKER_K = 2.0 * np.log(2.0) / np.pi          # AVAR of flicker = FLICKER_K * B^2 (floor 0.664 B)


def avar_white(T, N):
    """White noise with density N (m/s/sqrt(s) or rad/sqrt(s))."""
    return N**2 / np.asarray(T, dtype=float)


def avar_gm(T, sigma, tau):
    """First-order Gauss-Markov, steady-state std sigma, correlation time tau."""
    T = np.asarray(T, dtype=float)
    q = 2.0 * sigma**2 / tau
    return q * tau**2 / T * (1.0 - tau / (2.0 * T) * (3.0 - 4.0 * np.exp(-T / tau) + np.exp(-2.0 * T / tau)))


def avar_target(T, N, B):
    return avar_white(T, N) + FLICKER_K * B**2


def avar_model(T, N, sigmas, taus):
    return avar_white(T, N) + sum(avar_gm(T, s, t) for s, t in zip(sigmas, taus))


@lru_cache(maxsize=None)
def fit_gm_sigmas(N: float, B: float, taus: tuple, t_range: tuple, n_pts: int = 200) -> tuple:
    """Steady-state sigmas of the GM components so that white + GM sum matches the target.

    Least squares on the relative AVAR error over a log grid of t_range, weights >= 0.
    """
    T = np.logspace(np.log10(t_range[0]), np.log10(t_range[1]), n_pts)
    tgt = avar_target(T, N, B)
    A = np.stack([avar_gm(T, 1.0, tau) for tau in taus], axis=1) / tgt[:, None]
    rhs = 1.0 - avar_white(T, N) / tgt
    w, _ = nnls(A, rhs)
    return tuple(float(np.sqrt(x)) for x in w)


def model_fit_error(N, B, sigmas, taus, t_range, n_pts: int = 200) -> float:
    """Max |ADEV_model / ADEV_target - 1| over t_range."""
    T = np.logspace(np.log10(t_range[0]), np.log10(t_range[1]), n_pts)
    return float(np.max(np.abs(np.sqrt(avar_model(T, N, sigmas, taus) / avar_target(T, N, B)) - 1.0)))


def gm_change_rms(sigmas, taus, horizon):
    """RMS of b(t + horizon) - b(t) for a sum of independent GM processes."""
    return float(np.sqrt(sum(2.0 * s**2 * (1.0 - np.exp(-horizon / t)) for s, t in zip(sigmas, taus))))


def oadev(y, dt, T_list):
    """Overlapping Allan deviation of a rate signal y sampled at dt, at averaging times T_list.

    Returns (T_actual, adev); T_actual is T rounded to a whole number of samples.
    """
    y = np.asarray(y, dtype=float)
    theta = np.concatenate([[0.0], np.cumsum(y)]) * dt          # integrated signal
    out_T, out = [], []
    for T in T_list:
        m = int(round(T / dt))
        if m < 1 or 2 * m >= len(theta):
            continue
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        tau = m * dt
        out_T.append(tau)
        out.append(np.sqrt(np.mean(d * d) / (2.0 * tau * tau)))
    return np.array(out_T), np.array(out)
