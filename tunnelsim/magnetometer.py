"""Phone magnetometer (v0.5.13, owner 2026-10-29): used only in open sky, to start the mount yaw.
Inside the tunnel the field is distorted by steel and the filters never read it (DESIGN §2).

Truth side: reads the truth attitude and the mount.
    m_p = (I + S) C_pb C_bn m_n + C_pb h_b + n
  m_n   Earth field in ENU, Istanbul [ASSUMED, WMM-2025 order of magnitude]: total 47 µT,
        inclination 56°, declination +6°
  h_b   the car's own field at the mount, fixed in the body frame, so it rotates with the phone and
        Android's rotate-the-phone calibration cannot remove it: N(0, 3 µT) per axis per seed [ASSUMED]
  S     residual soft iron, N(0, 2 %) per element per seed [ASSUMED]
  n     white noise 0.4 µT at 25 Hz [ASSUMED, phone AK0991x class]
"""
from __future__ import annotations

import numpy as np

FIELD_UT = 47.0          # [ASSUMED]
INCL_DEG = 56.0          # [ASSUMED]
DECL_DEG = 6.0           # [ASSUMED]
HARD_IRON_UT = 3.0       # [ASSUMED]
SOFT_IRON = 0.02         # [ASSUMED]
NOISE_UT = 0.4           # [ASSUMED]
RATE_HZ = 25.0


def earth_field_enu(total=FIELD_UT, incl_deg=INCL_DEG, decl_deg=DECL_DEG):
    """Earth field in ENU [µT]; declination east of north, inclination down."""
    i, d = np.deg2rad(incl_deg), np.deg2rad(decl_deg)
    h = total * np.cos(i)
    return np.array([h * np.sin(d), h * np.cos(d), -total * np.sin(i)])


def simulate_mag(tr, seeds, C_bp=None, hard_iron_ut=HARD_IRON_UT, soft_iron=SOFT_IRON, noise_ut=NOISE_UT):
    """(t (K,), m (B,K,3) in µT, phone frame) on the truth grid at RATE_HZ."""
    step = max(1, int(round(1.0 / (RATE_HZ * tr.dt))))
    idx = np.arange(0, len(tr.t), step)
    C_pb = (tr.C_bp if C_bp is None else C_bp).T
    m_b = np.einsum("kji,j->ki", tr.C_nb[idx], earth_field_enu())       # C_bn m_n
    m_p0 = m_b @ C_pb.T
    out = []
    for s in seeds:
        rng = np.random.default_rng(np.random.SeedSequence([int(s), 0x6D6167]))   # "mag"
        S = np.eye(3) + soft_iron * rng.standard_normal((3, 3))
        h_p = C_pb @ (hard_iron_ut * rng.standard_normal(3))
        out.append(m_p0 @ S.T + h_p + noise_ut * rng.standard_normal(m_p0.shape))
    return tr.t[idx], np.stack(out)
