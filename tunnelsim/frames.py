"""Frame conventions and rotation helpers.

Frames
------
n : navigation frame, local ENU (x East, y North, z Up), flat earth, origin at road start.
r : road frame, x along the road tangent, y to the left, z up (perpendicular to the road).
b : vehicle body frame, FLU (x Forward, y Left, z Up). Differs from r by suspension pitch.
p : phone frame, Android/iOS convention: x right of screen, y top of screen, z out of screen.

C_ab denotes the rotation matrix that maps vectors from frame b to frame a: v_a = C_ab v_b.

Attitude parametrisation (no roll in this model):
    C_nr = Rz(psi) @ Ry(-gamma)      psi: heading from East, CCW; gamma: road grade, nose up > 0
    C_rb = Ry(-theta_d)              theta_d: suspension pitch (squat > 0, dive < 0)
"""
from __future__ import annotations

import numpy as np

G0 = 9.80665
GRAVITY_N = np.array([0.0, 0.0, -G0])


def rot_x(a):
    a = np.asarray(a, dtype=float)
    c, s = np.cos(a), np.sin(a)
    o, z = np.ones_like(a), np.zeros_like(a)
    return np.stack([
        np.stack([o, z, z], -1),
        np.stack([z, c, -s], -1),
        np.stack([z, s, c], -1),
    ], -2)


def rot_y(a):
    a = np.asarray(a, dtype=float)
    c, s = np.cos(a), np.sin(a)
    o, z = np.ones_like(a), np.zeros_like(a)
    return np.stack([
        np.stack([c, z, s], -1),
        np.stack([z, o, z], -1),
        np.stack([-s, z, c], -1),
    ], -2)


def rot_z(a):
    a = np.asarray(a, dtype=float)
    c, s = np.cos(a), np.sin(a)
    o, z = np.ones_like(a), np.zeros_like(a)
    return np.stack([
        np.stack([c, -s, z], -1),
        np.stack([s, c, z], -1),
        np.stack([z, z, o], -1),
    ], -2)


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def so3_exp(phi):
    """Rotation matrix for rotation vector phi (Rodrigues)."""
    phi = np.asarray(phi, dtype=float)
    ang = np.linalg.norm(phi)
    K = skew(phi)
    if ang < 1e-12:
        return np.eye(3) + K + 0.5 * K @ K
    return (np.eye(3) + np.sin(ang) / ang * K
            + (1 - np.cos(ang)) / ang**2 * K @ K)


# Nominal phone-in-mount orientation: portrait, screen facing the driver.
# Phone axes expressed in body (FLU): x_p = -y_b (right), y_p = +z_b (up), z_p = -x_b (rearward).
C_BP_NOMINAL = np.column_stack([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
])


def mount_rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """C_bp for a phone mount: nominal orientation followed by an offset in the body frame.

    The offset angles are small, body-frame Euler angles (ZYX). A positive pitch tilts the
    phone screen up / back, which is how most dashboard mounts sit.
    """
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    C_off = rot_z(y) @ rot_y(-p) @ rot_x(r)
    return C_off @ C_BP_NOMINAL


def so3_log(C):
    """Rotation vector phi with so3_exp(phi) = C (angle in [0, pi))."""
    C = np.asarray(C, dtype=float)
    c = np.clip(0.5 * (np.trace(C) - 1.0), -1.0, 1.0)
    ang = np.arccos(c)
    w = np.array([C[2, 1] - C[1, 2], C[0, 2] - C[2, 0], C[1, 0] - C[0, 1]])
    if ang < 1e-8:
        return 0.5 * w
    return ang / (2.0 * np.sin(ang)) * w


def attitude_error_n(C_hat_np, C_np):
    """Attitude error, estimate - truth, as a nav-frame rotation vector (docs/contracts.md 5):

        C_hat_np = Exp([dtheta_n]x) C_np,   dtheta_n = Log(C_hat_np C_np^T)

    A positive component is a positive (right-hand) rotation of the estimate about that nav
    axis relative to the truth. Note FLU pitch: a positive rotation about +y is nose DOWN.
    """
    return so3_log(np.asarray(C_hat_np) @ np.asarray(C_np).T)
