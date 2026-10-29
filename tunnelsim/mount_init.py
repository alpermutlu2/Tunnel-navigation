"""Mount alignment in open sky (v0.5.13, owner's design 2026-10-29): how the phone sits in the car,
estimated before the tunnel instead of assumed known.

Estimator side: imports only numpy. Inputs are the phone's own data in a window of open sky:
IMU (f, w), magnetometer (m), GNSS fixes; plus the Earth field from a world model (on Android,
GeomagneticField / WMM). Output: C_bp (body <- phone) per batch member.

1. Tilt from gravity (owner: "complementary init"): over GNSS epochs where the car does not
   accelerate (|a_GNSS| < 0.1 m/s²) and the gyro is quiet, the 1 s mean specific force is gravity in
   the phone frame. The car's own pitch there is the road grade, atan(v_up / v_h) from GNSS; roll 0.
2. Yaw from the magnetometer (owner): the phone attitude from gravity and the field (TRIAD), against
   the car's attitude from the GNSS course and grade: C_bp = C_bn · C_np, averaged over those epochs.
3. Yaw from the velocity direction (owner): where the car accelerates or turns, the change of the
   specific force against the GNSS acceleration, a_n = Δv/Δt, expressed in the phone frame through
   C_bp(δ) = R_z(δ) C_bp(mag). The δ that fits best replaces the magnetometer's yaw when the
   acceleration seen is enough (Σ|a_h|² ≥ EXCITE_MIN); else the magnetometer's yaw stays.
   Using the change (minus the still-epoch mean) removes the accelerometer bias.
"""
from __future__ import annotations

import numpy as np

G0 = 9.80665
FIELD_UT, INCL_DEG, DECL_DEG = 47.0, 56.0, 6.0    # world magnetic model at the drive (Istanbul) [ASSUMED]
QUIET_ACC = 0.1          # m/s²  [ASSUMED]
QUIET_GYRO = 0.03        # rad/s [ASSUMED]
MOVING_V = 5.0           # m/s: course usable above this [ASSUMED]
EXCITE_ACC = 0.3         # m/s²: an epoch counts as accelerating above this [ASSUMED]
EXCITE_MIN = 3.0         # (m/s²)²: Σ|a_h|² over accelerating epochs needed to trust step 3 [ASSUMED]
WINDOW_S = 90.0          # s of open sky used, from the first fix [ASSUMED]


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _basis(a, b):
    e1 = a / np.linalg.norm(a)
    e2 = np.cross(a, b); e2 /= np.linalg.norm(e2)
    return np.column_stack([e1, e2, np.cross(e1, e2)])


def _orthonormal(M):
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1; R = U @ Vt
    return R


def _C_nb(course, grade):
    """Car attitude (body FLU -> ENU) from the GNSS course and the road grade, roll 0."""
    x = np.array([np.cos(grade) * np.cos(course), np.cos(grade) * np.sin(course), np.sin(grade)])
    y = np.array([-np.sin(course), np.cos(course), 0.0])
    return np.column_stack([x, y, np.cross(x, y)])


def earth_field_enu():
    i, d = np.deg2rad(INCL_DEG), np.deg2rad(DECL_DEG)
    h = FIELD_UT * np.cos(i)
    return np.array([h * np.sin(d), h * np.cos(d), -FIELD_UT * np.sin(i)])


def estimate_mount(imu_t, f, w, mag_t, mag, gnss, window_s=WINDOW_S, use_mag=True, use_accel=True):
    """C_bp (B,3,3) and per-member diagnostics. f, w (B,N,3) phone frame; mag (B,K,3) µT."""
    B = f.shape[0]
    gt = gnss["t"]
    m_n = earth_field_enu()
    out, info = np.zeros((B, 3, 3)), []
    for b in range(B):
        okv = np.all(np.isfinite(gnss["vel"][b]), axis=1)
        j0 = int(np.argmax(okv))
        sel = np.nonzero(okv & (gt >= gt[j0]) & (gt <= gt[j0] + window_s))[0]
        sel = sel[(sel > 0) & (sel < len(gt) - 1)]
        rows = []
        for j in sel:
            if not (okv[j - 1] and okv[j + 1]):
                continue
            v = gnss["vel"][b, j]
            a = (gnss["vel"][b, j + 1] - gnss["vel"][b, j - 1]) / (gt[j + 1] - gt[j - 1])
            mi = (imu_t >= gt[j] - 0.5) & (imu_t < gt[j] + 0.5)
            mm = (mag_t >= gt[j] - 0.5) & (mag_t < gt[j] + 0.5)
            if not mi.any() or not mm.any():
                continue
            vh = float(np.hypot(v[0], v[1]))
            rows.append((v, a, vh, f[b, mi].mean(0), w[b, mi].mean(0), mag[b, mm].mean(0)))
        quiet = [r for r in rows if np.linalg.norm(r[1]) < QUIET_ACC and np.linalg.norm(r[4]) < QUIET_GYRO and r[2] > MOVING_V]
        # 1 + 2: tilt from gravity, yaw from the magnetometer (TRIAD), against the car's course/grade
        Ms = []
        for v, a, vh, fm, wm, mm_ in quiet:
            C_np = _basis(np.array([0.0, 0.0, 1.0]), m_n) @ _basis(fm, mm_).T if use_mag else None
            if C_np is None:
                continue
            Ms.append(_C_nb(np.arctan2(v[1], v[0]), np.arctan2(v[2], vh)).T @ C_np)
        C_bp = _orthonormal(np.sum(Ms, axis=0)) if Ms else np.eye(3)
        src = "mag" if Ms else "none"
        # 3: yaw from the velocity direction where the car accelerates (bias removed by differencing)
        acc = [r for r in rows if r[2] > 3.0 and np.linalg.norm(r[1][:2]) > EXCITE_ACC]
        excite = float(sum(np.linalg.norm(r[1][:2]) ** 2 for r in acc))
        delta = 0.0
        if use_accel and quiet and excite >= EXCITE_MIN:
            f_ref = np.mean([r[3] for r in quiet], axis=0)
            g_ref = np.mean([_C_nb(np.arctan2(r[0][1], r[0][0]), np.arctan2(r[0][2], r[2])).T @ np.array([0.0, 0.0, G0])
                             for r in quiet], axis=0)                       # gravity in the body frame, still epochs
            grid = np.deg2rad(np.arange(-45.0, 45.01, 0.25))
            cost = np.zeros_like(grid)
            for v, a, vh, fm, wm, mm_ in acc:
                C_bn = _C_nb(np.arctan2(v[1], v[0]), np.arctan2(v[2], vh)).T
                df_b_pred = C_bn @ (a + np.array([0.0, 0.0, G0])) - g_ref             # change of f, body frame
                df_p = fm - f_ref
                for i, d in enumerate(grid):
                    cost[i] += np.sum((df_p - (_rz(d) @ C_bp).T @ df_b_pred) ** 2)
            delta = float(grid[int(np.argmin(cost))])
            C_bp = _rz(delta) @ C_bp
            src = "mag+accel" if Ms else "accel"
        out[b] = C_bp
        info.append({"n_quiet": len(quiet), "n_accel": len(acc), "excitation": round(excite, 2),
                     "yaw_from": src, "accel_correction_deg": round(float(np.rad2deg(delta)), 2)})
    return out, info


def mount_error_deg(C_bp_hat, C_bp_true):
    """Roll, pitch, yaw error of the estimate in the body frame [deg], (B,3)."""
    E = C_bp_hat @ C_bp_true.T
    roll = np.arctan2(E[:, 2, 1], E[:, 2, 2])
    pitch = -np.arcsin(np.clip(E[:, 2, 0], -1, 1))
    yaw = np.arctan2(E[:, 1, 0], E[:, 0, 0])
    return np.rad2deg(np.stack([roll, pitch, yaw], 1))
