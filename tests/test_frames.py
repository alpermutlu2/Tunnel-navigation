"""Sign conventions of docs/contracts.md (attitude error, pitch direction, mount chain)."""
import numpy as np
import pytest

from tunnelsim.frames import (C_BP_NOMINAL, attitude_error_n, mount_rotation, rot_x, rot_y, rot_z,
                              so3_exp, so3_log)

AXES = {"roll_x": (rot_x, 0), "pitch_y": (rot_y, 1), "yaw_z": (rot_z, 2)}


@pytest.mark.parametrize("axis", list(AXES))
@pytest.mark.parametrize("eps", [+0.01, -0.01, +0.3, -0.3])
def test_attitude_error_is_estimate_minus_truth(axis, eps):
    """Estimate rotated by +eps about a nav axis relative to the truth -> +eps on that axis."""
    rot, i = AXES[axis]
    C_true = rot_z(0.7) @ rot_y(-0.05) @ rot_x(0.02)          # arbitrary truth
    C_hat = rot(eps) @ C_true                                 # left (nav-side) perturbation
    d = attitude_error_n(C_hat, C_true)
    expect = np.zeros(3)
    expect[i] = eps
    np.testing.assert_allclose(d, expect, atol=1e-12)
    np.testing.assert_allclose(attitude_error_n(C_true, C_hat), -expect, atol=1e-12)


def test_small_angle_form_has_plus_sign():
    phi = np.array([1e-4, -2e-4, 3e-4])
    C = rot_z(1.0)
    C_hat = so3_exp(phi) @ C
    K = C_hat @ C.T - np.eye(3)                               # ~ [phi]x
    np.testing.assert_allclose([K[2, 1], K[0, 2], K[1, 0]], phi, rtol=1e-3)


def test_so3_log_inverts_exp():
    rng = np.random.default_rng(0)
    for _ in range(100):
        phi = rng.normal(size=3)
        phi *= rng.uniform(0, 3.0) / np.linalg.norm(phi)
        np.testing.assert_allclose(so3_log(so3_exp(phi)), phi, atol=1e-9)


def test_positive_rotation_about_flu_y_is_nose_down():
    nose = rot_y(0.1) @ np.array([1.0, 0, 0])
    assert nose[2] < 0                                        # contracts.md: pitch-up = Ry(-angle)


def test_mount_chain():
    """C_nb = C_np C_bp^T, and the nominal mount maps forward to -z_p, left to -x_p, up to +y_p."""
    C_pb = C_BP_NOMINAL.T
    np.testing.assert_allclose(C_pb @ [1, 0, 0], [0, 0, -1])
    np.testing.assert_allclose(C_pb @ [0, 1, 0], [-1, 0, 0])
    np.testing.assert_allclose(C_pb @ [0, 0, 1], [0, 1, 0])
    C_bp = mount_rotation(1.0, 12.0, 3.0)
    C_nb = rot_z(0.4) @ rot_y(-0.03)
    C_np = C_nb @ C_bp
    np.testing.assert_allclose(C_np @ C_bp.T, C_nb, atol=1e-15)
