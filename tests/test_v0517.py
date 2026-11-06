"""v0.5.17: the re-acquisition cap limits the along-track part of a position correction only (owner,
after a map stress test held a 10–17 m cross-track error for 16–27 s after the exit)."""
import numpy as np

from tunnelsim.eskf import EskfConfig, _update_capped


def _setup():
    # a 2-D position + 1 other state; prior position error large in both directions
    P = np.diag([400.0, 400.0, 1.0, 4.0])[None].repeat(1, 0).astype(float)
    H = np.zeros((1, 3, 4)); H[0, :, 0:3] = np.eye(3)
    R = np.diag([1.0, 1.0, 1.0])[None]
    z = np.array([[12.0, 15.0, 0.0]])          # fix says: 12 m along (x), 15 m across (y)
    return P, H, R, z


def test_cap_on_the_whole_correction_limits_both_directions():
    P, H, R, z = _setup()
    x = np.zeros((1, 4))
    _update_capped(x, P, z, H, R, None, 2.0)
    assert np.linalg.norm(x[0, :3]) <= 2.0 + 1e-9


def test_cap_along_only_limits_along_and_keeps_across():
    P, H, R, z = _setup()
    x = np.zeros((1, 4))
    along = np.array([[1.0, 0.0]])             # the car points along x
    _, alpha = _update_capped(x, P, z, H, R, None, 2.0, along)
    assert abs(x[0, 0]) <= 2.0 + 1e-9          # along-track step capped
    assert abs(x[0, 1]) > 14.0                 # cross-track correction taken (nearly) whole
    assert alpha[0] < 1.0
    ev = np.linalg.eigvalsh(P[0])
    assert ev.min() > 0.0                      # Joseph form: covariance stays valid with this gain
    assert P[0, 1, 1] < 2.0                    # and the cross-track variance collapses as it should


def test_default_is_along_only():
    assert EskfConfig().reacq_cap_along_only is True


def test_vertical_stays_scaled_with_along_only():
    # taken whole, the vertical collapsed P_zz while the along-track variance stayed large; a later
    # barometer update then moved the position 13 m along the road through P_xz (300-seed run)
    P = np.diag([400.0, 400.0, 400.0, 4.0])[None].astype(float)
    P[0, 0, 2] = P[0, 2, 0] = 300.0
    H = np.zeros((1, 3, 4)); H[0, :, 0:3] = np.eye(3)
    R = np.diag([1.0, 1.0, 1.0])[None]
    z = np.array([[12.0, 15.0, 10.0]])
    x = np.zeros((1, 4))
    _, alpha = _update_capped(x, P, z, H, R, None, 2.0, np.array([[1.0, 0.0]]))
    assert abs(x[0, 0]) <= 2.0 + 1e-9
    assert abs(x[0, 1]) > 14.0                  # cross-track whole
    assert abs(x[0, 2]) < 0.5 * 10.0            # vertical scaled by α, not taken whole
    assert np.linalg.eigvalsh(P[0]).min() > 0.0
