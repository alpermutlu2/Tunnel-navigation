"""v0.5.17.1: a GNSS fix the gate rejects does not count towards ending the re-acquisition phase
(from a review of v0.5.17: rejected fixes gave alpha = 1 and counted as settled)."""
import numpy as np

from tunnelsim.eskf import EskfConfig, _settle_step, _update_capped


def test_rejected_fix_has_alpha_one_and_is_not_accepted():
    P = np.diag([4.0, 4.0, 4.0, 1.0])[None]
    H = np.zeros((1, 3, 4)); H[0, :, 0:3] = np.eye(3)
    R = np.eye(3)[None]
    x = np.zeros((1, 4))
    acc, alpha = _update_capped(x, P, np.array([[200.0, 0.0, 0.0]]), H, R, 16.27, 2.0, np.array([[1.0, 0.0]]))
    assert not acc[0] and alpha[0] == 1.0      # the trap: "uncapped" although nothing was applied
    assert np.all(x == 0.0)


def test_rejected_fixes_in_a_row_do_not_settle():
    settle = np.zeros(1, int)
    for _ in range(6):                          # six rejected fixes in a row
        settle = _settle_step(settle, np.array([False]), np.array([1.0]))
    assert settle[0] == 0
    old = np.zeros(1, int)
    for _ in range(6):
        old = _settle_step(old, np.array([False]), np.array([1.0]), accepted_only=False)
    assert old[0] == 6                          # v0.5.17 would have ended the phase


def test_accepted_uncapped_counts_capped_or_rejected_resets():
    s = _settle_step(np.array([3]), np.array([True]), np.array([1.0]))
    assert s[0] == 4
    assert _settle_step(np.array([3]), np.array([True]), np.array([0.5]))[0] == 0
    assert _settle_step(np.array([3]), np.array([False]), np.array([1.0]))[0] == 0


def test_default_counts_accepted_only():
    assert EskfConfig().reacq_settle_accepted_only is True


def test_phase_ends_when_locked_out_after_min_time():
    from tunnelsim.eskf import _reacq_done
    cfg = EskfConfig()
    age = np.array([5.0, 12.0, 12.0, 12.0])
    settle = np.array([0, 0, 5, 2])
    rej = np.array([9, 5, 0, 3])
    done, lock = _reacq_done(age, settle, rej, cfg)
    assert done.tolist() == [False, True, True, False]   # too early; locked out; settled; neither
    assert lock.tolist() == [False, True, False, False]


def test_all_rejected_ends_where_v0517_did():
    # every fix rejected: v0.5.17 counted them as settled; v0.5.17.1 counts them as a lock-out; same N
    cfg = EskfConfig()
    assert cfg.reacq_rej_exit_n == cfg.reacq_settle_fixes
