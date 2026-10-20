import sys, numpy as np
"""Barometer model check (review 6, item 4): second-order error of the window-centre model and
the statistics of the 1 s average error, to compare with the filter R."""
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from tunnelsim.experiments import simulate_run
for key in ("S3", "S1"):
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, np.arange(20))
    h = tr.pos_phone_n[:, 2] if hasattr(tr, "pos_phone_n") else tr.pos_n[:, 2]
    vz = tr.vel_phone_n[:, 2]
    T = np.arange(np.ceil(tr.t[0]) + 1, tr.t[-1], 1.0)
    model_err, avg_res = [], []
    h_bt = np.interp(bt, tr.t, h)
    for tt in T:
        m = (bt > tt - 1.0) & (bt <= tt)
        lag = tt - bt[m].mean()
        h_avg_true = h_bt[m].mean()                       # what a perfect sensor's average is
        h_pred = np.interp(tt, tr.t, h) - lag * np.interp(tt, tr.t, vz)
        model_err.append(h_avg_true - h_pred)
        avg_res.append(bh[:, m].mean(1) - h_avg_true)    # sensor error of the average (all terms)
    model_err = np.array(model_err); avg_res = np.array(avg_res).T
    d1 = np.diff(avg_res, axis=1)                          # differences remove offset + slow GM
    print(key, "model error (2nd order, deterministic): max |.| %.4f m, rms %.4f m" % (np.abs(model_err).max(), np.sqrt((model_err**2).mean())))
    print(key, "1 s average error: std of 1 s differences / sqrt2 = %.3f m (white part of the average)" % (d1.std() / np.sqrt(2)))
    r = avg_res - avg_res.mean(1, keepdims=True)
    ac = [np.mean(r[:, :-l] * r[:, l:]) / np.mean(r * r) for l in (1, 5, 30)]
    print(key, "autocorr of the average error at 1/5/30 s:", np.round(ac, 3), " total std %.3f m" % r.std())
