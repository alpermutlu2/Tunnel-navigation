"""Run-to-run scatter of OADEV for one 24 h record of the IMU noise model (decision R2).

The 24 h Allan test compares ONE realisation with the model curve, so its tolerance must come
from how much one realisation scatters. This tool measures it: many independent 24 h records
of white + bias-instability per axis, relative std of OADEV at each T.

White + GM processes are generated at 1 Hz with the same densities; for T >= 10 s this gives
the same AVAR and scatter as 100 Hz (checked for gyro z with 40 runs at 100 Hz:
2.2 %, 4.2 %, 6.0 % at 100, 300, 1000 s).

Usage: python tools/allan_scatter.py [runs]
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tunnelsim.allan import oadev  # noqa: E402
from tunnelsim.profiles import NOTE10_LSM6DSR as P  # noqa: E402
from tunnelsim.sensors import bi_components, gm_sum  # noqa: E402

T = [10.0, 30.0, 100.0, 300.0, 1000.0]


def main(runs: int = 150, seed: int = 123):
    dt, n = 1.0, 86400
    rng = np.random.default_rng(seed)
    worst = np.zeros(len(T))
    for sensor, NN, BB in (("accel", P.accel_vrw.a, P.accel_bias_instab.a),
                           ("gyro", P.gyro_arw.a, P.gyro_bias_instab.a)):
        for i, ax in enumerate("xyz"):
            sig, taus = bi_components(NN[i], BB[i], P)
            A = np.array([oadev(rng.standard_normal(n) * NN[i] / np.sqrt(dt)
                                + gm_sum(rng, n, dt, sig, taus), dt, T)[1] for _ in range(runs)])
            r = A.std(0) / A.mean(0)
            worst = np.maximum(worst, r)
            print(f"{sensor}_{ax}: " + "  ".join(f"T={t:g}:{x * 100:4.1f}%" for t, x in zip(T, r)))
    print("worst axis 1 sigma: " + "  ".join(f"T={t:g}:{x * 100:4.1f}%" for t, x in zip(T, worst)))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 150)
