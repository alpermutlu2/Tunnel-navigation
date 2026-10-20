# Through the Tunnel

Phone-based position estimation through a GNSS outage of a few minutes (road tunnels,
500 m to 5 km), using only the phone's own sensors plus the navigation map.

Work in progress. Current stage: **v0.4.0**: truth generator, IMU/barometer/GNSS error models
(verified at sensor level), strapdown INS, error budget, GNSS pre-tunnel calibration and
reference algorithms (constant velocity, INS, oracle ZUPT). The ESKF (v0.5) and the visual
simulator follow.

- `DESIGN.md`: assumptions, sensor models, error budget, estimator roadmap, decision log
- `docs/decisions/`, `docs/reviews/`: decision records and external reviews
- `CHANGELOG.md`: what changed per version and how the numbers moved
- `reports/<version>/metrics.json`: metrics snapshot per version (target, achieved, tolerance)

Every parameter is tagged [MEAS] (measured, cited), [SPEC] (datasheet) or [ASSUMED]
(`tunnelsim/profiles.py`); sensitivity cases are named overrides, never defaults.

```bash
pip install numpy scipy matplotlib pytest
python -m pytest -q tests              # 123 tests (+2 slow: 24 h Allan, budget vs analytic)
python tools/snapshot.py               # write reports/<version>/metrics.json (runs slow tests)
python tools/snapshot.py --diff 0.3.0 0.4.0
python tools/budget.py 1000            # final error budget, 1000 seeds
python tools/allan_scatter.py          # run-to-run OADEV scatter behind the 24 h tolerances
python tools/plot_truth.py             # truth review figure
```
