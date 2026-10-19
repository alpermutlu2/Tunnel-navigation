# Through the Tunnel

Phone-based position estimation through a GNSS outage of a few minutes (road tunnels,
500 m to 5 km), using only the phone's own sensors plus the navigation map.

Work in progress. Current stage: **sensor simulation** (v0.3.0): truth generator plus IMU,
barometer and GNSS error models, each verified at sensor level. Reference algorithms and the
observability analysis (v0.4), the estimators and the browser demo follow.

- `DESIGN.md`: assumptions, sensor models, error budget, estimator roadmap, decision log
- `docs/decisions/`, `docs/reviews/`: decision records and external reviews
- `CHANGELOG.md`: what changed per version and how the numbers moved
- `reports/<version>/metrics.json`: metrics snapshot per version (target, achieved, tolerance)

Every parameter is tagged [MEAS] (measured, cited), [SPEC] (datasheet) or [ASSUMED]
(`tunnelsim/profiles.py`); sensitivity cases are named overrides, never defaults.

```bash
pip install numpy scipy matplotlib pytest
python -m pytest -q tests              # 84 tests (+1 slow 24 h Allan run with --runslow)
python tools/snapshot.py               # write reports/<version>/metrics.json (runs slow tests)
python tools/snapshot.py --diff 0.2.0 0.3.0
python tools/allan_scatter.py          # run-to-run OADEV scatter behind the 24 h tolerances
python tools/plot_truth.py             # truth review figure
```
