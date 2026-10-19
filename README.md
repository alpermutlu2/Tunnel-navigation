# Through the Tunnel

Phone-based position estimation through a GNSS outage of a few minutes (road tunnels,
500 m to 5 km), using only the phone's own sensors plus the navigation map.

Work in progress. Current stage: **truth generator** (v0.2.0). Sensor simulation,
estimators and the browser demo follow.

- `DESIGN.md`: assumptions, sensor models, error budget, estimator roadmap, decision log
- `CHANGELOG.md`: what changed per version and how the numbers moved
- `reports/<version>/metrics.json`: metrics snapshot per version

```bash
pip install numpy scipy matplotlib pytest
python -m pytest -q tests          # 36 tests
python tools/snapshot.py           # write reports/<version>/metrics.json
python tools/snapshot.py --diff 0.2.0 0.3.0
python tools/plot_truth.py         # truth review figure
```
