# Changelog

Each version has a git tag and a metrics snapshot in `reports/<version>/metrics.json`
(`python tools/snapshot.py`). Compare two versions with
`python tools/snapshot.py --diff <old> <new>`.

Design decisions and their reasons live in `DESIGN.md` (decision log, section 12) and
`docs/decisions/`. This file records what changed in the code and how the numbers moved.

## Unreleased (v0.3.0 in progress): sensor simulation

Decisions: `docs/decisions/2026-10-19-v0.3-sensors.md` (K1 to K9, R1 to R4).

IMU (this commit; barometer and GNSS follow):
- `tunnelsim/meas.py`: measured types only (`ImuMeas`); imports nothing from the truth side
  (tested).
- `tunnelsim/profiles.py`: every number is a `Param` tagged [MEAS]/[SPEC]/[ASSUMED] (tested);
  one `NOMINAL` config, named `OVERRIDES` that must themselves be tagged. Temperature fields
  removed (R4); gyro scale 0.3 % nominal, 1 % [SPEC] as override; accel turn-on 10 mg now
  [SPEC].
- `tunnelsim/allan.py`: analytic AVAR (white, GM), NNLS fit of three GM (τ = 5, 50, 500 s) to
  the paper's parametric AVAR over 10 to 1000 s, OADEV.
- `tunnelsim/sensors.py`: `simulate_imu` with white noise, bias instability (gm3 baseline,
  gm1 reference), turn-on bias, scale and misalignment, road and engine vibration added
  before the sensor errors (R1), sample-rate offset, jitter, gaps, timestamp scale error.
  Each term has its own RNG stream keyed by a fixed id; switching a term off or dropping
  samples does not change other terms (tested).
- `tools/allan_scatter.py`: run-to-run OADEV scatter, the source of the 24 h tolerances.
- Tests 36 -> 65 (+1 slow, run by `--runslow` and by `tools/snapshot.py`).

New metrics (`sensors.imu` in metrics.json, each with target, achieved, tolerance):
- Analytic fit, worst axis 1.7 % (tolerance 10 %). gm1 reference reported against the same
  target.
- 24 h static run: all axes within `max(10 %, 3σ)`; largest gyro z at 300 s, +11.9 % vs
  12.3 % (2.8σ, see the R2 correction in the decision record).
- Timestamps: rate offset ±2 % with correct timestamps changes the 240 s integral of f by
  < 5e-6 m/s; a 1 % timestamp scale error gives 23.7 m/s, as predicted (ε·∫f dt).
- Vibration rms within 2 % of the profile.

Open (truth, needs a decision): in S2 the linear sideslip model gives a step in β̇ at each
clothoid end, so the truth angular acceleration (np.gradient) is an impulse whose height
grows with the sample rate (0.18 / 0.38 / 0.77 m/s² at 100 / 200 / 400 Hz) while its area
stays r·Δω ≈ 4 mm/s. Resampling the truth at jittered instants then leaves ~1 mm/s
velocity-level error per joint (S1, S3 to S5: < 2e-5 m/s). Marked as a strict xfail in
`test_interpolation_error_negligible[S2]`. Truth metrics unchanged.

## v0.2.0 (2026-10-19): truth generator after external review

First version under git. v0.1.0 below was not committed; it is recorded here from the
working notes so the history stays complete.

- Truth: body sideslip (`beta = -k_beta * a_lat`), phone lever arm (1.6, 0.15, 0.7) m,
  reference point = rear-axle centre. Truth no longer satisfies NHC perfectly
  (0.11 m/s lateral at the phone in S2 curves, 0.11 m/s vertical in S4 stops).
- Speed events trigger on the predicted current position (was: previous sample).
- `Road.curvature_rate()` added.
- Tests 14 -> 36: closed-form circle and Fresnel clothoid, altitude vs quadrature,
  centreline vs adaptive RK (discretisation error reported separately, max 0.07 mm),
  start/end boundaries, event trigger bound, mixed manoeuvre (brake + clothoid + grade).
- Strapdown consistency: <= 0.13 cm on S1-S5, 1.9 cm on the mixed manoeuvre. Mutation
  checks: removing lever-arm acceleration -> 1.6 m, removing beta_dot -> 8.4 m.
- DESIGN.md: grade profile no longer assumed (A3), truth/estimator separation policy,
  observability step, oracle reference, NEES/NIS metrics.

## v0.1.0 (2026-10-17): truth generator (not committed)

- Sensor profiles: Note10 (LSM6DSR) and MPU9250 Allan terms from Suvorkin et al. 2024,
  other terms marked [ASSUMED].
- Scenarios S1-S5, road geometry with clothoids and minimum-jerk grade transitions,
  minimum-jerk speed ramps.
- Truth generator without sideslip or lever arm; 14 tests, strapdown <= 0.13 cm.
- Found: piecewise-linear grade made even ideal strapdown drift 7 m on S3 (gravity leakage
  from pitch-rate steps); fixed with smooth grade transitions.
