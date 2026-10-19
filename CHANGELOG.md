# Changelog

Each version has a git tag and a metrics snapshot in `reports/<version>/metrics.json`
(`python tools/snapshot.py`). Compare two versions with
`python tools/snapshot.py --diff <old> <new>`.

Design decisions and their reasons live in `DESIGN.md` (decision log, section 12) and
`docs/decisions/`. This file records what changed in the code and how the numbers moved.

## v0.3.0 (2026-10-19): sensor simulation

Metric diff vs 0.2.0 (`tools/snapshot.py --diff 0.2.0 0.3.0`): the only existing metrics that
moved are the five truth metrics listed under "Truth" below, all from the sideslip lag;
`sensors.*` is new. Tests 36 -> 85 (one slow), all pass. Design note for v0.5 bias calibration:
DESIGN 5.0.

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
- Tests 36 -> 65 (1 of them slow, run by `--runslow` and by `tools/snapshot.py`).

New metrics (`sensors.imu` in metrics.json, each with target, achieved, tolerance):
- Analytic fit, worst axis 1.7 % (tolerance 10 %). gm1 reference reported against the same
  target.
- 24 h static run: all axes within `max(10 %, 3σ)`; largest gyro z at 300 s, +11.9 % vs
  12.3 % (2.8σ, see the R2 correction in the decision record).
- Timestamps: rate offset ±2 % with correct timestamps changes the 240 s integral of f by
  < 5e-6 m/s; a 1 % timestamp scale error gives 23.7 m/s, as predicted (ε·∫f dt).
- Vibration rms within 2 % of the profile.

Barometer (K7):
- `simulate_baro`: white + GM + constant offset + cabin GM + tunnel pressure gradient, 25 Hz;
  `BaroMeas` in meas.py; tagged `BARO` profile replaces the v0.2 placeholder (weather drift
  term dropped, covered by the GM). Nominal GM(0.1 m, 60 s); white-only, GM(0.3, 300),
  GM(0.3, 60), cabin, 10 and 30 Pa/km tunnel gradient as named overrides.
- Note10 barometer part not confirmed; LPS22HB quoted as representative, values [ASSUMED].
- New metrics `sensors.baro`: grade floor per case and window (exact, Monte Carlo, closed
  form; MC within 4 % of exact, tolerance 6 %), tunnel gradient exit step (matches k_p L/ρg).
- Tests +11.
- Exit transition (review 2, 4.2): the tunnel-gradient term decays after the exit over
  L_p = 200 m (50/500 m swept) instead of stepping; the step is the stress override
  `baro_exit_step`. Metric `sensors.baro.tunnel_gradient` gains the exit level, largest
  per-sample jump (30 Pa/km: 8.90 m step -> 0.035 m at L_p = 200 m) and decay check.
  Nominal unaffected (gradient off). Tests +1.

GNSS (K8, K9):
- `simulate_gnss`, `GnssMeas` (NaN where unavailable), tagged `GNSS` profile replacing the v0.2
  placeholder. Open-sky GM position and white + GM velocity; entry degradation; nominal stop or
  coasting from the receiver's own last fix; velocity before position at the exit; exit bias
  as random-direction multipath (nominal) or lagging receiver (scenario); reported accuracy
  from model σ × ratio; IMU–GNSS time offset. Unspecified details chosen [ASSUMED], listed in
  DESIGN 3.3.
- New metrics `sensors.gnss`: open-sky error std and normalised squared error per reported
  ratio (all within 3 %, tolerance 5 %); calibration floor per velocity-noise case
  (60 s: white-only 7.3e-4, nominal 1.8e-3 m/s²); outage and re-acquisition timing; exit bias
  magnitude and decay; time-offset errors (v·δ, a·δ).
- Tests +8.

Versioning: `__version__` is `0.3.0.dev0` (was still `0.2.0`, so `tools/snapshot.py` would
have overwritten `reports/0.2.0/`; found by external review 2). `snapshot.py` now refuses to
overwrite a released report without `--force`. Review recorded in `docs/reviews/`.

Truth (separate commit): first-order lag on body sideslip, `τ_β β̇ + β = −k_β·a_lat`,
τ_β = 0.2 s [ASSUMED] (`Vehicle.sideslip_tau_s`; 0 reproduces v0.2).
- Why: found by the IMU resampling test. With the algebraic model β̇ stepped at clothoid
  ends, so the truth angular acceleration (np.gradient) was an impulse whose height grew with
  the sample rate (0.18 / 0.38 / 0.77 m/s² at 100 / 200 / 400 Hz), area r·Δω ≈ 4 mm/s.
  Resampling at jittered instants left ~1 mm/s velocity-level error per joint in S2. Not
  physical: a real car's sideslip settles with a lag.
- Metrics that moved (vs 0.2.0), all from this change:
  - `truth.S2.strapdown_max_err_m` 1.312e-3 -> 3.12e-5 and `truth.MIX.strapdown_max_err_m`
    1.851e-2 -> 1.72e-4: the v0.2 strapdown residuals came from the same impulse.
  - `truth.MIX.sideslip_max_deg` 0.751 -> 0.748, `truth.MIX.nhc_lat_vel_max_mps`
    0.1063 -> 0.1078, `truth.S2.nhc_lat_vel_max_mps` 0.1062 -> 0.1061: β now lags a_lat.
- S2 velocity-level resampling error 9.8e-4 -> 3.0e-5 m/s; the S2 xfail is removed.
- Mutation check re-measured: dropping β̇ still gives 8.3 m on MIX (v0.2: 8.4 m).

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
