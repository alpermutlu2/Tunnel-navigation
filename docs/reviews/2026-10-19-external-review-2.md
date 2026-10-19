# External review 2 (2026-10-19, after the IMU and barometer commits)

Reviewed: repository state at `40aa755` (source, DESIGN.md, CHANGELOG, reported test results).
The reviewer did not re-run the tests. English rendering of the Turkish review; status column
added by us. Decisions marked *open* are the project owner's.

## Summary of the review

- The project has turned from an ideal-motion simulator into a controlled, reproducible,
  statistically verified error-modelling base for IMU and barometer. No navigation result
  exists yet; what has improved is the reliability of the test bench.
- Most valuable change: the sideslip lag. It removed a truth discontinuity before sensors were
  layered on, so later integration errors cannot be confused between sensor and truth model.
  (MIX strapdown consistency 18.51 mm → 0.172 mm; a numerical consistency figure, not
  navigation accuracy.)
- IMU: per-term RNG streams are the key property for comparative experiments. τ = 5, 50,
  500 s is justified by the fit, not arbitrary. Tolerances come from measured run-to-run
  scatter instead of a blanket ±10 %. Caveat: matching our own Allan target is internal
  validation; it does not show a real phone's bias behaves the same way.
- Main concern: as model complexity grows, the simulator could end up validating its own
  assumptions instead of stressing the navigation algorithm. Hence the weight on temperature,
  pressure transitions and model-mismatch tests.

## Points and status

| # | Point | Reviewer's proposal | Status |
|---|---|---|---|
| 4.1 | Temperature removed from the truth (R4). Unobservable ≠ remove from truth; unobservable effects are exactly what tests the algorithm's limits. Size is BI-level (9.6e-4 m/s², 8.5e-5 rad/s over 3 min) | Keep nominal as is; add a truth-only "temperature residual" **stress scenario** in v0.4, at the latest v0.5. No filter state needed | **accepted**: v0.4, linear ramp, off in nominal (decision R5) |
| 4.2 | Barometer tunnel-gradient term steps at the exit portal (8.9 m for 3.5 km, 30 Pa/km). Not a general model of the pressure field and cabin response; can make exit recovery artificially hard or portal detection artificially easy | Keep the in-tunnel gradient; after the exit `d(s) = d(s_exit)·exp(−(s − s_exit)/L_p)`, L_p a sensitivity parameter (not physically validated); a time constant can be added for the cabin later. Keep the step as a separate stress test | **done** in v0.3: L_p = 200 m nominal, 50/500 m swept, `baro_exit_step` stress override (decision R5) |
| 4.3 | GNSS still on the v0.2 placeholder: velocity error correlation, coasting fixes, Doppler/position recovering at different times, reported vs true accuracy, multipath and receiver lag as separate scenarios. Entry calibration and exit re-convergence depend on these | Finish GNSS before closing v0.3 | **done** (`simulate_gnss`, DESIGN 3.3) |
| 4.4 | `__version__` still "0.2.0"; `snapshot.py` would overwrite `reports/0.2.0/metrics.json`. README still says 36 tests / truth stage | Dev version during development; never overwrite released reports by default; tag + comparison at v0.3 close | **fixed** (`0.3.0.dev0`; released reports need `--force`). README at v0.3 close |
| 5 | Grade is no longer assumed in the map, while DESIGN still says pitch is held by NHC + baro + grade. NHC gives no forward speed; in `f̃_x ≈ v̇ + g·sin γ + b_ax` the three terms need extra information to separate | No 15-state ESKF before the observability analysis. Study three cases (constant speed, gentle braking, changing grade), each with GNSS and after loss | agreed; already v0.4 scope (DESIGN 5, item 0). The three cases are added to that item |

## Reviewer's priorities

1. Close v0.3 safely: GNSS model, timestamp and reported-accuracy tests, baro portal transition
   as its own scenario, snapshot version fix.
2. v0.4 reference check: along-track error budget with ideal attitude, ideal calibration and
   realistic IMU data; show when constant speed beats raw accel integration.
3. Observability: which combinations of v_x, b_ax, pitch, mount angle and grade separate.
4. Model-mismatch tests: temperature drift, wrong map grade, other vehicle vibration, GNSS
   latency.

Wanted for the next review: for a 3 min GNSS outage, how the position-error distribution
changes with **all** IMU error terms, not only forward-axis bias. This decides which error
sources the ESKF and IMM must focus on. (Planned: v0.4 per-term budget; the all-terms
Monte Carlo distribution is added to v0.4.)
