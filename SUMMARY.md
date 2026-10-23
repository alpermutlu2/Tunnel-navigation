# Through the Tunnel: summary

Short version of [`REPORT.md`](REPORT.md), which has the full results, mechanisms and sources.
All numbers come from simulation, 50 seeds per scenario (`reports/0.5.3/`, low-end IMU
`reports/0.5.5/`). Every parameter is tagged [MEAS] (measured, cited), [SPEC] (datasheet) or
[ASSUMED] (chosen here).

## Problem

A phone in a car loses GNSS in a single-lane tunnel of 0.5–5 km. The navigation app must keep
showing the car, call out an exit inside the tunnel at the right moment, and recover cleanly at the
exit. The app has the map, so the real unknown is the **distance along the road**. A 22 m error
shifts the callout by 1 s at 80 km/h.

A measured error budget set the direction:
- Free integration of a Galaxy Note10 IMU [MEAS] drifts 1.2–1.5 km, because pitch errors leak
  gravity.
- With pitch held, the accelerometer bias leaves ~30 m after 3 minutes.
- A constant-velocity guess is good at steady speed and fails at the first brake.

## Approach

An error-state Kalman filter on the phone's 100 Hz IMU, with layers that each add one kind of
information:

- **GNSS before the tunnel:** calibrates the biases and attitude.
- **Vehicle constraints:** no sideways or vertical body motion, which holds pitch. The sideways
  row is skipped in curves, where the car slips.
- **Barometer:** holds the vertical channel.
- **Stop detection:** zero-velocity updates at stops.
- **Driver model (cruise):** a stated assumption that the speed wanders slowly (σ 0.5 m/s,
  τ 60 s), not a measurement.
- **Map in the tunnel:** heading and lateral position against the route, with the map's own
  offset and length errors as states.
- **Callout rule:** announce when ŝ + 1·σ_s reaches the callout point, so the callout errs early.
  The voice says the estimated distance, rounded to 50 m.
- **Clean exit:** GNSS re-enters gradually, so the car does not jump on the screen.
- **IMU class detection:** at start-up the IMU class is detected from gyro noise and the filter
  takes that class's tuning.

## What is simulated

Truth, sensors and estimator are separate layers, and a test enforces that the estimator never
sees truth.

- **Truth:** the road (curves, grades) and a car with smooth speed changes, sideslip and
  suspension pitch. The phone sits on a dashboard mount with a lever arm.
- **Sensors:**
  - IMU with measured noise terms, bias drift, vibration, scale and misalignment;
  - barometer at 25 Hz;
  - GNSS at 1 Hz, which fades before the portal, gives nothing inside, and comes back at the exit
    with 30 m of multipath.
- **Scenarios:** straight (S1), S-bend (S2), V-profile (S3), stop-and-go (S4), braking (S5) and
  speed wander (S6), in tunnels of 0.5, 1, 3 and 5 km. In each, the route leaves through a 20°
  branch inside the tunnel.
- **Live:** in a live session any tunnel length, grade, speed and stop can be set from the page.

## Key assumptions

- **Phone:** a rigid mount.
- **IMU:** Note10 (LSM6DSR) Allan terms [MEAS].
- **GNSS:** 3 m and 0.1 m/s errors, correlated [ASSUMED].
- **Barometer:** a closed cabin and no ventilation pressure gradient [ASSUMED].
- **Map:** lateral error σ 2 m and length error σ 0.5 % [ASSUMED].
- **Vehicle and driver:** sideslip 0.4°/(m/s²), speed wander as above [ASSUMED].
- **Not used:**
  - magnetometer, distorted in tunnels;
  - ambient light, since GNSS loss already marks the portal;
  - camera, beacons, OBD, not available to an average user.

## Results (Note10)

| | straight, steady (S1, S6) | stop-and-go, braking (S4, S5) | S-bend (S2) | 5 km |
|---|---|---|---|---|
| along-track error at the exit branch, rms | 12–13 m | 25–31 m | 3.5 m | 25 m |
| along-track error at the portal, rms | 5.6–6.2 m | 5.6–7.5 m | 4.1 m | 14 m |
| cross-track error at the portal, rms | 1.5 m | 1.3–1.9 m | 3.2 m | 2.1 m |
| callout as announced: mean, share > 1 s late | 0.8 s early, 0 % | 0.8–2.6 s early, 0–2 % | 0.3 s early, 0 % | 1.6 s early, 0 % |

- **Consistent.** The filter's own uncertainty covers its error everywhere: mean (e/σ)² 0.11–0.27,
  conservative.
- **Exit branch.** The branch is recognised in 500 of 500 runs. When the car goes straight on
  instead, "main" is reported in 500 of 500.
- **Exit.** The largest jump at the exit is under 6 m (p95).
- **Callout rule.** Without it, 30–40 % of callouts on S4 and S5 came more than 1 s late.

## Where it breaks

1. **Stops and braking** give the largest errors. The callout rule turns them into early rather
   than late callouts.
2. **The driver model is an assumption.** Without GNSS a small real acceleration and a sensor bias
   look the same. A stricter "driver holds speed" model is 30–50 % better, but fails as soon as the
   speed wanders by ±1 km/h.
3. **A tunnel pressure gradient** of 30 Pa/km is read as a climb: 110–120 m error and a wrong branch
   decision. A gradient state fixes that case but costs normal roads, because the barometer cannot
   tell the gradient from a pitch error. It should be measured in a real tunnel.
4. **The map's own error limits what the map adds.** At twice the assumed map error the filter
   becomes optimistic, and on 5 km the map then does worse than no map.
5. **A low-end IMU** (MPU9250 class [MEAS]) works with its own tuning, with the fast gyro bias
   terms treated as white noise:
   - 14–43 m at the portal in steady and long tunnels;
   - consistent ((e/σ)² 0.2–1.0) and no late callouts;
   - stop-and-go still fails (~1 km): attitude drifts during long stops, so the app should not
     trust σ there and should fall back to a different model.
6. **Levelling from gravity** (an extra update) was tried and left off: it helps one scenario and
   hurts five.

## Limitations and next steps

- **Limitations:**
  - simulation only: the IMU noise is measured, while the rest of the sensor and map behaviour is
    assumed;
  - a rigid mount, one lane, one car;
  - an offline stop detector.
- **With real devices:**
  - static Allan runs;
  - recordings in Istanbul tunnels with the portals as checkpoints;
  - measuring map polyline errors and tunnel pressure gradients;
  - a causal stop detector;
  - the IMU class from the sensor name.

## AI usage

- **How AI was used:**
  - **Builder:** an AI coding agent (Claude, Anthropic) wrote the code, tests, experiments and
    document drafts.
  - **Reviewers:** other AI models, Claude Fable 5.1 among them. Their reviews are the Markdown
    files in `docs/reviews/`; the author filtered each before passing it on.
  - **Workflow:** before each piece, the agent proposed options with formulas, and the author
    approved, changed or rejected them (`docs/decisions/`).
- **Where the author corrected or overruled the AI:**
  - **Focus:** start where the bias can be estimated, then remove assumptions.
  - **Driver model:** rejected the agent's observability claim and proposed the Gauss–Markov
    wander model, judged by consistency.
  - **Exit branch and NHC turn gate:** required both, with the checks behind them.
  - **Map layer:** its design, with map errors as states.
  - **Callout:** overruled the agent's k = 1.65 in favour of k = 1.
  - **Simulator:** set its requirements and found its playback bug.
- **Detail:** REPORT §6 lists every intervention and the choices the agent made. It also lists
  the AI-side errors and what caught them: the author, AI reviews, the agent's own tests, or the
  consistency metric.
