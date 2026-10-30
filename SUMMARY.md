# Through the Tunnel: summary

Short version of [`REPORT.md`](REPORT.md), which has the full results, mechanisms and sources.
All numbers come from simulation. The main results are ranges over six independent sets of 50
seeds (0–299); tuning was done on seeds 0–49, features were designed on 50–299, and a further 100
seeds (400–499) were run once on the v0.5.13 configuration for validation (`reports/0.5.13/`; each
earlier version spent its own validation set). Every parameter is tagged [MEAS] (measured, cited), [SPEC] (datasheet) or
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
- **Mount alignment in open sky:** how the phone sits in the car is estimated, not assumed: tilt
  from the gravity mean, yaw first from the magnetometer, then from the direction of the car's
  acceleration. The remaining yaw error is a filter state.
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

- **Phone:** a rigid mount, at an unknown angle (estimated on the approach).
- **IMU:** Note10 (LSM6DSR) Allan terms [MEAS].
- **GNSS:** 3 m and 0.1 m/s errors, correlated [ASSUMED].
- **Barometer:** a closed cabin and no ventilation pressure gradient [ASSUMED].
- **Map:** lateral error σ 2 m and length error σ 0.5 % [ASSUMED].
- **Vehicle and driver:** sideslip 0.4°/(m/s²), speed wander as above [ASSUMED].
- **Not used:**
  - magnetometer in the tunnel, where steel distorts it (it is used only in open sky, for the
    mount);
  - ambient light, since GNSS loss already marks the portal;
  - camera, BLE beacons, OBD: not available to an average user, and a tunnel with beacons is not a
    fair assumption.

## Results (Note10)

Ranges over six sets of 50 seeds (0–299):

| | straight, steady (S1, S6) | stop-and-go, braking (S4, S5) | S-bend (S2) | 5 km |
|---|---|---|---|---|
| along-track error at the exit branch, rms | 11–18 m | 25–48 m | 4.2–5.2 m | 25–38 m |
| along-track error at the portal, rms | 5.6–8.0 m | 4.5–8.7 m | 4.0–5.5 m | 13–17 m |
| cross-track error at the portal, rms (v0.5.10, 50 seeds) | 1.5 m | 1.3–1.9 m | 3.2 m | 2.1 m |
| callouts more than 1 s late (k = 1), of 300 | 0 | 3 (S4), 15 (S5) | 0 | 2 |
| shown position under 5 m after the exit, median per set | 12–17 s | 11–18 s | 12–16 s | 10–12 s |

- **Consistent.** The filter's own uncertainty covers its error everywhere: mean (e/σ)² ≤ 0.38 in
  every set, conservative.
- **Callout.** Without the rule, 0–56 % of callouts come more than 1 s late; with k = 1, 20 of
  3000. k = 1.3 cuts them 4 fold (5 of 3000; 0 of 1000 in validation), at 0.3–0.9 s more
  advance. Prompts at 1000, 500 and 200 m and "now" follow the same rule; when the shown 2σ is over
  100 m the prompt names no distance ("Exit A ahead, keep right").
- **Exit branch.** Recognised in 2999 of 3000 runs that take it (one stop-and-go run decided
  "main", 41 m off). When the car goes straight on, "main" is reported in 599 of 600 (one
  undecided). Such a car has no curve to correct it: on stop-and-go, 11 % of straight-on runs are
  more than 100 m off at the exit, until GNSS returns. Until "main" is decided (13 s on
  stop-and-go) the car is shown on the branch, up to 38 m off.
- **Exit.** The largest jump at the exit is under 6.3 m (p95); the shown position is under 5 m
  for good a median 10–18 s after the first fix.
- **Mount.** Estimated on the approach to ~0.6° per axis rms, it costs nothing measurable when the
  driver accelerates or turns before the tunnel. Without that, the yaw rests on the magnetometer
  (5.8°) and the portal error triples (17–25 m) and is not consistent: open.
- **Validation on 100 untouched seeds (400–499),** run once, nothing changed after: no run more
  than 100 m off; inside the ranges except the S3 diverge error in one set (17.9 m against
  11.6–15.7 m); 6 of 1000 callouts more than 1 s late with k = 1.
- **Branch window.** v0.5.11's validation found one S4 run 110 m off: its dead-reckoning error
  (−220 m) put the turn outside the branch decision's fixed 100 m window. The window now grows
  with σ_s (max(100 m, 2σ_s)); the run is 3 m off and nothing else moved. Those seeds are now a
  design tier.
- **Seed tiers and method.** A callout claim of "1 in 500 late" made on 0–49 became 22 in 3000;
  a low-end failure count of 35 → 0 on 20 seeds was 35 → 6 on 300. 20 seeds to look, 300 for the
  number.
- **A failure found on new seeds and fixed.**
  - Found: in 2 of 200 runs on S4 the filter ended ~600 m off with a small σ. It reached the exit
    curve far behind the car, and the map update at the wrong point wrote the curve into yaw.
  - Fixed: re-association along the arc (the yaw says where in the curve the car is). It removes
    those failures (0 in 200, S4 portal p95 11 m) and keeps the straight-on control and the
    known-failure cases unchanged.
  - An innovation gate stays as a safety net. On its own it had traded the two failures for three
    smaller ones.

## Three filter sizes

A 4-state (1D), a 10-state and the 16-state ESKF ran on the same data (REPORT §4.9):
- **At the exit branch** on a mid-range phone, the three agree within ~2 m, so accuracy does not
  separate them.
- **What the 16 states buy is independence from assumptions:**
  - the map's grade: the 4-state filter is 3.4 km off without it and ~100 m off with a 10 % error;
  - the vehicle model: suspension pitch, 94 m on S4;
  - a σ that covers the error, which the callout rule needs;
  - a branch hypothesis and a capped exit.
- **On a low-end IMU the smaller filters win,** because they do not trust the gyro for pitch.
- **The cost of 16 states** is the tuning and verification surface (82 config fields), not
  computation.

## Supervisor

The app runs all three and shows one (REPORT §4.12):
- **Class rule:** ESKF on a Note10-class phone, 10-state on a low-end one.
- **Majority monitor:** raises the shown σ when the selected filter disagrees with both others.
- **Follower at the exit:** no jump on the screen.
- **Rehearsing short outages in open sky could not tell the filters apart** and was dropped.
- **What the monitor can and cannot see:** it catches failures where the models differ (a tunnel
  pressure gradient, suspension pitch), late. It cannot see one in a part they share.
- **Low-end stop-and-go:** 89–335 m in v0.5.11, 16–21 m in five of six sets now and 14 / 20 m in
  validation (one set at 225 m: one run, seed 232, a wrong lock in the curve after the crawl, partly
  explained). The 10-state filter got a curve landmark (the yaw change over 12 s against the map's
  curve), its own branch decision (the same profile against the branch and the main tunnel), a
  one-way speed constraint ("a car does not reverse in a one-way tunnel", [ASSUMED]) and a speed
  row from the lateral force in turns (v·ω).
- **Recovery through the supervisor:** under 5 m a median 12–19 s after the exit on the Note10,
  17–28 s on a low-end phone.

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
   terms treated as white noise and a looser cruise test:
   - 10–25 m at the portal in steady and long tunnels;
   - consistent ((e/σ)² 0.2–0.6) and no late callouts;
   - stop-and-go fails for the ESKF (~0.9 km). At crawling speed the vehicle constraints see a pitch
     error only through v·δθ, so the cheap gyro's tilt leaks gravity into the speed. The supervisor
     selects the 10-state filter there (14–21 m in seven of eight sets); a road-grade pitch prior
     brought the ESKF to 88 m but stays off (not consistent, needs the map grade).
6. **Levelling from gravity** (an extra update) was tried and left off: it helps one scenario and
   hurts five.

## Limitations and next steps

- **Limitations:**
  - simulation only: the IMU noise is measured, while the rest of the sensor and map behaviour is
    assumed;
  - a rigid mount, one lane, one car; the magnetometer model is assumed;
  - the mount without an approach manoeuvre (magnetometer yaw only) is not consistent yet;
  - a causal stop detector tuned on the [ASSUMED] vibration model: conservative, it catches 80 % of
    the stop time and never fires while the car moves.
- **With real devices:**
  - static Allan runs;
  - recordings in Istanbul tunnels with the portals as checkpoints;
  - measuring map polyline errors and tunnel pressure gradients;
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
  - **Filter size and supervisor:** read 4 vs 16 states as a question of assumptions, asked for
    the middle size (built in a separate AI session) and the supervisor.
  - **Seeds:** read the 200-seed result as a selection effect and asked for an untouched
    validation tier.
  - **Re-association:** proposed it, and required the failure cases before it became the default.
  - **Comparison with another repository:** chose what to take (recovery and prompt metrics, the
    mount init, the lateral-force row, the permalink) and rejected BLE beacons in the tunnel.
  - **Mount:** designed the init (gravity tilt, magnetometer then acceleration-direction yaw) and
    chose a mount-yaw state in the 10-state filter too.
  - **Low-end stop-and-go:** corrected the agent's reading of the gyro noise (a profile artefact),
    designed the curve landmark and the 10-state branch test, and rejected a gate that assumed a
    straight main tunnel.
- **Detail:** REPORT §6 lists every intervention and the choices the agent made. It also lists
  the AI-side errors and what caught them: the author, AI reviews, the agent's own tests, or the
  consistency metric.
