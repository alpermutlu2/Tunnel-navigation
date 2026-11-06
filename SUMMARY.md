# Through the Tunnel: summary

Short version of [`REPORT.md`](REPORT.md), which has the full results, mechanisms and sources.
All numbers come from simulation. The main results are ranges over six independent sets of 50
seeds (0–299); tuning was done on seeds 0–49, features were designed on 50–299, and a further 100
seeds (500–599) were run once on the v0.5.14 configuration for validation (`reports/0.5.14/`) and
600–699 on v0.5.15 for the supervisor and the 10-state filter (`reports/0.5.15/`; each earlier
version spent its own validation set). Every parameter is tagged [MEAS] (measured, cited), [SPEC] (datasheet) or
[ASSUMED] (chosen here). Units: Code, filter and documents use SI (speeds in m/s); the simulator's screen shows speeds in km/h, as an app would. Scenario speeds are given in km/h, and where a document compares a speed with them it gives both.

## Problem

A phone in a car loses GNSS in a single-lane tunnel of 0.5–5 km. The navigation app must keep
showing the car, call out an exit inside the tunnel at the right moment, and recover cleanly at the
exit. The app has the map, so the real unknown is the **distance along the road**. A 22 m error
shifts the callout by 1 s at 80 km/h.

**The product at a glance** (what the screen shows: the supervisor of §Supervisor, 300 seeds, ranges of
per-set rms; per class eight nominal cases (S1–S6, 5 km, a 3 % 5 km tunnel) and, in brackets, four
with a tunnel pressure gradient; REPORT §4.16):

| | Note10 class (ESKF, k = 1) | low-end class (10-state, k = 0.7) |
|---|---|---|
| along-track error at the exit branch | 2.3–39 m (2.2–38 m) | 9.5–48 m (10–63 m) |
| along-track error at the portal | 1.5–16.5 m (1.4–6.6 m) | 7.5–25 m (13–26 m) |
| worst mean (e/σ)² at the portal, car takes the exit | 0.34 | 0.76 |
| callouts more than 1 s late | 16 of 2400, 14 on braking (1 of 1200) | 8 of 2400 (1 of 1200) |
| callouts more than 5 s early | 5 of 2400 (0 of 1200) | 96 of 2400, 59 on 5 km (38 of 1200) |
| branch shown, car takes it | right in 3600 of 3600 | right in 3600 of 3600 |
| car goes straight on (S1, S4, 5 km) | "main" in 900 of 900; 54–141 m at the portal, (e/σ)² up to 2.5 per set: **not consistent** | "main" never decided (900 of 900); drawn on the main tunnel; 22–95 m, consistent |

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
- **Map grade (v0.5.14):** where the map has the road's grade, it is a weak measurement of pitch;
  the tunnel's ventilation pressure gradient and the map's grade error are then states, so the
  barometer cannot be fooled by the gradient. Off where the map has no grade.
- **Stop detection:** zero-velocity updates at stops.
- **Driver model (cruise):** a stated assumption that the speed wanders slowly (σ 0.5 m/s =
  1.8 km/h, τ 60 s), not a measurement.
- **Map in the tunnel:** heading and lateral position against the route, with the map's own
  offset and length errors as states.
- **Callout rule:** announce when ŝ + k·σ_s reaches the callout point, so the callout errs early
  (k = 1; 0.7 on a low-end phone, whose σ is 4–6 times its error on flat tunnels).
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
- **Barometer:** a closed cabin; no ventilation pressure gradient in the nominal runs, ±25–30 Pa/km
  tested [ASSUMED].
- **Map:** lateral error σ 2 m and length error σ 0.5 %; a grade profile where the map has one, with
  a 10 % scale error σ [ASSUMED].
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
| along-track error at the exit branch, rms | 11–18 m | 28–39 m | 3.9–5.1 m | 26–38 m |
| along-track error at the portal, rms | 5.7–8.1 m | 4.6–8.0 m | 4.3–5.4 m | 12–16 m |
| cross-track error at the portal, rms (v0.5.10, 50 seeds) | 1.5 m | 1.3–1.9 m | 3.2 m | 2.1 m |
| callouts more than 1 s late (k = 1), of 300 | 0 | 2 (S4), 11 (S5) | 0 | 0 |
| shown position under 5 m after the exit, median per set | 13–17.5 s | 11–16.5 s | 12–16 s | 10–12 s |

The V-profile tunnel (S3) is 2.6–3.3 m at the exit branch and 1.7–2.1 m at the portal since the
map grade became a measurement (11.6–15.7 and 8.9–11.8 m before).

- **Consistent where the car takes the exit.** In these scenarios the filter's uncertainty covers its
  error at the portal: mean (e/σ)² ≤ 0.37 in every set, conservative, and every run within 2σ. That
  is a claim about these metrics, not every moment or every mismatch: a Note10 car going straight on
  is not consistent through the supervisor ((e/σ)² up to 2.35 on S1 and 2.54 on 5 km per set, not
  traced), nor is the mount without
  an approach manoeuvre (1.69).
- **Callout.** Without the rule, 0–66 % of callouts come more than 1 s late; with k = 1, 13 of
  3000. k = 1.3 cuts them to 5 of 3000 (2 of 1000 in validation), at 0.1–0.8 s more advance. Prompts at 1000, 500 and 200 m and "now" follow the same rule; when the shown 2σ is over
  100 m the prompt names no distance ("Exit A ahead, keep right").
- **Exit branch.** Recognised in 3000 of 3000 runs that take it. When the car goes straight on, the
  ESKF (Note10) reports "main" in 600 of 600; on a low-end phone the 10-state filter never decides
  "main" for such a car (below). Such a car has no curve to correct it: on stop-and-go, 5 of 300
  straight-on runs are more than 100 m off at the exit (34 in v0.5.13), until GNSS returns. Until
  "main" is decided (11–12 s on stop-and-go) the car was shown on the branch, 32–36 m off; since
  v0.5.15 it is drawn on the main tunnel until "branch" is decided (≤ 7 m), and a car that does take
  the branch is drawn on the main tunnel for those 3–4 s. The app must not re-route meanwhile
  [ASSUMED].
- **Exit.** The largest jump at the exit is under 6.3 m (p95); the shown position is under 5 m
  for good a median 10–17.5 s after the first fix.
- **Mount.** Estimated on the approach to ~0.6° per axis rms, it costs nothing measurable when the
  driver accelerates or turns before the tunnel. Without that, the yaw rests on the magnetometer
  (5.8°) and the portal error triples (17–25 m) and is not consistent: open.
- **Validation on 100 untouched seeds (500–599),** run once, nothing changed after: no run more
  than 100 m off; inside the ranges except single sets at the edge (S2 and 1 km portal, ≤ 0.2 m);
  7 of 1000 callouts more than 1 s late with k = 1; the pressure-gradient cases right in every run.
- **Branch window.** A validation run 110 m off (the turn outside the decision's fixed 100 m
  window) made the window grow with σ_s; the run is 3 m off and nothing else moved.
- **Seed tiers and method.** A callout claim of "1 in 500 late" made on 0–49 became 22 in 3000;
  a low-end failure count of 35 → 0 on 20 seeds was 35 → 6 on 300. 20 seeds to look, 300 for the
  number.
- **A failure found on new seeds and fixed.** In 2 of 200 S4 runs the filter reached the exit curve
  far behind the car and wrote the curve into yaw (~600 m off, small σ). Re-association along the
  arc removes them (0 in 200); an innovation gate stays as a safety net.

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
- **What the monitor can and cannot see:** it catches failures where the models differ (a tunnel
  pressure gradient, suspension pitch), late. It cannot see one in a part they share.
- **Low-end stop-and-go:** 89–335 m in v0.5.11, 12–15 m in all six sets now, with no run over
  100 m. The last failure (seed 232, −1.6 km) was traced: the barometer rows at a wrong pitch drove
  the speed down, then a speed row collapsed σ; the first explanation of it was wrong. The 10-state filter got a curve landmark (the yaw change over 12 s against the map's
  curve), its own branch decision (the same profile against the branch and the main tunnel), a
  one-way speed constraint ("a car does not reverse in a one-way tunnel", [ASSUMED]) and a speed
  row from the lateral force in turns (v·ω); in v0.5.14 the map grade as a pitch measurement.
- **Pressure gradient in the product:** at 30 Pa/km the app showed the car ~100 m off and on the
  wrong branch in every run (Note10); a low-end phone, never measured there before, was 181–405 m off
  (20 seeds). Now 1.4–6.6 m (Note10) and 12–25 m (low-end), right branch in every run (REPORT §4.15).
- **Recovery through the supervisor:** under 5 m a median 12–19 s after the exit on the Note10,
  17–28 s on a low-end phone.

## Where it breaks

1. **Stops and braking** give the largest errors. The callout rule turns them into early rather
   than late callouts.
2. **The driver model is an assumption.** Without GNSS a small real acceleration and a sensor bias
   look the same. A stricter "driver holds speed" model is 30–50 % better, but fails as soon as the
   speed wanders by ±1 km/h.
3. **A tunnel pressure gradient** of 30 Pa/km is read as a climb: 110–120 m error and a wrong branch
   decision. A gradient state alone costs normal roads: in a straight, constant-grade tunnel the
   barometer cannot tell the gradient from a pitch error. With the map's grade as a measurement of
   pitch (v0.5.14) the residual goes to the gradient and both products handle it (S3 at 30 Pa/km:
   2.3 m on a Note10). On a constant grade a map grade error and a gradient still cannot be told
   apart; the filter knows their sum and its σ says so. Without the map's grade the old behaviour
   returns. The gradient is [ASSUMED] and should be measured in a real tunnel.
4. **The map's own error limits what the map adds.** At twice the assumed map error the filter
   becomes optimistic, and on 5 km the map then does worse than no map.
5. **A low-end IMU** (MPU9250 class [MEAS]) works with its own tuning, with the fast gyro bias
   terms treated as white noise and a looser cruise test:
   - the ESKF alone, at k = 1 (v0.5.8, 30 seeds): 10–25 m at the portal in steady and long tunnels, consistent
     ((e/σ)² 0.2–0.6), no late callouts;
   - stop-and-go fails for the ESKF (~0.9 km). At crawling speed the vehicle constraints see a pitch
     error only through v·δθ, so the cheap gyro's tilt leaks gravity into the speed. The supervisor
     selects the 10-state filter there (12–15 m in all six sets in v0.5.14);
   - the price of the map grade on a low-end phone: a σ 4–6 times the error on flat tunnels, from
     the gradient state's assumed prior. With k = 1 callouts came 2–6 s early; since v0.5.16 the
     low-end phone uses k = 0.7 (5 km: 6 → 4 s early, 59 of 300 more than 5 s early), with 1 of 1200
     late in the gradient cases and 4 of 300 on the S-bend (k = 0.5 had 16). Accepted knowingly; the
     lasting fix is the gradient measured in a real tunnel;
   - a car going straight on: the 10-state filter never decides "main". Since v0.5.15 it pauses its
     map rows meanwhile (5 km: 753 → 77 m) and the car is drawn on the main tunnel.
6. **Levelling from gravity** (an extra update) was tried and left off: it helps one scenario and
   hurts five.

## Limitations and next steps

- **Limitations:**
  - simulation only: the IMU noise is measured, while the rest of the sensor and map behaviour is
    assumed;
  - a rigid mount, one lane, one car; the magnetometer model is assumed;
  - the mount without an approach manoeuvre (magnetometer yaw only) is not consistent yet;
  - the map-grade layer needs a grade profile in the map; its shape errors are not simulated, and
    with a Note10 the 10-state filter stays without it (overconfident in stop-and-go, source open);
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
  - **Map grade (v0.5.14):** had the low-end gradient failure measured, asked for the map grade as a
    source instead of another filter, made the map's grade error a state, required that a map
    without grade give the old filter exactly, and set the one-way constraint's form per filter on
    the condition that both forms ran on 300 seeds.
  - **"Not yet" and callout timing (v0.5.15):** chose "main first" for the drawn point with the
    follower's limit across, approved pausing the 10-state filter's map rows, and set the low-end
    callout margin by an acceptance rule he wrote before the 300-seed run (the agent's first rule
    could not fail); on its borderline result he chose k = 0.7 (v0.5.16).
  - **Stress tests (v0.5.16.1–17):** found the remaining gaps himself by stress-testing the live
    simulator and watching its playback: a 16-state car spinning at stops (a drawing error), two
    places where the page drew an estimate with the truth's help, mixed speed units, and a
    cross-track error held 16–27 s after the exit under three times the assumed map errors. He
    chose the fix for the last one (cap only the along-track correction, §4.17 of the report).
- **Detail:** REPORT §6 lists every intervention and the choices the agent made. It also lists
  the AI-side errors and what caught them: the author, AI reviews, the agent's own tests, or the
  consistency metric.
