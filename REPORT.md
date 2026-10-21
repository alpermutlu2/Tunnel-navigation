# Through the Tunnel: write-up

Pointr case study. A phone in a car estimates its position through a GNSS outage in a road
tunnel. The navigation app has to keep showing the car, call out an exit inside the tunnel at the
right moment, and recover cleanly when the car comes out.

All results come from simulation: 50-seed Monte Carlo runs (`tools/evaluate.py`,
`reports/0.5.3/`) unless a section says otherwise. Every parameter is tagged [MEAS] (measured,
cited), [SPEC] (datasheet) or [ASSUMED] (chosen here); see `tunnelsim/profiles.py` and
`DESIGN.md` §2. The simulator runs interactively (`python tools/serve.py`) or one page per run
(`tools/simulate.py`); the README has the commands.

## 1. Framing

The app has the map and the tunnel has one lane, so the unknown is the **along-track distance
s**. The exit callout depends only on s: an error δs shifts the callout by δs/v, and 22 m is 1 s
at 80 km/h. The core question is therefore how well we know the speed, and for how long.

A measured error budget set the design (`DESIGN.md` §4, 200 seeds, 180 s):

- **Free pitch is unusable.** The Galaxy Note10 gyro (LSM6DSR, Allan terms [MEAS], Suvorkin et
  al. 2024) integrated freely gives 1.2–1.5 km of error through gravity leakage.
- **With perfect attitude, accelerometer bias drift leaves ~30 m.** White noise adds about 1 m.
- **Calibrating the bias** needs about 120 s of GNSS before the portal, with some speed changes.
- **Constant velocity (CV)** from the entry speed beats the INS at constant speed (9.5 vs 19 m)
  and fails as soon as the driver brakes (0.6–2.6 km).

## 2. Approach

The estimator is an error-state Kalman filter (`tunnelsim/eskf.py`). It runs on 100 Hz IMU data.
Its 16 core states are position, velocity, attitude, accelerometer bias, gyro bias and barometer
offset. Cruise mode adds 2 states and the map layer adds 2 more.

| Layer | Role |
|---|---|
| GNSS position and velocity; χ² gate; R inflated for correlated noise; small bias random walks | Calibrate biases and attitude before the outage (robust over optimal) |
| Non-holonomic constraint (NHC): no lateral or vertical body velocity | Holds pitch. Lateral row skipped while \|ω_z\|·v > 0.3 m/s², because of sideslip in curves (§4.3) |
| Barometer, 1 s average modelled at the window centre | Vertical channel and pitch |
| ZUPT from an IMU stop detector | Stop-and-go traffic |
| Cruise prior: speed = v_ref + d, d Gauss–Markov (σ 0.5 m/s, τ 60 s) | A stated model of the driver, not a measurement (§4.2) |
| Map in the tunnel: 1 Hz heading and cross-track against the route polyline; map lateral offset (σ 2 m) and length error (σ 0.5 %) as states; paused around the diverge until the in-loop branch decision | Heading and lateral position; along-track information where the road curves, bounded by the map's own uncertainty (§4.5) |
| Callout when ŝ + 1·σ_s reaches s_exit − 300 m | Errs early: a late callout can make the driver miss the exit |
| Re-acquisition: 3 s velocity only, decaying position R, ≤ 2 m per update | No jump at the exit, even with multipath |
| Gate-lock recovery, adaptive noise | Safety net; never fires in the nominal scenarios (§4.6) |

The estimator never imports truth objects; a test enforces this (`docs/contracts.md`).

## 3. Results

In every scenario the route leaves the main tunnel through a 20° branch inside it, and the
callout comes before that diverge. "At diverge" is the straight-tunnel result. "At portal" is
the result when the car comes out.

50 seeds per scenario; `reports/0.5.3/evaluation_50.json`.

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | S1-1km | S1-5km |
|---|---|---|---|---|---|---|---|---|---|---|
| | 3 km straight | S-bend | V-profile | stop-and-go | braking | wander ±1 km/h | ±2 km/h | ±4 km/h | 1 km | 5 km |
| along-track at diverge, rms [m] | 12.1 | 3.5 | 12.1 | 31.1 | 25.0 | 13.4 | 13.3 | 13.2 | 4.8 | 24.6 |
| along-track at portal, rms / p95 [m] | 5.6 / 10 | 4.1 / 7 | 9.0 / 17 | 5.6 / 12 | 7.5 / 15 | 6.0 / 10 | 6.2 / 11 | 5.6 / 11 | 2.5 / 5 | 13.7 / 25 |
| cross-track at portal, rms [m] | 1.5 | 3.2 | 2.0 | 1.3 | 1.9 | 1.5 | 1.5 | 1.5 | 1.4 | 2.1 |
| mean (e/σ)² at portal | 0.13 | 0.23 | 0.18 | 0.11 | 0.27 | 0.15 | 0.16 | 0.13 | 0.22 | 0.25 |
| callout timing, filter only (k = 0): mean / > 1 s late | +0.1 / 2 % | +0.1 / 0 % | +0.2 / 4 % | +0.2 / 30 % | +0.9 / 40 % | +0.1 / 2 % | +0.1 / 2 % | +0.0 / 2 % | -0.0 / 0 % | +0.4 / 24 % |
| callout as announced (k = 1): mean [s] (− early) / late / > 1 s late | -0.8 / 2 % / 0 % | -0.3 / 2 % / 0 % | -0.9 / 4 % / 0 % | -2.6 / 4 % / 0 % | -0.8 / 16 % / 2 % | -0.8 / 2 % / 0 % | -0.8 / 2 % / 0 % | -0.9 / 4 % / 0 % | -0.2 / 12 % / 0 % | -1.6 / 4 % / 0 % |
| branch reported (car takes it) / main reported (car goes straight) | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % |
| largest exit step, p95 [m] | 3.4 | 5.5 | 4.0 | 3.5 | 3.7 | 3.4 | 3.3 | 3.6 | 4.9 | 4.3 |
| not < 10 m 60 s after the exit | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1 |

- **Showing the car.**
  - Along the road, in a straight tunnel at steady speed: 12–13 m after 1.5–2 min, 25–31 m
    with braking or stops.
  - Across the road: 1.3–3 m, which is the map's own lateral uncertainty; the filter cannot do
    better than the map here. Without the map layer, the cross-track error was 26 m on S1 and 106 m at 5 km, because heading
    is not observable in a straight tunnel.
  - The exit curve brings the portal error down to 2.5–14 m.
  - The filter is consistent: mean (e/σ)² 0.11–0.27.
- **Callout.** "The right moment" is read here as *never late, and only as early as the position
  uncertainty requires*: a late callout can make the driver miss the exit, an early one costs
  reaction time. The table gives both the filter's own timing (k = 0, how good the estimate is) and
  the announced callout (the product rule ŝ + k·σ_s, k = 1). With k = 1, one callout in 500 runs
  is more than 1 s late (S5), and 0–16 % are late at all, mostly by a fraction of a second. On
  average callouts come 0.2–2.6 s early, most on S4. Without the rule, 30 % (S4) and 40 % (S5) of
  callouts were more than 1 s late. Why k = 1: the filter's σ_s is about twice the real error (mean
  (e/σ)² 0.1–0.3), so k = 1 is in effect a ~2σ margin. k = 1.65 gives no late callout at all but
  comes up to 4.4 s (S4) early; at 50 km/h that is ~60 m, and "Exit A in 300 m" would then really
  mean 360 m. k has to be chosen again once σ is calibrated on real data.
- **Branch.**
  - Car takes the branch: the in-loop decision reports the branch in 500 of 500 runs, 2–3.5 s
    after the diverge.
  - Car goes straight on (negative control, map with the branch): "main" in 500 of 500 runs. The
    portal error then stays at the no-map value (S1 18 m, S4 45 m: no exit curve), which shows that the map's
    along-track gain comes only from curves.
- **Recovery.** The largest step at the exit is ≤ 6 m (p95). The estimate is within 10 m of truth
  60 s after the exit in 499 of 500 runs.

**Tunnel length** (diverge at min(0.73·L, L − 200 m) [ASSUMED]). At the diverge: 4.8 m after
1 km, 12 m after 3 km, 25 m after 5 km. The error grows faster than linearly, since bias drift
goes as ~t² (`reports/0.5.1/error_vs_distance.png`). At 0.5 km the callout point is still under
GNSS.

## 4. Where it breaks, and why

**4.1 Stops and braking.** ZUPT resets speed at stops, but each stop and start is integrated, and
the bias acts during long stops. S4 and S5 have the largest errors (25–31 m at the diverge). The
k·σ callout turns that into early rather than late callouts: 2.6 s early on average.

**4.2 Driver behaviour is an assumption.** Without GNSS a small real acceleration and an
accelerometer bias look the same (ã = a + b_a + n). Three speed-hold forms were tried:

1. **Absolute hold:** overconfident, (e/σ)² 6–33. The pseudo-measurement's error is the state
   error itself.
2. **Cloned speed-increment constraint:** consistent and 30–50 % better, but it fails once the
   speed wanders by ±1 km/h.
3. **Gauss–Markov wander (default):** consistent in every case, but it keeps only a small part of
   the gain.

The gap between (2) and (3) is the value of "the driver holds speed exactly". It does not come
from the sensors.

**4.3 Sideslip violates the lateral NHC.** In the exit curve the car slips by β ≈ 0.6°
[ASSUMED]: a 0.23 m/s lateral velocity against NHC σ 0.1 m/s, with the same sign throughout the
curve. After α degrees of turn, the lateral row sees the old speed error weighted by sin α. The
sideslip is amplified by about 1/sin α into a speed error, and every seed shows a −20 m transient
(`reports/0.5.0/nhc_trace_S1.png`). Inflating R only slows this down; skipping the row in turns
removes it.

**4.4 Tunnel pressure gradient.** Ventilation can create a longitudinal pressure gradient that
the barometer reads as climb. Its size is [ASSUMED]. At 30 Pa/km the filter reads it as pitch:
the error at the diverge is 110–120 m, (e/σ)² 9–14, and the branch decision is wrong. A gradient
state fixes that case (19 m) but costs accuracy on normal roads (S5 23 → 109 m). The reason is
structural: on a straight tunnel with a constant grade, a constant pitch error and a gradient
both give a height error linear in s, and the barometer cannot tell them apart. The state is off
by default; the gradient should be measured in a real tunnel.

**4.5 What the map adds depends on the map.** Map errors drawn per seed from the prior (1×) and
from twice the prior (2×), 30 seeds (`reports/0.5.2/map_random_stress_30.json`):

| | exact map | 1× (2 m, 0.5 %) | 2× (4 m, 1 %) | no map |
|---|---|---|---|---|
| S2: portal rms / (e/σ)² | 4.1 m / 0.23 | 6.7 m / 0.61 | 11.0 m / 1.7 | 17 m |
| 5 km: portal rms / (e/σ)² | 13.5 m / 0.24 | 23.1 m / 0.71 | 39.3 m / 2.1 | 34 m |

A 0.5 % length error draws the 5 km exit curve 18 m off, and curve matching can only be as good
as the curve's position. Treating map errors as white noise gave (e/σ)² 1.7–1.9 already at 1×.
The fix was map-error states, including the length error's lateral effect after a turn. At 2×
the filter is still somewhat optimistic, and at 5 km the map then does worse than no map. The σ
values are [ASSUMED]: OSM tunnel geometry is traced from imagery, so a 1 % error over 5 km is
plausible. They must be measured on real maps.

**4.6 Low-end IMU.** An MPU9250-class IMU [MEAS] has 30–80× the Note10's gyro bias instability.
The filter becomes overconfident and the χ² gate locks out GNSS before the tunnel (3.5 km at the
exit). Gate-lock recovery brings this to 1.3 km, but (e/σ)² ≈ 2000. The design does not cover
low-end phones: they need an IMU-class check at start-up and a fallback to CV and map cues.

**4.7 Assumptions that did not matter** (S1, 30 seeds): GNSS coasting into the tunnel; optimistic
reported accuracy; a 1–3° mount error (absorbed by the lumped accelerometer bias while the mount
stays rigid); the shape of the bias model.

## 5. Limitations and next steps

- **Simulation only.** The IMU noise terms are measured. The time shape of the noise, vibration,
  sideslip, barometer, map and GNSS behaviour are [ASSUMED].
- **Simplified map errors.** Map errors are a lateral offset and a length scale. Shape errors,
  missing curves and a wrong diverge position are not simulated. The truth car drives on the lane
  centre.
- **The ZUPT detector is offline** (forward-backward filtering). A causal version is similar at
  the exit and worse for the callout.
- **Narrow setting.** One IMU class, a rigid phone mount, a single lane.
- **Next, with real devices:**
  - static Allan runs to check §3 of `DESIGN.md`;
  - recordings in Istanbul tunnels, with the portals as checkpoints;
  - measuring the map's polyline error and the tunnels' pressure gradient;
  - a causal stop detector;
  - an IMU-class check at start-up.

## 6. AI usage

The code, tests, experiments and document drafts were written by an AI coding agent: Claude
(Anthropic), working in a cloud workspace on this repository.

- **Decisions and reviews.** The author (Alper Mutlu) made the decisions: problem framing,
  assumptions, every filter-design and error-budget choice, scenarios and acceptance criteria.
  Before each piece of code, the agent proposed options with formulas, and the author approved,
  changed or rejected them (`docs/decisions/`). The author also had the repository reviewed by
  other AI assistants and models; the last reviews came from Claude Fable 5.1. He filtered those
  reviews and passed on what he accepted (`docs/reviews/`). The cross-track finding, the map
  layer's design and the random map stress came out of that loop.
- **Process controls**, used because generated code can be confidently wrong:
  - parameter tags;
  - truth/estimator separation enforced by a test;
  - consistency (e/σ)² as an acceptance criterion, not only accuracy;
  - a CHANGELOG entry and a metrics snapshot for each version, with every changed metric
    explained;
  - decisions and reviews kept as commits;
  - release tags set by the author.
- **Errors on the AI side that this process caught:**
  - In reviews: an attitude-error sign; an oracle bound presented as achievable; an
    observability claim stronger than the model supports; a cross-track error that no metric
    measured.
  - By consistency metrics: the overconfident absolute speed hold; a 0.5 s barometer timestamp
    lag; the sideslip/NHC transient; map errors treated as white noise; a map stress test that
    only relabelled s and so tested nothing.
  - By tests and Monte Carlo: an innovation sign; a barometer index that never advanced; a stop
    detector that fired at launch.
