# Changelog

Each version has a git tag and a metrics snapshot in `reports/<version>/metrics.json`
(`python tools/snapshot.py`). Compare two versions with
`python tools/snapshot.py --diff <old> <new>`.

Design decisions and their reasons live in `DESIGN.md` (decision log, section 12) and
`docs/decisions/`. This file records what changed in the code and how the numbers moved.

## v0.5.13 (2026-10-30): mount estimated in open sky, lateral-force speed row, recovery and prompt metrics

From a comparison with another case-study repository; the owner chose items 1, 2, 3, 4 and 6 and
rejected BLE beacons (item 5: a tunnel with beacons is not a fair assumption). Validation is
400–499, run once (`reports/0.5.13/validation_400-499.json`, `ekf10_validation_400-499.json`,
`supervisor_validation.json`); 300–399 was v0.5.12's and was not used here. REPORT §4.14.

- **Mount alignment** (`tunnelsim/mount_init.py`, owner's design): tilt from the gravity mean over
  quiet GNSS epochs, yaw from the magnetometer (TRIAD against the GNSS course and grade), then
  yaw from the acceleration direction (grid search of R_z(δ) against the GNSS acceleration, once
  Σ|a_h|² ≥ 3 (m/s²)²). First 90 s of open sky. `filter_mount()` in `experiments.py`; every
  experiment and `tools/simulate.py` now use the estimated mount by default (`mount="known"` gives
  the old behaviour; `--mount-err` forces the given error).
- **Magnetometer** (`tunnelsim/magnetometer.py`, truth side, open sky only): 47 µT, inclination
  56°, declination 6°, the car's own field 3 µT, soft iron 2 %, noise 0.4 µT at 25 Hz, all
  [ASSUMED]. DESIGN A4 changed accordingly.
- **Mount-yaw state** in the ESKF (`EskfConfig.mount_yaw_sigma`, set from the init: 6° after the
  magnetometer only, 1° after the acceleration step [ASSUMED]) and, owner's option (a), in the
  10-state filter (`Ekf10Config.mount_yaw_sigma`).
- **Lateral-force speed row** in the 10-state filter (`Ekf10Config.lateral_speed`, on): in turns
  f_y − b̂_y = v(ω − b_gz) − ψ_m f_x, threshold |ω| > max(0.02, 4σ_ω,1s) and (v·σ_ω,1s)² in R. The
  fixed-threshold first form (710 m, 12 failures in 20 low-end S4 seeds) was not kept.
- **Recovery and prompt metrics** (`tunnelsim/navmetrics.py`): time to 10 m and 5 m for good after
  the first fix, and the error shown meanwhile; prompts at 1000/500/200 m and "now"
  (D = max(30 m, 2 s·v̂) [ASSUMED]) with early/late, missed and vague (2σ > 100 m [ASSUMED]). In
  `eskf_experiment` (`recovery_time`, `prompts`), `eval_sets` (`rec_*`, `prompt_*`) and the
  supervisor metrics.
- **Simulator:** the view in the address (`#tab=…&t=…&show=…&nav=…&k=…&lead=…&max=nav`); the
  callout says "Exit A ahead, keep right" without a distance when 2σ > 100 m.
- **Tools:** `eval_sets.py` variants `known`, `cup`, `noexc`, `cup-known`, `noexc-known`, `--only`,
  resume, `--validation` 400–499; `ekf10_sets.py` variants v0.5.12 / lateral / v0.5.13 and
  `--only-final`; `supervisor_eval.py` resume and validation 400–499.
- **Tests:** two v0.5.11/0.5.12 mechanism tests pinned to `mount="known"` (they test the mechanism,
  not the mount); a new mount test (cup holder, S1, error < 2.5°).

How the numbers moved (0–299, six sets; every change from the estimated mount unless stated):
- ESKF Note10, against v0.5.12 (true mount): within ±0.7 m of the v0.5.12 ranges on every
  scenario except two. S4 portal 4.7–6.1 → 4.5–7.9 m (upper end; seed 271 below). S2 (S-bend)
  diverge 3.5–4.6 → 4.2–5.2 m: S2 has the most curve before the diverge, where a residual mount
  yaw acts through the lateral NHC row; the true-mount A/B was run on S1 and S4 only, so on S2 that
  reading is not measured. 5 km portal 12.3–16.2 → 12.9–16.7 m. Worst (e/σ)² ≤ 0.38. Same-version A/B, true mount: S1 5.5–6.9 m, S4 4.7–6.1 m. Mount error rms
  ≤ 0.7° per axis, yaw from mag+accel in 600 of 600.
- Branch: 3000 → 2999 of 3000 (S4 seed 271 decided "main", 8.3 → 41.1 m).
- Callouts > 1 s late, k = 1: 22 → 20 of 3000 (S5 17 → 15). Validation 6 of 1000, k = 1.3 none.
- Straight-on S4 runs > 100 m: 39 → 34 of 300; 9 of 100 in validation.
- No approach manoeuvre (magnetometer yaw, 5.8° rms): S1 portal 16.6–25.4 m, (e/σ)² 1.69: open.
  The same with the true mount: 5.1–6.9 m.
- 10-state alone, low-end S4: 94 → 93.3 m rms (22.0 → 19.6 m without failures), 2 → 1 runs
  > 100 m (seed 200 fixed by the lateral row; seed 232 −1566 → −1581 m), (e/σ)² 5.1 → 5.0. With the
  true mount the lateral row alone gave 27.9 m (seed 232 −340 m). Note10 S2 3.5 → 4.4 m ((e/σ)²
  1.13 → 0.98; 6.0 m and 3.29 without the mount-yaw state). Low-end 5 km 16.0 → 18.0 m.
  Validation low-end S4 19.0 → 17.4 m.
- Supervisor, low-end S4 portal: 20.4–23.9 → 16.1–20.5 m in five sets (225 m in 200–249, seed 232);
  validation 14.4 / 20.0 m. Low-end S2 9.9–11.6 → 9.0–9.7 m; Note10 S4 4.7–6.1 → 4.5–7.8 m; the rest
  ±1 m. Exit step ≤ 1.1 m (Note10), ≤ 0.34 m (low-end), 7.0 m (pressure, S3).
- New metrics, ESKF Note10: to 5 m median 10–18 s per set, 12–34 of 300 not within 60 s; "now"
  prompt 2–48 m early on average, missed ≤ 1 of 50 (S4). Supervisor: to 5 m 12–19 s (Note10),
  17–28 s (low-end), 36–52 s (pressure).
- Snapshot (`metrics.json`, Note10 ESKF, 0–49, now with the estimated mount): every scenario moves,
  because the filter's mount changed. Diverge rms within ±0.65 m (S1 11.71 → 11.53, S4 30.68 →
  30.03, 5 km 21.09 → 20.65) except S2 3.28 → 3.93 m (the S-bend, as above); portal within ±0.45 m
  (S1 4.38 → 4.32, S4 4.79 → 4.62, 5 km 13.39 → 12.95, S6L 5.01 → 5.18); (e/σ)² within ±0.04
  (largest 1 km 0.253 → 0.216). Exit step p95 within ±0.25 m. Tests 200 → 201.

## v0.5.12 (2026-10-26): branch window, 10-state curve landmark and branch decision, one-way speed

Seed tiers: 200–299 moved to feature design (the branch window came from seed 247); validation is
300–399, run once (`reports/0.5.12/validation_300-399.json`, `ekf10_validation_300-399.json`,
`supervisor_validation.json`). Main results are over six sets (0–299). Method: 20 seeds to look,
300 for the number.

- **Branch window** (`EskfConfig.map_pre_sigma_k = 2`): the decision window starts
  max(100 m, 2σ_s) before the diverge. S4 seed 247 −110.1 → −3.4 m; S4 200–249 16.7 → 6.1 m; other
  S4 sets ±0.1 m; S1-5km 0–49 12.78 → 12.94 m; the rest bit-identical. Straight-on control
  unchanged; its "not yet" time and the shown point's distance from the road are now measured
  (S4: 12.9 s, 38 m mean largest; `eskf_experiment()["undecided"]`).
- **10-state curve landmark** (`Ekf10Config.curve_landmark`, owner's design): the yaw change over
  12 s against the map's heading-change profile for candidates in ŝ ± 3σ_s, with the sensor's
  short-term gyro terms [MEAS] in R (not the profile's folded 0.063 rad/√s); together with an
  s-guard on the curve rows while σ_s exceeds the shortest clothoid. One switch.
- **10-state branch decision**: the same profile against the branch (J_kol) and the main tunnel
  (J_ana); "branch" turns the landmark on, "main" switches the map rows to the main tunnel. The
  supervisor shows the selected filter's decision (low-end S4: right in 98–100 % per set, the
  ESKF's 2–28 %).
- **One-way speed constraint** (`no_reverse`, both filters, [ASSUMED]): while v̂ < 0 in an outage,
  z = 0 − v̂, R = (0.5 m/s)². The owner's tight form R = (0.02 m/s)² was measured and rejected
  (low-end S4 10-state 12 runs > 100 m against 2; ESKF Note10 S4 5.0 → 6.5 m on 10 seeds).
- **Re-association R floor k·v²** (`map_reassoc_slip_k`, off): identical on S1, S4 and 5 km.
- Removed: the 10-state re-association prototypes (by yaw, by yaw rate; neither fired).

How the numbers moved:
- 10-state alone, low-end S4, 300 seeds: 224 → 94 m rms (29.1 → 22.0 m without failures),
  35 → 2 runs > 100 m, (e/σ)² 26 → 5.1. Note10 S4 25.5 → 16.5 m, Note10 5 km 22.3 → 9.4 m.
  Validation: low-end S4 134 → 19 m, 13 → 0 runs > 100 m.
- Supervisor, low-end S4: 89–335 m → 19–24 m in seven of eight sets (225 m in 200–249, seed 232).
  Note10 cases within 0.2 m except S4 200–249 16.8 → 6.0 m (seed 247). Low-end others: 5 km
  −2.8…+5.1 m per set, S5 −4.0…+0.1 m, the rest −1.7…+0.4 m.
- ESKF Note10: the speed constraint moves only S4, by ≤ 0.05 m per set; low-end ESKF S4 stays
  ~900 m (its failure is not reversing).
- Callouts, k = 1: 22 of 3000 > 1 s late (11 of 2000 over 0–199 before); validation 13 of 1000.
- Straight-on S4 runs > 100 m: 39 of 300 (the speed constraint added one in 0–49); 11 of 100 in
  validation.
- Snapshot (`metrics.json`, Note10 ESKF): only S4 moves, by the speed constraint: portal 4.86 →
  4.79 m, diverge 30.58 → 30.68 m, (e/σ)² 0.082 → 0.080.
- Tests 195 → 200 (`tests/test_v0512.py`).
- Simulator, driver's view: checkboxes for the 16-, 10- and 4-state filters and the constant-speed
  reference as dots on the road, or pills with the gap when out of view; the speed shown is the
  selected filter's (was the ESKF's). Deck updated to v0.5.12 (new slide: the curve landmark).
- In-browser mode opened as a file (file://): the browser refused `py/worker.js` ("cannot be accessed
  from origin 'null'"); the page now embeds the worker and the package and starts the worker from a
  Blob. Over http(s) nothing changes.
- Driver's view in full screen: the callout bubble's text outgrew its fixed 34 px box (font from the
  width, box from nothing). Every text, bar and marker now scales with one unit from the short side,
  and the bubble's height and width follow its font.
- Closing review (v9), documents only: the low-end straight-on cost (10-state "not yet" through the
  tunnel, 302–439 m at the portal) in §4.13 and §5; the supervisor's straight-on control stated as
  filter-level only; the "> 5 s early" column in §4.12 (low-end S4 60–74 %); the monitor as a
  consistency warning; recovery time and coverage as next checks (§5).

## v0.5.11 (2026-10-25): three filters, supervisor, map re-association, seed tiers

Review v8 patch (same version, no metric changed):
- Docs: the branch statement (one validation run stayed undecided, none reported the branch);
  straight-on S4 runs > 100 m off at the portal (28 of 200, 10 of 100 in validation) added to §3
  and §5; §5 points to §4.12; three §6.3 rows (v1 negative control, the scale-state sign, the
  "1 in 500" selection effect); stale SUMMARY item removed.
- Simulator note: the app error is along the map, the 16-state error along the true road; they
  differ by the cross-track error's projection (≤ 1 m nominal, ~10 m under the pressure gradient).
- Code comment on the re-association's `br_dec == 1` restriction and the double-counted prior.
- Site: a low-end S4 (stop-and-go) tab.
- Tests 191 → 195 (`tests/test_supervisor_reassoc.py`): S4 seed 63 fixed by re-association;
  straight-on control unchanged with it on or off; monitor modes; display follower.

- **Seed tiers.** Development 0–49 (all tuning so far), feature design 50–199, validation 200–299
  (final configuration, run once). Main results are now ranges over four sets of 50
  (`tools/eval_sets.py`); the development set gave the low end (S1 diverge 12.1 vs 13.0–16.9 m).
  The callout claim "1 in 500 late" (k = 1, 0–49) is 11 in 2000 over 200 seeds (S5 0–10 % per
  set); k = 1.3 halves it (none on 0–199, 3/100 on S5 in validation; k unchanged, row added).
- **Validation 200–299** (final configuration, once; `validation_200-299.json`): inside the ranges
  except S4 seed 247 (−110 m: a −220 m crawl error put the turn outside the 100 m branch window,
  branch decided "main") and S5 late callouts 7/100 (k = 1), 3/100 (k = 1.3).
- **Map NIS gate** (`map_nis_gate`, default on, owner). χ² 99.9 % on the map heading/cross-track
  rows. On its own: S4 failures 2 → 3 (−694/−571 m → +104/+117/+121 m), worst (e/σ)² 44 → 1.2.
  Branch deferral (σ_s > clothoid length) rejected: branch decided in 18 %, portal ~33 m.
- **Map re-association** (`map_reassoc`, default on, owner's proposal). Candidates in ŝ ± 3σ_s
  scored by the heading residual; an along-track pseudo-measurement to the best one; only on a
  decided branch (v1 broke the straight-on control: 92–170 m); with the map length-error state
  (v2 was over-confident with a 2× map error). Four sets: S4 failures 0, portal 4.7–6.0 m, p95
  ≤ 11 m, (e/σ)² ≤ 0.13; S5 portal 7.5–9.7 → 6.5–8.1 m; 5 km 13.7–17.1 → 12.3–16.1 m; no
  scenario worse; straight-on control unchanged. Failure cases (`failure_cases_100.json`):
  pressure gradient unchanged from the gate (no re-association on a "main" decision), 2× map error
  18–27 m with (e/σ)² 1.6–2.4. Per seed (`perseed_*.json`): most worsened S4 seed 1, 1.1 → 12.5 m.
- **10-state filter** (`tunnelsim/ekf10.py`, built in a separate AI session under the owner's
  review; harness in `experiments.ekf10_experiment`, tests). Three-filter comparison
  (`tools/compare_filters.py`).
- **1D baseline:** map-grade error stress (`grade_scale`): 10 % off → S3 portal 85–117 m,
  (e/σ)² 290–710.
- **Pitch prior** for low-end IMUs (`pitch_prior_*`, off) and the trusted-stop ZUPT gate
  (`zupt_trust_s`, off): A/B in `reports/0.5.11/pitch_prior_ab_30.json` (REPORT §4.10).
- **Supervisor** (`tunnelsim/supervisor.py`): class rule, majority disagreement monitor, display
  follower; open-sky rehearsals measured and dropped (`supervisor_{dev,heldout}.json`,
  `supervisor_sets_*.json`).
- **Fixes:** route-grid search windows past the end of the route (MapView, 10-state, 1D).
- **Simulator:** 4-, 10- and 16-state traces and the app's selected filter with show toggles;
  re-association and disagreement in the banner; results-table rows for each.
- **Docs:** REPORT §3, §4.9–4.12, §5, §6; SUMMARY; README; deck.
- Snapshot `reports/0.5.11/metrics.json` vs 0.5.10 (snapshot seeds): portal improves on S4
  (4.96 → 4.86 m), S5 (8.6 → 7.6 m, (e/σ)² 0.36 → 0.28), S6/S6s (−0.1 m), 5 km (13.6 → 13.4 m);
  diverge unchanged everywhere (re-association acts after the branch); tests 186 → 191, all pass.

## v0.5.10 (2026-10-25): causal stop detector by default; evaluation re-run

- `EskfConfig.zupt_causal` False → True (owner 2026-10-25): the stop detector no longer filters
  forward-backward, so it uses no future samples; the filters start from their steady state.
  The 1D baseline shares the detector. New fields `zupt_band_order` (4) and `zupt_lp_order` (2).
- Order A/B (`tools/zupt_order_ab.py` → `reports/0.5.10/zupt_order_ab.json`). Owner's choice:
  order 4, deliberately conservative.
  - It catches 80 % of the S4 stop time, never > 0.5 s false.
  - Twice the order restores the offline engine rejection, but then takes a 3 m/s crawl as a
    stop in 3/50 seeds: S4 portal 91 m, (e/σ)² 7.7.
  - Order 3 is slightly better but sits next to the cliff at order 2 (S4 334 m).
  - Detector tests now check both the offline and the causal behaviour.
- 50-seed evaluation re-run at this version (`reports/0.5.10/evaluation_50.json`, with the
  straight-on control); the report now cites it instead of `reports/0.5.3/`. Only S4 changed
  (the only scenario with stops):
  - diverge 31.1 → 34.7 m; portal 5.6 → 5.8 m; (e/σ)² 0.11 → 0.12;
  - k = 1 callout −2.6 → −2.3 s, none > 1 s late; k = 0 > 1 s late 30 → 42 %;
  - exit step p95 3.5 → 4.0 m; straight-on control portal 45 → 51 m.
  Every other scenario is identical except the main `callout` block of the results file, which
  is now recorded at k = 1. In 0.5.3 it was recorded at k = 1.65, the default before v0.5.4,
  while the report quoted the `callout_k["1.0"]` row (found by an external AI comparison).
- 1D comparison re-run (`reports/0.5.10/reference1d_30.json`). S4 only:
  - Note10, 1D: 34.8 / 91.4 → 34.1 / 89.0 m, (e/σ)² 864 → 818;
  - Note10, ESKF: 30.3 / 4.8 → 33.7 / 4.7 m;
  - low-end, 1D: 64.8 / 80.2 → 64.0 / 79.7 m;
  - low-end, ESKF: 603 / 857 → 638 / 894 m, (e/σ)² 155 → 183.
- REPORT §3, §4.1, §4.6, §4.9, §5 (detector A/B table), §6.3; SUMMARY; deck and simulator rebuilt.
- Snapshot `reports/0.5.10/metrics.json` vs 0.5.9: only S4 changed (snapshot seeds). Diverge rms
  26.2 → 30.6 m, portal 5.4 → 5.0 m, (e/σ)² 0.10 → 0.09, callout rms 2.6 → 2.3 s, exit step p95
  3.4 → 3.5 m; tests 186 passed.

## v0.5.9 (2026-10-24): 1D map-constrained EKF baseline

- New `tunnelsim/reference1d.py` (owner 2026-10-24): 4-state 1D EKF [s, v, b_a, b_g], map as a hard
  constraint (curvature, grade), curvature-gyro update at 1 Hz with gyro BI in R, GNSS projected on
  the route, the ESKF's ZUPT detector, R factors, P0 and IMU-class profile; re-lock phase after the
  tunnel. Re-implemented from a description of a common design; a reference, not used by the app.
- `experiments.ekf1d_experiment`, `tools/compare1d.py` → `reports/0.5.9/reference1d_30.json`
  (30 seeds, paired with the ESKF; Note10 and low-end; map with/without grade; mount 2° off).
  Diverge within 2 m of the ESKF on straight tunnels; portal S4 91 vs 4.8 m (suspension pitch;
  4.9 m without it), 5 km 75 vs 14 m (non-linear curve update moves speed, σ_s collapses);
  (e/σ)² 5–864; S3 without map grade 3.4 km. Low-end: better at the diverge everywhere, S4 80 vs
  857 m. REPORT §4.9, §5 (smaller filter not built), §6; SUMMARY.
- Simulator: 1D trace on the map, the error and speed plots, a "now" box and three rows of the
  results table (pre-run and live; `browser.html` too).
- Tests: `tests/test_reference1d.py` (import purity; S1 diverge within 3 m of the ESKF; S3 without
  grade > 1 km; S4 (e/σ)² > 5).
- Snapshot `reports/0.5.9/metrics.json` vs 0.5.8: no metric changed (ESKF untouched); tests 182 → 186.

## v0.5.8 (2026-10-24): low-end cruise threshold

- `FILTER_PROFILES["mpu9250"]["cruise_acc_th"]` 0.02 → 0.05 m/s² (owner's finding, 30-seed A/B
  `reports/0.5.7/lowend_cruise_th_30.json`): low-end portal S1 13.6 → 10.4 m, S2 16.0 → 13.4,
  S5 22.0 → 16.7, 5 km 42.9 → 24.9; (e/σ)² 0.16–0.62; no late callout. S4 1098 → 857 m, still
  broken (low-speed pitch). Note10 untouched. REPORT §4.6, SUMMARY, low-end tab updated.
- Snapshot `reports/0.5.8/metrics.json` vs 0.5.7: no metric changed (Note10 scenarios).

## v0.5.7 (2026-10-24): low-end gyro profile, crawl levelling A/B

- `FILTER_PROFILES["mpu9250"]["gyro_noise"]` 6.3e-3 → 0.0634 rad/√s (owner): the bias-instability
  GM terms with τ ≤ 50 s folded into white noise, √(ARW² + vib² + Σ 2τσᵢ²), worst axis. Low-end,
  30 seeds (`reports/0.5.7/lowend_fold50_30.json`): portal S1 37 → 14 m, S5 85 → 22 m, 5 km 614 →
  43 m, S2 16 m; (e/σ)² 0.19–1.0 instead of 2–146; no late callout. S4 unchanged (~1.1 km,
  (e/σ)² 340); oracle stops and levelling at stops do not help (`lowend_s4_level_10.json`).
  Sweep ×1–×15: `lowend_q_sweep_{a,b,c}_10.json`. Note10 untouched.
- Levelling restricted to crawling speed (`level_vmax`, off): S4 30.3 → 29.1–29.6 m, no effect
  elsewhere (`level_crawl_30.json`); stays off.
- REPORT §4.6–4.7, SUMMARY, the low-end tab of the simulator updated.
- Owner's review: the low-end S4 failure is low-speed pitch unobservability (NHC and barometer see
  pitch through v·δθ; filter speed falls 7 → 2.6 m/s while crawling at 8.3 m/s and stays ~6.6 m/s
  at the second stop), not yaw drift at stops; REPORT §4.6, §6 and SUMMARY corrected.
- `docs/sim/py/tunnelsim.zip` was git-ignored (`*.zip`), so `browser.html` could not start on
  GitHub Pages (owner); now committed with a `.gitignore` exception.
- Snapshot `reports/0.5.7/metrics.json` vs 0.5.6: no metric changed (the snapshot covers the Note10
  scenarios); tests 181 → 182 (live-slider overrides).

## v0.5.6 (2026-10-21): live custom tunnel, report completeness (review 12)

- Live custom tunnel: the LIVE bar builds a scenario from tunnel length, grade inside the tunnel,
  speed and one stop (`scenario.get_scenario("custom:L=..,g=..,v=..,stop=..,at=..")`), shown as a
  "Custom" tab; invalid settings return the reason (e.g. no room for a stop before the diverge).
- REPORT: §2.1 "What is simulated" (truth, sensors, drives, scenario table, metrics) and §2.2
  assumptions table with tags and the sensors left out (magnetometer, ambient light, camera, BLE,
  OBD); §4.7 adds the σ_s / curve-matching effect of levelling; one car noted as a limitation.
- Recording `docs/media/sim_S5_seed0.mp4` and its still re-made with the current page (driver's
  view, k = 1, spoken distance); `tools/record_frames.js` takes a tab key.
- Experimental in-browser live mode (owner): `docs/sim/browser.html` answers /api/ping and /api/run
  from a Web Worker running the same package under Pyodide 0.26.4 (`tools/browser/`, zip built by
  `build_site.py`); `index.html` unchanged. Packaging tested by running the zipped package alone;
  the page wiring tested with a mock worker; Pyodide itself not testable here (CDN blocked).
- Live speed-change sliders (owner): ±30 km/h from the cruise speed at a chosen fraction of the
  tunnel, held 20 s (`simulate_run(speed_change=)`). Without the server, the LIVE bar now shows
  disabled with the command to start it, instead of being hidden.
- REPORT §6 rewritten (owner): how AI was used, a table of where the author decided, corrected
  or overruled, the choices the agent made, and the AI-side errors with what caught them;
  SUMMARY's AI paragraph to match.
- Model sliders in the LIVE bar (owner, review 12 list): speed ×, exit multipath, pressure
  gradient, vibration, accel bias instability ×, map lateral offset and length error, driver
  model σ_d/τ_d, NHC turn gate. Sensor values via `set:`/`scale:` overrides tagged [ASSUMED,
  "live slider"] (test); `simulate_run(speed_scale=)`, `simulate.run(map_err=)`. The vibration and
  pressure-gradient selects became sliders. Playback 2× and 60× added.
- Results table tab (owner): every tab side by side, one drive each, with the current settings;
  pre-run drives without the server, live re-run of all tabs with the LIVE settings.
- Fix (owner, on GitHub Pages): playback at 1×/5×/10× did not move. The page data is on a 0.5 s
  grid and the play loop dropped the part of a frame's time step below one sample; it now keeps a
  continuous play clock (checked: 2 s of wall time = 2 / 10 / 20 / 60 s of drive).
- `SUMMARY.md`: a two-page version of the report (owner request); REPORT stays the full version.
- No filter change. Snapshot `reports/0.5.6/metrics.json` vs 0.5.5: no metric changed; tests
  180 → 181 (custom tunnel).

## v0.5.5 (2026-10-21): low-end IMU profile, levelling A/B (review 11)

- IMU class at start-up (`EskfConfig.imu_class = "auto"`, `detect_imu_class`): gyro white noise over
  the first 10 s, σ̂ = std(Δω)/√2, threshold 0.027 rad/s per sample at 100 Hz (geometric mean of
  Note10 ≤ 0.013 with noisy vibration and MPU9250 ≥ 0.059). Every low-end run (200) and every
  Note10 run checked is classed correctly; Note10 results are bit-identical (S1, S3, S4).
- `FILTER_PROFILES["mpu9250"]`: the Note10 tuning rules applied to the MPU9250 Allan terms
  (gyro noise 6.3e-3, gyro bias RW × 30.7, accel bias RW × 1.08, p0_bg 0.4°, p0_ba 0.45). The
  gyro bias RW × 0.5 / × 2 sweep barely changes accuracy (`reports/0.5.5/lowend_gyro_rw_sweep_10.json`).
- Low-end, 30 seeds (`reports/0.5.5/lowend_profile_30.json`): portal 1.6 km → 37 m (S1),
  340 → 16 m (S2), 1.4 km → 85 m (S5); S4 1.1 km and 5 km 614 m still broken; (e/σ)² 2–700. Report
  §4.6 rewritten: works in short/straight tunnels, breaks in stop-and-go and long tunnels, and
  overconfident throughout. The low-end tab in the simulator changes accordingly.
- Levelling pseudo-measurement (`EskfConfig.level`, off): f̄ = −Cᵀg + b_a at 1 Hz in cruise mode
  and at stops, gated on the 1 s mean |ω_y,z|·v. 10 seeds: S4 diverge 29 → 14 m at σ 0.02, but
  portal and (e/σ)² worse on S2/S3/S5/S6/5 km (grades and speed wander); stops only: no effect
  (30 seeds). Stays off; report §4.7.
- Snapshot `reports/0.5.5/metrics.json` vs 0.5.4: no metric changed; tests 178 → 180 (levelling
  Jacobian, IMU class detection).

## v0.5.4 (2026-10-21): callout k = 1, single-page simulator

- Callout margin k 1.65 → 1 (owner, review): σ_s is ~2× conservative, so k = 1 is in effect ~2σ;
  > 1 s late 1/500 (S5), late at all 0–16 %, mean 0.2–2.6 s early (k = 1.65: 0 late, up to 4.4 s
  early on S4). The filter is unchanged, so the 0.5.3 evaluation holds; the k = 1 numbers are its
  `callout_k["1.0"]` entries (`reports/0.5.3/evaluation_50.json`). Re-select k on real data.
- Report: filter-only callout timing (k = 0) and the announced callout (k = 1) as separate rows;
  "right moment" = never late, as early as the uncertainty requires.
- `docs/sim/index.html` (`tools/build_site.py`): one self-contained page, a tab per scenario,
  map on/off, k and callout-distance sliders in the browser; replaces the per-scenario pages.
- Same page, live mode: served by `tools/serve.py` (`/`, `/api/ping`, `/api/run`), it shows a LIVE
  bar (seed, sensors, mount error, filter switches) and re-runs the chosen tab in Python (~7 s);
  the old control panel moved to `/controls`.
- Driver's view: a navigation-app panel (heading-up, maneuver banner with distance, the spoken
  callout, speed, "No GPS · estimated ±2σ", optional true-car ghost); double-click for full screen.
  Display only; no filter or metric change.
- Every panel goes full screen on double-click, with a playback bar (play/pause, ±1 s, ±10 s,
  scrub, speed); keyboard: Space, ← →, Shift+← →. Display only.
- Review 11 (simulator part) fixes, display only: the spoken callout says the estimated distance to the
  exit rounded to 50 m ("In 350 m"), matching the banner, with the true distance next to it; the
  callout timing label says early or late by its sign; the live status no longer sticks at
  "running…" when the settings return to the pre-run drive; `roundRect` fallback for older browsers.
- Snapshot `reports/0.5.4/metrics.json` vs 0.5.3: only callout metrics changed, all from k 1.65 → 1
  (the filter is the same). Callouts come later and closer to the right moment: mean early bias
  S5 −1.8 → −0.7 s, S6 −1.4 → −0.9 s, timing rmse S4 4.3 → 2.6 s. Late fraction
  (snapshot seeds) rises to 5–15 %: S2, S3, S4 0 → 5 %, S5 0 → 15 %, 1 km 5 → 15 %. Within 2σ_t
  went to 100 % everywhere. The cost was accepted with k = 1; the 50-seed rates are in the report.

## v0.5.3 (2026-10-21): callout errs early, map scale term, report shortened

- Callout announced when ŝ + 1.65·σ_s reaches s_exit − 300 m (owner): > 1 s late 30 % (S4) and
  40 % (S5) → 0 % in every scenario; mean 0.3–4.4 s early. `callout_k` sweep (k = 0/1/1.65/2) in
  the evaluation. The `within_2sigma_t` coverage of the callout is no longer meaningful (the rule
  biases the callout by design).
- `map_scale_lateral` on by default (review 10): per-seed random map errors at the prior gave
  (e/σ)² 1.7–1.9 without it, 0.6–0.7 with it (`reports/0.5.2/map_random_stress_30.json`); cost
  on an exact map: portal 2.0–10.0 → 2.5–13.7 m, cross-track up to 3 m.
- Branch metric from the in-loop decision (offline `branch.py` kept as `branch_offline`):
  500/500 branch, 500/500 main in the negative control.
- `REPORT.md` rewritten to ~2300 words; random map stress table; σ of the map are [ASSUMED] and
  to be measured.
- 50 seeds, `reports/0.5.3/evaluation_50.json`.

## v0.5.2 (2026-10-21): map in the tunnel, interactive simulator

- Map layer (review 9, owner's design): 1 Hz heading and cross-track pseudo-measurements against the
  route during the outage, map lateral offset and scale error as states (reset per tunnel),
  explicit curve-matching Jacobian, offset noise in curves, pause around the diverge until the
  in-loop branch decision (`tunnelsim/mapview.py`, `EskfConfig.map_in_tunnel`, default on).
  50 seeds (`reports/0.5.2/evaluation_50.json`) vs 0.5.1: portal along-track 17–19 → 3–7 m
  (S4 44 → 5, 5 km 34 → 10 m); cross-track (new metric) 26–106 → 1.5 m; S2 diverge 13 → 3 m and
  callout 0.42 → 0.13 s; straight-tunnel diverge unchanged (S1 12.1 m); recovery failures 12 → 0
  (S4); negative controls 500/500 "main". Map-error stress: 93–97 % within 2σ.
- Cross-track and heading error metrics in the evaluation; evaluation table includes S1-1km/5km.
- Snapshot diff vs 0.5.1: only the `eskf` section moves (map layer; new cross-track/heading entries
  and S1-1km/5km); truth, sensors, INS and budget unchanged.

- `tools/serve.py`: local web page with controls (scenario, seed, IMU profile, vibration, tunnel
  pressure gradient, filter mount error, GNSS degradations, filter switches); each change re-runs
  one seed (~7 s), results cached; standard library only. Test +1.
- Map draws the true car (ground truth) and the estimated car (and the CV reference) as cars to
  scale, rotated to their headings, with labels (case text: "one or more cars … show the true
  position next to your estimate"). Media refreshed.
- No estimator or metric change.

## v0.5.1 (2026-10-20): review 8 (Fable) follow-up

- Filter changes, each checked against the 0.5.0 evaluation (50 seeds, `reports/0.5.1/evaluation_50.json`):
  - lateral NHC row skipped in turns instead of R = 10⁶: every reported metric identical to 0.5.0
    at the reported precision (expected < 1 m);
  - GNSS gate-lock recovery (P inflation after 5 open-sky rejections) and adaptive accel/gyro
    noise from the NIS: on by default, never fired on the nominal scenarios (0 inflations, 0
    rejections), so no metric change; low-end IMU 3.5 → 1.3 km at the portal, (e/σ)² ≈ 2000;
  - pressure-gradient state: implemented, **off by default** (on a straight constant-grade tunnel
    the gradient is structurally indistinguishable from a constant pitch error; it fixes
    10–30 Pa/km but costs S1/S3/S5; trace `reports/0.5.1/grad_trace_S5.png`).
- Error vs distance into the tunnel (S1-5km), `reports/0.5.1/error_vs_distance.png`.
- Snapshot diff vs 0.5.0 (`tools/snapshot.py --diff 0.5.0 0.5.1`): only new entries for the
  S1-0.5km/1km/5km scenarios and the test count (158 → 176); every existing metric unchanged.

- Simulator: phone sensor panel (accelerometer, gyroscope, barometer, GNSS speed), error-plot
  y-axis from the ESKF (CV clipped), CLI `--eskf KEY=VALUE`, `--mount-err R,P,Y`,
  `--no-sideslip`; new recording and screenshot in `docs/media/`.
- Scenarios S1-0.5km, S1-1km, S1-5km (tunnel-length sweep); `EskfConfig.zupt_causal` option
  (default off, offline detector unchanged). No change to existing scenario or filter numbers.
- Checks: `reports/0.5.0/fable_checks_30.json` (length sweep, pressure gradient, low-end IMU gate
  lock, causal ZUPT).
- `REPORT.md` shortened (~3500 → ~2300 words) and extended with the length sweep, pressure
  gradient, gate lock, offline ZUPT and "no map in the filter" points; AI-usage section names the
  other AI reviewers.
- `requirements.txt` (Python ≥ 3.10); `tools/record_frames.js` takes the browser from PW_CHROMIUM.
- From this version the commit author is the owner; Claude stays as co-author.

## v0.5.0 (2026-10-20): ESKF, visual simulator, exit branch, evaluation, write-up

- `tunnelsim/eskf.py`: batched 16-state ESKF (GNSS position/velocity with NIS gate, NHC,
  barometer with offset state, ZUPT from an IMU stop detector). Design and results:
  `docs/decisions/2026-10-20-v0.5-eskf.md`; `reports/0.5.0.dev0/eskf_50.json`.
- `tunnelsim/observability.py`: structural rank check of the filter's error model.
- `tunnelsim/experiments.py`: `simulate_run`, `eskf_experiment` (along-track error, consistency,
  exit callout timing, exit recovery).
- `tools/simulate.py` + `tools/simulate_template.html`: visual simulator, one scenario/seed/
  overrides to a self-contained HTML player.
- Tests +6.
- A, exit re-acquisition phase (velocity-only start, decaying R inflation, gain-capped updates
  of ≤ 2 m incl. velocity updates, convergence-based end, reported-σ limit): exit step p95
  45–76 m → 4–6 m; slower settling after S4 (16/50 not within 10 m in 60 s). Recovery metric
  threshold 5 → 10 m (≈ 2× GNSS horizontal rms). Tests +2.
- B, cruise (speed-hold) pseudo-measurement mode, behind `EskfConfig.cruise` (default False,
  not accepted): exit rms improves on S1/S4/S5 but the filter becomes overconfident (mean
  (e/σ)² 6–33). No metric change with the default. `reports/0.5.0.dev0/cruise_ab_50.json`.
- Scenario S6 (S1 with ±2 km/h, 40 s speed ramps inside the tunnel): breaking case for B.
- B accepted in the "increment" form (speed clone state, 1 s increment pseudo-measurement) and
  on by default: exit rms S1–S5 20/18/32/54/36 → 13/14/10/39/28 m, consistent; S6 breaks
  (58 m, (e/σ)² 4.7). `(a)` absolute hold kept as `cruise_mode="hold"` for comparison.
- Fix: barometer 1 s average modelled at its window centre (was the window end, ~0.5 s lag →
  pitch error at grade changes; S3 callout). Changes every ESKF number;
  `reports/0.5.0.dev0/eskf_baro_fix_50.json`.
- Review 6: GM speed-wander cruise mode (`cruise_mode="gm"`); S6s/S6L wander variants;
  callout late rate and claimed timing uncertainty. `reports/0.5.0.dev0/cruise_gm_50.json`.
- Exit branch on the route in every scenario (review 6): 20° right-hand branch at the diverge
  (clothoid 40 m + arc 70 m + clothoid 40 m), total length kept; S2 diverge moved 1800 → 2000 m
  (onto the straight). `build_scenarios(branch=False)` keeps the v0.4 geometry, used by the error
  budget and the observability check. Changes every ESKF number after the diverge; INS
  error-free attitude floor tolerance 1e-6 → 2e-6 rad.
- S6 third ramp moved 2000 → 2100 m (after the previous ramp ends).
- Review 7: lateral NHC dropped while |ω_z|·v > 0.3 m/s² (default; sideslip in the exit curve
  gave a −20 m common-mode along-track transient); threshold and no-sideslip checks; two-sided
  branch decision with a position-certainty condition for "main" (negative control: car straight
  on); filter mount-error option; assumption-removal runs (C).
- `tools/evaluate.py` (final Monte Carlo, `reports/0.5.0/evaluation_50.json`), ESKF section in the
  snapshot, `tools/plot_nhc_trace.py`, `tools/baro_model_check.py`.
- `REPORT.md` (write-up incl. AI usage), README rewritten.
- Metric diff vs 0.4.0 (`python tools/snapshot.py --diff 0.4.0 0.5.0`), all explained by the exit
  branch now being on the route (curves where S1/S3–S5 were straight):
  - `budget.*`: unchanged (uses `branch=False`).
  - `truth.*`: S1 sideslip 0 → 0.63°, lateral NHC velocity 0 → 0.14 m/s (the exit curve);
    error-free strapdown floor up to 0.36 mm, S3 centreline discretisation 6e-6 → 1.6e-4 m
    (clothoids); new S6/S6s/S6L entries.
  - `ins.error_free.*`: attitude floor up to 1.2e-6 rad (test tolerance 1e-6 → 2e-6 rad).
  - `sensors.imu.interpolation.*` pointwise errors and `timestamps.rate_offset_*` grow with the
    yaw rate in the curve; integral errors stay at 3–4e-6 m/s.
  - `sensors.gnss.exit.nominal.mean_along_track_m` −4.0 → −0.8 m: the portal is now on the
    branch heading.
  - New `eskf` section (20 seeds); 158 tests pass with `--runslow`.

## v0.4.0 (2026-10-20): INS, error budget, calibration and reference algorithms

Scope moved by the focus decision: the temperature stress scenario goes to the
assumption-removal stage; the observability analysis is reduced to a structural rank check
done with the ESKF (v0.5). Metric diff vs 0.3.0: truth metrics moved only through the scenario
changes listed below (longer approach and lead-out, approach maneuvers); `ins`, `budget`
sections are new.

- Plan and decisions: `docs/decisions/2026-10-19-v0.4-plan.md` (review 3, V1 to V6);
  coordinate and measurement contracts `docs/contracts.md`. DESIGN 4 corrected: forward axis
  is phone −z (constant BI-sized accel bias ~26 m, was ~10 m), pitch axis phone −x (~0.84 km,
  was ~1.4 km). No code change yet.
- Contracts revised after review 4: attitude error `Ĉ = Exp([δθ]×)C` (estimate − truth),
  FLU pitch-direction trap documented, C_nb chain, independent mount/lever-arm oracle flags,
  calibration epoch, timing concepts, signed/abs metrics, 8-state contract.
  `frames.so3_log`, `frames.attitude_error_n` + `tests/test_frames.py` (16 sign tests).
- Step 2, strapdown INS (`tunnelsim/ins.py`): rotation-matrix mechanisation of the plan
  (midpoint rate, trapezoid accel and position), dt from reported timestamps, batch dimension
  for Monte Carlo, calibration applied inside the INS (`ImuCalibration`), attitude sequence as
  an optional input (ideal-attitude experiments), `reference_point` (p_R = p_P − C_np C_bpᵀ r_b).
  Imports only numpy, frames, meas (tested).
- New metrics `ins`: error-free reproduction of the truth (≤ 1.3 mm on S1–S5 and MIX, attitude
  ≤ 5e-7 rad; same as the truth strapdown); convergence on closed-form motion exactly second
  order (2.00 per halving, one- and two-axis rotation; two-axis coning 0.7 mm at 100 Hz over
  60 s); MIX plateaus at 6e-5 m from 200 Hz, the truth's own discretisation floor
  (np.gradient for ω̇, sideslip FOH, event timing on the grid), not a mechanisation error;
  irregular dt (jitter + dropped samples) adds nothing measurable.
- Harness fix found on the way: with sampling jitter the first IMU sample is not at the truth
  grid time; the oracle initial state is now taken at the true time of the first sample
  (was 6 mm offset on S1 = v·jitter).
- Tests 101 -> 114.
- Scenarios (truth change, done now so the truth changes once): approach = 200 s of driving at
  entry speed before every portal (GNSS calibration windows up to 180 s); lead-out ≥ 800 m and
  ≥ 220 s after the portal at entry speed (fixed 180 s INS budget; S1 and S2 had only 171 s and
  148 s). Tunnel geometry and speed events unchanged relative to the portal. Truth metrics that
  moved vs 0.3.0, all from the longer runs: durations (S1 238 → 420 s, S2 216 → 420, S3 298 →
  421, S4 475 → 585, S5 242 → 463), strapdown residuals accumulate over the longer run (S2
  3.1e-5 → 5.3e-5 m, S3 4.8e-6 → 1.6e-5, S5 4.95e-4 → 8.8e-4, S1 5e-11 → 7e-9), centreline
  discretisation S2 5.1e-5 → 1.0e-4 m (longer road), time in tunnel −0.01 s (sample alignment).
- Step 4 (short): `tunnelsim/reference.py` (estimator side: LS accel bias from GNSS velocity,
  conditional GM shape incl. the constant part, GNSS entry state, INS from the entry with
  hold / condgm / none); `ins.strapdown` gains a time-varying bias shape and a zero-velocity
  mask; `tunnelsim/experiments.py` (harness). Results in DESIGN 4.4 and
  `reports/0.4.0.dev0/step4_100.json`. Tests +5.
- Focus decision (`docs/decisions/2026-10-20-focus.md`): estimable baseline first, then remove
  assumptions; observability reduced to a rank check; ESKF next.
- Scenarios (truth change): approach maneuvers in every scenario (+15 / −15 km/h / back to v0
  at 20, 43, 66 s; steady from 76 s, portal at ~200 s), so the pre-tunnel filter must estimate
  the bias from excitation. Truth metrics moved vs 0.3.0 (beyond the approach-length change):
  strapdown residuals now include the maneuvers (S1 1.6e-4, S2 1.8e-4, S3 1.6e-4, S4 1.5e-3,
  S5 1.0e-3 m), vertical NHC violation from suspension pitch in the approach (S1/S2 0 → 0.117,
  S3 0.015 → 0.103 m/s). The 200-seed budget (`budget_200.json`) was computed before the
  maneuvers; it starts at the portal at the same speed, so it is not affected beyond seed noise.
- Override `imu_fwd_bi_xy`: forward-axis (phone z) BI at the x/y level (review 5).
- Step 3, error budget (`tunnelsim/budget.py`, analysis harness): fixed 180 s INS-only budget
  and real-exit error for S1–S5, 22 configurations, 200 seeds (paired), bias / rmse / p95_abs
  with bootstrap CI, growth curves, variance shares and additivity; per-seed IMU generated once
  and recomposed (tested identical to the generator). Analytic references incl. three bias
  instability kernels (raw, held entry bias, GM prediction). Results in DESIGN 4.3 and
  `reports/0.4.0.dev0/budget_200.json`; `tools/budget.py` for the 1000-seed final run. New
  metric section `budget` in the snapshot. Tests +6 (one slow).
- Irregular-dt test moved to closed-form motion (exact signals at jittered instants): 600 s,
  two-axis, +3 % vs the uniform grid (8.6 → 8.9 cm, coning). On scenarios the generator's
  jitter needs the truth interpolated off-grid; that resampling floor grows with run length
  (S4, 585 s: 1.2 cm) and is reported, not charged to the mechanisation.

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
