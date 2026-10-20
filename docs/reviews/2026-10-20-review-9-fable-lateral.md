# Review 9 (2026-10-20): second repository review at 4c4626b (Claude Fable 5.1, passed on by the owner)

Summary in English (original in Turkish).

Verdict: deliverable; 7 of 8 earlier points closed; the interactive simulator solves the live
parameter-change requirement. Open: v0.5.0/v0.5.1 tags not pushed.

New finding: **the filter does not use the map inside the tunnel, so the cross-track error is
larger than the along-track error and is not measured anywhere.** Heading is unobservable in a
straight tunnel (gyro z bias, 0.8–2.5° drift), the filter moves along its own wrong heading and the
lateral error grows as v·δψ·t; all metrics project on the road tangent. Reviewer's numbers (8 seeds):
S1 portal along/lateral 15.9 / 25.8 m, S1-5km 26 / 55 m, S4 46 / 88 m. Consequences:
1. requirement (a) "keep showing the car": the estimated car runs beside the tunnel;
2. the "+7 m unexplained" jump at the 5 km diverge is the lateral error projected on the 20° curve
   tangent (≈ 0.34·L);
3. slow recovery after the exit (S4, 5 km) is draining the lateral error, not ~45 m along-track;
4. the branch check's nearest-centreline projection is fragile with 50–90 m lateral error.

Prototype (reviewer's scratch copy): in the outage, at 1 Hz, heading pseudo-measurement
ψ̂ − ψ_map(ŝ) (σ 1°) and cross-track pseudo-measurement n·(p̂_R − c(ŝ)) (σ 1.5 m), route map
without errors: lateral → 0.2 m, portal along-track 3–5× smaller (S1 16 → 3 m, S4 46 → 6 m,
S2 diverge 14 → 5 m, callout 0.51 → 0.21 s), recovery problem gone, consistency kept. Caveats
from the reviewer: map without errors, two candidates needed at the diverge.

Recommendation: `EskfConfig.map_in_tunnel`, default on, re-run 50 seeds; or default off with a
30-seed A/B and a "what the map adds" section; in any case report lateral and heading errors.
Other: report wording (+7 m, recovery), `reports/0.5.1/` references, sensor panel 0.5 s averaging,
lateral-error tile on the page, push the tags.
