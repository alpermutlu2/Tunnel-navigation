# Review 8 (2026-10-20): repository review at df3fdfa (Claude Fable 5.1, passed on by the owner)

Summary in English (original in Turkish). The owner obtained this review from Claude Fable 5.1;
several earlier reviews also came from other AI assistants/models and were filtered by the owner.

Verdict: deliverables met; remaining gaps in two classes.

Requested by the case text, still missing:
1. Sensor traces in the simulator (accelerometer, gyroscope, barometer, GNSS speed).
2. Tunnel lengths 0.5–5 km (all scenarios were 2.5–3.5 km).
3. Live parameter changes from the CLI (`--eskf`, `--mount-err`, tunnel length, no sideslip).

Methodological points a reviewer would catch:
4. The ZUPT detector is non-causal (forward-backward filters, zero delay).
5. No map in the filter inside the tunnel; the along-track error is the projection on the true
   road tangent (implicit perfect map matching). Say so.
6. Tunnel pressure gradient missing from the report: own run S3 + 30 Pa/km: exit 125 m,
   (e/σ)² 10, callout 5 s, branch confirmation 0 %.
7. Low-end IMU breaking mechanism: the χ² gate locks out ~57 % of GNSS fixes and the filter
   diverges before the tunnel; describe and list recovery (open the gate after N rejections,
   inflate P, per-profile Q/P0) as next step.
8. Commit authorship (38/40 commits authored by Claude) and the missing v0.5.0 tag.

Packaging: requirements/pyproject and Python ≥ 3.10; error plot y-axis crushed by the CV curve
(clip); machine-specific path in `record_frames.js`; footnote that `assumptions_30.json` predates
the branch-check fix; optional second car; NHC row skipping instead of R = 1e6.

Owner's instruction: take it into account; shorten the report.

Response (this commit range): items 1–7 and the packaging points done (`REPORT.md` §3–§5,
`reports/0.5.0/fable_checks_30.json`); 8 and the second car left to the owner / not done.
