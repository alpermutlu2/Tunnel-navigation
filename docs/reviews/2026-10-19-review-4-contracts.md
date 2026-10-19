# Review 4 (2026-10-19): coordinate and measurement contracts

Review of `docs/contracts.md` at `e8b8e62`, before any INS code. Findings and how they were
resolved are recorded in `docs/decisions/2026-10-19-v0.4-plan.md` section 8 and in the
revised contracts.

| Priority | Finding | Resolution |
|---|---|---|
| P0 | Attitude error `Ĉ = (I − [φ]×)C` contradicts estimate − truth | `Ĉ = Exp([δθ]×)C`, Log map, ± roll/pitch/yaw tests. Checked: `rot_z(ε) ≈ I + [ε e_z]×` |
| P1 | "Perfect calibration" epoch undefined (INS start vs tunnel entry); future bias must not be used; where corrections are applied | `calibration_epoch`; estimator applies corrections via its interface |
| P1 | Timing concepts mixed (sampling jitter vs timestamp error vs clock scale vs GNSS delay) | Separate table; δ sign and common clock stated; common true epochs |
| P1 | Error-free consistency "with vibration on" contradicts vibration being sensor-side | Two separate tests (the item had come from review 3's list) |
| P2 | C_nb chain, independent mount/lever arm, signed vs abs P95, 180 s INS-only label, 8-state contract | All added |

Our addition while checking P0: in FLU a positive rotation about +y is nose down, so all
nose-up angles enter as Ry(−angle); the scalar pitch error is minus the y rotation component.
