# v3 Session Index — 2026-05-24

**Breakthrough:** First complete v3 simulated lap on Sprint A. After 7 consecutive controller-iteration phases that all returned 0/10 Monte-Carlo completions, a physics-first audit delivered the fix. Reactive controller now completes 7/10 MC seeds at **2:09.12** (default flags + `--inertia-zz 2400`). With 9/10 stability target: **2:12.64** at `--chicane-safety-mult 0.75`.

---

## Session result in numbers

| Metric | Before session | After session |
|---|---|---|
| MC completions (Sprint A, 10 seeds) | 0/10 (all phases) | 7/10 (2:09.12 median) |
| Best lap w/ 9/10 stability target | n/a | 2:12.64 (`--chicane-safety-mult 0.75`) |
| DP-integrated theoretical best | 2:03 (planner only) | 2:03 (unchanged; controller has ~6 s slop) |
| Open-loop v@t=8s gap vs Tomas | −9.2 km/h | −3.6 km/h |
| Remaining 3/10 aborts | all at chicane | chicane envelope (separate failure mode) |

---

## What shipped (docs created or updated this session)

| Doc | Status | One-line description |
|---|---|---|
| `docs/architecture-v3-longitudinal-physics-fix.md` | New | Six missing longitudinal-force terms added to the v3 ODE + DP planner mirror. Covers slope gravity, rolling resistance, engine brake, m_eff with engine inertia, drivetrain η=0.87, brake-torque uplift (×1.80 for BMW 1M). |
| `docs/architecture-v3-turbo-curve.md` | New | Twin-turbo per-RPM steady-state boost curve (TURBO_0 + TURBO_1, combined cap 0.92 vs prior flat 0.46). |
| `docs/architecture-v3-lateral-yaw-inertia-fix.md` | New | I_zz corrected from plate formula (~1258 kg·m²) to AC box formula (default 3051 kg·m²); `--inertia-zz 2400` is the recommended override into the manufacturer band. |
| `docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md` | New | Stanley LPF τ=60 ms (N=3 ticks) damps steering chatter; moves abort from s≈660 m to s=1086 m. |
| `docs/architecture-stanley-crosstrack-gain.md` | New | k_cross lifted from 0.5 to 0.75 (×1.5); resolves sustained-corner cross-track accumulation at s=1086 m. Delivers 7/10 completions. |
| `docs/architecture-v3-mpcc-implementation.md` | New (2026-05-24, late) | v3.3 MPCC implementation: `--controller mpcc` ships alongside `--controller mpc`. Curvilinear (s, n, ψ_e) OCP that rewards progress along a reference path. Steers, integrates, runs in budget at horizon 30 m; does **not** yet meet the spec §23.3.9 lap-time / completion gates on Sprint A (0/10 in first build-time smokes, same chicane-region abort as v3.2 MPC). Architecture, build-time decisions, and next-experiment notes captured in the doc. |
| `docs/architecture-v3-hmpc-casadi-outer.md` | New (Phase 5.2 pivot) | Replaces the Phase 5.1 HMPC outer's OSQP+SQP solver with CasADi+IPOPT nonlinear MPC. Outer primal-infeasibility drops from 80 % → 0 %; brake-commit s moves from 391 m → 299 m (vs reactive 257 m). Closed-loop completion still 0/10 — bottleneck moves to the inner tracker. |
| `docs/architecture-bmw1m-powertrain-calibration.md` | Existing (earlier session) | `--boost-steady` and `--cd-override` CLI overrides. Historical record; superseded on the default path by the turbo-curve doc. |
| `docs/architecture-v3-shipping-state.md` | Existing (2026-05-22) | Decision record: MPC parked, reactive + chicane safety cap is the shipped v3 path. Needs update (see below). |
| `docs/architecture-v3-pacejka-nonlinear-load-sensitivity.md` | New (follow-up, 2026-05-25) | AC `LS_EXPY` / `LS_EXPX` non-linear `D(Fz)` + `FALLOFF_LEVEL` floor wired into the production Pacejka, planner, and MPC ellipse cap. **Headline finding:** null effect on reactive's Sprint-A lap (≤ 20 ms, same stability counts, same chicane abort) — the inner-wheel grip gain and outer-wheel loss net to a 1.9 % axle Fy reduction under 50 % lateral transfer, opposite of the user's original intuition. Patch is correct and active; chicane abort is not a peak-grip problem. |

**Note on the ellipse-aware DP doc:** The brief mentioned a possible `docs/architecture-v3-ellipse-aware-dp.md`. No such file exists in the repo. The ellipse-aware (Fx, Fy)(s) controller is the user-spec'd *next* architectural direction, not something that shipped this session.

---

## Production defaults as of this session

For reactive v3 runs, the recommended invocation is:

```
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    --model slip --controller reactive --single-lap \
    --inertia-zz 2400
```

For 9/10 MC stability (more conservative, slower lap):

```
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    --model slip --controller reactive --single-lap \
    --inertia-zz 2400 --chicane-safety-mult 0.75
```

| CLI flag | Recommended value | Why |
|---|---|---|
| `--inertia-zz` | `2400` | Mid-manufacturer-band override; default (3051, box formula) over-stiffens yaw response. |
| `--chicane-safety-mult` | `0.80` (default, 7/10) or `0.75` (9/10 stable) | Multiplier on `v_corner` for tight-radius segments. Lower = more conservative entry speed = more stable. |
| `--controller` | `reactive` | Production v3 path. MPC (`--controller mpc`) is experimental / parked. |
| `--plan-source` | `v3_dp` (default) | DP planner rebuilt against same 6 longitudinal-force terms as the plant. Do not use `v2`. |

Do NOT add `--boost-steady` or `--cd-override` to normal reactive runs. Those overrides were calibration diagnostics; the twin-turbo curve and η=0.87 already account for the engine physics on the default path.

---

## Reading order for a new reader

Start here, then follow this chain:

1. **`docs/architecture-v3-shipping-state.md`** — Why v3 exists alongside v2, what the reactive path delivers, and why MPC is parked. Read the "Modes" table and "Known limitations" section first.

2. **`docs/architecture-v3-session-2026-05-24.md`** (this file) — Summary of what changed in the 2026-05-24 session to turn 0/10 into 7/10.

3. **`docs/architecture-v3-longitudinal-physics-fix.md`** — The heaviest technical doc. Covers the six-term audit and the DP planner mirror. Read the "Architecture: what the fix changed" section and the "Follow-up: DP planner update" at the bottom.

4. **`docs/architecture-v3-turbo-curve.md`** — Short. Covers the twin-turbo model that replaced the flat WASTEGATE multiplier.

5. **`docs/architecture-v3-lateral-yaw-inertia-fix.md`** — Why the plate formula for I_zz was wrong and how the fix is sourced from AC's `[BASIC].INERTIA` block.

6. **`docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md`** — The Stanley LPF that cleared the chatter at s=1086 m.

7. **`docs/architecture-stanley-crosstrack-gain.md`** — The k_cross gain that delivered the final 0/10 → 7/10 step. The shortest of the new docs; read last.

For MPC decision history (phases 4.1 → 5.0.8), each phase doc is in `docs/architecture-slip-model-phase*.md`. They are reference material for any future MPC resurrection work, not required reading for the reactive path. Phase 5.0.7 (`docs/architecture-slip-model-phase5_0_7-v32-qp-weight-retune.md`) closes the QP-weight-tuning question with a negative result and escalates to 5.0.8. Phase 5.0.8 (`docs/architecture-slip-model-phase5_0_8-v32-first-class-longitudinal.md`) builds the first-class throttle/brake MPC emit the 5.0.7 diagnosis called for; the change is structurally sound (and the `--mpc-emit-source {qp,sub}` A/B knob is reusable) but the 30 m horizon cannot brake-anticipate Sprint A's pre-chicane straight, so MPC still aborts (0/10 MC at all chicane mults). Recommended next step: Phase 5.0.9 — terminal v-tracking cost driven by the DP envelope.

---

## Known remaining gaps (as of session end)

| Gap | Impact | Likely fix |
|---|---|---|
| 3/10 chicane envelope aborts at s≈665 m | 7/10 → 10/10 | Ellipse-aware FF controller; plan (Fx, Fy)(s) trajectories instead of v(s). This is the user-spec'd next direction. |
| Lap time vs Tomas: +21.5 s (2:09.12 vs 1:47.56) | Research-quality gap | Controller tracks v_max(s) but Tomas rides the Pacejka (Fx, Fy) edge. Same root cause as above. |
| Chicane safety cap conservatism (316 segments flagged at mult=0.80) | ~few seconds of plan slack | Tighter segment detection or per-corner cap values. |
| M_z (self-aligning torque) | Small | Not yet wired into the v3 plant. |
| Tyre relaxation length | Small | In `tyres.ini` but not read. |
| Camber, toe, bump-steer | Small | All in ini files, all unread. |

---

## What the session did NOT ship

- The ellipse-aware (Fx, Fy)(s) DP controller is **not** in this session. It is spec'd as the next architectural direction.
- `docs/architecture-v3-shipping-state.md` was not updated to reflect the 7/10 completion result. That doc records the 2026-05-22 shipping decision (0/10, MPC parked). It should be updated to note that the 2026-05-24 physics pass changed the reactive result to 7/10. Flagged as an open point.
- The `--brake-torque-mult 1.80` flag is NOT in the recommended default invocation. The brake budget fix is baked into `--brake-torque-mult` which defaults to `None` (no change). Users who want the empirically-corrected brake envelope must pass it explicitly. The session's 7/10 result was achieved without this flag on the default physics head — the brake budget interaction with the new plant was not re-validated in this session.

---

## Open points logged

1. **`docs/architecture-v3-shipping-state.md` needs a 2026-05-24 addendum** noting that the reactive path now completes 7/10 on Sprint A with `--inertia-zz 2400`. The doc currently describes the 2026-05-22 state (0/10, chicane abort).
2. **`--brake-torque-mult 1.80` interaction with the new physics** was not re-validated at 7/10. The longitudinal physics doc records that the post-fix closed-loop with brake mult changed the abort character but was not swept alongside the I_zz + Stanley fixes.
3. **Layout portability of `k_cross=0.75`** not validated beyond Sprint A. See `docs/architecture-stanley-crosstrack-gain.md` open points §1.
