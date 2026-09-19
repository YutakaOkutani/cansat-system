# 07 Mission Specification

> **Audience: AI coding and debugging agents working on mission behavior.** This page owns Phase intent and transition semantics. Read source for current thresholds.

## Mission contract

- Start at Phase 0 through `run_full_mission()` and terminate through Phase 7 or safe shutdown.
- Use the normal forward sequence `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7`.
- Allow bounded recovery transitions `4 -> 3`, `5 -> 3`, and `5 -> 4`.
- Preserve the GPS-arrival latch across Phase 4/5 recovery so noisy distance does not undo a confirmed arrival.
- Enforce both local handler timers and cumulative controller budgets; Phase re-entry must not reset cumulative mission exposure.
- Keep verified goal, forced goal, total timeout, subset completion, interruption, and abnormal exit distinguishable.
- Stop motors in Phase 0 and Phase 7 and on every shutdown path.

Current numeric thresholds, pins, speeds, and budgets are authoritative in `mission/const.py`.

## Configuration contract

- Load only repository-root `mission.toml` in production.
- Require `[target]` and `[radio]`; reject missing, malformed, unknown, wrong-type, non-finite, or out-of-range values before hardware initialization.
- Load optional `run-context.toml` only as behavior-neutral provenance; a missing file resolves to `unclassified/mission`.
- Keep target coordinates and radio mode out of tracked constants.
- Ignore legacy target environment variables and reject legacy production CLI arguments.
- Keep the example target deliberately invalid until an operator edits the local copy.
- Treat current rule-derived timing, goal-marker, and field assumptions as NSE2026 provenance until another competition explicitly replaces them.

## Phase responsibilities

| Phase | Required behavior | Success transition | Recovery or forced transition | Critical evidence |
| --- | --- | --- | --- | --- |
| P0 release/descent | Keep motors stopped; accept only fresh BMP/BNO acceleration; latch altitude-drop or confirmed impact; hold before release | P1 after held release evidence | P1 on bounded timeout; restore radio either way | sensor stale ages, baseline/delta, exit reason/detail |
| P1 separation | Run the Manager-owned separation pattern for a bounded interval; collect only diagnostic heading-offset evidence | P2 after interval | Controller budget also forces P2 | elapsed time, motor command, candidate quality |
| P2 escape/calibration/alignment | Escape parachute, calibrate, learn GPS/BNO offset from a straight stable segment, confirm heading readiness | P3 when heading is ready; P4 if GPS arrival was already confirmed | Bounded reorientation/retry; best-effort/fallback offset; controller timeout | stage/mode, calibration, progress, reject reason, offset validity |
| P3 GPS navigation | Compute distance/azimuth from valid GPS; prefer GPS-aligned BNO for high-rate steering; confirm arrival over samples/time | P4 on confirmed arrival | P4 on local/cumulative timeout; continue conservatively when heading is unavailable | GPS quality/sequence, heading source/trust, arrival latch |
| P4 visual search/alignment | Search on a forward arc; steer immediately toward a credible candidate and confirm distinct frames | P5 only on confirmed detection | P3 on GPS disagreement only without a visible cone; P7 on timeout without a visible cone or exhausted camera recovery | probability, method/direction, confirmation count, camera health/reinit attempts |
| P5 visual approach | Approach using visual steering; confirm reached over multiple frames | P6 with `GOAL_REACHED` | P4 after bounded loss; P3 when far/camera dead without a visible cone; P7 on timeout without a visible cone | entry reason, loss/reach counts, timeout, command |
| P6 final ram | Apply short bounded forward command only when not globally timed out | P7 after duration | Immediate P7 on total-timeout state | mission end reason, duration, command |
| P7 terminal | Stop motors, resolve arrival semantics, signal goal/give-up, request shutdown | Process exit | None | arrival reason, mission end reason, final row |

## Phase 2 invariants

- Do not skip the escape stage even if GPS already reports arrival; transition to P4 only after escape completes.
- Treat the P1 offset candidate as diagnostic only because parachute drag can bias it.
- Learn alignment only from accepted GPS fixes, valid BNO heading, sufficient straight-line progress, stable heading, and consistent subsegments.
- After BNO recovery, discard the active alignment segment and collect a fresh reference.
- Exclude settling/reorientation time from the next straight-leg collection budget.
- Bound stalled progress, BNO wait, and retry count.
- Mark a fallback offset as unverified; do not present it as calibrated evidence.

## Phase 3 invariants

- Do not use raw magnetometer-only steering when GPS and aligned BNO are untrusted.
- Prefer aligned BNO after GPS-derived offset is valid; use GPS heading as fallback before alignment.
- Treat GPS course as sparse evidence for alignment, not automatically as the high-rate motor heading.
- Require arrival confirmation; one close fix must not transition.
- Keep arrival latch set after confirmation.
- When no trusted heading exists, keep behavior bounded and observable rather than inventing a heading.

## Phase 4/5 invariants

- Keep detector acquisition/normalization in the sensor/vision layer and mission interpretation in the Phase.
- Require short-term consistency for P4 detection; a weak single frame must not transition.
- A fresh visible cone overrides capture windows, local/cumulative P4/P5 timeouts, and GPS disagreement. Follow its current image position; resume search only after visual loss. The global mission deadline still stops the mission.
- At the image edge, use a moderated both-wheels-forward arc with image-error hysteresis. Issue at most one motion command per new camera frame. Estimate the frame period from observation timestamps and stop when the next frame is late, bounded by the configured hold ceiling; keep stale-frame and global-stop checks responsive.
- Keep the camera detector and capture pipeline inactive throughout P0-P3. Activate them only on entry to P4/P5, and release them whenever the mission returns to a non-vision phase; a camera disconnect before P4 must not affect navigation or motor control.
- Compensate P4 candidate direction with heading when available and reject discontinuous vertical position or scale before confirmation.
- In P5, confirm close-range evidence only while the cone is centered for a straight final ram; reset confirmation when it leaves that window. Continue image-directed steering for an off-center close cone, using the latest image direction rather than a lagging filtered direction.
- Distinguish cone loss (`P5 -> P4`) from a P4 timeout or exhausted camera recovery (`P4 -> P7`).
- In P4, allow at most three camera recreations at five-second intervals and require a valid captured frame before declaring recovery; retain at least a 15-second recovery window.
- Without a visible cone, P4/P5 timeouts stop motors and skip final ram. Time alone must never authorize P6.
- Do not let a camera failure create an unbounded search/approach loop.
- Keep the legacy camera relay outside the production architecture.

## Timeout and terminal semantics

- Derive the global timeout from cumulative Phase budgets, final-ram allowance, and transition margin.
- Preserve the minimum P3 navigation reserve when changing earlier budgets.
- Accumulate P3/P4/P5 time across re-entry.
- On global timeout, force safe terminal handling; do not perform the final ram.
- `GOAL_REACHED` is verified visual completion.
- `PHASE5_VISUAL_LOST_TIMEOUT` reports give-up without a visible cone. `PHASE5_TIMEOUT_FORCED_GOAL` remains a legacy-log reason; current P5 timeouts never authorize a ram.
- `MISSION_TOTAL_TIMEOUT` is give-up behavior.
- Any new terminal reason must update log tests, Phase 7 resolution, and analysis interpretation.

## Change matrix

| Change | Inspect together |
| --- | --- |
| Any Phase transition | handler, controller cumulative transition, `runs/spec/`, log reason |
| P0 detection | BNO/BMP freshness, radio restore, `p0_detect.py` |
| P2 alignment | motor patterns, GPS fix sequence, BNO recovery, `phase2_flow.py`, `phase3_heading.py` |
| P3 navigation | sensor GPS acceptance, heading selection, motor policy, navigation specs |
| P4/P5 vision | camera thread, `lib/cone_detect.py`, motor policy, saved field frames |
| P6/P7 | total-timeout branch, stop behavior, final reasons/log row |
| Mission budgets | every local timeout, re-entry path, minimum P3 reserve, full mission envelope |

## AI Checklist

- Does the Phase still have one explicit objective and bounded exit?
- Are success, fallback, forced progress, and give-up distinct?
- Are noisy observations confirmed and freshness-aware?
- Does Phase re-entry preserve cumulative budgets and required latches?
- Did I check adjacent Phases, Manager behavior, logs, and mapped specs?
