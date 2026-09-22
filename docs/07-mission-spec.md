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
| P5 visual approach | Approach using visual steering; confirm aligned camera/range pairs | P6 without declaring success | P4 after bounded loss; P3 when far/camera dead without a visible cone; P7 on timeout without a visible cone | entry reason, loss/reach counts, timeout, command |
| P6 range approach | Alternate bounded forward pulses and stopped observations; keep camera active | P7 with `GOAL_PROXIMITY_CONFIRMED` after distinct stopped camera/range confirmations | Stop on invalid evidence; P7 on observation loss or bounded timeout | range sequence/time, goal decision/count, motor deadline, end reason |
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

- For a cone already identified in three distinct, temporally consistent, non-full-width frames, a clipped close view may retain identity for at most six seconds. This does not raise the detector probability or establish a new identity from a red screen.
- Continuation requires a centered observation within six seconds (usable from 2–60 cm), fresh heading and current near range (2–15 cm), current compatible color/region evidence, and at most 10 degrees of heading excursion from the centering anchor. Check every IMU update, so small oscillations do not accumulate into target loss but turn-away/turn-back revokes identity. The frame and range must remain time-aligned. Ordinary centered views during acquisition are remembered but grant no continuation until three consistent ordinary views establish identity. Clipped observations cannot refresh an existing centering anchor.
- A brief invalid echo, weak ROI or color dropout suspends permission without erasing the six-second identity memory. Strong negative-region evidence or a large turn revokes it. A frame-filling, strongly red surface may use prior identity despite weak positive ROI support; it cannot establish identity from a red screen alone. No dropout counts as a goal vote or authorizes motion.
- A fresh frame-filling object with valid near range latches a stop across P4/P5/P6. P4/P5 transfer ambiguous close holds into bounded P6 observation instead of searching or waiting indefinitely. A visually qualified close target with unavailable sonar also transfers after a bounded wait. This observation-only entry does not authorize motion or success.
- Qualified camera/range pairs still require two distinct confirmations to enter P6. Ordinary off-axis targets at 6–30 cm can hand over early for bounded alignment. Only P6 may move after a close hold; its motor thread independently rechecks current evidence, shutdown, IMU disturbances and the pulse deadline.
- CSV schema 3 / cone diagnostic schema 5 adds `ConeCloseTrackReason`, `ConeCloseTrackCount`, `ConeCloseTrackEligible`, and `ConeCloseTrackHold`. Eligibility is the last published frame decision; consumers additionally check live sensor freshness and the history deadline. Older CSVs lack these fields and remain readable.

- Keep detector acquisition/normalization in the sensor/vision layer and mission interpretation in the Phase.
- Require short-term consistency for P4 detection; a weak single frame must not transition.
- A fresh visible cone overrides capture windows, local/cumulative P4/P5 timeouts, and GPS disagreement. Follow its current image position; resume search only after visual loss. The global mission deadline still stops the mission.
- At the image edge, use a moderated both-wheels-forward arc with image-error hysteresis. Issue at most one motion command per new camera frame. Estimate the frame period from observation timestamps and stop when the next frame is late, bounded by the configured hold ceiling; keep stale-frame and global-stop checks responsive.
- Keep the camera detector and capture pipeline inactive throughout P0-P3. Activate them in P4/P5/P6, and release them whenever the mission returns to a non-vision phase; a camera disconnect before P4 must not affect navigation or motor control.
- Compensate P4 candidate direction with heading when available and reject discontinuous vertical position or scale before confirmation.
- In P5, require distinct fresh aligned camera/range pairs to enter P6. Retain visual identity/quality gates, but do not require the old close-occupancy threshold. Stop while confirming matched pairs. Reset on mismatch and bound a visual-close wait with unavailable range.
- Distinguish cone loss (`P5 -> P4`) from a P4 timeout or exhausted camera recovery (`P4 -> P7`).
- In P4, allow at most three camera recreations at five-second intervals and require a valid captured frame before declaring recovery; retain at least a 15-second recovery window.
- Without a visible cone, ordinary P4/P5 timeouts stop motors and skip final approach. A latched close hold may enter P6 for stationary recovery only. Time alone never authorizes motion or success.
- Do not let a camera failure create an unbounded search/approach loop.
- Keep the legacy camera relay outside the production architecture.

## Ranging and future obstacle perception

- Remove all sonar-only obstacle avoidance from P2/P3/P4. A near echo does not establish an impassable obstacle or a stuck vehicle. Preserve heading-sensor health checks and unrelated navigation recovery.
- Reserve future obstacle/stuck detection for image interpretation and fused evidence of range and actual progress; do not implement a speculative avoidance fallback.
- Timestamp individual echo attempts before GPIO Zero smoothing. Failed attempts invalidate the range immediately; rereading an attempt never refreshes it. Attempt sequence includes failures; goal confirmation requires new valid observations from both sensors.
- Keep distance/validity/time in shared sonar state. Put camera/range matching in `mission/goal.py`, decisions in Phase handlers, and deadline/freshness interlocks in MotorManager.
- P6 stops on missing, stale or incompatible observations. After at least 0.3 s settling, require distinct fresh time-aligned camera/sonar pairs acquired after settling. A heading change over 5 degrees (including an IMU excursion between control ticks), or range jump over 8 cm, restarts settling and clears votes.
- Success requires at least three 2–3 cm votes among the last five qualified stopped pairs, all within three seconds, with the newest pair also at 2–3 cm. A 3.2 cm outlier is not a near vote; distances below 2 cm remain invalid. No averaging changes these boundaries. Motion always clears votes.
- Resume forward motion only after two stopped pairs above 3.5 cm. Use 0.05 s pulses at up to 8 cm, otherwise 0.15 s. Two ordinary off-axis pairs in the same direction may authorize a 0.05 s forward-wheel alignment pivot only at 6 cm or farther; clipped images never authorize alignment. Recheck direction and clearance in the motor loop. P6 uses nonblocking PWM writes so the 20 ms motor loop can enforce deadlines (subject to scheduler latency).
- Share a 12-pulse budget across translation and alignment. Four forward pulses must reduce measured range by at least 0.5 cm before another group. Limit P6 to 20 s total and unavailable qualified observations to six seconds; terminate unconfirmed instead of retrying P5 indefinitely.
- Monitor camera frame age in P6 independently of the acquisition worker. Request one recreation when stale, on the existing camera worker if it returns; six seconds without frames terminates with `GOAL_CAMERA_TIMEOUT`. Camera close waits at most one second, detaches the device, and defers recreation while close is blocked so shutdown can write its final record. Driver recovery still requires a new valid frame.
- Count proximity confirmations only from images and echoes acquired after stopping and settling. Never count motion-time or duplicate observations as stopped confirmations.
- Sonar measures a reflector, not its identity. Central alignment and visual plausibility reduce but do not eliminate grass/cone association errors. Validate mounting and target geometry on hardware.
- The active target is sensor-face proximity intended to bring the projecting vehicle front into contact. Thresholds and motion values live in `mission/const.py`; contact itself remains unverified without additional evidence.

## Timeout and terminal semantics

- Derive the global timeout from cumulative Phase budgets, final-approach allowance, and transition margin.
- Preserve the minimum P3 navigation reserve when changing earlier budgets.
- Accumulate P3/P4/P5 time across re-entry.
- On global timeout, force safe terminal handling; do not perform the final approach.
- `GOAL_REACHED` is a legacy visual-completion reason. Current P6 emits `GOAL_PROXIMITY_CONFIRMED`, which confirms stopped proximity, not physical contact. The requested objective is contact; the sensor-to-front offset is estimated, not measured.
- `GOAL_PROXIMITY_UNCONFIRMED`, `GOAL_CAMERA_TIMEOUT`, `GOAL_MOTION_LIMIT`, `GOAL_NO_PROGRESS`, and `GOAL_APPROACH_TIMEOUT` are current non-success exits (`GOAL_RANGE_UNCONFIRMED` / `GOAL_OBSERVATION_LOST` remain legacy reasons). Do not convert elapsed time into success.
- `PHASE5_VISUAL_LOST_TIMEOUT` reports give-up without a visible cone. `PHASE5_TIMEOUT_FORCED_GOAL` remains a legacy-log reason; current P5 timeouts never authorize blind forward motion.
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
