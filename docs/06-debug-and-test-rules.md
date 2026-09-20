# 06 Debug and Test Rules

> **Audience: AI coding and debugging agents.** Read for every defect, regression, test, or field-failure task.

## Diagnosis order

Use evidence in this order:

1. Identify the run manifest, software revision, observed Phase, last motor command, transition/terminal reason, and elapsed time from the mission CSV.
2. Check validity, freshness, sequence, and recovery fields for the sensors used by that decision.
3. Reconstruct the exact handler and Manager path; do not tune a threshold before locating the failed contract.
4. Reproduce the decision with a `runs/spec/` fake when physical I/O is not essential.
5. Isolate hardware with one `runs/diag/` tool when acquisition or actuation remains suspect.
6. Re-run the smallest `runs/orch/` Phase range that contains the failure.
7. Run full E2E only after lower tiers pass.
8. Use `analysis/log.py` or `analysis/explorer.py` to validate the explanation across the full run.

Do not diagnose from console text alone, and do not treat one plausible numeric value as proof of sensor health.

## Test tiers

| Tier | Location | Proves | Does not prove |
| --- | --- | --- | --- |
| Specification | `runs/spec/` | Math, validation, decision boundaries, schema, cleanup contracts | Physical wiring or device health |
| Device diagnostic | `runs/diag/` | One device/interface on actual hardware | Phase strategy or full integration |
| Vision evidence | `runs/cam/` | Capture/detector behavior under field images | Mission-wide termination |
| Partial E2E | `runs/orch/` | Real controller, Managers, threads, and selected Phase range | Earlier skipped Phase assumptions |
| Full E2E | `main.py` | Production entry and complete mission path | Repeatability unless logs/runs are compared |
| Offline analysis | `analysis/` | Explanation, coverage, trends, anomaly evidence | Runtime safety by itself |

Run all hardware-free specifications from the repository root:

```bash
python3 -m unittest discover -s runs/spec -v
```

Do not replace this with a page of per-file commands. Select targeted modules from `runs/spec/` only when iterating locally; run the full suite before handoff.

## Regression mapping

| Changed concern | Minimum specs to inspect |
| --- | --- |
| Startup or shutdown | `controller_exception_safety.py`, `mission_config.py` |
| Config or radio | `mission_config.py`, `radio_control.py` |
| Log/state fields | `log_schema.py` |
| Phase 0 | `p0_detect.py` |
| Phase 2 or budgets | `phase2_flow.py`, `phase3_heading.py` |
| Navigation/arrival | `navigation_flow.py`, `phase3_heading.py` |
| Sonar/goal proximity | `sonar_diag.py`, `sonar_driver.py`, `sonar_freshness.py`, `goal_approach.py`, `phase6_flow.py` |
| Motor mapping/diagnostic | `motor_diag_safety.py`, `phase3_heading.py` |

Add a new focused spec when no row owns the changed boundary.

## E2E rules

- Treat `runs/orch/` as E2E for a bounded Phase range, not as a replacement mission implementation.
- Keep orchestration differences limited to start Phase, allowed Phase set, debug label, and log directory.
- Remember that a later-Phase start skips earlier physical and calibration assumptions; initialize only what the production controller normally initializes, and document the skipped assumption in the test record.
- Use a real, locally reviewed `mission.toml`; `main.py` does not accept target/radio overrides.
- Start with radio control `off`, wheels lifted or low speed, and an operator-controlled power cutoff.
- Confirm log creation and a stationary safe state before allowing movement.
- Verify motor stop on subset exit, exception, Ctrl-C, and Phase 7.
- Inspect the CSV and reached image in the selected log directory before declaring success.
- Promote one risk step at a time: spec -> device diagnostic -> smallest orchestration range -> full mission.
- Apply all physical preconditions in [`reference/hardware-safety.md`](reference/hardware-safety.md).

## Failure triage

| Symptom | Inspect first | Do not do first |
| --- | --- | --- |
| Wrong Phase transition | transition reason, confirmation count, entry marker, cumulative time | Change the threshold |
| Rover drives wrong direction | logical command, `motor_map.py`, direction/trim specs | Swap pins in a Phase |
| GPS oscillation | fix quality, sequence, baseline, heading source/trust, BNO offset | Use raw coordinates without validity |
| Frozen or jumping heading | BNO stale/recovery fields, GPS alignment, motor command | Re-enable raw magnetometer fallback |
| False goal proximity | camera/range alignment, `SonarSeq`, observation timestamps, stopped confirmation count | Treat a cached range or a grass reflection as cone contact |
| Cone false positive/loss | probability, method, direction consistency, saved frames | Tune multiple ROI/threshold variables together |
| Missing/shifted CSV fields | header/row length spec, producer, analysis coverage | Patch only the parser |
| Wi-Fi remains off | radio event, restore deadline, shutdown path | Depend on SSH as the only recovery |
| Mission stops with optional subsystem | exception boundary and fallback | Hide the exception |

## Evidence standard

- Preserve the complete failing run bundle or legacy log and its input fixture.
- State the causal chain: observation -> validity -> decision -> command/transition -> terminal result.
- For threshold changes, compare before/after evidence across more than one sample when possible.
- Record which test tier passed and which hardware-dependent tier was not run.
- Do not claim field/E2E verification from unit tests.

## AI Checklist

- Did I start from structured evidence and the exact Phase path?
- Did I distinguish decision failure from device failure?
- Is the regression locked at the lowest useful test tier?
- Were E2E preconditions and skipped Phase assumptions explicit?
- Did I verify terminal motor/radio/log behavior?
