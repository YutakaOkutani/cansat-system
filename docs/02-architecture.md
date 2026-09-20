# 02 Architecture

> **Audience: AI coding and debugging agents.** Read before any code change. This page owns dependency direction and layer contracts.

## Production flow

```text
main.py
  -> mission.run loads and validates mission.toml
  -> CanSatController constructs state and runtime bookkeeping
  -> HardwareManager initializes devices, log, and worker threads
  -> sensor workers update CanSatState
  -> controller loop snapshots state and dispatches one Phase handler
  -> Phase handler records decisions/transitions through controller APIs and state
  -> motor worker applies phase-aware actuator policy through MotorManager
  -> log worker snapshots state plus controller diagnostics
  -> Phase7/request_shutdown restores radio, stops motors, and writes a final row
```

Do not bypass this flow in production changes.

## Module ownership

| Module | Owns | Must not own |
| --- | --- | --- |
| `mission/config.py` | Strict local control/run-context parsing and immutable config objects | Hardware initialization or control decisions from metadata |
| `mission/paths.py`, `mission/run_bundle.py` | Runtime data layout, run identity, metadata/config snapshots | Phase decisions or hardware policy |
| `mission/const.py` | Single-airframe pins, tuning, timing, budgets, schema constants | Target coordinates or runtime secrets |
| `mission/run.py` | Full, range, and single-Phase controller entry functions | Phase strategy or CLI parsing |
| `mission/ctrl.py` | Controller construction, lifecycle, dispatch, cumulative timeout, shutdown | Device protocol or Phase-local algorithms |
| `mission/st.py` | Locked shared facts and atomic snapshots | I/O, timers, or transition side effects |
| `mission/nav.py` | Side-effect-free distance/azimuth math | GPS transport or control decisions |
| `mission/gps_util.py` | GPS serial/NMEA parsing and quality helpers | Phase transitions |
| `mission/motor_map.py` | Logical direction to physical motor mapping | Phase-specific speeds or GPIO writes |
| `mission/mgr/hw_mgr.py` | Device/log/worker initialization | Mission success decisions |
| `mission/mgr/sns_mgr.py` | Acquisition, validation, freshness, state publication, CSV row building | Phase strategy |
| `mission/mgr/mtr_mgr.py` | Safe motor output and continuous phase-aware movement policy | Phase transition authority |
| `mission/mgr/led_mgr.py` | Operator-visible LED signals | Mission state authority |
| `mission/mgr/radio_mgr.py` | Bounded Wi-Fi block/restore execution | Telemetry or Phase strategy |
| `mission/phases/base.py`, `p0.py`-`p7.py` | Phase interface, entry state, decisions, transitions, reasons | Device-driver access or replacement workers |
| `lib/bmp180.py`, `lib/bno055.py` | Low-level sensor adapters | State-machine policy |
| `lib/cone_detect.py` | Cone detection and reached evidence | Phase transition policy |
| `lib/roi_capture.py` | ROI capture tool (timestamped captures are kept separate from the production reference) | Automatic detector tuning |
| `analysis/log.py` | Control/debug log analysis | Runtime control |
| `analysis/explorer.py` | Offline route/environment reconstruction | Authoritative physical map or runtime control |
| `analysis/log_selector.py` | Input-log selection | Log interpretation policy |

## Dependency direction

Use the following direction:

```text
entry point -> run/config -> controller -> phases + managers -> lib/device APIs
                                      \-> shared state
logs -> analysis
runs -> production APIs
telemetry -> published/copied state only
```

- A Phase may request actions through manager methods exposed by the controller; it must not import GPIO, serial, camera, or device drivers.
- A Manager may depend on `lib/`, constants, and shared state; it must not decide mission success or phase strategy.
- `lib/` must not import phase or controller behavior.
- `runs/` may import production APIs; production code must not import `runs/`.
- `analysis/` consumes logs; mission execution must not depend on analysis packages.
- Telemetry must observe state without becoming a control dependency.

Existing direct `controller.devices` access in Phase handlers is legacy coupling. Do not extend it; add a Manager-level operation when changing that behavior.

## Controller contract

`CanSatController` composes `HardwareManager`, `SensorManager`, `MotorManager`, `LedManager`, and `RadioManager`.

- Keep the controller responsible for lifecycle, phase dispatch, cross-phase time budgets, shared runtime bookkeeping, log location, and shutdown.
- Keep phase-specific decision logic in `mission/phases/`.
- Keep continuous sensor/actuator implementation in Managers.
- Do not turn `ctrl.py` into a second Phase or hardware layer.
- Route every exit path through idempotent `request_shutdown()` behavior.

## State contract

`CanSatState` is the lock-protected cross-thread observation boundary.

- Writers must use the typed `update_*` methods.
- Readers must use `snapshot()` and make one decision from one snapshot where possible.
- Store current sensor/navigation/vision facts in `CanSatState`.
- Store lifecycle history, timers, confirmation counters, and algorithm working memory on the controller only when they are not shared sensor facts.
- Represent a measurement with its validity/freshness signal. A retained numeric value is not proof that the measurement is current.
- Do not expose mutable internal lists or acquire the state lock outside `CanSatState`.

## Manager contract

- `HardwareManager`: initialize devices, initialize the CSV before workers, start workers, and leave unavailable devices explicit.
- `SensorManager`: validate/normalize input, track stale/recovery state, update `CanSatState`, and serialize log rows.
- `MotorManager`: own logical-to-physical mapping, ramps, clamping, phase-aware motion, final-approach deadline/freshness interlocks, and final stop. Sonar alone no longer triggers obstacle avoidance.
- `LedManager`: own operational signals.
- `RadioManager`: own `rfkill` invocation and restoration failsafe.

Hardware access, retries, and actuator safety belong here even when only one Phase currently uses them.

## Phase contract

- One handler call evaluates the supplied state and may update phase-local runtime state.
- A Phase owns its entry reset, success condition, fallback, timeout intent, and transition reason.
- A Phase must not start replacement sensor or motor threads.
- A Phase transition must remain visible in state/logs and must not silently skip safety cleanup.
- Backward transitions are valid recovery strategies, not exceptional control flow.

## Time and shutdown contract

- Enforce both Phase-local timers and controller-level cumulative budgets.
- Count cumulative budgets across re-entry to prevent Phase 3/4/5 loops from exceeding the mission envelope.
- Treat global timeout as a forced safe terminal path.
- On exceptions, interruption, subset completion, or normal termination: restore mission-controlled Wi-Fi, stop motors, and append a final log row.
- Keep shutdown safe when invoked more than once.

## Observability contract

- The mission CSV is the primary reconstruction record; standard output is an operator aid.
- Keep the header and serialized row ordered as one schema contract.
- Log decision inputs, validity/freshness, command type, transition/termination reason, and time.
- Give every execution a unique run ID and keep its CSV, reached image, camera evidence, manifest, configuration snapshot, and optional notes in one run directory.
- Keep control configuration in `mission.toml` and non-control competition/run metadata in `run-context.toml`; snapshot both without making metadata affect behavior.
- Keep versioned ROI references outside generated-data trees and snapshot the inputs actually used by a run.
- Let analysis consume both run bundles and legacy `robust_log_*.csv`; place new analysis output inside its source bundle.
- Keep offline analysis outside the control process.

## AI Checklist

- Does the dependency still point toward Managers and low-level libraries?
- Is the controller only orchestrating cross-cutting lifecycle work?
- Does shared cross-thread data pass through `CanSatState`?
- Can every exit stop motors and restore radio?
- Can logs reconstruct the decision and transition?
