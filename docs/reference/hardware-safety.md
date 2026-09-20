# Hardware and Radio Safety Contract

> **Audience: AI agents changing or running GPIO, sensors, motors, radio control, or real-hardware E2E.** Load only for those tasks. These constraints are not safely recoverable from Python alone.

## HC-SR04 electrical constraint

- Do not connect a standard 5 V HC-SR04 `ECHO` output directly to Raspberry Pi GPIO24.
- Treat Raspberry Pi GPIO input as 3.3 V-only.
- Treat the released `gerber/NSE2026 v2_2026-04-13.zip` board as lacking an ECHO level-conversion divider.
- Direct connection is allowed when the module's ECHO output meets the Raspberry Pi GPIO input limits under the actual supply conditions (for example, a suitable 3.3 V-output module or one with integrated level conversion). Check the specific module's documentation or pulse voltage; successful readings alone do not establish electrical compatibility.
- For a 5 V ECHO output, add external level conversion before connecting it to GPIO. If the output specification is unknown, establish it before choosing the wiring. The established divider for a 5 V output is:

```text
HC-SR04 ECHO -- 10 kΩ --+-- GPIO24
                        |
                       10 kΩ
                        |
                       GND
```

- This divider reduces 5 V to about 2.5 V; verify the target Pi's input-high requirement. Do not apply it again to an already compatible 3.3 V output.
- Connect `TRIG` to GPIO23, power VCC according to the module specification (5 V for a standard HC-SR04), and share GND. Direct and level-converted ECHO wiring use the same software configuration.
- Do not infer electrical safety from successful readings or from a compatible product name.
- If ECHO was previously connected directly, check its output specification before reuse; add level conversion if it outputs 5 V.
- Do not modify the released Gerber archive to represent a future board revision.

## Motor bench constraint

- Lift wheels or begin at the lowest safe speed before testing direction, trim, ramp, or Phase motion.
- Provide a local operator and immediate power cutoff; do not rely on SSH for motor stop.
- Verify the logical direction through `mission/motor_map.py` and the motor safety specs before changing GPIO polarity.
- Exercise one command at a time and confirm stop after normal exit, Ctrl-C, input EOF, exception, and timeout.
- Keep physical mapping and trim in `mission/const.py`; do not compensate inside a Phase.
- Stop motors before camera, sensor, socket, or log cleanup.

## Radio control constraint

- Production radio configuration comes only from untracked `mission.toml`.
- Use `control = "off"` for ordinary bench work.
- `control = "mission"` may disable Wi-Fi only for a Phase 0 start.
- Restore Wi-Fi on Phase 0 exit, restore deadline, non-Phase-0 entry, and shutdown.
- Keep `dry_run` available to verify command construction without invoking `rfkill`.
- When `use_sudo` is enabled, allow only non-interactive `rfkill block wifi` and `rfkill unblock wifi`; do not broaden privilege.
- Prepare a local recovery path before a real block test because SSH will disconnect.
- A radio command failure must be logged but must not take over mission control.
- Telemetry must never become the restoration or mission-safety authority.

## Hardware promotion gate

Before a real-hardware E2E:

1. Pass the full `runs/spec/` suite.
2. Verify configuration locally with radio off.
3. Run the smallest relevant `runs/diag/` check.
4. Run the smallest relevant `runs/orch/` range with a separate debug log directory.
5. Inspect validity/staleness, command types, transition reasons, and final stop in the CSV.
6. Only then run `main.py`.

Do not claim a tier was run when hardware or operator safety conditions were unavailable.

## AI Checklist

- Does ECHO meet GPIO input limits, either directly or through suitable level conversion?
- Are wheels controlled and a local power cutoff available?
- Did motor mapping remain centralized and did every exit stop?
- Is radio off for ordinary bench work with a local recovery path for real tests?
- Did verification progress from specs to the smallest real-hardware scope?
