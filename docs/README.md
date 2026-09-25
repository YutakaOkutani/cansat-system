# AI Context Index

> **Audience: AI coding and debugging agents only.** Human setup and production execution guidance belongs in [`../README.md`](../README.md); sonar wiring, diagnostics, and approach details belong in [`operations/sonar.md`](operations/sonar.md). Do not load every file by default.

## Required read order

Read only the shortest route that covers the task:

1. Always read [`01-repository-overview.md`](01-repository-overview.md).
2. Before changing code, read [`02-architecture.md`](02-architecture.md), [`03-design-principles.md`](03-design-principles.md), [`04-repository-rules.md`](04-repository-rules.md), and [`05-coding-rules.md`](05-coding-rules.md), in that order.
3. For a defect or test change, also read [`06-debug-and-test-rules.md`](06-debug-and-test-rules.md).
4. For mission behavior, phase transitions, navigation, sensors, motors, camera, timeouts, or logs, also read [`07-mission-spec.md`](07-mission-spec.md).
5. Load a reference only when its trigger applies:
   - Physical hardware, GPIO, motors, Wi-Fi shutdown: [`reference/hardware-safety.md`](reference/hardware-safety.md)
   - `runs/telemetry/` or `runs/cam/relay_*`: [`reference/telemetry-contract.md`](reference/telemetry-contract.md)
   - Competition policy, timing provenance, goal marker, or field assumptions: matching page under [`competitions/`](competitions/)
6. Load [`maintenance/context-audit.md`](maintenance/context-audit.md) only when maintaining `docs/`.

The numbered files are ordered context, not tutorials. Source code remains authoritative for implementation details and current numeric constants.

Legacy `index.md`, `development/testing.md`, `operations/radio_control.md`, and `telemetry/*.md` are link-preserving aliases. Never load them after their owner pages.

## Structure

```text
docs/
├── README.md
├── 01-repository-overview.md
├── 02-architecture.md
├── 03-design-principles.md
├── 04-repository-rules.md
├── 05-coding-rules.md
├── 06-debug-and-test-rules.md
├── 07-mission-spec.md
├── reference/
│   ├── hardware-safety.md
│   └── telemetry-contract.md
├── competitions/
│   └── nse2026.md
├── flowchart/
│   └── .gitkeep
├── maintenance/
│   └── context-audit.md
└── legacy aliases (do not load as context)
```

## Document ownership

| File | Sole responsibility | Do not duplicate here |
| --- | --- | --- |
| `01-repository-overview.md` | Repository map and task routing | Layer internals, phase algorithms |
| `02-architecture.md` | Dependency direction and layer contracts | Coding procedure, threshold values |
| `03-design-principles.md` | Decision priorities and invariants | File inventory, commands |
| `04-repository-rules.md` | Change boundaries and prohibited actions | Python style, mission algorithms |
| `05-coding-rules.md` | Implementation rules by concern | Debug procedure, phase specification |
| `06-debug-and-test-rules.md` | Diagnosis order, test tiers, E2E safety | Architecture explanation |
| `07-mission-spec.md` | Mission state machine and phase intent | Full constant tables |
| `operations/sonar.md` | Human sonar wiring, diagnostic procedure, and approach details | General setup |
| `reference/*` | High-risk or subsystem-specific contracts | General repository guidance |
| `competitions/*` | Competition provenance and rule-derived assumptions | Full constant tables or runtime selection logic |
| `flowchart/` | Reserved location for future source-controlled diagrams | Current mission specification |
| `maintenance/context-audit.md` | Context value audit and refactor record | Runtime guidance |
| Legacy alias pages | Preserve links from files outside the docs refactor scope | New information |

## Context rules

- Treat these documents as constraints. Verify claims against code before editing.
- Do not copy source-visible function lists, CLI help, complete schemas, or constant tables into `docs/`.
- Record a rule only when its absence could cause a wrong design, unsafe test, or incompatible change.
- Update the owning page and its links in the same change. Do not describe one rule in multiple pages.
- End every Markdown page in `docs/` with an `AI Checklist`.

## AI Checklist

- Have I loaded only the route required by this task?
- Have I read the numbered pages in order before changing code?
- Have I treated source code as authoritative for current implementation details?
- Am I updating the one document that owns this information?
