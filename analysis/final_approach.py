"""Final approach and shutdown reports shared by both analysis entry points.

Uses only the standard library so old CSVs can be inspected without plotting.
"""
import csv
from pathlib import Path

from mission.diagnostics import FINAL_DIAGNOSTIC_DEFAULTS


def write_final_approach_report(log_path, out_dir):
    with Path(log_path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames or []
        rows = list(reader)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    keys = [key for key in (
        'ElapsedSec', 'Phase', 'MissionEndReason', 'Phase7ArrivalReason',
        'GoalDecision', 'GoalConfirmCount', 'ConeSeq', 'ConeStatus',
        'ConeCloseTrackReason', 'ConeCloseTrackEligible', 'ConeCloseTrackHold',
        'SonarDistanceCm', 'SonarValid', 'MotorCmdType',
        *FINAL_DIAGNOSTIC_DEFAULTS,
    ) if key in columns]
    with (out_dir / 'final_approach_timeline.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            if row.get('Phase') in ('5', '6', '7') or row.get('ShutdownRequested') == '1' or row.get('RunFinallyReached') == '1':
                writer.writerow(row)

    def last(key):
        if key not in columns or not rows:
            return 'unknown'
        # A cleared field is meaningful (e.g. no device remains in close()).
        return rows[-1].get(key, '') or '(none recorded)'

    lines = ['Final approach / shutdown', f'Source: {log_path}']
    for phase in ('6', '7'):
        first = next((r for r in rows if r.get('Phase') == phase), None)
        lines.append(f'Phase {phase} first recorded: ' +
                     (first.get('ElapsedSec', '?') + ' sec' if first else 'not recorded'))
    for key in ('MissionEndReason', 'Phase7HandlerEntered', 'TerminalLEDCommand',
                'RecoveryMissionId', 'RecoveryCount', 'RecoveryFromRunId', 'RecoveryFromPhase',
                'ShutdownReason', 'ShutdownStage', 'ShutdownCompleted',
                'ShutdownError', 'HardwareCloseDevice', 'HardwareCloseCompleted',
                'InterruptCount', 'InterruptPhase', 'InterruptElapsedSec', 'InterruptStage',
                'RunExceptionType', 'RunFinallyReached',
                'Phase6PulseRequestedCount', 'Phase6PulseStartedCount',
                'Phase6LastPulseDurationSec', 'Phase6LastPulseStopReason'):
        lines.append(f'{key}: {last(key)}')
    lines.extend([
        'Phase=7 records a state transition, not proof that its handler ran.',
        'ShutdownCompleted=1 means the shutdown routine finished; inspect ShutdownError too.',
        'RunFinallyReached=1 is not an OS exit-code or proof that the process exited.',
        'ShutdownStage=forced_exit indicates the process deadline (code 80 after Phase7, otherwise 1); final logging is best effort.',
        'Absent columns in older logs are unknown, not zero or success.',
        'Motor pulse counters/times record output commands, not measured wheel motion.',
    ])
    (out_dir / 'final_approach_summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
