"""Append-only CSV diagnostics; each owner publishes a replacement dictionary.

Blank means not observed, never an inferred successful completion.
"""

FINAL_DIAGNOSTIC_DEFAULTS = {
    'CloseTrackAnchorPresent': 0, 'CloseTrackCenterPresent': 0,
    'CloseTrackRemainingSec': '', 'CloseTrackCenterHeadingDeg': '',
    'Phase6Stage': 'inactive', 'Phase6Action': '', 'Phase6Gate': '',
    'Phase6ElapsedSec': '', 'Phase6WaitSec': '', 'Phase6SettleRemainingSec': '',
    'Phase6PulseRequestedCount': 0, 'Phase6VoteCount': 0,
    'GoalEvalConeSeq': '', 'GoalEvalSonarSeq': '', 'GoalEvalDistanceCm': '',
    'GoalEvalSampleSkewSec': '', 'GoalEvalReason': '', 'GoalEvalMatched': '', 'GoalEvalConfirmCount': 0,
    'GoalEvalElapsedSec': '',
    'Phase6PulseStartedCount': 0, 'Phase6LastPulseId': '',
    'Phase6LastPulseStartElapsedSec': '', 'Phase6LastPulseEndElapsedSec': '',
    'Phase6LastPulseDurationSec': '', 'Phase6LastPulseStopReason': '',
    'Phase6MotorGate': '',
    'MotorStopReason': 'unspecified',
    'ShutdownRequested': 0, 'ShutdownReason': '', 'ShutdownStage': 'not_requested',
    'ShutdownCompleted': 0, 'ShutdownError': '',
    'InterruptCount': 0, 'InterruptPhase': '', 'InterruptElapsedSec': '',
    'InterruptStage': '', 'RunExceptionType': '', 'RunFinallyReached': 0,
    'Phase7HandlerEntered': 0, 'TerminalLEDCommand': '',
    'HardwareCloseDevice': '', 'HardwareCloseCompleted': 0,
    'RecoveryMissionId': '', 'RecoveryCount': 0,
    'RecoveryFromRunId': '', 'RecoveryFromPhase': '',
}


def elapsed(controller, wall):
    start = getattr(controller, 'mission_start_time', None)
    return round(max(0.0, wall - start), 3) if start is not None else ''


def diagnostic_values(controller, snapshot, mono):
    values = dict(FINAL_DIAGNOSTIC_DEFAULTS)
    for owner in ('phase6_diagnostics', 'motor_diagnostics', 'lifecycle_diagnostics'):
        values.update(getattr(controller, owner, {}))
    track = snapshot.get('cone_close_track', {})
    values.update(CloseTrackAnchorPresent=int(bool(track.get('anchor_present', False))),
                  CloseTrackCenterPresent=int(bool(track.get('center_present', False))),
                  CloseTrackRemainingSec=round(max(0, track.get('deadline', 0) - mono), 3),
                  CloseTrackCenterHeadingDeg=track.get('center_heading', ''))
    return [values[key] for key in FINAL_DIAGNOSTIC_DEFAULTS]


def lifecycle(controller, **changes):
    controller.lifecycle_diagnostics = {
        **getattr(controller, 'lifecycle_diagnostics', {}), **changes,
    }
