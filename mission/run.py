from mission.config import load_mission_config, load_run_context
from mission.const import Phase, PHASE7_ERROR_EXIT_CODE, RECOVERY_ERROR_EXIT_CODE
import traceback


def _build_controller(log_dir=None, config=None):
    config = config if config is not None else load_mission_config()
    run_context = load_run_context()
    from mission.ctrl import CanSatController

    print(
        f"Mission config: {config.source}; "
        f"target=({config.target.latitude:.6f}, {config.target.longitude:.6f}); "
        f"radio={config.radio.control}; dry_run={int(config.radio.dry_run)}; "
        f"use_sudo={int(config.radio.use_sudo)}"
    )
    print(
        f"Run context: event={run_context.event_id}; kind={run_context.run_kind}; "
        f"label={run_context.label or '-'}; source={run_context.source or 'default'}"
    )
    controller = CanSatController(config, run_context, log_dir=log_dir)
    from mission.shutdown import ProcessExitDeadline

    controller.arm_process_exit_deadline = ProcessExitDeadline(controller).arm
    print(f"Run bundle: {controller.run_dir}")
    return controller


def _run_controller(controller, **kwargs):
    """Only Phase7 cleanup errors opt out of systemd's failure recovery."""
    try:
        controller.run(**kwargs)
    except Exception:
        if not getattr(controller, '_terminal_phase_reached', False):
            raise
        traceback.print_exc()
        raise SystemExit(PHASE7_ERROR_EXIT_CODE)
    if (getattr(controller, '_terminal_phase_reached', False)
            and getattr(controller, 'lifecycle_diagnostics', {}).get('ShutdownError')):
        raise SystemExit(PHASE7_ERROR_EXIT_CODE)


def run_full_mission(log_dir=None):
    from mission.recovery import RecoveryStore, RecoveryError

    config = load_mission_config()
    try:
        with RecoveryStore() as recovery:
            record = recovery.load(config)
            if record is not None and record['phase'] == int(Phase.PHASE7):
                print(f"Mission already ended: {record['reason']}; no hardware started. "
                      "Use python -m mission.recovery reset for a new mission.", flush=True)
                return
            controller = _build_controller(log_dir=log_dir, config=config)
            recovery.bind(controller, config)
            _run_controller(controller, start_phase=Phase(record['phase']) if record else Phase.PHASE0)
    except RecoveryError as exc:
        print(f'Recovery refused: {exc}', flush=True)
        raise SystemExit(RECOVERY_ERROR_EXIT_CODE) from exc


def run_phase_sequence(start_phase, allowed_phases, log_dir=None):
    controller = _build_controller(log_dir=log_dir)
    _run_controller(controller, start_phase=start_phase, allowed_phases=allowed_phases)


def run_single_phase(phase, log_dir=None):
    run_phase_sequence(
        start_phase=phase,
        allowed_phases=(phase,),
        log_dir=log_dir,
    )
