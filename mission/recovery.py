"""Small durable checkpoint for main.py; never restore measurements or PWM.

Phase algorithms still own their decisions. Restart their observation windows,
retain consumed budgets and completed stages, and keep Phase7 terminal on disk.
"""
import argparse
from dataclasses import asdict
import fcntl
import json
import math
import os
from pathlib import Path
import time

from mission.const import Phase, RECOVERY_SAVE_INTERVAL_SEC, RECOVERY_CLOCK_TOLERANCE_SEC
from mission.paths import DEFAULT_DATA_ROOT


STATE_PATH = DEFAULT_DATA_ROOT / 'mission-state.json'


class RecoveryError(RuntimeError):
    pass


def identity(config):
    return {'target': asdict(config.target), 'radio': asdict(config.radio)}


class RecoveryStore:
    def __init__(self, path=STATE_PATH):
        self.path = Path(path)
        self.record = None
        self._last_key = None
        self._last_save = float('-inf')

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = self.path.with_suffix('.lock').open('a')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._lock.close()
            raise RecoveryError('A full mission is already running; state is locked') from exc
        return self

    def __exit__(self, *_):
        self._lock.close()

    def load(self, config=None):
        if not self.path.exists():
            return None
        try:
            record = json.loads(self.path.read_text())
            if record['schema'] != 1 or type(record['phase']) is not int or not 0 <= record['phase'] <= 7:
                raise ValueError('unsupported schema or phase')
            if not isinstance(record['mission_id'], str) or not record['mission_id']:
                raise ValueError('missing mission identity')
            if type(record['resume_count']) is not int or record['resume_count'] < 0:
                raise ValueError('invalid resume count')
            if not isinstance(record['run_id'], str) or not isinstance(record['reason'], str):
                raise ValueError('invalid provenance')
            for value in [record['saved_at'], record['mission_elapsed'], record['visit_elapsed'],
                          record['stage_elapsed'], *record['phase_elapsed'].values()]:
                if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                    raise ValueError('invalid saved time')
            if set(record['phase_elapsed']) != {str(i) for i in range(8)}:
                raise ValueError('incomplete phase budgets')
            if record['phase2_stage'] not in ('escape', 'calibration', 'offset'):
                raise ValueError('invalid Phase2 stage')
            if type(record['phase6_pulses']) is not int or record['phase6_pulses'] < 0:
                raise ValueError('invalid pulse count')
            if config is not None and record['config'] != identity(config):
                raise ValueError('mission configuration changed; explicitly reset before a new mission')
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            raise RecoveryError(f'Cannot resume {self.path}: {exc}') from exc
        self.record = record
        return record

    def _write(self, record):
        temp = self.path.with_suffix('.tmp')
        try:
            with temp.open('w') as stream:
                json.dump(record, stream, allow_nan=False, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except (OSError, ValueError) as exc:
            raise RecoveryError(f'Checkpoint write failed: {exc}') from exc
        self.record = record

    def bind(self, c, config):
        previous = self.record
        self.config = identity(config)
        self.mission_id = previous['mission_id'] if previous else c.run_id
        self.resume_count = previous['resume_count'] + 1 if previous else 0
        c.recovery_store = self
        c.recovery_record = previous
        c._recovery_starting = True
        from mission.diagnostics import lifecycle
        metadata = dict(RecoveryMissionId=self.mission_id, RecoveryCount=self.resume_count,
                        RecoveryFromRunId=previous['run_id'] if previous else '',
                        RecoveryFromPhase=previous['phase'] if previous else '')
        lifecycle(c, **metadata)
        c.run_bundle.record_recovery(metadata)
        if previous is None:
            # Persist the mission before hardware setup can fail or power drops.
            self._write(dict(schema=1, phase=0, config=self.config, mission_id=self.mission_id,
                             run_id=c.run_id, resume_count=0, saved_at=time.time(),
                             mission_elapsed=0, phase_elapsed={str(i): 0 for i in range(8)},
                             visit_elapsed=0, phase2_stage='escape', stage_elapsed=0,
                             phase6_pulses=0, reason='RUNNING'))

    def restore(self, c):
        record = c.recovery_record
        if record is None:
            return
        now = time.time()
        if now + RECOVERY_CLOCK_TOLERANCE_SEC < record['saved_at']:
            raise RecoveryError('Clock moved backwards; mission deadline cannot be restored')
        c.mission_start_time = now - record['mission_elapsed'] - max(0, now - record['saved_at'])
        c.phase_elapsed_totals = {Phase(int(k)): v for k, v in record['phase_elapsed'].items()}
        phase = Phase(record['phase'])
        # initialize_phase already established a new visit marker. Do not double
        # count saved phase time by backdating that marker as well.
        visit = record['visit_elapsed']
        c._recovery_visit_elapsed = visit
        if phase == Phase.PHASE1:
            c.time_phase1_start = now - visit
        elif phase == Phase.PHASE2:
            from mission.phases.p2 import Phase2Handler
            Phase2Handler._enter_stage(c, record['phase2_stage'], now, c.st.snapshot())
            c.phase2_stage_start = now - record['stage_elapsed']
        elif phase == Phase.PHASE3:
            c.time_phase3_start = now - visit
            # Existing Phase3 GPS-only fallback must re-establish live heading.
            c.phase3_heading_entry_ready = False
        elif phase == Phase.PHASE5:
            c.phase5_resume_elapsed = visit
        elif phase == Phase.PHASE6:
            c.phase6_resume_elapsed = visit
            c.phase6_resume_pulses = record['phase6_pulses']
            # The initial checkpoint runs before the handler consumes resume_*.
            c.phase6_pulses = record['phase6_pulses']
        print(f'Resuming mission {self.mission_id}: Phase{int(phase)}; attempt={self.resume_count}', flush=True)

    def save(self, c, *, force=False):
        if not getattr(c, '_recovery_ready', False):
            return
        wall, mono = time.time(), time.monotonic()
        phase = int(c.st.snapshot()['phase'])
        c._sync_phase_time_tracking(Phase(phase))
        stage = getattr(c, 'phase2_stage', 'escape')
        pulses = getattr(c, 'phase6_pulses', 0)
        key = (phase, stage, pulses)
        if not force and key == self._last_key and mono - self._last_save < RECOVERY_SAVE_INTERVAL_SEC:
            return
        previous_phase = self.record['phase']
        if phase != previous_phase:
            c._recovery_visit_elapsed = 0
        visit = max(0, wall - c.phase_entry_time) + getattr(c, '_recovery_visit_elapsed', 0)
        record = dict(schema=1, phase=phase, config=self.config, mission_id=self.mission_id,
                      run_id=c.run_id, resume_count=self.resume_count, saved_at=wall,
                      mission_elapsed=max(0, wall - c.mission_start_time),
                      phase_elapsed={str(int(p)): c._current_phase_elapsed(p, wall) for p in Phase},
                      visit_elapsed=visit, phase2_stage=stage,
                      stage_elapsed=max(0, wall - (getattr(c, 'phase2_stage_start', None) or wall)),
                      phase6_pulses=pulses, reason=c.mission_end_reason)
        self._write(record)
        self._last_key, self._last_save = key, mono

    def reset(self):
        self.path.unlink(missing_ok=True)
        self.path.with_suffix('.tmp').unlink(missing_ok=True)
        directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)


def main():
    parser = argparse.ArgumentParser(description='Inspect or reset mission recovery while stopped')
    parser.add_argument('action', choices=('status', 'reset'))
    args = parser.parse_args()
    with RecoveryStore() as store:
        if args.action == 'reset':
            store.reset()
            print('Recovery state cleared. Next full mission starts at Phase0.')
        else:
            print(json.dumps(store.load(), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
