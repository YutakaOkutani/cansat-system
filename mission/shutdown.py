"""Process-level deadline, enabled only by executable mission runners.

Keep the daemon watchdog armed after run() returns: a driver-owned non-daemon
thread or an interpreter exit hook must not keep the executable alive forever.
"""
import os
import threading

from mission.const import PROCESS_SHUTDOWN_TIMEOUT_SEC, PHASE7_ERROR_EXIT_CODE
from mission.diagnostics import lifecycle


class ProcessExitDeadline:
    def __init__(self, controller, timeout=PROCESS_SHUTDOWN_TIMEOUT_SEC):
        self.controller = controller
        self.timeout = timeout
        self._lock = threading.Lock()
        self._armed = False

    def arm(self):
        with self._lock:
            if self._armed:
                return
            self._armed = True
            timer = threading.Timer(self.timeout, self._expire)
            timer.daemon = True
            timer.start()

    def _expire(self):
        c = self.controller
        # No state lock here: the blocked operation might hold that lock.
        exit_code = PHASE7_ERROR_EXIT_CODE if getattr(c, '_terminal_phase_reached', False) else 1
        previous = getattr(c, 'lifecycle_diagnostics', {})
        lifecycle(c, ShutdownStage='forced_exit',
                  ShutdownError='process_exit_deadline:' + previous.get('ShutdownStage', 'unknown'))

        def report():
            print(f'Shutdown deadline exceeded; forcing process exit (code {exit_code}).', flush=True)
            c._write_final_log_row()

        # Logging/stdout may be the blocked operation. Neither may postpone exit.
        worker = threading.Thread(target=report, daemon=True)
        worker.start()
        worker.join(timeout=0.2)
        os._exit(exit_code)
