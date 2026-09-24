from __future__ import annotations

import hashlib
import json
import shutil
import socket
import subprocess
import threading
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

from mission.const import MISSION_LOG_SCHEMA_VERSION, ROI_GLOB_PATTERNS
from mission.paths import DEFAULT_RUNS_ROOT, REPO_ROOT, ROI_REFERENCE_DIR


RUN_MANIFEST_SCHEMA = "cansat.run.v1"


def _git_metadata() -> dict[str, object]:
    result: dict[str, object] = {"commit": None, "dirty": None}
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        ).stdout
        result.update(commit=commit, dirty=bool(status.strip()))
    except (OSError, subprocess.SubprocessError):
        pass
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass
class RunBundle:
    run_id: str
    run_dir: Path
    log_path: Path
    camera_dir: Path
    reached_image_path: Path
    manifest_path: Path
    _manifest: dict[str, object]

    def __post_init__(self) -> None:
        self._manifest_lock = threading.Lock()

    def _write_manifest(self) -> None:
        temp_path = self.manifest_path.with_suffix(".json.tmp")
        temp_path.write_text(
            json.dumps(self._manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temp_path.replace(self.manifest_path)

    def snapshot_roi_inputs(self, roi_items: list[dict[str, object]]) -> None:
        if not roi_items or self._manifest.get("roi_inputs"):
            return
        input_dir = self.run_dir / "inputs" / "roi"
        input_dir.mkdir(parents=True, exist_ok=True)
        records = []
        for item in roi_items:
            source = Path(str(item.get("path", "")))
            if not source.is_file():
                continue
            destination = input_dir / source.name
            shutil.copy2(source, destination)
            records.append(
                {
                    "source": str(source.resolve()),
                    "snapshot": str(destination.relative_to(self.run_dir)),
                    "sha256": _sha256(destination),
                    "label": str(item.get("label", "")),
                    "weight": float(item.get("weight", 1.0)),
                }
            )
        if records:
            with self._manifest_lock:
                self._manifest["roi_inputs"] = records
                self._write_manifest()

    def finalize(self, reason: str) -> None:
        with self._manifest_lock:
            self._manifest["status"] = "finished"
            self._manifest["finished_at"] = datetime.now().astimezone().isoformat()
            self._manifest["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._manifest["end_reason"] = str(reason)
            self._write_manifest()

    def record_recovery(self, metadata) -> None:
        with self._manifest_lock:
            self._manifest['recovery'] = dict(metadata)
            self._write_manifest()


def create_run_bundle(mission_config, run_context, *, log_root=None, now=None) -> RunBundle:
    started_at = now or datetime.now().astimezone()
    if started_at.tzinfo is None:
        started_at = started_at.astimezone()
    timestamp = started_at.strftime("%Y%m%d-%H%M%S")
    run_id = f"{timestamp}-{uuid.uuid4().hex[:8]}"

    if log_root is None:
        parent = DEFAULT_RUNS_ROOT / run_context.event_id / run_context.run_kind
    else:
        parent = Path(log_root).expanduser()
    run_dir = parent / run_id
    run_dir.mkdir(parents=True, exist_ok=False)

    log_path = run_dir / f"mission_{run_id}.csv"
    camera_dir = run_dir / "camera"
    reached_image_path = run_dir / "capture_reached.png"
    manifest_path = run_dir / "run-manifest.json"

    mission_snapshot = run_dir / "mission-config.toml"
    shutil.copy2(mission_config.source, mission_snapshot)
    context_snapshot = None
    if run_context.source is not None and Path(run_context.source).is_file():
        context_snapshot = run_dir / "run-context.toml"
        shutil.copy2(run_context.source, context_snapshot)

    manifest = {
        "schema": RUN_MANIFEST_SCHEMA,
        "status": "running",
        "run_id": run_id,
        "started_at": started_at.isoformat(),
        "started_at_utc": started_at.astimezone(timezone.utc).isoformat(),
        "hostname": socket.gethostname(),
        "software": _git_metadata(),
        "log_schema_version": MISSION_LOG_SCHEMA_VERSION,
        "context": asdict(run_context) | {"source": str(run_context.source) if run_context.source else None},
        "artifacts": {
            "mission_log": log_path.name,
            "camera_dir": camera_dir.name,
            "reached_image": reached_image_path.name,
            "mission_config_snapshot": mission_snapshot.name,
            "run_context_snapshot": context_snapshot.name if context_snapshot else None,
        },
        "roi_reference_root": str(ROI_REFERENCE_DIR),
        "roi_patterns": list(ROI_GLOB_PATTERNS),
    }
    bundle = RunBundle(
        run_id=run_id,
        run_dir=run_dir,
        log_path=log_path,
        camera_dir=camera_dir,
        reached_image_path=reached_image_path,
        manifest_path=manifest_path,
        _manifest=manifest,
    )
    bundle._write_manifest()

    if run_context.notes.strip():
        (run_dir / "notes.md").write_text(run_context.notes.rstrip() + "\n", encoding="utf-8")
    return bundle
