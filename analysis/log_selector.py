from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path

from mission.paths import DEFAULT_DATA_ROOT

REPO_ROOT = Path(__file__).resolve().parents[1]
LEGACY_LOGS_DIR = REPO_ROOT / "analysis" / "robust_logs"
LOGS_DIR = LEGACY_LOGS_DIR
RUN_MANIFEST_NAME = "run-manifest.json"
BUNDLE_LOG_NAME = "mission.csv"
BUNDLE_LOG_PATTERN = "mission_[0-9]*.csv"


def _format_size(size_bytes: int) -> str:
    if size_bytes < 1024:
        return f"{size_bytes} B"
    elif size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f} KB"
    else:
        return f"{size_bytes / (1024 * 1024):.1f} MB"


def get_robust_log_candidates() -> list[Path]:
    """Return new run-bundle logs and legacy robust logs, newest first."""
    candidates = []
    if DEFAULT_DATA_ROOT.exists():
        candidates.extend(DEFAULT_DATA_ROOT.rglob(BUNDLE_LOG_NAME))
        candidates.extend(DEFAULT_DATA_ROOT.rglob(BUNDLE_LOG_PATTERN))
    if LEGACY_LOGS_DIR.exists():
        candidates.extend(LEGACY_LOGS_DIR.rglob(BUNDLE_LOG_NAME))
        candidates.extend(LEGACY_LOGS_DIR.rglob(BUNDLE_LOG_PATTERN))
        candidates.extend(LEGACY_LOGS_DIR.rglob("robust_log_*.csv"))
    candidates = list(dict.fromkeys(candidates))
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates


def load_run_manifest(log_path: Path) -> dict[str, object]:
    manifest_path = Path(log_path).parent / RUN_MANIFEST_NAME
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def is_run_bundle_log(log_path: Path) -> bool:
    path = Path(log_path)
    return (path.name == BUNDLE_LOG_NAME or path.match(BUNDLE_LOG_PATTERN)) and (
        path.parent / RUN_MANIFEST_NAME
    ).is_file()


def create_analysis_output_dir(log_path: Path, analyzer: str, legacy_root: Path) -> Path:
    if is_run_bundle_log(log_path):
        root = Path(log_path).parent / "analysis" / analyzer
    else:
        root = legacy_root / Path(log_path).stem
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = root / f"run_{stamp}"
    suffix = 1
    while out_dir.exists():
        suffix += 1
        out_dir = root / f"run_{stamp}_{suffix:02d}"
    out_dir.mkdir(parents=True, exist_ok=False)
    return out_dir


def find_latest_log() -> Path:
    """Return the newest run-bundle or legacy mission log path."""
    candidates = get_robust_log_candidates()
    if not candidates:
        raise FileNotFoundError(
            f"No mission logs found in {DEFAULT_DATA_ROOT} or {LEGACY_LOGS_DIR} (including robust_log_*.csv)"
        )
    return candidates[0]


def resolve_log_path(file_path: str | Path | None = None) -> Path:
    """Resolve log path.

    If file_path is provided, return its resolved Path.
    If file_path is None:
      - If stdin is an interactive terminal (isatty), display interactive log selection menu.
      - If stdin is non-interactive, return the latest log.
    """
    if file_path:
        path = Path(file_path).resolve()
        if path.is_dir():
            candidates = list(path.glob(BUNDLE_LOG_PATTERN))
            legacy_path = path / BUNDLE_LOG_NAME
            if legacy_path.is_file():
                candidates.append(legacy_path)
            if not candidates:
                raise FileNotFoundError(f"No mission log found in: {path}")
            if len(candidates) > 1:
                raise ValueError(f"Multiple mission logs found in {path}; specify a CSV file")
            path = candidates[0]
        if not path.exists():
            raise FileNotFoundError(f"Specified log file not found: {path}")
        return path

    candidates = get_robust_log_candidates()
    if not candidates:
        raise FileNotFoundError(
            f"No mission logs found in {DEFAULT_DATA_ROOT} or {LEGACY_LOGS_DIR} (including robust_log_*.csv)"
        )

    # Fallback to latest log in non-interactive environment (e.g. CI, pipe)
    if not sys.stdin.isatty():
        latest = candidates[0]
        print(f"[INFO] Non-interactive shell detected. Automatically selected latest log: {latest.name}")
        return latest

    # Display interactive selection menu
    total = len(candidates)
    width = len(str(total))

    print("\n" + "=" * 70)
    print("  Select a mission log to analyze")
    print("=" * 70)

    for i, candidate in enumerate(candidates, start=1):
        stat = candidate.stat()
        mtime_str = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
        size_str = _format_size(stat.st_size)
        tag = " (Latest)" if i == 1 else ""
        try:
            label = str(candidate.relative_to(DEFAULT_DATA_ROOT))
        except ValueError:
            label = candidate.name
        print(f" [{i:{width}d}] {label:<60} ({mtime_str}, {size_str}){tag}")

    print("=" * 70)

    while True:
        try:
            user_input = input(f"Select log number [1-{total}] (default: 1): ").strip()
            if not user_input:
                selected = candidates[0]
                print(f"[INFO] Selected default (Latest): {selected.name}\n")
                return selected

            choice = int(user_input)
            if 1 <= choice <= total:
                selected = candidates[choice - 1]
                print(f"[INFO] Selected [{choice}]: {selected.name}\n")
                return selected
            else:
                print(f"[WARN] Invalid number. Please enter a value between 1 and {total}.")
        except ValueError:
            print("[WARN] Invalid input. Please enter a number or press Enter for default.")
        except (KeyboardInterrupt, EOFError):
            print("\n[INFO] Operation cancelled by user.")
            sys.exit(0)
