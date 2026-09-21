import io
import json
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from analysis import log_selector
from mission.config import load_mission_config, load_run_context
from mission.run_bundle import create_run_bundle


MISSION_CONFIG = """
[target]
latitude = 38.260728
longitude = 140.854073

[radio]
control = "off"
pre_off_delay_sec = 3
restore_timeout_sec = 180
use_sudo = true
dry_run = false
"""

RUN_CONTEXT = """
[event]
id = "nse2026"
name = "NSE 2026"

[run]
kind = "field-test"
label = "grass"
location = "field-a"
operator = "team"
tags = ["camera"]
notes = "Wind from the west."
"""


class RunBundleTest(unittest.TestCase):
    def _configs(self, root: Path):
        mission_path = root / "mission.toml"
        context_path = root / "run-context.toml"
        mission_path.write_text(MISSION_CONFIG, encoding="utf-8")
        context_path.write_text(RUN_CONTEXT, encoding="utf-8")
        return load_mission_config(mission_path), load_run_context(context_path)

    def test_bundle_contains_manifest_snapshots_and_notes(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            mission_config, run_context = self._configs(root)
            with patch("mission.run_bundle._git_metadata", return_value={"commit": "abc", "dirty": False}):
                bundle = create_run_bundle(
                    mission_config,
                    run_context,
                    log_root=root / "runs",
                    now=datetime(2026, 8, 31, 12, 0, tzinfo=timezone.utc),
                )

            self.assertEqual(bundle.log_path.name, f"mission_{bundle.run_id}.csv")
            self.assertTrue(bundle.log_path.name.startswith("mission_20260831-120000-"))
            self.assertEqual(bundle.run_dir.parent, root / "runs")
            self.assertTrue((bundle.run_dir / "mission-config.toml").is_file())
            self.assertTrue((bundle.run_dir / "run-context.toml").is_file())
            self.assertEqual((bundle.run_dir / "notes.md").read_text(encoding="utf-8"), "Wind from the west.\n")

            manifest = json.loads(bundle.manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["schema"], "cansat.run.v1")
            self.assertEqual(manifest["artifacts"]["mission_log"], bundle.log_path.name)
            self.assertEqual(manifest["context"]["event_id"], "nse2026")
            self.assertEqual(manifest["software"]["commit"], "abc")

            roi_path = root / "roi_cone.png"
            roi_path.write_bytes(b"reference-image")
            bundle.snapshot_roi_inputs(
                [{"path": str(roi_path), "label": "positive", "weight": 1.0}]
            )
            manifest = json.loads(bundle.manifest_path.read_text(encoding="utf-8"))
            roi_record = manifest["roi_inputs"][0]
            self.assertEqual(roi_record["snapshot"], "inputs/roi/roi_cone.png")
            self.assertEqual(len(roi_record["sha256"]), 64)
            self.assertTrue((bundle.run_dir / roi_record["snapshot"]).is_file())

            bundle.finalize("GOAL_REACHED")
            manifest = json.loads(bundle.manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "finished")
            self.assertEqual(manifest["end_reason"], "GOAL_REACHED")

    def test_analysis_accepts_run_directory_and_writes_inside_bundle(self):
        for name in ("mission.csv", "mission_20260831-120000-abcd1234.csv"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp_dir:
                root = Path(tmp_dir)
                run_dir = root / "run"
                run_dir.mkdir()
                log_path = run_dir / name
                log_path.write_text("RunId\nrun-1\n", encoding="utf-8")
                (run_dir / "run-manifest.json").write_text("{}", encoding="utf-8")

                self.assertEqual(log_selector.resolve_log_path(run_dir), log_path.resolve())
                self.assertEqual(log_selector.resolve_log_path(log_path), log_path.resolve())
                out_dir = log_selector.create_analysis_output_dir(log_path, "log", root / "legacy")
                self.assertEqual(out_dir.parents[1], run_dir / "analysis")

    def test_analysis_discovers_old_and_timestamped_logs(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            paths = [
                root / "data" / "old" / "mission.csv",
                root / "data" / "new" / "mission_20260831-120000-abcd1234.csv",
                root / "legacy" / "robust_log_2026-0810-104332-810168.csv",
                root / "legacy" / "mission.csv",
                root / "legacy" / "mission_20260921-120000-abcd1234.csv",
            ]
            for path in paths:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("RunId\nrun-1\n", encoding="utf-8")
            (root / "legacy" / "mission_events.csv").write_text("", encoding="utf-8")
            with patch.object(log_selector, "DEFAULT_DATA_ROOT", root / "data"), patch.object(
                log_selector, "LEGACY_LOGS_DIR", root / "legacy"
            ):
                self.assertCountEqual(log_selector.get_robust_log_candidates(), paths)
                candidates = log_selector.get_robust_log_candidates()
                choice = str(candidates.index(paths[-1]) + 1)
                with patch.object(log_selector.sys.stdin, "isatty", return_value=True), patch(
                    "builtins.input", return_value=choice
                ), patch("sys.stdout", new_callable=io.StringIO) as output:
                    self.assertEqual(log_selector.resolve_log_path(), paths[-1])
                for path in paths:
                    self.assertIn(path.name, output.getvalue())


if __name__ == "__main__":
    unittest.main()
