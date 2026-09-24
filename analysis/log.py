from __future__ import annotations

import sys
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    PLOTLY_AVAILABLE = True
except Exception:
    PLOTLY_AVAILABLE = False


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lib.cone_diagnostics import CONE_DIAGNOSTIC_LOG_COLUMNS

try:
    from mission.const import LOG_HEADER
except Exception:
    LOG_HEADER = []

from analysis.final_approach import write_final_approach_report
from mission.diagnostics import FINAL_DIAGNOSTIC_DEFAULTS

from analysis.log_selector import (
    create_analysis_output_dir,
    find_latest_log,
    load_run_manifest,
    resolve_log_path,
)

COLUMN_GROUPS = [
    {"key": "recovery_and_radio", "title": "Sensor recovery / Radio",
     "cols": ["BMPStaleSec", "BMPUpdatedElapsedSec", "BMPValid",
              "BNOAccStaleSec", "BNOAccUpdatedElapsedSec", "BNOAccValid",
              "CameraFailCount", "CameraRecoveryElapsedSec", "CameraRecoveryExhausted",
              "CameraReinitAttemptCount", "RadioConfigSource", "RadioControlMode",
              "RadioDisabled", "RadioLastEvent", "RadioRestoreDeadlineElapsedSec"]},
    {"key": "final_approach_shutdown", "title": "Final approach / Shutdown",
     "cols": list(FINAL_DIAGNOSTIC_DEFAULTS)},
    {
        "key": "time_and_phase",
        "title": "Time & Mission Phase",
        "cols": [
            "LogSchemaVersion",
            "RunId",
            "ElapsedSec",
            "Phase",
            "MissionElapsedSec",
            "MissionEndReason",
            "Phase7ArrivalReason",
            "Phase0ExitReason",
            "Phase0ExitDetail",
            "MissionTotalTimeout",
        ],
    },
    {
        "key": "imu_accel",
        "title": "IMU - Acceleration",
        "cols": ["AccX", "AccY", "AccZ"],
    },
    {
        "key": "imu_gyro",
        "title": "IMU - Gyroscope",
        "cols": ["GyroX", "GyroY", "GyroZ"],
    },
    {
        "key": "imu_mag",
        "title": "IMU - Magnetometer",
        "cols": ["MagX", "MagY", "MagZ"],
    },
    {
        "key": "gps_position",
        "title": "GPS - Position & Quality",
        "cols": [
            "LAT",
            "LNG",
            "GpsSpeedMps",
            "GPSFixQual",
            "GPSSats",
            "GPSHdop",
            "GpsHeading",
            "GpsHeadingValid",
            "GPSFixSeq",
            "GPSHeadingBaselineM",
        ],
    },
    {
        "key": "phase0_altitude",
        "title": "Phase 0 - Current / Maximum Altitude & Descent [m]",
        "cols": ["Phase0CurrentAltitude", "Phase0MaxAltitude", "Phase0AltitudeDrop"],
    },
    {
        "key": "altitude_pressure",
        "title": "Altitude & Pressure",
        "cols": ["ALT", "Pres"],
    },
    {
        "key": "navigation",
        "title": "Navigation",
        "cols": [
            "Distance",
            "Azimuth",
            "TargetLat",
            "TargetLng",
            "Angle",
            "Direction",
            "AngleValid",
            "NavHeading",
            "NavHeadingSource",
            "HeadingDiff",
            "HeadingTrust",
            "BNOTrusted",
            "BNOOffsetDeg",
            "BNOOffsetValid",
            "BNOOffsetCandidateDeg",
            "BNOOffsetCandidateCount",
            "Phase2Stage",
            "BNOCalibSys",
            "BNOCalibGyro",
            "BNOCalibAcc",
            "BNOCalibMag",
            "Phase2OffsetRefDeg",
            "Phase2OffsetHeadingErrorDeg",
            "Phase2OffsetDistanceM",
            "Phase2OffsetPathEfficiency",
            "Phase2OffsetCourseDeg",
            "Phase2OffsetBNOMeanDeg",
            "Phase2OffsetBNOSpreadDeg",
            "Phase2OffsetSubsegmentDiffDeg",
            "Phase2OffsetAttemptCount",
            "Phase2OffsetMode",
            "Phase2OffsetTurnTargetDeg",
            "Phase2OffsetStageRetryCount",
            "Phase2OffsetNearGoalActive",
            "Phase2OffsetLegTimeLimitSec",
            "Phase2OffsetRejectReason",
            "Phase1OffsetCandidateValid",
            "Phase1OffsetCandidateDeg",
            "Phase1OffsetDistanceM",
            "Phase1OffsetPathEfficiency",
            "Phase1OffsetBNOSpreadDeg",
            "Phase1OffsetSubsegmentDiffDeg",
            "Phase1OffsetRejectReason",
            "ArrivalInside",
            "ArrivalConfirmCount",
            "Phase3ArrivedLatched",
        ],
    },
    {
        "key": "vision_detection",
        "title": "Vision / Detection",
        "cols": [
            *CONE_DIAGNOSTIC_LOG_COLUMNS,
            "ConeStaleSec",
            "ConeUpdatedElapsedSec",
            "ConePhaseDecision",
            "ConePhaseThreshold",
            "ConePhaseReachedProbabilityThreshold",
            "ConePhaseCenterTolerance",
            "ConePhaseDirectionTolerance",
            "ConePhaseRequiredConfirmFrames",
            "ConePhaseDetected",
            "ConePhaseReachedEffective",
            "ConePhaseCentered",
            "ConePhaseDirectionConsistent",
            "ConePhaseConfirmCount",
            "Phase4ConeConfirmCount",
            "Phase4ConeConfirmMarker",
            "Phase5ConeLostCount",
            "Phase5ReachConfirmCount",
            "Phase5EntryReason",
            "SonarDistanceCm",
            "ObstacleDist",  # Legacy logs
            "SonarSeq",
            "SonarObservedAt",
            "GoalDecision",
            "GoalConfirmCount",
            "SonarValid",
            "SonarStaleSec",
        ],
    },
    {
        "key": "fall_and_sensor_health",
        "title": "Fall & Sensor Health",
        "cols": [
            "Fall",
            "BNOStaleSec",
            "BNORecoveryActive",
            "BNORecoveryCount",
            "BNORecoverySeq",
        ],
    },
    {
        "key": "motor_commands",
        "title": "Motor Commands",
        "cols": [
            "MotorCmdType",
            "MotorCmdUpdatedElapsedSec",
            "Motor1CmdSpeed",
            "Motor1CmdForward",
            "Motor2CmdSpeed",
            "Motor2CmdForward",
        ],
    },
]

ANOMALY_RULES = {
    "Pres": {"min": 300.0, "max": 1200.0, "label": "BMP pressure out of range"},
    "ALT": {"min": -500.0, "max": 10000.0, "label": "BMP altitude out of range"},
    "BNOStaleSec": {"min": 0.0, "max": 1.0, "label": "BNO stale"},
    "GPSFixQual": {"min": 0.0, "max": 8.0, "label": "GPS fix quality invalid"},
    "GPSSats": {"min": 0.0, "max": 50.0, "label": "GPS satellites invalid"},
    "GPSHdop": {"min": 0.0, "max": 10.0, "label": "GPS HDOP high"},
    "GpsSpeedMps": {"min": 0.0, "max": 40.0, "label": "GPS speed suspicious"},
}

# Heavy per-row label generation is disabled by default for PC responsiveness on large logs.
ANOMALY_LIGHTWEIGHT = True
ANOMALY_HIGHLIGHT_MAX_RATE = 0.80


def write_analysis_context(out_dir: Path, *, log_path: Path) -> None:
    manifest = load_run_manifest(log_path)
    lines = [
        "CanSat Analysis Context",
        f"Source log: {log_path}",
    ]
    if manifest:
        lines.extend(
            [
                f"Run ID: {manifest.get('run_id', '')}",
                f"Event: {manifest.get('context', {}).get('event_id', '')}",
                f"Run kind: {manifest.get('context', {}).get('run_kind', '')}",
                f"Git commit: {manifest.get('software', {}).get('commit', '')}",
            ]
        )
    (out_dir / "analysis_context.txt").write_text("\n".join(lines), encoding="utf-8")


# Kept for backward compatibility, delegated to log_selector.
# (find_latest_log is imported from analysis.log_selector above)


def prepare_output_dir(log_path: Path) -> Path:
    return create_analysis_output_dir(log_path, "log", REPO_ROOT / "analysis" / "outputs")


def _safe_numeric_series(df: pd.DataFrame, col: str) -> pd.Series:
    return pd.to_numeric(df[col], errors="coerce")


def _phase0_mask(df: pd.DataFrame) -> pd.Series:
    if "Phase" not in df.columns:
        return pd.Series(False, index=df.index)
    phase_num = pd.to_numeric(df["Phase"], errors="coerce")
    if phase_num.notna().any():
        return (phase_num == 0).fillna(False)
    return df["Phase"].astype(str).str.contains("0", na=False)


def detect_anomalies(df: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    if df.empty:
        result = df.copy()
        result["AnomalyCount"] = 0
        result["AnomalyLabels"] = ""
        return result

    result = df.copy()
    n = len(result)
    anomaly_count = np.zeros(n, dtype=np.int16)
    highlight_count = np.zeros(n, dtype=np.int16)
    missing_count = np.zeros(n, dtype=np.int16)
    summary_records = []
    missing_summary_records = []
    phase0_mask = _phase0_mask(result)
    active_mask = (~phase0_mask).fillna(True)
    active_np = active_mask.to_numpy(dtype=bool, copy=False)

    # Cache numeric conversions once to avoid repeated to_numeric() calls on large logs.
    numeric_cache: dict[str, pd.Series] = {}

    def num(col: str) -> pd.Series:
        s = numeric_cache.get(col)
        if s is None:
            s = _safe_numeric_series(result, col)
            numeric_cache[col] = s
        return s

    # Treat zero pressure as "missing/unavailable" instead of anomaly for this project logs.
    pressure_zero_mask = pd.Series(False, index=result.index)
    if "Pres" in result.columns:
        pres = num("Pres")
        pressure_zero_mask = (pres == 0).fillna(False)
        if pressure_zero_mask.any():
            missing_count += pressure_zero_mask.to_numpy(dtype=np.int16, na_value=0)
            missing_summary_records.append(
                {
                    "missing_type": "BMP pressure missing (Pres==0)",
                    "column": "Pres",
                    "rows_flagged": int(pressure_zero_mask.sum()),
                    "phase0_rows": int((pressure_zero_mask & phase0_mask).sum()),
                    "non_phase0_rows": int((pressure_zero_mask & active_mask).sum()),
                }
            )

    for col, rule in ANOMALY_RULES.items():
        if col not in result.columns:
            continue
        s = num(col)
        valid_mask = s.notna()
        if col == "Pres":
            valid_mask = valid_mask & (~pressure_zero_mask)
        bad = (valid_mask & ((s < rule["min"]) | (s > rule["max"]))).fillna(False)
        bad = (bad & active_mask).fillna(False)  # Weaken detection during Phase0 initialization.
        if bad.any():
            bad_np = bad.to_numpy(dtype=np.int16, na_value=0)
            anomaly_count += bad_np
            rate = float(bad_np.mean())
            used_for_highlight = rate <= ANOMALY_HIGHLIGHT_MAX_RATE
            if used_for_highlight:
                highlight_count += bad_np
            summary_records.append(
                {
                    "rule": rule["label"],
                    "column": col,
                    "rows_flagged": int(bad.sum()),
                    "flag_rate": round(rate, 6),
                    "used_for_highlight": used_for_highlight,
                    "min_threshold": rule["min"],
                    "max_threshold": rule["max"],
                }
            )

    if {"GPSFixQual", "LAT", "LNG"}.issubset(result.columns):
        fix = num("GPSFixQual")
        lat = num("LAT")
        lng = num("LNG")
        bad = ((fix > 0) & ((lat == 0) | (lng == 0))).fillna(False)
        bad = (bad & active_mask).fillna(False)
        if bad.any():
            bad_np = bad.to_numpy(dtype=np.int16, na_value=0)
            anomaly_count += bad_np
            rate = float(bad_np.mean())
            used_for_highlight = rate <= ANOMALY_HIGHLIGHT_MAX_RATE
            if used_for_highlight:
                highlight_count += bad_np
            summary_records.append(
                {
                    "rule": "GPS fix reported but LAT/LNG is zero",
                    "column": "LAT/LNG",
                    "rows_flagged": int(bad.sum()),
                    "flag_rate": round(rate, 6),
                    "used_for_highlight": used_for_highlight,
                    "min_threshold": None,
                    "max_threshold": None,
                }
            )

    if "GPSSats" in result.columns:
        sats = num("GPSSats")
        bad = (sats.notna() & (sats < 4)).fillna(False)
        bad = (bad & active_mask).fillna(False)
        if bad.any():
            bad_np = bad.to_numpy(dtype=np.int16, na_value=0)
            anomaly_count += bad_np
            rate = float(bad_np.mean())
            used_for_highlight = rate <= ANOMALY_HIGHLIGHT_MAX_RATE
            if used_for_highlight:
                highlight_count += bad_np
            summary_records.append(
                {
                    "rule": "Low GPS satellites",
                    "column": "GPSSats",
                    "rows_flagged": int(bad.sum()),
                    "flag_rate": round(rate, 6),
                    "used_for_highlight": used_for_highlight,
                    "min_threshold": 4,
                    "max_threshold": None,
                }
            )

    imu_triplets = [
        ("AccX", "AccY", "AccZ", "BNO accel missing"),
        ("GyroX", "GyroY", "GyroZ", "BNO gyro missing"),
        ("MagX", "MagY", "MagZ", "BNO mag missing"),
    ]
    for c1, c2, c3, label in imu_triplets:
        if {c1, c2, c3}.issubset(result.columns):
            s1 = num(c1)
            s2 = num(c2)
            s3 = num(c3)
            bad_np = (s1.isna().to_numpy() & s2.isna().to_numpy() & s3.isna().to_numpy() & active_np).astype(np.int16)
            if bad_np.any():
                anomaly_count += bad_np
                rate = float(bad_np.mean())
                used_for_highlight = rate <= ANOMALY_HIGHLIGHT_MAX_RATE
                if used_for_highlight:
                    highlight_count += bad_np
                summary_records.append(
                    {
                        "rule": label,
                        "column": f"{c1},{c2},{c3}",
                        "rows_flagged": int(bad_np.sum()),
                        "flag_rate": round(rate, 6),
                        "used_for_highlight": used_for_highlight,
                        "min_threshold": None,
                        "max_threshold": None,
                    }
                )

    result["AnomalyCount"] = anomaly_count.astype(np.int32)
    result["HighlightAnomalyCount"] = highlight_count.astype(np.int32)
    result["MissingCount"] = missing_count.astype(np.int32)
    result["HasAnomaly"] = result["AnomalyCount"] > 0
    result["HasHighlightAnomaly"] = result["HighlightAnomalyCount"] > 0
    result["IsPhase0"] = phase0_mask

    if not ANOMALY_LIGHTWEIGHT:
        result["AnomalyLabels"] = ""

    anomaly_rows = result[result["HasAnomaly"]].copy()
    if not anomaly_rows.empty:
        cols = [c for c in ["ElapsedSec", "Phase", "AnomalyCount", "HighlightAnomalyCount", "MissingCount"] if c in anomaly_rows.columns]
        if "AnomalyLabels" in anomaly_rows.columns:
            cols.append("AnomalyLabels")
        anomaly_rows[cols].to_csv(out_dir / "anomaly_events.csv", index=False)

    if summary_records:
        pd.DataFrame(summary_records).sort_values(["rows_flagged", "rule"], ascending=[False, True]).to_csv(
            out_dir / "anomaly_summary.csv", index=False
        )
    if missing_summary_records:
        pd.DataFrame(missing_summary_records).sort_values(["rows_flagged", "missing_type"], ascending=[False, True]).to_csv(
            out_dir / "missing_summary.csv", index=False
        )

    return result


def write_group_csvs(df: pd.DataFrame, out_dir: Path) -> None:
    for group in COLUMN_GROUPS:
        present_cols = [c for c in group["cols"] if c in df.columns]
        if not present_cols:
            continue
        export_cols = present_cols if "ElapsedSec" in present_cols else (["ElapsedSec"] + present_cols if "ElapsedSec" in df.columns else present_cols)
        df[export_cols].to_csv(out_dir / f"{group['key']}.csv", index=False)


def write_vision_diagnostics(df: pd.DataFrame, out_dir: Path) -> None:
    """Export explainable vision gate failures while accepting legacy log schemas."""
    required_any = {
        "ConeIsReached",
        "ConeCloseReachedOK",
        "ConeIsDetected",
        "ConeProbabilityDetected",
        "ConeValid",
        "ConePenaltyFlags",
    }
    if not required_any.intersection(df.columns):
        return

    numeric_cache = {}

    def num(column, default=0.0):
        if column not in df.columns:
            return pd.Series(default, index=df.index, dtype=float)
        if column not in numeric_cache:
            numeric_cache[column] = pd.to_numeric(df[column], errors="coerce").fillna(default)
        return numeric_cache[column]

    phase = num("Phase", -1)
    prob = num("ConeProb")
    masks = [
        (
            "close_track_continuation",
            (num("ConeCloseTrackEligible") > 0) & phase.isin([4, 5, 6]),
        ),
        (
            "close_track_hold_unconfirmed",
            (num("ConeCloseTrackHold") > 0) & (num("ConeCloseTrackEligible") <= 0)
            & phase.isin([4, 5, 6]),
        ),
        (
            "reached_rejected_by_phase_gate",
            (num("ConeIsReached") > 0) & (num("ConePhaseReachedEffective") <= 0) & phase.isin([4, 5]),
        ),
        (
            "close_reached_probability_demoted",
            (num("ConeCloseReachedOK") > 0) & (prob < 0.28),
        ),
        (
            "detector_flag_probability_mismatch",
            num("ConeIsDetected").round() != num("ConeProbabilityDetected").round(),
        ),
        (
            "invalid_camera_observation",
            (num("ConeValid") <= 0) & phase.isin([4, 5]),
        ),
        (
            "roi_histogram_unavailable",
            (num("ConeValid") > 0) & (num("ConeROIHistAvailable") <= 0) & phase.isin([4, 5]),
        ),
    ]

    context_cols = [
        col
        for col in [
            "ElapsedSec",
            "Phase",
            "ConeSeq",
            "ConeValid",
            "ConeStatus",
            "ConeProb",
            "ConeRawProb",
            "ConeCandidateProb",
            "ConePreFilterProb",
            "ConeIsDetected",
            "ConeProbabilityDetected",
            "ConeIsReached",
            "ConeRawReached",
            "ConeCloseReachedOK",
            "ConeROISupport",
            "ConeStrictRedOK",
            "ConeStrictRedRejectReason",
            "ConeCloseReachedRejectReason",
            "ConeCloseTrackReason",
            "ConeCloseTrackCount",
            "ConeCloseTrackEligible",
            "ConeCloseTrackHold",
            "ConePenaltyFlags",
            "ConePhaseDecision",
            "ConePhaseReachedEffective",
            "ConePhaseConfirmCount",
        ]
        if col in df.columns
    ]
    event_frames = []
    summary_rows = []
    for event_type, mask in masks:
        mask = mask.fillna(False)
        count = int(mask.sum())
        summary_rows.append({"event_type": event_type, "rows": count})
        if count:
            events = df.loc[mask, context_cols].copy()
            events.insert(0, "event_type", event_type)
            event_frames.append(events)

    if event_frames:
        pd.concat(event_frames, ignore_index=True).to_csv(
            out_dir / "vision_gate_events.csv", index=False
        )
    pd.DataFrame(summary_rows).to_csv(out_dir / "vision_gate_summary.csv", index=False)

    if "ConePenaltyFlags" in df.columns:
        penalties = []
        for value in df["ConePenaltyFlags"].dropna().astype(str):
            for flag in value.split("|"):
                flag = flag.strip()
                if flag:
                    penalties.append(flag)
        if penalties:
            counts = pd.Series(penalties).value_counts().rename_axis("penalty_flag").reset_index(name="rows")
            counts.to_csv(out_dir / "vision_penalty_summary.csv", index=False)


def write_coverage_reports(df: pd.DataFrame, out_dir: Path) -> dict:
    grouped_cols = [c for g in COLUMN_GROUPS for c in g["cols"]]
    grouped_set = set(grouped_cols)
    expected_set = set(LOG_HEADER) if LOG_HEADER else set(df.columns)
    actual_set = set(df.columns)
    legacy_optional_missing = set()
    if "LogSchemaVersion" not in actual_set:
        legacy_optional_missing = {"LogSchemaVersion", "RunId"} & expected_set

    versions = pd.to_numeric(df.get("LogSchemaVersion", pd.Series(dtype=float)),
                             errors="coerce").dropna()
    if versions.empty or versions.max() < 4:
        legacy_optional_missing.update(set(FINAL_DIAGNOSTIC_DEFAULTS) - actual_set)
    elif versions.max() < 6:
        legacy_optional_missing.update({c for c in FINAL_DIAGNOSTIC_DEFAULTS
                                        if c.startswith('Recovery')} - actual_set)
    if versions.empty or versions.max() < 5:
        legacy_optional_missing.update({'Phase0CurrentAltitude', 'Phase0MaxAltitude',
                                        'Phase0AltitudeDrop'} - actual_set)

    duplicate_group_cols = sorted({c for c in grouped_cols if grouped_cols.count(c) > 1})
    expected_missing_in_groups = sorted(expected_set - grouped_set)
    grouped_not_in_expected = sorted(grouped_set - expected_set - {"ObstacleDist"}) if LOG_HEADER else []
    actual_missing_from_csv = sorted(expected_set - actual_set - legacy_optional_missing)
    actual_extra_in_csv = sorted(actual_set - expected_set) if LOG_HEADER else []

    lines = [
        f"Source CSV: {df.attrs.get('source_path', '')}",
        f"Row count: {len(df)}",
        f"Column count: {len(df.columns)}",
        "",
        "[Coverage vs LOG_HEADER]",
        f"Duplicate columns across groups: {duplicate_group_cols or 'None'}",
        f"LOG_HEADER columns missing from groups: {expected_missing_in_groups or 'None'}",
        f"Grouped columns not in LOG_HEADER: {grouped_not_in_expected or 'None'}",
        f"Legacy-compatible context columns absent: {sorted(legacy_optional_missing) or 'None'}",
        f"LOG_HEADER columns missing from CSV: {actual_missing_from_csv or 'None'}",
        f"CSV columns not in LOG_HEADER: {actual_extra_in_csv or 'None'}",
        "",
        "[Group Definitions]",
    ]
    for g in COLUMN_GROUPS:
        lines.append(f"- {g['key']}: {', '.join(g['cols'])}")

    (out_dir / "coverage_report.txt").write_text("\n".join(lines), encoding="utf-8")

    summary_rows = []
    for g in COLUMN_GROUPS:
        for c in g["cols"]:
            summary_rows.append(
                {
                    "group_key": g["key"],
                    "group_title": g["title"],
                    "column": c,
                    "in_csv": c in df.columns,
                    "in_log_header": (c in LOG_HEADER) if LOG_HEADER else None,
                }
            )
    pd.DataFrame(summary_rows).to_csv(out_dir / "column_group_mapping.csv", index=False)

    return {
        "duplicate_group_cols": duplicate_group_cols,
        "expected_missing_in_groups": expected_missing_in_groups,
        "grouped_not_in_expected": grouped_not_in_expected,
        "actual_missing_from_csv": actual_missing_from_csv,
        "actual_extra_in_csv": actual_extra_in_csv,
    }


def write_basic_summaries(df: pd.DataFrame, out_dir: Path) -> None:
    df.describe(include="all").transpose().to_csv(out_dir / "summary_describe_all.csv")

    categorical_rows = []
    for col in df.columns:
        if col == "ElapsedSec":
            continue
        numeric = pd.to_numeric(df[col], errors="coerce")
        if numeric.notna().sum() > 0 and numeric.notna().sum() >= max(1, int(len(df) * 0.8)):
            continue
        counts = df[col].astype(str).value_counts(dropna=False).head(20)
        for value, count in counts.items():
            categorical_rows.append({"column": col, "value": value, "count": int(count)})
    if categorical_rows:
        pd.DataFrame(categorical_rows).to_csv(out_dir / "categorical_value_counts_top20.csv", index=False)


def plot_integrated_timeseries(df: pd.DataFrame, out_dir: Path) -> None:
    if "ElapsedSec" not in df.columns:
        return

    t = _safe_numeric_series(df, "ElapsedSec")
    plot_groups = []
    for group in COLUMN_GROUPS:
        numeric_cols = []
        for col in group["cols"]:
            if col == "ElapsedSec" or col not in df.columns:
                continue
            s = _safe_numeric_series(df, col)
            if s.notna().any():
                numeric_cols.append(col)
        if numeric_cols:
            plot_groups.append((group["title"], numeric_cols))

    if not plot_groups:
        return

    fig, axes = plt.subplots(len(plot_groups), 1, figsize=(14, max(4, 2.8 * len(plot_groups))), sharex=True)
    if len(plot_groups) == 1:
        axes = [axes]

    for ax, (title, cols) in zip(axes, plot_groups):
        for col in cols:
            ax.plot(t, _safe_numeric_series(df, col), label=col, alpha=0.85, linewidth=1.0)
        ax.set_title(title, fontweight="bold")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="upper right", fontsize="x-small", ncol=2)

    axes[-1].set_xlabel("Elapsed Seconds [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "integrated_sensor_log.png", dpi=150)
    plt.close(fig)


def plot_anomaly_overview(df: pd.DataFrame, out_dir: Path) -> None:
    if "ElapsedSec" not in df.columns or "AnomalyCount" not in df.columns:
        return
    t = _safe_numeric_series(df, "ElapsedSec")
    y = _safe_numeric_series(df, "AnomalyCount")

    fig, ax = plt.subplots(figsize=(14, 3.5))
    ax.plot(t, y, color="crimson", linewidth=1.2, label="AnomalyCount")
    mask = y.fillna(0) > 0
    if mask.any():
        ax.scatter(t[mask], y[mask], color="red", s=10, alpha=0.8, label="Detected anomaly")
    ax.set_title("Detected Anomalies Timeline", fontweight="bold")
    ax.set_xlabel("Elapsed Seconds [s]")
    ax.set_ylabel("Count")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper right", fontsize="small")
    fig.tight_layout()
    fig.savefig(out_dir / "anomaly_timeline.png", dpi=150)
    plt.close(fig)


def plot_interactive_html(df: pd.DataFrame, out_dir: Path) -> None:
    if not PLOTLY_AVAILABLE or "ElapsedSec" not in df.columns:
        return

    t = _safe_numeric_series(df, "ElapsedSec")
    plot_groups = []
    for group in COLUMN_GROUPS:
        cols = []
        for col in group["cols"]:
            if col == "ElapsedSec" or col not in df.columns:
                continue
            s = _safe_numeric_series(df, col)
            if s.notna().any():
                cols.append(col)
        if cols:
            plot_groups.append((group["title"], cols))

    if not plot_groups:
        return

    fig = make_subplots(
        rows=len(plot_groups),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.02,
        specs=[[{"secondary_y": True}]] + [[{"secondary_y": False}] for _ in range(len(plot_groups) - 1)],
        subplot_titles=[title for title, _ in plot_groups],
    )

    for row_i, (group_title, cols) in enumerate(plot_groups, start=1):
        for col in cols:
            use_secondary_y = row_i == 1 and col == "Phase"
            fig.add_trace(
                go.Scattergl(
                    x=t,
                    y=_safe_numeric_series(df, col),
                    mode="lines",
                    name=col,
                    legendgroup=col,
                    showlegend=(row_i == 1),
                    line={"width": 1},
                    hovertemplate="t=%{x:.3f}s<br>%{y}<extra>" + col + "</extra>",
                ),
                row=row_i,
                col=1,
                secondary_y=use_secondary_y,
            )

    if plot_groups:
        first_title, first_cols = plot_groups[0]
        if first_title == "Time & Mission Phase" and "Phase" in first_cols:
            fig.update_yaxes(title_text="Phase", row=1, col=1, secondary_y=True, tickmode="linear", dtick=1)
            fig.update_yaxes(title_text="Elapsed/Mission Values", row=1, col=1, secondary_y=False)

    fig.update_layout(
        height=max(500, 260 * len(plot_groups)),
        width=1300,
        title_text="CanSat Integrated Sensor Log (Interactive)",
        hovermode="x unified",
    )
    fig.update_xaxes(title_text="Elapsed Seconds [s]", row=len(plot_groups), col=1)
    fig.write_html(out_dir / "integrated_sensor_log_interactive.html", include_plotlyjs="cdn")

    if {"LAT", "LNG"}.issubset(df.columns):
        lat = _safe_numeric_series(df, "LAT")
        lng = _safe_numeric_series(df, "LNG")
        valid = lat.notna() & lng.notna() & (lat != 0) & (lng != 0)
        if valid.any():
            map_fig = go.Figure()
            map_fig.add_trace(
                go.Scatter(
                    x=lng[valid],
                    y=lat[valid],
                    mode="lines+markers",
                    marker={"size": 4},
                    name="Trajectory",
                    hovertemplate="LNG=%{x}<br>LAT=%{y}<extra></extra>",
                )
            )
            map_fig.update_layout(
                title="CanSat Trajectory (Interactive)",
                xaxis_title="Longitude",
                yaxis_title="Latitude",
                hovermode="closest",
            )
            map_fig.write_html(out_dir / "trajectory_map_interactive.html", include_plotlyjs="cdn")


def plot_trajectory(df: pd.DataFrame, out_dir: Path) -> None:
    if not {"LAT", "LNG"}.issubset(df.columns):
        return

    lat = _safe_numeric_series(df, "LAT")
    lng = _safe_numeric_series(df, "LNG")
    valid = lat.notna() & lng.notna() & (lat != 0) & (lng != 0)
    if not valid.any():
        return

    lat_v = lat[valid]
    lng_v = lng[valid]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(lng_v, lat_v, color="blue", label="Actual Path")
    ax.scatter(lng_v.iloc[0], lat_v.iloc[0], color="green", label="Start")
    ax.scatter(lng_v.iloc[-1], lat_v.iloc[-1], color="orange", label="End")

    if {"TargetLat", "TargetLng"}.issubset(df.columns):
        tgt_lat = _safe_numeric_series(df, "TargetLat").dropna()
        tgt_lng = _safe_numeric_series(df, "TargetLng").dropna()
        if not tgt_lat.empty and not tgt_lng.empty:
            ax.scatter(tgt_lng.iloc[0], tgt_lat.iloc[0], color="red", marker="*", s=180, label="Target")

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("CanSat Trajectory Map")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory_map.png", dpi=150)
    plt.close(fig)


def analyze_cansat_log(file_path: str | Path | None = None) -> Path:
    log_path = resolve_log_path(file_path)

    df = pd.read_csv(log_path)
    df.attrs["source_path"] = str(log_path)
    out_dir = prepare_output_dir(log_path)
    write_analysis_context(out_dir, log_path=log_path)
    write_final_approach_report(log_path, out_dir)

    coverage = write_coverage_reports(df, out_dir)
    df = detect_anomalies(df, out_dir)
    write_group_csvs(df, out_dir)
    write_vision_diagnostics(df, out_dir)
    write_basic_summaries(df, out_dir)
    plot_integrated_timeseries(df, out_dir)
    plot_trajectory(df, out_dir)
    plot_anomaly_overview(df, out_dir)
    plot_interactive_html(df, out_dir)

    print(f"[INFO] Source log: {log_path}")
    print(f"[INFO] Output dir : {out_dir}")
    print(f"[INFO] Rows/Cols   : {len(df)} / {len(df.columns)}")
    if "HasAnomaly" in df.columns:
        print(f"[INFO] Anomalies   : {int(df['HasAnomaly'].sum())} rows flagged")
    print(f"[INFO] Plotly HTML : {'enabled' if PLOTLY_AVAILABLE else 'skipped (plotly not installed)'}")
    if any(coverage.values()):
        print("[WARN] Column coverage mismatch detected. See coverage_report.txt")
    else:
        print("[INFO] Column coverage OK (vs LOG_HEADER and group definitions).")

    return out_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze CanSat robust mission log.")
    parser.add_argument(
        "log_path",
        nargs="?",
        help="Path to a run directory, mission_*.csv, mission.csv, or legacy robust_log_*.csv.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    analyze_cansat_log(args.log_path)
