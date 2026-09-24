from __future__ import annotations

import argparse
import math
import re
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path

np = None
pd = None

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except Exception:
    plt = None
    MATPLOTLIB_AVAILABLE = False

try:
    import plotly.graph_objects as go

    PLOTLY_AVAILABLE = True
except Exception:
    PLOTLY_AVAILABLE = False


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from mission.const import GPS_MAX_HDOP, GPS_MIN_FIX_QUAL, GPS_MIN_SATELLITES, LOG_PREFIX
except Exception:
    GPS_MAX_HDOP = 5.0
    GPS_MIN_FIX_QUAL = 1
    GPS_MIN_SATELLITES = 4
    LOG_PREFIX = "robust_log_"

from analysis.final_approach import write_final_approach_report
from mission.diagnostics import FINAL_DIAGNOSTIC_DEFAULTS

from analysis.log_selector import (
    create_analysis_output_dir,
    find_latest_log,
    load_run_manifest,
    resolve_log_path,
)


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".webp"}

@dataclass(frozen=True)
class CameraFrame:
    path: Path
    captured_at: datetime


def _safe_numeric_series(df: pd.DataFrame, col: str) -> pd.Series:
    if col not in df.columns:
        return pd.Series(np.nan, index=df.index, dtype=float)
    return pd.to_numeric(df[col], errors="coerce")


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


def ensure_runtime_dependencies() -> None:
    global np, pd
    if np is None:
        try:
            import numpy as _np
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                "numpy is required for analysis/explorer.py. Install PC-side analysis deps first."
            ) from exc

        np = _np
    if pd is None:
        try:
            import pandas as _pd
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                "pandas is required for analysis/explorer.py. Install PC-side analysis deps first."
            ) from exc

        pd = _pd


# Kept for backward compatibility, delegated to log_selector.
# (find_latest_log is imported from analysis.log_selector above)


def prepare_output_dir(log_path: Path) -> Path:
    return create_analysis_output_dir(
        log_path,
        "explorer",
        REPO_ROOT / "analysis" / "explorer_outputs",
    )


def parse_log_start_time(log_path: Path) -> datetime | None:
    manifest = load_run_manifest(log_path)
    started_at = manifest.get("started_at")
    if isinstance(started_at, str):
        try:
            return datetime.fromisoformat(started_at)
        except ValueError:
            pass
    stem = log_path.stem
    pattern = re.escape(LOG_PREFIX) + r"(?P<dt>\d{4}-\d{4}-\d{6})"
    match = re.search(pattern, stem)
    if not match:
        return None
    try:
        return datetime.strptime(match.group("dt"), "%Y-%m%d-%H%M%S")
    except ValueError:
        return None


def build_mission_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    result = df.copy()
    if "SonarDistanceCm" not in result and "ObstacleDist" in result:
        result["SonarDistanceCm"] = result["ObstacleDist"]
    has_sonar_valid = "SonarValid" in result.columns
    has_cone_diagnostics = "ConeDiagSchemaVersion" in result.columns
    numeric_cols = [
        "ElapsedSec",
        "Phase",
        "LAT",
        "LNG",
        "ALT",
        "Pres",
        "GpsSpeedMps",
        "GPSFixQual",
        "GPSSats",
        "GPSHdop",
        "SonarDistanceCm",
        "SonarValid",
        "SonarStaleSec",
        "Angle",
        "AngleValid",
        "Direction",
        "Distance",
        "Azimuth",
        "ConeDir",
        "ConeProb",
        "ConeValid",
        "ConeSeq",
        "ConeStaleSec",
        "ConeIsDetected",
        "ConeProbabilityDetected",
        "ConeIsReached",
        "ConeRawReached",
        "ConeCloseReachedOK",
        "ConeCloseTrackCount",
        "ConeCloseTrackEligible",
        "ConeCloseTrackHold",
        "ConeRawProb",
        "ConeCandidateProb",
        "ConePreFilterProb",
        "ConeOccupancy",
        "ConeFrameRedOccupancy",
        "ConeROISupport",
        "ConeROIHistAvailable",
        "ConeStrictRedOK",
        "ConePhaseReachedEffective",
        "ConePhaseConfirmCount",
        "ConePhaseReachedProbabilityThreshold",
        "ConePhaseCenterTolerance",
        "ConePhaseDirectionTolerance",
        "ConePhaseRequiredConfirmFrames",
        "MissionElapsedSec",
        "TargetLat",
        "TargetLng",
    ]
    for col in numeric_cols:
        result[col] = _safe_numeric_series(result, col)
    for col in (
        "ConeMethod",
        "ConeStatus",
        "ConePenaltyFlags",
        "ConeStrictRedRejectReason",
        "ConeCloseReachedRejectReason",
        "ConeCloseTrackReason",
        "ConePhaseDecision",
    ):
        if col not in result.columns:
            result[col] = ""
    result["sonar_valid"] = (
        result["SonarValid"].fillna(0.0) > 0.0
        if has_sonar_valid
        else pd.Series(True, index=result.index)
    )

    result["Phase"] = result["Phase"].round().astype("Int64")
    result["gps_valid"] = (
        result["LAT"].notna()
        & result["LNG"].notna()
        & (result["LAT"] != 0)
        & (result["LNG"] != 0)
        & (result["GPSFixQual"].fillna(0) >= GPS_MIN_FIX_QUAL)
        & (result["GPSSats"].fillna(0) >= GPS_MIN_SATELLITES)
        & (result["GPSHdop"].fillna(np.inf) <= GPS_MAX_HDOP)
    )

    gps_points = result[result["gps_valid"]].copy()
    if gps_points.empty:
        raise ValueError("No valid GPS rows found. LAT/LNG or GPS quality columns are not usable.")

    origin_lat = float(gps_points["LAT"].iloc[0])
    origin_lng = float(gps_points["LNG"].iloc[0])
    origin_alt = float(gps_points["ALT"].dropna().iloc[0]) if gps_points["ALT"].notna().any() else 0.0
    cos_lat = math.cos(math.radians(origin_lat))
    meters_per_deg_lat = 111320.0
    meters_per_deg_lng = meters_per_deg_lat * max(cos_lat, 1.0e-6)

    result["x_m"] = (result["LNG"] - origin_lng) * meters_per_deg_lng
    result["y_m"] = (result["LAT"] - origin_lat) * meters_per_deg_lat
    result["z_m"] = result["ALT"] - origin_alt

    dx = result["x_m"].diff()
    dy = result["y_m"].diff()
    step_m = np.hypot(dx, dy)
    result["step_m"] = step_m.fillna(0.0)
    result["track_distance_m"] = step_m.fillna(0.0).cumsum()

    track_heading = (np.degrees(np.arctan2(dx, dy)) + 360.0) % 360.0
    angle_heading = result["Angle"].where(result["AngleValid"].fillna(0) > 0)
    result["heading_deg"] = angle_heading.combine_first(track_heading).combine_first(result["Direction"])

    target_lat = result["TargetLat"].dropna()
    target_lng = result["TargetLng"].dropna()
    if target_lat.empty or target_lng.empty or float(target_lat.iloc[0]) == 0.0 or float(target_lng.iloc[0]) == 0.0:
        tgt_lat = float("nan")
        tgt_lng = float("nan")
    else:
        tgt_lat = float(target_lat.iloc[0])
        tgt_lng = float(target_lng.iloc[0])

    result.attrs["origin_lat"] = origin_lat
    result.attrs["origin_lng"] = origin_lng
    result.attrs["origin_alt"] = origin_alt
    result.attrs["target_lat"] = tgt_lat
    result.attrs["target_lng"] = tgt_lng
    result.attrs["target_x_m"] = (tgt_lng - origin_lng) * meters_per_deg_lng
    result.attrs["target_y_m"] = (tgt_lat - origin_lat) * meters_per_deg_lat
    result.attrs["cone_diagnostics_available"] = has_cone_diagnostics
    return result


def summarize_mission(df: pd.DataFrame) -> tuple[pd.DataFrame, dict[str, float | int | str]]:
    phase_rows = []
    phase_series = df["Phase"].dropna().astype(int)
    elapsed = df["ElapsedSec"].ffill().fillna(0.0)

    for phase, group in df.groupby("Phase", dropna=True):
        idx = group.index
        phase_rows.append(
            {
                "phase": int(phase),
                "rows": int(len(group)),
                "start_sec": float(elapsed.loc[idx].min()),
                "end_sec": float(elapsed.loc[idx].max()),
                "duration_sec": float(max(0.0, elapsed.loc[idx].max() - elapsed.loc[idx].min())),
                "distance_m": float(group["step_m"].fillna(0.0).sum()),
                "max_alt_rel_m": float(group["z_m"].max()) if group["z_m"].notna().any() else np.nan,
                "min_alt_rel_m": float(group["z_m"].min()) if group["z_m"].notna().any() else np.nan,
            }
        )

    mission_elapsed = float(elapsed.max()) if elapsed.notna().any() else 0.0
    gps_valid_count = int(df["gps_valid"].sum())
    summary = {
        "rows": int(len(df)),
        "mission_elapsed_sec": mission_elapsed,
        "gps_valid_rows": gps_valid_count,
        "gps_valid_ratio": float(gps_valid_count / len(df)) if len(df) else 0.0,
        "traveled_distance_m": float(df["step_m"].fillna(0.0).sum()),
        "altitude_min_rel_m": float(df["z_m"].min()) if df["z_m"].notna().any() else 0.0,
        "altitude_max_rel_m": float(df["z_m"].max()) if df["z_m"].notna().any() else 0.0,
        "phase_start": int(phase_series.iloc[0]) if not phase_series.empty else -1,
        "phase_end": int(phase_series.iloc[-1]) if not phase_series.empty else -1,
        "max_speed_mps": float(df["GpsSpeedMps"].max()) if df["GpsSpeedMps"].notna().any() else 0.0,
        "best_cone_prob": float(df["ConeProb"].max()) if df["ConeProb"].notna().any() else 0.0,
        "cone_reached_rows": int((df["ConeIsReached"].fillna(0.0) > 0.0).sum()),
        "cone_close_reached_rows": int((df["ConeCloseReachedOK"].fillna(0.0) > 0.0).sum()),
        "cone_diagnostics_available": bool(df.attrs.get("cone_diagnostics_available", False)),
        "cone_valid_ratio": (
            float((df["ConeValid"].fillna(0.0) > 0.0).mean())
            if len(df) and df.attrs.get("cone_diagnostics_available", False)
            else np.nan
        ),
        "closest_obstacle_cm": float(df.loc[df["sonar_valid"], "SonarDistanceCm"].replace(0, np.nan).min())
        if df.loc[df["sonar_valid"], "SonarDistanceCm"].replace(0, np.nan).notna().any()
        else np.nan,
        "mission_end_reason": str(df["MissionEndReason"].dropna().iloc[-1]) if "MissionEndReason" in df.columns and df["MissionEndReason"].dropna().any() else "",
    }
    return pd.DataFrame(phase_rows).sort_values("phase"), summary


def build_obstacle_map(df: pd.DataFrame, obstacle_max_cm: float) -> pd.DataFrame:
    obs = df[
        df["gps_valid"]
        & df["sonar_valid"]
        & df["heading_deg"].notna()
        & df["SonarDistanceCm"].notna()
        & (df["SonarDistanceCm"] > 0)
        & (df["SonarDistanceCm"] <= obstacle_max_cm)
    ].copy()
    if obs.empty:
        return obs

    obs["obstacle_m"] = obs["SonarDistanceCm"] / 100.0
    heading_rad = np.radians(obs["heading_deg"])
    obs["obstacle_x_m"] = obs["x_m"] + obs["obstacle_m"] * np.sin(heading_rad)
    obs["obstacle_y_m"] = obs["y_m"] + obs["obstacle_m"] * np.cos(heading_rad)
    obs["obstacle_z_m"] = obs["z_m"].fillna(0.0)
    obs["obstacle_weight"] = 1.0 / np.maximum(obs["obstacle_m"], 0.2)
    return obs


def build_event_table(df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []

    phase_change = df["Phase"].ne(df["Phase"].shift(1)) & df["Phase"].notna()
    for _, row in df[phase_change].iterrows():
        rows.append(
            {
                "event_type": "phase_change",
                "elapsed_sec": float(row["ElapsedSec"]),
                "phase": int(row["Phase"]),
                "label": f"Phase -> {int(row['Phase'])}",
                "x_m": float(row["x_m"]) if pd.notna(row["x_m"]) else np.nan,
                "y_m": float(row["y_m"]) if pd.notna(row["y_m"]) else np.nan,
                "z_m": float(row["z_m"]) if pd.notna(row["z_m"]) else np.nan,
            }
        )

    cone_mask = (
        (df["ConeProb"].fillna(0.0) >= 0.25)
        | (df["ConeIsReached"].fillna(0.0) > 0.0)
        | (df["ConeCloseReachedOK"].fillna(0.0) > 0.0)
    )
    cone_events = df[cone_mask].copy()
    if not cone_events.empty:
        cone_events["cone_bucket"] = (cone_events["ElapsedSec"].fillna(0.0) / 5.0).astype(int)
        cone_events = cone_events.sort_values("ConeProb", ascending=False).drop_duplicates("cone_bucket")
        for _, row in cone_events.head(20).iterrows():
            rows.append(
                {
                    "event_type": "cone_detection",
                    "elapsed_sec": float(row["ElapsedSec"]),
                    "phase": int(row["Phase"]) if pd.notna(row["Phase"]) else np.nan,
                    "label": (
                        f"Cone prob={float(row['ConeProb']):.2f} dir={float(row['ConeDir']):.2f} "
                        f"reached={int(float(row['ConeIsReached'])) if pd.notna(row['ConeIsReached']) else 0} "
                        f"method={row['ConeMethod']} penalties={row['ConePenaltyFlags']}"
                    ),
                    "x_m": float(row["x_m"]) if pd.notna(row["x_m"]) else np.nan,
                    "y_m": float(row["y_m"]) if pd.notna(row["y_m"]) else np.nan,
                    "z_m": float(row["z_m"]) if pd.notna(row["z_m"]) else np.nan,
                }
            )

    close_obs = df[df["sonar_valid"] & (df["SonarDistanceCm"].fillna(np.inf) <= 60.0)].copy()
    if not close_obs.empty:
        close_obs["obs_bucket"] = (close_obs["ElapsedSec"].fillna(0.0) / 3.0).astype(int)
        close_obs = close_obs.sort_values("SonarDistanceCm", ascending=True).drop_duplicates("obs_bucket")
        for _, row in close_obs.head(20).iterrows():
            rows.append(
                {
                    "event_type": "close_obstacle",
                    "elapsed_sec": float(row["ElapsedSec"]),
                    "phase": int(row["Phase"]) if pd.notna(row["Phase"]) else np.nan,
                    "label": f"Reflection {float(row['SonarDistanceCm']):.0f}cm",
                    "x_m": float(row["x_m"]) if pd.notna(row["x_m"]) else np.nan,
                    "y_m": float(row["y_m"]) if pd.notna(row["y_m"]) else np.nan,
                    "z_m": float(row["z_m"]) if pd.notna(row["z_m"]) else np.nan,
                }
            )

    if not rows:
        return pd.DataFrame(columns=["event_type", "elapsed_sec", "phase", "label", "x_m", "y_m", "z_m"])
    return pd.DataFrame(rows).sort_values(["elapsed_sec", "event_type"]).reset_index(drop=True)


def _idw_grid(points_x: np.ndarray, points_y: np.ndarray, points_z: np.ndarray, grid_size: int = 80) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    finite = np.isfinite(points_x) & np.isfinite(points_y) & np.isfinite(points_z)
    if finite.sum() < 3:
        return None

    px = points_x[finite]
    py = points_y[finite]
    pz = points_z[finite]
    if len(px) > 800:
        sample_idx = np.linspace(0, len(px) - 1, 800).astype(int)
        px = px[sample_idx]
        py = py[sample_idx]
        pz = pz[sample_idx]

    pad = max(3.0, 0.05 * max(px.max() - px.min(), py.max() - py.min(), 1.0))
    gx = np.linspace(px.min() - pad, px.max() + pad, grid_size)
    gy = np.linspace(py.min() - pad, py.max() + pad, grid_size)
    grid_x, grid_y = np.meshgrid(gx, gy)
    dx = grid_x[..., None] - px[None, None, :]
    dy = grid_y[..., None] - py[None, None, :]
    dist = np.hypot(dx, dy)
    weights = 1.0 / np.maximum(dist, 1.0) ** 2
    grid_z = np.sum(weights * pz[None, None, :], axis=2) / np.sum(weights, axis=2)
    return grid_x, grid_y, grid_z


def plot_static_map(
    df: pd.DataFrame,
    obstacle_df: pd.DataFrame,
    events_df: pd.DataFrame,
    out_dir: Path,
    terrain_grid: tuple[np.ndarray, np.ndarray, np.ndarray] | None,
) -> None:
    if not MATPLOTLIB_AVAILABLE:
        return
    gps = df[df["gps_valid"]].copy()
    if gps.empty:
        return

    fig, ax = plt.subplots(figsize=(10.5, 9))
    if terrain_grid is not None:
        grid_x, grid_y, grid_z = terrain_grid
        contour = ax.contourf(grid_x, grid_y, grid_z, levels=18, cmap="terrain", alpha=0.45)
        cbar = fig.colorbar(contour, ax=ax, pad=0.01)
        cbar.set_label("Relative altitude [m]")

    sc = ax.scatter(
        gps["x_m"],
        gps["y_m"],
        c=gps["z_m"].fillna(0.0),
        cmap="viridis",
        s=10,
        alpha=0.9,
        label="Trajectory",
    )
    fig.colorbar(sc, ax=ax, pad=0.04, label="Path altitude [m]")
    ax.plot(gps["x_m"], gps["y_m"], color="black", linewidth=0.8, alpha=0.55)

    ax.scatter(gps["x_m"].iloc[0], gps["y_m"].iloc[0], color="limegreen", s=90, marker="o", label="Start")
    ax.scatter(gps["x_m"].iloc[-1], gps["y_m"].iloc[-1], color="orangered", s=90, marker="X", label="End")

    target_x = df.attrs.get("target_x_m")
    target_y = df.attrs.get("target_y_m")
    if target_x is not None and target_y is not None:
        ax.scatter(float(target_x), float(target_y), color="gold", edgecolor="black", s=180, marker="*", label="Target")

    if not obstacle_df.empty:
        ax.scatter(
            obstacle_df["obstacle_x_m"],
            obstacle_df["obstacle_y_m"],
            c=obstacle_df["obstacle_m"],
            cmap="autumn_r",
            s=20,
            alpha=0.65,
            marker="s",
            label="Projected sonar reflections",
        )

    if not events_df.empty:
        phase_df = events_df[events_df["event_type"] == "phase_change"]
        if not phase_df.empty:
            ax.scatter(phase_df["x_m"], phase_df["y_m"], color="white", edgecolor="black", s=80, marker="D", label="Phase change")
            for _, row in phase_df.iterrows():
                ax.text(float(row["x_m"]), float(row["y_m"]), str(int(row["phase"])), fontsize=8, ha="left", va="bottom")

        cone_df = events_df[events_df["event_type"] == "cone_detection"]
        if not cone_df.empty:
            ax.scatter(cone_df["x_m"], cone_df["y_m"], color="deepskyblue", s=70, marker="^", label="Cone event")

    ax.set_title("CanSat Explorer Map")
    ax.set_xlabel("Local East [m]")
    ax.set_ylabel("Local North [m]")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.axis("equal")
    ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(out_dir / "explorer_map.png", dpi=170)
    plt.close(fig)


def plot_altitude_profile(df: pd.DataFrame, out_dir: Path) -> None:
    if not MATPLOTLIB_AVAILABLE:
        return
    gps = df[df["gps_valid"]].copy()
    if gps.empty:
        return

    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(gps["track_distance_m"], gps["z_m"], color="saddlebrown", linewidth=1.4)
    phase_change = gps["Phase"].ne(gps["Phase"].shift(1)) & gps["Phase"].notna()
    for _, row in gps[phase_change].iterrows():
        ax.axvline(float(row["track_distance_m"]), color="gray", linewidth=0.8, alpha=0.35)
        ax.text(float(row["track_distance_m"]), float(row["z_m"]), f"P{int(row['Phase'])}", fontsize=8, ha="left", va="bottom")
    ax.set_title("Traversed Terrain Profile")
    ax.set_xlabel("Travelled distance [m]")
    ax.set_ylabel("Relative altitude [m]")
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_dir / "terrain_profile.png", dpi=170)
    plt.close(fig)


def plot_interactive_3d(
    df: pd.DataFrame,
    obstacle_df: pd.DataFrame,
    events_df: pd.DataFrame,
    out_dir: Path,
    terrain_grid: tuple[np.ndarray, np.ndarray, np.ndarray] | None,
) -> None:
    if not PLOTLY_AVAILABLE:
        return

    gps = df[df["gps_valid"]].copy()
    if gps.empty:
        return

    fig = go.Figure()
    if terrain_grid is not None:
        grid_x, grid_y, grid_z = terrain_grid
        fig.add_trace(
            go.Surface(
                x=grid_x,
                y=grid_y,
                z=grid_z,
                colorscale="Earth",
                opacity=0.55,
                showscale=False,
                name="Terrain",
                hovertemplate="x=%{x:.1f}m<br>y=%{y:.1f}m<br>z=%{z:.1f}m<extra>Terrain</extra>",
            )
        )

    fig.add_trace(
        go.Scatter3d(
            x=gps["x_m"],
            y=gps["y_m"],
            z=gps["z_m"].fillna(0.0),
            mode="lines+markers",
            marker={"size": 3.5, "color": gps["Phase"].fillna(-1), "colorscale": "Turbo"},
            line={"width": 5, "color": "#0f172a"},
            name="Trajectory",
            hovertemplate="t=%{text:.1f}s<br>x=%{x:.1f}m<br>y=%{y:.1f}m<br>z=%{z:.1f}m<extra>Path</extra>",
            text=gps["ElapsedSec"].fillna(0.0),
        )
    )

    if not obstacle_df.empty:
        fig.add_trace(
            go.Scatter3d(
                x=obstacle_df["obstacle_x_m"],
                y=obstacle_df["obstacle_y_m"],
                z=obstacle_df["obstacle_z_m"],
                mode="markers",
                marker={"size": 3, "color": obstacle_df["obstacle_m"], "colorscale": "YlOrRd", "opacity": 0.8},
                name="Sonar reflections",
                hovertemplate="dist=%{text:.2f}m<br>x=%{x:.1f}m<br>y=%{y:.1f}m<extra>Sonar reflection</extra>",
                text=obstacle_df["obstacle_m"],
            )
        )

    if not events_df.empty:
        fig.add_trace(
            go.Scatter3d(
                x=events_df["x_m"],
                y=events_df["y_m"],
                z=events_df["z_m"].fillna(0.0),
                mode="markers+text",
                marker={"size": 5, "color": "#22c55e"},
                text=events_df["event_type"],
                textposition="top center",
                name="Events",
                hovertemplate="%{customdata}<br>t=%{text}<extra>Event</extra>",
                customdata=events_df["label"],
            )
        )

    target_x = df.attrs.get("target_x_m")
    target_y = df.attrs.get("target_y_m")
    if target_x is not None and target_y is not None:
        fig.add_trace(
            go.Scatter3d(
                x=[float(target_x)],
                y=[float(target_y)],
                z=[0.0],
                mode="markers",
                marker={"size": 8, "color": "gold", "symbol": "diamond"},
                name="Target",
            )
        )

    fig.update_layout(
        title="CanSat Explorer Reconstruction 3D",
        scene={
            "xaxis_title": "East [m]",
            "yaxis_title": "North [m]",
            "zaxis_title": "Relative altitude [m]",
            "aspectmode": "data",
        },
        height=820,
    )
    fig.write_html(out_dir / "explorer_map_3d.html", include_plotlyjs="cdn")


def _parse_datetime_from_text(text: str) -> datetime | None:
    patterns = [
        ("%Y%m%d_%H%M%S", r"(\d{8}_\d{6})"),
        ("%Y-%m%d-%H%M%S", r"(\d{4}-\d{4}-\d{6})"),
        ("%Y%m%d-%H%M%S", r"(\d{8}-\d{6})"),
    ]
    for fmt, pattern in patterns:
        match = re.search(pattern, text)
        if not match:
            continue
        try:
            return datetime.strptime(match.group(1), fmt)
        except ValueError:
            continue
    return None


def collect_camera_frames(camera_dir: Path) -> list[CameraFrame]:
    frames: list[CameraFrame] = []
    for path in sorted(camera_dir.rglob("*")):
        if path.suffix.lower() not in IMAGE_EXTS or not path.is_file():
            continue
        captured_at = _parse_datetime_from_text(path.name)
        if captured_at is None:
            captured_at = datetime.fromtimestamp(path.stat().st_mtime)
        frames.append(CameraFrame(path=path, captured_at=captured_at))
    return frames


def write_camera_event_gallery(
    events_df: pd.DataFrame,
    out_dir: Path,
    mission_start_dt: datetime | None,
    camera_dir: Path | None,
    tolerance_sec: float = 3.0,
) -> None:
    if camera_dir is None or mission_start_dt is None or events_df.empty:
        return
    if not camera_dir.exists():
        return

    frames = collect_camera_frames(camera_dir)
    if not frames:
        return

    image_out = out_dir / "camera_event_frames"
    image_out.mkdir(parents=True, exist_ok=True)
    gallery_rows = []

    focus_events = events_df[events_df["event_type"].isin(["cone_detection", "close_obstacle"])].copy()
    for _, row in focus_events.iterrows():
        event_dt = mission_start_dt + timedelta(seconds=float(row["elapsed_sec"]))
        nearest = min(frames, key=lambda frame: abs((frame.captured_at - event_dt).total_seconds()))
        delta_sec = abs((nearest.captured_at - event_dt).total_seconds())
        if delta_sec > tolerance_sec:
            continue

        dst_name = f"{row['event_type']}_{int(float(row['elapsed_sec'])):05d}{nearest.path.suffix.lower()}"
        dst = image_out / dst_name
        if not dst.exists():
            shutil.copy2(nearest.path, dst)
        gallery_rows.append(
            {
                "event_type": row["event_type"],
                "elapsed_sec": float(row["elapsed_sec"]),
                "label": str(row["label"]),
                "camera_time": nearest.captured_at.isoformat(sep=" "),
                "delta_sec": round(delta_sec, 3),
                "image_path": str(dst.relative_to(out_dir)),
            }
        )

    if not gallery_rows:
        return

    gallery_df = pd.DataFrame(gallery_rows).sort_values("elapsed_sec")
    gallery_df.to_csv(out_dir / "camera_event_gallery.csv", index=False)

    html_lines = [
        "<html><head><meta charset='utf-8'><title>Camera Event Gallery</title></head><body>",
        "<h1>Camera Event Gallery</h1>",
    ]
    for _, row in gallery_df.iterrows():
        html_lines.extend(
            [
                "<div style='margin-bottom:24px;'>",
                f"<h3>{row['event_type']} @ {row['elapsed_sec']:.1f}s</h3>",
                f"<p>{row['label']} | camera delta={row['delta_sec']:.2f}s | {row['camera_time']}</p>",
                f"<img src='{row['image_path']}' style='max-width:720px;height:auto;border:1px solid #ccc;'/>",
                "</div>",
            ]
        )
    html_lines.append("</body></html>")
    (out_dir / "camera_event_gallery.html").write_text("\n".join(html_lines), encoding="utf-8")


def write_summary_text(
    summary: dict[str, float | int | str],
    phase_summary: pd.DataFrame,
    df: pd.DataFrame,
    obstacle_df: pd.DataFrame,
    out_dir: Path,
    mission_start_dt: datetime | None,
) -> None:
    lines = [
        "CanSat Explorer Reconstruction Summary",
        f"Source rows: {summary['rows']}",
        f"Mission elapsed: {summary['mission_elapsed_sec']:.1f}s",
        f"Travelled distance: {summary['traveled_distance_m']:.1f}m",
        f"GPS valid ratio: {summary['gps_valid_ratio']:.1%}",
        f"Altitude range: {summary['altitude_min_rel_m']:.1f}m .. {summary['altitude_max_rel_m']:.1f}m (relative)",
        f"Max GPS speed: {summary['max_speed_mps']:.2f}m/s",
        f"Best cone probability: {summary['best_cone_prob']:.2f}",
        (
            f"Cone valid ratio: {summary['cone_valid_ratio']:.1%}"
            if summary["cone_diagnostics_available"]
            else "Cone valid ratio: N/A (legacy log schema)"
        ),
        f"Cone reached rows: {summary['cone_reached_rows']}",
        f"Cone close-reached-qualified rows: {summary['cone_close_reached_rows']}",
        f"Closest sonar reflection: {summary['closest_obstacle_cm']:.1f}cm" if not pd.isna(summary["closest_obstacle_cm"]) else "Closest sonar reflection: N/A",
        f"Mission end reason: {summary['mission_end_reason'] or 'N/A'}",
        f"Mission start (from log filename): {mission_start_dt.isoformat(sep=' ')}" if mission_start_dt else "Mission start (from log filename): unavailable",
        "",
        "Target / origin:",
        f"Origin LAT/LNG: {df.attrs.get('origin_lat'):.6f}, {df.attrs.get('origin_lng'):.6f}",
        f"Target LAT/LNG: {df.attrs.get('target_lat'):.6f}, {df.attrs.get('target_lng'):.6f}",
        f"Target local XY: ({df.attrs.get('target_x_m'):.1f}, {df.attrs.get('target_y_m'):.1f}) m",
        "",
        "Phase summary:",
    ]
    if phase_summary.empty:
        lines.append("No phase summary available")
    else:
        for _, row in phase_summary.iterrows():
            lines.append(
                "  "
                + f"P{int(row['phase'])}: {row['duration_sec']:.1f}s, "
                + f"distance={row['distance_m']:.1f}m, rows={int(row['rows'])}"
            )
    lines.extend(
        [
            "",
            f"Projected sonar reflection points: {len(obstacle_df)}",
            "Generated files:",
            "  explorer_map.png",
            "  terrain_profile.png",
            "  explorer_track.csv",
            "  obstacle_points.csv",
            "  mission_events.csv",
            "  phase_summary.csv",
        ]
    )
    if PLOTLY_AVAILABLE:
        lines.append("  explorer_map_3d.html")
    (out_dir / "summary.txt").write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reconstruct CanSat explorer-style map from robust mission log."
    )
    parser.add_argument(
        "log_path",
        nargs="?",
        help="Path to a run directory, mission_*.csv, mission.csv, or legacy robust_log_*.csv.",
    )
    parser.add_argument(
        "--camera-dir",
        type=Path,
        default=None,
        help="Optional camera image directory. Matching is done by filename timestamp or file mtime.",
    )
    parser.add_argument(
        "--obstacle-max-cm",
        type=float,
        default=400.0,
        help="Upper bound of sonar range used for map projection.",
    )
    parser.add_argument(
        "--grid-size",
        type=int,
        default=80,
        help="Terrain interpolation grid size for map and 3D output.",
    )
    return parser.parse_args()


def analyze_explorer_log(
    file_path: str | Path | None = None,
    *,
    camera_dir: Path | None = None,
    obstacle_max_cm: float = 400.0,
    grid_size: int = 80,
) -> Path:
    ensure_runtime_dependencies()
    log_path = resolve_log_path(file_path)

    # Final-phase diagnosis remains available even if GPS cannot produce a map.
    out_dir = prepare_output_dir(log_path)
    write_analysis_context(out_dir, log_path=log_path)
    write_final_approach_report(log_path, out_dir)

    raw_df = pd.read_csv(log_path)
    df = build_mission_dataframe(raw_df)
    phase_summary, summary = summarize_mission(df)
    obstacle_df = build_obstacle_map(df, obstacle_max_cm=obstacle_max_cm)
    events_df = build_event_table(df)
    terrain_grid = _idw_grid(
        df.loc[df["gps_valid"], "x_m"].to_numpy(dtype=float),
        df.loc[df["gps_valid"], "y_m"].to_numpy(dtype=float),
        df.loc[df["gps_valid"], "z_m"].fillna(0.0).to_numpy(dtype=float),
        grid_size=max(30, int(grid_size)),
    )
    mission_start_dt = parse_log_start_time(log_path)
    if camera_dir is None:
        bundled_camera_dir = log_path.parent / "camera"
        if bundled_camera_dir.is_dir():
            camera_dir = bundled_camera_dir

    export_cols = [
        col
        for col in [
            "ElapsedSec",
            "Phase",
            "LAT",
            "LNG",
            "ALT",
            "x_m",
            "y_m",
            "z_m",
            "track_distance_m",
            "heading_deg",
            "GpsSpeedMps",
            "ConeDir",
            "ConeProb",
            "ConeValid",
            "ConeSeq",
            "ConeStaleSec",
            "ConeIsDetected",
            "ConeProbabilityDetected",
            "ConeIsReached",
            "ConeRawReached",
            "ConeCloseReachedOK",
            "ConeRawProb",
            "ConeCandidateProb",
            "ConePreFilterProb",
            "ConeOccupancy",
            "ConeFrameRedOccupancy",
            "ConeROISupport",
            "ConeROIHistAvailable",
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
            "ConePhaseReachedProbabilityThreshold",
            "ConePhaseCenterTolerance",
            "ConePhaseDirectionTolerance",
            "ConePhaseRequiredConfirmFrames",
            "SonarDistanceCm",
            "SonarValid",
            "SonarStaleSec",
            "Distance",
            "Azimuth",
            "MissionEndReason",
            *FINAL_DIAGNOSTIC_DEFAULTS,
        ]
        if col in df.columns
    ]
    df[export_cols].to_csv(out_dir / "explorer_track.csv", index=False)
    obstacle_cols = [
        col
        for col in [
            "ElapsedSec",
            "Phase",
            "obstacle_m",
            "obstacle_x_m",
            "obstacle_y_m",
            "obstacle_z_m",
            "SonarDistanceCm",
            "heading_deg",
        ]
        if col in obstacle_df.columns
    ]
    obstacle_df[obstacle_cols].to_csv(out_dir / "obstacle_points.csv", index=False)
    events_df.to_csv(out_dir / "mission_events.csv", index=False)
    phase_summary.to_csv(out_dir / "phase_summary.csv", index=False)

    plot_static_map(df, obstacle_df, events_df, out_dir, terrain_grid)
    plot_altitude_profile(df, out_dir)
    plot_interactive_3d(df, obstacle_df, events_df, out_dir, terrain_grid)
    write_camera_event_gallery(events_df, out_dir, mission_start_dt, camera_dir)
    write_summary_text(summary, phase_summary, df, obstacle_df, out_dir, mission_start_dt)

    print(f"[INFO] Source log     : {log_path}")
    print(f"[INFO] Output dir     : {out_dir}")
    print(f"[INFO] Valid GPS rows  : {int(df['gps_valid'].sum())} / {len(df)}")
    print(f"[INFO] Travel distance : {summary['traveled_distance_m']:.1f} m")
    print(f"[INFO] Sonar reflections projected: {len(obstacle_df)}")
    print(f"[INFO] Matplotlib      : {'enabled' if MATPLOTLIB_AVAILABLE else 'skipped (matplotlib not installed)'}")
    print(f"[INFO] Plotly 3D       : {'enabled' if PLOTLY_AVAILABLE else 'skipped (plotly not installed)'}")
    return out_dir


def main() -> None:
    args = parse_args()
    analyze_explorer_log(
        args.log_path,
        camera_dir=args.camera_dir,
        obstacle_max_cm=args.obstacle_max_cm,
        grid_size=args.grid_size,
    )


if __name__ == "__main__":
    main()
