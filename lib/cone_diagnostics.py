"""Shared, hardware-free schema helpers for cone detector diagnostics."""

from __future__ import annotations

import math


CONE_DIAGNOSTIC_SCHEMA_VERSION = 5

# key, mission CSV column, default, value kind
CONE_DIAGNOSTIC_FIELDS = (
    ("schema_version", "ConeDiagSchemaVersion", CONE_DIAGNOSTIC_SCHEMA_VERSION, "int"),
    ("valid", "ConeValid", 0, "int"),
    ("status", "ConeStatus", "not_started", "str"),
    ("sequence", "ConeSeq", 0, "int"),
    ("direction", "ConeDir", 0.5, "float"),
    ("image_direction", "ConeImageDir", 0.5, "float"),
    ("image_direction_valid", "ConeImageDirValid", 0, "int"),
    ("probability", "ConeProb", 0.0, "float"),
    ("method", "ConeMethod", "", "str"),
    ("is_detected", "ConeIsDetected", 0, "int"),
    ("probability_detected", "ConeProbabilityDetected", 0, "int"),
    ("is_reached", "ConeIsReached", 0, "int"),
    ("raw_reached", "ConeRawReached", 0, "int"),
    ("close_reached_ok", "ConeCloseReachedOK", 0, "int"),
    ("raw_probability", "ConeRawProb", 0.0, "float"),
    ("candidate_probability", "ConeCandidateProb", 0.0, "float"),
    ("pre_filter_probability", "ConePreFilterProb", 0.0, "float"),
    ("occupancy", "ConeOccupancy", 0.0, "float"),
    ("frame_red_occupancy", "ConeFrameRedOccupancy", 0.0, "float"),
    ("shape_score", "ConeShapeScore", 0.0, "float"),
    ("cone_shape_score", "ConeGeometryScore", 0.0, "float"),
    ("sv_score", "ConeSVScore", 0.0, "float"),
    ("hue_redness_score", "ConeHueScore", 0.0, "float"),
    ("roi_support_ratio", "ConeROISupport", 0.0, "float"),
    ("roi_absolute_support", "ConeROIAbsoluteSupport", 0.0, "float"),
    ("roi_negative_support", "ConeROINegativeSupport", 0.0, "float"),
    ("candidate_count", "ConeCandidateCount", 0, "int"),
    ("candidate_rank", "ConeCandidateRank", 0, "int"),
    ("candidate_temporal_bonus", "ConeCandidateTemporalBonus", 0.0, "float"),
    ("edge_touch_count", "ConeEdgeTouchCount", 0, "int"),
    ("bbox_x", "ConeBBoxX", 0, "int"),
    ("bbox_y", "ConeBBoxY", 0, "int"),
    ("bbox_width", "ConeBBoxWidth", 0, "int"),
    ("bbox_height", "ConeBBoxHeight", 0, "int"),
    ("bbox_width_frac", "ConeBBoxWidthFrac", 0.0, "float"),
    ("bbox_height_frac", "ConeBBoxHeightFrac", 0.0, "float"),
    ("bbox_bottom_frac", "ConeBBoxBottomFrac", 0.0, "float"),
    ("bbox_aspect", "ConeBBoxAspect", 0.0, "float"),
    ("strict_red_ok", "ConeStrictRedOK", 0, "int"),
    ("strict_red_reject_reason", "ConeStrictRedRejectReason", "", "str"),
    ("close_region_ok", "ConeCloseRegionOK", 0, "int"),
    ("dominant_close", "ConeDominantClose", 0, "int"),
    ("close_reached_reject_reason", "ConeCloseReachedRejectReason", "", "str"),
    ("ground_penalty", "ConeGroundPenalty", 1.0, "float"),
    ("penalty_flags", "ConePenaltyFlags", "", "str"),
    ("roi_hist_available", "ConeROIHistAvailable", 0, "int"),
    ("roi_positive_count", "ConeROIPositiveCount", 0, "int"),
    ("roi_negative_count", "ConeROINegativeCount", 0, "int"),
    ("roi_positive_weight", "ConeROIPositiveWeight", 0.0, "float"),
    ("roi_negative_weight", "ConeROINegativeWeight", 0.0, "float"),
    ("as_is_probability", "ConeAsIsProb", 0.0, "float"),
    ("swap_probability", "ConeSwapProb", 0.0, "float"),
    ("swap_used", "ConeSwapUsed", 0, "int"),
    ("swap_margin", "ConeSwapMargin", 0.0, "float"),
    ("close_track_reason", "ConeCloseTrackReason", "inactive", "str"),
    ("close_track_count", "ConeCloseTrackCount", 0, "int"),
    ("close_track_eligible", "ConeCloseTrackEligible", 0, "int"),
    ("close_track_hold", "ConeCloseTrackHold", 0, "int"),
)

CONE_DIAGNOSTIC_KEYS = tuple(field[0] for field in CONE_DIAGNOSTIC_FIELDS)
CONE_DIAGNOSTIC_LOG_COLUMNS = tuple(field[1] for field in CONE_DIAGNOSTIC_FIELDS)
CONE_DIAGNOSTIC_LOG_BY_KEY = {field[0]: field[1] for field in CONE_DIAGNOSTIC_FIELDS}


def _coerce_value(value, default, kind):
    if kind == "str":
        return str(default if value is None else value)
    if kind == "int":
        try:
            return int(value)
        except (TypeError, ValueError, OverflowError):
            return int(default)
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def normalize_cone_diagnostics(values=None):
    """Return one complete, type-normalized diagnostic record."""
    source = values or {}
    return {
        key: _coerce_value(source.get(key, default), default, kind)
        for key, _column, default, kind in CONE_DIAGNOSTIC_FIELDS
    }


def detector_diagnostics(detector, *, valid=True, status="ok", sequence=0):
    """Snapshot the public result and debug fields of a detector instance."""
    scores = dict(getattr(detector, "debug_scores", {}) or {})
    scores.update(
        {
            "valid": int(bool(valid)),
            "status": status,
            "sequence": sequence,
            "direction": getattr(detector, "cone_direction", 0.5),
            "probability": getattr(detector, "probability", 0.0),
            "method": getattr(detector, "debug_method", "unknown"),
            "is_detected": int(bool(getattr(detector, "is_detected", False))),
            "is_reached": int(bool(getattr(detector, "is_reached", False))),
            "occupancy": getattr(detector, "occupancy", 0.0),
            "frame_red_occupancy": getattr(detector, "frame_red_occupancy", 0.0),
        }
    )
    return normalize_cone_diagnostics(scores)


def mission_log_values(values):
    """Format diagnostics in the exact order used by the mission CSV schema."""
    normalized = normalize_cone_diagnostics(values)
    row = []
    for key, _column, _default, kind in CONE_DIAGNOSTIC_FIELDS:
        value = normalized[key]
        if kind == "float":
            row.append(f"{value:.6f}")
        else:
            row.append(value)
    return row
