"""Shared visual-candidate policy for the camera phases.

Phase handlers decide whether a track is stable enough to change phase, while
the motor thread decides how to keep that same track in view.  Both decisions
must start from identical per-frame evidence; otherwise the phase handler can
confirm a cone while the motor thread continues its search turn.
"""

from mission.const import (
    CAMERA_TINY_OCCUPANCY_THRESHOLD,
    CAMERA_WEAK_MIN_CANDIDATE_PROBABILITY,
    CAMERA_WEAK_MIN_HUE_SCORE,
    CAMERA_WEAK_MIN_ROI_ABSOLUTE_SUPPORT,
    CAMERA_WEAK_MIN_ROI_SUPPORT,
    CAMERA_WEAK_MIN_SHAPE_SCORE,
    CAMERA_WEAK_MIN_SV_SCORE,
    CAMERA_WEAK_RELAXED_SV_SCORE,
    CAMERA_WEAK_STRONG_HUE_SCORE,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    CONE_CENTER_POSITION,
    PHASE5_RAM_CENTER_TOLERANCE,
)


def _float_value(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def cone_centered_for_final_ram(snapshot):
    """Require a valid image direction before committing to a straight ram."""
    direction = _float_value(snapshot.get("cone_direction"), float("nan"))
    return (
        CONE_CENTER_POSITION - PHASE5_RAM_CENTER_TOLERANCE
        <= direction
        <= CONE_CENTER_POSITION + PHASE5_RAM_CENTER_TOLERANCE
    )


def evaluate_cone_candidate(snapshot):
    """Return strict/weak evidence from one camera observation.

    Weak evidence deliberately requires ROI support.  Strong hue and shape
    alone are insufficient because moving lamps and the night sky produced
    repeatable cone-shaped blobs with no ROI support in the 19:00 field log.
    """

    debug = dict(snapshot.get("cone_debug", {}) or {})
    probability = _float_value(snapshot.get("cone_probability", 0.0))
    candidate_probability = _float_value(
        debug.get("candidate_probability", 0.0)
    )
    shape = _float_value(debug.get("cone_shape_score", 0.0))
    hue = _float_value(debug.get("hue_redness_score", 0.0))
    sv = _float_value(debug.get("sv_score", 0.0))
    roi_support = _float_value(debug.get("roi_support_ratio", 0.0))
    roi_absolute_support = _float_value(
        debug.get("roi_absolute_support", 0.0)
    )
    occupancy = _float_value(debug.get("occupancy", 0.0))
    ground_penalty = _float_value(debug.get("ground_penalty", 1.0), 1.0)
    bbox_y = _float_value(debug.get("bbox_y", 0.0))
    bbox_height = _float_value(debug.get("bbox_height", 0.0))
    bbox_bottom = _float_value(debug.get("bbox_bottom_frac", 0.0))
    bbox_height_frac = _float_value(debug.get("bbox_height_frac", 0.0))
    if bbox_bottom <= 0.0 and bbox_height > 0.0 and bbox_height_frac > 0.0:
        frame_height = bbox_height / bbox_height_frac
        bbox_bottom = (bbox_y + bbox_height) / max(frame_height, 1.0)
    penalty_flags = str(debug.get("penalty_flags", ""))
    strict_red_ok = bool(int(_float_value(debug.get("strict_red_ok", 0.0))))
    # detector.is_reached の正式な意味は close_reached_ok。診断値も見ることで
    # 保存ログや診断スナップショットでも同じ判断を再現できるようにする。
    close_reached = bool(
        int(_float_value(snapshot.get("cone_is_reached", 0.0)))
        or int(_float_value(debug.get("close_reached_ok", 0.0)))
    )

    tiny = 0.0 < occupancy < float(CAMERA_TINY_OCCUPANCY_THRESHOLD)
    credible_size = occupancy <= 0.0 or not tiny
    plausible_position = (
        ground_penalty >= 0.5
        and "upper_sky" not in penalty_flags
        and (bbox_height <= 0.0 or bbox_bottom >= 0.35)
    )
    color_quality = (
        hue >= CAMERA_WEAK_MIN_HUE_SCORE
        and sv >= CAMERA_WEAK_MIN_SV_SCORE
    ) or (
        hue >= CAMERA_WEAK_STRONG_HUE_SCORE
        and sv >= CAMERA_WEAK_RELAXED_SV_SCORE
    )

    strict = bool(
        probability > CONE_PROBABILITY_THRESHOLD_PHASE4
        and strict_red_ok
        and credible_size
        and plausible_position
    )
    weak = bool(
        candidate_probability >= CAMERA_WEAK_MIN_CANDIDATE_PROBABILITY
        and shape >= CAMERA_WEAK_MIN_SHAPE_SCORE
        and color_quality
        and roi_support >= CAMERA_WEAK_MIN_ROI_SUPPORT
        and (
            not tiny
            or roi_absolute_support >= CAMERA_WEAK_MIN_ROI_ABSOLUTE_SUPPORT
        )
        and plausible_position
    )

    return {
        "candidate": bool(strict or weak),
        "strict": strict,
        "weak": weak,
        "close_reached": close_reached,
        "tiny": tiny,
        "probability": probability,
        "candidate_probability": candidate_probability,
        "occupancy": occupancy,
        "shape": shape,
        "hue": hue,
        "sv": sv,
        "roi_support": roi_support,
        "roi_absolute_support": roi_absolute_support,
    }
