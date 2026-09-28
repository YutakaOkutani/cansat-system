import math
import re
from dataclasses import dataclass, field, fields
from mission.approach import ApproachConfig
from pathlib import Path

from mission.paths import RUN_CONTEXT_PATH

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 and older
    tomllib = None


class MissionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class TargetConfig:
    latitude: float
    longitude: float


@dataclass(frozen=True)
class RadioConfig:
    control: str
    pre_off_delay_sec: float
    restore_timeout_sec: float
    use_sudo: bool
    dry_run: bool


@dataclass(frozen=True)
class MissionConfig:
    target: TargetConfig
    radio: RadioConfig
    source: Path
    approach: ApproachConfig = field(default_factory=ApproachConfig)


@dataclass(frozen=True)
class RunContextConfig:
    event_id: str
    event_name: str
    run_kind: str
    label: str
    location: str
    operator: str
    tags: tuple[str, ...]
    notes: str
    source: Path | None


_REPO_ROOT = Path(__file__).resolve().parents[1]
MISSION_CONFIG_PATH = _REPO_ROOT / "mission.toml"
_ROOT_KEYS = {"target", "radio", "approach"}
_TARGET_KEYS = {"latitude", "longitude"}
_RADIO_KEYS = {
    "control",
    "pre_off_delay_sec",
    "restore_timeout_sec",
    "use_sudo",
    "dry_run",
}
_RUN_CONTEXT_ROOT_KEYS = {"event", "run"}
_EVENT_KEYS = {"id", "name"}
_RUN_KEYS = {"kind", "label", "location", "operator", "tags", "notes"}
_SAFE_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]*$")


def _require_table(data, key, path):
    value = data.get(key)
    if not isinstance(value, dict):
        raise MissionConfigError(f"{path}: [{key}] table is required")
    return value


def _reject_unknown_keys(table, allowed, label, path):
    unknown = sorted(set(table) - allowed)
    if unknown:
        raise MissionConfigError(f"{path}: unknown key(s) in {label}: {', '.join(unknown)}")


def _require_number(table, key, label, path):
    if key not in table:
        raise MissionConfigError(f"{path}: {label}.{key} is required")
    value = table[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionConfigError(f"{path}: {label}.{key} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise MissionConfigError(f"{path}: {label}.{key} must be finite")
    return value


def _require_bool(table, key, label, path):
    if key not in table:
        raise MissionConfigError(f"{path}: {label}.{key} is required")
    value = table[key]
    if not isinstance(value, bool):
        raise MissionConfigError(f"{path}: {label}.{key} must be true or false")
    return value


def _optional_string(table, key, label, path, default=""):
    value = table.get(key, default)
    if not isinstance(value, str):
        raise MissionConfigError(f"{path}: {label}.{key} must be a string")
    return value.strip()


def load_run_context(path=None):
    if tomllib is None:
        raise MissionConfigError("run-context.toml requires Python 3.11 or newer")
    context_path = Path(path) if path is not None else RUN_CONTEXT_PATH
    if not context_path.exists():
        return RunContextConfig(
            event_id="unclassified",
            event_name="Unclassified",
            run_kind="mission",
            label="",
            location="",
            operator="",
            tags=(),
            notes="",
            source=None,
        )
    try:
        with context_path.open("rb") as file_obj:
            data = tomllib.load(file_obj)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise MissionConfigError(f"Failed to read {context_path}: {exc}") from exc

    if not isinstance(data, dict):
        raise MissionConfigError(f"{context_path}: root must be a TOML table")
    _reject_unknown_keys(data, _RUN_CONTEXT_ROOT_KEYS, "root", context_path)
    event = _require_table(data, "event", context_path)
    run = _require_table(data, "run", context_path)
    _reject_unknown_keys(event, _EVENT_KEYS, "event", context_path)
    _reject_unknown_keys(run, _RUN_KEYS, "run", context_path)

    event_id = _optional_string(event, "id", "event", context_path)
    run_kind = _optional_string(run, "kind", "run", context_path)
    if not event_id or not _SAFE_ID_RE.fullmatch(event_id):
        raise MissionConfigError(
            f"{context_path}: event.id must use lowercase letters, numbers, dot, underscore, or hyphen"
        )
    if not run_kind or not _SAFE_ID_RE.fullmatch(run_kind):
        raise MissionConfigError(
            f"{context_path}: run.kind must use lowercase letters, numbers, dot, underscore, or hyphen"
        )

    tags = run.get("tags", [])
    if not isinstance(tags, list) or any(not isinstance(tag, str) for tag in tags):
        raise MissionConfigError(f"{context_path}: run.tags must be an array of strings")

    return RunContextConfig(
        event_id=event_id,
        event_name=_optional_string(event, "name", "event", context_path, event_id),
        run_kind=run_kind,
        label=_optional_string(run, "label", "run", context_path),
        location=_optional_string(run, "location", "run", context_path),
        operator=_optional_string(run, "operator", "run", context_path),
        tags=tuple(tag.strip() for tag in tags if tag.strip()),
        notes=_optional_string(run, "notes", "run", context_path),
        source=context_path.resolve(),
    )


def load_mission_config(path=None):
    if tomllib is None:
        raise MissionConfigError("mission.toml requires Python 3.11 or newer")
    config_path = Path(path) if path is not None else MISSION_CONFIG_PATH
    try:
        with config_path.open("rb") as file_obj:
            data = tomllib.load(file_obj)
    except FileNotFoundError as exc:
        raise MissionConfigError(
            f"Mission config not found: {config_path}. "
            "Copy mission.toml.example to mission.toml and set the target coordinates."
        ) from exc
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise MissionConfigError(f"Failed to read {config_path}: {exc}") from exc

    if not isinstance(data, dict):
        raise MissionConfigError(f"{config_path}: root must be a TOML table")
    _reject_unknown_keys(data, _ROOT_KEYS, "root", config_path)

    target = _require_table(data, "target", config_path)
    radio = _require_table(data, "radio", config_path)
    _reject_unknown_keys(target, _TARGET_KEYS, "target", config_path)
    _reject_unknown_keys(radio, _RADIO_KEYS, "radio", config_path)

    latitude = _require_number(target, "latitude", "target", config_path)
    longitude = _require_number(target, "longitude", "target", config_path)
    if not -90.0 <= latitude <= 90.0:
        raise MissionConfigError(f"{config_path}: target.latitude must be between -90 and 90")
    if not -180.0 <= longitude <= 180.0:
        raise MissionConfigError(f"{config_path}: target.longitude must be between -180 and 180")

    control = radio.get("control")
    if not isinstance(control, str) or control not in ("off", "mission"):
        raise MissionConfigError(f'{config_path}: radio.control must be "off" or "mission"')
    pre_off_delay_sec = _require_number(radio, "pre_off_delay_sec", "radio", config_path)
    restore_timeout_sec = _require_number(radio, "restore_timeout_sec", "radio", config_path)
    if pre_off_delay_sec < 0.0:
        raise MissionConfigError(f"{config_path}: radio.pre_off_delay_sec must be non-negative")
    if restore_timeout_sec < 0.0:
        raise MissionConfigError(f"{config_path}: radio.restore_timeout_sec must be non-negative")

    approach_table = data.get('approach', {})
    if not isinstance(approach_table, dict):
        raise MissionConfigError(f'{config_path}: approach must be a table')
    defaults = ApproachConfig()
    _reject_unknown_keys(approach_table, {f.name for f in fields(defaults)}, 'approach', config_path)
    overrides = {}
    for name in approach_table:
        value = _require_number(approach_table, name, 'approach', config_path)
        upper = (100 if name.endswith('_pwm') else 30 if name == 'max_turn_deg'
                 else 120 if 'output_limit' in name else 10 if 'progress_output' in name else .6)
        if not 0 < value <= upper:
            raise MissionConfigError(f'{config_path}: approach.{name} must be > 0 and <= {upper}')
        overrides[name] = value
    approach = ApproachConfig(**overrides)
    for base, boost in (('far_forward_sec', 'far_forward_boost_sec'),
                        ('mid_forward_sec', 'mid_forward_boost_sec'),
                        ('far_align_sec', 'far_align_boost_sec'),
                        ('far_forward_pwm', 'far_forward_boost_pwm'),
                        ('far_align_pwm', 'far_align_boost_pwm')):
        if getattr(approach, boost) < getattr(approach, base):
            raise MissionConfigError(f'{config_path}: approach.{boost} must be >= {base}')

    if approach.near_align_boost_sec < .05 or approach.near_align_boost_pwm < 45:
        raise MissionConfigError(f'{config_path}: near alignment boost must preserve the base pulse')

    return MissionConfig(
        target=TargetConfig(latitude=latitude, longitude=longitude),
        radio=RadioConfig(
            control=control,
            pre_off_delay_sec=pre_off_delay_sec,
            restore_timeout_sec=restore_timeout_sec,
            use_sudo=_require_bool(radio, "use_sudo", "radio", config_path),
            dry_run=_require_bool(radio, "dry_run", "radio", config_path),
        ),
        source=config_path.resolve(),
        approach=approach,
    )
