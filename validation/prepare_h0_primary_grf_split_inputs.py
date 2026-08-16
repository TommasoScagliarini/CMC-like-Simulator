"""Create no-clobber cross-speed profile overlays for H0 split adaptation."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = REPO_ROOT / "validation" / "h0_primary_grf_split_inputs"
PRIMARY_PROFILE = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)
ANALOG_PROFILE = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
EXPECTED_SOURCE_SHA256 = {
    "primary": "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0",
    "analog": "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e",
}
TRIAL_SPEEDS_MPS = {"02": 0.95, "04": 1.05, "08": 1.25}


class InputPreparationError(RuntimeError):
    pass


def _reject_constant(token: str) -> None:
    raise ValueError(f"non-finite JSON token {token!r}")


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(
        path.read_text(encoding="utf-8"), parse_constant=_reject_constant
    )
    if not isinstance(value, Mapping):
        raise InputPreparationError(f"profile must be an object: {path}")
    json.dumps(value, allow_nan=False)
    return dict(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _record(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.resolve().relative_to(REPO_ROOT)),
        "sha256": _sha256(path.resolve()),
        "size_bytes": path.stat().st_size,
    }


def _write_exclusive(path: Path, value: Any) -> None:
    serialized = json.dumps(
        value,
        indent=2,
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    ) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(path):
        raise InputPreparationError(f"refusing to clobber: {path}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise InputPreparationError(f"refusing to clobber: {path}") from exc
        temporary.unlink()
    finally:
        if temporary.exists():
            temporary.unlink()


def _overlay(source: Mapping[str, Any], speed_mps: float) -> dict[str, Any]:
    if not math.isfinite(speed_mps) or speed_mps <= 0.0:
        raise InputPreparationError("speed must be finite and positive")
    result = copy.deepcopy(dict(source))
    ground = result.get("ground")
    if not isinstance(ground, Mapping):
        raise InputPreparationError("profile ground object is missing")
    result["ground"] = dict(ground)
    result["ground"]["surface_velocity"] = [0.0, 0.0, float(speed_mps)]
    return result


def _without_velocity(value: Mapping[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(dict(value))
    ground = result.get("ground")
    if not isinstance(ground, Mapping):
        raise InputPreparationError("profile ground object is missing")
    result["ground"] = dict(ground)
    result["ground"].pop("surface_velocity", None)
    return result


def prepare() -> dict[str, Any]:
    sources = {"primary": PRIMARY_PROFILE, "analog": ANALOG_PROFILE}
    for role, path in sources.items():
        if _sha256(path) != EXPECTED_SOURCE_SHA256[role]:
            raise InputPreparationError(f"{role} source profile hash drifted")
    loaded = {role: _load(path) for role, path in sources.items()}
    output_records: dict[str, Any] = {}
    for trial, speed in TRIAL_SPEEDS_MPS.items():
        trial_records: dict[str, Any] = {}
        for role, source in loaded.items():
            output = OUTPUT_ROOT / f"trial_{trial}_{role}_surface_velocity.json"
            overlay = _overlay(source, speed)
            if _without_velocity(overlay) != _without_velocity(source):
                raise InputPreparationError(
                    f"trial {trial} {role} overlay changed more than velocity"
                )
            _write_exclusive(output, overlay)
            reloaded = _load(output)
            if reloaded != overlay:
                raise InputPreparationError(f"trial {trial} {role} save drift")
            trial_records[role] = _record(output)
        primary_velocity = _load(
            REPO_ROOT / trial_records["primary"]["path"]
        )["ground"]["surface_velocity"]
        analog_velocity = _load(
            REPO_ROOT / trial_records["analog"]["path"]
        )["ground"]["surface_velocity"]
        expected_velocity = [0.0, 0.0, speed]
        if primary_velocity != expected_velocity or analog_velocity != expected_velocity:
            raise InputPreparationError("paired overlay velocity mismatch")
        output_records[trial] = {
            "speed_mps": speed,
            "surface_velocity_mps": expected_velocity,
            "profiles": trial_records,
        }
    manifest = {
        "schema_version": 1,
        "status": "H0_PRIMARY_GRF_SPLIT_INPUTS_PREPARED",
        "mutation_whitelist": ["ground.surface_velocity"],
        "source_profiles": {role: _record(path) for role, path in sources.items()},
        "trials": output_records,
        "protected_trials_opened": [],
        "primary_profile_overwritten": False,
        "analog_profile_overwritten": False,
    }
    _write_exclusive(OUTPUT_ROOT / "manifest.json", manifest)
    return manifest


if __name__ == "__main__":
    print(json.dumps(prepare(), indent=2, sort_keys=True, allow_nan=False))

