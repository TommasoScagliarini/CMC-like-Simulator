"""Freeze a selected actor checkpoint with an auditable feature contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Mapping

import warm_start


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"expected a JSON object: {path}")
    return dict(payload)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _actor_features(source_run: Path) -> tuple[str, ...]:
    for filename in ("adaptation_report.json", "run_summary.json"):
        path = source_run / filename
        if not path.is_file():
            continue
        payload = _read_json(path)
        adaptation = payload.get("adaptation", payload)
        if isinstance(adaptation, Mapping):
            names = adaptation.get("actor_feature_names")
            if isinstance(names, list) and names:
                return tuple(str(name) for name in names)
    raise ValueError(
        f"no actor_feature_names found in adaptation_report.json or run_summary.json: "
        f"{source_run}"
    )


def freeze(args: argparse.Namespace) -> dict[str, Any]:
    source_run = Path(args.source_run).expanduser().resolve()
    source_module = warm_start.resolve_source_checkpoint(args.source_module)
    output_dir = Path(args.output_dir).expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(f"freeze output already exists: {output_dir}")

    source_state = warm_start.load_module_state(source_module)
    actor_digest = warm_start.actor_state_digest(source_state)
    if args.expected_actor_digest and actor_digest != args.expected_actor_digest:
        raise ValueError(
            f"selected actor digest {actor_digest} != expected "
            f"{args.expected_actor_digest}"
        )
    features = _actor_features(source_run)
    first_width = int(source_state["pi_encoder.0.weight"].shape[1])
    if len(features) != first_width:
        raise ValueError(
            f"feature count {len(features)} != source first-layer width {first_width}"
        )

    output_dir.mkdir(parents=True)
    frozen_module = output_dir / "rl_module_warm_start"
    shutil.copytree(source_module, frozen_module, copy_function=shutil.copy2)
    source_config = source_run / "training_cfg.resolved.yaml"
    if not source_config.is_file():
        raise FileNotFoundError(f"missing resolved source config: {source_config}")
    frozen_config = output_dir / "training_cfg.resolved.yaml"
    shutil.copy2(source_config, frozen_config)

    module_files = {
        path.name: _sha256(path)
        for path in sorted(frozen_module.iterdir())
        if path.is_file()
    }
    actor_manifest = {
        "schema_version": 1,
        "actor_feature_count": len(features),
        "actor_feature_names": list(features),
        "actor_digest": actor_digest,
        "module_state_sha256": module_files["module_state.pkl"],
    }
    actor_manifest_path = output_dir / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    warm_start.write_report(actor_manifest_path, actor_manifest)

    reloaded_state = warm_start.load_module_state(frozen_module)
    reload_comparison = warm_start.compare_actor_states(source_state, reloaded_state)
    if not reload_comparison["exact"]:
        raise RuntimeError("frozen actor differs from selected source checkpoint")
    freeze_manifest = {
        "ok": True,
        "schema_version": 1,
        "selection_status": "frozen_for_warm_start_preflight",
        "selection_reason": args.selection_reason,
        "source_run": str(source_run),
        "source_module": str(source_module),
        "frozen_module": str(frozen_module),
        "frozen_config": str(frozen_config),
        "actor_feature_manifest": str(actor_manifest_path),
        "actor_digest": actor_digest,
        "actor_feature_count": len(features),
        "module_file_sha256": module_files,
        "source_config_sha256": _sha256(source_config),
        "frozen_config_sha256": _sha256(frozen_config),
        "source_to_frozen_actor": reload_comparison,
    }
    warm_start.write_report(output_dir / "freeze_manifest.json", freeze_manifest)
    return freeze_manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", required=True)
    parser.add_argument("--source-module", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--expected-actor-digest", default=None)
    parser.add_argument("--selection-reason", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(freeze(parse_args()), indent=2))
