"""Portable paths and immutable input receipts for the tuning workbench."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]


def resolve(path, root=REPO):
    path = Path(path)
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path):
    path = Path(path).resolve()
    try:
        return path.relative_to(REPO).as_posix()
    except ValueError:
        return str(path)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def load_config(path):
    path = resolve(path)
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("schema") != "reward_tuning_v1":
        raise ValueError("Expected schema: reward_tuning_v1")
    for key in ("source_checkpoint", "source_config", "output_dir", "plot_dir", "corpus"):
        if key not in config:
            raise ValueError(f"Missing tuning setting: {key}")
    plots = resolve(config["plot_dir"])
    if REPO / "plot" not in plots.parents:
        raise ValueError("Tuning figures must live in a dedicated root plot/ subdirectory")
    config["_config_path"] = str(path)
    config["_repo_root"] = str(REPO)
    return config
