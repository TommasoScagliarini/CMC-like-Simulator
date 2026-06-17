"""Generate the cross-platform historical training registry.

The script is intentionally lightweight: it reads run artifacts only
(`summary.json`, `training_cfg.resolved.yaml`, checkpoint metadata, and
`train_iterations.jsonl`) and never imports Ray, Torch, or OpenSim.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


SCRIPT_PATH = Path(__file__).resolve()
BASELINE_DIR = SCRIPT_PATH.parent
TRAJ_GEN_DIR = SCRIPT_PATH.parents[1]
REPO_ROOT = SCRIPT_PATH.parents[2]
RUNS_ROOT = TRAJ_GEN_DIR / "runs"
TRAINING_ROOT = RUNS_ROOT / "training"
DEFAULT_MANUAL_PATH = RUNS_ROOT / "historical_runs.manual.yaml"
DEFAULT_MARKDOWN_PATH = RUNS_ROOT / "historical_runs.md"
DEFAULT_INDEX_PATH = RUNS_ROOT / "historical_runs.index.json"


def _platform_tag() -> str:
    if sys.platform.startswith("win"):
        return "win"
    if sys.platform == "darwin":
        return "mac"
    if sys.platform.startswith("linux"):
        return "linux"
    return sys.platform or "unknown"


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def _read_json_dict(path: Path) -> dict[str, Any]:
    data = _read_json(path)
    return data if isinstance(data, dict) else {}


def _read_yaml_dict(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def _write_text_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os_pid()}.tmp")
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(path)


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    text = json.dumps(payload, indent=2, sort_keys=False, default=str) + "\n"
    _write_text_atomic(path, text)


def os_pid() -> int:
    try:
        import os

        return os.getpid()
    except Exception:
        return 0


def _repo_rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
    except (OSError, ValueError):
        return path.as_posix()


def _clean_path_value(value: Any) -> str | None:
    if value in (None, ""):
        return None
    text = str(value).replace("\\", "/")
    repo = str(REPO_ROOT).replace("\\", "/")
    if text.startswith(repo + "/"):
        return text[len(repo) + 1 :]
    marker = "Trajectory Generator/"
    marker_index = text.find(marker)
    if marker_index >= 0:
        return text[marker_index:]
    if re.match(r"^[A-Za-z]:/", text) or text.startswith("/"):
        parts = [part for part in text.split("/") if part]
        return "external:" + "/".join(parts[-3:])
    return text


def _clean_paths_in_mapping(data: dict[str, Any]) -> dict[str, Any]:
    cleaned: dict[str, Any] = {}
    for key, value in data.items():
        if isinstance(value, dict):
            cleaned[key] = _clean_paths_in_mapping(value)
        elif isinstance(value, list):
            cleaned[key] = [
                _clean_path_value(item)
                if isinstance(item, str)
                and ("path" in key.lower() or "checkpoint" in key.lower())
                else item
                for item in value
            ]
        elif isinstance(value, str) and (
            "path" in key.lower()
            or "checkpoint" in key.lower()
            or re.match(r"^[A-Za-z]:[\\/]", value)
            or value.startswith("/")
        ):
            cleaned[key] = _clean_path_value(value)
        else:
            cleaned[key] = value
    return cleaned


def _mtime_dt(path: Path) -> datetime | None:
    try:
        return datetime.fromtimestamp(path.stat().st_mtime)
    except OSError:
        return None


def _dt_from_rllib_name(name: str) -> datetime | None:
    match = re.match(
        r"(?P<date>\d{4}-\d{2}-\d{2})_(?P<h>\d{2})-(?P<m>\d{2})-(?P<s>\d{2})",
        name,
    )
    if not match:
        return None
    raw = (
        f"{match.group('date')} "
        f"{match.group('h')}:{match.group('m')}:{match.group('s')}"
    )
    try:
        return datetime.strptime(raw, "%Y-%m-%d %H:%M:%S")
    except ValueError:
        return None


def _iso(dt: datetime | None) -> str | None:
    if dt is None:
        return None
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def _parse_iso(value: Any) -> datetime | None:
    if not value:
        return None
    text = str(value)
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        pass
    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d"):
        try:
            return datetime.strptime(text[: len(fmt)], fmt)
        except ValueError:
            continue
    return None


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _safe_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _format_duration(seconds: float | None) -> str:
    if seconds is None or not math.isfinite(seconds) or seconds < 0:
        return "-"
    total = int(round(seconds))
    hours = total // 3600
    minutes = (total % 3600) // 60
    secs = total % 60
    if hours:
        return f"{hours}h {minutes:02d}m"
    if minutes:
        return f"{minutes}m {secs:02d}s"
    return f"{secs}s"


def _format_number(value: Any, digits: int = 3) -> str:
    number = _safe_float(value)
    if number is None:
        return "-"
    return f"{number:.{digits}f}".rstrip("0").rstrip(".")


def _read_iteration_history(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    item = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(item, dict):
                    rows.append(item)
    except OSError:
        return rows
    return rows


def _load_resolved_cfg(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def _first_last_iterations(history: list[dict[str, Any]]) -> tuple[int | None, int | None]:
    iterations = [
        value
        for value in (_safe_int(row.get("iteration")) for row in history)
        if value is not None
    ]
    if not iterations:
        return None, None
    return min(iterations), max(iterations)


def _best_iteration(history: list[dict[str, Any]]) -> tuple[int | None, float | None]:
    best_iter: int | None = None
    best_return: float | None = None
    for row in history:
        ret = _safe_float(row.get("episode_return_mean"))
        iteration = _safe_int(row.get("iteration"))
        if ret is None or iteration is None:
            continue
        if best_return is None or ret > best_return:
            best_return = ret
            best_iter = iteration
    return best_iter, best_return


def _rllib_start_times(run_dir: Path) -> list[datetime]:
    rllib_dir = run_dir / "rllib"
    times: list[datetime] = []
    try:
        children = list(rllib_dir.iterdir())
    except OSError:
        return times
    for child in children:
        if not child.is_dir():
            continue
        by_name = _dt_from_rllib_name(child.name)
        if by_name is not None:
            times.append(by_name)
            continue
        by_mtime = _mtime_dt(child)
        if by_mtime is not None:
            times.append(by_mtime)
    return sorted(times)


def _infer_start_finish(
    run_dir: Path,
    summary: dict[str, Any],
    manual: dict[str, Any],
) -> tuple[datetime | None, datetime | None]:
    start = _parse_iso(manual.get("started_at") or manual.get("date"))
    finish = _parse_iso(manual.get("finished_at"))
    if start is None:
        start = _parse_iso(summary.get("started_at"))
    if finish is None:
        finish = _parse_iso(summary.get("finished_at") or summary.get("updated_at"))
    rllib_times = _rllib_start_times(run_dir)
    if start is None and rllib_times:
        start = rllib_times[0]
    if finish is None:
        for candidate in (
            run_dir / "summary.json",
            run_dir / "checkpoint_last_meta.json",
            run_dir,
        ):
            finish = _mtime_dt(candidate)
            if finish is not None:
                break
    if start is None:
        start = _mtime_dt(run_dir)
    return start, finish


def _infer_elapsed(
    run_dir: Path,
    summary: dict[str, Any],
    manual: dict[str, Any],
    start: datetime | None,
    finish: datetime | None,
) -> float | None:
    manual_elapsed = _safe_float(manual.get("elapsed_wall_time_s"))
    if manual_elapsed is not None:
        return manual_elapsed
    summary_elapsed = _safe_float(summary.get("elapsed_wall_time_s"))
    inferred_elapsed: float | None = None
    if start is not None and finish is not None and finish >= start:
        inferred_elapsed = (finish - start).total_seconds()
    if summary_elapsed is None:
        return inferred_elapsed
    # Supervisor resume runs can leave child-only elapsed times. Prefer the
    # wider artifact window when the summary value is implausibly small.
    if (
        inferred_elapsed is not None
        and inferred_elapsed > summary_elapsed
        and summary_elapsed < 300.0
    ):
        return inferred_elapsed
    return summary_elapsed


def _infer_platform(
    summary: dict[str, Any],
    manual: dict[str, Any],
) -> str:
    if manual.get("platform"):
        return str(manual["platform"])
    if summary.get("platform"):
        return str(summary["platform"])
    haystack = " ".join(
        str(summary.get(key, ""))
        for key in ("output_dir", "rllib_log_root", "checkpoint_best")
    )
    if re.search(r"[A-Za-z]:\\|[A-Za-z]:/", haystack):
        return "win"
    if "/Users/" in haystack or "/opt/anaconda" in haystack:
        return "mac"
    return "unknown"


def _status(summary: dict[str, Any]) -> str:
    if not summary:
        return "missing-summary"
    if summary.get("ok") is True:
        return "completed"
    if summary.get("interrupted") is True:
        return "interrupted"
    return str(summary.get("stop_reason") or "incomplete")


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, dict):
        return [f"{key}: {item}" for key, item in value.items()]
    return [value]


def _compact_config(cfg: dict[str, Any]) -> dict[str, Any]:
    model = cfg.get("model") if isinstance(cfg.get("model"), dict) else {}
    ppo = cfg.get("ppo") if isinstance(cfg.get("ppo"), dict) else {}
    sim = cfg.get("simulation") if isinstance(cfg.get("simulation"), dict) else {}
    grf = cfg.get("grf") if isinstance(cfg.get("grf"), dict) else {}
    reward = cfg.get("reward") if isinstance(cfg.get("reward"), dict) else {}
    parallelism = (
        cfg.get("parallelism") if isinstance(cfg.get("parallelism"), dict) else {}
    )
    return {
        "model": {
            "layers": model.get("num_hidden_layers"),
            "hidden": model.get("dim_hidden_layers"),
            "activation": model.get("fcnet_activation"),
            "asymmetric_actor_critic": model.get("asymmetric_actor_critic"),
            "seed": model.get("seed"),
        },
        "ppo": {
            "batch": ppo.get("train_batch_size"),
            "minibatch": ppo.get("minibatch_size"),
            "epochs": ppo.get("num_epochs"),
            "lr": ppo.get("lr"),
            "gamma": ppo.get("gamma"),
            "lam": ppo.get("lam"),
            "clip": ppo.get("clip_param"),
            "vf_clip": ppo.get("vf_clip_param"),
            "vf_loss_coeff": ppo.get("vf_loss_coeff"),
        },
        "simulation": {
            "setup": sim.get("setup_xml"),
            "segment_s": sim.get("segment_duration"),
            "episode_s": sim.get("episode_duration"),
            "iterations_target": sim.get("iterations"),
            "policy_knots": sim.get("policy_knots"),
            "action_mode": sim.get("action_mode"),
            "ref_model": sim.get("pros_ref_model"),
            "ref_cutoff_hz": sim.get("pros_ref_cutoff_hz"),
            "episode_start_offset_s": sim.get("episode_start_offset_s"),
            "random_init": sim.get("random_init"),
            "init_to_target": sim.get("imitation_initialize_to_target"),
        },
        "grf": {
            "mode": grf.get("grf_mode"),
            "profile": grf.get("online_grf_profile"),
            "observation": grf.get("online_grf_observation"),
            "applied_side": grf.get("online_grf_applied_side"),
            "disabled_prescribed_side": grf.get("disable_prescribed_grf_side"),
        },
        "reward": {
            "mode": reward.get("reward_mode"),
            "imitation_weight": reward.get("imitation_weight"),
            "served_imitation_weight": reward.get("served_imitation_weight"),
            "knee_pos": reward.get("imitation_knee_position_weight"),
            "ankle_pos": reward.get("imitation_ankle_position_weight"),
            "knee_vel": reward.get("imitation_knee_velocity_weight"),
            "ankle_vel": reward.get("imitation_ankle_velocity_weight"),
            "blend_served": reward.get("blend_served_imitation"),
            "blend_actual": reward.get("blend_imitation"),
            "smoothness": reward.get("smoothness_weight"),
            "command_rate": reward.get("command_rate_weight"),
        },
        "parallelism": {
            "env_runners": parallelism.get("num_env_runners"),
            "ray_cpus": parallelism.get("ray_num_cpus"),
        },
    }


def _config_sentence(compact: dict[str, Any]) -> str:
    def value_text(value: Any) -> str:
        if value is None:
            return "-"
        if isinstance(value, list):
            return ",".join(str(item) for item in value) if value else "-"
        return str(value)

    model = compact["model"]
    ppo = compact["ppo"]
    sim = compact["simulation"]
    grf = compact["grf"]
    reward = compact["reward"]
    parallelism = compact["parallelism"]
    parts = [
        (
            f"model {value_text(model.get('layers'))}x"
            f"{value_text(model.get('hidden'))} {value_text(model.get('activation'))}"
        ),
        (
            f"PPO batch={value_text(ppo.get('batch'))}, "
            f"mini={value_text(ppo.get('minibatch'))}, "
            f"epochs={value_text(ppo.get('epochs'))}, "
            f"gamma={value_text(ppo.get('gamma'))}, "
            f"lam={value_text(ppo.get('lam'))}"
        ),
        (
            f"sim segment={value_text(sim.get('segment_s'))}s, "
            f"episode={value_text(sim.get('episode_s'))}s, "
            f"target_iter={value_text(sim.get('iterations_target'))}, "
            f"action={value_text(sim.get('action_mode'))}, "
            f"knots={value_text(sim.get('policy_knots'))}"
        ),
        (
            f"ref={value_text(sim.get('ref_model'))}@"
            f"{value_text(sim.get('ref_cutoff_hz'))}Hz, "
            f"start_offset={value_text(sim.get('episode_start_offset_s'))}s"
        ),
        (
            f"GRF={value_text(grf.get('mode'))}, "
            f"applied={value_text(grf.get('applied_side'))}, "
            f"obs={value_text(grf.get('observation'))}"
        ),
        (
            f"reward={value_text(reward.get('mode'))}, "
            f"imitation={value_text(reward.get('imitation_weight'))}, "
            f"served={value_text(reward.get('served_imitation_weight'))}, "
            f"ankle_pos={value_text(reward.get('ankle_pos'))}"
        ),
        (
            f"parallel env_runners={value_text(parallelism.get('env_runners'))}, "
            f"ray_cpus={value_text(parallelism.get('ray_cpus'))}"
        ),
    ]
    return "; ".join(parts)


def _extract_run(run_dir: Path, manual_runs: dict[str, Any]) -> dict[str, Any]:
    run_id = run_dir.name
    manual = manual_runs.get(run_id, {})
    if manual is None:
        manual = {}
    if not isinstance(manual, dict):
        raise ValueError(f"manual override for run {run_id!r} must be a mapping")
    summary = _read_json_dict(run_dir / "summary.json")
    cfg = _load_resolved_cfg(run_dir / "training_cfg.resolved.yaml")
    history = _read_iteration_history(run_dir / "train_iterations.jsonl")
    first_iter, last_iter = _first_last_iterations(history)
    best_iter, best_return_history = _best_iteration(history)
    start, finish = _infer_start_finish(run_dir, summary, manual)
    elapsed = _infer_elapsed(run_dir, summary, manual, start, finish)
    compact = _compact_config(cfg)
    status = _status(summary)
    best_return = _safe_float(manual.get("best_return"))
    if best_return is None:
        best_return = _safe_float(summary.get("best_episode_return_mean"))
    if best_return is None:
        best_return = best_return_history
    if manual.get("best_iteration") is not None:
        best_iter = _safe_int(manual.get("best_iteration"))
    checkpoint_best_meta = _clean_paths_in_mapping(
        _read_json_dict(run_dir / "checkpoint_best_meta.json")
    )
    checkpoint_last_meta = _clean_paths_in_mapping(
        _read_json_dict(run_dir / "checkpoint_last_meta.json")
    )
    lineage = str(manual.get("lineage") or run_id)
    notes = [str(item) for item in _as_list(manual.get("notes"))]
    if status != "completed" and not notes:
        notes.append("Incomplete run; keep for provenance, not as a best baseline.")
    return {
        "id": run_id,
        "title": str(manual.get("title") or run_id),
        "path": _repo_rel(run_dir),
        "status": status,
        "ok": bool(summary.get("ok")) if summary else False,
        "platform": _infer_platform(summary, manual),
        "started_at": _iso(start),
        "finished_at": _iso(finish),
        "elapsed_wall_time_s": elapsed,
        "elapsed": _format_duration(elapsed),
        "iterations": {
            "first": first_iter,
            "last": last_iter,
            "count": len(history) if history else _safe_int(summary.get("iterations_run")),
            "target": compact["simulation"].get("iterations_target")
            or _safe_int(summary.get("iterations_run")),
            "completed_summary": _safe_int(summary.get("iterations_completed")),
            "start_summary": _safe_int(summary.get("iteration_start")),
        },
        "best": {
            "iteration": best_iter,
            "episode_return_mean": best_return,
        },
        "resume_from": _clean_path_value(summary.get("resume_from")),
        "supervisor_resume_from": _clean_path_value(summary.get("supervisor_resume_from")),
        "parent_run": manual.get("parent_run"),
        "lineage": lineage,
        "lineage_order": _safe_float(manual.get("lineage_order")),
        "notes": notes,
        "report_refs": [str(item) for item in _as_list(manual.get("report_refs"))],
        "config_highlights": compact,
        "config_sentence": _config_sentence(compact),
        "config_path": (
            _repo_rel(run_dir / "training_cfg.resolved.yaml")
            if (run_dir / "training_cfg.resolved.yaml").is_file()
            else None
        ),
        "summary_path": (
            _repo_rel(run_dir / "summary.json")
            if (run_dir / "summary.json").is_file()
            else None
        ),
        "checkpoint_best_meta": checkpoint_best_meta,
        "checkpoint_last_meta": checkpoint_last_meta,
        "raw_config": cfg,
    }


def _sort_key(run: dict[str, Any]) -> tuple[str, float, str]:
    started = run.get("started_at") or "9999-99-99 99:99:99"
    order = run.get("lineage_order")
    if order is None:
        order = 9999.0
    return (str(started), float(order), str(run.get("id")))


def _group_lineages(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for run in runs:
        groups.setdefault(str(run["lineage"]), []).append(run)
    output: list[dict[str, Any]] = []
    for lineage, items in groups.items():
        ordered = sorted(items, key=_sort_key)
        output.append(
            {
                "id": lineage,
                "runs": [item["id"] for item in ordered],
                "started_at": ordered[0].get("started_at"),
                "best_episode_return_mean": max(
                    (
                        _safe_float(item.get("best", {}).get("episode_return_mean"))
                        for item in ordered
                    ),
                    default=None,
                ),
            }
        )
    return sorted(output, key=lambda item: str(item.get("started_at") or ""))


def _markdown_table(rows: list[list[str]]) -> str:
    if not rows:
        return ""
    widths = [0 for _ in rows[0]]
    for row in rows:
        for index, cell in enumerate(row):
            widths[index] = max(widths[index], len(cell))
    lines: list[str] = []
    for row_index, row in enumerate(rows):
        line = "| " + " | ".join(
            cell.ljust(widths[index]) for index, cell in enumerate(row)
        ) + " |"
        lines.append(line)
        if row_index == 0:
            lines.append("| " + " | ".join("-" * width for width in widths) + " |")
    return "\n".join(lines)


def _md_link(path: str | None, label: str) -> str:
    if not path:
        return "-"
    return f"[{label}]({path.replace(' ', '%20')})"


def _iteration_text(run: dict[str, Any]) -> str:
    info = run["iterations"]
    first = info.get("first")
    last = info.get("last")
    count = info.get("count")
    target = info.get("target")
    if first is not None and last is not None:
        base = f"{first}-{last}"
    elif last is not None:
        base = str(last)
    else:
        base = "-"
    if count is not None:
        base += f" ({count})"
    if target is not None:
        base += f" / target {target}"
    return base


def _run_refs(run: dict[str, Any]) -> str:
    pieces = []
    if run.get("summary_path"):
        pieces.append(_md_link(run.get("summary_path"), "summary"))
    if run.get("config_path"):
        pieces.append(_md_link(run.get("config_path"), "config"))
    return ", ".join(pieces) if pieces else "-"


def _render_markdown(
    runs: list[dict[str, Any]],
    lineages: list[dict[str, Any]],
    report_only: list[dict[str, Any]],
    *,
    generated_at: str,
) -> str:
    lines: list[str] = [
        "# Historical Runs",
        "",
        "Generated by `Trajectory Generator/baseline_MLP/update_historical_runs.py`.",
        "Do not edit `historical_runs.index.json` directly; put human corrections in `historical_runs.manual.yaml`.",
        "",
        f"Last generated: `{generated_at}`",
        "",
        "## Overview",
        "",
    ]
    table_rows = [
        [
            "Date",
            "Platform",
            "Run",
            "Lineage",
            "Status",
            "Iterations",
            "Best return",
            "Elapsed",
        ]
    ]
    for run in sorted(runs, key=_sort_key):
        best = run.get("best", {})
        best_text = _format_number(best.get("episode_return_mean"))
        if best.get("iteration") is not None:
            best_text += f" @ {best['iteration']}"
        table_rows.append(
            [
                str(run.get("started_at") or "-"),
                str(run.get("platform") or "-"),
                str(run.get("id")),
                str(run.get("lineage")),
                str(run.get("status")),
                _iteration_text(run),
                best_text,
                str(run.get("elapsed") or "-"),
            ]
        )
    lines.append(_markdown_table(table_rows))
    lines.extend(["", "## Lineages", ""])
    run_by_id = {run["id"]: run for run in runs}
    for lineage in lineages:
        lineage_runs = [run_by_id[run_id] for run_id in lineage["runs"] if run_id in run_by_id]
        chain = " -> ".join(run["id"] for run in lineage_runs)
        lines.extend([f"### {lineage['id']}", "", f"Chain: `{chain}`", ""])
        for run in lineage_runs:
            best = run.get("best", {})
            best_text = _format_number(best.get("episode_return_mean"))
            if best.get("iteration") is not None:
                best_text += f" at iteration {best['iteration']}"
            lines.extend(
                [
                    f"#### {run['id']}",
                    "",
                    f"- Date: `{run.get('started_at') or '-'}`",
                    f"- Platform: `{run.get('platform') or '-'}`",
                    f"- Status: `{run.get('status')}`",
                    f"- Iterations: `{_iteration_text(run)}`",
                    f"- Time elapsed: `{run.get('elapsed') or '-'}`",
                    f"- Best return: `{best_text}`",
                    f"- Resume source: `{run.get('resume_from') or '-'}`",
                    f"- Files: {_run_refs(run)}",
                    f"- Config: {run.get('config_sentence')}",
                ]
            )
            notes = run.get("notes") or []
            if notes:
                lines.append("- Notes: " + " ".join(str(item) for item in notes))
            reports = run.get("report_refs") or []
            if reports:
                lines.append("- Reports: " + ", ".join(f"`{item}`" for item in reports))
            lines.append("")
    if report_only:
        lines.extend(["## Historical Notes From Reports", ""])
        report_rows = [["Date", "ID", "Platform", "Summary", "Reports"]]
        for item in report_only:
            reports = ", ".join(str(ref) for ref in _as_list(item.get("report_refs")))
            report_rows.append(
                [
                    str(item.get("date") or "-"),
                    str(item.get("id") or "-"),
                    str(item.get("platform") or "unknown"),
                    str(item.get("summary") or item.get("title") or "-"),
                    reports or "-",
                ]
            )
        lines.append(_markdown_table(report_rows))
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def _normalise_manual_report_only(manual: dict[str, Any]) -> list[dict[str, Any]]:
    entries = manual.get("report_only") or manual.get("historical_notes") or []
    if not isinstance(entries, list):
        raise ValueError("manual report_only must be a list")
    result: list[dict[str, Any]] = []
    for item in entries:
        if not isinstance(item, dict):
            raise ValueError("each report_only entry must be a mapping")
        result.append(dict(item))
    return result


def update_history(
    *,
    runs_root: Path = RUNS_ROOT,
    training_root: Path | None = None,
    manual_path: Path = DEFAULT_MANUAL_PATH,
    markdown_path: Path = DEFAULT_MARKDOWN_PATH,
    index_path: Path = DEFAULT_INDEX_PATH,
) -> dict[str, Any]:
    training_root = training_root or (runs_root / "training")
    manual = _read_yaml_dict(manual_path)
    manual_runs = manual.get("runs") or {}
    if not isinstance(manual_runs, dict):
        raise ValueError("manual runs must be a mapping")
    run_dirs = []
    if training_root.is_dir():
        run_dirs = sorted(
            [path for path in training_root.iterdir() if path.is_dir()],
            key=lambda path: path.name.lower(),
        )
    runs = [_extract_run(path, manual_runs) for path in run_dirs]
    lineages = _group_lineages(runs)
    report_only = _normalise_manual_report_only(manual)
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    index = {
        "schema_version": 1,
        "generated_at": generated_at,
        "platform_generated_on": _platform_tag(),
        "runs_root": _repo_rel(runs_root),
        "training_root": _repo_rel(training_root),
        "manual_path": _repo_rel(manual_path),
        "markdown_path": _repo_rel(markdown_path),
        "runs": runs,
        "lineages": lineages,
        "report_only": report_only,
    }
    markdown = _render_markdown(
        runs,
        lineages,
        report_only,
        generated_at=generated_at,
    )
    _write_json_atomic(index_path, index)
    _write_text_atomic(markdown_path, markdown)
    return index


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-root", type=Path, default=RUNS_ROOT)
    parser.add_argument("--training-root", type=Path, default=None)
    parser.add_argument("--manual", type=Path, default=DEFAULT_MANUAL_PATH)
    parser.add_argument("--markdown", type=Path, default=DEFAULT_MARKDOWN_PATH)
    parser.add_argument("--index", type=Path, default=DEFAULT_INDEX_PATH)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    index = update_history(
        runs_root=args.runs_root,
        training_root=args.training_root,
        manual_path=args.manual,
        markdown_path=args.markdown,
        index_path=args.index,
    )
    if not args.quiet:
        print(
            "Historical runs updated: "
            f"{index['markdown_path']} ({len(index['runs'])} run(s), "
            f"{len(index['report_only'])} report-only note(s))"
        )


if __name__ == "__main__":
    main()
