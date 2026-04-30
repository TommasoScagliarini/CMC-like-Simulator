"""
Evaluate candidate OpenSim models in the CMC-like simulator.

The pipeline RRA score is useful, but the simulator can still expose practical
issues such as SEA saturation, infeasible static optimisation, or very large
reserve torques. This script runs short identical CMC-like trials for several
candidate models and writes a compact CSV/Markdown ranking.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, asdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PIPELINE_ROOT = Path.home() / "Desktop" / "Opensim OMNIBUS" / "model_pipeline_dir"
EVALUATION_ROOT = PIPELINE_ROOT / "06_cmc_like_evaluation"

KINEMATICS = PIPELINE_ROOT / "05_cmc_like_ready" / "data" / "IK_rra_window.mot"
EXTERNAL_LOADS = PIPELINE_ROOT / "05_cmc_like_ready" / "data" / "ExternalForces.xml"
RESERVE_ACTUATORS = PIPELINE_ROOT / "05_cmc_like_ready" / "data" / "CMC_Actuators.xml"

DEFAULT_T_START = 18.965
DEFAULT_DURATION = 0.10
DEFAULT_WORKERS = 4


@dataclass(frozen=True)
class CandidateSpec:
    name: str
    model: Path


@dataclass
class CandidateResult:
    name: str
    model: str
    output_dir: str
    returncode: int
    status: str
    elapsed_s: float
    warnings_total: int = 0
    errors_total: int = 0
    exceptions_total: int = 0
    nonfinite_total: int = 0
    qp_fallback_total: int = 0
    so_feasibility_warnings: int = 0
    muscle_min_fiber_warnings: int = 0
    locked_coord_warnings: int = 0
    tau_reserve_norm_mean: float = math.nan
    tau_reserve_norm_max: float = math.nan
    muscle_share_mean: float = math.nan
    muscle_capable_share_mean: float = math.nan
    muscle_capable_reserve_norm_max: float = math.nan
    unactuated_reserve_norm_max: float = math.nan
    so_residual_norm_max: float = math.nan
    equilibrium_failures_max: float = math.nan
    reserve_torque_max_abs: float = math.nan
    pelvis_reserve_torque_max_abs: float = math.nan
    joint_reserve_torque_max_abs: float = math.nan
    sea_knee_saturation_frames: int = 0
    sea_ankle_saturation_frames: int = 0
    sea_knee_feasibility_min: float = math.nan
    sea_ankle_feasibility_min: float = math.nan
    score: float = math.inf
    notes: str = ""


def default_candidates() -> list[CandidateSpec]:
    candidates = [
        CandidateSpec(
            "pipeline_ready_pelvis",
            PIPELINE_ROOT
            / "05_cmc_like_ready"
            / "Adjusted_newmarkers_pipeline_ready.osim",
        ),
        CandidateSpec(
            "pipeline_scaled_no_com",
            PIPELINE_ROOT / "01_scaling" / "scaled_SEASEA.osim",
        ),
        CandidateSpec(
            "manual_newmarkers_scaled",
            Path.home() / "Downloads" / "Archive" / "newmarkersScaled.osim",
        ),
        CandidateSpec(
            "manual_adjusted_newmarkers",
            Path.home()
            / "Downloads"
            / "Archive"
            / "TODO"
            / "Adjusted_newmarkers.osim",
        ),
        CandidateSpec(
            "manual_restricted2",
            Path.home()
            / "Downloads"
            / "Archive"
            / "TODO"
            / "Adjusted_newmarkers_resctricted2.osim",
        ),
        CandidateSpec(
            "manual_restricted_pelvis",
            Path.home()
            / "Downloads"
            / "Archive"
            / "TODO"
            / "Adjusted_newmarkers_restricted_pelvis.osim",
        ),
    ]
    return [candidate for candidate in candidates if candidate.model.is_file()]


def parse_sto(path: Path) -> tuple[list[str], list[dict[str, float]]]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    try:
        header_index = lines.index("endheader") + 1
    except ValueError:
        raise ValueError(f"Not an OpenSim STO with endheader: {path}")
    header = lines[header_index].split("\t")
    rows: list[dict[str, float]] = []
    for line in lines[header_index + 1 :]:
        if not line.strip():
            continue
        values = [float(item) for item in line.split("\t")]
        rows.append(dict(zip(header, values)))
    return header, rows


def finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def mean(values: list[float]) -> float:
    values = finite(values)
    return sum(values) / len(values) if values else math.nan


def max_or_nan(values: list[float]) -> float:
    values = finite(values)
    return max(values) if values else math.nan


def metric_column(rows: list[dict[str, float]], column: str) -> list[float]:
    return [row[column] for row in rows if column in row]


def count_pattern(text: str, pattern: str) -> int:
    return len(re.findall(pattern, text, flags=re.IGNORECASE))


def analyze_output(result: CandidateResult) -> CandidateResult:
    output_dir = Path(result.output_dir)
    logs = [output_dir / "stdout.log", *output_dir.glob("*phase3_log_*.txt")]
    text = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for path in logs
        if path.is_file()
    )
    result.warnings_total = count_pattern(text, r"\bwarning\b|RuntimeWarning")
    result.errors_total = count_pattern(text, r"\berror\b")
    result.exceptions_total = count_pattern(text, r"exception|traceback")
    result.nonfinite_total = count_pattern(text, r"non-finite|\bnan\b|\binf\b")
    result.qp_fallback_total = count_pattern(text, r"did not converge|bounded least-squares fallback")
    result.so_feasibility_warnings = count_pattern(text, r"SO feasibility backtracking did not meet")
    result.muscle_min_fiber_warnings = count_pattern(text, r"minimum fiber length")
    result.locked_coord_warnings = count_pattern(text, r"coordinate .* is locked")

    status_file = output_dir / "sim_output_run_status.txt"
    if status_file.is_file():
        for line in status_file.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("status="):
                result.status = line.split("=", 1)[1].strip()
                break

    recruitment = output_dir / "sim_output_recruitment.sto"
    if recruitment.is_file():
        _, rows = parse_sto(recruitment)
        result.tau_reserve_norm_mean = mean(metric_column(rows, "tau_reserve_norm"))
        result.tau_reserve_norm_max = max_or_nan(metric_column(rows, "tau_reserve_norm"))
        result.muscle_share_mean = mean(metric_column(rows, "muscle_share"))
        result.muscle_capable_share_mean = mean(metric_column(rows, "muscle_capable_share"))
        result.muscle_capable_reserve_norm_max = max_or_nan(
            metric_column(rows, "muscle_capable_reserve_norm")
        )
        result.unactuated_reserve_norm_max = max_or_nan(
            metric_column(rows, "unactuated_reserve_norm")
        )
        result.so_residual_norm_max = max_or_nan(metric_column(rows, "residual_norm"))
        result.equilibrium_failures_max = max_or_nan(
            metric_column(rows, "equilibrium_failures")
        )

    reserve_torques = output_dir / "sim_output_reserve_torques.sto"
    if reserve_torques.is_file():
        header, rows = parse_sto(reserve_torques)
        all_max: list[float] = []
        pelvis_max: list[float] = []
        joint_max: list[float] = []
        for column in header[1:]:
            values = [abs(row[column]) for row in rows]
            column_max = max_or_nan(values)
            all_max.append(column_max)
            if column.startswith("pelvis_"):
                pelvis_max.append(column_max)
            else:
                joint_max.append(column_max)
        result.reserve_torque_max_abs = max_or_nan(all_max)
        result.pelvis_reserve_torque_max_abs = max_or_nan(pelvis_max)
        result.joint_reserve_torque_max_abs = max_or_nan(joint_max)

    sea_diag = output_dir / "sim_output_sea_diagnostics.sto"
    if sea_diag.is_file():
        _, rows = parse_sto(sea_diag)
        knee_sat = metric_column(rows, "SEA_Knee_tau_input_saturated")
        ankle_sat = metric_column(rows, "SEA_Ankle_tau_input_saturated")
        knee_scale = metric_column(rows, "SEA_Knee_sea_feasibility_scale")
        ankle_scale = metric_column(rows, "SEA_Ankle_sea_feasibility_scale")
        result.sea_knee_saturation_frames = sum(1 for value in knee_sat if value > 0.5)
        result.sea_ankle_saturation_frames = sum(1 for value in ankle_sat if value > 0.5)
        result.sea_knee_feasibility_min = min(finite(knee_scale)) if finite(knee_scale) else math.nan
        result.sea_ankle_feasibility_min = min(finite(ankle_scale)) if finite(ankle_scale) else math.nan

    failure_penalty = 1_000_000.0 if result.returncode != 0 or result.status != "complete" else 0.0
    result.score = (
        failure_penalty
        + 2.0 * clean(result.tau_reserve_norm_mean)
        + 0.5 * clean(result.reserve_torque_max_abs)
        + 3.0 * clean(result.muscle_capable_reserve_norm_max)
        + 50.0 * (result.sea_knee_saturation_frames + result.sea_ankle_saturation_frames)
        + 100.0 * result.so_feasibility_warnings
        + 10.0 * result.qp_fallback_total
        + 5.0 * result.nonfinite_total
    )
    return result


def clean(value: float) -> float:
    return value if math.isfinite(value) else 100_000.0


def run_candidate(args: tuple[CandidateSpec, Path, float, float, bool]) -> CandidateResult:
    candidate, evaluation_root, t_start, t_end, skip_existing = args
    output_dir = evaluation_root / candidate.name
    output_dir.mkdir(parents=True, exist_ok=True)
    stdout_log = output_dir / "stdout.log"
    status_file = output_dir / "sim_output_run_status.txt"

    if skip_existing and status_file.is_file():
        result = CandidateResult(
            name=candidate.name,
            model=str(candidate.model),
            output_dir=str(output_dir),
            returncode=0,
            status="existing",
            elapsed_s=0.0,
            notes="Reused existing output.",
        )
        return analyze_output(result)

    cmd = [
        sys.executable,
        str(REPO_ROOT / "main.py"),
        "--model-bundle",
        str(candidate.model.parent),
        "--model",
        str(candidate.model),
        "--kinematics",
        str(KINEMATICS),
        "--external-loads",
        str(EXTERNAL_LOADS),
        "--reserve-actuators",
        str(RESERVE_ACTUATORS),
        "--t-start",
        f"{t_start:.9g}",
        "--t-end",
        f"{t_end:.9g}",
        "--output-dir",
        str(output_dir),
        "--solver",
        "osqp",
        "--sea-feasibility-scaling",
        "--log",
    ]
    started = time.perf_counter()
    completed = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    elapsed = time.perf_counter() - started
    stdout_log.write_text(completed.stdout, encoding="utf-8", errors="replace")
    result = CandidateResult(
        name=candidate.name,
        model=str(candidate.model),
        output_dir=str(output_dir),
        returncode=completed.returncode,
        status="missing_status",
        elapsed_s=elapsed,
    )
    return analyze_output(result)


def write_summary(results: list[CandidateResult], destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    fields = list(asdict(results[0]).keys()) if results else list(CandidateResult.__dataclass_fields__)
    with destination.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for result in sorted(results, key=lambda item: item.score):
            writer.writerow(asdict(result))


def write_markdown(results: list[CandidateResult], destination: Path, t_start: float, t_end: float) -> None:
    ranked = sorted(results, key=lambda item: item.score)
    lines = [
        "# CMC-like Candidate Evaluation",
        "",
        f"Window: {t_start:.3f} - {t_end:.3f} s",
        "Solver: OSQP",
        "SEA feasibility scaling: enabled",
        "",
    ]
    if ranked:
        best = ranked[0]
        lines.extend(
            [
                f"Best candidate: `{best.name}`",
                f"Model: `{best.model}`",
                f"Score: {best.score:.6g}",
                "",
            ]
        )
    lines.extend(
        [
            "| rank | candidate | status | warnings | SO warn | reserve mean | reserve max | pelvis max | joint max | SEA sat frames | score |",
            "| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for idx, result in enumerate(ranked, start=1):
        sea_sat = result.sea_knee_saturation_frames + result.sea_ankle_saturation_frames
        lines.append(
            "| "
            + " | ".join(
                [
                    str(idx),
                    f"`{result.name}`",
                    result.status,
                    str(result.warnings_total),
                    str(result.so_feasibility_warnings),
                    f"{result.tau_reserve_norm_mean:.3g}",
                    f"{result.reserve_torque_max_abs:.3g}",
                    f"{result.pelvis_reserve_torque_max_abs:.3g}",
                    f"{result.joint_reserve_torque_max_abs:.3g}",
                    str(sea_sat),
                    f"{result.score:.3g}",
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "Notes:",
            "- Lower score is better; it penalizes failed runs, high reserve use, SO feasibility warnings, non-finite values, and SEA saturation.",
            "- A complete run is a stability criterion, not a realism guarantee. Reserve magnitudes remain the main realism limiter.",
        ]
    )
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--t-start", type=float, default=DEFAULT_T_START)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--max-workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--output-root", type=Path, default=EVALUATION_ROOT / "candidate_sweep")
    parser.add_argument(
        "--only",
        action="append",
        default=[],
        help="Evaluate only the named candidate. Can be passed multiple times.",
    )
    parser.add_argument("--skip-existing", action="store_true")
    args = parser.parse_args()

    if not KINEMATICS.is_file():
        raise FileNotFoundError(KINEMATICS)
    if not EXTERNAL_LOADS.is_file():
        raise FileNotFoundError(EXTERNAL_LOADS)
    if not RESERVE_ACTUATORS.is_file():
        raise FileNotFoundError(RESERVE_ACTUATORS)

    t_end = args.t_start + args.duration
    candidates = default_candidates()
    if args.only:
        wanted = set(args.only)
        candidates = [candidate for candidate in candidates if candidate.name in wanted]
        missing = sorted(wanted - {candidate.name for candidate in candidates})
        if missing:
            raise ValueError(f"Unknown or unavailable candidate(s): {', '.join(missing)}")
    if not candidates:
        raise RuntimeError("No candidate models found.")

    args.output_root.mkdir(parents=True, exist_ok=True)
    worker_args = [
        (candidate, args.output_root, args.t_start, t_end, args.skip_existing)
        for candidate in candidates
    ]
    with ProcessPoolExecutor(max_workers=max(1, args.max_workers)) as executor:
        results = list(executor.map(run_candidate, worker_args))

    write_summary(results, args.output_root / "candidate_summary.csv")
    write_markdown(results, args.output_root / "candidate_summary.md", args.t_start, t_end)

    ranked = sorted(results, key=lambda item: item.score)
    print(f"Evaluated {len(results)} candidate(s).")
    print(f"Summary: {args.output_root / 'candidate_summary.md'}")
    if ranked:
        print(f"Best: {ranked[0].name} ({ranked[0].status}, score={ranked[0].score:.6g})")
    return 0 if ranked and ranked[0].status == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
