"""Collect the finalized paired diagnostic, preserving all native artifacts."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def local_file(raw):
    path = Path(str(raw).replace("\\", "/"))
    if path.is_file():
        return path
    if not path.is_absolute() and (REPO / path).is_file():
        return REPO / path
    for marker in ("/Trajectory Generator/", "/models/", "/validation/", "/online_grf_profiles/"):
        if marker in path.as_posix():
            candidate = REPO / marker.strip("/") / path.as_posix().split(marker, 1)[1]
            if candidate.is_file():
                return candidate
    candidate = REPO / path.name
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(raw)


def fingerprint(path):
    path = path.resolve()
    return {"path": path.relative_to(REPO).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    out = AUDIT / "result_summary.json"
    if out.exists():
        raise FileExistsError("Final summary already exists; preserve it")
    protocol_path = AUDIT / "protocol.json"
    comparison_path = AUDIT / "comparison_metrics.json"
    protocol, comparison = read(protocol_path), read(comparison_path)
    native_manifest_path = AUDIT / "postprocessing_native_manifest.json"
    native_manifest = read(native_manifest_path)
    for files in native_manifest["subjects"].values():
        for label, expected in files.items():
            path = REPO / label
            if path.stat().st_size != expected["size_bytes"] or fingerprint(path)["sha256"] != expected["sha256"]:
                raise RuntimeError("A native artifact changed during postprocessing: " + label)
    protected = []
    for item in protocol["protected_inputs"]:
        actual = fingerprint(local_file(item["path"]))
        protected.append({**actual, "unchanged": actual["sha256"] == item["sha256"]})
    if not all(item["unchanged"] for item in protected):
        raise RuntimeError("A frozen input changed")
    artifacts = [Path(__file__), protocol_path, comparison_path, AUDIT / "selection.json", native_manifest_path]
    provenance_path = AUDIT / "numerical_review_provenance.json"
    provenance = read(provenance_path)
    artifacts.extend([provenance_path, AUDIT / "plugin_binary_observation_during_run.json", AUDIT / "plugin_binary_post_run_recheck.json", AUDIT / "scientific_review.json"])
    subjects = {}
    for subject in ("AB06", "AB07"):
        run = REPO / "results" / f"{subject}_paired_prescribed_HS_9s_2026_10_07"
        numerical_path = AUDIT / subject / "numerical_verification.json"
        numerical = read(numerical_path)
        receipt = read(run / "receipt.json")
        if numerical["receipt"]["sha256"] != fingerprint(run / "receipt.json")["sha256"]:
            raise RuntimeError("Numerical audit refers to a different native receipt")
        physical = comparison["results"][subject]
        plot_root = REPO / "plot" / f"{subject}_paired_prescribed_HS_9s_2026_10_07"
        manifests = list(plot_root.glob("*/plot_rendering_manifest.json"))
        if len(manifests) != 1:
            raise ValueError("Require the single explicit plot batch for this run")
        plot_path, plot_manifest = manifests[0], read(manifests[0])
        if plot_manifest["png_count"] != 7 or len(plot_manifest["files"]) != 7 or not plot_manifest["run_files_unchanged_after_plotting"]["passed"]:
            raise ValueError("Plot count or native artifact integrity failed")
        visual = read(plot_path.parent / "visual_review.json")
        if not visual["all_passed"]:
            raise ValueError("Plot visual review is incomplete")
        figures = []
        for filename, entry in plot_manifest["files"].items():
            observed = fingerprint(plot_path.parent / filename)
            if observed["sha256"] != entry["sha256"]:
                raise RuntimeError("A canonical figure changed after rendering")
            figures.append(observed)
        events = physical.get("event_channels", {}).get("primary", {}).get("binary_geometry_FSM", {})
        subjects[subject] = {
            "run": run.relative_to(REPO).as_posix(),
            "receipt_status": receipt["status"],
            "runtime_completed_requested_duration": bool(receipt.get("completed_requested_duration", False)),
            "physical_clocks_reached_requested_end": numerical["summary"]["clocks_at_requested_end"],
            "recorded_physical_rows": numerical["summary"]["recorded_output_rows"],
            "runtime_error": receipt.get("error"),
            "final_boundary_diagnosis": numerical.get("final_boundary_diagnosis"),
            "duration_s": receipt["actual_episode_end_s"] - receipt["actual_episode_start_s"],
            "steps": receipt["steps"], "wall_time_s": receipt["wall_time_s"],
            "physical_clocks_s": {"environment": receipt.get("final_time_s"), "runner": receipt.get("final_runner_time_s"), "state": receipt.get("final_state_time_s")},
            "primary_nominal_cycles_observed": physical["fully_observed_primary_cycle_count"],
            "primary_aggregate": physical.get("primary_aggregate"),
            "primary_per_cycle": physical["primary_cycle_metrics"],
            "primary_binary_onset": {name: value["channels"]["onset"]["matching"] for name, value in events.items()},
            "startup": physical.get("startup"),
            "runtime_FSM_secondary": physical.get("full_episode_secondary_journal", {}).get("last_counters"),
            "numerical_summary": numerical["summary"],
            "high_rate_contact": numerical["high_rate_contact"],
            "SEA": numerical["SEA"],
            "MTP_separate": numerical["MTP_separate"],
            "reserves_and_recruitment": numerical["reserves_and_recruitment"],
            "canonical_plot_folder": plot_path.parent.relative_to(REPO).as_posix(),
            "canonical_figures": figures, "visual_review": visual,
        }
        artifacts.extend([numerical_path, run / "receipt.json", plot_path, plot_path.parent / "visual_review.json"])
    configuration = comparison["configuration_comparability"]
    settings_equal = (not configuration["effective_env_unexpected_differences"]
                      and not configuration["effective_simulator_unexpected_differences"]
                      and configuration["profile_association_exactly_equal"]
                      and configuration["SEA_XML_semantic_comparison"]["semantic_equal"]
                      and provenance["all_SEA_properties_equal_or_within_serialization"])
    primary_complete = all(item["primary_nominal_cycles_observed"] == 3 for item in subjects.values())
    runtime_complete = all(item["runtime_completed_requested_duration"] for item in subjects.values())
    expected_terminal_error = (
        subjects["AB06"]["receipt_status"] == "failed"
        and subjects["AB06"]["runtime_error"] == "ValueError: Policy boundaries must span a positive integer number of samples."
        and (subjects["AB06"]["final_boundary_diagnosis"] or {}).get("applicable", False)
        and subjects["AB06"]["physical_clocks_reached_requested_end"]
        and subjects["AB07"]["runtime_completed_requested_duration"]
    )
    status = "paired_diagnostic_with_limitations"
    if settings_equal and primary_complete:
        if runtime_complete:
            status = "paired_diagnostic_complete"
        elif expected_terminal_error:
            status = "paired_physical_comparison_complete_with_terminal_adapter_failure"
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "both_native_runtimes_completed_normally": runtime_complete,
        "network_loaded": False, "physical_controller_settings_equal": settings_equal,
        "configuration_comparability": configuration,
        "primary_protocol": "Three measured-GRF nominal cycles per subject after one separate startup cycle; reset at a preselected measured HS, nine-second runtime, native one-millisecond force samples.",
        "impulse_percent_reference": "100 * (sum simulated left vertical impulse / sum measured left vertical impulse - 1), integrated over the exact same three nominal cycles for each subject.",
        "timing_reference": "Binary geometry onset versus measured left-GRF 20 N events: four unique HS boundaries and three TO. Missing/extra events and matching tolerances remain in the matching objects.",
        "subjects": subjects,
        "protected_inputs": {"all_unchanged": True, "count": len(protected), "checks": protected},
        "native_artifacts_unchanged_by_postprocessing": {"all_unchanged": True, "count": sum(len(files) for files in native_manifest["subjects"].values()), "manifest": fingerprint(native_manifest_path)},
        "artifact_fingerprints": [fingerprint(path) for path in artifacts],
        "interpretation_limits": [
            "This comparison concerns two IK-driven subject models and their experimental data. It does not evaluate a learned policy.",
            "Subject anatomy, mass, kinematics and measured right loading differ together; the result does not identify an isolated anatomical cause.",
            "Numerical completion and coherent SEA identities do not establish physiological validity; contact, reserves and MTP drift remain explicit diagnostics.",
        ],
    }
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"summary": str(out.relative_to(REPO)), "status": result["status"]}))


if __name__ == "__main__":
    main()
