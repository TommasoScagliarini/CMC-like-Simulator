"""Validate the finished audit and paired evaluations without rerunning physics."""
from __future__ import annotations

import json
import importlib.metadata
import pickle
from pathlib import Path

import numpy as np

from run_audit import HERE, REPO, OUTPUT, verify_manifest, sha256, write_json


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def verify_hashes(base, hashes):
    for name, expected in hashes.items():
        if sha256(Path(base) / name) != expected:
            raise AssertionError(f"Changed artifact: {name}")
    return len(hashes)


def verify_replay_order(replay_dir):
    directories = [Path(replay_dir) / name / "ppo_audit/minibatches"
                   for name in ("control", "corrected_boundary")]
    names = [sorted(path.name for path in folder.glob("*.pkl")) for folder in directories]
    assert names[0] == names[1] and len(names[0]) == 81
    checked = set()
    for name in names[0]:
        batches = []
        for directory in directories:
            with (directory / name).open("rb") as stream:
                batches.append(pickle.load(stream)["batch"])
        assert batches[0].keys() == batches[1].keys()
        for key in batches[0]:
            if key in ("advantages", "value_targets"):
                continue
            assert np.array_equal(batches[0][key], batches[1][key]), (name, key)
            checked.add(key)
    return {"passed": True, "minibatches_checked": len(names[0]),
            "unchanged_columns_at_every_minibatch": sorted(checked),
            "scope": "Same sample order and all non-target inputs in both replay branches"}


def main():
    manifest = verify_manifest()
    finished = read(OUTPUT / "audit_completed.json")
    assert finished["completed"] and finished["calls"] == {"before": 1, "after": 1}
    optimizer = read(OUTPUT / "resume_optimizer_audit.json")
    assert optimizer["passed"] and optimizer["ppo_training_state"]["passed"]
    audit = OUTPUT / "ppo_audit"
    probe = read(audit / "manifest.json")
    assert probe["complete"] and probe["extra_policy_forwards"] == 0
    probe_hashes = verify_hashes(audit, read(audit / "artifact_hashes.json"))
    diagnostics = read(audit / "batch_diagnostics.json")["default_policy"]
    assert diagnostics["rows_real"] == 4096
    assert diagnostics["stored_logp_vs_raw_gaussian_max_abs_error"] < 1e-4
    gae = diagnostics["gae_recomputed"]
    # This reproduces the installed implementation, including its defect.
    # Agreement with stock GAE is a capture-integrity check, not correctness.
    assert gae["post_batch_order_identical"]
    assert gae["targets_max_abs_error"] < 1e-4
    assert gae["advantages_max_abs_error"] < 1e-4
    learner_analysis = read(HERE / "learner_analysis/metrics.json")
    independent_error = learner_analysis["boundaries"]["independent_real_target_max_abs_error"]
    assert independent_error > 50.0
    native = learner_analysis["native_pipeline_bootstrap_reproducer"]
    assert native["unfinished_fragment"]["target_max_abs_difference"] > 3.9
    assert native["time_truncation"]["target_max_abs_difference"] > 3.9
    assert native["true_termination"]["target_max_abs_difference"] > 0.29
    upstream = read(HERE / "upstream/source_verification.json")
    stock_sources = [row for row in upstream["files"] if row["ref"] == "ray-2.55.1"]
    assert len(stock_sources) == 3 and all(row["installed_matches"] for row in stock_sources)
    for row in upstream["files"]:
        assert sha256(HERE / "upstream" / row["ref"] / row["source"]) == row["sha256"]
    ray_root = Path(importlib.metadata.distribution("ray").locate_file("ray"))
    for row in stock_sources:
        assert sha256(ray_root / row["source"]) == row["installed_sha256"]
    replay_dir = HERE / "boundary_replay_v2"
    replay = read(replay_dir / "replay_complete.json")
    assert replay["complete"] and replay["control_reproduction"]["pass"]
    assert replay["control_reproduction"]["minibatches_checked"] == 81
    assert all(replay["unchanged_batch_columns"].values())
    assert not replay["fresh_sampling"] and not replay["ray_initialized"]
    replay_inputs = verify_hashes(Path("/"), read(replay_dir / "input_hashes.json"))
    replay_artifacts = {}
    for branch in ("control", "corrected_boundary"):
        branch_audit = replay_dir / branch / "ppo_audit"
        replay_artifacts[branch] = verify_hashes(branch_audit, read(branch_audit / "artifact_hashes.json"))
    correction = replay["corrected_boundary"]["boundary_correction"]
    assert correction["masked_targets_unchanged"] and correction["extra_value_forwards"] == 0
    order = verify_replay_order(replay_dir)
    write_json(HERE / "replay_order_verification.json", order)
    equivalence = read(HERE / "upstream/real_target_equivalence.json")
    assert equivalence["passed"] and equivalence["real_rows_compared"] == 4096
    verify_hashes(Path("/"), equivalence["input_hashes"])
    ready = read(HERE / "corrected_evaluation_ready.json")
    assert ready["ready"] and ready["control_exact"]
    verify_hashes(Path("/"), ready["input_hashes"])
    evaluations = read(HERE / "evaluation_manifest.json")["cases"]
    assert len(evaluations) == 9
    corrected_manifest = read(HERE / "corrected_evaluation_manifest.json")
    assert len(corrected_manifest["cases"]) == 4
    for group in ("code_hashes", "case_hashes"):
        verify_hashes(Path("/"), corrected_manifest[group])
    evaluations += corrected_manifest["cases"]
    by_role_seed = {}
    receipts = []
    plot_count = 0
    for case in evaluations:
        folder = Path(case["output_dir"])
        completed = read(folder / "evaluation_complete.json")
        verify_hashes(folder, completed["artifact_hashes"])
        summary = read(folder / "rollout_summary.json")
        noise = read(folder / "paired_noise_receipt.json")
        trace = read(folder / "rollout_policy_trace.json")
        assert completed["checkpoint_unchanged"]
        assert len(trace) == len(noise["records"]) == completed["steps"] == summary["steps"]
        verify_hashes(Path("/"), noise["protocol"]["checkpoint_input_hashes"])
        assert noise["protocol"]["sigma_mode"] == "own"
        figures = read(folder / "plot_receipt.json")["figures"]
        assert len(figures) == 7
        verify_hashes(REPO, figures)
        assert all(str(name).startswith("plot/") for name in figures)
        plot_count += len(figures)
        if case["scenario"] == "noisy":
            role = case.get("comparison_role", case["role"])
            by_role_seed[(role, case["noise_seed"])] = noise
            assert (folder / "critic_transitions.npz").is_file()
        receipts.append({"name": case["name"], "steps": completed["steps"],
                         "end_reason": summary["end_reason"],
                         "completed_full_episode": completed["completed_full_episode"]})
    for seed in (123, 125, 127):
        streams = [by_role_seed[(role, seed)]["full_standard_normal_stream_sha256"]
                   for role in ("before", "after", "corrected")]
        assert len(set(streams)) == 1
    analysis_checks = {}
    for name in ("learner_analysis", "replay_comparison"):
        folder = HERE / name
        verification = read(folder / "verification.json")
        assert verification["all_inputs_unchanged"]
        analysis_checks[name] = {
            "inputs": verify_hashes(Path("/"), read(folder / "input_hashes.json")),
            "outputs": verify_hashes(Path("/"), verification["output_hashes"]),
        }
    heldout = read(HERE / "replay_comparison/metrics.json")
    assert heldout["all_requested_heldout_complete"]
    assert sum(bool(row.get("available")) for row in heldout["heldout"]) == 12
    exploration = read(HERE / "exploration/verification.json")
    assert all(exploration["input_hashes_unchanged"].values())
    analysis_checks["exploration"] = {
        "outputs": verify_hashes(REPO, exploration["artifacts_sha256"]),
        "comparison_inputs": verify_hashes(
            Path("/"), read(HERE / "exploration/comparison.json")["input_hashes"]),
        "gaussian_clipping_inputs": verify_hashes(
            Path("/"), read(HERE / "exploration/gaussian_clipping.json")["input_hashes"]),
    }
    for name in ("physical_comparison", "corrected_physical_comparison"):
        folder = HERE / name
        plots = read(folder / "plot_receipt.json")
        analysis_checks[name] = {
            "inputs": verify_hashes(Path("/"), read(folder / "input_hashes.json")),
            "figures": verify_hashes(REPO, plots["figures"]),
            "outputs_and_sources": verify_hashes(Path("/"), read(folder / "output_hashes.json")),
        }
    physical = read(HERE / "physical_comparison/metrics.json")
    assert physical["status"] == "all_requested_cases_finalized" and physical["finalized_case_count"] == 9
    corrected_physical = read(HERE / "corrected_physical_comparison/metrics.json")
    expected_compared_cases = {case["name"] for case in evaluations if case["role"] != "first_pass"}
    assert set(corrected_physical["cases"]) == expected_compared_cases
    assert all(pair.get("comparison") != "pending" for pair in corrected_physical["pairs"].values())
    assert plot_count == 91
    result = {"passed": True, "frozen_inputs_verified": len(manifest["input_hashes"]),
              "probe_artifacts_verified": probe_hashes, "real_training_transitions": 4096,
              "ppo_iterations": 1, "canonical_plots_verified": plot_count,
              "on_policy_collections": 1,
              "same_batch_replay_updates": {"control": 1, "corrected_boundary": 1},
              "control_update_reproduced_bit_exact": True,
              "corrected_and_control_minibatch_inputs_same_order": order,
              "replay_inputs_verified": replay_inputs,
              "replay_artifacts_verified": replay_artifacts,
              "corrected_advantage_sign_change_fraction": correction["real_advantage_sign_change_fraction"],
              "real_target_max_abs_difference_vs_official_ray_fix": equivalence["max_abs_difference_real_targets"],
              "stock_gae_capture_matches": True,
              "stock_gae_mathematically_correct": False,
              "independent_target_max_abs_difference": independent_error,
              "native_connector_defect_reproduced": True,
              "official_ray_source_matches_local_installation": True,
              "derived_analyses_verified": analysis_checks,
              "heldout_critic_rollout_datasets": 12,
              "evaluation_receipts": receipts,
              "limits": "Artifact and numerical checks on macOS; no Windows live execution or statistical generalization claim"}
    write_json(HERE / "verification.json", result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
