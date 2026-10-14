"""Physical gates must not reward early termination or unavailable metrics."""
from copy import deepcopy
import json

import pytest

from unified_MLP_39.reward_tuning import compare as comparison
from unified_MLP_39.reward_tuning.compare import compare_metrics, _paired_receipts
from unified_MLP_39.reward_tuning.io import sha256, write_json


def metrics():
    tracking = {joint: {"actual_rmse_deg": 4., "served_rmse_deg": 4., "excursion_deg": 20.,
                       "phase_bins": {str(i): {"actual_rmse_deg": 4., "served_rmse_deg": 4., "samples": 10}
                                      for i in range(10)}} for joint in ("knee", "ankle")}
    return {"episode": {"completed": True, "valid_cycles": 2, "end_reason": "episode_time_limit", "penetration_max_mm": 15.},
            "tracking": tracking, "contact": {"vertical_impulse_n_s": 1000.},
            "knee": {"rate_rms_nm_s": 100., "hp10_rms_nm": 8., "hp20_rms_nm": 4.},
            "kinematic_smoothness": {j + "_" + c: {"jerk_rms_rad_s3": 100., "position_hp10_rms_deg": .1}
                                     for j in ("knee", "ankle") for c in ("actual", "served")}}


def checks(candidate, reference=None):
    return {c["name"]: c["passed"] for c in compare_metrics(candidate, reference or metrics(), nominal=True)}


def test_identical_policy_passes_physical_constraints():
    assert all(checks(metrics()).values())


def test_tracking_uses_minimum_of_relative_and_absolute_margin():
    value = metrics()
    value["tracking"]["ankle"]["actual_rmse_deg"] = 4.41
    assert checks(value)["tracking_ankle_actual_global"] is False
    reference = metrics()
    reference["tracking"]["knee"]["actual_rmse_deg"] = 10.
    value = deepcopy(reference)
    value["tracking"]["knee"]["actual_rmse_deg"] = 10.51
    assert checks(value, reference)["tracking_knee_actual_global"] is False


def test_phase_regression_is_not_hidden_by_global_rmse():
    value = metrics()
    value["tracking"]["knee"]["phase_bins"]["3"]["served_rmse_deg"] = 6.
    result = checks(value)
    assert result["tracking_knee_served_global"] is True
    assert result["tracking_knee_served_phase_3"] is False


def test_missing_contact_does_not_pass_support_gate():
    value = metrics()
    value["contact"] = None
    assert checks(value)["support_impulse"] is None


def test_short_failed_episode_cannot_pass_completion():
    value = metrics()
    value["episode"].update(completed=False, valid_cycles=1, end_reason="grf_penetration_hard")
    result = checks(value)
    assert result["completion_of_reference_scenario"] is False
    assert result["no_new_hard_termination"] is False
    assert result["valid_cycles"] is False


def test_flattening_or_unloading_fail_separately():
    value = metrics()
    value["tracking"]["knee"]["excursion_deg"] = 17.
    value["contact"]["vertical_impulse_n_s"] = 890.
    result = checks(value)
    assert result["excursion_knee"] is False
    assert result["support_impulse"] is False


def test_missing_noise_receipt_is_unknown(tmp_path):
    assert _paired_receipts({"best80": tmp_path})["passed"] is None


def comparison_fixture(tmp_path, monkeypatch, candidate_role=None):
    """Seal small artifact receipts while isolating the physical metric reader."""
    role = candidate_role or "ankle_10"
    cases = []
    for scenario in ("nominal", "noise_123", "noise_125", "noise_127"):
        for name in ("best80", "control", role):
            folder = tmp_path / "rollouts" / f"{name}_{scenario}"
            for filename in ("rollout_summary.json", "rollout_policy_trace.json"):
                write_json(folder / filename, {})
            write_json(folder / "paired_noise_receipt.json", {
                "full_draws_sha256": scenario, "sigma_sha256": "shared_sigma",
                "records": [{"standard_normal": [.1, .2], "sigma": [.2, .3]}]})
            artifact_hashes = {p.name: sha256(p) for p in folder.iterdir()}
            write_json(folder / "evaluation_complete.json", {
                "role": name, "scenario": scenario, "artifact_hashes": artifact_hashes})
            cases.append({"role": name, "scenario": scenario, "output_dir": str(folder)})
    manifest = {"repo_root": str(tmp_path), "input_hashes": {}, "evaluation_cases": cases}
    if candidate_role is not None:
        manifest["candidate_role"] = candidate_role
    manifest_path = tmp_path / "pilot/pilot_manifest.json"
    write_json(manifest_path, manifest)
    write_json(tmp_path / "pilot/pilot_state.json", {"manifest_sha256": sha256(manifest_path)})

    def load(path, *, identity, scenario):
        value = metrics()
        value["episode"].update(start_s=13., stop_s=18.)
        value["ankle"] = {"positive_area_nm_s": 8. if identity == role else 10.}
        return {"metrics": value, "input_hashes": {}}

    monkeypatch.setattr(comparison, "load_dataset", load)
    monkeypatch.setattr(comparison, "measure", lambda dataset, *args, **kwargs: dataset["metrics"])
    return cases


@pytest.mark.parametrize("candidate_role", [None, "ankle_5"])
def test_compare_uses_manifest_candidate_with_legacy_default(tmp_path, monkeypatch, candidate_role):
    comparison_fixture(tmp_path, monkeypatch, candidate_role)
    result = comparison.compare({}, tmp_path)
    assert result["candidate_role"] == (candidate_role or "ankle_10")
    assert result["status"] == "promising"
    assert len(result["scenarios"]) == 4
    assert all(c["passed"] for c in result["nominal_ankle_area"])
    assert all(c["median_delta_nm_s"] == -2. for c in result["noisy_ankle_area"].values())


def test_compare_rejects_modified_evaluation_artifact_before_reading(tmp_path, monkeypatch):
    cases = comparison_fixture(tmp_path, monkeypatch, "ankle_5")
    from pathlib import Path
    write_json(Path(cases[0]["output_dir"]) / "rollout_summary.json", {"tampered": True})
    monkeypatch.setattr(comparison, "load_dataset", lambda *args, **kwargs: pytest.fail("Read changed artifact"))
    with pytest.raises(ValueError, match="input changed"):
        comparison.compare({}, tmp_path)


@pytest.mark.parametrize("unsealed_sto", [False, True])
def test_compare_missing_evidence_never_passes(tmp_path, monkeypatch, unsealed_sto):
    cases = comparison_fixture(tmp_path, monkeypatch, "ankle_5")
    from pathlib import Path
    folder = Path(cases[2]["output_dir"])
    if unsealed_sto:
        file = folder / "sim_outputs/rollout_episode_sea_torques.sto"
        file.parent.mkdir()
        file.write_text("unsealed physical data", encoding="utf-8")
    else:
        receipt_path = folder / "evaluation_complete.json"
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        del receipt["artifact_hashes"]["paired_noise_receipt.json"]
        write_json(receipt_path, receipt)
    result = comparison.compare({}, tmp_path)
    assert result["status"] == "incomplete"
    assert result["missing_cases"] == ["ankle_5/nominal: incomplete artifact receipt"]
    assert "nominal" not in result["scenarios"]


def test_compare_rejects_misidentified_evaluation_receipt(tmp_path, monkeypatch):
    cases = comparison_fixture(tmp_path, monkeypatch, "ankle_5")
    from pathlib import Path
    receipt_path = Path(cases[2]["output_dir"]) / "evaluation_complete.json"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["role"] = "ankle_10"
    write_json(receipt_path, receipt)
    with pytest.raises(ValueError, match="different case"):
        comparison.compare({}, tmp_path)
