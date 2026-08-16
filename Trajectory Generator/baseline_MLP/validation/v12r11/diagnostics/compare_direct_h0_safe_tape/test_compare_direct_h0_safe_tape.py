from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest


DIAGNOSTIC_ROOT = Path(__file__).resolve().parent
if str(DIAGNOSTIC_ROOT) not in sys.path:
    sys.path.insert(0, str(DIAGNOSTIC_ROOT))

import compare_direct_h0_safe_tape as comparison  # noqa: E402


@pytest.fixture(scope="module")
def result() -> dict:
    return comparison.build_result()


def test_locked_inputs_and_deterministic_result_are_exact(result: dict) -> None:
    assert result["diagnostic_passed"] is True
    assert result["status"] == ("PASS_V12R11_DIRECT_H0_SAFE_TAPE_OFFLINE_COMPARISON")
    assert result["comparison_contract"]["row_counts"] == {
        "r11": 209,
        "r10": 212,
        "safe": 500,
    }
    assert result["r10_step_journal"]["manifest_sha256"] == (
        "a311bb924f12127ad4f068aa5653085ffeb21a320f0178335987cf9253408a9e"
    )
    payload = comparison._payload(result)
    assert comparison.DEFAULT_OUTPUT.read_bytes() == payload
    assert comparison._sha256_bytes(payload) == (
        "e90dbd99b6670cdd9e267c6dc4209809506331a647068a60c105ab47eb763a32"
    )


def test_exact_divergence_and_action_dynamics(result: dict) -> None:
    onset = result["exact_divergence_onset"]
    for pair in ("r11_vs_safe", "r10_vs_safe", "r11_vs_r10"):
        assert onset[pair]["action"]["first_exact_vector_mismatch_step"] == 1
        assert (
            onset[pair]["invariant_observation"]["first_exact_vector_mismatch_step"]
            == 2
        )

    r11_action = onset["r11_vs_safe"]["action"]
    assert r11_action["root_mean_square_all_values"] == pytest.approx(
        0.07149490480866039
    )
    assert r11_action["maximum_absolute"] == pytest.approx(0.2613184452056885)
    assert r11_action["maximum_absolute_step"] == 207
    assert r11_action["maximum_absolute_dimension"] == "action_1"
    assert r11_action["terminal_aligned_signed_delta"] == pytest.approx(
        [-0.09367632865905762, 0.22116965055465698]
    )

    r10_action = onset["r10_vs_safe"]["action"]
    assert r10_action["root_mean_square_all_values"] == pytest.approx(
        0.11257166568930702
    )
    assert r10_action["maximum_absolute"] == pytest.approx(0.4574432745575905)
    assert r10_action["maximum_absolute_step"] == 210
    assert r10_action["maximum_absolute_dimension"] == "action_1"
    assert r10_action["terminal_aligned_signed_delta"] == pytest.approx(
        [0.21834397315979004, -0.4081554189324379]
    )

    clipping = result["action_dynamics"]
    assert clipping["r11"]["clipped_step_count"] == 0
    assert clipping["r10"]["clipped_steps"] == [207, 208, 209, 210]
    assert clipping["safe"]["clipped_step_count"] == 0


def test_penetration_ramps_and_event_timeline_are_exact(result: dict) -> None:
    penetration = result["penetration_dynamics"]
    assert penetration["r11"]["maximum_m"] == pytest.approx(0.025489193765034043)
    assert penetration["r11"]["maximum_step"] == 209
    assert penetration["r11"]["rise_to_global_peak"]["start_step"] == 195
    assert penetration["r10"]["maximum_m"] == pytest.approx(0.026729949134248383)
    assert penetration["r10"]["maximum_step"] == 212
    assert penetration["r10"]["rise_to_global_peak"]["start_step"] == 202
    assert penetration["safe"]["maximum_m"] == pytest.approx(0.024323924384327976)
    assert penetration["safe"]["maximum_step"] == 211
    assert penetration["safe"]["rise_to_global_peak"]["start_step"] == 192
    assert penetration["safe"]["minimum_clearance_to_limit_m"] == pytest.approx(
        0.0006760756156720255
    )
    assert penetration["r11_minus_safe"]["sustained_positive_suffix_start_step"] == 209
    assert penetration["r10_minus_safe"]["sustained_positive_suffix_start_step"] == 211

    events = result["event_timelines"]
    assert [(row["step"], row["event"]) for row in events["r11"]] == [
        (96, "toe_off"),
        (147, "heel_strike"),
    ]
    assert [(row["step"], row["event"]) for row in events["r10"]] == [
        (100, "toe_off"),
        (150, "heel_strike"),
    ]
    assert [row["step"] for row in events["safe"]] == [109, 148, 252, 302, 406, 455]


def test_empirical_18_feature_support_and_fit_contract_are_exact(result: dict) -> None:
    support = result["support_18_invariant_features"]
    calibration = support["safe_calibration"]["loo_nearest_neighbor_rms_z"]
    assert calibration["p99"] == pytest.approx(0.20330878485396986)
    assert calibration["maximum"] == pytest.approx(0.9673139558385082)

    assert support["r11"]["exact_safe_row_match_count"] == 1
    assert support["r11"]["first_non_exact_safe_row_step"] == 2
    r11_nn = support["r11"]["nearest_neighbor_rms_z"]
    assert r11_nn["first_above_p99_step"] == 14
    assert r11_nn["above_p99_row_count"] == 154
    assert r11_nn["first_beyond_maximum_loo_envelope_step"] == 208
    assert r11_nn["beyond_maximum_loo_envelope_row_count"] == 2

    assert support["r10"]["exact_safe_row_match_count"] == 1
    assert support["r10"]["first_non_exact_safe_row_step"] == 2
    r10_nn = support["r10"]["nearest_neighbor_rms_z"]
    assert r10_nn["first_above_p99_step"] == 72
    assert r10_nn["above_p99_row_count"] == 80
    assert r10_nn["first_beyond_maximum_loo_envelope_step"] == 210
    assert r10_nn["beyond_maximum_loo_envelope_row_count"] == 3

    target = result["v12r12_fit_requirements"]["safe_target_contract"]
    assert target["row_count"] == 500
    assert target["unique_invariant_feature_row_count"] == 500
    assert target["duplicate_invariant_feature_row_count"] == 0
    assert target["conflicting_target_group_count"] == 0
    assert target["target_binding_exact_for_all_rows"] is True


def test_script_has_hard_offline_boundary() -> None:
    source = Path(comparison.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])
    assert imported_roots <= {
        "__future__",
        "argparse",
        "collections",
        "hashlib",
        "json",
        "math",
        "pathlib",
        "typing",
    }
    assert imported_roots.isdisjoint(
        {"torch", "ray", "gymnasium", "opensim", "pickle", "subprocess", "importlib"}
    )


def test_small_numeric_helpers_are_deterministic() -> None:
    assert comparison._rise_to_peak([0.2, 0.1, 0.15, 0.21, 0.19]) == {
        "start_step": 2,
        "peak_step": 4,
        "row_count": 3,
        "start_m": 0.1,
        "peak_m": 0.21,
        "rise_m": pytest.approx(0.11),
        "mean_rise_per_step_m": pytest.approx(0.055),
        "last_increment_m": pytest.approx(0.06),
    }
    assert comparison._true_intervals([False, True, True, False, True]) == [
        {"start_step": 2, "end_step": 3},
        {"start_step": 5, "end_step": 5},
    ]
