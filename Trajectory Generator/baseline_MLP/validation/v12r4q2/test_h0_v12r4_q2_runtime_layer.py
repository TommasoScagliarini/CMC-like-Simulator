"""Read-only and temporary-root tests for the additive Q2 runtime layer."""

from __future__ import annotations

import copy
import importlib.util
import os
import sys
from pathlib import Path
from typing import Any

import pytest

Q2_ROOT = Path(__file__).resolve().parent
if str(Q2_ROOT) not in sys.path:
    sys.path.insert(0, str(Q2_ROOT))

import freeze_h0_v12r4_q2_qualification_protocol as protocol  # noqa: E402
import h0_v12r4_q2_qualification_gates as gates  # noqa: E402
import h0_v12r4_q2_runtime_contract as contract  # noqa: E402
import prepare_h0_v12r4_q2_qualification_noise_tapes as noise  # noqa: E402
import run_h0_v12r4_q2_qualification as runner  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_process_binding() -> Any:
    contract.clear_candidate_binding_for_tests()
    yield
    contract.clear_candidate_binding_for_tests()


def _candidate() -> tuple[str, dict[str, Any]]:
    tree_sha256 = "a" * 64
    module = {
        "path": contract.R4_CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": tree_sha256,
        "file_count": 1,
        "files": [
            {
                "path": "module_state.pkl",
                "sha256": "b" * 64,
                "size_bytes": 123,
            }
        ],
    }
    return f"h0_v12r4_p3::{tree_sha256}", module


def _write(path: Path, payload: dict[str, Any]) -> None:
    noise.forensic.write_json_exclusive(path, payload)


def _temp_prerequisites(
    tmp_path: Path,
) -> tuple[dict[str, Path], dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    candidate_id, candidate_module = _candidate()
    requirements = contract.prerequisite_requirements()
    names = [row["name"] for row in requirements]
    paths = {
        name: tmp_path / f"{index:02d}_{name}.json" for index, name in enumerate(names)
    }
    selection = {
        "rule": contract.R4_CANDIDATE_SELECTION_RULE,
        "module_path": contract.R4_CANDIDATE_MODULE_PATH.as_posix(),
        "candidate_id": "DEFERRED_UNTIL_FIT_P3",
        "candidate_tree_sha256": "DEFERRED_UNTIL_FIT_P3",
    }
    protocol_payload = {
        "status": requirements[0]["required_status"],
        "passed": True,
        "candidate_selection": selection,
        "q2_design_freeze": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
    }
    _write(paths[names[0]], protocol_payload)
    protocol_record = noise._record(paths[names[0]])

    lock_payload = {
        "status": requirements[1]["required_status"],
        "passed": True,
        "candidate_selection": selection,
        "q2_design_freeze": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
        "protocol_freeze": protocol_record,
    }
    _write(paths[names[1]], lock_payload)
    lock_record = noise._record(paths[names[1]])

    candidate_payload = {
        "status": requirements[2]["required_status"],
        "passed": True,
        "candidate_selection_rule": contract.R4_CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
    }
    _write(paths[names[2]], candidate_payload)
    candidate_record = noise._record(paths[names[2]])

    final_payload = {
        "status": requirements[3]["required_status"],
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
        "candidate_freeze": candidate_record,
        "retry_authorized": False,
        "resume_authorized": False,
        "runtime_promoted": False,
    }
    _write(paths[names[3]], final_payload)
    final_record = noise._record(paths[names[3]])

    ledger_payload = {
        "status": requirements[4]["required_status"],
        "passed": True,
        "terminal": True,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
        "candidate_freeze": candidate_record,
        "final_development_receipt": final_record,
        "protocol_freeze": protocol_record,
        "execution_lock": lock_record,
        "q2_paths_opened": [],
        "runtime_promoted": False,
    }
    _write(paths[names[4]], ledger_payload)

    payloads = {
        names[0]: protocol_payload,
        names[1]: lock_payload,
        names[2]: candidate_payload,
        names[3]: final_payload,
        names[4]: ledger_payload,
    }
    records = {name: noise._record(paths[name]) for name in names}
    return paths, payloads, records


def test_design_freeze_record_is_the_published_immutable_input() -> None:
    path = noise.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)

    assert noise._record(path) == noise.DESIGN_FREEZE_RECORD
    verified = noise._verified_design_binding()
    assert verified["record"] == noise.DESIGN_FREEZE_RECORD
    assert verified["payload"]["candidate_binding_state"] == "DEFERRED"


def test_runtime_contract_starts_deferred_and_binds_only_canonical_r4_candidate() -> (
    None
):
    candidate_id, module = _candidate()

    assert contract.P1_CANDIDATE_ID is None
    with pytest.raises(contract.Q2CandidateBindingError, match="deferred"):
        contract.current_candidate_binding()
    binding = contract.bind_candidate(candidate_id, module)
    assert binding == {"candidate_id": candidate_id, "candidate_module": module}
    assert (
        contract.role_contract(contract.BASELINE_ROLE)["candidate_id"] == candidate_id
    )
    candidate_role = contract.role_contract(contract.CANDIDATE_ROLE)
    assert candidate_role["actor_id"] == candidate_id
    assert candidate_role["actor_module"] == module

    drifted = copy.deepcopy(module)
    drifted["tree_sha256"] = "c" * 64
    with pytest.raises(contract.Q2CandidateBindingError):
        contract.bind_candidate(f"h0_v12r4_p3::{'c' * 64}", drifted)


@pytest.mark.parametrize(
    ("candidate_id", "mutation"),
    [
        ("wrong", None),
        (None, ("path", "wrong/path")),
        (None, ("tree_sha256", "z" * 64)),
        (None, ("files", [])),
    ],
)
def test_candidate_binding_rejects_alias_path_hash_or_tree_drift(
    candidate_id: str | None, mutation: tuple[str, Any] | None
) -> None:
    expected_id, module = _candidate()
    if mutation is not None:
        module[mutation[0]] = mutation[1]
    with pytest.raises(contract.Q2CandidateBindingError):
        contract.validate_candidate_binding(candidate_id or expected_id, module)


def test_reference_tapes_match_all_five_frozen_hashes_without_materialization() -> None:
    root = noise.resolve_relative(contract.NOISE_ROOT)
    assert not os.path.lexists(root)

    tapes = noise.build_tapes()

    assert len(tapes) == 5
    assert {
        name: noise.array_sha256(definition["standard_normal"])
        for name, definition in tapes.items()
    } == noise.design_freezer.contract.EXPECTED_TAPE_ARRAY_SHA256
    assert not os.path.lexists(root)


def test_r4_prerequisite_validation_resolves_one_exact_candidate(
    tmp_path: Path,
) -> None:
    _, payloads, records = _temp_prerequisites(tmp_path)

    gate = noise.validate_prerequisite_payloads(
        payloads, records, enforce_canonical_record_paths=False
    )

    candidate_id, module = _candidate()
    assert gate["passed"] is True
    assert gate["status"] == contract.PREREQUISITE_PASS_STATUS
    assert gate["candidate_id"] == candidate_id
    assert gate["candidate_module"] == module
    assert gate["frozen_gate"]["passed"] is True
    assert contract.current_candidate_binding()["candidate_id"] == candidate_id


def test_r4_prerequisite_validation_rejects_selection_or_terminal_drift(
    tmp_path: Path,
) -> None:
    _, payloads, records = _temp_prerequisites(tmp_path)
    names = list(payloads)
    payloads[names[0]]["candidate_selection"]["rule"] = "wrong"

    gate = noise.validate_prerequisite_payloads(
        payloads, records, enforce_canonical_record_paths=False
    )
    assert gate["passed"] is False
    assert gate["checks"]["pre_fit_selection_rule_exact"] is False

    _, payloads, records = _temp_prerequisites(tmp_path / "second")
    payloads[names[-1]]["candidate_id"] = "wrong"
    gate = noise.validate_prerequisite_payloads(
        payloads, records, enforce_canonical_record_paths=False
    )
    assert gate["passed"] is False
    assert gate["checks"]["terminal_candidate_exact"] is False


def test_load_five_temp_prerequisites_reads_strict_canonical_chain(
    tmp_path: Path,
) -> None:
    paths, _, _ = _temp_prerequisites(tmp_path)

    loaded = noise.load_and_validate_prerequisites(
        prerequisite_paths=paths,
        require_qualification_unopened=True,
    )

    assert loaded["gate"]["passed"] is True
    assert list(loaded["records"]) == [
        row["name"] for row in contract.prerequisite_requirements()
    ]


def test_prepare_and_verify_temp_tapes_is_no_clobber_and_never_opens_canonical(
    tmp_path: Path,
) -> None:
    prerequisites_root = tmp_path / "prerequisites"
    prerequisites_root.mkdir()
    paths, _, _ = _temp_prerequisites(prerequisites_root)
    output = tmp_path / "noise"

    manifest = noise.prepare(
        output_root=output,
        prerequisite_paths=paths,
        enforce_canonical_destination=False,
    )

    candidate_id, module = _candidate()
    assert manifest["passed"] is True
    assert manifest["candidate_id"] == candidate_id
    assert manifest["candidate_module"] == module
    assert len(manifest["tapes"]) == 5
    assert (
        noise.verify_manifest(noise_root=output, enforce_canonical_destination=False)
        == manifest
    )
    with pytest.raises(noise.V12R4Q2QualificationNoiseError, match="clobber"):
        noise.prepare(
            output_root=output,
            prerequisite_paths=paths,
            enforce_canonical_destination=False,
        )
    assert not os.path.lexists(noise.resolve_relative(contract.NOISE_ROOT))


def test_canonical_noise_prepare_is_blocked_while_r4_is_unopened() -> None:
    assert not os.path.lexists(noise.resolve_relative(contract.NOISE_ROOT))
    with pytest.raises(noise.V12R4Q2QualificationNoiseError):
        noise.prepare()
    assert not os.path.lexists(noise.resolve_relative(contract.NOISE_ROOT))


def test_protocol_build_is_blocked_until_noise_and_r4_are_complete() -> None:
    assert not os.path.lexists(protocol.NOISE_MANIFEST_PATH)
    with pytest.raises(protocol.V12R4Q2QualificationProtocolFreezeError):
        protocol.build_protocol_freeze()
    assert not os.path.lexists(protocol.PROTOCOL_FREEZE_PATH)


def test_protocol_assembly_binds_candidate_matrix_sources_and_inputs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    design = noise._verified_design_binding()
    prerequisite_gate = {
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": module,
    }
    artifact = {"path": "x", "sha256": "a" * 64, "size_bytes": 1}
    monkeypatch.setattr(
        protocol,
        "_prerequisite_gate",
        lambda: {"gate": prerequisite_gate, "records": {"r4": artifact}},
    )
    monkeypatch.setattr(
        protocol,
        "_noise_gate",
        lambda: {
            "passed": True,
            "manifest_record": artifact,
            "tapes": {},
        },
    )
    monkeypatch.setattr(
        protocol,
        "_source_gate",
        lambda: {"passed": True, "checks": {"closed": True}, "records": {}},
    )
    monkeypatch.setattr(
        protocol,
        "_input_gate",
        lambda binding=None: {
            "passed": True,
            "checks": {"bound": True},
            "records": {"candidate_module": module},
        },
    )
    monkeypatch.setattr(noise, "_verified_design_binding", lambda: design)

    payload = protocol._assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "qualification_run_root_absent": True,
            "noise_manifest_present": True,
        }
    )

    assert payload["passed"] is True
    assert payload["selected_candidate_id"] == candidate_id
    assert payload["selected_candidate"] == module
    assert payload["rollout_matrix"] == list(contract.ROLLOUT_MATRIX)
    assert payload["stage_order"] == list(contract.STAGE_IDS)
    assert payload["next_stage"] == "PREPARE_NO_CLOBBER_Q2_EXECUTION_LOCK"


def test_runtime_source_closure_explicitly_binds_safe_q1_reuse() -> None:
    gate = protocol._source_gate()

    assert gate["passed"] is True
    assert len(gate["records"]) == 12
    assert gate["records"]["frozen_q1_gates"]["path"].endswith(
        "v12p1q/h0_v12r3_p1_qualification_gates.py"
    )
    assert gate["records"]["frozen_q1_runner_engine"]["path"].endswith(
        "v12p1q/run_h0_v12r3_p1_qualification.py"
    )


def test_runner_uses_isolated_engine_and_q2_paths_without_mutating_q1() -> None:
    import run_h0_v12r3_p1_qualification as q1_runner

    assert runner._ENGINE is not q1_runner
    assert runner._ENGINE.contract is contract
    assert runner._ENGINE.gates is gates
    assert runner._ENGINE.noise is noise
    assert runner._ENGINE.protocol_freezer is protocol
    assert q1_runner.contract.PROTOCOL_ID != contract.PROTOCOL_ID
    assert runner._ENGINE.RUN_ROOT == runner.REPO_ROOT.joinpath(
        *contract.RUN_ROOT.parts
    )


def test_runner_stage_order_is_baseline_six_candidate_six_then_aggregate() -> None:
    assert len(contract.STAGE_IDS) == 13
    assert list(contract.STAGE_IDS[:6]) == [
        f"rollout__baseline__{case_id}" for case_id in contract.CASE_IDS
    ]
    assert list(contract.STAGE_IDS[6:12]) == [
        f"rollout__candidate__{case_id}" for case_id in contract.CASE_IDS
    ]
    assert contract.STAGE_IDS[-1] == "finalize_qualification"
    assert runner.NEXT_STAGE_AFTER_PASS == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"


def test_runner_lock_build_is_blocked_without_protocol_and_opens_nothing() -> None:
    with pytest.raises(runner.V12R4Q2QualificationExecutionError):
        runner.build_execution_lock()
    assert not os.path.lexists(runner._ENGINE.LOCK_PATH)
    assert not os.path.lexists(runner._ENGINE.RUN_ROOT)


def test_runner_lock_wrapper_requires_bound_candidate_and_exact_matrix(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    candidate_id, module = _candidate()

    def sync() -> dict[str, Any]:
        return contract.bind_candidate(candidate_id, module)

    monkeypatch.setattr(runner, "_sync_candidate_from_protocol", sync)
    monkeypatch.setattr(
        runner._ENGINE,
        "build_execution_lock",
        lambda require_unoccupied=True: {
            "candidate_id": candidate_id,
            "candidate_module": module,
            "rollout_matrix": list(contract.ROLLOUT_MATRIX),
            "stage_order": list(contract.STAGE_IDS),
            "passed": True,
        },
    )

    payload = runner.build_execution_lock()
    assert payload["passed"] is True

    monkeypatch.setattr(
        runner._ENGINE,
        "build_execution_lock",
        lambda require_unoccupied=True: {
            "candidate_id": candidate_id,
            "candidate_module": module,
            "rollout_matrix": [],
            "stage_order": list(contract.STAGE_IDS),
            "passed": True,
        },
    )
    with pytest.raises(runner.V12R4Q2QualificationExecutionError, match="matrix"):
        runner.build_execution_lock()


def test_q2_ledger_extension_stops_before_zero_update(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        runner,
        "_original_ledger_payload",
        lambda **kwargs: {"passed": kwargs["passed"]},
    )

    passed = runner._q2_ledger_payload(
        passed=True, attempted_stage=None, completed_stages=[], error=None
    )
    failed = runner._q2_ledger_payload(
        passed=False, attempted_stage="x", completed_stages=[], error=RuntimeError()
    )

    assert passed["next_stage"] == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"
    assert passed["checkpoint_zero_created"] is False
    assert failed["next_stage"] == "STOP_TERMINAL"


def test_q2_gate_wrappers_fail_unbound_and_delegate_only_after_binding(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assert (
        gates.common_rollout_gate({}, role="baseline", case_id=contract.CASE_IDS[0])[
            "passed"
        ]
        is False
    )
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    monkeypatch.setattr(
        gates._Q1_ENGINE,
        "common_rollout_gate",
        lambda summary, role, case_id: {"passed": True, "role": role, "case": case_id},
    )
    monkeypatch.setattr(
        gates._Q1_ENGINE,
        "condition_matched_gate",
        lambda baseline, candidate, case_id: {"passed": True, "case": case_id},
    )
    monkeypatch.setattr(
        gates._Q1_ENGINE,
        "aggregate_qualification_gate",
        lambda summary: {"passed": True},
    )

    assert (
        gates.common_rollout_gate({}, role="baseline", case_id=contract.CASE_IDS[0])[
            "passed"
        ]
        is True
    )
    assert (
        gates.condition_matched_gate({}, {}, case_id=contract.CASE_IDS[0])["passed"]
        is True
    )
    assert gates.aggregate_qualification_gate({})["passed"] is True


def test_reused_q1_gates_pass_real_q2_rollout_pair_and_aggregate_shapes() -> None:
    helper_path = (
        runner.Q1_ROOT / "test_h0_v12r3_p1_qualification_contract_and_gates.py"
    )
    spec = importlib.util.spec_from_file_location("_q2_q1_gate_fixture", helper_path)
    assert spec is not None and spec.loader is not None
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    helper.contract = contract
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    case_id = contract.CASE_IDS[0]

    baseline = helper._valid_rollout(contract.BASELINE_ROLE, case_id)
    candidate = helper._valid_rollout(contract.CANDIDATE_ROLE, case_id)
    aggregate = helper._valid_aggregate_payload()

    assert (
        gates.common_rollout_gate(
            baseline, role=contract.BASELINE_ROLE, case_id=case_id
        )["passed"]
        is True
    )
    assert (
        gates.common_rollout_gate(
            candidate, role=contract.CANDIDATE_ROLE, case_id=case_id
        )["passed"]
        is True
    )
    assert (
        gates.condition_matched_gate(baseline, candidate, case_id=case_id)["passed"]
        is True
    )
    assert gates.aggregate_qualification_gate(aggregate)["passed"] is True


def test_no_canonical_runtime_artifact_was_created() -> None:
    assert (
        noise._record(noise.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH))
        == noise.DESIGN_FREEZE_RECORD
    )
    for path in (
        contract.NOISE_ROOT,
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.RUN_ROOT,
    ):
        assert not os.path.lexists(noise.resolve_relative(path))
