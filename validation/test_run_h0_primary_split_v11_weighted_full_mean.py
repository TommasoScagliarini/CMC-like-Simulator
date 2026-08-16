from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

import run_h0_primary_split_v11_weighted_full_mean as runner


def _record_stub(path: object) -> dict[str, object]:
    return {"path": str(path), "sha256": "0" * 64, "size_bytes": 1}


def _passing_lineage() -> dict[str, object]:
    return {
        "v10s_terminal_failure": {
            "ledger": _record_stub("ledger"),
            "p0_gate": _record_stub("gate"),
            "p0_summary": _record_stub("summary"),
            "gate": {"passed": True},
        },
        "v11_design_audit": {
            "receipt": _record_stub("audit"),
            "gate": {"passed": True},
            "current_bindings": {
                "source_bindings": {},
                "source_h0": {},
                "corpus": {},
            },
            "current_binding_gate": {"passed": True},
        },
    }


def test_frozen_runner_is_content_pinned_and_transformed_narrowly() -> None:
    observed = hashlib.sha256(runner._FROZEN_RUNNER_PATH.read_bytes()).hexdigest()
    assert observed == runner._FROZEN_RUNNER_SHA256
    assert observed == (
        "4240afdb37b99734f8dbe80b67c9dbef491a43ff65d6bbc78cab2f15a3300faf"
    )
    source = runner._TRANSFORMED_RUNNER_SOURCE
    assert "V10S" not in source
    assert (
        "import h0_primary_split_v11_weighted_full_mean_contract as contract"
        in source
    )
    assert "import h0_primary_split_v11_weighted_fit as fit_engine" in source
    assert "import h0_primary_split_v10s_blend as safe_dagger" in source
    assert "h0_primary_split_v10s_safe_dagger_contract as contract" not in source
    assert "h0_primary_split_v10s_fit as fit_engine" not in source
    assert runner.contract.PROTOCOL_ID == (
        "AB06_H0_PRIMARY_SPLIT_V11_V26_WEIGHTED_FULL_MEAN_SAFE_DAGGER"
    )
    assert runner.fit_engine.__name__ == "h0_primary_split_v11_weighted_fit"


def test_stage_order_safe_blend_and_final_pure_v26_are_preserved() -> None:
    assert runner._expected_stage_order() == runner.contract.STAGE_IDS
    assert len(runner.contract.STAGE_IDS) == 18
    assert dict(runner.contract.ROUND_ALPHAS) == {1: 0.25, 2: 0.5, 3: 0.75}
    collection = runner._TRANSFORMED_RUNNER_SOURCE[
        runner._TRANSFORMED_RUNNER_SOURCE.index("def _collect_safe_dagger") :
        runner._TRANSFORMED_RUNNER_SOURCE.index("def verify_collection_receipt")
    ]
    assert "safe_dagger.SafetyLatchState" in collection
    assert "safe_dagger.select_safe_dagger_action" in collection
    assert "safe_dagger.blend_policy_means" in collection
    assert "safe_dagger.apply_single_noise" in collection
    assert '"teacher_queried_on_same_state": True' in collection
    final = runner._TRANSFORMED_RUNNER_SOURCE[
        runner._TRANSFORMED_RUNNER_SOURCE.index("def _final_rollout") :
        runner._TRANSFORMED_RUNNER_SOURCE.index("def verify_final_rollout_receipt")
    ]
    assert "RLModule.from_checkpoint(SOURCE_H0_MODULE)" not in final
    assert '"teacher_query_count": 0' in final
    assert '"mean_blend_count": 0' in final
    assert '"safety_intervention_count": 0' in final


def test_worker_subprocess_reenters_v11_wrapper() -> None:
    command = runner._worker_command("fit_p0", "x" * 32)
    assert Path(command[1]).resolve() == Path(runner.__file__).resolve()
    assert Path(command[1]).name == "run_h0_primary_split_v11_weighted_full_mean.py"
    assert command[2:] == [
        "--worker",
        "--stage",
        "fit_p0",
        "--supervisor-token",
        "x" * 32,
    ]


def test_v11_candidate_evidence_is_propagated_p3_to_final() -> None:
    source = runner._TRANSFORMED_RUNNER_SOURCE
    freeze = source[source.index("def _freeze_p3") : source.index("def _candidate_freeze")]
    for field in (
        "fit_contract_id",
        "design_audit_id",
        "design_audit_receipt",
        "v10s_terminal_failure_id",
        "source_checkpoint_scope",
        "critic_present",
        "critic_parameter_count",
        "normalization_folded_into_first_layer",
        "runtime_normalization_wrapper_present",
        "prescribed_clock_present",
        "disabled_clock_columns_0_1_bit_zero",
    ):
        assert field in freeze
    assert 'p3_summary.get("fit_contract_id")' in freeze
    assert 'p3_summary.get("design_audit_receipt")' in freeze
    assert 'p3_summary.get(\n            "source_checkpoint_scope"' in freeze
    assert 'p3_summary.get("critic_present")' in freeze
    assert 'p3_summary.get(\n            "critic_parameter_count"' in freeze
    assert "receipt = {\n        **summary," in freeze
    assert 'p3_summary.get(\n            "runtime_normalization_wrapper_present"' in freeze

    final = source[source.index("def _final_rollout") : source.index(
        "def verify_final_rollout_receipt"
    )]
    for field in (
        "fit_contract_id",
        "design_audit_id",
        "design_audit_receipt",
        "source_checkpoint_scope",
        "critic_present",
        "critic_parameter_count",
        "normalization_folded_into_first_layer",
        "runtime_normalization_wrapper_present",
        "prescribed_clock_present",
        "disabled_clock_columns_0_1_bit_zero",
    ):
        assert field in final
    assert 'freeze.get("fit_contract_id")' in final
    assert 'freeze.get(\n            "source_checkpoint_scope"' in final
    assert 'freeze.get("critic_present")' in final
    assert 'freeze.get(\n            "critic_parameter_count"' in final
    assert final.count("source_checkpoint_scope") == 4
    assert final.count("critic_present") == 4
    assert final.count("critic_parameter_count") == 4
    assert 'freeze.get(\n            "runtime_normalization_wrapper_present"' in final

    aggregate = source[source.index("def _finalize_development") : source.index(
        "def verify_final_development_receipt"
    )]
    assert 'freeze.get("fit_contract_id")' in aggregate
    assert 'freeze.get("design_audit_receipt")' in aggregate
    assert 'freeze.get(\n            "source_checkpoint_scope"' in aggregate
    assert 'freeze.get("critic_present")' in aggregate
    assert 'freeze.get(\n            "critic_parameter_count"' in aggregate
    assert aggregate.count("source_checkpoint_scope") == 4
    assert aggregate.count("critic_present") == 4
    assert aggregate.count("critic_parameter_count") == 4
    assert 'freeze.get(\n            "normalization_folded_into_first_layer"' in aggregate
    assert 'freeze.get(\n            "disabled_clock_columns_0_1_bit_zero"' in aggregate


def test_all_declared_write_paths_are_isolated_from_frozen_v10s(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    audit = runner._v11_path_isolation_audit()
    assert audit["passed"] is True
    assert audit["violations"] == []
    assert all(audit["globals_match_contract"].values())
    old_root = runner._READ_ONLY_V10S_RUN_ROOT
    for relative in audit["declared_mutation_paths"].values():
        path = runner.REPO_ROOT / str(relative)
        assert path != old_root
        assert old_root not in path.parents

    monkeypatch.setattr(runner, "RUN_ROOT", old_root)
    drifted = runner._v11_path_isolation_audit()
    assert drifted["passed"] is False
    assert any(row["name"] == "run_root" for row in drifted["violations"])


def test_lineage_audit_invokes_both_contract_gates(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    paths: dict[str, Path] = {}
    for name in (
        "V10S_TERMINAL_LEDGER_PATH",
        "V10S_P0_GATE_PATH",
        "V10S_P0_SUMMARY_PATH",
        "DESIGN_AUDIT_RECEIPT_PATH",
    ):
        path = tmp_path / f"{name}.json"
        path.write_text(json.dumps({"kind": name}), encoding="utf-8")
        paths[name] = path
    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(runner, "_lineage_path", lambda name: paths[name])
    monkeypatch.setattr(runner, "_record", _record_stub)
    monkeypatch.setattr(
        runner.contract,
        "v10s_terminal_failure_gate",
        lambda ledger, gate, summary: calls.append(
            ("terminal", (ledger, gate, summary))
        )
        or {"passed": True},
    )
    monkeypatch.setattr(
        runner.contract,
        "design_audit_gate",
        lambda receipt: calls.append(("design", receipt)) or {"passed": True},
    )
    current = {
        "source_bindings": {},
        "source_h0": {},
        "corpus": {},
    }
    monkeypatch.setattr(runner, "_design_audit_current_bindings", lambda: current)
    monkeypatch.setattr(
        runner.contract,
        "design_audit_current_binding_gate",
        lambda receipt, bindings: calls.append(("current", bindings))
        or {"passed": bindings == current},
    )
    audit = runner._v11_lineage_audit()
    assert audit["v10s_terminal_failure"]["gate"] == {"passed": True}
    assert audit["v11_design_audit"]["gate"] == {"passed": True}
    assert audit["v11_design_audit"]["current_bindings"] == current
    assert audit["v11_design_audit"]["current_binding_gate"] == {"passed": True}
    assert [row[0] for row in calls] == ["terminal", "design", "current"]

    monkeypatch.setattr(
        runner.contract,
        "design_audit_gate",
        lambda receipt: {"passed": False},
    )
    with pytest.raises(runner.V11SafeDaggerError, match="design audit"):
        runner._v11_lineage_audit()


def test_design_audit_current_source_hash_drift_fails_lineage(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    source_paths: dict[str, Path] = {}
    for name in ("contract", "fitter", "audit_cli", "validator"):
        path = tmp_path / f"{name}.py"
        path.write_text(f"# {name}\n", encoding="utf-8")
        source_paths[name] = path
    h0 = tmp_path / "source_h0"
    h0.mkdir()
    (h0 / "state.bin").write_bytes(b"h0")
    corpus = tmp_path / "corpus.npz"
    corpus.write_bytes(b"corpus")

    def local_record(path: object) -> dict[str, object]:
        resolved = Path(path)
        payload = resolved.read_bytes()
        return {
            "path": str(resolved),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
        }

    expected_sources = {
        name: local_record(path) for name, path in sorted(source_paths.items())
    }
    evidence_paths: dict[str, Path] = {}
    payloads = {
        "V10S_TERMINAL_LEDGER_PATH": {"kind": "ledger"},
        "V10S_P0_GATE_PATH": {"kind": "gate"},
        "V10S_P0_SUMMARY_PATH": {"kind": "summary"},
        "DESIGN_AUDIT_RECEIPT_PATH": {"source_bindings": expected_sources},
        "V10S_P0_CORPUS_PATH": {"unused": True},
    }
    for name, payload in payloads.items():
        path = corpus if name == "V10S_P0_CORPUS_PATH" else tmp_path / f"{name}.json"
        if path is not corpus:
            path.write_text(json.dumps(payload), encoding="utf-8")
        evidence_paths[name] = path

    monkeypatch.setattr(
        runner.contract,
        "DESIGN_AUDIT_SOURCE_RELATIVE_PATHS",
        {name: str(path) for name, path in source_paths.items()},
    )
    monkeypatch.setattr(runner.contract, "SOURCE_H0_MODULE_PATH", str(h0))
    monkeypatch.setattr(
        runner, "resolve_relative", lambda path: Path(str(path)).resolve()
    )
    monkeypatch.setattr(
        runner, "_lineage_path", lambda name: evidence_paths[name]
    )
    monkeypatch.setattr(runner, "_record", local_record)
    monkeypatch.setattr(
        runner,
        "_tree_record",
        lambda path: {"path": str(path), "tree_sha256": "h" * 64},
    )
    monkeypatch.setattr(
        runner.contract,
        "v10s_terminal_failure_gate",
        lambda ledger, gate, summary: {"passed": True},
    )
    monkeypatch.setattr(
        runner.contract, "design_audit_gate", lambda receipt: {"passed": True}
    )
    monkeypatch.setattr(
        runner.contract,
        "design_audit_current_binding_gate",
        lambda receipt, current: {
            "passed": receipt.get("source_bindings")
            == current.get("source_bindings")
        },
    )

    first = runner._v11_lineage_audit()
    assert first["v11_design_audit"]["current_binding_gate"]["passed"] is True
    source_paths["fitter"].write_text("# fitter drift\n", encoding="utf-8")
    with pytest.raises(runner.V11SafeDaggerError, match="source bindings drifted"):
        runner._v11_lineage_audit()


def test_design_audit_current_bindings_rejects_manifest_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        runner.contract,
        "DESIGN_AUDIT_SOURCE_RELATIVE_PATHS",
        {
            "contract": "contract.py",
            "fitter": "fitter.py",
            "audit_cli": "audit.py",
        },
    )
    with pytest.raises(runner.V11SafeDaggerError, match="source manifest drifted"):
        runner._design_audit_current_bindings()


def test_preflight_adds_lineage_pin_and_isolation_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    base = {
        "status": "base",
        "passed": True,
        "checks": {"inherited": True},
        "next_stage": "base",
    }
    monkeypatch.setattr(
        runner,
        "_INHERITED_BUILD_PREFLIGHT",
        lambda require_unoccupied: dict(base),
    )
    monkeypatch.setattr(runner, "_v11_lineage_audit", _passing_lineage)
    monkeypatch.setattr(
        runner,
        "_v11_path_isolation_audit",
        lambda: {"passed": True, "violations": []},
    )
    monkeypatch.setattr(
        runner,
        "_frozen_runner_binding",
        lambda: {"expected_sha256": runner._FROZEN_RUNNER_SHA256},
    )
    result = runner.build_preflight(require_unoccupied=True)
    assert result["passed"] is True
    assert result["status"] == runner.contract.PREFLIGHT_PASS_STATUS
    assert result["checks"]["v10s_terminal_failure_bound"] is True
    assert result["checks"]["v11_design_audit_pass"] is True
    assert result["checks"]["v11_design_audit_current_bindings"] is True
    assert result["checks"]["v11_write_paths_isolated"] is True
    assert result["v11_lineage"] == _passing_lineage()

    monkeypatch.setattr(
        runner,
        "_v11_path_isolation_audit",
        lambda: {"passed": False, "violations": [{"name": "bad"}]},
    )
    diagnostic = runner.build_preflight(require_unoccupied=False)
    assert diagnostic["passed"] is False
    assert diagnostic["checks"]["v11_write_paths_isolated"] is False
    with pytest.raises(runner.V11SafeDaggerError, match="preflight failed"):
        runner.build_preflight(require_unoccupied=True)


def test_execution_lock_recomputes_and_binds_lineage(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        runner,
        "_INHERITED_LOCK_PAYLOAD",
        lambda preflight: {"base": preflight["marker"]},
    )
    monkeypatch.setattr(runner, "_v11_lineage_audit", _passing_lineage)
    monkeypatch.setattr(
        runner,
        "_v11_path_isolation_audit",
        lambda: {"passed": True, "violations": []},
    )
    monkeypatch.setattr(
        runner,
        "_frozen_runner_binding",
        lambda: {"expected_sha256": runner._FROZEN_RUNNER_SHA256},
    )
    payload = runner._lock_payload({"marker": "preflight"})
    assert payload["base"] == "preflight"
    assert payload["v11_lineage"] == _passing_lineage()
    assert payload["write_path_isolation"]["passed"] is True
    assert payload["next_stage"] == (
        "EXECUTE_V11_WEIGHTED_FULL_MEAN_SAFE_DAGGER_ONCE"
    )

    monkeypatch.setattr(
        runner,
        "_v11_path_isolation_audit",
        lambda: {"passed": False, "violations": [{"name": "bad"}]},
    )
    with pytest.raises(runner.V11SafeDaggerError, match="before execution lock"):
        runner._lock_payload({"marker": "preflight"})


def test_preexecution_revalidates_lineage_and_path_isolation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[str] = []
    monkeypatch.setattr(
        runner,
        "_INHERITED_PREEXECUTION_ABSENCE",
        lambda: calls.append("inherited"),
    )
    monkeypatch.setattr(
        runner, "_v11_lineage_audit", lambda: calls.append("lineage") or {}
    )
    monkeypatch.setattr(
        runner,
        "_v11_path_isolation_audit",
        lambda: calls.append("isolation") or {"passed": True},
    )
    runner._preexecution_absence()
    assert calls == ["inherited", "lineage", "isolation"]

    monkeypatch.setattr(
        runner, "_v11_path_isolation_audit", lambda: {"passed": False}
    )
    with pytest.raises(runner.V11SafeDaggerError, match="isolation drifted"):
        runner._preexecution_absence()


def test_fit_worker_dispatches_only_to_v11_weighted_fit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[dict[str, object]] = []
    monkeypatch.setattr(
        runner.fit_engine,
        "run_fit_stage",
        lambda **kwargs: calls.append(kwargs) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_cumulative_collection_receipt_paths",
        lambda stage: (Path("r1.json"),) if stage == "p1" else (),
    )
    monkeypatch.setattr(
        runner, "verify_fit_receipt", lambda stage: {"passed": True, "stage": stage}
    )
    result = runner._run_fit_stage("p1")
    assert result == {"passed": True, "stage": "p1"}
    assert len(calls) == 1
    assert calls[0]["stage"] == "p1"
    assert calls[0]["dagger_receipt_paths"] == (Path("r1.json"),)
    assert calls[0]["enforce_canonical_destination"] is True


def test_cli_rejects_unclaimed_worker_without_writing() -> None:
    assert runner.main(["--worker"]) == 2
