"""Source-only tests for the V12R6 protocol/lock freezer."""

from __future__ import annotations

import ast
import copy
import json
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any

import pytest


V12R6_ROOT = Path(__file__).resolve().parent
if os.fspath(V12R6_ROOT) not in sys.path:
    sys.path.insert(0, os.fspath(V12R6_ROOT))

import freeze_h0_v12r6_functional_composite as freezer  # noqa: E402


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _sandbox(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    root = tmp_path.resolve()
    state = root / "state"
    state.mkdir(parents=True)
    source_a = root / "sources" / "a.py"
    source_b = root / "sources" / "b.py"
    source_a.parent.mkdir()
    source_a.write_text("# source a\n", encoding="utf-8")
    source_b.write_text("# source b\n", encoding="utf-8")
    locked = root / "inputs" / "locked.json"
    _write_json(locked, {"locked": True})
    p2 = root / "inputs" / "p2_module"
    r5 = root / "inputs" / "r5_module"
    for directory, label in ((p2, "p2"), (r5, "r5")):
        directory.mkdir(parents=True)
        (directory / "module_state.pkl").write_bytes(label.encode("ascii"))
        (directory / "metadata.json").write_text(
            json.dumps({"label": label}), encoding="utf-8"
        )

    monkeypatch.setattr(freezer, "REPO_ROOT", root)
    contract = freezer.contract
    monkeypatch.setattr(
        contract,
        "PROTOCOL_FREEZE_PATH",
        PurePosixPath("state/protocol_freeze.json"),
    )
    monkeypatch.setattr(
        contract,
        "EXECUTION_LOCK_PATH",
        PurePosixPath("state/execution_lock.json"),
    )
    monkeypatch.setattr(contract, "RUN_ROOT", PurePosixPath("run/v12r6"))
    monkeypatch.setattr(
        contract,
        "PIPELINE_CLAIM_PATH",
        PurePosixPath("run/v12r6/pipeline_claim.json"),
    )
    monkeypatch.setattr(
        contract,
        "PIPELINE_LEDGER_PATH",
        PurePosixPath("run/v12r6/pipeline_ledger.json"),
    )
    q_names = ("protocol_freeze", "execution_lock", "run_root", "noise_root")
    monkeypatch.setattr(
        contract,
        "HISTORICAL_Q2_CLOSED_PATHS",
        {name: PurePosixPath(f"closed/q2/{name}") for name in q_names},
    )
    monkeypatch.setattr(
        contract,
        "HISTORICAL_Q3_CLOSED_PATHS",
        {name: PurePosixPath(f"closed/q3/{name}") for name in q_names},
    )
    monkeypatch.setattr(
        contract,
        "Q3_CLOSED_PATHS",
        {name: PurePosixPath(f"closed/v12r6q3/{name}") for name in q_names},
    )
    p2_record = freezer.tree_record("inputs/p2_module")
    r5_record = freezer.tree_record("inputs/r5_module")
    locked_record = freezer._artifact_record("inputs/locked.json")  # noqa: SLF001
    monkeypatch.setattr(contract, "P2_MODULE_TREE", p2_record)
    monkeypatch.setattr(contract, "R5_MODULE_TREE", r5_record)
    monkeypatch.setattr(
        contract,
        "SOURCE_RECORDS",
        {
            "p2_module": copy.deepcopy(p2_record),
            "r5_forensic_module": copy.deepcopy(r5_record),
            "locked_input": copy.deepcopy(locked_record),
        },
    )
    source_paths = ("sources/a.py", "sources/b.py")
    monkeypatch.setattr(freezer, "production_source_paths", lambda: source_paths)
    return {
        "root": root,
        "source_a": source_a,
        "source_b": source_b,
        "locked": locked,
        "p2": p2,
        "r5": r5,
        "protocol": root / contract.PROTOCOL_FREEZE_PATH.as_posix(),
        "lock": root / contract.EXECUTION_LOCK_PATH.as_posix(),
        "run": root / contract.RUN_ROOT.as_posix(),
        "q3": root / next(iter(contract.Q3_CLOSED_PATHS.values())).as_posix(),
        "source_paths": source_paths,
    }


@pytest.mark.parametrize(
    "value",
    ("", "../escape", "a/../b", "/absolute", "a//b", "a\\..\\escape"),
)
def test_resolver_rejects_noncanonical_and_windows_traversal(value: str) -> None:
    with pytest.raises(freezer.V12R6FreezeError, match="non-canonical"):
        freezer.resolve_relative(value)


def test_tree_record_is_stable_and_rejects_file_and_directory_links(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path.resolve()
    monkeypatch.setattr(freezer, "REPO_ROOT", root)
    tree = root / "tree"
    nested = tree / "a_nested"
    nested.mkdir(parents=True)
    (tree / "z.txt").write_text("z", encoding="utf-8")
    (nested / "b.txt").write_text("b", encoding="utf-8")

    first = freezer.tree_record("tree")
    second = freezer.tree_record("tree")

    assert first == second
    assert first["file_count"] == 2
    assert [row["path"] for row in first["files"]] == ["a_nested/b.txt", "z.txt"]

    external = root / "external"
    external.mkdir()
    (external / "hidden.txt").write_text("hidden", encoding="utf-8")
    linked_directory = tree / "linked_directory"
    try:
        linked_directory.symlink_to(external, target_is_directory=True)
    except (NotImplementedError, OSError):
        pytest.skip("directory symlinks are unavailable")
    with pytest.raises(freezer.V12R6FreezeError, match="unsafe directory"):
        freezer.tree_record("tree")
    linked_directory.unlink()

    linked_file = tree / "linked_file"
    linked_file.symlink_to(tree / "z.txt")
    with pytest.raises(freezer.V12R6FreezeError, match="unsafe file"):
        freezer.tree_record("tree")


def test_artifact_record_rejects_a_linked_ancestor(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path.resolve()
    monkeypatch.setattr(freezer, "REPO_ROOT", root)
    real = root / "real"
    real.mkdir()
    (real / "value.json").write_text("{}\n", encoding="utf-8")
    alias = root / "alias"
    try:
        alias.symlink_to(real, target_is_directory=True)
    except (NotImplementedError, OSError):
        pytest.skip("directory symlinks are unavailable")

    with pytest.raises(freezer.V12R6FreezeError, match="symlink/junction"):
        freezer._artifact_record("alias/value.json")  # noqa: SLF001


def test_in_memory_protocol_payload_has_exact_source_closure_without_writes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)

    payload = freezer.expected_protocol_payload()

    assert payload["passed"] is True
    assert payload["contract_self_check"]["passed"] is True
    assert payload["production_source_count"] == 2
    assert set(payload["production_source_closure"]) == set(world["source_paths"])
    assert payload["source_records"] == freezer.contract.SOURCE_RECORDS
    assert payload["development_cases"][0]["case_id"] == (
        "deterministic_offset_plus_0p20"
    )
    assert payload["stage_order"] == list(freezer.contract.STAGE_IDS)
    assert payload["one_shot"] is True
    assert payload["qualification_execution_authorized"] is False
    assert payload["new_collection_count"] == 0
    assert not world["protocol"].exists()
    assert not world["lock"].exists()


def test_prepare_publishes_exact_protocol_and_lock_only_in_sandbox(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)

    result = freezer.prepare()

    assert result["passed"] is True
    assert world["protocol"].is_file()
    assert world["lock"].is_file()
    assert not world["run"].exists()
    assert result["protocol"] == freezer.expected_protocol_payload()
    assert result["lock"] == freezer.verify_execution_lock(require_pristine=True)
    assert all(result["lock"]["checks"].values())
    assert all(result["lock"]["occupancy"].values())
    assert result["lock"]["closed_path_snapshot"] == {
        "historical_q2": [],
        "historical_q3": [],
        "future_v12r6q3": [],
    }
    repeated = freezer.prepare()
    assert repeated["protocol"] == result["protocol"]
    assert repeated["lock"] == result["lock"]


def test_existing_tampered_protocol_is_never_clobbered(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)
    freezer.publish_protocol_freeze()
    _write_json(world["protocol"], {"tampered": True})
    tampered_bytes = world["protocol"].read_bytes()

    with pytest.raises(freezer.V12R6FreezeError, match="protocol freeze"):
        freezer.publish_protocol_freeze()

    assert world["protocol"].read_bytes() == tampered_bytes
    assert not world["lock"].exists()


def test_source_closure_drift_invalidates_existing_protocol_without_rewrite(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)
    freezer.publish_protocol_freeze()
    protocol_bytes = world["protocol"].read_bytes()
    world["source_a"].write_text("# source a drifted\n", encoding="utf-8")

    with pytest.raises(freezer.V12R6FreezeError, match="source closure drifted"):
        freezer.verify_protocol_freeze()

    assert world["protocol"].read_bytes() == protocol_bytes
    assert not world["lock"].exists()


def test_locked_input_and_tree_drift_are_detected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)
    freezer.publish_protocol_freeze()
    world["locked"].write_text('{"locked": false}\n', encoding="utf-8")
    with pytest.raises(freezer.V12R6FreezeError, match="locked input artifact"):
        freezer.verify_protocol_freeze()

    other = _sandbox(tmp_path / "tree_case", monkeypatch)
    freezer.publish_protocol_freeze()
    (other["p2"] / "module_state.pkl").write_bytes(b"p2-drifted")
    with pytest.raises(freezer.V12R6FreezeError, match="source module tree"):
        freezer.verify_protocol_freeze()


@pytest.mark.parametrize("blocker", ("run", "qualification"))
def test_protocol_publication_rejects_opened_namespace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, blocker: str
) -> None:
    world = _sandbox(tmp_path, monkeypatch)
    path = world["run"] if blocker == "run" else world["q3"]
    path.mkdir(parents=True)

    with pytest.raises(
        freezer.V12R6FreezeError,
        match="run root exists|qualification output opened",
    ):
        freezer.publish_protocol_freeze()

    assert not world["protocol"].exists()
    assert not world["lock"].exists()


def test_execution_lock_refuses_nonpristine_run_and_is_not_published(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)
    freezer.publish_protocol_freeze()
    world["run"].mkdir(parents=True)

    payload = freezer.expected_execution_lock_payload()

    assert payload["passed"] is False
    assert payload["checks"]["occupancy"] is False
    assert payload["occupancy"]["run_root_absent"] is False
    with pytest.raises(freezer.V12R6FreezeError, match="preconditions failed"):
        freezer.publish_execution_lock()
    assert not world["lock"].exists()


def test_verify_lock_allows_claimed_run_only_when_pristine_not_required(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox(tmp_path, monkeypatch)
    freezer.prepare()
    world["run"].mkdir(parents=True)

    assert freezer.verify_execution_lock(require_pristine=False)["passed"] is True
    with pytest.raises(freezer.V12R6FreezeError, match="no longer pristine"):
        freezer.verify_execution_lock(require_pristine=True)


def test_freezer_is_execution_inert_and_production_closure_excludes_tests() -> None:
    source = Path(freezer.__file__).read_text(encoding="utf-8")
    syntax = ast.parse(source)
    imported = {
        alias.name
        for node in ast.walk(syntax)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    paths = freezer.production_source_paths()

    assert not any(name.startswith("ray") for name in imported)
    assert not any(name.startswith("opensim") for name in imported)
    assert ".reset(" not in source
    assert ".step(" not in source
    assert not any(Path(path).name.startswith("test_") for path in paths)
    assert (
        freezer.contract.VALIDATION_ROOT / "build_h0_v12r6_composite_actor.py"
    ).as_posix() in paths
    assert (
        freezer.contract.VALIDATION_ROOT / "run_h0_v12r6_functional_composite.py"
    ).as_posix() in paths


def test_cli_requires_exactly_one_mode() -> None:
    parser = freezer._parser()  # noqa: SLF001

    with pytest.raises(SystemExit):
        parser.parse_args([])
    with pytest.raises(SystemExit):
        parser.parse_args(["--prepare", "--verify"])
    assert parser.parse_args(["--prepare"]).prepare is True
    assert parser.parse_args(["--verify"]).verify is True
