from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from validation import readiness_gatekeeper as gate


PROTOCOL_DIGEST = "a" * 64


def _record(name: str, digest: str = PROTOCOL_DIGEST) -> dict[str, object]:
    return {"path": name, "sha256": digest}


def _gate_payload(
    *,
    step_id: str = "0",
    status: str = "PASS",
    prerequisite_locks: list[dict[str, object]] | None = None,
) -> dict[str, object]:
    return {
        "schema_version": 1,
        "step_id": step_id,
        "status": status,
        "protocol_sha256": PROTOCOL_DIGEST,
        "source_hashes_match": status == "PASS",
        "prerequisite_locks": prerequisite_locks or [],
        "data_access_receipts": [_record("receipt.json", "b" * 64)],
        "checks": [{"id": "synthetic", "status": status}],
        "failed_checks": [] if status == "PASS" else ["synthetic"],
        "artifacts": [_record("artifact.json", "c" * 64)],
        "decision": gate.CANONICAL_DECISIONS[step_id],
        "protocol": _record("protocol.json"),
        "source_hashes": [_record("source.dat", "d" * 64)],
        "candidate": _record("candidate.dat", "e" * 64),
    }


def _materialized_gate_payload(
    root: Path,
    *,
    step_id: str,
    tag: str = "",
    status: str = "PASS",
    prerequisite_locks: list[dict[str, object]] | None = None,
) -> tuple[dict[str, object], Path]:
    suffix = f"_{tag}" if tag else ""
    files: dict[str, Path] = {}
    for role in ("protocol", "source", "candidate", "receipt", "artifact"):
        path = root / f"{role}{step_id}{suffix}.dat"
        path.write_bytes(f"{role}-{step_id}-{tag}".encode("utf-8"))
        files[role] = path
    payload = _gate_payload(
        step_id=step_id,
        status=status,
        prerequisite_locks=prerequisite_locks,
    )
    payload["protocol_sha256"] = gate.sha256_file(files["protocol"])
    payload["protocol"] = gate.source_record(files["protocol"], root)
    payload["source_hashes"] = [gate.source_record(files["source"], root)]
    payload["candidate"] = gate.source_record(files["candidate"], root)
    payload["data_access_receipts"] = [
        gate.source_record(files["receipt"], root)
    ]
    payload["artifacts"] = [gate.source_record(files["artifact"], root)]
    return payload, files["source"]


class StrictJSONTests(unittest.TestCase):
    def test_rejects_duplicate_keys_at_any_depth(self) -> None:
        for raw in (
            '{"status":"PASS","status":"FAIL"}',
            '{"outer":{"value":1,"value":2}}',
        ):
            with self.subTest(raw=raw):
                with self.assertRaises(gate.StrictJSONError):
                    gate.loads_json_strict(raw)

    def test_rejects_nonfinite_tokens_and_numeric_overflow(self) -> None:
        for raw in ("NaN", "Infinity", "-Infinity", "1e999", '{"x":-1e999}'):
            with self.subTest(raw=raw):
                with self.assertRaises(gate.StrictJSONError):
                    gate.loads_json_strict(raw)

    def test_strict_file_loader_requires_utf8_json(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "invalid.json"
            path.write_bytes(b"\xff")
            with self.assertRaises(gate.StrictJSONError):
                gate.load_json_strict(path)

    def test_accepts_finite_strict_json(self) -> None:
        value = gate.loads_json_strict('{"a":[1,2.5,true,null],"b":{"c":"ok"}}')
        self.assertEqual(value["a"][1], 2.5)


class GateSchemaTests(unittest.TestCase):
    def test_accepts_bound_pass_schema_v1(self) -> None:
        payload = _gate_payload()
        self.assertEqual(gate.validate_gate_payload(payload), payload)

    def test_requires_every_contract_field(self) -> None:
        for field in sorted(gate.REQUIRED_GATE_FIELDS):
            payload = _gate_payload()
            del payload[field]
            with self.subTest(field=field):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_rejects_invalid_version_step_status_and_hash(self) -> None:
        mutations = (
            ("schema_version", True),
            ("schema_version", 2),
            ("step_id", 0),
            ("step_id", "3"),
            ("step_id", []),
            ("status", "RUNNING"),
            ("status", "PASS_WITH_WARNINGS"),
            ("status", []),
            ("protocol_sha256", "ABC"),
            ("source_hashes_match", 1),
        )
        for field, value in mutations:
            payload = _gate_payload()
            payload[field] = value
            with self.subTest(field=field, value=value):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_pass_requires_matching_sources_and_no_failures(self) -> None:
        payload = _gate_payload()
        payload["source_hashes_match"] = False
        with self.assertRaises(gate.GateSchemaError):
            gate.validate_gate_payload(payload)

        payload = _gate_payload()
        payload["failed_checks"] = ["not_really_pass"]
        with self.assertRaises(gate.GateSchemaError):
            gate.validate_gate_payload(payload)

    def test_pass_requires_canonical_decision_and_all_provenance_bindings(
        self,
    ) -> None:
        for step_id, decision in gate.CANONICAL_DECISIONS.items():
            payload = _gate_payload(step_id=step_id)
            payload["decision"] = f"NOT_{decision}"
            with self.subTest(step_id=step_id, field="decision"):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

        mutations = (
            ("decision", None),
            ("protocol", None),
            ("source_hashes", None),
            ("source_hashes", []),
            ("candidate", None),
            ("data_access_receipts", []),
        )
        for field, replacement in mutations:
            payload = _gate_payload()
            if replacement is None:
                del payload[field]
            else:
                payload[field] = replacement
            with self.subTest(field=field, replacement=replacement):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_receipts_checks_and_artifacts_have_strict_record_shapes(self) -> None:
        mutations = (
            ("data_access_receipts", ["receipt.json"]),
            ("data_access_receipts", [{"path": "receipt.json"}]),
            ("checks", ["passed"]),
            ("checks", [{"id": "x", "status": "WARNING"}]),
            ("checks", [{"id": "", "status": "PASS"}]),
            ("failed_checks", [{}]),
            ("artifacts", ["summary.json"]),
            ("artifacts", [{"path": "summary.json"}]),
        )
        for field, replacement in mutations:
            payload = _gate_payload()
            payload[field] = replacement
            with self.subTest(field=field):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_pass_requires_nonempty_checks_and_artifacts(self) -> None:
        for field in ("checks", "artifacts"):
            payload = _gate_payload()
            payload[field] = []
            with self.subTest(field=field):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_failed_checks_exactly_reference_non_pass_checks(self) -> None:
        payload = _gate_payload(status="ERROR")
        payload["checks"] = [
            {"id": "completed", "status": "PASS"},
            {"id": "crashed", "status": "ERROR"},
        ]
        payload["failed_checks"] = ["crashed"]
        gate.validate_gate_payload(payload)
        payload["failed_checks"] = ["completed"]
        with self.assertRaises(gate.GateSchemaError):
            gate.validate_gate_payload(payload)

    def test_failure_status_requires_a_failed_check(self) -> None:
        for status in ("FAIL", "BLOCKED", "ERROR"):
            payload = _gate_payload(status=status)
            payload["failed_checks"] = []
            with self.subTest(status=status):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_rejects_non_array_contract_fields(self) -> None:
        for field in (
            "prerequisite_locks",
            "data_access_receipts",
            "checks",
            "failed_checks",
            "artifacts",
        ):
            payload = _gate_payload()
            payload[field] = {}
            with self.subTest(field=field):
                with self.assertRaises(gate.GateSchemaError):
                    gate.validate_gate_payload(payload)

    def test_protocol_record_must_bind_protocol_sha(self) -> None:
        payload = _gate_payload()
        payload["protocol"] = {"path": "protocol.json", "sha256": "b" * 64}
        with self.assertRaises(gate.GateSchemaError):
            gate.validate_gate_payload(payload)


class HashBindingTests(unittest.TestCase):
    def test_source_record_and_verification_detect_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "inputs" / "source.bin"
            source.parent.mkdir()
            source.write_bytes(b"frozen")
            record = gate.source_record(source, root)
            self.assertEqual(record["path"], "inputs/source.bin")
            self.assertTrue(gate.verify_source_hashes([record], root))

            source.write_bytes(b"tampered")
            with self.assertRaises(gate.HashVerificationError):
                gate.verify_source_hashes([record], root)

    def test_rejects_absolute_escape_and_duplicate_source_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.bin"
            source.write_bytes(b"x")
            record = gate.source_record(source, root)
            with self.assertRaises(gate.HashVerificationError):
                gate.resolve_repo_relative_path("../outside", root)
            with self.assertRaises(gate.HashVerificationError):
                gate.resolve_repo_relative_path(source, root)
            with self.assertRaises(gate.HashVerificationError):
                gate.resolve_repo_relative_path(r"C:\outside", root)
            with self.assertRaises(gate.HashVerificationError):
                gate.verify_source_hashes([record, record], root)

    def test_embedded_protocol_sources_and_candidate_are_rehashed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload, _source = _materialized_gate_payload(root, step_id="0")
            candidate = gate.resolve_repo_relative_path(
                payload["candidate"]["path"],
                root,
            )
            self.assertTrue(gate.verify_gate_bindings(payload, root))

            candidate.write_bytes(b"changed")
            with self.assertRaises(gate.HashVerificationError):
                gate.verify_gate_bindings(payload, root)

    def test_embedded_data_access_receipt_and_artifact_are_rehashed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload, _source = _materialized_gate_payload(root, step_id="0")
            receipt = gate.resolve_repo_relative_path(
                payload["data_access_receipts"][0]["path"],
                root,
            )
            receipt.write_bytes(b"tampered receipt")
            with self.assertRaises(gate.HashVerificationError):
                gate.verify_gate_bindings(payload, root)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload, _source = _materialized_gate_payload(root, step_id="0")
            artifact = gate.resolve_repo_relative_path(
                payload["artifacts"][0]["path"],
                root,
            )
            artifact.write_bytes(b"tampered artifact")
            with self.assertRaises(gate.HashVerificationError):
                gate.verify_gate_bindings(payload, root)


class PrerequisiteTests(unittest.TestCase):
    def _write_step_zero(
        self,
        root: Path,
        *,
        status: str = "PASS",
        tag: str = "",
    ) -> tuple[Path, dict[str, object], Path]:
        suffix = f"_{tag}" if tag else ""
        lock = root / f"step0{suffix}.json"
        payload, source = _materialized_gate_payload(
            root,
            step_id="0",
            tag=tag,
            status=status,
        )
        gate.write_gate_status_no_clobber(lock, payload)
        return lock, gate.source_record(lock, root), source

    def test_verifies_pass_lock_hash_status_decision_and_live_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _lock, reference, source = self._write_step_zero(root)
            reference["step_id"] = "0"
            reference["decision"] = "PRIMARY_GRF_READY"
            verified = gate.verify_prerequisite_lock(reference, root)
            self.assertEqual(verified["status"], "PASS")

            source.write_bytes(b"post-pass-drift")
            with self.assertRaises(gate.PrerequisiteLockError):
                gate.verify_prerequisite_lock(reference, root)

    def test_rejects_tampered_or_non_pass_lock(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            lock, reference, _source = self._write_step_zero(root)
            lock.write_text("{}\n", encoding="utf-8")
            with self.assertRaises(gate.PrerequisiteLockError):
                gate.verify_prerequisite_lock(reference, root)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _lock, reference, _source = self._write_step_zero(
                root,
                status="BLOCKED",
            )
            with self.assertRaises(gate.PrerequisiteLockError):
                gate.verify_prerequisite_lock(reference, root)

    def test_step_one_gate_requires_complete_step_zero_chain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _lock0, reference0, _source = self._write_step_zero(root)
            step1, _source1 = _materialized_gate_payload(
                root,
                step_id="1",
                prerequisite_locks=[reference0],
            )
            lock1 = root / "step1.json"
            gate.write_gate_status_no_clobber(lock1, step1)
            verified = gate.verify_gate_file(
                lock1,
                root,
                expected_step_id="1",
                require_pass=True,
            )
            self.assertEqual(verified["step_id"], "1")

            missing_chain = root / "step1_missing.json"
            missing_payload, _missing_source = _materialized_gate_payload(
                root,
                step_id="1",
                tag="missing",
            )
            gate.write_gate_status_no_clobber(
                missing_chain,
                missing_payload,
            )
            with self.assertRaises(gate.PrerequisiteLockError):
                gate.verify_gate_file(missing_chain, root, require_pass=True)

    def test_step_two_requires_direct_step_zero_and_step_one_locks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _lock0, reference0, _source = self._write_step_zero(root)
            lock1 = root / "step1.json"
            step1, _source1 = _materialized_gate_payload(
                root,
                step_id="1",
                prerequisite_locks=[reference0],
            )
            gate.write_gate_status_no_clobber(
                lock1,
                step1,
            )
            reference1 = gate.source_record(lock1, root)
            lock2 = root / "step2.json"
            step2, _source2 = _materialized_gate_payload(
                root,
                step_id="2",
                prerequisite_locks=[reference0, reference1],
            )
            gate.write_gate_status_no_clobber(
                lock2,
                step2,
            )
            self.assertEqual(
                gate.verify_gate_file(lock2, root, require_pass=True)["step_id"],
                "2",
            )

            only_step1 = root / "step2_incomplete.json"
            incomplete, _source_incomplete = _materialized_gate_payload(
                root,
                step_id="2",
                tag="incomplete",
                prerequisite_locks=[reference1],
            )
            gate.write_gate_status_no_clobber(
                only_step1,
                incomplete,
            )
            with self.assertRaises(gate.PrerequisiteLockError):
                gate.verify_gate_file(only_step1, root, require_pass=True)

    def test_prerequisite_pass_requires_its_own_canonical_decision(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload, _source = _materialized_gate_payload(root, step_id="0")
            payload["decision"] = "H0_SEP_READY"
            lock = root / "wrong_decision_step0.json"
            gate.write_json_no_clobber(lock, payload)
            reference = gate.source_record(lock, root)
            with self.assertRaises(gate.PrerequisiteLockError):
                gate.verify_prerequisite_lock(reference, root)

    def test_step_two_rejects_mixed_direct_and_transitive_step_zero(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _lock_a, reference_a, _source_a = self._write_step_zero(
                root,
                tag="a",
            )
            _lock_b, reference_b, _source_b = self._write_step_zero(
                root,
                tag="b",
            )
            step1, _source1 = _materialized_gate_payload(
                root,
                step_id="1",
                prerequisite_locks=[reference_a],
            )
            lock1 = root / "step1_from_a.json"
            gate.write_gate_status_no_clobber(lock1, step1)
            reference1 = gate.source_record(lock1, root)
            mixed, _source2 = _materialized_gate_payload(
                root,
                step_id="2",
                prerequisite_locks=[reference_b, reference1],
            )
            lock2 = root / "step2_mixed.json"
            gate.write_gate_status_no_clobber(lock2, mixed)
            with self.assertRaisesRegex(
                gate.PrerequisiteLockError,
                "incoherent prerequisite lineage",
            ):
                gate.verify_gate_file(lock2, root, require_pass=True)


class NoClobberAndFailureTests(unittest.TestCase):
    def test_json_output_is_strict_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "artifact.json"
            gate.write_json_no_clobber(path, {"value": 1})
            before = path.read_bytes()
            with self.assertRaises(gate.NoClobberError):
                gate.write_json_no_clobber(path, {"value": 2})
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(json.loads(before), {"value": 1})

    def test_nonfinite_payload_is_rejected_before_destination_creation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            with self.assertRaises(gate.StrictJSONError):
                gate.write_json_no_clobber(path, {"value": float("nan")})
            self.assertFalse(path.exists())

    def test_output_directory_claim_is_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            self.assertEqual(gate.claim_output_directory(output), output)
            with self.assertRaises(gate.NoClobberError):
                gate.claim_output_directory(output)

    def test_failure_helper_records_phase_access_checks_and_artifacts(self) -> None:
        payload = gate.build_failure_payload(
            step_id="0",
            status="ERROR",
            protocol_sha256=PROTOCOL_DIGEST,
            phase="prescribed_replay",
            last_data_access={
                "trial_id": "02",
                "semantic_access_started": True,
            },
            failed_checks=["unexpected_exception"],
            artifacts=[_record("trace.csv", "f" * 64)],
            source_hashes_match=True,
            error="synthetic",
        )
        self.assertEqual(payload["phase"], "prescribed_replay")
        self.assertEqual(payload["last_data_access"]["trial_id"], "02")
        self.assertEqual(payload["failed_checks"], ["unexpected_exception"])
        self.assertEqual(
            payload["checks"],
            [{"id": "unexpected_exception", "status": "ERROR"}],
        )
        self.assertEqual(payload["artifacts_preserved"], payload["artifacts"])

    def test_failure_last_data_access_requires_explicit_boolean(self) -> None:
        for access in ({}, {"semantic_access_started": 0}):
            with self.subTest(access=access):
                with self.assertRaises(gate.GateSchemaError):
                    gate.build_failure_payload(
                        step_id="0",
                        status="ERROR",
                        protocol_sha256=PROTOCOL_DIGEST,
                        phase="preflight",
                        last_data_access=access,
                        failed_checks=["invalid_access_receipt"],
                        artifacts=[],
                        source_hashes_match=False,
                    )

    def test_failure_writer_is_immutable_and_rejects_pass_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            path = gate.write_failure_json(
                output,
                step_id="2",
                status="BLOCKED",
                protocol_sha256=PROTOCOL_DIGEST,
                phase="preflight",
                last_data_access=None,
                failed_checks=["missing_prerequisite"],
                artifacts=[],
                source_hashes_match=False,
            )
            self.assertEqual(path.name, "failure.json")
            with self.assertRaises(gate.NoClobberError):
                gate.write_failure_json(
                    output,
                    step_id="2",
                    status="BLOCKED",
                    protocol_sha256=PROTOCOL_DIGEST,
                    phase="preflight",
                    last_data_access=None,
                    failed_checks=["second_attempt"],
                    artifacts=[],
                    source_hashes_match=False,
                )

        with self.assertRaises(gate.GateSchemaError):
            gate.build_failure_payload(
                step_id="0",
                status="PASS",
                protocol_sha256=PROTOCOL_DIGEST,
                phase="invalid",
                last_data_access=None,
                failed_checks=["invalid"],
                artifacts=[],
                source_hashes_match=True,
            )


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
