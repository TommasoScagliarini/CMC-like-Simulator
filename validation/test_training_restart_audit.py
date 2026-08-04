from __future__ import annotations

import json
import io
import sys
import tempfile
from pathlib import Path
import unittest
from contextlib import redirect_stdout


VALIDATION_DIR = Path(__file__).resolve().parent
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))

import audit_training_restarts as audit  # noqa: E402


class TrainingRestartAuditTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.run_dir = self.root / "run"
        self.log_dir = self.root / "ray" / "logs"
        self.run_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)
        self.pid = 4242

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _write_summary(self, **overrides) -> None:
        payload = {
            "ok": True,
            "stop_reason": "completed",
            "iterations_completed": 51,
            "num_env_runners": 12,
            "restart_count": 0,
            "crash_restart_count": 0,
            "crash_restarts": [],
            "skipped_iterations": [],
        }
        payload.update(overrides)
        (self.run_dir / "summary.json").write_text(
            json.dumps(payload), encoding="utf-8"
        )
        (self.run_dir / "watchdog_state.json").write_text(
            json.dumps({"pid": self.pid}), encoding="utf-8"
        )
        (self.run_dir / "train_iterations.jsonl").write_text(
            json.dumps({"iteration": 51}) + "\n", encoding="utf-8"
        )

    def _write_driver_log(self, *lines: str) -> Path:
        path = self.log_dir / (
            "python-core-driver-01000000ffffffffffffffffffffffffffffffff_"
            f"{self.pid}.log"
        )
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def _clean_log_lines(self) -> tuple[str, ...]:
        return (
            "Actor with class name: 'SingleAgentEnvRunner' and ID: 'abc' "
            "has constructor arguments in the object store and max_restarts > 0. "
            "If lost, the actor restart will fail.",
            "actor_manager: state: PENDING_CREATION, num_restarts: 0 actor_id=abc",
            "actor_manager: state: ALIVE, num_restarts: 0 actor_id=abc",
        )

    def test_clean_log_without_persisted_fault_metrics_passes(self) -> None:
        self._write_summary()
        self._write_driver_log(*self._clean_log_lines())

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertTrue(report["ok"])
        self.assertFalse(report["fault_tolerance_metrics"]["attested"])
        self.assertEqual(report["ray_driver_logs"]["finding_count"], 0)

    def test_positive_actor_restart_count_fails(self) -> None:
        self._write_summary()
        self._write_driver_log(
            *self._clean_log_lines(),
            "actor_manager: state: ALIVE, num_restarts: 1 actor_id=abc",
        )

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn(
            "no_ray_restart_or_failure_evidence", report["failed_checks"]
        )
        self.assertIn(
            "positive_actor_restart_count",
            {item["kind"] for item in report["ray_driver_logs"]["findings"]},
        )

    def test_ray_actor_error_fails(self) -> None:
        self._write_summary()
        self._write_driver_log(
            *self._clean_log_lines(),
            "ray.exceptions.RayActorError: actor died before finishing the task",
        )

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn(
            "ray_actor_error",
            {item["kind"] for item in report["ray_driver_logs"]["findings"]},
        )

    def test_explicit_env_runner_death_message_fails(self) -> None:
        self._write_summary()
        self._write_driver_log(
            *self._clean_log_lines(),
            "SingleAgentEnvRunner actor died unexpectedly while sampling",
        )

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn(
            "env_runner_failure_or_restart",
            {item["kind"] for item in report["ray_driver_logs"]["findings"]},
        )

    def test_terminal_dead_actor_notifications_are_expected_shutdown(self) -> None:
        self._write_summary()
        self._write_driver_log(
            *self._clean_log_lines(),
            "actor_manager: state: DEAD, num_restarts: 0 actor_id=abc",
            "RayletIpcClient::Disconnect, exit_type=INTENDED_USER_EXIT, "
            "exit_detail=Shutdown by ray.shutdown().",
        )

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertTrue(report["ok"])
        self.assertEqual(
            len(report["ray_driver_logs"]["terminal_dead_actor_notifications"]),
            1,
        )

    def test_early_dead_actor_notification_fails_closed(self) -> None:
        self._write_summary()
        self._write_driver_log(
            *self._clean_log_lines(),
            "actor_manager: state: DEAD, num_restarts: 0 actor_id=abc",
            *("unrelated driver activity" for _ in range(257)),
            "RayletIpcClient::Disconnect, exit_type=INTENDED_USER_EXIT, "
            "exit_detail=Shutdown by ray.shutdown().",
        )

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn(
            "dead_actor_outside_terminal_shutdown",
            {item["kind"] for item in report["ray_driver_logs"]["findings"]},
        )

    def test_supervisor_restart_fails_even_with_clean_ray_log(self) -> None:
        self._write_summary(
            restart_count=1,
            crash_restart_count=1,
            crash_restarts=[{"child_stop_reason": "error"}],
        )
        self._write_driver_log(*self._clean_log_lines())

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn("no_supervisor_restarts", report["failed_checks"])

    def test_missing_scoped_driver_log_fails_closed(self) -> None:
        self._write_summary()

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn(
            "scoped_ray_driver_log_available", report["failed_checks"]
        )

    def test_persisted_fault_tolerance_restart_fails(self) -> None:
        self._write_summary(
            fault_tolerance={
                "num_healthy_workers": 12,
                "num_remote_worker_restarts": 1,
            }
        )
        self._write_driver_log(*self._clean_log_lines())

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertTrue(report["fault_tolerance_metrics"]["attested"])
        self.assertIn(
            "fault_tolerance_restart_count_zero_if_present",
            report["failed_checks"],
        )

    def test_zero_fault_tolerance_metrics_and_healthy_workers_pass(self) -> None:
        self._write_summary(
            fault_tolerance={
                "num_healthy_workers": 12,
                "num_remote_worker_restarts": 0,
            }
        )
        self._write_driver_log(*self._clean_log_lines())

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertTrue(report["ok"])
        self.assertTrue(report["fault_tolerance_metrics"]["attested"])

    def test_malformed_supervisor_restart_list_fails_closed(self) -> None:
        self._write_summary(crash_restarts={"unexpected": "mapping"})
        self._write_driver_log(*self._clean_log_lines())

        report = audit.audit_training_restarts(self.run_dir, self.log_dir)

        self.assertFalse(report["ok"])
        self.assertIn("no_supervisor_restarts", report["failed_checks"])

    def test_cli_writes_atomic_json_contract_and_returns_success(self) -> None:
        self._write_summary()
        self._write_driver_log(*self._clean_log_lines())
        output = self.root / "audit.json"

        with redirect_stdout(io.StringIO()):
            return_code = audit.main(
                [
                    "--run-dir",
                    str(self.run_dir),
                    "--ray-log-dir",
                    str(self.log_dir),
                    "--output",
                    str(output),
                ]
            )

        payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(return_code, 0)
        self.assertTrue(payload["ok"])
        self.assertFalse(any(output.parent.glob(f".{output.name}.*.tmp")))


if __name__ == "__main__":
    unittest.main()
