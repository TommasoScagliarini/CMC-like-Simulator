from __future__ import annotations

import copy
import contextlib
import io
import json
import unittest
from pathlib import Path

from validation import build_pilot_evidence_table as evidence
from validation import ppo_pilot_screen as screen
from validation import test_ppo_pilot_screen as screen_test_helpers


class PilotEvidenceTableTests(unittest.TestCase):
    def setUp(self) -> None:
        # Reuse the full, production-compatible pilot fixture rather than a
        # weaker hand-written approximation of the screen report contract.
        self.fixture = screen_test_helpers.PilotScreenTests(
            "test_full_screen_runs_critical_first_and_never_touches_held_out"
        )
        self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        for record in self.fixture.records:
            record["episode_return_mean"] = float(record["iteration"])
            record["episode_len_mean"] = 500.0
        self.fixture._write_history(self.fixture.records)
        self.fixture._write_run_summary()
        _, runner = self.fixture._runner()
        self.screen_report = screen.screen_pilot(
            self.fixture._config("evidence_screen"),
            command_runner=runner,
        )
        self.screen_path = self.fixture.root / "evidence_screen" / screen.REPORT_FILENAME
        self.drift_path = self.fixture.root / "policy_drift_from_h0_milestones.json"
        self._write_json(self.drift_path, self._drift_report())
        self.addendum_path = self.fixture.root / "selection_addendum.json"
        self._write_json(self.addendum_path, self._selection_addendum())
        self.output_path = self.fixture.root / "candidate_evidence.json"

    @staticmethod
    def _write_json(path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _drift_report(self) -> dict:
        roles = list(evidence.EXPECTED_TRACE_ROLES)
        offsets = (
            self.fixture._protocol_payload()["sampling"]["start_offsets_s"]
        )
        trace_offsets = [offsets[0], offsets[1], offsets[2], offsets[2]]
        traces = []
        for index, role in enumerate(roles):
            traces.append(
                {
                    "path": str(self.fixture.root / "development_traces" / role / "rollout_policy_trace.json"),
                    "sha256": f"{index + 1:064x}",
                    "protocol_role": role,
                    "action_selection": "stochastic" if index == 3 else "deterministic",
                    "episode_start_offset_s": trace_offsets[index],
                    "action_seed": 123,
                }
            )
        rows = []
        for iteration in evidence.EXPECTED_ALL_LOGICAL_ITERATIONS:
            milestone = self.fixture.run_dir / f"{screen.MILESTONE_PREFIX}{iteration:06d}"
            rows.append(
                {
                    "logical_iteration": iteration,
                    "milestone": str(milestone),
                    "rl_module": str(milestone / "rl_module_last"),
                    "status": "PASS",
                    "logstd_bit_exact": True,
                    "metrics_finite": True,
                    "action_mean_rmse": iteration * 1.0e-4,
                    "action_mean_abs_max": iteration * 2.0e-4,
                    "kl_reference_to_candidate_mean": iteration * 1.0e-6,
                    "kl_reference_to_candidate_max": iteration * 2.0e-6,
                    "reference_actor_digest": self.fixture.reference_actor_digest,
                    "candidate_actor_digest": f"{iteration:064x}",
                    "per_development_trace": [
                        {"trace": trace["path"]} for trace in traces
                    ],
                }
            )
        return {
            "schema_version": 1,
            "audit": "cumulative_policy_drift_from_h0",
            "ok": True,
            "reference_h0_rl_module": str(self.fixture.reference_checkpoint),
            "reference_h0_actor_digest": self.fixture.reference_actor_digest,
            "training_run": str(self.fixture.run_dir),
            "heldout_seeds_excluded": [126, 127, 128],
            "development_traces": traces,
            "milestone_count": 50,
            "failed_logical_iterations": [],
            "milestones": rows,
        }

    def _selection_addendum(self) -> dict:
        return {
            "schema_version": 1,
            "status": "preregistered_selection_interpretation_addendum",
            "created_at": "2026-07-15T03:05:19+02:00",
            "immutable": True,
            "parent_protocol": {
                "path": str(self.fixture.protocol),
                "sha256": self.fixture._digest(self.fixture.protocol),
            },
            "scope": "clarify_only_the_existing_cumulative_empirical_kl_tie_breaker",
            "selection_interpretation": {
                "cumulative_means": "direct_H0_to_current_milestone_drift_not_the_sum_of_incremental_update_KLs",
                "source_report": "policy_drift_from_h0_milestones.json",
                "source_field": "milestones[].kl_reference_to_candidate_mean",
                "statistic": "fixed_observation_aggregate.empirical_kl_reference_to_candidate_mean",
                "development_trace_set": "the_four_preregistered_fixed_H0_traces",
                "ordering": "ascending",
                "candidate_scope": "only_milestones_eligible_under_every_development_gate",
                "empirical_kl_max_role": "diagnostic_only_not_a_selection_tie_breaker",
            },
            "unchanged_contract": {
                "primary_order": "minimum worst condition-matched reserve ratio versus H0",
                "first_tie_breaker": "minimum worst penetration ratio versus H0",
                "second_tie_breaker": "minimum cumulative empirical KL versus H0",
                "third_tie_breaker": "earlier pilot update",
                "automatic_selection_or_promotion": False,
                "held_out_seeds": [126, 127, 128],
                "held_out_status": "sealed",
            },
            "timing_attestation": {
                "training_status": "active",
                "latest_completed_logical_iteration": 9,
                "completed_new_actor_updates": 8,
                "development_screening_started": False,
                "held_out_opened": False,
            },
        }

    def _build(
        self,
        *,
        screen_path: Path | None = None,
        addendum_path: Path | None = None,
    ) -> dict:
        return evidence.build_evidence_table(
            protocol_path=self.fixture.protocol,
            selection_addendum_path=addendum_path or self.addendum_path,
            screen_report_path=screen_path or self.screen_path,
            drift_report_path=self.drift_path,
            output_path=self.output_path,
        )

    def test_complete_table_joins_exactly_eight_milestones_without_selection(self) -> None:
        table = self._build()

        self.assertTrue(table["ok"])
        self.assertEqual(table["status"], "COMPLETE")
        self.assertEqual(table["row_count"], 8)
        self.assertEqual(
            [row["logical_iteration"] for row in table["rows"]],
            list(screen.EXPECTED_SCREENED_LOGICAL_ITERATIONS),
        )
        self.assertTrue(table["contract"]["held_out_data_read"] is False)
        self.assertFalse(table["contract"]["ranking_performed"])
        self.assertFalse(table["contract"]["checkpoint_selected"])
        self.assertFalse(table["contract"]["checkpoint_copied"])
        self.assertFalse(table["contract"]["checkpoint_promoted"])
        self.assertFalse(
            table["preregistered_candidate_ordering"]["ordering_performed"]
        )
        self.assertEqual(
            table["preregistered_candidate_ordering"][
                "cumulative_kl_interpretation"
            ]["ordering"],
            "ascending",
        )
        self.assertEqual(
            table["inputs"]["selection_addendum"]["path"],
            str(self.addendum_path),
        )
        self.assertEqual(
            table["inputs"]["selection_addendum"]["sha256"],
            self.fixture._digest(self.addendum_path),
        )
        self.assertIsNone(
            table["preregistered_candidate_ordering"]["selected_logical_iteration"]
        )
        first = table["rows"][0]
        self.assertEqual(first["screen_eligibility"], "ELIGIBLE")
        self.assertEqual(first["robustness"]["four_case_status"], "PASS")
        self.assertEqual(len(first["robustness"]["cases"]), 4)
        self.assertIn("episode_return_mean", first["training"])
        self.assertIn("action_mean_rmse", first["policy_drift_from_h0"])
        self.assertTrue(first["preregistered_ordering_keys"]["eligible_for_ordering"])
        self.assertIsNotNone(
            first["preregistered_ordering_keys"][
                "primary_worst_condition_matched_reserve_ratio_vs_h0"
            ]
        )
        reserve = first["robustness"]["cases"][0]["reserve"]
        self.assertAlmostEqual(
            reserve["margin_vs_h0_nm"],
            reserve["reference_h0_nm"] - reserve["observed_nm"],
        )

        evidence._write_json_atomic_no_clobber(self.output_path, table)
        persisted = json.loads(self.output_path.read_text(encoding="utf-8"))
        self.assertEqual(persisted["rows"], table["rows"])

    def test_completed_physical_rejections_remain_valid_evidence(self) -> None:
        _, failing_runner = self.fixture._runner(fail_all_critical=True)
        failed_report = screen.screen_pilot(
            self.fixture._config("evidence_physical_fail"),
            command_runner=failing_runner,
        )
        failed_path = self.fixture.root / "evidence_physical_fail" / screen.REPORT_FILENAME
        self.assertTrue(failed_report["screening_completed"])
        self.assertFalse(failed_report["ok"])
        self.assertEqual(failed_report["operational_failures"], [])

        table = self._build(screen_path=failed_path)

        self.assertEqual(table["eligible_logical_iterations"], [])
        self.assertEqual(table["row_count"], 8)
        for row in table["rows"]:
            self.assertEqual(row["screen_eligibility"], "REJECTED")
            self.assertEqual(row["robustness"]["four_case_status"], "FAIL")
            self.assertFalse(
                row["preregistered_ordering_keys"]["eligible_for_ordering"]
            )
            self.assertIsNone(
                row["preregistered_ordering_keys"][
                    "primary_worst_condition_matched_reserve_ratio_vs_h0"
                ]
            )
            cases = row["robustness"]["cases"]
            self.assertEqual([case["execution"] for case in cases[:3]], ["NOT_RUN"] * 3)
            self.assertEqual(cases[3]["execution"], "RUN")
            self.assertEqual(cases[3]["gate_status"], "FAIL")

    def test_incomplete_or_operational_screen_is_refused_without_output(self) -> None:
        for field, value in (("screening_completed", False), ("operational_failures", [11])):
            with self.subTest(field=field):
                report = copy.deepcopy(self.screen_report)
                report[field] = value
                path = self.fixture.root / f"bad_{field}.json"
                self._write_json(path, report)
                with self.assertRaises(evidence.EvidenceInputError):
                    self._build(screen_path=path)
                self.assertFalse(self.output_path.exists())

    def test_incomplete_drift_report_is_refused_without_output(self) -> None:
        report = self._drift_report()
        report["milestones"].pop()
        report["milestone_count"] = 49
        self._write_json(self.drift_path, report)

        with self.assertRaises(evidence.EvidenceInputError):
            self._build()

        self.assertFalse(self.output_path.exists())

    def test_heldout_drift_trace_is_refused_without_reading_it(self) -> None:
        report = self._drift_report()
        report["development_traces"][3]["action_seed"] = 126
        self._write_json(self.drift_path, report)

        with self.assertRaisesRegex(evidence.EvidenceInputError, "held-out"):
            self._build()

        self.assertFalse(self.output_path.exists())

    def test_existing_output_is_refused_and_preserved(self) -> None:
        self.output_path.write_text("sentinel", encoding="utf-8")

        with self.assertRaisesRegex(evidence.EvidenceInputError, "existing output"):
            self._build()

        self.assertEqual(self.output_path.read_text(encoding="utf-8"), "sentinel")

    def test_missing_selection_addendum_is_refused_without_output(self) -> None:
        missing = self.fixture.root / "missing_selection_addendum.json"

        with self.assertRaisesRegex(
            evidence.EvidenceInputError, "selection addendum"
        ):
            self._build(addendum_path=missing)

        self.assertFalse(self.output_path.exists())

    def test_cli_requires_explicit_selection_addendum(self) -> None:
        parser = evidence.build_parser()
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(
                    [
                        "--protocol",
                        str(self.fixture.protocol),
                        "--screen-report",
                        str(self.screen_path),
                        "--drift-report",
                        str(self.drift_path),
                        "--output",
                        str(self.output_path),
                    ]
                )

        parsed = parser.parse_args(
            [
                "--protocol",
                str(self.fixture.protocol),
                "--selection-addendum",
                str(self.addendum_path),
                "--screen-report",
                str(self.screen_path),
                "--drift-report",
                str(self.drift_path),
                "--output",
                str(self.output_path),
            ]
        )
        self.assertEqual(parsed.selection_addendum, self.addendum_path)

    def test_selection_addendum_contract_mismatches_are_refused(self) -> None:
        variants = []

        wrong_parent_path = self._selection_addendum()
        wrong_parent_path["parent_protocol"]["path"] = str(
            self.fixture.root / "another_protocol.json"
        )
        variants.append(("parent_path", wrong_parent_path, "parent-protocol path"))

        wrong_parent_sha = self._selection_addendum()
        wrong_parent_sha["parent_protocol"]["sha256"] = "0" * 64
        variants.append(("parent_sha", wrong_parent_sha, "parent-protocol SHA-256"))

        mutable = self._selection_addendum()
        mutable["immutable"] = False
        variants.append(("mutable", mutable, "not immutable"))

        wrong_status = self._selection_addendum()
        wrong_status["status"] = "draft"
        variants.append(("status", wrong_status, "wrong status"))

        wrong_field = self._selection_addendum()
        wrong_field["selection_interpretation"]["source_field"] = (
            "milestones[].kl_reference_to_candidate_max"
        )
        variants.append(("field", wrong_field, "field/statistic/ordering"))

        wrong_statistic = self._selection_addendum()
        wrong_statistic["selection_interpretation"]["statistic"] = "maximum"
        variants.append(("statistic", wrong_statistic, "field/statistic/ordering"))

        wrong_ordering = self._selection_addendum()
        wrong_ordering["selection_interpretation"]["ordering"] = "descending"
        variants.append(("ordering", wrong_ordering, "field/statistic/ordering"))

        opened_heldout = self._selection_addendum()
        opened_heldout["timing_attestation"]["held_out_opened"] = True
        variants.append(("heldout", opened_heldout, "timing or held-out sealing"))

        for name, payload, error in variants:
            with self.subTest(name=name):
                path = self.fixture.root / f"bad_addendum_{name}.json"
                self._write_json(path, payload)
                with self.assertRaisesRegex(evidence.EvidenceInputError, error):
                    self._build(addendum_path=path)
                self.assertFalse(self.output_path.exists())

    def test_output_inside_training_run_is_refused_without_mutation(self) -> None:
        unsafe_output = self.fixture.run_dir / "candidate_evidence.json"

        with self.assertRaisesRegex(
            evidence.EvidenceInputError, "inside the immutable training run"
        ):
            evidence.build_evidence_table(
                protocol_path=self.fixture.protocol,
                selection_addendum_path=self.addendum_path,
                screen_report_path=self.screen_path,
                drift_report_path=self.drift_path,
                output_path=unsafe_output,
            )

        self.assertFalse(unsafe_output.exists())


if __name__ == "__main__":
    unittest.main()
