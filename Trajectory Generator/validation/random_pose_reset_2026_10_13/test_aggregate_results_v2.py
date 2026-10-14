"""Offline synthetic cases only. No new campaign outcomes or native imports."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("random_pose_aggregate", HERE / "aggregate_results_v2.py")
M = importlib.util.module_from_spec(spec)
spec.loader.exec_module(M)


def fixture():
    times = [i / 1000 for i in range(5000)]
    actual = {"time": times, "pros_knee_angle": [-0.5 + 0.35 * math.sin(2 * math.pi * t) for t in times],
              "pros_ankle_angle": [0.2 * math.sin(2 * math.pi * t) for t in times]}
    reference = {"time": times, **{j + "_q_ref": list(actual[j]) for j in M.JOINTS}}
    grf = {"time": times, "left_penetration": [0.025] * len(times)}
    trace = []
    for i in range(1, 501):
        phase = {k: 0.0 for k in [*M.CRITERIA["zero_phase_flags"], *M.CRITERIA["zero_phase_counters"]]}
        phase.update(valid_cycle_count=float((i >= 101) + (i >= 201)), accepted_transitions_this_step=[])
        trace.append({"step": i, "time": i / 100, "phase_fsm": phase})
    for step, kind, onset, valid in [(11, "heel_strike", 0.1, -1), (31, "toe_off", 0.3, -1),
                                     (101, "heel_strike", 1.0, 1), (131, "toe_off", 1.3, -1),
                                     (201, "heel_strike", 2.0, 1)]:
        trace[step - 1]["phase_fsm"]["accepted_transitions_this_step"] = [
            {"event": kind, "event_time_s": onset, "confirmed_time_s": onset + 0.005, "cycle_valid": valid}]
    receipt = {"actual_start_s": 0.0, "actual_end_s": 5.0, "integration_steps": 500,
               "static_contact_gate": {"max_applied_penetration_m": 0.025}}
    summary = {"steps": 500, "end_reason": "episode_time_limit", "terminated": False, "phase_valid_cycle_count": 2}
    return receipt, summary, trace, actual, reference, grf


class TestBinomialConfidence(unittest.TestCase):
    def test_all_success_and_no_success_closed_forms(self):
        d = M.cp_interval(20, 20)
        self.assertAlmostEqual(d["two_sided95"][0], 0.025 ** (1 / 20), places=13)
        self.assertAlmostEqual(d["one_sided95_lower"], 0.05 ** (1 / 20), places=13)
        self.assertEqual(d["two_sided95"][1], 1)
        z = M.cp_interval(0, 20)
        self.assertEqual(z["two_sided95"][0], 0)
        self.assertAlmostEqual(z["two_sided95"][1], 1 - 0.025 ** (1 / 20), places=13)

    def test_monotonic_and_symmetric(self):
        intervals = [M.cp_interval(k, 20)["two_sided95"] for k in range(21)]
        self.assertTrue(all(a[0] <= b[0] and a[1] <= b[1] for a, b in zip(intervals, intervals[1:])))
        for k in range(21):
            self.assertAlmostEqual(intervals[k][0], 1 - intervals[20 - k][1], places=12)

    def test_invalid_n_k_and_alpha(self):
        for k, n, alpha in [(0, 0, .05), (2, 1, .05), (-1, 20, .05), (1.5, 20, .05), (0, 20, 0)]:
            with self.assertRaises(ValueError):
                M.cp_interval(k, n, alpha)


class TestPreregisteredMeasurements(unittest.TestCase):
    def test_good_case_has_two_distinct_qualifying_cycles(self):
        result = M.measurements(*fixture())
        self.assertTrue(all(result["gates"].values()))
        self.assertEqual(result["qualifying_cycle_count"], 2)
        self.assertAlmostEqual(result["first_hs_confirmed_elapsed_s"], .105)
        self.assertEqual(result["window_samples"]["post050"], 4500)

    def test_one_large_cycle_cannot_mask_one_small_cycle(self):
        args = fixture()
        actual, ref = args[3], args[4]
        actual["pros_knee_angle"] = [-0.5 + (0.1 if t < 1 else .35) * math.sin(2 * math.pi * t) for t in actual["time"]]
        ref["pros_knee_angle_q_ref"] = list(actual["pros_knee_angle"])
        result = M.measurements(*args)
        self.assertEqual(result["qualifying_cycle_count"], 1)
        self.assertFalse(result["gates"]["two_cycles_each_both_excursions"])

    def test_joint_maxima_from_different_cycles_do_not_count(self):
        args = fixture()
        a, ref = args[3], args[4]
        a["pros_knee_angle"] = [-0.5 + (.35 if t < 1 else .1) * math.sin(2 * math.pi * t) for t in a["time"]]
        a["pros_ankle_angle"] = [(.05 if t < 1 else .2) * math.sin(2 * math.pi * t) for t in a["time"]]
        for joint in M.JOINTS:
            ref[joint + "_q_ref"] = list(a[joint])
        result = M.measurements(*args)
        self.assertEqual(result["qualifying_cycle_count"], 0)
        self.assertFalse(result["gates"]["two_cycles_each_both_excursions"])

    def test_peak_penetration_strict_and_initial_included(self):
        args = fixture()
        args[0]["static_contact_gate"]["max_applied_penetration_m"] = .028
        self.assertFalse(M.measurements(*args)["gates"]["primary_penetration"])

    def test_tracking_rad_to_degrees_and_startup_exclusion(self):
        args = fixture()
        for i, t in enumerate(args[3]["time"]):
            args[4]["pros_knee_angle_q_ref"][i] += math.radians(20 if t < .5 else 1.999)
        result = M.measurements(*args)
        self.assertTrue(result["gates"]["post050_tracking"])
        self.assertAlmostEqual(result["tracking_rmse_deg"]["pros_knee_angle"]["post050"], 1.999, places=9)
        for i, t in enumerate(args[3]["time"]):
            if t >= .5:
                args[4]["pros_knee_angle_q_ref"][i] += math.radians(.002)
        self.assertFalse(M.measurements(*args)["gates"]["post050_tracking"])

    def test_resync_cancelled_and_invalid_flags_fail(self):
        for name in ["invalid_event_this_step", "cycle_rejected_this_step", "cycle_degraded", "timeout_exceeded", "resync_event_this_step"]:
            args = fixture()
            args[2][100]["phase_fsm"][name] = 1
            self.assertFalse(M.measurements(*args)["gates"]["zero_flags"])
        args = fixture()
        args[2][-1]["phase_fsm"]["hs_cancelled_count"] = 1
        self.assertFalse(M.measurements(*args)["gates"]["zero_counters"])

    def test_partial_failed_segment_separate_from_returned_prefix(self):
        args = fixture()
        args[0].update(actual_end_s=2.5, integration_steps=250)
        args[1].update(steps=250, end_reason="physical_guard_censored")
        args = list(args)
        args[2] = args[2][:250]
        for table in args[3:]:
            for key in table:
                table[key] = table[key][:2520]
        result = M.measurements(*args)
        self.assertEqual(result["partial_unreturned_segment_saved_samples"], 20)
        self.assertEqual(result["window_samples"]["full"], 2500)
        self.assertFalse(result["gates"]["completion"])


class TestFailureClassification(unittest.TestCase):
    def test_timeout_is_technical_even_harness_labels_physical(self):
        r = {"status": "physical_guard_censored", "error": "SegmentWallClockTimeout: integration too slow"}
        self.assertEqual(M.exception_class(r, {}), "TECHNICAL_CENSORED")

    def test_known_penetration_is_physical_but_unknown_native_is_technical(self):
        self.assertEqual(M.exception_class({"status": "physical_guard_censored", "error": "OnlineGRFPenetrationLimitExceeded: 0.031"}, {}), "PHYSICAL_FAILURE")
        self.assertEqual(M.exception_class({"status": "harness_or_native_failure", "error": "segmentation fault"}, {}), "TECHNICAL_CENSORED")

    def test_existing_env_state_guard_reasons_and_numerical_timeout(self):
        for reason in ["fall", "joint_divergence:pros_knee_angle", "grf_force", "grf_penetration_hard"]:
            self.assertEqual(M.exception_class({"status": "rollout_complete"}, {"end_reason": reason}), "PHYSICAL_FAILURE")
        for reason in ["step_wall_timeout", "numerical_failure"]:
            self.assertEqual(M.exception_class({"status": "rollout_complete"}, {"end_reason": reason}), "TECHNICAL_CENSORED")

    def test_known_physical_failure_survives_missing_saved_data(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            case = {"id": "random_001", "pose": {"pros_knee_angle": -.5, "pros_ankle_angle": 0}}
            receipt = {"case": case, "status": "physical_guard_censored", "error": "OnlineGRFPenetrationLimitExceeded: 0.031",
                       "policy_updates": 0, "frozen_inputs_unchanged": True}
            (p / "receipt.json").write_text(json.dumps(receipt))
            result = M.classify_folder(p, case)
            self.assertEqual(result["classification"], "PHYSICAL_FAILURE")
            self.assertIsNotNone(result["measurement_error"])

    def test_native_crash_recovers_paired_returned_records_only(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            trace = [{"step": i, "time": i * .01, "returned_reward": .25, "phase_fsm": {"valid_cycle_count": 0}} for i in range(1, 4)]
            rewards = [{"step": i, "time_before_s": (i - 1) * .01, "time_after_s": i * .01, "returned_reward": .25} for i in range(1, 3)]
            (p / "last_valid_trace.jsonl").write_text("\n".join(json.dumps(r) for r in trace) + '\n{"torn":')
            (p / "returned_rewards.jsonl").write_text("\n".join(json.dumps(r) for r in rewards))
            receipt, summary, recovered, evidence = M.recover_returned_prefix(p, {"integration_steps": 0, "status": "started"})
            self.assertEqual(len(recovered), 2)
            self.assertEqual(summary["episode_return"], .5)
            self.assertEqual(evidence["unpaired_trace_records"], 1)
            self.assertEqual(receipt["actual_end_s"], .02)
            self.assertEqual(M.exception_class(receipt, summary), "TECHNICAL_CENSORED")

    def test_full_storage_roundtrip_and_render_error_preserves_numeric_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            receipt, summary, trace, actual, reference, grf = fixture()
            case = {"id": "random_001", "pose": {"pros_knee_angle": -.5, "pros_ankle_angle": 0}}
            receipt.update(case=case, status="rollout_complete", policy_updates=0, frozen_inputs_unchanged=True,
                           postprocessing={"rendered": False, "error": "optional plot renderer failed"})
            for name, value in [("receipt.json", receipt), ("rollout_summary.json", summary), ("rollout_policy_trace.json", trace)]:
                (p / name).write_text(json.dumps(value))
            (p / "sim_outputs").mkdir()
            for suffix, table in [("kinematics", actual), ("kinematics_reference", reference), ("online_grf", grf)]:
                names = list(table)
                path = p / "sim_outputs" / ("rollout_episode_" + suffix + ".sto")
                body = ["Synthetic", "nRows=5000", "nColumns=" + str(len(names)), "inDegrees=no", "endheader", "\t".join(names)]
                body += ["\t".join(str(table[n][i]) for n in names) for i in range(5000)]
                path.write_text("\n".join(body))
            result = M.classify_folder(p, case)
            self.assertEqual(result["classification"], "GAIT_RECOVERY_PASS")
            self.assertTrue(result["completed_safely"])
            self.assertFalse(result["postprocessing"]["rendered"])
            self.assertEqual(result["failed_gates"], [])

    def test_aggregate_retains_20_unresolved_selected_poses_and_controls_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            cases = [{"id": "coherent_nominal", "pose": None}] + [{"id": f"random_{i:03d}", "pose": {"pros_knee_angle": -.5, "pros_ankle_angle": 0}} for i in range(1, 21)]
            (p / "selection.json").write_text(json.dumps({"cases": cases}))
            M.seal(p / "seal.json")
            result = M.aggregate(p / "selection.json", p / "no_runs", p / "seal.json")
            self.assertEqual(result["selected_random_n"], 20)
            self.assertEqual(len(result["controls"]), 1)
            self.assertEqual(result["unresolved_n"], 20)
            self.assertIsNone(result["success_probability_cp95"])
            self.assertEqual(result["all_selected_success_proportion_bounds"], [0, 1])


class TestReaderRevision2TerminalEvidence(unittest.TestCase):
    def journal(self):
        rows = [{"step": i, "time_before_s": (i - 1) * .01, "time_after_s": i * .01,
                 "returned_reward": .1, "terminated": False, "truncated": False, "failure": None,
                 "terminal_contact": None} for i in range(1, 32)]
        rows.append({"step": 32, "time_before_s": .31, "time_after_s": .31,
                     "returned_reward": -.5, "terminated": True, "truncated": False,
                     "end_reason": "morphology_causal_contract_failure",
                     "failure": "OnlineGRFPenetrationLimitExceeded: 0.0301299 exceeds0.03",
                     "terminal_contact": {"source": "hard_penetration_guard", "available": True,
                                          "time_s": .319, "side": "left", "penetration_m": .0301299,
                                          "limit_m": .03, "online_grf": {"left": {"penetration": .0301299}}}})
        return rows

    def test_terminal_guard_evidence_overrides_masked_summary_and_keeps_returned_reward(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            (p / "returned_rewards.jsonl").write_text("\n".join(json.dumps(r) for r in self.journal()))
            receipt, evidence = M.attach_returned_journal_evidence(p, {"status": "rollout_complete"})
            self.assertEqual(M.exception_class(receipt, {"end_reason": "morphology_causal_contract_failure"}), "PHYSICAL_FAILURE")
            self.assertEqual(evidence["returned_records"], 32)
            self.assertEqual(evidence["positive_duration_returned_segments"], 31)
            self.assertEqual(evidence["zero_duration_terminal_return_steps"], [32])
            self.assertAlmostEqual(evidence["returned_reward_sum"], 2.6)
            self.assertAlmostEqual(receipt["_measured_terminal_contact_peak_m"], .0301299)

    def test_zero_time_terminal_does_not_duplicate_fsm_event_and_partial_native_samples(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            (p / "returned_rewards.jsonl").write_text("\n".join(json.dumps(r) for r in self.journal()))
            receipt, summary, trace, actual, reference, grf = fixture()
            receipt.update(actual_end_s=.31, integration_steps=32)
            receipt, _ = M.attach_returned_journal_evidence(p, receipt)
            summary.update(steps=32, phase_valid_cycle_count=0, end_reason="morphology_causal_contract_failure", terminated=True)
            trace = trace[:31]
            terminal = copy.deepcopy(trace[-1])
            terminal["step"] = 32
            trace.append(terminal)
            for table in [actual, reference, grf]:
                for k in table:
                    table[k] = table[k][:320]
            values = M.measurements(receipt, summary, trace, actual, reference, grf)
            self.assertEqual(values["returned_policy_steps"], 32)
            self.assertEqual(values["partial_unreturned_segment_saved_samples"], 10)
            self.assertAlmostEqual(values["first_hs_confirmed_elapsed_s"], .105)
            self.assertAlmostEqual(values["left_primary_peak_penetration_m"], .0301299)
            self.assertEqual(values["complete_valid_cycles"], [])

    def test_zero_time_nonterminal_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            rows = self.journal()
            rows[-1].update(terminated=False, truncated=False)
            (p / "returned_rewards.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
            with self.assertRaises(ValueError):
                M.attach_returned_journal_evidence(p, {})


if __name__ == "__main__":
    unittest.main()
