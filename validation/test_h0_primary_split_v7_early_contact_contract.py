from __future__ import annotations

import copy
import math
import unittest

from validation import h0_primary_split_v7_early_contact_contract as contract


BODY_WEIGHT_N = 700.0


def _raw_time(index: int) -> float:
    return contract.EPISODE_START_TIME_S + index * contract.EXPECTED_SAMPLE_DT_S


def _policy_time(index: int) -> float:
    return contract.EPISODE_START_TIME_S + index * contract.EXPECTED_POLICY_DT_S


def _primary_time(index: int) -> float:
    return contract.EPISODE_START_TIME_S + (
        index - 1
    ) * contract.EXPECTED_SAMPLE_DT_S


def _primary_delivery_time(index: int) -> float:
    policy_index = (
        (index - 1) // contract.EXPECTED_PRIMARY_SAMPLES_PER_STEP + 1
    )
    return _policy_time(policy_index)


def _policy_boundary_times() -> list[float]:
    return [
        _policy_time(index) for index in range(1, contract.EXPECTED_STEPS + 1)
    ]


def _baseline_sample() -> dict:
    return {
        "time_s": contract.EPISODE_START_TIME_S,
        "left_heel_contact": True,
        "left_toe_contact": False,
    }


def _set_primary_run(
    primary: list[dict],
    *,
    start_index: int,
    stop_index: int | None,
    force_n: float = 70.0,
) -> None:
    stop = (
        contract.EXPECTED_PRIMARY_LOAD_SAMPLES + 1
        if stop_index is None
        else stop_index
    )
    for index in range(start_index, stop):
        row = primary[index - 1]
        row["left_normal_grf_bw"] = force_n / BODY_WEIGHT_N
        row["left_normal_force_n"] = force_n
        row["left_in_contact"] = True


def _evidence(
    *,
    candidate_off_index: int | None = 4869,
    primary_start_index: int | None = 4588,
    primary_stop_index: int | None = 4650,
    short_other_flight: bool = False,
) -> tuple[list[dict], list[dict], list[dict], float]:
    first_hs = 400 if short_other_flight else 650
    event_specs = [
        ("toe_off", 250, False),
        ("heel_strike", first_hs, False),
        ("toe_off", 1300, False),
        ("heel_strike", 1700, False),
        ("toe_off", 2300, False),
        ("heel_strike", 2700, False),
        ("toe_off", 3300, False),
        ("heel_strike", 3700, False),
        ("toe_off", 4336, False),
        ("heel_strike", 4569, True),
    ]
    if candidate_off_index is not None:
        event_specs.append(("toe_off", candidate_off_index, False))
    spec_by_index = {
        index: (name, candidate) for name, index, candidate in event_specs
    }
    raw: list[dict] = []
    in_contact = True
    leader = "heel"
    for index in range(1, contract.EXPECTED_RAW_SENSOR_SAMPLES + 1):
        if index in spec_by_index:
            name, candidate = spec_by_index[index]
            in_contact = name == "heel_strike"
            if in_contact:
                leader = "toe" if candidate else "heel"
        raw.append(
            {
                "time_s": _raw_time(index),
                "left_heel_contact": in_contact and leader == "heel",
                "left_toe_contact": in_contact and leader == "toe",
            }
        )
    events = contract.replay_v20_event_stream(
        baseline_sample=_baseline_sample(),
        raw_samples=raw,
        policy_boundary_times=_policy_boundary_times(),
    )["events"]

    primary = [
        {
            "sampled_time_s": _primary_time(index),
            "delivered_time_s": _primary_delivery_time(index),
            "left_normal_grf_bw": 0.0,
            "left_normal_force_n": 0.0,
            "left_in_contact": False,
        }
        for index in range(1, contract.EXPECTED_PRIMARY_LOAD_SAMPLES + 1)
    ]
    if primary_start_index is not None:
        _set_primary_run(
            primary,
            start_index=primary_start_index,
            stop_index=primary_stop_index,
        )
    return raw, primary, events, BODY_WEIGHT_N


def _classify(**kwargs) -> dict:
    raw, primary, events, body_weight_n = _evidence(**kwargs)
    return contract.classify_contact(
        baseline_sample=_baseline_sample(),
        raw_samples=raw,
        primary_load_samples=primary,
        shadow_events=events,
        body_weight_n=body_weight_n,
    )


def _tap_audit() -> dict:
    return {
        "instrumentation_id": "primary_grf_existing_call_observer_v1",
        "installed_after_reset": True,
        "reset_call_count": 0,
        "wrapper_call_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
        "original_call_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
        "one_original_call_per_wrapper_call": True,
        "second_primary_evaluations": 0,
        "return_forwarded_unmodified": True,
        "restored_in_finally": True,
        "descriptor_identity_restored": True,
    }


def _passing_summary(classification: dict) -> dict:
    raw, _primary, events, _body_weight_n = _evidence()
    boundaries = _policy_boundary_times()
    replayed = contract.replay_v20_event_stream(
        baseline_sample=_baseline_sample(),
        raw_samples=raw,
        policy_boundary_times=boundaries,
    )
    final_payload = {
        "source": contract.V20_EVENT_SOURCE,
        "event_contract_id": contract.SHADOW_EVENT_CONTRACT_ID,
        "sample_dt_s": contract.EXPECTED_SAMPLE_DT_S,
        "debounce_s": contract.EXPECTED_V20_DEBOUNCE_S,
        "max_delivery_delay_s": contract.EXPECTED_POLICY_DT_S,
        **replayed["final"],
    }
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DIAGNOSTIC_COLLECTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": contract.CASE_ID,
        "action_selection": "deterministic",
        "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
        "episode_start_offset_s": contract.EPISODE_START_OFFSET_S,
        "action_seed": None,
        "runtime_seed": contract.RUNTIME_SEED,
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_observation_contract_id": (
            contract.SOURCE_OBSERVATION_CONTRACT_ID
        ),
        "source_event_contract_id": contract.SOURCE_EVENT_CONTRACT_ID,
        "primary_load_contract_id": contract.PRIMARY_LOAD_CONTRACT_ID,
        "primary_load_evidence_role": contract.PRIMARY_LOAD_EVIDENCE_ROLE,
        "canonical_scientific_oracle": contract.CANONICAL_SCIENTIFIC_ORACLE,
        "primary_online_grf_used_as_event_source": False,
        "binary_phase_fsm_mode": "binary_shadow",
        "actor_event_source": "legacy_events",
        "binary_phase_event_contract_id": contract.SHADOW_EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "steps": contract.EXPECTED_STEPS,
        "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "v25_raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "primary_load_sample_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
        "primary_load_alignment_id": contract.PRIMARY_LOAD_ALIGNMENT_ID,
        "phase_valid_cycle_count": (
            contract.EXPECTED_LEGACY_PHASE_VALID_CYCLE_COUNT
        ),
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "grf_penetration_max_m": 0.024,
        "n_actor": contract.EXPECTED_ACTOR_FEATURES,
        "n_observation": contract.EXPECTED_FULL_FEATURES,
        "observation_dtype": contract.EXPECTED_OBSERVATION_DTYPE,
        "platform": copy.deepcopy(contract.EXPECTED_PLATFORM_IDENTITY),
        "v5_projected_trace_bit_exact": True,
        "legacy_phase_invalid_event_count": (
            contract.EXPECTED_LEGACY_PHASE_INVALID_EVENT_COUNT
        ),
        "contact_classification": classification,
        "v20_final_state_audit": contract.audit_v20_final_state(
            baseline_sample=_baseline_sample(),
            policy_boundary_times=boundaries,
            final_payload=final_payload,
            raw_samples=raw,
            shadow_events=events,
        ),
        "primary_observation_tap": _tap_audit(),
        "v6_terminal_status": "FAIL_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT",
        "v6_terminal_ledger_sha256": contract.EXPECTED_INPUT_SHA256[
            "v6_terminal_ledger"
        ],
        "v6_nominal_failure_sha256": contract.EXPECTED_INPUT_SHA256[
            "v6_nominal_failure"
        ],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
    }
    for field in (
        "action_mismatch_count",
        "actor_observation_mismatch_count",
        "actor_mean_mismatch_count",
        "teacher_std_mismatch_count",
        "projected_trace_mismatch_count",
        "time_mismatch_count",
        "step_contract_failure_count",
        "action_clipped_values",
        "timeout_count",
        "safety_stop_count",
        "invalid_event_count",
        "hard_invalid_count",
        "nonfinite_count",
        "so_solver_unaccepted_count",
        "sea_plugin_fallback_count",
        "routing_failure_count",
    ):
        summary[field] = 0
    return summary


class ContactClassificationTests(unittest.TestCase):
    def test_persistent_landing_selects_scoped_0p20_candidate(self) -> None:
        result = _classify()
        self.assertTrue(result["passed"])
        self.assertEqual(result["classification"], contract.PERSISTENT_LANDING)
        self.assertEqual(
            result["decision"], contract.MIN_SWING_CANDIDATE_DECISION
        )
        self.assertEqual(
            result["candidate_contract_id"],
            contract.MIN_SWING_CANDIDATE_CONTRACT_ID,
        )
        self.assertGreaterEqual(
            result["metrics"]["raw_guaranteed_contact_duration_s"], 0.05
        )
        self.assertGreaterEqual(
            result["metrics"]["primary_guaranteed_stance_duration_s"], 0.05
        )
        self.assertLessEqual(
            result["metrics"]["primary_crossing_delay_bounds_s"][1], 0.06
        )
        self.assertEqual(
            result["metrics"]["primary_guard_relation"], "definitely_after"
        )

    def test_transient_toe_scuff_uses_first_air_delay(self) -> None:
        result = _classify(
            candidate_off_index=4619,
            primary_start_index=None,
            primary_stop_index=None,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["classification"], contract.TRANSIENT_TOE_SCUFF)
        self.assertAlmostEqual(
            result["metrics"]["raw_guaranteed_contact_duration_s"],
            0.049,
            places=9,
        )
        self.assertAlmostEqual(
            result["metrics"]["raw_return_to_air_delay_s"], 0.050, places=9
        )

    def test_physically_early_landing_requires_actor_correction(self) -> None:
        result = _classify(primary_start_index=4580, primary_stop_index=4641)
        self.assertTrue(result["passed"])
        self.assertEqual(result["classification"], contract.PERSISTENT_LANDING)
        self.assertEqual(result["decision"], contract.ACTOR_CORRECTION_DECISION)
        self.assertEqual(
            result["metrics"]["primary_guard_relation"],
            "definitely_before_or_at",
        )

    def test_support_before_v25_hs_requires_actor_correction(self) -> None:
        result = _classify(primary_start_index=4560, primary_stop_index=4621)
        self.assertTrue(result["passed"])
        self.assertEqual(result["decision"], contract.ACTOR_CORRECTION_DECISION)
        self.assertLess(
            result["metrics"]["primary_onset_upper_bound_s"],
            contract.V6_EARLY_HS_ONSET_TIME_S,
        )

    def test_guard_boundary_is_resolved_by_interval_bounds(self) -> None:
        before = _classify(primary_start_index=4587, primary_stop_index=4638)
        after = _classify(primary_start_index=4588, primary_stop_index=4639)
        self.assertTrue(before["passed"])
        self.assertEqual(before["decision"], contract.ACTOR_CORRECTION_DECISION)
        self.assertTrue(after["passed"])
        self.assertEqual(after["decision"], contract.MIN_SWING_CANDIDATE_DECISION)

    def test_primary_support_active_at_first_sample_is_ambiguous(self) -> None:
        result = _classify(primary_start_index=1, primary_stop_index=4650)
        self.assertFalse(result["passed"])
        self.assertIn("not identifiable", result["reason"])

    def test_two_accepted_primary_runs_are_ambiguous(self) -> None:
        raw, primary, events, body_weight_n = _evidence()
        _set_primary_run(primary, start_index=4700, stop_index=4761)
        result = contract.classify_contact(
            baseline_sample=_baseline_sample(),
            raw_samples=raw,
            primary_load_samples=primary,
            shadow_events=events,
            body_weight_n=body_weight_n,
        )
        self.assertFalse(result["passed"])
        self.assertIn("multiple accepted", result["reason"])

    def test_scuff_cannot_ignore_primary_support_spanning_to(self) -> None:
        result = _classify(
            candidate_off_index=4619,
            primary_start_index=4000,
            primary_stop_index=4650,
        )
        self.assertFalse(result["passed"])
        self.assertIn("starts before TO", result["reason"])

    def test_accepted_support_ending_inside_flight_is_ambiguous(self) -> None:
        result = _classify(primary_start_index=4000, primary_stop_index=4401)
        self.assertFalse(result["passed"])
        self.assertIn("intersects the V25 flight", result["reason"])

    def test_boundary_censored_landing_is_accepted(self) -> None:
        result = _classify(candidate_off_index=None, primary_stop_index=None)
        self.assertTrue(result["passed"])
        self.assertTrue(result["metrics"]["raw_right_censored"])
        self.assertTrue(result["metrics"]["primary_right_censored"])

    def test_long_unloaded_contact_is_ambiguous(self) -> None:
        result = _classify(
            candidate_off_index=4649,
            primary_start_index=None,
            primary_stop_index=None,
        )
        self.assertFalse(result["passed"])
        self.assertEqual(result["classification"], contract.AMBIGUOUS_FAIL)

    def test_primary_onset_after_deadline_is_ambiguous(self) -> None:
        result = _classify(primary_start_index=4631, primary_stop_index=4682)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason_code"], contract.EARLY_GEOMETRIC_PROXY_REASON)
        self.assertEqual(
            result["decision"], contract.ACTOR_OR_GEOMETRY_FOLLOWUP_DECISION
        )

    def test_primary_onset_at_deadline_is_accepted(self) -> None:
        result = _classify(primary_start_index=4630, primary_stop_index=4681)
        self.assertTrue(result["passed"])

    def test_primary_guaranteed_dwell_49ms_fails_50ms_passes(self) -> None:
        short = _classify(primary_start_index=4588, primary_stop_index=4638)
        exact = _classify(primary_start_index=4588, primary_stop_index=4639)
        self.assertFalse(short["passed"])
        self.assertTrue(exact["passed"])
        self.assertAlmostEqual(
            exact["metrics"]["primary_guaranteed_stance_duration_s"],
            0.050,
            places=9,
        )

    def test_raw_guaranteed_dwell_49ms_scuffs_50ms_persists(self) -> None:
        short = _classify(
            candidate_off_index=4619,
            primary_start_index=None,
            primary_stop_index=None,
        )
        exact = _classify(
            candidate_off_index=4620,
            primary_start_index=4570,
            primary_stop_index=4621,
        )
        self.assertEqual(short["classification"], contract.TRANSIENT_TOE_SCUFF)
        self.assertTrue(exact["passed"])
        self.assertEqual(exact["classification"], contract.PERSISTENT_LANDING)
        self.assertAlmostEqual(
            exact["metrics"]["raw_guaranteed_contact_duration_s"],
            0.050,
            places=9,
        )

    def test_exact_20n_does_not_cross_strict_threshold(self) -> None:
        raw, primary, events, body_weight_n = _evidence()
        for row in primary:
            if row["left_normal_force_n"] > 0.0:
                row["left_normal_grf_bw"] = 20.0 / body_weight_n
                row["left_normal_force_n"] = 20.0
        result = contract.classify_contact(
            baseline_sample=_baseline_sample(),
            raw_samples=raw,
            primary_load_samples=primary,
            shadow_events=events,
            body_weight_n=body_weight_n,
        )
        self.assertFalse(result["passed"])

    def test_force_above_threshold_without_contact_fails_closed(self) -> None:
        raw, primary, events, body_weight_n = _evidence()
        active = next(row for row in primary if row["left_normal_force_n"] > 0.0)
        active["left_in_contact"] = False
        result = contract.classify_contact(
            baseline_sample=_baseline_sample(),
            raw_samples=raw,
            primary_load_samples=primary,
            shadow_events=events,
            body_weight_n=body_weight_n,
        )
        self.assertFalse(result["passed"])
        self.assertIn("without contact", result["reason"])

    def test_finite_values_cannot_overflow_force_conversion(self) -> None:
        raw, primary, events, _body_weight_n = _evidence()
        primary[0]["left_normal_grf_bw"] = 1.0e308
        primary[0]["left_normal_force_n"] = 1.0e308
        primary[0]["left_in_contact"] = True
        result = contract.classify_contact(
            baseline_sample=_baseline_sample(),
            raw_samples=raw,
            primary_load_samples=primary,
            shadow_events=events,
            body_weight_n=1.0e308,
        )
        self.assertFalse(result["passed"])
        self.assertIn("overflows", result["reason"])

    def test_any_flight_below_0p20_fails_closed(self) -> None:
        result = _classify(short_other_flight=True)
        self.assertFalse(result["passed"])
        self.assertIn("shorter than the 0.20", result["reason"])

    def test_complete_event_replay_rejects_omitted_raw_transition(self) -> None:
        raw, primary, events, body_weight_n = _evidence()
        for row in raw[899:910]:
            row["left_heel_contact"] = False
            row["left_toe_contact"] = False
        result = contract.classify_contact(
            baseline_sample=_baseline_sample(),
            raw_samples=raw,
            primary_load_samples=primary,
            shadow_events=events,
            body_weight_n=body_weight_n,
        )
        self.assertFalse(result["passed"])
        self.assertIn("complete exact V20 replay", result["reason"])

    def test_complete_event_replay_rejects_extra_or_mutated_event(self) -> None:
        for mutation in ("extra", "delivery"):
            raw, primary, events, body_weight_n = _evidence()
            if mutation == "extra":
                events.insert(1, copy.deepcopy(events[0]))
            else:
                early_hs = next(
                    event
                    for event in events
                    if event["event"] == "heel_strike"
                    and math.isclose(
                        event["event_time_s"],
                        contract.V6_EARLY_HS_ONSET_TIME_S,
                        abs_tol=contract.TIME_TOLERANCE_S,
                    )
                )
                early_hs["delivered_time_s"] += 0.001
            result = contract.classify_contact(
                baseline_sample=_baseline_sample(),
                raw_samples=raw,
                primary_load_samples=primary,
                shadow_events=events,
                body_weight_n=body_weight_n,
            )
            self.assertFalse(result["passed"], mutation)

    def test_replay_rejects_mutated_baseline(self) -> None:
        raw, primary, events, body_weight_n = _evidence()
        baseline = _baseline_sample()
        baseline["left_heel_contact"] = False
        result = contract.classify_contact(
            baseline_sample=baseline,
            raw_samples=raw,
            primary_load_samples=primary,
            shadow_events=events,
            body_weight_n=body_weight_n,
        )
        self.assertFalse(result["passed"])

    def test_primary_missing_duplicate_offgrid_and_nonfinite_fail_closed(self) -> None:
        for mutation in ("missing", "duplicate", "offgrid", "nonfinite"):
            raw, primary, events, body_weight_n = _evidence()
            if mutation == "missing":
                primary.pop()
            elif mutation == "duplicate":
                primary[100] = copy.deepcopy(primary[99])
            elif mutation == "offgrid":
                primary[100]["sampled_time_s"] += 0.0001
            else:
                primary[100]["left_normal_force_n"] = math.nan
            result = contract.classify_contact(
                baseline_sample=_baseline_sample(),
                raw_samples=raw,
                primary_load_samples=primary,
                shadow_events=events,
                body_weight_n=body_weight_n,
            )
            self.assertFalse(result["passed"], mutation)

    def test_policy_boundary_ledger_has_500_not_5000_entries(self) -> None:
        raw, _primary, _events, _body_weight_n = _evidence()
        with self.assertRaises(ValueError):
            contract.replay_v20_event_stream(
                baseline_sample=_baseline_sample(),
                raw_samples=raw,
                policy_boundary_times=[
                    _policy_time(index)
                    for index in range(
                        1, contract.EXPECTED_RAW_SENSOR_SAMPLES + 1
                    )
                ],
            )


class DiagnosticGateTests(unittest.TestCase):
    def test_persistent_scuff_and_early_physical_are_terminal_passes(self) -> None:
        cases = (
            _classify(),
            _classify(
                candidate_off_index=4619,
                primary_start_index=None,
                primary_stop_index=None,
            ),
            _classify(primary_start_index=4580, primary_stop_index=4641),
        )
        for classification in cases:
            gate = contract.diagnostic_gate(_passing_summary(classification))
            self.assertTrue(gate["passed"], gate)
            self.assertEqual(gate["status"], contract.DIAGNOSTIC_PASS_STATUS)

    def test_old_guard_check_names_do_not_satisfy_gate(self) -> None:
        summary = _passing_summary(_classify())
        classified = copy.deepcopy(summary["contact_classification"])
        classified["checks"]["primary_support_not_before_legacy_guard"] = True
        del classified["checks"][
            "primary_support_definitely_after_legacy_guard"
        ]
        summary["contact_classification"] = classified
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["classification"])

    def test_ambiguous_classification_fails_whole_diagnostic(self) -> None:
        ambiguous = _classify(
            candidate_off_index=4649,
            primary_start_index=None,
            primary_stop_index=None,
        )
        gate = contract.diagnostic_gate(_passing_summary(ambiguous))
        self.assertFalse(gate["passed"])
        self.assertFalse(gate["checks"]["classification"])

    def test_spoofed_decision_and_missing_counter_fail_closed(self) -> None:
        summary = _passing_summary(_classify())
        summary["contact_classification"]["decision"] = (
            contract.ACTOR_CORRECTION_DECISION
        )
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["classification"])

        summary = _passing_summary(_classify())
        del summary["so_solver_unaccepted_count"]
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["all_zero_counters"])

    def test_primary_corroboration_cannot_become_event_source(self) -> None:
        summary = _passing_summary(_classify())
        summary["primary_online_grf_used_as_event_source"] = True
        gate = contract.diagnostic_gate(summary)
        self.assertFalse(gate["checks"]["primary_load_role_exact"])
        self.assertFalse(
            contract.AUTHORITY["primary_online_grf_as_event_source_authorized"]
        )

    def test_v6_terminal_hash_binding_is_required(self) -> None:
        summary = _passing_summary(_classify())
        summary["v6_terminal_ledger_sha256"] = "0" * 64
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["v6_terminal_preserved"])

    def test_legacy_invalid_events_and_cycles_are_exact(self) -> None:
        summary = _passing_summary(_classify())
        summary["legacy_phase_invalid_event_count"] = 0
        self.assertFalse(
            contract.diagnostic_gate(summary)["checks"][
                "legacy_phase_invalid_events_exact"
            ]
        )
        summary = _passing_summary(_classify())
        summary["phase_valid_cycle_count"] = 3
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["legacy_phase_cycles_exact"])

    def test_alignment_platform_v20_and_tap_are_exact(self) -> None:
        mutations = (
            ("primary_load_alignment_id", "interpolated_forbidden", "primary_alignment_exact"),
            ("primary_observation_tap", {}, "primary_observation_tap"),
        )
        for field, value, check in mutations:
            summary = _passing_summary(_classify())
            summary[field] = value
            self.assertFalse(contract.diagnostic_gate(summary)["checks"][check])

        summary = _passing_summary(_classify())
        summary["platform"]["distributions"]["scipy"] = "0.0"
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["platform_identity_exact"])

        summary = _passing_summary(_classify())
        summary["v20_final_state_audit"]["metrics"][
            "candidate_cancellation_count"
        ] = 1
        self.assertFalse(contract.diagnostic_gate(summary)["checks"]["v20_final_state"])

    def test_non_mapping_summary_never_raises(self) -> None:
        gate = contract.diagnostic_gate(None)  # type: ignore[arg-type]
        self.assertFalse(gate["passed"])


class ScopeAndAuthorityTests(unittest.TestCase):
    def test_only_one_nominal_shadow_case_is_authorized(self) -> None:
        self.assertEqual(contract.CASE_IDS, (contract.CASE_ID,))
        case = contract.canonical_case()
        self.assertEqual(case["binary_phase_fsm_mode"], "binary_shadow")
        with self.assertRaises(ValueError):
            contract.canonical_case("another_case")

    def test_mutating_authority_remains_closed(self) -> None:
        for name in (
            "v6_retry_authorized",
            "binary_active_execution_authorized",
            "min_swing_runtime_change_authorized",
            "actor_updates_authorized",
            "critic_updates_authorized",
            "ppo_updates_authorized",
            "protected_trial_access_authorized",
            "reserve_trial_access_authorized",
            "runtime_promotion_authorized",
            "primary_grf_modification_authorized",
            "detector_geometry_modification_authorized",
            "binary_fsm_v20_modification_authorized",
            "sea_semantic_modification_authorized",
        ):
            self.assertFalse(contract.AUTHORITY[name], name)

    def test_predecessor_hashes_are_lowercase_sha256(self) -> None:
        self.assertEqual(
            set(contract.EXPECTED_INPUT_SHA256), set(contract.INPUT_RELATIVE_PATHS)
        )
        for value in contract.EXPECTED_INPUT_SHA256.values():
            self.assertEqual(len(value), 64)
            self.assertTrue(
                all(character in "0123456789abcdef" for character in value)
            )


if __name__ == "__main__":
    unittest.main()
