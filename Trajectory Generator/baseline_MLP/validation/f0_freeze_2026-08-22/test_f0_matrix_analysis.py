"""Static self-test of f0_matrix_analysis (no simulation, no rollouts).

All synthetic data live in a ``tempfile`` directory (under /private/tmp when
available, else ``tempfile.gettempdir()``) created per run and removed at the
end; the analyser's metrics directory is passed explicitly, so no real F0
artefact or fixed directory is touched. A synthetic copy of ``rollout_eval.py``
pins the argparse defaults used by the horizon and contract rules, and the
pinned configs are temporary YAML files. Covered:
  - global completeness (NOT_EXECUTED -> analysis_complete False);
  - horizon semantics: progress_upper_bound vs env_horizon_steps_expected
    (ratios 200.4/200.5/200.6 -> env 200/200/201), loop cap below the env
    horizon (loop_cap_hit, never completion), the F0 case 5/0.01 = 500,
    malformed/inconsistent CLI and digest mismatches -> FAIL_PROVENANCE;
  - finiteness: non-required reserve column NaN/Inf -> FAIL_OUTPUT_INCOMPLETE,
    derived-metric overflow -> FAIL_METRICS, recursive gate unit cases;
  - ABI: n_actor/n_observation, action_shape exactly [2], feature-name lists
    (length, non-empty, unique), n_actor == registry width of the receipt
    candidate, candidate receipt/job coherence, observation width == n_actor
    and action width == 2 at every row, step sequence exactly 1..steps, time
    finite strictly increasing;
  - exact FSM/event contracts: event_source exactly binary_active_v26 (mode
    binary_active) or exactly phase_fsm_input_mode, no prefix/suffix
    acceptance; summary contract fields exactly equal to config/CLI; causal
    ledger event_contract_id present, exact and constant;
  - causal payload continuity, counters, runtime coherence, summary schema,
    grids, comparison classes, stochastic aggregation, no-clobber, Markdown.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402
import f0_matrix_analysis as A  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402

PASSED = 0
BINARY_CONTRACT = "binary_point_v25+heel_qualified_fsm_v2"
DISABLED_CONTRACT = "binary_events_disabled_v1"


def tmp_base() -> str:
    """Portable temp root: /private/tmp when available (macOS/Linux), else tempfile.gettempdir()."""
    preferred = Path("/private/tmp")
    if preferred.is_dir() and os.access(preferred, os.W_OK):
        return str(preferred)
    return tempfile.gettempdir()


FAKE_ROLLOUT_EVAL = '''
import argparse
p = argparse.ArgumentParser()
p.add_argument("--segment-duration", type=float, default=0.01)
p.add_argument("--episode-duration", type=float, default=0.5)
p.add_argument("--max-steps", type=int, default=10000)
p.add_argument("--binary-phase-actor-fsm-version", choices=("v2", "v3"), default="v3")
p.add_argument("--phase-fsm-input-mode", choices=("legacy_events", "shadow", "two_sensor"), default="legacy_events")
p.add_argument("--event-contract-id", default="legacy_events_v1")
p.add_argument("--binary-phase-fsm-mode", choices=("disabled", "binary_shadow", "binary_active"), default="disabled")
p.add_argument("--binary-phase-event-contract-id", default="binary_events_disabled_v1")
p.add_argument("--include-controller-diagnostic-observation", action=argparse.BooleanOptionalAction, default=True)
'''
CAUSAL_OK = {"total_cancelled_transition_count": 0, "total_resolved_sample_count": 12, "total_dropped_sample_count": 3, "timeout_transition_count": 0, "failed_closed": False, "terminal_flushed": True, "event_contract_id": BINARY_CONTRACT}
DIAG = list(C.CONTROLLER_DIAGNOSTIC_FEATURES)
PRIV45 = [f"priv_{i}" for i in range(45)]


def ok(label: str) -> None:
    global PASSED
    PASSED += 1
    print(f"  ok   : {label}")


def expect(fn, exc_types, label: str, needle: str | None = None):
    try:
        fn()
    except exc_types as exc:
        if needle and needle not in str(exc):
            raise AssertionError(f"{label}: error does not mention {needle!r}: {exc}")
        return exc
    raise AssertionError(f"{label}: expected {exc_types}")


def write_sto(path: Path, names: list[str], data: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["synthetic", f"nRows={data.shape[0]}", f"nColumns={data.shape[1]}", "endheader", "\t".join(names)]
    lines += ["\t".join(f"{v:.12g}" for v in row) for row in data]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_config(path: Path, episode_duration: float, segment_duration: float, fsm_mode: str = "binary_active", fsm_version: str | None = None, morph_mode: str | None = "event_anchored_causal_delayed_experimental", causal_contract: str | None = BINARY_CONTRACT, include_diag: bool | None = False) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fsm_mode == "binary_active":
        grf = f"grf:\n  phase_fsm_input_mode: legacy_events\n  event_contract_id: legacy_events_v1\n  binary_phase_fsm_mode: {fsm_mode}\n  binary_phase_event_contract_id: {BINARY_CONTRACT}\n"
    else:
        grf = f"grf:\n  binary_phase_fsm_mode: {fsm_mode}\n"
    if fsm_version:
        grf += f"  binary_phase_actor_fsm_version: {fsm_version}\n"
    reward = f"reward:\n  morphology_phase_mode: {morph_mode}\n" if morph_mode else "reward:\n  morphology_weight: 0.0\n"
    if causal_contract:
        reward += f"  morphology_causal_event_contract_id: {causal_contract}\n"
    env = f"env:\n  include_controller_diagnostic_observation: {str(include_diag).lower()}\n" if include_diag is not None else ""
    path.write_text(f"simulation:\n  episode_duration: {episode_duration}\n  segment_duration: {segment_duration}\n{grf}{reward}{env}", encoding="utf-8")
    return C.sha256_file(path)


def actor_names_for(candidate: str, n_actor: int) -> list[str]:
    """Registry manifest names when the width matches the candidate, else a synthetic list."""
    if candidate in C.CANDIDATES and C.CANDIDATES[candidate]["width"] == n_actor:
        return list(C.load_actor_feature_manifest(candidate)["actor_feature_names"])
    return [f"actor_f{i}" for i in range(n_actor)]


class Fixture:
    def __init__(self, root: Path):
        self.root = root
        self.rollout_eval = root / "rollout_eval_fake.py"
        self.rollout_eval.write_text(FAKE_ROLLOUT_EVAL, encoding="utf-8")
        self.rollout_eval_sha = C.sha256_file(self.rollout_eval)
        self.cfg = root / "cfg.yaml"
        write_config(self.cfg, 2.0, 0.01)
        # July-like: no binary block keys (defaults), no causal mode, no causal contract
        self.cfg_legacy = root / "cfg_legacy.yaml"
        write_config(self.cfg_legacy, 2.0, 0.01, fsm_mode="disabled", morph_mode=None, causal_contract=None)
        # legacy mode but with a causal contract declared (payload allowed, must match)
        self.cfg_legacy_contract = root / "cfg_legacy_contract.yaml"
        write_config(self.cfg_legacy_contract, 2.0, 0.01, fsm_mode="disabled", morph_mode=None, causal_contract=BINARY_CONTRACT)

    def make_output(self, name: str, steps: int, *, config: Path | None = None, mode: str = "binary_active", knee_amp: float = 0.3, ankle_amp: float = 0.2, tau_knee_value: float = 5.0, fsm_version: str = "v3", event_source: str | None = None, fsm_counters: dict | None = None, drop_fsm_counters: tuple = (), causal=None, legacy_events: list | None = None, mutate=None, summary_overrides: dict | None = None, config_sha_override: str | None = None, rollout_eval_sha_override: str | None = None, extra_cli: list | None = None, cli_version: str | None = "same", candidate: str = "B0820_V3_BEST", n_actor: int = 35, obs_width: int | None = None, action_width: int = 2, actor_names: list | None = None, obs_names: list | None = None, n_observation: int | None = None, include_diag: bool = False) -> Path:
        root = self.root / name
        config = config or self.cfg
        binary = mode == "binary_active"
        actor_list = list(actor_names) if actor_names is not None else actor_names_for(candidate, n_actor)
        obs_list = list(obs_names) if obs_names is not None else actor_list + DIAG + PRIV45
        if event_source is None:
            event_source = A.EVENT_SOURCE_BINARY_ACTIVE if binary else "legacy_events"
        rows = steps * A.SAMPLES_PER_STEP
        t = np.arange(rows) * 0.001
        knee = knee_amp * np.sin(2 * math.pi * 1.0 * t)
        ankle = ankle_amp * np.sin(2 * math.pi * 1.0 * t) - 0.05
        u_knee = 0.5 * np.sin(2 * math.pi * 1.0 * t)
        u_ankle = 0.2 * np.cos(2 * math.pi * 1.0 * t)
        tau_knee = np.full(rows, tau_knee_value)
        # keep every recorded value finite: for the overflow case only the derived power tau*qdot must overflow
        tau_knee_motor = tau_knee * 1.1 if abs(tau_knee_value) < 1e300 else tau_knee.copy()
        tau_ankle = 5.0 * np.sign(np.sin(2 * math.pi * 1.0 * t) + 1e-12)
        tau_ankle[tau_ankle == 0] = 5.0
        series = {
            A.KINEMATICS_FILE: (["time", "pros_knee_angle", "pros_ankle_angle"], np.column_stack([t, knee, ankle])),
            A.REFERENCE_FILE: (["time", "pros_knee_angle_q_ref", "pros_knee_angle_qdot_ref", "pros_ankle_angle_q_ref", "pros_ankle_angle_qdot_ref"], np.column_stack([t, knee + 0.01, np.gradient(knee, t), ankle + 0.01, np.gradient(ankle, t)])),
            A.SEA_CONTROLS_FILE: (["time", "pros_knee_angle", "pros_ankle_angle"], np.column_stack([t, u_knee, u_ankle])),
            A.SEA_TORQUES_FILE: (["time", "SEA_Knee_tau_spring", "SEA_Ankle_tau_spring", "SEA_Knee_tau_motor", "SEA_Ankle_tau_motor"], np.column_stack([t, tau_knee, tau_ankle, tau_knee_motor, tau_ankle * 1.1])),
            A.GRF_FILE: (["time", "left_normal_force", "left_penetration", "left_in_contact"], np.column_stack([t, 400.0 * (np.sin(2 * math.pi * t) > 0), 0.015 * (np.sin(2 * math.pi * t) > 0), (np.sin(2 * math.pi * t) > 0).astype(float)])),
            A.RESERVE_FILE: (["time", "hip_flexion_l_reserve_torque", "pros_knee_angle_reserve_torque", "pros_ankle_angle_reserve_torque"], np.column_stack([t, 30.0 * np.ones(rows), np.zeros(rows), np.zeros(rows)])),
        }
        width = obs_width if obs_width is not None else n_actor
        trace = []
        for i in range(steps):
            fsm = {"valid_hs_count": 2 if i >= 1 else 1, "valid_to_count": 1, "valid_cycle_count": 1 if i >= 2 else 0, "invalid_event_count": 0, "fsm_behaviour_version": fsm_version, "event_source": event_source, "resync_count": 0, "hs_cancelled_count": 1}
            if fsm_counters:
                fsm.update(fsm_counters)
            for key in drop_fsm_counters:
                fsm.pop(key, None)
            if callable(causal):
                payload = causal(i)
            elif causal is not None:
                payload = dict(causal)
            else:
                payload = dict(CAUSAL_OK)
            action = [0.4 + 0.001 * i, -0.1, 0.05][:action_width] if action_width <= 3 else [0.1] * action_width
            row = {
                "step": i + 1, "time": 13.0 + i * 0.01,
                "raw_policy_action": list(action), "applied_policy_action": list(action),
                "actor_observation_vector_before": (np.arange(width) * 0.01 + i * 1e-3).tolist(),
                "phase_fsm": fsm,
                "morphology_causal_diagnostics": payload,
                "legacy_online_events": list(legacy_events) if (legacy_events is not None and i == 0) else [],
            }
            trace.append(row)
        summary = {
            "ok": True, "steps": steps, "episode_return": 2.0, "reward_mean": 0.01, "end_reason": "episode_time_limit", "phase_valid_hs_count": 2, "phase_valid_to_count": 1, "phase_valid_cycle_count": 1, "invalid_event_count": 0, "grf_penetration_max_m": 0.015, "reserve_norm_max_nm": 30.0, "action_abs_max": 0.6, "action_clipped_steps": 0, "morphology_settled_segments": 3, "morphology_settled_samples": 12, "morphology_discarded_segments": 0, "morphology_discarded_samples": 3, "grf_penetration_penalty_threshold_m": 0.02, "grf_penetration_termination_m": 0.028,
            "binary_phase_fsm_mode": mode, "phase_fsm_input_mode": "legacy_events", "event_contract_id": "legacy_events_v1", "binary_phase_event_contract_id": BINARY_CONTRACT if binary else DISABLED_CONTRACT,
            "n_actor": n_actor, "n_observation": n_observation if n_observation is not None else len(obs_list), "action_shape": [2], "actor_feature_names": actor_list, "observation_feature_names": obs_list,
            "include_controller_diagnostic_observation": include_diag,
        }
        if summary_overrides:
            summary.update(summary_overrides)
        if mutate:
            mutate(series, trace, summary)
        root.mkdir(parents=True, exist_ok=True)
        for fname, (names, data) in series.items():
            write_sto(root / "sim_outputs" / fname, names, data)
        (root / M.TRACE_FILE).write_text(json.dumps(trace), encoding="utf-8")
        (root / M.SUMMARY_FILE).write_text(json.dumps(summary), encoding="utf-8")
        command = ["python", "rollout_eval.py", "--checkpoint", "x", "--no-auto-config", "--config", str(config)]
        if cli_version == "same":
            command += ["--binary-phase-actor-fsm-version", fsm_version]
        elif cli_version:
            command += ["--binary-phase-actor-fsm-version", cli_version]
        command += list(extra_cli or [])
        receipt = {"schema_version": 4, "status": "ok", "returncode": 0, "duration_s": 1.0, "summary_sha256": "x", "trace_sha256": "y", "git_head": "h", "candidate": candidate, "config": C.rel(config), "config_sha256": config_sha_override or C.sha256_file(config), "rollout_eval_sha256": rollout_eval_sha_override or self.rollout_eval_sha, "module_state_sha256": "m", "command": command}
        (root / M.RECEIPT_FILE).write_text(json.dumps(receipt), encoding="utf-8")
        return root


def describe(job_id: str, family: str, cls: str, candidate: str, start: str, mode: str, seed: int, out_dir: Path, repeat: int = 1, runtime: str = "v3_canonical") -> dict:
    return {"job_id": job_id, "family": family, "comparison_class": cls, "candidate": candidate, "runtime": runtime, "start": start, "action_selection": mode, "seed": seed, "repeat": repeat, "output_dir": C.rel(out_dir), "historical_reference_summary": None, "policy_native_to_runtime": cls == A.ISOMETRIC_CLASS, "runtime_is_target_v3": runtime == "v3_canonical"}


def no_verify(rec, out_dir):
    return None


def main() -> int:
    base = tmp_base()
    tmp = Path(tempfile.mkdtemp(prefix="f0_matrix_selftest_", dir=base))
    try:
        fx = Fixture(tmp)
        steps = 200  # 2.0 s at 1 kHz
        t = np.arange(steps * A.SAMPLES_PER_STEP) * 0.001
        LEGACY = dict(config=fx.cfg_legacy, mode="disabled", drop_fsm_counters=("resync_count", "hs_cancelled_count"), candidate="JUL_H0")
        LEGACY_CONTRACT = dict(LEGACY, config=fx.cfg_legacy_contract)

        def analyse(out_dir: Path, job_id="B0820_V3_BEST__v3_canonical__nominal__det", cls=A.ISOMETRIC_CLASS, cand="B0820_V3_BEST", runtime="v3_canonical", family="det"):
            return A.analyse_job(describe(job_id, family, cls, cand, "nominal", "deterministic", 123, out_dir, runtime=runtime), verify=no_verify, rollout_eval_path=fx.rollout_eval)

        def analyse_jul(out_dir: Path):
            return analyse(out_dir, "JUL_H0__july_legacy__nominal__det", cls="historical_config_replay_on_current_code", cand="JUL_H0", runtime="july_legacy", family="replay")

        # ---------- shape metrics and grid validators ----------------------------
        q = 0.3 * np.sin(2 * math.pi * t)
        s = A.series_shape(q, t)
        assert abs(s["rom"] - 0.6) < 2e-3 and s["zero_crossings"] == 3 and s["turning_points"] == 4 and abs(s["velocity_abs_max"] - 0.3 * 2 * math.pi) < 0.02
        ramp = np.linspace(0.0, 1.0, len(t))
        tq = A.torque_shape(np.full(len(t), 5.0), np.gradient(ramp, t), t)
        assert abs(tq["work_net_j"] - 5.0) < 0.1 and tq["sign_changes"] == 0
        ok("series_shape / torque_shape analytic checks (ROM, crossings, turning points, velocity, work 5 J)")
        A.validate_grid(t, "ok")
        dup = t.copy()
        dup[10] = dup[9]
        dec = t.copy()
        dec[20] = dec[19] - 0.001
        nonf = t.copy()
        nonf[5] = math.nan
        for bad, label, needle in ((dup, "duplicate", "not strictly increasing"), (dec, "decreasing", "not strictly increasing"), (nonf, "nonfinite", "non-finite")):
            expect(lambda: A.validate_grid(bad, label), A.GridMismatchError, label, needle)
            expect(lambda: A.series_shape(q, bad), A.GridMismatchError, f"series_shape on {label} grid")
            expect(lambda: A.torque_shape(np.full(len(bad), 5.0), np.zeros(len(bad)), bad), A.GridMismatchError, f"torque_shape on {label} grid")
        expect(lambda: A.validate_grid(t[:1], "short"), A.GridMismatchError, "short", "too short")
        expect(lambda: A.verify_grid(t, t + 0.0005, "shifted"), A.GridMismatchError, "shifted grid", "mismatch")
        expect(lambda: A.verify_grid(t, t[:-1], "length"), A.GridMismatchError, "length", "length mismatch")
        expect(lambda: A.tracking(q, q[:-1]), A.GridMismatchError, "tracking length")
        ok("validate_grid/verify_grid: duplicate, decreasing, non-finite, short, shifted and mismatched grids fail before gradient/work")

        # ---------- finiteness gate (unit) -------------------------------------------
        A.assert_finite_metrics({"a": {"present": False, "value": None}, "tracking_served_vs_actual": {"knee": {"pearson_r": None, "rmse": 0.1}}, "s": "txt", "b": True, "n": [1, 2.5]})
        expect(lambda: A.assert_finite_metrics({"a": {"present": True, "value": None}}), A.MetricsFiniteError, "counter present with None", "not finite")
        expect(lambda: A.assert_finite_metrics({"a": {"present": False, "value": 1.0}}), A.MetricsFiniteError, "absent counter with value", "absent counter")
        expect(lambda: A.assert_finite_metrics({"x": float("nan")}), A.MetricsFiniteError, "NaN leaf", "non-finite")
        expect(lambda: A.assert_finite_metrics({"x": {"y": float("inf")}}), A.MetricsFiniteError, "Inf leaf", "non-finite")
        expect(lambda: A.assert_finite_metrics({"y": None}), A.MetricsFiniteError, "unexpected None", "unexpected None")
        ok("assert_finite_metrics: NaN/Inf/unexpected None rejected; nullable paths and absent counters accepted")

        # ---------- provenance: horizon rule, CLI, contracts ---------------------------------
        defaults = A.rollout_eval_argparse_defaults(fx.rollout_eval)
        assert defaults == {"max_steps": 10000, "episode_duration": 0.5, "segment_duration": 0.01, "actor_fsm_version": "v3", "phase_fsm_input_mode": "legacy_events", "event_contract_id": "legacy_events_v1", "binary_phase_fsm_mode": "disabled", "binary_phase_event_contract_id": DISABLED_CONTRACT, "include_controller_diagnostic_observation": True}
        real_defaults = A.rollout_eval_argparse_defaults(C.ROLLOUT_EVAL)
        assert real_defaults["max_steps"] == 10000 and real_defaults["episode_duration"] == 0.5 and real_defaults["segment_duration"] == 0.01 and real_defaults["actor_fsm_version"] == "v3"
        assert real_defaults["phase_fsm_input_mode"] == "legacy_events" and real_defaults["event_contract_id"] == "legacy_events_v1" and real_defaults["binary_phase_fsm_mode"] == "disabled" and real_defaults["binary_phase_event_contract_id"] == DISABLED_CONTRACT and real_defaults["include_controller_diagnostic_observation"] is True
        ok("argparse defaults parsed by AST from the synthetic and the real rollout_eval.py (max_steps 10000, 0.5 s, 0.01 s, v3, legacy_events/legacy_events_v1/disabled/binary_events_disabled_v1, include diagnostics True)")
        for ed, env_expected, progress_expected in ((2.004, 200, 201), (2.005, 200, 201), (2.006, 201, 201), (5.0, 500, 500), (2.0, 200, 200)):
            h = A.expected_horizon({"simulation": {"episode_duration": ed, "segment_duration": 0.01}}, defaults, {})
            assert h["env_horizon_steps_expected"] == env_expected and h["progress_upper_bound"] == progress_expected and h["expected_recorded_steps"] == env_expected and h["loop_cap"] == 10000, (ed, h)
        h = A.expected_horizon({"simulation": {"episode_duration": 2.0, "segment_duration": 0.01}}, defaults, {"max_steps": 150})
        assert h["env_horizon_steps_expected"] == 200 and h["expected_recorded_steps"] == 150 and h["loop_cap_below_env_horizon"] is True and h["sources"]["max_steps"] == "cli"
        assert A.expected_horizon({"simulation": {}}, defaults, {})["env_horizon_steps_expected"] == 50
        assert A.expected_horizon({"simulation": {"episode_duration": 2.005, "segment_duration": 0.01}}, defaults, {})["env_horizon_boundary_sensitive"] is True
        for bad_cli in ({"episode_duration": 0.0}, {"segment_duration": -0.01}, {"max_steps": 0}, {"episode_duration": float("inf")}):
            expect(lambda: A.expected_horizon({"simulation": {"episode_duration": 2.0, "segment_duration": 0.01}}, defaults, bad_cli), A.ProvenanceError, f"bad {bad_cli}")
        ok("horizon: env ceil(ratio-0.5) -> 200.4/200.5/200.6 => 200/200/201 (progress bound 201), 5/0.01 => 500, cap 150 < env -> expected_recorded 150, boundary_sensitive disclosed, invalid values fail")
        assert A.cli_values(["python", "x.py", "--max-steps", "150"], A.ROLLOUT_EVAL_CLI_FLAGS) == {"max_steps": 150}
        assert A.cli_values(["python", "--episode-duration=2.5"], A.ROLLOUT_EVAL_CLI_FLAGS) == {"episode_duration": 2.5}
        assert A.cli_values(["python", "--binary-phase-fsm-mode", "disabled", "--event-contract-id=legacy_events_v1"], A.ROLLOUT_EVAL_CLI_FLAGS) == {"binary_phase_fsm_mode": "disabled", "event_contract_id": "legacy_events_v1"}
        assert A.cli_values(["python", C.NO_CONTROLLER_DIAGNOSTIC_FLAG], A.ROLLOUT_EVAL_CLI_FLAGS) == {"include_controller_diagnostic_observation": False}
        assert A.cli_values(["python", "--include-controller-diagnostic-observation"], A.ROLLOUT_EVAL_CLI_FLAGS) == {"include_controller_diagnostic_observation": True}
        for bad, needle in ((["python", "--max-steps"], "without value"), (["python", "--max-steps", "--config"], "without value"), (["python", "--max-steps", "abc"], "not numeric"), (["python", "--max-steps", "10", "--max-steps", "20"], "inconsistent"), (["python", "--binary-phase-actor-fsm-version", "v9"], "not in"), (["python", "--binary-phase-fsm-mode="], "empty value"), (["python", "--include-controller-diagnostic-observation", C.NO_CONTROLLER_DIAGNOSTIC_FLAG], "both present"), ("not a list", "not a list")):
            expect(lambda: A.cli_values(bad, A.ROLLOUT_EVAL_CLI_FLAGS), A.ProvenanceError, str(bad), needle)
        ok("CLI extraction: numeric, string and BooleanOptionalAction flags; missing/empty value, non-numeric, repeated-inconsistent, invalid version, --flag with --no-flag, non-list command fail closed")
        cfg_diag_true = tmp / "cfg_diag_true.yaml"
        write_config(cfg_diag_true, 2.0, 0.01, include_diag=True)
        exp_w = A.expected_runtime(C.load_yaml(fx.cfg), defaults, {}, "v3_canonical")
        assert exp_w["expected_observation_width"] == 84 and exp_w["include_controller_diagnostic_observation"] is False and exp_w["include_controller_diagnostic_observation_source"] == "config"
        assert A.expected_runtime(C.load_yaml(fx.cfg_legacy), defaults, {}, "july_legacy")["expected_observation_width"] == 84
        exp_39 = A.expected_runtime(C.load_yaml(cfg_diag_true), defaults, {"include_controller_diagnostic_observation": False}, "v26_imitation_native")
        assert exp_39["expected_observation_width"] == 88 and exp_39["include_controller_diagnostic_observation"] is False and exp_39["include_controller_diagnostic_observation_source"] == "cli"
        expect(lambda: A.expected_runtime(C.load_yaml(cfg_diag_true), defaults, {}, "v26_imitation_native"), A.ProvenanceError, "config include True without CLI override", "registry requires False")
        expect(lambda: A.expected_runtime(C.load_yaml(fx.cfg_legacy), defaults, {}, "unknown_runtime"), A.ProvenanceError, "unknown runtime", "unknown to the F0 registry")
        ok("expected_runtime: observation width 84/88 from the runtime registry; include_controller_diagnostic_observation CLI>config>default must equal the registry (False) else FAIL_PROVENANCE")
        cfg_bin = C.load_yaml(fx.cfg)
        exp = A.expected_runtime(cfg_bin, defaults, {})
        assert exp["event_source_expected"] == A.EVENT_SOURCE_BINARY_ACTIVE and exp["binary_phase_fsm_mode"] == "binary_active" and exp["phase_fsm_input_mode"] == "legacy_events" and exp["binary_phase_event_contract_id"] == BINARY_CONTRACT and exp["morphology_causal_event_contract_id"] == BINARY_CONTRACT and exp["causal_ledger_required"] is True
        exp_l = A.expected_runtime(C.load_yaml(fx.cfg_legacy), defaults, {})
        assert exp_l["event_source_expected"] == "legacy_events" and exp_l["binary_phase_fsm_mode"] == "disabled" and exp_l["binary_phase_event_contract_id"] == DISABLED_CONTRACT and exp_l["contract_sources"]["event_contract_id"] == "rollout_eval_default" and exp_l["morphology_causal_event_contract_id"] is None and exp_l["causal_ledger_required"] is False
        exp_cli = A.expected_runtime(cfg_bin, defaults, {"binary_phase_fsm_mode": "binary_shadow"})
        assert exp_cli["event_source_expected"] == "legacy_events" and exp_cli["contract_sources"]["binary_phase_fsm_mode"] == "cli"
        expect(lambda: A.expected_runtime({"grf": {"binary_phase_fsm_mode": "binary_active"}, "reward": {"morphology_phase_mode": "event_anchored_causal_delayed_experimental"}}, defaults, {}), A.ProvenanceError, "causal mode without contract", "morphology_causal_event_contract_id absent")
        assert A.binary_v3_counters_required("v3", A.EVENT_SOURCE_BINARY_ACTIVE) and not A.binary_v3_counters_required("v3", "binary_active") and not A.binary_v3_counters_required("v2", A.EVENT_SOURCE_BINARY_ACTIVE)
        ok("expected_runtime: exact event_source (binary_active_v26 iff mode binary_active, else phase_fsm_input_mode), contract ids from config/CLI/default, causal contract mandatory with causal mode, counters required only on exact v3+binary_active_v26")

        # ---------- nominal job: kept fixes --------------------------------------------
        h0 = fx.make_output("H0_nom", steps, candidate="B0820_H0", legacy_events=[{"side": "left", "event": "heel_strike"}, {"side": "left", "event": "heel_strike"}, {"side": "left", "event": "toe_off"}, {"side": "right", "event": "heel_strike"}])
        res = analyse(h0, "B0820_H0__v3_canonical__nominal__det", cand="B0820_H0")
        assert res["verdict"] == "PASS_ANALYSED", res.get("verdict_reason")
        m = res["metrics"]
        assert abs(m["tracking_served_vs_actual"]["knee"]["rmse"] - 0.01) < 1e-9 and abs(m["served_reference"]["knee"]["rom"] - 0.6) < 2e-3
        assert abs(m["sea_controls_normalized"]["knee"]["abs_max"] - 0.5) < 1e-3 and A.STO_SEMANTICS[A.SEA_CONTROLS_FILE].startswith("comandi SEA normalizzati")
        assert set(m["time_grids_verified_against_kinematics"]) == set(A.EXPECTED_STO) - {A.KINEMATICS_FILE}
        hz = m["horizon"]
        assert hz["env_horizon_steps_expected"] == 200 and hz["progress_upper_bound"] == 200 and hz["horizon_completed"] is True and hz["loop_cap_hit"] is False and hz["sources"]["max_steps"] == "rollout_eval_default" and m["horizon_provenance"]["rollout_eval_sha256"] == fx.rollout_eval_sha
        assert m["fsm_counters"]["resync_count"] == {"present": True, "value": 0.0} and m["fsm_counters"]["hs_cancelled_count"]["value"] == 1.0 and m["fsm_counters"]["required_on_this_runtime"] is True
        assert m["causal_ledger"]["timeout_transition_count"] == {"present": True, "value": 0.0} and m["causal_ledger"]["required_by_config"] is True and m["causal_ledger"]["event_contract_id"] == BINARY_CONTRACT and m["events_legacy_left"] == {"present": True, "heel_strike": 2, "toe_off": 1}
        assert "actor_fsm_events_summary" in m and "events_v3" not in json.dumps(m) and m["fsm_runtime"]["expected"]["version_source"] == "cli"
        assert m["abi"] == {**m["abi"], "n_actor": 35, "n_observation": 84, "n_observation_expected": 84, "action_shape": [2], "registry_width": 35, "actor_prefix_exact": True, "actor_equals_manifest": True, "include_controller_diagnostic_observation": False, "actor_feature_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256} and res["evidence"]["trace"]["sequence_ok"] is True and res["evidence"]["trace"]["causal_contract_constant"] is True
        assert m["fsm_runtime"]["event_source"] == A.EVENT_SOURCE_BINARY_ACTIVE and m["fsm_runtime"]["binary_phase_event_contract_id"] == BINARY_CONTRACT and res["provenance"]["candidate_width"] == 35
        assert m["sea_torques"]["ankle"]["tau_spring"]["sign_changes"] == 3 and m["reserve"]["prosthetic_abs_max_nm"] == 0.0 and m["reserve"]["columns"] == 3
        ok("nominal job PASS_ANALYSED: q_ref tracking 0.01, u normalized, all grids shared, horizon completed (200/200), counters present, ABI 35/84/[2], exact contracts, neutral naming")
        jul_nom = fx.make_output("JUL_nom_legacy", steps, causal={}, **LEGACY)
        res = analyse_jul(jul_nom)
        assert res["verdict"] == "PASS_ANALYSED", res.get("verdict_reason")
        assert res["metrics"]["fsm_runtime"]["expected"]["event_source_expected"] == "legacy_events" and res["metrics"]["fsm_runtime"]["expected"]["morphology_causal_event_contract_id"] is None and res["metrics"]["causal_ledger"]["present"] is False and res["metrics"]["causal_ledger"]["event_contract_id"] is None and res["metrics"]["fsm_counters"]["required_on_this_runtime"] is False
        ok("legacy July-like job PASS_ANALYSED: event_source exactly legacy_events, contract defaults from rollout_eval argparse, causal ledger absent (never zero)")

        # ---------- global completeness ------------------------------------------------
        payload = A.analyse_matrix([
            describe("B0820_H0__v3_canonical__nominal__det", "det", A.ISOMETRIC_CLASS, "B0820_H0", "nominal", "deterministic", 123, h0),
            describe("B0820_V3_LAST__v3_canonical__nominal__det", "det", A.ISOMETRIC_CLASS, "B0820_V3_LAST", "nominal", "deterministic", 123, tmp / "missing_dir"),
        ], verify=no_verify, rollout_eval_path=fx.rollout_eval)
        assert payload["verdict_counts"] == {"PASS_ANALYSED": 1, "NOT_EXECUTED": 1} and payload["analysis_complete"] is False and payload["executed_subset_complete"] is True
        ok("global gate: one NOT_EXECUTED job -> analysis_complete False (executed_subset_complete True)")

        # ---------- ABI: widths, candidate, action shape, feature names, sequence ------------
        res = analyse(fx.make_output("abi_obs36", steps, obs_width=36))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("actor_observation_vector_before width 36 != expected 35" in p for p in res["evidence"]["trace"]["payload_problems"]), res.get("verdict_reason")
        res = analyse(fx.make_output("abi_cand_mismatch", steps, candidate="JUL_H0"))
        assert res["verdict"] == "FAIL_PROVENANCE" and "!= job candidate" in res["verdict_reason"]
        res = analyse(fx.make_output("abi_cand_unknown", steps, candidate="NOT_A_CANDIDATE"))
        assert res["verdict"] == "FAIL_PROVENANCE" and "unknown to the F0 registry" in res["verdict_reason"]
        def analyse_39(out_dir: Path):
            return analyse(out_dir, "V26_39D__v26_imitation_native__nominal__det", cls="compatibility_control_39D", cand="V26_39D", runtime="v26_imitation_native", family="ctrl39")

        res = analyse_39(fx.make_output("abi_registry_width", steps, candidate="V26_39D", n_actor=35))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("n_actor 35 != registry width 39" in p for p in res["evidence"]["summary"]["problems"])
        res = analyse_39(fx.make_output("abi_39_ok", steps, candidate="V26_39D", n_actor=39))
        assert res["verdict"] == "PASS_ANALYSED", res.get("verdict_reason")
        assert res["metrics"]["abi"] == {**res["metrics"]["abi"], "n_actor": 39, "n_observation": 88, "n_observation_expected": 88, "registry_width": 39, "actor_equals_manifest": True, "actor_prefix_exact": True, "actor_feature_manifest_sha256": C.ACTOR_MANIFEST_39_SHA256}
        m39 = C.load_actor_feature_manifest("V26_39D")["actor_feature_names"]
        res = analyse_39(fx.make_output("abi_43_actor", steps, candidate="V26_39D", n_actor=43, actor_names=m39 + DIAG, obs_names=m39 + DIAG + PRIV45))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("n_actor 43 != registry width 39" in p for p in res["evidence"]["summary"]["problems"]) and any("controller diagnostics" in p for p in res["evidence"]["summary"]["problems"])
        ok("ctrl39 ABI: 39 manifest names + 88 observation PASS; n_actor 43 (diagnostics inside the actor) rejected on width AND diagnostics")
        res = analyse(fx.make_output("abi_action3", steps, action_width=3))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("raw_policy_action width 3 != expected 2" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("abi_action1", steps, action_width=1))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("applied_policy_action width 1 != expected 2" in p for p in res["evidence"]["trace"]["payload_problems"])
        for label, overrides in (("action_shape [2,1]", {"action_shape": [2, 1]}), ("action_shape [3]", {"action_shape": [3]}), ("action_shape scalar", {"action_shape": 2}), ("n_actor 34 vs 35 names", {"n_actor": 34}), ("n_observation 83 vs 84 names", {"n_observation": 83}), ("n_actor zero", {"n_actor": 0, "actor_feature_names": []}), ("duplicate actor names", {"actor_feature_names": ["dup"] * 35}), ("empty obs name", {"observation_feature_names": [""] + [f"obs_f{i}" for i in range(83)]}), ("obs names not a list", {"observation_feature_names": "obs"})):
            res = analyse(fx.make_output("abi_" + label.replace(" ", "_").replace("[", "").replace("]", "").replace(",", ""), steps, summary_overrides=overrides))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["summary"]["valid"] is False, (label, res.get("verdict"), res.get("verdict_reason"))
        ok("ABI: observation width != n_actor, candidate mismatch/unknown -> FAIL_PROVENANCE, n_actor != registry width (V26_39D 35 vs 39), action width 1/3, action_shape != [2], feature-name lists (length, duplicates, empty, type) -> FAIL")

        # ---------- ABI semantics: prefix order, manifest equality, diagnostics, widths, flag ----------
        m35 = C.load_actor_feature_manifest("B0820_V3_BEST")["actor_feature_names"]
        permuted = [m35[1], m35[0]] + m35[2:]
        res = analyse(fx.make_output("sem_actor_permuted", steps, actor_names=permuted, obs_names=permuted + DIAG + PRIV45))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("!= content-addressed manifest" in p for p in res["evidence"]["summary"]["problems"]) and not any("observation_feature_names[:n_actor]" in p for p in res["evidence"]["summary"]["problems"]), res["evidence"]["summary"]["problems"]
        disjoint = [f"other_{i}" for i in range(35)]
        res = analyse(fx.make_output("sem_actor_disjoint", steps, actor_names=disjoint, obs_names=disjoint + DIAG + PRIV45))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("!= content-addressed manifest" in p for p in res["evidence"]["summary"]["problems"])
        res = analyse(fx.make_output("sem_obs_prefix_permuted", steps, obs_names=permuted + DIAG + PRIV45))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("observation_feature_names[:n_actor] != actor_feature_names (exact order required)" in p for p in res["evidence"]["summary"]["problems"])
        res = analyse(fx.make_output("sem_obs_prefix_disjoint", steps, obs_names=disjoint + DIAG + PRIV45))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("exact order required" in p for p in res["evidence"]["summary"]["problems"])
        with_diag = m35[:31] + DIAG
        res = analyse(fx.make_output("sem_diag_in_actor", steps, actor_names=with_diag, obs_names=with_diag + m35[31:] + PRIV45))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("controller diagnostics" in p for p in res["evidence"]["summary"]["problems"]) and any("!= content-addressed manifest" in p for p in res["evidence"]["summary"]["problems"])
        res = analyse(fx.make_output("sem_obs_width_83", steps, n_observation=83, obs_names=m35 + DIAG + PRIV45[:-1]))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("!= expected full observation width 84" in p for p in res["evidence"]["summary"]["problems"])
        res = analyse(fx.make_output("sem_flag_true_summary", steps, include_diag=True))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("include_controller_diagnostic_observation True != expected False" in p for p in res["evidence"]["summary"]["problems"])

        def drop_flag(series, trace, summary):
            summary.pop("include_controller_diagnostic_observation")

        res = analyse(fx.make_output("sem_flag_missing", steps, mutate=drop_flag))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("include_controller_diagnostic_observation missing" in p for p in res["evidence"]["summary"]["problems"])
        res = analyse_39(fx.make_output("sem_cfg_true_no_flag", steps, candidate="V26_39D", n_actor=39, config=cfg_diag_true))
        assert res["verdict"] == "FAIL_PROVENANCE" and "registry requires False" in res["verdict_reason"]
        res = analyse_39(fx.make_output("sem_cfg_true_with_flag", steps, candidate="V26_39D", n_actor=39, config=cfg_diag_true, extra_cli=[C.NO_CONTROLLER_DIAGNOSTIC_FLAG]))
        assert res["verdict"] == "PASS_ANALYSED" and res["provenance"]["expected_runtime"]["include_controller_diagnostic_observation_source"] == "cli", res.get("verdict_reason")
        sha_job = fx.make_output("sem_manifest_sha", steps, candidate="V26_39D", n_actor=39)  # built with the genuine pin
        pinned = C.CANDIDATES["V26_39D"]["actor_feature_manifest_sha256"]
        C.CANDIDATES["V26_39D"]["actor_feature_manifest_sha256"] = "0" * 64
        try:
            res = analyse_39(sha_job)
        finally:
            C.CANDIDATES["V26_39D"]["actor_feature_manifest_sha256"] = pinned
        assert res["verdict"] == "FAIL_PROVENANCE" and "manifest digest mismatch" in res["verdict_reason"]
        ok("ABI semantics: permuted/disjoint actor vs content-addressed manifest FAIL; permuted/disjoint observation prefix FAIL (exact order); diagnostics inside the actor FAIL; n_observation 83 vs 84 FAIL; include flag True/missing FAIL; config True without CLI flag -> FAIL_PROVENANCE, with --no-include flag PASS; manifest digest pin mismatch -> FAIL_PROVENANCE")

        def dup_step(series, trace, summary):
            trace[100]["step"] = 100

        def skip_step(series, trace, summary):
            trace[100]["step"] = 102

        def bool_step(series, trace, summary):
            trace[0]["step"] = True

        def time_flat(series, trace, summary):
            trace[100]["time"] = trace[99]["time"]

        def time_back(series, trace, summary):
            trace[100]["time"] = trace[99]["time"] - 0.02

        def time_nan(series, trace, summary):
            trace[50]["time"] = float("nan")

        for label, mut, needle in (("duplicate step", dup_step, "step 100 != expected 101"), ("skipped step", skip_step, "step 102 != expected 101"), ("bool step", bool_step, "step True != expected 1"), ("flat time", time_flat, "not strictly increasing"), ("backward time", time_back, "not strictly increasing"), ("NaN time", time_nan, "time not finite")):
            res = analyse(fx.make_output("seq_" + label.replace(" ", "_"), steps, mutate=mut))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["sequence_ok"] is False and any(needle in p for p in res["evidence"]["trace"]["payload_problems"]), (label, res.get("verdict_reason"), res["evidence"]["trace"]["payload_problems"][:3])
        ok("trace sequence: duplicate/skipped/bool step, flat/backward/NaN time -> FAIL_OUTPUT_INCOMPLETE (exact 1..steps, strictly increasing finite time)")

        # ---------- exact FSM/event contracts ----------------------------------------------
        for label, src in (("prefix binary_active", "binary_active"), ("suffix binary_active_v26x", "binary_active_v26x"), ("legacy on binary config", "legacy_events"), ("empty", "")):
            res = analyse(fx.make_output("src_" + label.replace(" ", "_"), steps, event_source=src))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("!= expected exactly 'binary_active_v26'" in p or "event_source missing" in p for p in res["evidence"]["trace"]["payload_problems"]), (label, res.get("verdict_reason"))
        res = analyse_jul(fx.make_output("src_binary_on_legacy", steps, causal={}, event_source=A.EVENT_SOURCE_BINARY_ACTIVE, **LEGACY))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("!= expected exactly 'legacy_events'" in p for p in res["evidence"]["trace"]["payload_problems"])
        for label, overrides in (("mode binary_shadow", {"binary_phase_fsm_mode": "binary_shadow"}), ("mode prefix", {"binary_phase_fsm_mode": "binary"}), ("input_mode two_sensor", {"phase_fsm_input_mode": "two_sensor"}), ("event_contract typo", {"event_contract_id": "legacy_events_v2"}), ("binary contract prefix", {"binary_phase_event_contract_id": "binary_point_v25"}), ("binary contract suffix", {"binary_phase_event_contract_id": BINARY_CONTRACT + "_x"})):
            res = analyse(fx.make_output("ctr_" + label.replace(" ", "_"), steps, summary_overrides=overrides))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("exact match required" in p for p in res["evidence"]["summary"]["problems"]), (label, res.get("verdict_reason"))
        for field in A.SUMMARY_CONTRACT_FIELDS:
            def drop(series, trace, summary, _f=field):
                summary.pop(_f)
            res = analyse(fx.make_output("ctr_missing_" + field, steps, mutate=drop))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any(f"{field} missing" in p for p in res["evidence"]["summary"]["problems"]), field
        res = analyse(fx.make_output("ctr_cli_override", steps, extra_cli=["--binary-phase-fsm-mode", "disabled"]))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["provenance"]["expected_runtime"]["event_source_expected"] == "legacy_events" and any("binary_phase_fsm_mode 'binary_active' != pinned config/CLI 'disabled'" in p for p in res["evidence"]["summary"]["problems"])
        ok("exact contracts: event_source prefix/suffix/legacy-on-binary/binary-on-legacy FAIL; summary mode/input_mode/contract ids typo, prefix, suffix or missing FAIL; CLI overrides config (no prefix acceptance anywhere)")
        res = analyse(fx.make_output("causal_contract_typo", steps, causal=dict(CAUSAL_OK, event_contract_id=BINARY_CONTRACT + "x")))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("event_contract_id" in p and "exact match required" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("causal_contract_prefix", steps, causal=dict(CAUSAL_OK, event_contract_id="binary_point_v25")))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE"
        res = analyse(fx.make_output("causal_contract_missing", steps, causal={k: v for k, v in CAUSAL_OK.items() if k != "event_contract_id"}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("event_contract_id missing on an active payload" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("causal_contract_change", steps, causal=lambda i: dict(CAUSAL_OK, event_contract_id=BINARY_CONTRACT if i < 100 else "binary_point_v26+heel_qualified_fsm_v3")))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["causal_contract_constant"] is False and any("changes along the trace" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse_jul(fx.make_output("causal_active_no_config_contract", steps, **LEGACY))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("pinned config has no reward.morphology_causal_event_contract_id" in p for p in res["evidence"]["trace"]["payload_problems"])
        ok("causal ledger contract: typo/prefix/missing/mid-trace change -> FAIL; active payload without a config contract -> FAIL")

        # ---------- horizon at job level -------------------------------------------------
        for ed, completed, env_expected in ((2.004, True, 200), (2.005, True, 200), (2.006, False, 201)):
            cfg_r = tmp / f"cfg_{ed}.yaml"
            write_config(cfg_r, ed, 0.01)
            res = analyse(fx.make_output(f"h_ratio_{ed}", steps, config=cfg_r))
            assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["horizon"]["env_horizon_steps_expected"] == env_expected and res["metrics"]["horizon"]["progress_upper_bound"] == 201 and res["metrics"]["horizon"]["horizon_completed"] is completed, (ed, res.get("verdict_reason"), res.get("metrics", {}).get("horizon"))
        cfg5 = tmp / "cfg5.yaml"
        write_config(cfg5, 5.0, 0.01)
        res = analyse(fx.make_output("h_500", 500, config=cfg5))
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["horizon"]["env_horizon_steps_expected"] == 500 and res["metrics"]["horizon"]["horizon_completed"] is True
        res = analyse(fx.make_output("h_cap", 150, extra_cli=["--max-steps", "150"], summary_overrides={"end_reason": "loop_cap"}))
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["horizon"]["loop_cap_hit"] is True and res["metrics"]["horizon"]["horizon_completed"] is False and res["metrics"]["horizon"]["expected_recorded_steps"] == 150
        res = analyse(fx.make_output("h_cap_tl", 150, extra_cli=["--max-steps", "150"]))
        assert res["metrics"]["horizon"]["horizon_completed"] is False and res["metrics"]["horizon"]["horizon_end_reason_but_step_mismatch"] is True
        res = analyse(fx.make_output("h_dataset_end", steps, summary_overrides={"end_reason": "dataset_end"}))
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["horizon"]["ended_at_dataset_end"] is True and res["metrics"]["horizon"]["horizon_completed"] is False
        res = analyse(fx.make_output("h_cli_bad", steps, extra_cli=["--max-steps"]))
        assert res["verdict"] == "FAIL_PROVENANCE" and "without value" in res["verdict_reason"]
        res = analyse(fx.make_output("h_cli_dup", steps, extra_cli=["--max-steps", "10", "--max-steps", "20"]))
        assert res["verdict"] == "FAIL_PROVENANCE" and "inconsistent" in res["verdict_reason"]
        res = analyse(fx.make_output("h_sha", steps, config_sha_override="0" * 64))
        assert res["verdict"] == "FAIL_PROVENANCE" and "digest mismatch" in res["verdict_reason"]
        res = analyse(fx.make_output("h_eval_sha", steps, rollout_eval_sha_override="1" * 64))
        assert res["verdict"] == "FAIL_PROVENANCE" and "rollout_eval.py digest mismatch" in res["verdict_reason"]
        ok("job-level horizon: 200.4/200.5 completed at 200 steps, 200.6 not (env 201), 5/0.01 = 500 completed, cap 150 -> loop_cap_hit never completion, dataset_end never equated, malformed CLI/digests -> FAIL_PROVENANCE")

        # ---------- finiteness at job level ---------------------------------------------------
        def nan_extra_reserve(series, trace, summary):
            names, data = series[A.RESERVE_FILE]
            data[7, names.index("hip_flexion_l_reserve_torque")] = math.nan

        res = analyse(fx.make_output("f_reserve_nan", steps, mutate=nan_extra_reserve))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"][A.RESERVE_FILE]["consumed_nonfinite_count"] == 1 and res["evidence"][A.RESERVE_FILE]["valid"] is False

        def inf_extra_reserve(series, trace, summary):
            names, data = series[A.RESERVE_FILE]
            data[3, names.index("hip_flexion_l_reserve_torque")] = math.inf

        res = analyse(fx.make_output("f_reserve_inf", steps, mutate=inf_extra_reserve))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE"

        def no_reserve_cols(series, trace, summary):
            names, data = series[A.RESERVE_FILE]
            series[A.RESERVE_FILE] = (["time", "pros_knee_angle_reserve_torque", "pros_ankle_angle_reserve_torque"], data[:, [0, 2, 3]])

        res = analyse(fx.make_output("f_reserve_only_required", steps, mutate=no_reserve_cols))
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["reserve"]["columns"] == 2
        res = analyse(fx.make_output("f_overflow", steps, tau_knee_value=1.7e308))
        assert res["verdict"] == "FAIL_METRICS" and "non-finite" in res["verdict_reason"]
        ok("finiteness: NaN/Inf in a non-required reserve column -> FAIL_OUTPUT_INCOMPLETE; derived power/work overflow -> FAIL_METRICS; required-only reserve file accepted")

        # ---------- causal payload continuity ---------------------------------------------------
        res = analyse(fx.make_output("c_req_all_empty", steps, causal={}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("required" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("c_req_gap", steps, causal=lambda i: {} if i == 100 else dict(CAUSAL_OK)))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["causal_continuity"] is False
        res = analyse_jul(fx.make_output("c_notreq_all_empty", steps, causal={}, **LEGACY))
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["causal_ledger"]["present"] is False and res["metrics"]["causal_ledger"]["required_by_config"] is False and res["metrics"]["causal_ledger"]["total_cancelled_transition_count"]["present"] is False
        res = analyse_jul(fx.make_output("c_notreq_active", steps, **LEGACY_CONTRACT))
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["causal_ledger"]["present"] is True and res["metrics"]["causal_ledger"]["event_contract_id"] == BINARY_CONTRACT, res.get("verdict_reason")
        res = analyse_jul(fx.make_output("c_notreq_late", steps, causal=lambda i: {} if i < 5 else dict(CAUSAL_OK), **LEGACY_CONTRACT))
        assert res["verdict"] == "PASS_ANALYSED" and res["evidence"]["trace"]["causal_ledger_first_active_step"] == 5
        res = analyse_jul(fx.make_output("c_notreq_active_empty_active", steps, causal=lambda i: {} if 50 <= i < 60 else dict(CAUSAL_OK), **LEGACY_CONTRACT))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["causal_continuity"] is False

        def decreasing_causal(i):
            p = dict(CAUSAL_OK)
            p["total_resolved_sample_count"] = 12 if i < 100 else 11
            return p

        res = analyse(fx.make_output("c_decrement", steps, causal=decreasing_causal))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["counters_monotonic"] is False
        ok("causal ledger: required+all-empty FAIL, required+gap FAIL, not-required all-empty PASS, active-continuous PASS (contract declared), late activation PASS, active-empty-active FAIL, counter decrement FAIL")

        # ---------- counters: NaN/Inf/negative/fractional/bool/absent ---------------------
        cases = {
            "resync NaN": {"fsm_counters": {"resync_count": float("nan")}},
            "hs_cancelled Inf": {"fsm_counters": {"hs_cancelled_count": float("inf")}},
            "valid_hs negative": {"fsm_counters": {"valid_hs_count": -1}},
            "valid_cycle fractional": {"fsm_counters": {"valid_cycle_count": 1.5}},
            "invalid_event bool": {"fsm_counters": {"invalid_event_count": True}},
            "resync string": {"fsm_counters": {"resync_count": "0"}},
            "valid_to missing": {"drop_fsm_counters": ("valid_to_count",)},
            "resync missing on binary v3": {"drop_fsm_counters": ("resync_count",)},
            "timeout_transition_count absent": {"causal": {"total_cancelled_transition_count": 0, "total_resolved_sample_count": 1, "total_dropped_sample_count": 0, "event_contract_id": BINARY_CONTRACT}},
            "causal dropped Inf": {"causal": {"total_cancelled_transition_count": 0, "total_resolved_sample_count": 1, "total_dropped_sample_count": float("inf"), "timeout_transition_count": 0, "event_contract_id": BINARY_CONTRACT}},
        }
        for label, kwargs in cases.items():
            res = analyse(fx.make_output("k_" + label.replace(" ", "_"), steps, **kwargs))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE", (label, res.get("verdict"), res.get("verdict_reason"))
            assert res["evidence"]["trace"]["valid"] is False and res["evidence"]["trace"]["payload_problem_count"] > 0, label
            if "NaN" in label or "Inf" in label:
                assert res["evidence"]["trace"]["finite"] is False, label
        ok("trace counters: NaN/Inf (finite False), negative, fractional, bool, string, missing required, missing timeout_transition_count -> FAIL_OUTPUT_INCOMPLETE")
        v2 = fx.make_output("V2_nom", steps, fsm_version="v2", candidate="B0820_V2_BEST", drop_fsm_counters=("resync_count", "hs_cancelled_count"))
        res = analyse(v2, "B0820_V2_BEST__v2_b0820__nominal__det", cls="historical_config_replay_on_current_code", cand="B0820_V2_BEST", runtime="v2_b0820", family="replay")
        assert res["verdict"] == "PASS_ANALYSED" and res["metrics"]["fsm_runtime"]["fsm_behaviour_version"] == "v2" and "v3" not in res["metrics"]["fsm_runtime"]["note"]
        assert res["metrics"]["fsm_counters"]["resync_count"] == {"present": False, "value": None} and res["metrics"]["fsm_counters"]["required_on_this_runtime"] is False
        ok("v2 replay: reported as v2, absent resync/hs_cancelled reported present=False (never zero)")

        # ---------- runtime coherence -------------------------------------------------------
        def flip_version(series, trace, summary):
            trace[100]["phase_fsm"]["fsm_behaviour_version"] = "v2"

        res = analyse(fx.make_output("flip_version", steps, mutate=flip_version))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["constant_runtime"] is False

        def flip_source(series, trace, summary):
            trace[50]["phase_fsm"]["event_source"] = "legacy_events"

        res = analyse(fx.make_output("flip_source", steps, mutate=flip_source))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["constant_runtime"] is False
        res = analyse(fx.make_output("cli_v2_trace_v3", steps, cli_version="v2"))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("!= expected 'v2'" in p for p in res["evidence"]["trace"]["payload_problems"])

        def decreasing_counter(series, trace, summary):
            trace[150]["phase_fsm"]["valid_hs_count"] = 0

        res = analyse(fx.make_output("decreasing", steps, mutate=decreasing_counter))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["counters_monotonic"] is False
        res = analyse(fx.make_output("summary_vs_trace", steps, summary_overrides={"phase_valid_cycle_count": 2}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["summary_coherent"] is False
        ok("runtime coherence: version/source changing mid-trace, trace version != receipt CLI, decreasing counter, summary cycles 2 vs trace 1 -> FAIL_OUTPUT_INCOMPLETE")

        # ---------- FSM v3 heel-strike cancellation contract and summary == running max ----------------
        def cancel_at(step_idx, d_hs=-1, d_cancel=1, d_invalid=1, hs_before=None):
            def mut(series, trace, summary):
                for i, r in enumerate(trace):
                    f = r["phase_fsm"]
                    if hs_before is not None and 50 <= i < step_idx:
                        f["valid_hs_count"] = hs_before
                    if i >= step_idx:
                        f["valid_hs_count"] = (hs_before if hs_before is not None else f["valid_hs_count"]) + d_hs
                        if "hs_cancelled_count" in f:  # absent on v2/legacy payloads
                            f["hs_cancelled_count"] = f["hs_cancelled_count"] + d_cancel
                        f["invalid_event_count"] = f["invalid_event_count"] + d_invalid
            return mut

        res = analyse(fx.make_output("hs_cancel_ok", steps, mutate=cancel_at(100), summary_overrides={"phase_valid_hs_count": 2, "invalid_event_count": 1}))
        assert res["verdict"] == "PASS_ANALYSED", res.get("verdict_reason")
        tr = res["evidence"]["trace"]
        assert tr["hs_cancellation_contract_applied"] is True and tr["hs_cancellations_observed"] == 1 and tr["counters_monotonic"] is True and tr["summary_coherent"] is True
        assert tr["summary_vs_trace"]["phase_valid_hs_count"] == {"summary": 2.0, "trace_max": 2.0, "trace_last": 1.0}
        fc = res["metrics"]["fsm_counters"]
        assert fc["valid_hs_count_live_last_step"] == {"present": True, "value": 1.0} and fc["accepted_hs_count"] == {"present": True, "value": 3.0} and fc["hs_cancelled_count"]["value"] == 2.0
        assert res["metrics"]["actor_fsm_events_summary"]["phase_valid_hs_count"] == 2 and res["metrics"]["actor_fsm_events_trace_max"]["valid_hs_count"]["value"] == 2.0 and res["metrics"]["actor_fsm_events_trace_last_step"]["valid_hs_count"]["value"] == 1.0
        res = analyse(fx.make_output("hs_cancel_terminal", steps, mutate=cancel_at(steps - 1), summary_overrides={"phase_valid_hs_count": 2, "invalid_event_count": 1}))
        assert res["verdict"] == "PASS_ANALYSED" and res["evidence"]["trace"]["summary_vs_trace"]["phase_valid_hs_count"]["trace_last"] == 1.0, res.get("verdict_reason")
        res = analyse(fx.make_output("hs_cancel_summary_last", steps, mutate=cancel_at(steps - 1), summary_overrides={"phase_valid_hs_count": 1, "invalid_event_count": 1}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["summary_coherent"] is False and any("running max" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("hs_drop_no_cancel", steps, mutate=cancel_at(100, d_cancel=0, d_invalid=0), summary_overrides={"phase_valid_hs_count": 2}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["counters_monotonic"] is False and any("without a matching heel-strike cancellation" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("hs_drop_no_invalid", steps, mutate=cancel_at(100, d_invalid=0), summary_overrides={"phase_valid_hs_count": 2}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("without a matching heel-strike cancellation" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("hs_drop_by_two", steps, mutate=cancel_at(100, d_hs=-2, hs_before=3), summary_overrides={"phase_valid_hs_count": 3, "invalid_event_count": 1}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("decreases by 2" in p for p in res["evidence"]["trace"]["payload_problems"]) and any("accepted heel strikes" in p for p in res["evidence"]["trace"]["payload_problems"])
        res = analyse(fx.make_output("hs_drop_v2", steps, fsm_version="v2", candidate="B0820_V2_BEST", drop_fsm_counters=("resync_count", "hs_cancelled_count"), mutate=cancel_at(100, d_cancel=0), summary_overrides={"phase_valid_hs_count": 2, "invalid_event_count": 1}), "B0820_V2_BEST__v2_b0820__nominal__det", cls="historical_config_replay_on_current_code", cand="B0820_V2_BEST", runtime="v2_b0820", family="replay")
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["hs_cancellation_contract_applied"] is False and any("valid_hs_count decreases along the trace" in p for p in res["evidence"]["trace"]["payload_problems"])
        ok("FSM v3 cancellation contract: -1/+1/+1 PASS (live 1, accepted 3, summary = running max 2); terminal cancellation with summary = max PASS, summary = last FAIL; drop without cancellation, without invalid, by 2 (accepted HS decreases) FAIL; any drop on v2 FAIL")
        # v2 payload that happens to carry hs_cancelled_count: the contract is NOT applied, any drop fails
        res = analyse(fx.make_output("hs_cancel_on_v2", steps, fsm_version="v2", candidate="B0820_V2_BEST", mutate=cancel_at(100), summary_overrides={"phase_valid_hs_count": 2, "invalid_event_count": 1}), "B0820_V2_BEST__v2_b0820__nominal__det", cls="historical_config_replay_on_current_code", cand="B0820_V2_BEST", runtime="v2_b0820", family="replay")
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["hs_cancellation_contract_applied"] is False and any("valid_hs_count decreases along the trace" in p for p in res["evidence"]["trace"]["payload_problems"]), res.get("verdict_reason")
        # exact integer arithmetic beyond 2**53
        BIG = 2**53

        def big_valid_to(series, trace, summary):
            for i, r in enumerate(trace):
                r["phase_fsm"]["valid_to_count"] = BIG + 1 if i < 100 else BIG

        res = analyse(fx.make_output("big_valid_to_drop", steps, mutate=big_valid_to, summary_overrides={"phase_valid_to_count": BIG + 1}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["counters_monotonic"] is False and any("valid_to_count decreases" in p for p in res["evidence"]["trace"]["payload_problems"]), res.get("verdict_reason")

        def big_hs_drop_two(series, trace, summary):
            for i, r in enumerate(trace):
                f = r["phase_fsm"]
                if 50 <= i < 100:
                    f["valid_hs_count"] = BIG + 1
                elif i >= 100:
                    f["valid_hs_count"] = BIG - 1  # real drop of 2; in float64 (2**53+1 -> 2**53) it would look like -1
                    f["hs_cancelled_count"] = f["hs_cancelled_count"] + 1
                    f["invalid_event_count"] = f["invalid_event_count"] + 1

        res = analyse(fx.make_output("big_hs_drop_two", steps, mutate=big_hs_drop_two, summary_overrides={"phase_valid_hs_count": BIG + 1, "invalid_event_count": 1}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and any("decreases by 2" in p for p in res["evidence"]["trace"]["payload_problems"]), res.get("verdict_reason")

        def big_cycles(series, trace, summary):
            for i, r in enumerate(trace):
                r["phase_fsm"]["valid_cycle_count"] = BIG + 1 if i >= 2 else BIG

        res = analyse(fx.make_output("big_cycles_exact", steps, mutate=big_cycles, summary_overrides={"phase_valid_cycle_count": BIG + 1}))
        assert res["verdict"] == "PASS_ANALYSED", res.get("verdict_reason")
        svt = res["evidence"]["trace"]["summary_vs_trace"]["phase_valid_cycle_count"]
        assert svt == {"summary": BIG + 1, "trace_max": BIG + 1, "trace_last": BIG + 1} and isinstance(svt["trace_max"], int) and res["metrics"]["actor_fsm_events_trace_max"]["valid_cycle_count"]["value"] == BIG + 1 and isinstance(res["metrics"]["actor_fsm_events_trace_max"]["valid_cycle_count"]["value"], int)
        res = analyse(fx.make_output("big_cycles_summary_off_by_one", steps, mutate=big_cycles, summary_overrides={"phase_valid_cycle_count": BIG}))
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["trace"]["summary_coherent"] is False, res.get("verdict_reason")
        assert isinstance(A.counter({"x": 2.0}, "x")["value"], int) and A.counter({"x": BIG + 1}, "x")["value"] == BIG + 1 and A.as_int(3.0) == 3 and A.is_nonneg_int(BIG + 1) and A.is_finite_float(10**400)
        ok("integer precision: v2 with hs_cancelled_count and -1/+1/+1 FAIL (contract not applied); valid_to 2**53+1 -> 2**53 FAIL; v3 real drop of 2 across 2**53 FAIL (float would see -1); summary == trace max exact at 2**53+1 PASS, off by one FAIL; counters kept as exact ints")

        # ---------- CLI: --help exits 0 without side effects, unknown arguments exit 2 -------------------
        script = HERE / "f0_matrix_analysis.py"

        def listing(p: Path) -> set:
            return set(os.listdir(p)) if p.is_dir() else set()

        before = (listing(C.OUT_METRICS), listing(C.OUT_ROLLOUTS), listing(C.OUT_ROOT))
        help_run = subprocess.run([sys.executable, str(script), "--help"], capture_output=True, text=True, cwd=str(tmp))
        bogus_run = subprocess.run([sys.executable, str(script), "--bogus"], capture_output=True, text=True, cwd=str(tmp))
        after = (listing(C.OUT_METRICS), listing(C.OUT_ROLLOUTS), listing(C.OUT_ROOT))
        assert help_run.returncode == 0 and "usage:" in help_run.stdout and "f0_matrix_analysis" in help_run.stdout, (help_run.returncode, help_run.stdout[:200], help_run.stderr[-300:])
        assert bogus_run.returncode == 2 and "unrecognized arguments" in bogus_run.stderr, (bogus_run.returncode, bogus_run.stderr[-300:])
        assert before == after and not any(n.startswith("f0_matrix_analysis_") and n not in before[0] for n in after[0]) and not (tmp / "metrics").exists()
        ok("CLI: --help exits 0 and --bogus exits 2 without creating any artefact (metrics/, rollouts/ and root listings unchanged, nothing written in the temp cwd)")

        # ---------- summary schema ---------------------------------------------------------
        for label, overrides in (("missing reserve", None), ("string return", {"episode_return": "2.0"}), ("bool steps", {"steps": True}), ("negative clipped", {"action_clipped_steps": -1}), ("NaN penetration", {"grf_penetration_max_m": float("nan")}), ("ok false", {"ok": False}), ("empty end_reason", {"end_reason": ""}), ("n_actor string", {"n_actor": "35"}), ("n_observation negative", {"n_observation": -1})):
            def mut(series, trace, summary, _o=overrides):
                if _o is None:
                    summary.pop("reserve_norm_max_nm")
                else:
                    summary.update(_o)
            res = analyse(fx.make_output("s_" + label.replace(" ", "_"), steps, mutate=mut))
            assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence"]["summary"]["valid"] is False, (label, res.get("verdict"))
        ok("summary schema: missing field, string, bool, negative, NaN, ok False, empty end_reason, n_actor/n_observation invalid -> FAIL_OUTPUT_INCOMPLETE")

        # ---------- grids at job level ---------------------------------------------------------
        def dup_time(series, trace, summary):
            names, data = series[A.KINEMATICS_FILE]
            data[10, names.index("time")] = data[9, names.index("time")]

        res = analyse(fx.make_output("g_dup", steps, mutate=dup_time))
        assert res["verdict"] == "FAIL_METRICS" and "not strictly increasing" in res["verdict_reason"]

        def shift_torque(series, trace, summary):
            names, data = series[A.SEA_TORQUES_FILE]
            data[:, names.index("time")] += 0.0004

        res = analyse(fx.make_output("g_shift", steps, mutate=shift_torque))
        assert res["verdict"] == "FAIL_METRICS" and "time grid mismatch" in res["verdict_reason"]
        ok("job-level grids: duplicate time and shifted torque grid fail before metrics")

        # ---------- comparisons, stochastic, markdown, no-clobber -------------------------------
        best = fx.make_output("BEST_nom", steps, knee_amp=0.4, summary_overrides={"episode_return": 3.5})
        jul = fx.make_output("JUL_nom", steps, knee_amp=0.5, candidate="JUL_H0", summary_overrides={"episode_return": 40.0})
        c39 = fx.make_output("C39_nom", steps, knee_amp=0.45, candidate="V26_39D", n_actor=39, summary_overrides={"episode_return": 300.0})
        st1 = fx.make_output("H0_s123", steps, candidate="B0820_H0", summary_overrides={"episode_return": -1.0})
        st2 = fx.make_output("H0_s124", steps, candidate="B0820_H0", summary_overrides={"episode_return": -3.0, "end_reason": "grf_penetration"})
        # the JUL_H0 collision of artefact 004504: a replay-family stochastic job (july_legacy, seed 123) with the same
        # (start, candidate) as the contemporaneous stoch JUL_H0 series under v3_canonical (seeds 123/124)
        jul_replay_s123 = fx.make_output("JUL_replay_s123", steps, causal={}, summary_overrides={"episode_return": 55.0}, **LEGACY)
        jul_v3_s123 = fx.make_output("JUL_v3_s123", steps, candidate="JUL_H0", summary_overrides={"episode_return": 30.0})
        jul_v3_s124 = fx.make_output("JUL_v3_s124", steps, candidate="JUL_H0", summary_overrides={"episode_return": 20.0})
        described = [
            describe("B0820_H0__v3_canonical__nominal__det", "det", A.ISOMETRIC_CLASS, "B0820_H0", "nominal", "deterministic", 123, h0),
            describe("B0820_V3_BEST__v3_canonical__nominal__det", "det", A.ISOMETRIC_CLASS, "B0820_V3_BEST", "nominal", "deterministic", 123, best),
            describe("B0820_V2_BEST__v2_b0820__nominal__det", "replay", "historical_config_replay_on_current_code", "B0820_V2_BEST", "nominal", "deterministic", 123, v2, runtime="v2_b0820"),
            describe("JUL_H0__v3_canonical__nominal__det", "det", "historical_control", "JUL_H0", "nominal", "deterministic", 123, jul),
            describe("V26_39D__v26_imitation_native__nominal__det", "ctrl39", "compatibility_control_39D", "V26_39D", "nominal", "deterministic", 123, c39, runtime="v26_imitation_native"),
            describe("B0820_H0__v3_canonical__nominal__stoch_seed123", "stoch", A.ISOMETRIC_CLASS, "B0820_H0", "nominal", "stochastic", 123, st1),
            describe("B0820_H0__v3_canonical__nominal__stoch_seed124", "stoch", A.ISOMETRIC_CLASS, "B0820_H0", "nominal", "stochastic", 124, st2),
            describe("JUL_H0__july_legacy__nominal__stoch_seed123", "replay", "historical_config_replay_on_current_code", "JUL_H0", "nominal", "stochastic", 123, jul_replay_s123, runtime="july_legacy"),
            describe("JUL_H0__v3_canonical__nominal__stoch_seed123", "stoch", "historical_control", "JUL_H0", "nominal", "stochastic", 123, jul_v3_s123),
            describe("JUL_H0__v3_canonical__nominal__stoch_seed124", "stoch", "historical_control", "JUL_H0", "nominal", "stochastic", 124, jul_v3_s124),
        ]
        payload = A.analyse_matrix(described, verify=no_verify, preflight={"checked_existing": 10, "verified_identical": 10, "invalid": {}}, rollout_eval_path=fx.rollout_eval)
        assert payload["verdict_counts"] == {"PASS_ANALYSED": 10}, payload["verdict_counts"]
        assert payload["analysis_complete"] is True and payload["executed_subset_complete"] is True
        rows = {r["candidate"]: r for r in payload["comparisons"]["deterministic"]["nominal"]["rows"]}
        assert rows["B0820_H0"]["delta_vs_B0820_H0"]["episode_return"] == 0.0 and rows["B0820_V3_BEST"]["delta_vs_B0820_H0"]["knee_rom_rad"] > 0.19
        assert all(rows[c]["isometric_vs_reference"] is False and "delta_vs_B0820_H0" not in rows[c] for c in ("JUL_H0", "V26_39D", "B0820_V2_BEST")) and rows["B0820_V2_BEST"]["metrics"]["fsm_version"] == "v2"
        agg = payload["comparisons"]["stochastic"]["nominal"]["B0820_H0"]
        assert agg["n"] == 2 and abs(agg["return_mean_min_max"][0] + 2.0) < 1e-9 and abs(agg["horizon_completed_fraction"] - 0.5) < 1e-9 and agg["family"] == "stoch" and agg["runtime"] == "v3_canonical" and agg["series_id"] == "B0820_H0__v3_canonical__nominal__stoch"
        # regression for the 004504 collision: the replay JUL_H0 seed 123 must NOT be fused with the v3 stoch series
        jul_agg = payload["comparisons"]["stochastic"]["nominal"]["JUL_H0"]
        assert jul_agg["n"] == 2 and jul_agg["seeds"] == [123, 124] and len(set(jul_agg["seeds"])) == 2 and jul_agg["runtime"] == "v3_canonical" and jul_agg["comparison_class"] == "historical_control" and jul_agg["job_ids"] == ["JUL_H0__v3_canonical__nominal__stoch_seed123", "JUL_H0__v3_canonical__nominal__stoch_seed124"] and abs(jul_agg["return_mean_min_max"][0] - 25.0) < 1e-9, jul_agg
        ctrl = payload["comparisons"]["stochastic_controls_individual"]["nominal"]
        assert len(ctrl) == 1 and ctrl[0]["job_id"] == "JUL_H0__july_legacy__nominal__stoch_seed123" and ctrl[0]["family"] == "replay" and ctrl[0]["runtime"] == "july_legacy" and ctrl[0]["comparison_class"] == "historical_config_replay_on_current_code" and ctrl[0]["aggregated"] is False and abs(ctrl[0]["metrics"]["episode_return"] - 55.0) < 1e-9
        assert all(a["family"] == "stoch" for start in payload["comparisons"]["stochastic"].values() for a in start.values())
        md = A.render_markdown(payload, "teststamp")
        assert "## Job eseguiti" in md and "v2/binary_active_v26" in md and "events_v3" not in md and "Aggregati stocastici" in md and "200/200 completato" in md and "Controlli stocastici individuali" in md and "JUL_H0__july_legacy__nominal__stoch_seed123" in md
        assert "nessun prefisso" in payload["fsm_counter_policy"] and "abi_policy" in payload and "contract_policy" in payload and "stochastic_rule" in payload["comparisons"]
        ok("comparisons: deltas only for isometric jobs; stochastic aggregates only for family=stoch (JUL_H0 v3 n=2 seeds [123,124]); the replay JUL_H0 seed 123 is an individual control, never fused (004504 collision regression); Markdown lists both")
        # fail-closed: heterogeneous series or duplicate seeds raise ComparisonError (never a silent fusion)
        analysed_jobs = {j["job_id"]: j for j in payload["jobs"]}
        base_v3 = analysed_jobs["JUL_H0__v3_canonical__nominal__stoch_seed123"]
        dup = dict(base_v3, job_id="JUL_H0__v3_canonical__nominal__stoch_seed123_dup")
        expect(lambda: A.build_comparisons([base_v3, dup]), A.ComparisonError, "duplicate seeds in a stoch series", "duplicate seeds")
        hetero_runtime = dict(analysed_jobs["JUL_H0__v3_canonical__nominal__stoch_seed124"], runtime="v2_b0820")
        expect(lambda: A.build_comparisons([base_v3, hetero_runtime]), A.ComparisonError, "heterogeneous runtime in a stoch series", "heterogeneous")
        hetero_class = dict(analysed_jobs["JUL_H0__v3_canonical__nominal__stoch_seed124"], comparison_class="isometric_comparable")
        expect(lambda: A.build_comparisons([base_v3, hetero_class]), A.ComparisonError, "heterogeneous comparison_class in a stoch series", "heterogeneous")
        mislabeled = dict(analysed_jobs["JUL_H0__july_legacy__nominal__stoch_seed123"], family="stoch")
        expect(lambda: A.build_comparisons([base_v3, mislabeled]), A.ComparisonError, "replay job mislabeled as family stoch (runtime/class differ)", "heterogeneous")
        sealed = dict(analysed_jobs["JUL_H0__v3_canonical__nominal__stoch_seed124"], seed=126)
        expect(lambda: A.build_comparisons([base_v3, sealed]), A.ComparisonError, "sealed seed in a stoch series", "sealed seed")
        ok("build_comparisons fails closed on duplicate seeds, heterogeneous runtime/class, mislabeled family and sealed seeds (no silent fusion)")
        metrics_dir = tmp / "metrics_out"
        A.write_outputs(payload, "teststamp", metrics_dir=metrics_dir)
        expect(lambda: A.write_outputs(payload, "teststamp", metrics_dir=metrics_dir), FileExistsError, "no-clobber")
        assert (metrics_dir / "f0_matrix_analysis_teststamp.json").is_file() and not (C.OUT_METRICS / "f0_matrix_analysis_teststamp.json").exists()
        ok("write_outputs: explicit temp metrics dir, refuses to overwrite, real metrics dir untouched")

        def verify_fail(rec, out_dir):
            raise RuntimeError("existing output dir fails receipt verification (synthetic)")

        res = A.analyse_job(describe("JUL_H0__v3_canonical__minus020__det", "det", "historical_control", "JUL_H0", "minus020", "deterministic", 123, jul), verify=verify_fail, rollout_eval_path=fx.rollout_eval)
        assert res["verdict"] == "FAIL_VERIFICATION"
        ok("receipt verification failure -> FAIL_VERIFICATION")
        assert str(tmp).startswith(base) and not str(tmp).startswith(str(C.REPO))
        ok(f"all synthetic data confined to {tmp} (tempfile under {base}), nothing written under the repository")
        print(f"SELFTEST PASS ({PASSED} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
