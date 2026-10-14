"""Check the adapter against the original governor class without native imports."""
import ast
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from typing import Sequence
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.interpolate import BPoly, CubicHermiteSpline, PchipInterpolator

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("random_pose_harness", HERE / "random_pose_reset.py")
harness = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(harness)


def original_adapter_class():
    """Compile only the unmodified adapter AST; never import OpenSim/Ray/env."""
    path = harness.REPO / "Trajectory Generator/osim_trj_cmc_like.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ProstheticSegmentKinematics")
    scope = dict(np=np, Sequence=Sequence, KinematicsInterpolator=object, CoordDict=dict,
                 BPoly=BPoly, CubicHermiteSpline=CubicHermiteSpline, PchipInterpolator=PchipInterpolator)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), scope)
    return scope[node.name]


class BaseIK:
    coord_names = ["biological", *harness.JOINTS]

    def get(self, t):
        return ({"biological": 2 + t, harness.JOINTS[0]: -0.18, harness.JOINTS[1]: 0.06},
                {"biological": 1.0, harness.JOINTS[0]: 0.7, harness.JOINTS[1]: -0.8},
                {"biological": 0.0, harness.JOINTS[0]: 10.0, harness.JOINTS[1]: -12.0})


def adapter():
    delegate = original_adapter_class()(BaseIK(), harness.JOINTS, ref_model="butterworth3_jerk_limited",
        ref_lpf_cutoff_hz=4.0, ref_velocity_limits=[6.0, 3.5], ref_acceleration_limits=[60.0, 55.0],
        ref_jerk_limits=[3000.0, 2750.0], control_dt=0.001)
    pose = dict(zip(harness.JOINTS, (-0.65, -0.25)))
    return harness.CoherentBootstrapKinematics(delegate, pose, np), pose


class BootstrapTests(unittest.TestCase):
    def test_postprocessing_failure_preserves_physical_completion(self):
        receipt = {"status": "rollout_complete", "completed_requested_duration": True}
        helper = SimpleNamespace(analyze=lambda output: {})
        with patch.object(harness, "render", side_effect=OSError("plot I/O failed")):
            harness.postprocess_saved(Path("unused"), receipt, helper)
        self.assertEqual(receipt["status"], "rollout_complete")
        self.assertTrue(receipt["completed_requested_duration"])
        self.assertEqual(receipt["postprocessing_error"], "OSError: plot I/O failed")
        with patch.object(harness, "render", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                harness.postprocess_saved(Path("unused"), receipt, helper)

    def test_segment_wall_timeout_is_technical_censoring(self):
        SegmentWallClockTimeout = type("SegmentWallClockTimeout", (RuntimeError,), {})
        self.assertEqual(harness.exception_category(SegmentWallClockTimeout("slow")), "segment_wall_clock_timeout_censored")

    def test_probe_uses_original_flatten_clip_wrapper_for_two_dimensional_action(self):
        import gymnasium as gym
        source = harness.REPO / "Trajectory Generator/baseline_MLP/env_factory.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "FlattenClipAction")
        scope = dict(np=np, gym=gym, spaces=gym.spaces)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), scope)
        Wrapper = scope["FlattenClipAction"]
        class Environment(gym.Env):
            action_space = gym.spaces.Box(-1.0, 1.0, shape=(1, 2), dtype=np.float32)
        inner = Wrapper(Environment())
        outer = gym.Wrapper(inner)
        applied = harness.apply_wrapped_action(outer, np.array([-2.0, 1.5]), Wrapper)
        np.testing.assert_array_equal(applied, [[-1.0, 1.0]])
        self.assertEqual(applied.shape, (1, 2))

    def test_selection_is_reconstructed_from_hashed_receipts_not_mutable_outcomes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cases = [{"id": name, "kind": "test", "pose": None} for name in ("coherent_nominal", "random_001", "random_002")]
            helper = SimpleNamespace(relative=lambda path: str(Path(path)), sha256=harness.common().sha256,
                                     read_json=harness.common().read_json)
            hashes, outcomes = {}, []
            for case, eligible in zip(cases, (True, False, True)):
                folder = root / case["id"]
                folder.mkdir()
                status = "static_eligible" if eligible else "static_rejected"
                gate = {"eligible_to_integrate": eligible}
                path = folder / "receipt.json"
                path.write_text(json.dumps({"case": case, "status": status, "static_contact_gate": gate}))
                hashes[str(path)] = helper.sha256(path)
                outcomes.append({"id": case["id"], "status": status, "gate": gate})
            plan = {"cases": cases, "random_pose_sampling": {"selected_random_count": 1}}
            selection = {"preflight_receipt_hashes": hashes, "preflight_outcomes": outcomes,
                         "cases": [cases[0], cases[2]]}
            self.assertEqual(harness.validated_selection_cases(plan, selection, root, helper), selection["cases"])
            outcomes[1].update(status="static_eligible", gate={"eligible_to_integrate": True})
            selection["cases"] = [cases[0], cases[1]]
            with self.assertRaisesRegex(RuntimeError, "hashed native receipt"):
                harness.validated_selection_cases(plan, selection, root, helper)

    def test_known_force_exception_is_censored_without_fabricated_reward(self):
        failure = FloatingPointError("Online GRF exceeds 5 BW at t=13.01")
        self.assertEqual(harness.exception_category(failure), "physical_force_guard_censored")
        self.assertEqual(harness.exception_category(RuntimeError("bad adapter")), "harness_or_native_failure")
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            receipt = {"initial_state": {"time_s": 13.0}}
            rows = [{"time": 13.01, "returned_reward": 0.75, "phase_fsm": {"valid_hs_count": 0}}]
            args = SimpleNamespace(checkpoint="frozen", setup_xml="original", episode_start_offset_s=1.0,
                                   action_mode="absolute", _cfg_reward={"reward_mode": "ex_novo"})
            harness.preserve_censored_summary(folder, receipt, rows, None, args, failure)
            summary = json.loads((folder / "rollout_summary.json").read_text())
            self.assertEqual(summary["steps"], 1)
            self.assertEqual(summary["episode_return"], 0.75)
            self.assertEqual(summary["last_completed_transition_time_s"], 13.01)
            self.assertTrue(summary["truncated"])
            self.assertEqual(json.loads((folder / "rollout_policy_trace.json").read_text()), rows)

    def test_random_pose_and_zero_derivatives_preserve_biological_ik(self):
        kin, pose = adapter()
        for t in (13.0, 13.001, 13.01):
            q, v, a = kin.get(t)
            self.assertEqual(q["biological"], 2 + t)
            self.assertEqual(v["biological"], 1.0)
            self.assertEqual(a["biological"], 0.0)
            for name in harness.JOINTS:
                self.assertEqual(q[name], pose[name])
                self.assertEqual(v[name], 0.0)
                self.assertEqual(a[name], 0.0)

    def test_first_original_governor_segment_starts_from_random_not_ik(self):
        kin, pose = adapter()
        target = np.array([[-0.65, -0.25], [-0.61, -0.21]])
        kin.set_segment(np.array([13.0, 13.01]), target, np.array([[0.0, 0.0], [4.0, 4.0]]))
        q0, v0, a0 = kin.get(13.0)
        for name in harness.JOINTS:
            self.assertAlmostEqual(q0[name], pose[name], places=12)
            self.assertAlmostEqual(v0[name], 0.0, places=12)
            self.assertAlmostEqual(a0[name], 0.0, places=12)
        q_end, _, _ = kin.get(13.01)
        self.assertNotEqual(q_end[harness.JOINTS[0]], pose[harness.JOINTS[0]])
        self.assertNotEqual(q_end[harness.JOINTS[0]], -0.18)

    def test_second_segment_keeps_original_filter_state_continuity(self):
        kin, pose = adapter()
        times = np.array([13.0, 13.01])
        kin.set_segment(times, np.array([list(pose.values()), [-0.61, -0.21]]), np.zeros((2, 2)))
        before = kin.get(13.01)
        q_start = [before[0][name] for name in harness.JOINTS]
        v_start = [before[1][name] for name in harness.JOINTS]
        kin.set_segment(np.array([13.01, 13.02]), np.array([q_start, [-0.60, -0.20]]), np.array([v_start, [1.0, 1.0]]))
        after = kin.get(13.01)
        for order in range(3):
            for name in harness.JOINTS:
                self.assertLess(abs(after[order][name] - before[order][name]), 1e-8)

    def test_queries_outside_policy_segment_cannot_fall_back_to_ik(self):
        kin, pose = adapter()
        kin.set_segment(np.array([13.0, 13.01]), np.array([list(pose.values()), [-0.61, -0.21]]), np.zeros((2, 2)))
        with self.assertRaisesRegex(RuntimeError, "IK fallback prohibited"):
            kin.get(13.02)

    def test_repeated_clear_resets_filter_memory_and_reference(self):
        kin, pose = adapter()
        kin.set_segment(np.array([13.0, 13.01]), np.array([list(pose.values()), [-0.61, -0.21]]), np.zeros((2, 2)))
        kin.clear_segment()
        np.testing.assert_array_equal(kin.delegate._filt_q, list(pose.values()))
        np.testing.assert_array_equal(kin.delegate._filt_v, [0.0, 0.0])
        np.testing.assert_array_equal(kin.delegate._filt_a, [0.0, 0.0])
        self.assertEqual(kin.set_segment_count, 0)
        self.assertIsNone(kin.first_segment)
        self.assertEqual(kin.get(13.0)[0][harness.JOINTS[0]], pose[harness.JOINTS[0]])

    def test_domain_rejects_nonfinite_outside_and_biological_coordinates(self):
        for pose in ({harness.JOINTS[0]: float("nan"), harness.JOINTS[1]: 0.0},
                     {harness.JOINTS[0]: -1.21, harness.JOINTS[1]: 0.0},
                     {harness.JOINTS[0]: -0.2, harness.JOINTS[1]: 0.56},
                     {harness.JOINTS[0]: -0.2, "biological": 0.0}):
            with self.assertRaises(ValueError):
                harness.validate_pose(pose)


if __name__ == "__main__":
    unittest.main()
