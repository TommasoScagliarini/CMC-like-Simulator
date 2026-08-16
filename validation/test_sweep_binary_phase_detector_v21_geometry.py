"""Deterministic tests for the V21 binary-detector geometry sweep."""

from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = ROOT / "Trajectory Generator"
if str(TRAJECTORY_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_ROOT))
MODULE_PATH = ROOT / "validation" / "sweep_binary_phase_detector_v21_geometry.py"
SPEC = importlib.util.spec_from_file_location(
    "sweep_binary_phase_detector_v21_geometry_under_test",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
subject = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = subject
SPEC.loader.exec_module(subject)

from binary_phase_fsm import BinaryPhaseFSM  # noqa: E402


class ManualClock:
    """Monotonic clock whose value is controlled explicitly by each test."""

    def __init__(self, start: float = 100.0) -> None:
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += float(seconds)


class FakeTTY(io.StringIO):
    def isatty(self) -> bool:
        return True


class FakeNonTTY(io.StringIO):
    def isatty(self) -> bool:
        return False


class BrokenStream:
    """Stream failures must never abort the numerical sweep."""

    def isatty(self) -> bool:
        raise OSError("no terminal metadata")

    def write(self, _value: str) -> int:
        raise OSError("stream closed")

    def flush(self) -> None:
        raise OSError("stream closed")


class SweepProgressTests(unittest.TestCase):
    def test_tty_initial_frame_and_eta_are_deterministic(self) -> None:
        clock = ManualClock()
        stream = FakeTTY()
        progress = subject.SweepProgress(
            total=4,
            label="V21 sweep",
            stream=stream,
            width=4,
            min_redraw_interval_s=0.1,
            clock=clock,
        )

        initial = stream.getvalue()
        self.assertTrue(initial.startswith("\rV21 sweep [----]"))
        self.assertIn("0/4", initial)
        self.assertIn("0.00%", initial)
        self.assertIn("elapsed 00:00:00", initial)
        self.assertIn("ETA --:--:--", initial)
        self.assertNotIn("\n", initial)

        clock.advance(10.0)
        progress.update(1)
        rendered = stream.getvalue()

        self.assertIn("\rV21 sweep [#---]", rendered)
        self.assertIn("1/4", rendered)
        self.assertIn("25.00%", rendered)
        self.assertIn("elapsed 00:00:10", rendered)
        self.assertIn("ETA 00:00:30", rendered)

    def test_tty_throttles_intermediate_update_but_finish_is_forced(self) -> None:
        clock = ManualClock()
        stream = FakeTTY()
        progress = subject.SweepProgress(
            total=4,
            label="V21",
            stream=stream,
            width=4,
            min_redraw_interval_s=1.0,
            clock=clock,
        )
        initial = stream.getvalue()

        clock.advance(0.25)
        progress.update(1)
        self.assertEqual(stream.getvalue(), initial)

        clock.advance(0.25)
        progress.finish("INTERRUPTED")
        final = stream.getvalue()
        self.assertNotEqual(final, initial)
        self.assertIn("1/4", final)
        self.assertIn("INTERRUPTED", final)
        self.assertTrue(final.endswith("\n"))
        self.assertEqual(final.count("\n"), 1)

    def test_non_tty_uses_newlines_and_periodic_throttling(self) -> None:
        clock = ManualClock()
        stream = FakeNonTTY()
        progress = subject.SweepProgress(
            total=4,
            label="V21",
            stream=stream,
            width=4,
            non_tty_interval_s=30.0,
            clock=clock,
        )
        initial = stream.getvalue()

        self.assertEqual(initial.count("\n"), 1)
        self.assertNotIn("\r", initial)
        self.assertIn("0/4", initial)

        clock.advance(5.0)
        progress.update(1)
        self.assertEqual(stream.getvalue(), initial)

        clock.advance(25.0)
        progress.update(2)
        periodic = stream.getvalue()
        self.assertEqual(periodic.count("\n"), 2)
        self.assertIn("2/4", periodic)
        self.assertIn("50.00%", periodic)
        self.assertIn("elapsed 00:00:30", periodic)
        self.assertIn("ETA 00:00:30", periodic)

        clock.advance(1.0)
        progress.finish("STOPPED")
        final = stream.getvalue()
        self.assertEqual(final.count("\n"), 3)
        self.assertNotIn("\r", final)
        self.assertIn("STOPPED", final.splitlines()[-1])

    def test_completed_sweep_has_one_final_frame_and_finish_is_idempotent(self) -> None:
        for stream in (FakeTTY(), FakeNonTTY()):
            with self.subTest(tty=stream.isatty()):
                clock = ManualClock()
                progress = subject.SweepProgress(
                    total=2,
                    label="V21",
                    stream=stream,
                    width=4,
                    min_redraw_interval_s=10.0,
                    non_tty_interval_s=10.0,
                    clock=clock,
                )

                clock.advance(2.0)
                progress.update(2)
                progress.finish("DONE")
                once_finished = stream.getvalue()
                progress.finish("DONE")

                self.assertEqual(stream.getvalue(), once_finished)
                self.assertEqual(once_finished.count("2/2"), 1)
                self.assertEqual(once_finished.count("100.00%"), 1)
                self.assertIn("[####]", once_finished)
                self.assertIn("elapsed 00:00:02", once_finished)
                self.assertIn("ETA 00:00:00", once_finished)
                if stream.isatty():
                    self.assertTrue(once_finished.endswith("\n"))
                else:
                    self.assertNotIn("\r", once_finished)

    def test_completed_counter_must_be_monotonic_and_in_range(self) -> None:
        progress = subject.SweepProgress(
            total=3,
            label="V21",
            stream=FakeNonTTY(),
            clock=ManualClock(),
        )

        with self.assertRaises(TypeError):
            progress.update(1.0)
        with self.assertRaises(TypeError):
            progress.update(True)

        progress.update(1)
        with self.assertRaises(ValueError):
            progress.update(0)
        with self.assertRaises(ValueError):
            progress.update(4)
        with self.assertRaises(ValueError):
            progress.advance(-1)
        with self.assertRaises(ValueError):
            progress.advance(1.0)
        with self.assertRaises(ValueError):
            progress.advance(True)

    def test_constructor_rejects_invalid_dimensions(self) -> None:
        for invalid_total in (0, -1, 1.0, True):
            with self.subTest(total=invalid_total):
                with self.assertRaises(ValueError):
                    subject.SweepProgress(
                        total=invalid_total,
                        label="V21",
                        stream=FakeNonTTY(),
                    )

        for invalid_width in (0, -1, 1.0, True):
            with self.subTest(width=invalid_width):
                with self.assertRaises(ValueError):
                    subject.SweepProgress(
                        total=1,
                        label="V21",
                        width=invalid_width,
                        stream=FakeNonTTY(),
                    )

    def test_duration_formatter_covers_known_and_unknown_values(self) -> None:
        self.assertEqual(subject.format_duration(0.0), "00:00:00")
        self.assertEqual(subject.format_duration(3661.0), "01:01:01")
        self.assertEqual(subject.format_duration(None), "--:--:--")
        self.assertEqual(subject.format_duration(float("inf")), "--:--:--")
        self.assertEqual(subject.format_duration(-1.0), "--:--:--")

    def test_broken_output_stream_is_only_a_cosmetic_failure(self) -> None:
        progress = subject.SweepProgress(
            total=2,
            label="V21",
            stream=BrokenStream(),
            clock=ManualClock(),
        )

        progress.update(1)
        progress.update(2)
        progress.finish("DONE")
        self.assertTrue(progress.finished)


class SweepCoreTests(unittest.TestCase):
    @staticmethod
    def actual_event_signature(
        time_s: np.ndarray, heel: np.ndarray, toe: np.ndarray
    ) -> list[tuple]:
        fsm = BinaryPhaseFSM()
        fsm.reset(
            time_s=float(time_s[0]),
            heel_contact=bool(heel[0]),
            toe_contact=bool(toe[0]),
        )
        events: list[dict] = []
        first = 1
        while first < time_s.size:
            stop = min(first + subject.POLICY_SAMPLES, time_s.size)
            boundary = float(time_s[stop - 1])
            for index in range(first, stop):
                payload = fsm.update_sample(
                    time_s=float(time_s[index]),
                    heel_contact=bool(heel[index]),
                    toe_contact=bool(toe[index]),
                    delivered_time_s=boundary,
                )
                events.extend(payload["events_this_step"])
            first = stop
        return subject.event_signature(events)

    def test_fast_fsm_matches_frozen_fsm_on_random_traces(self) -> None:
        generator = np.random.default_rng(20260804)
        for case_index in range(500):
            sample_count = int(generator.integers(2, 251))
            time_s = np.arange(sample_count, dtype=float) / 1000.0
            heel = generator.integers(0, 2, sample_count).astype(bool)
            toe = generator.integers(0, 2, sample_count).astype(bool)
            with self.subTest(case=case_index, samples=sample_count):
                fast = subject.event_signature(
                    subject.fast_fsm_events(time_s, heel, toe)
                )
                self.assertEqual(
                    fast, self.actual_event_signature(time_s, heel, toe)
                )

    def test_fast_fsm_confirms_after_exactly_five_elapsed_ms(self) -> None:
        time_s = np.arange(8, dtype=float) / 1000.0
        heel = np.array([False, True, True, True, True, True, True, False])
        toe = np.zeros(time_s.size, dtype=bool)
        events = subject.fast_fsm_events(time_s, heel, toe)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["event"], "heel_strike")
        self.assertEqual(events[0]["event_time_s"], 0.001)
        self.assertEqual(events[0]["confirmed_time_s"], 0.006)

        heel = np.array([False, True, True, True, True, True, False, False])
        self.assertEqual(subject.fast_fsm_events(time_s, heel, toe), [])

    def test_two_sensor_gate_rejects_dead_or_non_distinct_channel(self) -> None:
        heel = np.zeros(40, dtype=bool)
        toe = np.zeros(40, dtype=bool)
        heel[5:20] = True
        toe[14:30] = True
        passing = subject.two_sensor_channel_gate(heel, toe)
        self.assertTrue(passing["pass"])
        self.assertEqual(passing["minimum_stable_run_s"], 0.005)

        dead_toe = subject.two_sensor_channel_gate(heel, np.zeros(40, dtype=bool))
        self.assertFalse(dead_toe["pass"])
        self.assertFalse(dead_toe["requirements_pass"]["toe_on"])

        identical = subject.two_sensor_channel_gate(heel, heel)
        self.assertFalse(identical["pass"])
        self.assertFalse(identical["requirements_pass"]["heel_only"])
        self.assertFalse(identical["requirements_pass"]["toe_only"])

    def test_two_sensor_gate_is_required_in_every_view(self) -> None:
        time_s = np.arange(80, dtype=float) / 1000.0
        heel = np.zeros(80, dtype=bool)
        toe = np.zeros(80, dtype=bool)
        for offset in (0, 40):
            heel[offset + 5 : offset + 20] = True
            toe[offset + 14 : offset + 30] = True
        views = [
            {"view_id": "slow", "interval_s": [0.0, 0.039]},
            {"view_id": "fast", "interval_s": [0.040, 0.079]},
        ]
        self.assertTrue(
            subject.two_sensor_view_gate(time_s, heel, toe, views)["pass"]
        )
        toe[40:] = False
        receipt = subject.two_sensor_view_gate(time_s, heel, toe, views)
        self.assertFalse(receipt["pass"])
        self.assertTrue(receipt["views"]["slow"]["pass"])
        self.assertFalse(receipt["views"]["fast"]["pass"])

    @staticmethod
    def fake_mesh_factory() -> object:
        x_min = -0.1109737753868103
        x_max = 0.17601655423641205
        triangles = np.asarray(
            [[[x_min, 0.0, -0.1], [x_max, 0.0, 0.1], [x_min, 0.1, 0.1]]]
        )
        return subject.MeshPointFactory(
            triangles,
            section_z_bounds=lambda _triangles, _x: (-0.1, 0.1),
            vertical_y_intersections=lambda _triangles, _x, _z: [0.0],
        )

    def test_default_grid_is_unique_and_contains_both_comparators(self) -> None:
        factory = self.fake_mesh_factory()
        candidates = subject.generate_coarse_candidates(
            factory,
            heel_x_fractions=subject._parse_csv_floats(
                subject.DEFAULT_HEEL_X_FRACTIONS, label="heel"
            ),
            toe_x_fractions=subject._parse_csv_floats(
                subject.DEFAULT_TOE_X_FRACTIONS, label="toe"
            ),
            heel_reaches_m=tuple(
                value / 1000.0
                for value in subject._parse_csv_floats(
                    subject.DEFAULT_HEEL_REACH_MM, label="heel reach"
                )
            ),
            toe_reaches_m=tuple(
                value / 1000.0
                for value in subject._parse_csv_floats(
                    subject.DEFAULT_TOE_REACH_MM, label="toe reach"
                )
            ),
        )
        self.assertEqual(len(candidates), 900)
        self.assertEqual(len({candidate.candidate_id for candidate in candidates}), 900)
        payloads = [candidate.payload() for candidate in candidates]
        self.assertEqual(
            sum(item["comparators"]["v19_common_25mm"] for item in payloads), 1
        )
        self.assertEqual(
            sum(
                item["comparators"]["v17_sphere_bottom_equivalent"]
                for item in payloads
            ),
            1,
        )

    def test_atomic_writes_are_strict_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as raw_directory:
            directory = Path(raw_directory)
            target = directory / "receipt.json"
            subject._write_json_exclusive(target, {"value": 1})
            self.assertEqual(json.loads(target.read_text()), {"value": 1})
            with self.assertRaises(FileExistsError):
                subject._write_json_exclusive(target, {"value": 2})
            self.assertEqual(json.loads(target.read_text()), {"value": 1})

            invalid = directory / "invalid.json"
            with self.assertRaises(ValueError):
                subject._write_json_exclusive(invalid, {"value": float("nan")})
            self.assertFalse(invalid.exists())
            self.assertEqual(list(directory.glob(".invalid.json.*.tmp")), [])

    def test_cli_rejects_invalid_grid_before_execution(self) -> None:
        parser = subject.build_parser()
        invalid_csv = parser.parse_args(["--heel-x-fractions", "nope"])
        with self.assertRaises(subject.V21SweepError):
            subject._validate_cli(invalid_csv)

        empty_fine = parser.parse_args(
            ["--fine-x-radius-mm", "0", "--fine-reach-radius-mm", "0"]
        )
        with self.assertRaises(subject.V21SweepError):
            subject._validate_cli(empty_fine)

    def test_trial_access_is_a_hardcoded_allowlist(self) -> None:
        parser_destinations = {action.dest for action in subject.build_parser()._actions}
        self.assertNotIn("trial", parser_destinations)
        self.assertNotIn("trials", parser_destinations)
        self.assertEqual(subject.TRIALS, ("02", "04"))
        self.assertIn("08", subject.FORBIDDEN_TRIALS)
        with self.assertRaises(subject.V21SweepError):
            subject._acquire_affine_trial(
                "08",
                v19=None,
                opensim=None,
                SimulatorConfig=None,
                KinematicsInterpolator=None,
                load_plugin=None,
                progress_options={},
            )

    def test_all_scientific_inputs_are_pinned(self) -> None:
        records = subject._verify_pinned_sources()
        self.assertIn(
            "validation/build_canonical_grf_event_oracle.py", records
        )
        self.assertNotIn("trial_08_canonical_event_ledger.json", "\n".join(records))


if __name__ == "__main__":
    unittest.main()
