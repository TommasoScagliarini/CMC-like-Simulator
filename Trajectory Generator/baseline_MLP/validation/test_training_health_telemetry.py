"""Unit tests for the observation-only training-health telemetry.

The telemetry counts, per training iteration, the FSM and morphology events that
the closed-loop qualification stages count per cell. It must satisfy three
properties, and these tests exist to hold it to them:

  1. it must never alter a run - no env key written, no reward touched, no
     exception escaping the sampling callback;
  2. it must count the SAME events the SAME way as the already-validated
     aggregation in v26c_j1_collect._summarise, which is checked here against
     committed traces and their committed receipts, not against a reimplementation;
  3. a real zero must be distinguishable from missing telemetry, at every layer:
     in the per-row rule, in what reaches the metrics logger, and in the JSONL row.

No RLlib training, no environment, no torch. Run:

    PYTHONDONTWRITEBYTECODE=1 python test_training_health_telemetry.py
"""

from __future__ import annotations

import copy
import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent
if str(BASELINE) not in sys.path:
    sys.path.insert(0, str(BASELINE))

sys.dont_write_bytecode = True

import tb_logging  # noqa: E402
import train_ppo_mlp  # noqa: E402

# The trainer resolves tb_logging lazily into a module global (train_ppo_mlp.py
# line 257). The tests do exactly what that loader does, so _training_health_metrics
# is exercised through its real dependency and not a stub.
train_ppo_mlp.tb_logging = tb_logging

V26C = HERE / "v26c_july_replica_2026-08-26"

CHECKS: list[tuple[str, bool, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(condition), detail))


def healthy_info(**overrides) -> dict:
    """A minimal info dict with every required telemetry key present and zero."""
    info = {
        "phase_fsm": {
            "timeout_exceeded": 0.0,
            "timeout_side": 0.0,
            "resync_event_this_step": 0.0,
            "resync_count": 0.0,
            "hs_cancelled_count": 0.0,
        },
        "reward_terms": {
            "phase_timeout_exceeded": 0.0,
            "phase_timeout_side": 0.0,
            "morphology_causal_failed_closed": 0.0,
        },
    }
    for section, values in overrides.items():
        info[section].update(values)
    return info


class FakeLogger:
    """Records log_value calls. Deliberately not RLlib's MetricsLogger."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, float, str]] = []

    def log_value(self, key, value, *, reduce=None, **kwargs):
        self.calls.append((str(key), float(value), str(reduce)))


class FakeWriter:
    """Records add_scalar calls, so the TensorBoard tag scheme can be asserted."""

    def __init__(self) -> None:
        self.tags: list[tuple[str, float, int]] = []

    def add_scalar(self, tag, value, step):
        self.tags.append((str(tag), float(value), int(step)))


# ---------------------------------------------------------------- the contract

EXPECTED_FIELDS = (
    "observed_rows",
    "missing_telemetry_rows",
    "phase_timeout_stance_rows",
    "morphology_causal_contract_failure_rows",
    "resync_event_rows",
    "resync_count_max",
    "hs_cancelled_count_max",
    "timeout_side_disagreement_rows",
)


def test_contract() -> None:
    check("T01 field tuple is exactly the eight declared fields, in order",
          tb_logging.TRAINING_HEALTH_FIELDS == EXPECTED_FIELDS,
          repr(tb_logging.TRAINING_HEALTH_FIELDS))
    check("T02 the TensorBoard section is training_health",
          tb_logging.TRAINING_HEALTH_PREFIX == "training_health")
    reduce_map = tb_logging._TRAINING_HEALTH_REDUCE
    check("T03 every field has a reduce mode",
          set(reduce_map) == set(EXPECTED_FIELDS))
    check("T04 the two cumulative counters reduce with max",
          reduce_map["resync_count_max"] == "max"
          and reduce_map["hs_cancelled_count_max"] == "max")
    check("T05 every row counter reduces with sum",
          all(reduce_map[f] == "sum" for f in EXPECTED_FIELDS
              if not f.endswith("_max")))
    # sum and max are both per-result reducers in this Ray version; lifetime_sum
    # is not, and using it would silently make the counters cumulative.
    check("T06 no field uses a lifetime reducer",
          all(mode in ("sum", "max") for mode in reduce_map.values()),
          str(sorted(set(reduce_map.values()))))
    check("T07 the stance side code is 1.0, as in prosthetic_phase_fsm",
          tb_logging._TIMEOUT_SIDE_STANCE == 1.0)
    check("T08 the required fsm keys are the five v3 telemetry keys",
          tb_logging._HEALTH_FSM_KEYS == ("timeout_exceeded", "timeout_side",
                                          "resync_event_this_step", "resync_count",
                                          "hs_cancelled_count"))
    check("T09 the required reward_terms keys match v26c_j1_collect's names",
          tb_logging._HEALTH_TERM_KEYS == ("phase_timeout_exceeded",
                                           "phase_timeout_side",
                                           "morphology_causal_failed_closed"))


# ------------------------------------------------- real zero vs missing telemetry

def test_real_zero_versus_missing() -> None:
    """The headline property. A zero and an absence must never look alike."""
    healthy = tb_logging.training_health_row(healthy_info())
    check("T10 a healthy row reports every field",
          set(healthy) == set(EXPECTED_FIELDS), str(sorted(healthy)))
    check("T11 a healthy row reports missing_telemetry_rows as a REAL zero",
          healthy["missing_telemetry_rows"] == 0.0)
    check("T12 a healthy row reports observed_rows 1.0",
          healthy["observed_rows"] == 1.0)
    check("T13 a healthy row reports every counter as a real zero",
          all(healthy[f] == 0.0 for f in EXPECTED_FIELDS if f != "observed_rows"))

    for label, info in (("no phase_fsm", {"reward_terms": healthy_info()["reward_terms"]}),
                        ("no reward_terms", {"phase_fsm": healthy_info()["phase_fsm"]}),
                        ("empty info", {}),
                        ("phase_fsm is not a mapping",
                         {"phase_fsm": [], "reward_terms": healthy_info()["reward_terms"]})):
        row = tb_logging.training_health_row(info)
        check(f"T14 [{label}] a missing row carries ONLY observed and missing",
              set(row) == {"observed_rows", "missing_telemetry_rows"}, str(sorted(row)))
        check(f"T15 [{label}] a missing row contributes NO zero to any counter",
              all(f not in row for f in EXPECTED_FIELDS
                  if f not in ("observed_rows", "missing_telemetry_rows")))
        check(f"T16 [{label}] missing_telemetry_rows is 1.0 and observed_rows is 0.0",
              row["missing_telemetry_rows"] == 1.0 and row["observed_rows"] == 0.0,
              str(row))

    # The two cases must be distinguishable by their key sets alone - which is
    # precisely what lets the trainer report None instead of 0.0 downstream.
    missing = tb_logging.training_health_row({})
    check("T17 healthy and missing rows have different key sets",
          set(healthy) != set(missing))


def test_observed_and_missing_partition_every_row() -> None:
    """observed_rows and missing_telemetry_rows PARTITION the rows.

    Exactly one of the two is 1.0 for any row, so per row they sum to 1 and over
    an aggregate they sum to the row count. Without this, "zero incidents over
    zero observed rows" would read as health rather than as silence.
    """
    import copy as _copy

    cases = [healthy_info()]
    stance = healthy_info(
        phase_fsm={"timeout_exceeded": 1.0, "timeout_side": 1.0},
        reward_terms={"phase_timeout_exceeded": 1.0, "phase_timeout_side": 1.0})
    cases.append(stance)
    cases.append(healthy_info(phase_fsm={"resync_event_this_step": 1.0}))
    cases.append({})
    cases.append({"phase_fsm": healthy_info()["phase_fsm"]})
    cases.append(None)
    broken = _copy.deepcopy(healthy_info())
    broken["phase_fsm"]["resync_count"] = float("nan")
    cases.append(broken)

    for index, info in enumerate(cases):
        row = tb_logging.training_health_row(info)
        total = row["observed_rows"] + row["missing_telemetry_rows"]
        check(f"T17a row {index}: observed + missing == 1 exactly", total == 1.0,
              f"observed={row['observed_rows']} missing={row['missing_telemetry_rows']}")
        check(f"T17b row {index}: each is exactly 0.0 or 1.0",
              row["observed_rows"] in (0.0, 1.0)
              and row["missing_telemetry_rows"] in (0.0, 1.0))

    totals = {"observed_rows": 0.0, "missing_telemetry_rows": 0.0}
    for info in cases:
        row = tb_logging.training_health_row(info)
        for field in totals:
            totals[field] += row[field]
    check("T17c aggregated, observed + missing == the number of rows",
          totals["observed_rows"] + totals["missing_telemetry_rows"] == float(len(cases)),
          str(totals))
    check("T17d the aggregate counts the four readable rows as observed",
          totals["observed_rows"] == 3.0, str(totals))
    check("T17e the aggregate counts the four unreadable rows as missing",
          totals["missing_telemetry_rows"] == 4.0, str(totals))

    # And the case the semantics exist for: an iteration where NOTHING could be
    # read must report zero observed rows, not a clean bill of health.
    blind = {"observed_rows": 0.0, "missing_telemetry_rows": 0.0}
    for _ in range(500):
        row = tb_logging.training_health_row({})
        for field in blind:
            blind[field] += row[field]
    check("T17f a fully blind iteration reports observed_rows 0, not health",
          blind["observed_rows"] == 0.0 and blind["missing_telemetry_rows"] == 500.0,
          str(blind))


def test_every_required_key_can_make_a_row_missing() -> None:
    """No required key may be silently defaulted. Each one, individually."""
    for section, keys in (("phase_fsm", tb_logging._HEALTH_FSM_KEYS),
                          ("reward_terms", tb_logging._HEALTH_TERM_KEYS)):
        for key in keys:
            for label, bad in (("absent", None), ("nan", float("nan")),
                               ("inf", float("inf")), ("text", "zero"),
                               ("none", None)):
                info = healthy_info()
                if label == "absent":
                    del info[section][key]
                else:
                    info[section][key] = bad
                row = tb_logging.training_health_row(info)
                check(f"T18 {section}.{key} [{label}] makes the row missing, not zero",
                      row.get("missing_telemetry_rows") == 1.0
                      and "phase_timeout_stance_rows" not in row)


# ------------------------------------------------------------- counting rules

def test_counting_rules() -> None:
    stance = healthy_info(
        phase_fsm={"timeout_exceeded": 1.0, "timeout_side": 1.0},
        reward_terms={"phase_timeout_exceeded": 1.0, "phase_timeout_side": 1.0})
    row = tb_logging.training_health_row(stance)
    check("T19 a stance timeout counts one stance row",
          row["phase_timeout_stance_rows"] == 1.0)
    check("T20 a stance timeout raises no disagreement",
          row["timeout_side_disagreement_rows"] == 0.0)

    swing = healthy_info(
        phase_fsm={"timeout_exceeded": 1.0, "timeout_side": 2.0},
        reward_terms={"phase_timeout_exceeded": 1.0, "phase_timeout_side": 2.0})
    row = tb_logging.training_health_row(swing)
    check("T21 a SWING timeout is not counted as stance",
          row["phase_timeout_stance_rows"] == 0.0)
    check("T22 a swing timeout raises no disagreement",
          row["timeout_side_disagreement_rows"] == 0.0)

    unknown = healthy_info(
        phase_fsm={"timeout_exceeded": 1.0, "timeout_side": 7.0},
        reward_terms={"phase_timeout_exceeded": 1.0, "phase_timeout_side": 7.0})
    row = tb_logging.training_health_row(unknown)
    check("T23 a timeout with an unknown side is recorded as a disagreement",
          row["timeout_side_disagreement_rows"] == 1.0
          and row["phase_timeout_stance_rows"] == 0.0)

    mismatch = healthy_info(
        phase_fsm={"timeout_exceeded": 1.0, "timeout_side": 2.0},
        reward_terms={"phase_timeout_exceeded": 1.0, "phase_timeout_side": 1.0})
    row = tb_logging.training_health_row(mismatch)
    check("T24 reward_terms disagreeing with phase_fsm is recorded",
          row["timeout_side_disagreement_rows"] == 1.0)

    morph = healthy_info(reward_terms={"morphology_causal_failed_closed": 1.0})
    check("T25 a morphology causal failure counts one row",
          tb_logging.training_health_row(morph)
          ["morphology_causal_contract_failure_rows"] == 1.0)

    resync = healthy_info(phase_fsm={"resync_event_this_step": 1.0,
                                     "resync_count": 3.0})
    row = tb_logging.training_health_row(resync)
    check("T26 a resync pulse counts one resync row",
          row["resync_event_rows"] == 1.0)
    check("T27 the cumulative resync counter passes through as its value",
          row["resync_count_max"] == 3.0)

    cancelled = healthy_info(phase_fsm={"hs_cancelled_count": 2.0})
    check("T28 the cumulative hs-cancel counter passes through as its value",
          tb_logging.training_health_row(cancelled)["hs_cancelled_count_max"] == 2.0)

    # A pulse without an event must stay zero: the rule is "> 0", not "is present".
    check("T29 a zero pulse is a zero row, not a one",
          tb_logging.training_health_row(healthy_info())["resync_event_rows"] == 0.0)


# ------------------------------------------------ equivalence with the validated rule

def load_trace(relative: str) -> list[dict]:
    path = V26C / relative
    if not path.is_file():
        return []
    return json.loads(path.read_text(encoding="utf-8"))


def aggregate(trace: list[dict]) -> dict[str, float]:
    """Reduce a trace exactly as one training iteration would: sum, then max."""
    totals = {f: 0.0 for f in EXPECTED_FIELDS}
    for step in trace:
        row = tb_logging.training_health_row(step)
        for field, value in row.items():
            if field.endswith("_max"):
                totals[field] = max(totals[field], value)
            else:
                totals[field] += value
    return totals


def test_matches_committed_evidence() -> None:
    """Against a committed trace AND its committed receipt, not a reimplementation.

    J12 cell F is the only trace in the chain with non-zero FSM events: one swing
    timeout and one cancelled heel strike. Its receipt records what the validated
    aggregation counted. The telemetry must agree with that record.
    """
    cases = (
        ("j12_runs/j12_closed_loop_v26c_2026-08-27_r1/j12_cell_F_trace.json",
         "j12_runs/j12_closed_loop_v26c_2026-08-27_r1/v26c_j12_closed_loop_receipt.json",
         "F", 354),
        ("j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/j19c_cell_G_trace.json",
         "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/"
         "v26c_j19c_heldout_g_i_receipt.json", "G", 500),
        ("j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/j19c_cell_H_trace.json",
         "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/"
         "v26c_j19c_heldout_g_i_receipt.json", "H", 500),
    )
    for trace_rel, receipt_rel, cell_id, steps in cases:
        trace = load_trace(trace_rel)
        if not trace:
            check(f"T30 [{cell_id}] the committed trace is readable", False, trace_rel)
            continue
        check(f"T30 [{cell_id}] the committed trace has {steps} rows",
              len(trace) == steps, str(len(trace)))
        totals = aggregate(trace)
        check(f"T31 [{cell_id}] every committed row carried its telemetry",
              totals["missing_telemetry_rows"] == 0.0
              and totals["observed_rows"] == float(steps),
              f"missing={totals['missing_telemetry_rows']} "
              f"observed={totals['observed_rows']}")
        check(f"T32 [{cell_id}] no committed row shows a side disagreement",
              totals["timeout_side_disagreement_rows"] == 0.0)

        receipt = json.loads((V26C / receipt_rel).read_text(encoding="utf-8"))
        recorded = None
        for cell in receipt.get("cells", []):
            if str(cell.get("id")) == cell_id:
                recorded = cell
                break
        if recorded is None:
            check(f"T33 [{cell_id}] the receipt records this cell", False)
            continue

        # Explicit paths, not a search. A search would happily land on
        # gate.checks.<name>, which is a PASS/FAIL boolean, not the measurement -
        # and comparing a count against a boolean is exactly the kind of vacuous
        # check this suite exists to avoid.
        summary = recorded.get("summary")
        telemetry = recorded.get("telemetry")
        check(f"T33 [{cell_id}] the receipt carries summary and telemetry blocks",
              isinstance(summary, dict) and isinstance(telemetry, dict))
        if not isinstance(summary, dict) or not isinstance(telemetry, dict):
            continue

        def committed(name: str):
            value = summary.get(name)
            mirrored = telemetry.get(name)
            if value is None or mirrored is None or value != mirrored:
                return None
            return value

        pairs = (
            ("phase_timeout_stance", "phase_timeout_stance_rows"),
            ("morphology_causal_contract_failure",
             "morphology_causal_contract_failure_rows"),
            ("resync_count", "resync_count_max"),
            ("hs_cancelled_count", "hs_cancelled_count_max"),
        )
        for receipt_name, field in pairs:
            expected = committed(receipt_name)
            check(f"T33 [{cell_id}] {field} reproduces the committed "
                  f"{receipt_name}",
                  expected is not None
                  and math.isclose(float(expected), totals[field], abs_tol=1e-9),
                  f"receipt={expected} telemetry={totals[field]}")

    # And the case that proves the check is not vacuous.
    trace = load_trace("j12_runs/j12_closed_loop_v26c_2026-08-27_r1/"
                       "j12_cell_F_trace.json")
    if trace:
        totals = aggregate(trace)
        check("T34 the J12-F cross-check is NOT vacuous: it carries a real event",
              totals["hs_cancelled_count_max"] == 1.0,
              str(totals["hs_cancelled_count_max"]))
        check("T35 J12-F's timeout was on the SWING side, so stance stays zero",
              totals["phase_timeout_stance_rows"] == 0.0)


# ------------------------------------------------------------ the logger layer

def test_logger_layer() -> None:
    logger = FakeLogger()
    tb_logging.RewardComponentsCallback._log_training_health(logger, healthy_info())
    keys = [call[0] for call in logger.calls]
    check("T36 a healthy row logs all eight fields",
          len(logger.calls) == 8 and len(set(keys)) == 8, str(len(logger.calls)))
    check("T37 every logged key sits under training_health/",
          all(key.startswith("training_health/") for key in keys))
    modes = {key.rsplit("/", 1)[1]: mode for key, _v, mode in logger.calls}
    check("T38 the logger uses max for the cumulative counters",
          modes["resync_count_max"] == "max"
          and modes["hs_cancelled_count_max"] == "max")
    check("T39 the logger uses sum for the row counters",
          all(modes[f] == "sum" for f in EXPECTED_FIELDS if not f.endswith("_max")))

    logger = FakeLogger()
    tb_logging.RewardComponentsCallback._log_training_health(logger, {})
    check("T40 a missing row logs exactly two values",
          len(logger.calls) == 2, str([c[0] for c in logger.calls]))
    check("T41 a missing row logs no counter at zero",
          not any("stance" in c[0] or "resync" in c[0] or "morphology" in c[0]
                  for c in logger.calls))

    # It must never raise: a telemetry bug may not stop sampling.
    for hostile in ({"phase_fsm": None, "reward_terms": None},
                    {"phase_fsm": {"timeout_exceeded": object()}},
                    {"reward_terms": 5}):
        try:
            tb_logging.RewardComponentsCallback._log_training_health(
                FakeLogger(), hostile)
            raised = False
        except Exception as error:  # noqa: BLE001 - that is what is being tested
            raised = repr(error)
        check("T42 hostile info never raises in the logger layer",
              raised is False, str(raised))


def test_callback_step_wiring() -> None:
    """on_episode_step must actually reach the health block."""

    class FakeEpisode:
        def __init__(self, info):
            self._info = info

        def get_infos(self, _index):
            return self._info

    logger = FakeLogger()
    info = healthy_info()
    info["reward_terms"]["tracking_loss"] = 0.5
    tb_logging.RewardComponentsCallback().on_episode_step(
        episode=FakeEpisode(info), metrics_logger=logger)
    health = [c for c in logger.calls if c[0].startswith("training_health/")]
    check("T43 on_episode_step emits the health block",
          len(health) == 8, str(len(health)))
    check("T44 on_episode_step still emits the pre-existing reward metrics",
          any(c[0].startswith("reward_loss/") for c in logger.calls))

    logger = FakeLogger()
    tb_logging.RewardComponentsCallback().on_episode_step(
        episode=FakeEpisode(None), metrics_logger=logger)
    check("T45 a non-dict info emits nothing at all, health included",
          logger.calls == [], str(logger.calls))


def test_tensorboard_tags() -> None:
    writer = FakeWriter()
    result = {
        "env_runners": {
            "training_health": {"phase_timeout_stance_rows": 0.0,
                                "observed_rows": 4096.0},
            "some_other_metric": 1.0,
        },
    }
    written = tb_logging.log_result_scalars(writer, result, 4096)
    tags = dict((tag, value) for tag, value, _s in writer.tags)
    check("T46 three scalars are written", written == 3, str(written))
    check("T47 training_health is promoted to a top-level TensorBoard section",
          "training_health/phase_timeout_stance_rows" in tags
          and "training_health/observed_rows" in tags, str(sorted(tags)))
    check("T48 an unrelated metric keeps its env_runners prefix",
          "env_runners/some_other_metric" in tags, str(sorted(tags)))
    check("T49 the step axis is passed through",
          all(step == 4096 for _t, _v, step in writer.tags))


# --------------------------------------------------------------- the JSONL row

def test_jsonl_row() -> None:
    env_metrics = {
        "env_runners/training_health/observed_rows": 4096.0,
        "env_runners/training_health/missing_telemetry_rows": 0.0,
        "env_runners/training_health/phase_timeout_stance_rows": 0.0,
        "env_runners/training_health/resync_count_max": 1.0,
        "env_runners/episode_end/episode_time_limit": 8.0,
    }
    row = train_ppo_mlp._training_health_metrics(env_metrics)
    check("T50 the row publishes exactly the eight declared fields",
          tuple(row) == EXPECTED_FIELDS, str(tuple(row)))
    check("T51 a field that arrived is a float",
          row["observed_rows"] == 4096.0 and isinstance(row["observed_rows"], float))
    check("T52 a REAL ZERO stays 0.0 in the row",
          row["missing_telemetry_rows"] == 0.0)
    check("T53 a field that never arrived is None, NOT 0.0",
          row["hs_cancelled_count_max"] is None
          and row["morphology_causal_contract_failure_rows"] is None)
    check("T54 the two are distinguishable in the row",
          row["missing_telemetry_rows"] is not None
          and row["hs_cancelled_count_max"] is None)
    check("T55 unrelated env metrics do not leak into the section",
          "episode_time_limit" not in row)

    empty = train_ppo_mlp._training_health_metrics({})
    check("T56 with no telemetry at all every field is None",
          all(value is None for value in empty.values()))
    check("T57 the row is JSON-serialisable with no custom encoder",
          json.loads(json.dumps({"training_health": row}))["training_health"]
          ["hs_cancelled_count_max"] is None)


def test_containment() -> None:
    """The telemetry must be unable to stop what it observes.

    Every failure surface it has - extraction, the metrics logger, the
    TensorBoard writer, the JSONL projection - is exercised with something that
    throws, and none of them may propagate. The negative case matters as much:
    a failure on a NON-health TensorBoard tag must still propagate, because
    silently swallowing pre-existing metric failures would be a behaviour change
    nobody asked for.
    """
    import contextlib
    import io as _io

    class ExplodingLogger:
        """A logger that fails on every training_health value."""

        def __init__(self, only_health=True):
            self.calls = []
            self.only_health = only_health

        def log_value(self, key, value, *, reduce=None, **kwargs):
            if not self.only_health or str(key).startswith("training_health/"):
                raise RuntimeError("hostile logger")
            self.calls.append((str(key), float(value), str(reduce)))

    class PartialLogger:
        """Fails on exactly one field, accepts the rest."""

        def __init__(self, bad):
            self.calls = []
            self.bad = bad

        def log_value(self, key, value, *, reduce=None, **kwargs):
            if str(key).endswith("/" + self.bad):
                raise ValueError("hostile field")
            self.calls.append((str(key), float(value), str(reduce)))

    class ExplodingWriter:
        def __init__(self, only_health=True):
            self.tags = []
            self.only_health = only_health

        def add_scalar(self, tag, value, step):
            if not self.only_health or str(tag).startswith("training_health/"):
                raise OSError("hostile writer")
            self.tags.append((str(tag), float(value), int(step)))

    tb_logging._HEALTH_WARNED.clear()
    logger = ExplodingLogger()
    try:
        tb_logging.RewardComponentsCallback._log_training_health(
            logger, healthy_info())
        raised = False
    except Exception as error:  # noqa: BLE001 - that is what is being tested
        raised = repr(error)
    check("T61 a logger that throws on every field does not propagate",
          raised is False, str(raised))

    tb_logging._HEALTH_WARNED.clear()
    logger = PartialLogger("resync_count_max")
    tb_logging.RewardComponentsCallback._log_training_health(logger, healthy_info())
    check("T62 one hostile field does not suppress the other seven",
          len(logger.calls) == 7, str(len(logger.calls)))

    tb_logging._HEALTH_WARNED.clear()
    original = tb_logging.training_health_row
    try:
        def exploding_row(info):
            raise TypeError("hostile extraction")

        tb_logging.training_health_row = exploding_row
        try:
            tb_logging.RewardComponentsCallback._log_training_health(
                FakeLogger(), healthy_info())
            raised = False
        except Exception as error:  # noqa: BLE001
            raised = repr(error)
    finally:
        tb_logging.training_health_row = original
    check("T63 a throwing extraction does not propagate", raised is False, str(raised))

    # on_episode_step must survive a logger hostile to the health block only.
    class FakeEpisode:
        def __init__(self, info):
            self._info = info

        def get_infos(self, _index):
            return self._info

    tb_logging._HEALTH_WARNED.clear()
    logger = ExplodingLogger(only_health=True)
    info = healthy_info()
    info["reward_terms"]["tracking_loss"] = 0.5
    try:
        tb_logging.RewardComponentsCallback().on_episode_step(
            episode=FakeEpisode(info), metrics_logger=logger)
        raised = False
    except Exception as error:  # noqa: BLE001
        raised = repr(error)
    check("T64 on_episode_step survives a health-hostile logger",
          raised is False, str(raised))
    check("T65 and it still logged the pre-existing reward metrics",
          any(call[0].startswith("reward_loss/") for call in logger.calls))

    # TensorBoard: health tags are guarded, everything else is untouched.
    tb_logging._HEALTH_WARNED.clear()
    writer = ExplodingWriter(only_health=True)
    result = {"env_runners": {"training_health": {"observed_rows": 4096.0},
                              "some_other_metric": 1.0}}
    try:
        written = tb_logging.log_result_scalars(writer, result, 4096)
        raised = False
    except Exception as error:  # noqa: BLE001
        written, raised = None, repr(error)
    check("T66 a writer that throws on a health tag does not propagate",
          raised is False, str(raised))
    check("T67 the non-health scalar is still written and counted",
          written == 1 and writer.tags == [("env_runners/some_other_metric", 1.0, 4096)],
          str((written, writer.tags)))

    tb_logging._HEALTH_WARNED.clear()
    writer = ExplodingWriter(only_health=False)
    try:
        tb_logging.log_result_scalars(writer, {"env_runners": {"other": 1.0}}, 1)
        propagated = False
    except OSError:
        propagated = True
    check("T68 a failure on a NON-health tag still propagates, as before",
          propagated,
          "the guard must not silently change pre-existing metric behaviour")

    # The JSONL projection.
    class HostileMetrics(dict):
        def items(self):
            raise RuntimeError("hostile metrics")

    tb_logging._HEALTH_WARNED.clear()
    try:
        row = train_ppo_mlp._training_health_metrics(HostileMetrics())
        raised = False
    except Exception as error:  # noqa: BLE001
        row, raised = None, repr(error)
    check("T69 a hostile env_metrics does not propagate into the row",
          raised is False, str(raised))
    check("T70 and the row degrades to all-None, never to zeros",
          row is not None and all(value is None for value in row.values())
          and tuple(row) == EXPECTED_FIELDS, str(row))

    # The diagnostic is emitted once per category, and never recurses.
    tb_logging._HEALTH_WARNED.clear()
    buffer = _io.StringIO()
    with contextlib.redirect_stderr(buffer):
        for _ in range(50):
            tb_logging.RewardComponentsCallback._log_training_health(
                ExplodingLogger(), healthy_info())
    text = buffer.getvalue()
    check("T71 the diagnostic names training_health and is emitted",
          "[training_health]" in text, text[:120])
    check("T72 it is emitted once per category, not once per step",
          text.count("[training_health]") == len(EXPECTED_FIELDS),
          "%d messages for %d fields" % (text.count("[training_health]"),
                                         len(EXPECTED_FIELDS)))
    check("T73 the warn path itself never raises on a hostile error object",
          tb_logging._health_warn_once("probe", RuntimeError("x")) is None)
    tb_logging._HEALTH_WARNED.clear()


def test_observation_only() -> None:
    """The rule must not mutate what it reads."""
    info = healthy_info()
    before = copy.deepcopy(info)
    tb_logging.training_health_row(info)
    check("T58 training_health_row does not mutate the info dict", info == before)
    tb_logging.RewardComponentsCallback._log_training_health(FakeLogger(), info)
    check("T59 the logger layer does not mutate the info dict either",
          info == before)

    source = pathlib.Path(tb_logging.__file__).read_text(encoding="utf-8")
    start = source.index("def training_health_row")
    end = source.index("_TRAINING_HEALTH_REDUCE = {")
    body = source[start:end]
    check("T60 the rule never assigns into info or reward_terms",
          'info[' not in body and 'terms[' not in body and 'fsm[' not in body,
          "the counting rule must read, never write")


def main() -> int:
    test_contract()
    test_real_zero_versus_missing()
    test_observed_and_missing_partition_every_row()
    test_every_required_key_can_make_a_row_missing()
    test_counting_rules()
    test_matches_committed_evidence()
    test_logger_layer()
    test_callback_step_wiring()
    test_tensorboard_tags()
    test_jsonl_row()
    test_containment()
    test_observation_only()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
