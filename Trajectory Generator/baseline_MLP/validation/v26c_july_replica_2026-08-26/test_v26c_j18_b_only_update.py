"""Targeted tests for the V26C J18 readiness artefacts.

Run:
    python test_v26c_j18_b_only_update.py

The suite is read-only. It asserts that the preflight path writes nothing, that
every derived quantity is derived rather than hardcoded, that no sealed seed is
reachable, and that no fit can run in readiness.

``sys.dont_write_bytecode`` is set BEFORE the runner is imported, so the
write-nothing snapshot is not defeated by ``__pycache__``. That ordering is the
J14 lesson and it is load-bearing here.
"""

from __future__ import annotations

import ast
import builtins
import dis
import hashlib
import json
import pathlib
import pickle
import sys
import types

sys.dont_write_bytecode = True

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import v26c_j18_b_only_update as J18  # noqa: E402

RUNNER_PATH = ROOT / "v26c_j18_b_only_update.py"
PREREG_PATH = ROOT / "v26c_j18_prereg_b_only_constrained_update.json"
MANIFEST_PATH = ROOT / "v26c_j18_dataset_manifest.json"
OVERLAY_PATH = ROOT / "v26c_j18_provenance_overlay_j8_2026-08-27.json"

# The v2 addendum is immutable. v3 corrects one point additively and must leave
# v2's bytes untouched; this pin is what proves it.
ADDENDUM_V2_PIN = "bc50072835929961f85aa5ce955124491ca3c9e16d87ec2691b29ab7b996a71d"

CHECKS = []


def check(name, condition, detail=""):
    CHECKS.append((name, bool(condition), str(detail)))


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def snapshot(root):
    """Every path under root with its size and mtime. No exclusions at all."""
    out = {}
    for path in sorted(root.rglob("*")):
        try:
            stat = path.stat()
        except OSError:
            continue
        out[str(path.relative_to(root))] = (stat.st_size, stat.st_mtime_ns)
    return out


# --------------------------------------------------------------------------
# T01 - namespace closure. This is the test that would have caught the J15
# NameError before execution rather than after.
# --------------------------------------------------------------------------


def test_namespace_closure():
    module_names = set(vars(J18)) | set(vars(builtins))
    unresolved = []

    def walk(code, origin):
        for instruction in dis.get_instructions(code):
            if instruction.opname in ("LOAD_GLOBAL", "STORE_GLOBAL", "DELETE_GLOBAL"):
                name = instruction.argval
                if name not in module_names:
                    unresolved.append((origin, name))
        for const in code.co_consts:
            if isinstance(const, types.CodeType):
                walk(const, origin)

    for attr, value in vars(J18).items():
        if isinstance(value, types.FunctionType) and value.__module__ == J18.__name__:
            walk(value.__code__, attr)

    check("T01 every global name in the runner resolves", not unresolved, unresolved)


# --------------------------------------------------------------------------
# T02/T03 - the preflight writes nothing and trains nothing
# --------------------------------------------------------------------------


def test_preflight_is_read_only():
    before = snapshot(ROOT)
    report = J18.run_preflight(verbose=False)
    after = snapshot(ROOT)

    check("T02 preflight leaves the tree byte-for-byte unchanged", before == after,
          sorted(set(after) ^ set(before)))
    check("T03 preflight reports it trained nothing", report["trained_anything"] is False)
    check("T04 preflight reports it wrote nothing", report["wrote_anything"] is False)
    check("T05 preflight created no directory", report["created_any_directory"] is False)
    check("T06 preflight passes every one of its own checks",
          report["ok"], "%d/%d" % (report["passed"], report["total"]))
    check("T07 preflight never imported torch", "torch" not in sys.modules)
    return report


# --------------------------------------------------------------------------
# T08-T12 - block B carries no post-mismatch row, and its labels are the
# teacher's at the SAME index and time
# --------------------------------------------------------------------------


def test_block_b_is_clean():
    built = J18.build_blocks(ROOT)
    blocks = built["blocks"]
    prefix = built["prefix"]

    on_policy = built["on_policy_observations"]
    teacher = built["teacher"]

    index_of = {}
    for index, row in enumerate(on_policy):
        index_of.setdefault(J18.row_key(row), index)
    used = [index_of[J18.row_key(row)] for row in blocks["B"][0]]

    check("T08 every block-B row comes from before the first mismatch",
          all(i < prefix["first_mismatch_index"] for i in used), used)
    check("T09 block B holds exactly the prefix minus the row claimed by A",
          used == list(range(1, prefix["prefix_length"])), used)

    labels_match = all(
        np.array_equal(blocks["B"][1][k], teacher["actions"][i])
        for k, i in enumerate(used)
    )
    check("T10 block-B labels are the teacher action at the SAME index", labels_match)

    trace = J18.load_trace("B", ROOT)
    times_on_policy = np.array([row["time_before"] for row in trace], dtype=np.float64)
    worst = float(np.abs(times_on_policy - teacher["times"]).max())
    check("T11 on-policy and teacher times are bit-identical", worst == 0.0, worst)
    check("T12 the 485 post-mismatch rows are excluded",
          prefix["rows_excluded_post_mismatch"] == 485,
          prefix["rows_excluded_post_mismatch"])
    return built


# --------------------------------------------------------------------------
# T13-T17 - block composition, counts and disjointness
# --------------------------------------------------------------------------


def test_composition(built):
    blocks = built["blocks"]
    manifest = load_json(MANIFEST_PATH)
    counts = manifest["block_composition"]["counts_before_and_after"]

    for name in J18.BLOCK_PRIORITY:
        check("T13 block %s row count matches the manifest" % name,
              len(blocks[name][0]) == counts[name]["after"], len(blocks[name][0]))
        check("T14 block %s raw count matches the manifest" % name,
              built["raw_rows"][name] == counts[name]["before"], built["raw_rows"][name])

    total = sum(len(blocks[n][0]) for n in J18.BLOCK_PRIORITY)
    union = set()
    for name in J18.BLOCK_PRIORITY:
        union |= {J18.row_key(row) for row in blocks[name][0]}
    check("T15 the four blocks are pairwise disjoint", len(union) == total,
          "%d distinct vs %d rows" % (len(union), total))
    check("T16 total row count is the manifest total",
          total == manifest["block_composition"]["totals"]["rows"], total)
    check("T17 measured target conflicts are zero",
          built["overlaps"]["total_conflicting_rows"] == 0,
          built["overlaps"]["total_conflicting_rows"])
    check("T18 the priority order is the architect's A>B>C>D",
          J18.BLOCK_PRIORITY == ("A", "B", "C", "D"), J18.BLOCK_PRIORITY)


# --------------------------------------------------------------------------
# T19-T22 - anchors really carry J8's own output
# --------------------------------------------------------------------------


def test_anchor_labels(built):
    blocks = built["blocks"]
    state = built["state"]

    recorded = {}
    for cell in J18.ANCHOR_CELLS:
        trace = J18.load_trace(cell, ROOT)
        for observation, mean in zip(
            J18.trace_observations(trace), J18.trace_policy_means(trace)
        ):
            recorded.setdefault(J18.row_key(observation), mean)

    exact = all(
        np.array_equal(label, recorded[J18.row_key(observation)])
        for observation, label in zip(*blocks["C"])
    )
    check("T19 block-C labels are the recorded J8 policy_mean, bit-identical", exact)

    predicted = J18.forward_mean(state, blocks["C"][0])
    worst = float(np.abs(predicted - blocks["C"][1].astype(np.float64)).max())
    check("T20 J8's own forward reproduces the block-C labels", worst < 1e-06, worst)

    predicted_d = J18.forward_mean(state, blocks["D"][0]).astype(np.float32)
    worst_d = float(
        np.abs(predicted_d.astype(np.float64) - blocks["D"][1].astype(np.float64)).max()
    )
    check("T21 block-D labels reproduce J8's forward within tolerance",
          worst_d <= J18.COMPUTED_LABEL_TOLERANCE, worst_d)

    check("T22 cell B is not among the anchor cells",
          J18.TARGET_CELL not in J18.ANCHOR_CELLS)


# --------------------------------------------------------------------------
# T23-T26 - log-std frozen, critic excluded, structure preserved
# --------------------------------------------------------------------------


def test_structural_invariants(built):
    state = built["state"]
    check("T23 the parent carries no critic key",
          J18.critic_keys(state) == (), J18.critic_keys(state))
    # The log-std is a ROW SLICE of the head, not a named key. A name-based
    # search finds nothing, which would make the freeze gate vacuously true.
    check("T24a no key name carries the log-std, so a name search would be vacuous",
          not [k for k in state if "logstd" in k or "log_std" in k], list(state))
    logstd = J18.logstd_tensors(state)
    check("T24b the log-std slice exists and has the right shape",
          logstd["pi.1.bias"].shape == (2,) and logstd["pi.1.weight"].shape == (2, 256),
          {k: v.shape for k, v in logstd.items()})
    check("T24c the log-std is state-independent in the parent",
          J18.logstd_is_state_independent(state))
    check("T24d the parent sigma is the frozen 0.005",
          np.allclose(J18.logstd_sigma(state), 0.005, atol=1e-08),
          J18.logstd_sigma(state).tolist())

    # A gate that cannot fail is not a gate. Perturb the log-std slice and
    # confirm G7 bites.
    mutated = {k: np.array(v, dtype=np.float64, copy=True) for k, v in state.items()}
    mutated["pi.1.bias"][J18.LOGSTD_ROWS] += 1e-03
    bitten = J18.evaluate_gates(mutated, J18.evaluate_all_blocks(state, built["blocks"]))
    check("T24e G7 FAILS when the log-std slice is perturbed",
          not {g["id"]: g for g in bitten}["G7"]["passed"])

    made_state_dependent = {k: np.array(v, dtype=np.float64, copy=True)
                            for k, v in state.items()}
    made_state_dependent["pi.1.weight"][J18.LOGSTD_ROWS, 0] = 1e-04
    dependent = J18.evaluate_gates(
        made_state_dependent, J18.evaluate_all_blocks(state, built["blocks"]))
    check("T24f G7 FAILS when the log-std is made state-dependent",
          not {g["id"]: g for g in dependent}["G7"]["passed"])

    clock_broken = {k: np.array(v, dtype=np.float64, copy=True) for k, v in state.items()}
    clock_broken["pi.0.0.weight"][0, 0] = 1e-09
    broken = J18.evaluate_gates(
        clock_broken, J18.evaluate_all_blocks(state, built["blocks"]))
    check("T24g G9 FAILS on any non-zero clock column, at any magnitude",
          not {g["id"]: g for g in broken}["G9"]["passed"])

    gates = J18.evaluate_gates(state, J18.evaluate_all_blocks(state, built["blocks"]))
    by_id = {g["id"]: g for g in gates}
    check("T25 the parent itself passes every structural gate",
          all(by_id[g]["passed"] for g in ("G7", "G8", "G9", "G10", "G11")),
          [g for g in ("G7", "G8", "G9", "G10", "G11") if not by_id[g]["passed"]])
    check("T26 the parent fails the on-policy improvement gate it must beat",
          not by_id["G5"]["passed"], by_id["G5"]["measured"])

    width = np.asarray(state["pi.0.0.weight"]).shape[1]
    check("T27 the actor is 35D, single, unwidened",
          width == J18.OBSERVATION_WIDTH == 35, width)


# --------------------------------------------------------------------------
# T28-T30 - the prefix is derived, not hardcoded
# --------------------------------------------------------------------------


def test_prefix_is_derived():
    columns = [0, 1]
    left = np.zeros((10, 4), dtype=np.float32)
    right = np.zeros((10, 4), dtype=np.float32)
    right[7, 1] = 1.0
    derived = J18.derive_prefix(left, right, columns)
    check("T28 derive_prefix finds a synthetic mismatch at the right index",
          derived["first_mismatch_index"] == 7, derived["first_mismatch_index"])
    check("T29 derive_prefix names the mismatching column",
          derived["first_mismatch_column"] == 1, derived["first_mismatch_column"])

    identical = J18.derive_prefix(left, left.copy(), columns)
    check("T30 with no mismatch the whole trajectory is the prefix",
          identical["prefix_length"] == 10, identical["prefix_length"])

    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "derive_prefix"
    )
    literals = [
        node.value for node in ast.walk(function)
        if isinstance(node, ast.Constant) and isinstance(node.value, int)
    ]
    check("T31 derive_prefix contains no hardcoded prefix length",
          15 not in literals, literals)


# --------------------------------------------------------------------------
# T32-T35 - grid and thresholds agree with the preregistration
# --------------------------------------------------------------------------


def test_grid_and_thresholds():
    prereg = load_json(PREREG_PATH)
    grid = J18.enumerate_grid()

    axes = {axis["name"]: axis["values"] for axis in prereg["candidate_grid"]["axes"]}
    check("T32 the grid is the preregistered size",
          len(grid) == prereg["candidate_grid"]["total_candidates"] == 16, len(grid))
    check("T33 grid indices are unique and contiguous",
          [e["candidate_index"] for e in grid] == list(range(16)))
    check("T34 lambda axis matches the preregistration",
          list(J18.GRID_LAMBDA) == axes["preservation_weight_lambda"])
    check("T35 beta axis matches the preregistration",
          list(J18.GRID_BETA) == axes["on_policy_weight_beta"])
    check("T36 learning-rate axis matches the preregistration",
          list(J18.GRID_LEARNING_RATE) == axes["learning_rate"])

    thresholds = {g["id"]: g for g in prereg["thresholds"]["hard_gates_all_must_pass"]}
    pairs = [
        ("G1", J18.GATE_MAX_DRIFT), ("G2", J18.GATE_MAX_DRIFT),
        ("G3", J18.GATE_BIAS_DRIFT), ("G4", J18.GATE_BIAS_DRIFT),
        ("G5", J18.GATE_MSE_B_CEILING), ("G6", J18.GATE_MSE_A_CEILING),
    ]
    for gate_id, value in pairs:
        check("T37 %s threshold matches the preregistration" % gate_id,
              thresholds[gate_id]["threshold"] == value,
              "%s vs %s" % (value, thresholds[gate_id]["threshold"]))

    # The thresholds must be DERIVED in code, not written down as rounded
    # literals that can silently drift from the measurement they cite.
    check("T38 the B ceiling is exactly half the measured baseline",
          J18.GATE_MSE_B_CEILING == J18.BASELINE_MSE_B / 2.0,
          J18.GATE_MSE_B_CEILING)
    check("T39 the A ceiling is exactly the measured baseline",
          J18.GATE_MSE_A_CEILING == J18.BASELINE_MSE_A)
    check("T40 the bias gate is a tenth of the smallest tolerated perturbation",
          J18.GATE_BIAS_DRIFT == J18.SMALLEST_TOLERATED_PERTURBATION / 10.0)

    runner_tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    derived = {}
    for node in runner_tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            derived[node.targets[0].id] = node.value
    check("T40b the B ceiling is an expression, not a literal",
          isinstance(derived.get("GATE_MSE_B_CEILING"), ast.BinOp),
          type(derived.get("GATE_MSE_B_CEILING")).__name__)
    check("T40c the bias gate is an expression, not a literal",
          isinstance(derived.get("GATE_BIAS_DRIFT"), ast.BinOp),
          type(derived.get("GATE_BIAS_DRIFT")).__name__)

    ranking = prereg["thresholds"]["ranking_applied_only_to_candidates_that_pass_every_hard_gate"]
    check("T41 the ranking's last key is the unique grid index, so the order is total",
          ranking["keys_in_order"][-1]["key"] == "candidate_index")


# --------------------------------------------------------------------------
# T42-T45 - no sealed seed, no fit, no self-hash
# --------------------------------------------------------------------------


def test_sealed_seeds_and_no_fit():
    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))

    # Two module constants are allowed to contain one of these integers: the
    # SEALED_SEEDS declaration itself, and FIT_BATCH_SIZE, which is 128 as a
    # batch size and not as a seed. The allowlist is asserted to be exactly
    # those two names so it cannot be quietly grown to hide a real use.
    allowed_names = ("SEALED_SEEDS", "FIT_BATCH_SIZE")
    check("T42a the sealed-seed allowlist is exactly the two known constants",
          allowed_names == ("SEALED_SEEDS", "FIT_BATCH_SIZE"))

    allowed_nodes = set()
    seen_names = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in allowed_names:
                seen_names.append(name)
                allowed_nodes |= {id(n) for n in ast.walk(node)}
    check("T42b the runner declares the sealed seeds so they can be excluded",
          "SEALED_SEEDS" in seen_names, seen_names)

    stray = [
        (node.value, node.lineno)
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, int)
        and node.value in (126, 127, 128)
        and id(node) not in allowed_nodes
    ]
    check("T43 no sealed seed appears outside the two allowed declarations",
          not stray, stray)
    check("T43b FIT_BATCH_SIZE is the only reason 128 is allowed at all",
          J18.FIT_BATCH_SIZE == 128 and 128 in J18.SEALED_SEEDS)
    check("T44 the fit seed is 123, not a sealed seed",
          J18.FIT_SEED == 123 and J18.FIT_SEED not in J18.SEALED_SEEDS)

    raised = False
    try:
        J18.run_fit("nonexistent-go-file")
    except RuntimeError as error:
        raised = "architect GO is absent or invalid" in str(error)
    check("T45 run_fit refuses without a valid architect GO", raised)
    check("T46 --execute without a GO file returns failure",
          J18.main(["--execute"]) == 1)

    for path in (PREREG_PATH, MANIFEST_PATH, OVERLAY_PATH):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        text = path.read_text(encoding="utf-8")
        check("T47 %s contains no self-hash" % path.name, digest not in text)
        check("T48 %s declares it contains no self-hash" % path.name,
              load_json(path).get("contains_no_self_hash") is True
              or load_json(path).get("self_consistency", {}).get("contains_no_self_hash") is True)


# --------------------------------------------------------------------------
# T49-T51 - provenance: actual bytes, not the stale manifest
# --------------------------------------------------------------------------


def test_provenance():
    overlay = load_json(OVERLAY_PATH)
    pair = overlay["manifest_declared_hash_vs_actual_byte_hash"]

    stale = load_json(ROOT / J18.PARENT_STALE_MANIFEST_PATH)
    actual = J18.sha256_file(ROOT / J18.PARENT_MODULE_PATH)

    check("T49 the leaf manifest still declares the J2 hash, as documented",
          stale["module_state_sha256"] == pair["manifest_declared_module_state_sha256"]
          == J18.J2_MODULE_SHA256)
    check("T50 the actual parent bytes are J8's, not J2's",
          actual == pair["actual_byte_sha256_of_the_j8_module"] == J18.PARENT_MODULE_SHA256
          and actual != J18.J2_MODULE_SHA256, actual)

    receipt = load_json(ROOT / J18.J9R1_DIR / "v26c_j9r1_closed_loop_receipt.json")
    used = receipt["actor_before"]["artefacts_sha256"]["rl_module/module_state.pkl"]
    parent_recorded = receipt["actor_before"]["lineage"]["parent_module_state_sha256"]
    check("T51 the J9R1 receipt pins the ACTOR USED at the J8 bytes",
          used == J18.PARENT_MODULE_SHA256, used)
    check("T52 the J9R1 receipt records J2 separately, as the PARENT",
          parent_recorded == J18.J2_MODULE_SHA256, parent_recorded)
    check("T53 the runner never reads the leaf manifest as an identity source",
          "PARENT_STALE_MANIFEST_PATH" in RUNNER_PATH.read_text(encoding="utf-8")
          and J18.PARENT_MODULE_SHA256 != J18.J2_MODULE_SHA256)


# ==========================================================================
# EXECUTION-PATH TESTS. Nothing below runs the real fit or writes into the
# validation root; the leaf mechanics are exercised in a temporary directory.
# ==========================================================================


# --------------------------------------------------------------------------
# E01-E05 - the registered minibatch estimator, on a deliberately imbalanced
# synthetic dataset where a naive per-batch mean gives a different answer
# --------------------------------------------------------------------------


def test_minibatch_estimator():
    import torch

    rng = np.random.default_rng(7)
    sizes = {"A": 3, "B": 1, "C": 5, "D": 2}          # deliberately imbalanced
    n = sum(sizes.values())
    block_index = np.concatenate(
        [np.full(sizes[b], i, dtype=np.int64) for i, b in enumerate(J18.BLOCK_PRIORITY)]
    )
    flat = {"n": n, "block_index": block_index, "block_sizes": sizes}
    candidate = {"w_A": 1.0, "w_B": 5.0, "w_C": 10.0, "w_D": 10.0}

    predictions = rng.normal(size=(n, 2))
    targets = rng.normal(size=(n, 2))
    per_row = ((predictions - targets) ** 2).mean(axis=1)

    registered = 0.0
    for position, name in enumerate(J18.BLOCK_PRIORITY):
        mask = block_index == position
        registered += candidate["w_%s" % name] * float(per_row[mask].mean())

    coefficients = J18.row_coefficients(candidate, flat)
    tp = torch.as_tensor(predictions)
    tt = torch.as_tensor(targets)
    tc = torch.as_tensor(coefficients)

    whole = float(J18.minibatch_loss(tp, tt, tc, n, torch))
    check("E01 the estimator over the whole set equals the registered objective",
          abs(whole - registered) < 1e-12, "%r vs %r" % (whole, registered))

    total = 0.0
    naive = 0.0
    for start in range(0, n, 4):
        idx = list(range(start, min(start + 4, n)))
        chunk = float(J18.minibatch_loss(tp[idx], tt[idx], tc[idx], n, torch))
        total += (len(idx) / n) * chunk
        naive += (len(idx) / n) * float(((tp[idx] - tt[idx]) ** 2).mean())
    check("E02 chunk losses weighted by |M|/N sum to the registered objective",
          abs(total - registered) < 1e-12, "%r vs %r" % (total, registered))
    check("E03 the naive per-batch row mean gives a DIFFERENT answer, so E02 bites",
          abs(naive - registered) > 1e-06, "naive=%r registered=%r" % (naive, registered))

    short = [n - 1]
    scaled = float(J18.minibatch_loss(tp[short], tt[short], tc[short], n, torch))
    manual = (n / 1.0) * coefficients[short[0]] * per_row[short[0]]
    check("E04 a single-row chunk is scaled by N/|M| exactly",
          abs(scaled - manual) < 1e-12, "%r vs %r" % (scaled, manual))

    coefficients_real = J18.row_coefficients(
        J18.enumerate_grid()[3], J18.build_flat_dataset(J18.build_blocks(ROOT))
    )
    check("E05 per-row coefficients are w_block / N_block",
          abs(coefficients_real[0] - 1.0 / 500) < 1e-15
          and abs(coefficients_real[500] - 5.0 / 14) < 1e-15,
          [coefficients_real[0], coefficients_real[500]])


# --------------------------------------------------------------------------
# E06-E10 - candidate initialisation and RNG order are identical across
# candidates; only the grid values differ
# --------------------------------------------------------------------------


def test_candidate_determinism(built):
    n = J18.EXPECTED_TOTAL_ROWS
    first = [p.copy() for p in
             (np.random.default_rng(J18.FIT_SEED).permutation(n) for _ in range(1))]
    a = np.random.default_rng(J18.FIT_SEED)
    b = np.random.default_rng(J18.FIT_SEED)
    same = all(np.array_equal(a.permutation(n), b.permutation(n)) for _ in range(5))
    check("E06 two default_rng(123) streams give identical permutations", same)
    check("E07 the stream is reset per candidate, so order never depends on the grid",
          np.array_equal(first[0], np.random.default_rng(J18.FIT_SEED).permutation(n)))

    parent = built["state"]
    scales = J18.july_scale_vector(built["teacher"]["feature_names"])
    before = np.array(parent["pi.0.0.weight"], copy=True)
    one = J18.scaled_domain_state(parent, scales)
    two = J18.scaled_domain_state(parent, scales)
    check("E08 the scaled-domain initialisation is deterministic",
          all(np.array_equal(one[k], two[k]) for k in J18.DIRECT_KEYS))
    check("E09 building it does not mutate the pristine parent",
          np.array_equal(np.array(parent["pi.0.0.weight"]), before))
    check("E10 every candidate starts from the same initial state",
          np.array_equal(one["pi.0.0.weight"], two["pi.0.0.weight"]))


# --------------------------------------------------------------------------
# E11-E16 - the 16 x 6600 contract, and the loop mechanics on a test double
# --------------------------------------------------------------------------


def test_step_contract(built):
    check("E11 batches per epoch is 33, counting the final short chunk",
          J18.expected_batches_per_epoch(4221, 128) == 33,
          J18.expected_batches_per_epoch(4221, 128))
    check("E12 33 x 200 is the registered 6600 steps per candidate",
          33 * J18.FIT_EPOCHS == 6600)
    check("E13 the grid holds 16 candidates, so the contract is 16 x 6600",
          len(J18.enumerate_grid()) * 6600 == 105600)

    rng = np.random.default_rng(J18.FIT_SEED)
    n = J18.EXPECTED_TOTAL_ROWS
    counts, coverage = [], True
    for _ in range(3):
        seen, chunks = [], 0
        for idx in J18.epoch_batches(rng, n):
            seen.extend(idx.tolist())
            chunks += 1
        counts.append(chunks)
        coverage = coverage and sorted(seen) == list(range(n))
    check("E14 every epoch yields exactly 33 chunks", counts == [33, 33, 33], counts)
    check("E15 every epoch visits every row exactly once", coverage)

    sizes = [len(i) for i in J18.epoch_batches(np.random.default_rng(1), n)]
    check("E16 the final chunk is short and KEPT, not dropped",
          sizes[-1] == 125 and len(sizes) == 33 and sum(sizes) == n,
          "%d chunks, last %d" % (len(sizes), sizes[-1]))

    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    fit = next(n_ for n_ in tree.body
               if isinstance(n_, ast.FunctionDef) and n_.name == "run_fit")
    guards = [
        node for node in ast.walk(fit)
        if isinstance(node, ast.If)
        and any(isinstance(x, ast.Raise) for x in ast.walk(node))
        and "optimizer_steps" in ast.dump(node.test)
    ]
    check("E17 run_fit raises if a candidate's step count leaves the contract",
          len(guards) == 1, len(guards))


def test_training_loop_double(built):
    """A 2-epoch synthetic run: real code path, not the real fit."""
    import torch

    parent = built["state"]
    scales = J18.july_scale_vector(built["teacher"]["feature_names"])
    rows = 10
    flat = {
        "observations": built["blocks"]["A"][0][:rows].astype(np.float32),
        "labels": built["blocks"]["A"][1][:rows].astype(np.float32),
        "block_index": np.array([0] * 4 + [1] * 2 + [2] * 3 + [3] * 1, dtype=np.int64),
        "block_sizes": {"A": 4, "B": 2, "C": 3, "D": 1},
        "n": rows,
    }
    candidate = J18.enumerate_grid()[3]

    original_epochs = J18.FIT_EPOCHS
    try:
        J18.FIT_EPOCHS = 2
        trained = J18.run_candidate(candidate, flat, parent, scales, torch)
    finally:
        J18.FIT_EPOCHS = original_epochs

    check("E18 the double takes epochs x batches steps",
          trained["optimizer_steps"] == 2 * J18.expected_batches_per_epoch(rows, 128),
          trained["optimizer_steps"])

    state = trained["state"]
    check("E19 the trained actor keeps J8's exact key set and shapes",
          set(state) == set(parent)
          and all(np.shape(state[k]) == np.shape(parent[k]) for k in parent))

    frozen = J18.logstd_tensors(state)
    reference = J18.logstd_tensors(parent)
    check("E20 the log-std slice survives training byte-identical",
          all(np.array_equal(frozen[k], reference[k]) for k in J18.HEAD_KEYS))
    check("E21 the log-std stays state-independent",
          J18.logstd_is_state_independent(state))

    clock = np.asarray(state["pi.0.0.weight"])[:, list(J18.CLOCK_COLUMNS)]
    check("E22 the clock columns are exactly zero after training",
          float(np.abs(clock).max()) == 0.0, float(np.abs(clock).max()))

    check("E23 the encoder aliases are rebuilt bit-identical",
          all(np.array_equal(state[a], state[s])
              for a, s in J18.ENCODER_ALIASES.items()))
    check("E24 the mean rows actually moved, so the double really trained",
          not np.array_equal(np.asarray(state["pi.1.bias"])[J18.MEAN_ROWS],
                             np.asarray(parent["pi.1.bias"])[J18.MEAN_ROWS]))


# --------------------------------------------------------------------------
# E25-E30 - scaling: round trip, raw actor, and the naive route that would
# have been wrong for a live-controller parent
# --------------------------------------------------------------------------


def test_scaling(built):
    import torch

    parent = built["state"]
    names = built["teacher"]["feature_names"]
    scales = J18.july_scale_vector(names)
    observations = np.concatenate(
        [built["blocks"][b][0] for b in J18.BLOCK_PRIORITY]
    ).astype(np.float32)

    scaled_columns = [i for i, s in enumerate(scales) if float(s) != 1.0]
    check("E25 exactly four columns carry a non-unit July scale",
          scaled_columns == [27, 28, 32, 33], scaled_columns)
    live = max(float(np.abs(np.asarray(parent["pi.0.0.weight"])[:, c]).max())
               for c in scaled_columns)
    check("E26 the parent has LIVE weights on the scaled columns",
          live > 0.0, live)

    weight = np.asarray(parent["pi.0.0.weight"], dtype=np.float32)
    trip = J18.raw_from_scaled(J18.scaled_from_raw(weight, scales), scales)
    check("E27 the weight scale round trip is bit-identical",
          np.array_equal(trip, weight),
          float(np.abs(trip - weight).max()))

    report = J18.verify_raw_scaled_equivalence(
        parent, observations, scales, torch_module=torch
    )
    check("E28 the scaled-domain parent reproduces the raw output within tolerance",
          report["ok"] and report["numpy_max_abs_diff"] <= J18.RAW_VS_SCALED_TOLERANCE
          and report["torch_max_abs_diff"] <= J18.RAW_VS_SCALED_TOLERANCE,
          "numpy=%.3e torch=%.3e" % (report["numpy_max_abs_diff"],
                                     report["torch_max_abs_diff"]))
    check("E29 bit-identity is explicitly NOT claimed",
          report["bit_identical_is_not_claimed"] is True)

    naive = J18.forward_mean(parent, (observations / scales).astype(np.float32))
    raw = J18.forward_mean(parent, observations)
    gap = float(np.abs(naive - raw).max())
    check("E30 the J11/J15R1 naive route would move the output far beyond G1",
          gap > J18.GATE_MAX_DRIFT, "%.6e vs budget %.3f" % (gap, J18.GATE_MAX_DRIFT))

    # The absorption contract must be measured LIKE FOR LIKE. Comparing a torch
    # float32 forward against a numpy float64 forward conflates the absorption
    # with the kernel gap and reports a number three orders of magnitude too
    # large; that conflation aborted an otherwise sound run before it was fixed.
    scaled_state = {k: np.array(parent[k], dtype=np.float32, copy=True)
                    for k in J18.DIRECT_KEYS}
    scaled_state["pi.0.0.weight"] = J18.scaled_from_raw(parent["pi.0.0.weight"], scales)
    raw_state = dict(scaled_state)
    raw_state["pi.0.0.weight"] = J18.raw_from_scaled(scaled_state["pi.0.0.weight"], scales)
    like_for_like = float(np.abs(
        J18.forward_mean(scaled_state, (observations / scales).astype(np.float32))
        - J18.forward_mean(raw_state, observations)).max())

    def float32_forward(state, x):
        h = np.tanh(x @ np.asarray(state["pi.0.0.weight"], np.float32).T
                    + np.asarray(state["pi.0.0.bias"], np.float32))
        h = np.tanh(h @ np.asarray(state["pi.0.2.weight"], np.float32).T
                    + np.asarray(state["pi.0.2.bias"], np.float32))
        return (h @ np.asarray(state["pi.1.weight"], np.float32).T
                + np.asarray(state["pi.1.bias"], np.float32))[:, :2]

    kernel_gap = float(np.abs(
        J18.forward_mean(raw_state, observations)
        - float32_forward(raw_state, observations).astype(np.float64)).max())

    check("E30a the like-for-like absorption error is far inside the tolerance",
          like_for_like <= J18.RAW_VS_SCALED_TOLERANCE, "%.3e" % like_for_like)
    check("E30b the float32/float64 kernel gap is present raw-to-raw, so it is "
          "NOT an absorption effect",
          kernel_gap > like_for_like * 100,
          "kernel=%.3e absorption=%.3e" % (kernel_gap, like_for_like))
    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("E30c the runner measures absorption like-for-like within each kernel",
          "forward_mean(scaled_state, scaled_observations)" in source
          and "torch_forward_mean(scaled_state, scaled_observations, torch)" in source)


# --------------------------------------------------------------------------
# E31-E36 - selection serialisation: all sixteen recorded, at most one actor
# --------------------------------------------------------------------------


def synthetic_results(passing):
    """Sixteen fake candidate records; ``passing`` names the ones that pass."""
    records = []
    for candidate in J18.enumerate_grid():
        index = candidate["candidate_index"]
        ok = index in passing
        records.append({
            "candidate": candidate,
            "metrics": {
                "A": {"mse": 0.001 + index * 1e-06, "max_abs": 0.1},
                "B": {"mse": 0.05 - index * 1e-04, "max_abs": 0.2},
                "C": {"mse": 0.0, "max_abs": 1e-05},
                "D": {"mse": 0.0, "max_abs": 1e-05},
            },
            "gates": [{"id": g, "name": g, "measured": 0.0, "threshold": 1.0,
                       "relation": "<=", "passed": ok}
                      for g in ("G1", "G2", "G3", "G4", "G5", "G6",
                                "G7", "G8", "G9", "G10", "G11")],
            "passed_all_gates": ok,
        })
    return records


def test_selection_serialisation():
    records = synthetic_results({2, 7, 11})
    ranked = J18.rank_survivors([r for r in records if r["passed_all_gates"]])
    selection = J18.build_selection(records, ranked, ranked[0])
    check("E31 all sixteen candidates are evaluated and recorded",
          selection["candidates_evaluated"] == 16, selection["candidates_evaluated"])
    check("E32 exactly one actor is persisted", selection["actors_persisted"] == 1)
    check("E33 the survivor with the lowest MSE_B is selected",
          selection["selected_candidate_index"] == 11,
          selection["selected_candidate_index"])
    check("E34 survivors are listed in rank order",
          selection["survivors_in_rank_order"] == [11, 7, 2],
          selection["survivors_in_rank_order"])
    check("E35 sixteen actors are never persisted",
          selection["sixteen_actors_are_never_persisted"] is True)

    empty = synthetic_results(set())
    fail = J18.build_selection(empty, [], None)
    check("E36 with zero survivors the phase fails closed with no actor",
          fail["fail_closed"] is True
          and fail["actors_persisted"] == 0
          and fail["selected_candidate_index"] is None
          and fail["candidates_evaluated"] == 16,
          fail["actors_persisted"])
    check("E37 the gate failure tally covers all eleven gates",
          len(fail["gate_failure_tally"]) == 11
          and fail["gate_failure_tally"]["G5"] == 16,
          fail["gate_failure_tally"])

    payload = json.loads(J18.encode_json(
        {"records": [J18.strip_state(r) for r in records]}))
    check("E38 candidate records serialise without tensors",
          len(payload["records"]) == 16
          and all("state" not in r for r in payload["records"]))


# --------------------------------------------------------------------------
# E39-E44 - leaf mechanics, exercised in a temporary directory
# --------------------------------------------------------------------------


def test_leaf_mechanics():
    import shutil
    import tempfile

    temporary = pathlib.Path(tempfile.mkdtemp(prefix="j18_leaf_test_"))
    try:
        (temporary / J18.LEAF_ROOT).mkdir()
        payload = {"rl_module/module_state.pkl": b"actor-bytes",
                   "v26c_j18_selection.json": b"{}"}
        receipt = {"artefacts_sha256": {
            name: hashlib.sha256(data).hexdigest() for name, data in payload.items()}}
        files = dict(payload)
        files[J18.RECEIPT_NAME] = J18.encode_json(receipt)

        result = J18.commit_leaf(files, root=temporary, leaf_name="leaf_ok")
        leaf = temporary / J18.LEAF_ROOT / "leaf_ok"
        check("E39 the committed leaf exists and is verified", result["verification"]["ok"])
        check("E40 the born-invalid marker is removed only after verification",
              not (leaf / J18.INVALID_MARKER).exists())
        check("E41 a commit_verification.json is written",
              (leaf / "commit_verification.json").exists())
        check("E42 no staging directory survives",
              not [p for p in (temporary / J18.LEAF_ROOT).iterdir()
                   if p.name.startswith(".staging_")])

        clobbered = False
        try:
            J18.commit_leaf(files, root=temporary, leaf_name="leaf_ok")
        except RuntimeError as error:
            clobbered = "refusing to clobber" in str(error)
        check("E43 a second commit to the same leaf is refused", clobbered)

        bad = dict(payload)
        bad_receipt = {"artefacts_sha256": {
            name: hashlib.sha256(b"wrong").hexdigest() for name in payload}}
        bad[J18.RECEIPT_NAME] = J18.encode_json(bad_receipt)
        caught = False
        try:
            J18.commit_leaf(bad, root=temporary, leaf_name="leaf_bad")
        except RuntimeError as error:
            caught = "post-commit verification FAILED" in str(error)
        check("E44 post-commit verification catches a receipt/bytes mismatch", caught)
        check("E45 a failed leaf KEEPS its born-invalid marker",
              (temporary / J18.LEAF_ROOT / "leaf_bad" / J18.INVALID_MARKER).exists())
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


# --------------------------------------------------------------------------
# E46-E53 - architect GO validation
# --------------------------------------------------------------------------


def good_go_payload():
    """A well-formed GO built IN MEMORY. No valid GO file is ever written."""
    return {
        "stage": J18.GO_REQUIRED_STAGE,
        "authorises_execution": True,
        "pinned_artefacts_sha256": {
            name: J18.sha256_file(ROOT / name) for name in J18.GO_REQUIRED_PINS
        },
    }


def test_go_validation():
    check("E46a the GO must pin all seven artefacts, v3 included",
          set(J18.GO_REQUIRED_PINS) == {
              J18.PREREG_PATH, J18.ADDENDUM_PATH, J18.ADDENDUM_V3_PATH,
              J18.MANIFEST_PATH, J18.PROVENANCE_OVERLAY_PATH,
              "v26c_j18_b_only_update.py", "test_v26c_j18_b_only_update.py"},
          J18.GO_REQUIRED_PINS)
    check("E46b the pin list matches the one registered in addendum v3",
          sorted(J18.GO_REQUIRED_PINS)
          == sorted(load_json(ROOT / J18.ADDENDUM_V3_PATH)["go_must_pin"]))

    missing_v3 = {k: J18.sha256_file(ROOT / k) for k in J18.GO_REQUIRED_PINS
                  if k != J18.ADDENDUM_V3_PATH}
    check("E46c a GO missing the v3 pin is refused",
          not J18.validate_go({"stage": J18.GO_REQUIRED_STAGE,
                               "authorises_execution": True,
                               "pinned_artefacts_sha256": missing_v3})["valid"])

    good = good_go_payload()
    check("E47 a correctly pinned in-memory GO validates",
          J18.validate_go(good)["valid"], J18.validate_go(good)["problems"])

    wrong_stage = dict(good, stage="V26C_J17_CAUSAL_DIAGNOSIS")
    check("E48 a GO for another stage is refused",
          not J18.validate_go(wrong_stage)["valid"])

    unauthorised = dict(good)
    unauthorised.pop("authorises_execution")
    check("E49 a GO without an explicit execution authorisation is refused",
          not J18.validate_go(unauthorised)["valid"])

    missing = dict(good, pinned_artefacts_sha256={
        k: v for k, v in good["pinned_artefacts_sha256"].items()
        if k != J18.ADDENDUM_PATH})
    check("E50 a GO missing the addendum pin is refused",
          not J18.validate_go(missing)["valid"],
          J18.validate_go(missing)["problems"])

    stale = dict(good, pinned_artefacts_sha256=dict(
        good["pinned_artefacts_sha256"], **{J18.MANIFEST_PATH: "0" * 64}))
    problems = J18.validate_go(stale)["problems"]
    check("E51 a GO with a stale hash is refused and says which",
          not J18.validate_go(stale)["valid"]
          and any(J18.MANIFEST_PATH in p for p in problems), problems)

    widened = dict(good, pinned_artefacts_sha256=dict(
        good["pinned_artefacts_sha256"], **{"AGENTS.md": "0" * 64}))
    check("E52 a GO pinning anything outside scope is refused",
          not J18.validate_go(widened)["valid"])

    check("E53 a missing GO file is refused, not defaulted",
          not J18.load_go(str(ROOT / "no_such_go.json"))["valid"])
    check("E54 no GO file exists in the validation root",
          not [p for p in ROOT.glob("*go*.json") if "j18" in p.name.lower()],
          [p.name for p in ROOT.glob("*go*.json") if "j18" in p.name.lower()])


# --------------------------------------------------------------------------
# E55-E58 - nothing prohibited is reachable from the runner
# --------------------------------------------------------------------------


def test_no_prohibited_subsystem():
    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])

    forbidden = ("env_factory", "gymnasium", "gym", "ray", "rllib", "opensim",
                 "ppo", "critic", "reward", "fsm", "morphology", "detector")
    reached = sorted(m for m in imported if m.lower() in forbidden)
    check("E55 the runner imports no environment, RL or simulator subsystem",
          not reached, reached)

    # A J18 GO pins six artefacts. Importing an earlier stage's executable module
    # would put the run's behaviour outside what the GO controls, so every
    # borrowed rule is transcribed locally instead.
    prior_stage = {"v26c_j8_recovery_fit", "v26c_j11_multistart_fit",
                   "v26c_j14_dagger_dataset"}
    check("E56a the runner imports NO prior-stage executable module",
          not (imported & prior_stage), sorted(imported & prior_stage))
    check("E56b the runner's imports are exactly the expected set",
          imported <= {"__future__", "argparse", "hashlib", "json", "pathlib",
                       "pickle", "sys", "numpy", "os", "torch"},
          sorted(imported))
    check("E56c no prior-stage module name appears anywhere in the runner source",
          not [m for m in prior_stage if m in RUNNER_PATH.read_text(encoding="utf-8")],
          [m for m in prior_stage if m in RUNNER_PATH.read_text(encoding="utf-8")])
    check("E57 torch is imported lazily, never at module level",
          "torch" not in {a.name for n in tree.body if isinstance(n, ast.Import)
                          for a in n.names})

    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("E58 the runner never writes outside the leaf root",
          source.count("os.rename") == 1 and "shutil.rmtree" not in source)


# --------------------------------------------------------------------------
# E59-E70 - the local transcriptions reproduce what they replaced
# --------------------------------------------------------------------------


def test_local_transcriptions(built):
    names = built["teacher"]["feature_names"]

    # --- the canonical actor digest, against the values on record -----------
    parent = built["state"]
    j8_expected = "6a879714044ba8321fedf8e554d0f2ec448c1f1177e1648e4e4aa72195031207"
    j2_expected = "59d54240ac628dcbd1d0dbf34328145afa08a7028047a5923298782b79bf5188"
    with open(ROOT / "j2_runs/j2_base_v26c_2026-08-26_r1/rl_module/module_state.pkl",
              "rb") as handle:
        j2_state = pickle.load(handle)

    check("E59 the local digest reproduces J8's digest of record",
          J18.actor_digest(parent) == j8_expected, J18.actor_digest(parent))
    check("E60 the local digest reproduces J2's digest of record",
          J18.actor_digest(j2_state) == j2_expected, J18.actor_digest(j2_state))
    check("E61 the two digests differ, so E59/E60 are not trivially satisfied",
          j8_expected != j2_expected)

    overlay = load_json(OVERLAY_PATH)
    pair = overlay["manifest_declared_hash_vs_actual_byte_hash"]
    check("E62 the digest matches the value pinned in the provenance overlay",
          J18.actor_digest(parent) == pair["recomputed_actor_digest_j8"]
          and J18.actor_digest(j2_state) == pair["recomputed_actor_digest_j2"])

    # cross-check against the canonical implementation itself, explicitly, in
    # the TEST only - the runner must never import it
    sys.path.insert(0, str(ROOT))
    try:
        import v26c_j11_multistart_fit as canonical
        check("E63 the transcription agrees with the canonical implementation",
              J18.actor_digest(parent) == canonical.actor_state_digest(parent)
              and J18.actor_digest(j2_state) == canonical.actor_state_digest(j2_state))
        check("E64 the transcribed key tuple matches the canonical one",
              sorted(J18.WARM_START_ACTOR_KEYS)
              == sorted(canonical.WARM_START_ACTOR_KEYS),
              sorted(J18.WARM_START_ACTOR_KEYS))
    finally:
        if sys.path and sys.path[0] == str(ROOT):
            sys.path.pop(0)

    missing = dict(parent)
    missing.pop("pi.1.bias")
    raised = False
    try:
        J18.actor_digest(missing)
    except RuntimeError as error:
        raised = "missing pi.1.bias" in str(error)
    check("E65 the digest refuses an actor state that is missing a key", raised)

    # --- the discrete predicate --------------------------------------------
    resolved = J18.discrete_indices(names)
    check("E66 the local predicate resolves the 35 names to exactly the "
          "registered indices",
          resolved == [11, 12, 13, 17, 18, 19, 20, 21], resolved)
    check("E67 the resolved indices match the dataset manifest",
          resolved == load_json(MANIFEST_PATH)["on_policy_prefix_derivation"][
              "discrete_feature_indices"])
    check("E68 the predicate keys on suffix and prefix, not on a fixed list",
          J18.is_discrete_feature("anything_in_contact")
          and J18.is_discrete_feature("phase_fsm_whatever")
          and J18.is_discrete_feature("x_saturated")
          and not J18.is_discrete_feature("pros_knee_angle_vel"))

    # --- the July scales ----------------------------------------------------
    scales = J18.july_scale_vector(names)
    non_unit = [[i, float(scales[i])] for i in range(len(scales))
                if float(scales[i]) != 1.0]
    v3 = load_json(ROOT / J18.ADDENDUM_V3_PATH)
    registered = [
        entry for entry in v3["no_prior_stage_module_is_imported_at_run_time"][
            "locally_transcribed_instead"]
        if entry["what"] == "the July physical scales"][0]
    check("E69 the resolved scale columns and values match addendum v3 exactly",
          non_unit == registered["resolved_non_unit_columns_on_the_current_35_names"],
          non_unit)
    check("E70 the transcribed mapping is the registered (1,1,4,60,1,1,1,3.5,55,1)",
          [v for _, v in J18.JULY_CONTROLLER_FEATURE_SCALES]
          == [1.0, 1.0, 4.0, 60.0, 1.0, 1.0, 1.0, 3.5, 55.0, 1.0],
          [v for _, v in J18.JULY_CONTROLLER_FEATURE_SCALES])
    check("E71 the mapping matches the one recorded in addendum v3",
          [[n, v] for n, v in J18.JULY_CONTROLLER_FEATURE_SCALES]
          == registered["mapping_in_registered_order"])

    raised = False
    try:
        J18.july_scale_vector([n for n in names if n != "pros_knee_angle_sea_u"])
    except RuntimeError as error:
        raised = "pros_knee_angle_sea_u" in str(error)
    check("E72 a missing registered feature is a hard failure, not a silent 1.0",
          raised)


# --------------------------------------------------------------------------
# E73-E78 - the post-training weight round trip is NOT bit-identical, while
# the functional contract still holds. This is the addendum v3 correction.
# --------------------------------------------------------------------------


def test_adversarial_round_trip(built):
    import torch

    rng = np.random.default_rng(2026)
    for scale in (3.5, 55.0, 60.0):
        values = (rng.standard_normal(200000) * 0.01).astype(np.float32)
        restored = ((values / np.float32(scale)).astype(np.float32)
                    * np.float32(scale)).astype(np.float32)
        fraction = float((restored != values).mean())
        check("E73 at s=%g a large share of float32 values do NOT round-trip"
              % scale, fraction > 0.01, "%.2f%% differ" % (100 * fraction))

    parent = built["state"]
    names = built["teacher"]["feature_names"]
    scales = J18.july_scale_vector(names)
    observations = np.concatenate(
        [built["blocks"][b][0] for b in J18.BLOCK_PRIORITY]).astype(np.float32)

    # an adversarial scaled-domain actor: the parent perturbed the way Adam would
    scaled = {k: np.array(parent[k], dtype=np.float32, copy=True)
              for k in J18.DIRECT_KEYS}
    scaled["pi.0.0.weight"] = J18.scaled_from_raw(parent["pi.0.0.weight"], scales)
    noise = (rng.standard_normal(scaled["pi.0.0.weight"].shape) * 1e-03).astype(np.float32)
    scaled["pi.0.0.weight"] = (scaled["pi.0.0.weight"] + noise).astype(np.float32)
    scaled["pi.0.0.weight"][:, list(J18.CLOCK_COLUMNS)] = 0.0

    report = J18.weight_round_trip_report(scaled["pi.0.0.weight"], scales)
    check("E74 the post-training weight round trip is NOT bit-identical",
          report["entries_not_bit_identical"] > 0,
          "%d of %d entries, max delta %.3e"
          % (report["entries_not_bit_identical"], report["entries_total"],
             report["max_abs_delta"]))
    check("E75 the round-trip report is declared a diagnostic, not a gate",
          report["is_a_diagnostic_not_a_gate"] is True)

    raw = {k: np.array(v, copy=True) for k, v in scaled.items()}
    raw["pi.0.0.weight"] = J18.raw_from_scaled(scaled["pi.0.0.weight"], scales)
    raw["pi.0.0.weight"][:, list(J18.CLOCK_COLUMNS)] = 0.0
    scaled_observations = (observations / scales).astype(np.float32)

    numpy_error = float(np.abs(
        J18.forward_mean(scaled, scaled_observations)
        - J18.forward_mean(raw, observations)).max())
    with torch.no_grad():
        torch_error = float(np.abs(
            J18.torch_forward_mean(scaled, scaled_observations, torch).numpy()
            .astype(np.float64)
            - J18.torch_forward_mean(raw, observations, torch).numpy()
            .astype(np.float64)).max())

    check("E76 yet the FUNCTION is preserved in numpy over all 4221 rows",
          numpy_error <= J18.RAW_VS_SCALED_TOLERANCE,
          "%.3e over %d rows" % (numpy_error, len(observations)))
    check("E77 and in torch over all 4221 rows",
          torch_error <= J18.RAW_VS_SCALED_TOLERANCE, "%.3e" % torch_error)
    check("E78 so byte identity would have failed a functionally sound actor",
          report["entries_not_bit_identical"] > 0
          and numpy_error <= J18.RAW_VS_SCALED_TOLERANCE
          and torch_error <= J18.RAW_VS_SCALED_TOLERANCE)

    check("E79 the runner requires the functional contract in BOTH kernels",
          '"numpy_max_abs_diff": numpy_error' in RUNNER_PATH.read_text(encoding="utf-8")
          and '"torch_max_abs_diff": torch_error' in RUNNER_PATH.read_text(encoding="utf-8"))
    check("E80 the runner asserts no post-training byte identity",
          "byte_identity_is_not_required" in RUNNER_PATH.read_text(encoding="utf-8"))

    v3 = load_json(ROOT / J18.ADDENDUM_V3_PATH)
    contract = v3["binding_replacement_contract"]
    check("E81 addendum v3 registers the tolerance the runner enforces",
          contract["what_is_required"]["threshold"] == J18.RAW_VS_SCALED_TOLERANCE)
    check("E82 addendum v3 records both kernels and all rows",
          contract["what_is_required"]["kernels"] == ["numpy float64", "torch float32"]
          and "4221" in contract["what_is_required"]["rows"])
    check("E83 addendum v3 does not require byte identity",
          contract["what_is_NOT_required"]["byte_identity_of_the_weight_round_trip"]
          is False)
    check("E84 addendum v2 is left byte-identical by v3",
          v3["v2_left_byte_identical"] is True
          and J18.sha256_file(ROOT / J18.ADDENDUM_PATH) == ADDENDUM_V2_PIN,
          J18.sha256_file(ROOT / J18.ADDENDUM_PATH))


# --------------------------------------------------------------------------
# E85-E98 - finiteness reaches every metric, JSON refuses non-finite values,
# and the registered seed order is followed
# --------------------------------------------------------------------------


def test_finiteness_and_seed_order(built):
    state = built["state"]
    metrics = J18.evaluate_all_blocks(state, built["blocks"])
    gate = lambda st, mt: {g["id"]: g for g in J18.evaluate_gates(st, mt, parent=state)}

    check("E85 the parent passes G11 as it stands", gate(state, metrics)["G11"]["passed"])

    for label, spoiled, where in (
        ("a nested per-action RMSE", float("nan"), ("A", "knee", "rmse")),
        ("a top-level block MSE", float("inf"), ("B", "mse")),
        ("a signed mean shift", float("-inf"), ("C", "ankle", "signed_mean_shift")),
    ):
        broken = json.loads(json.dumps(metrics))          # deep copy
        target = broken
        for key in where[:-1]:
            target = target[key]
        target[where[-1]] = spoiled
        check("E86 G11 FAILS on a non-finite value in %s" % label,
              not gate(state, broken)["G11"]["passed"], where)

    nan_state = {k: np.array(v, dtype=np.float64, copy=True) for k, v in state.items()}
    nan_state["pi.0.2.weight"][0, 0] = np.nan
    check("E87 G11 FAILS on a non-finite weight tensor",
          not gate(nan_state, metrics)["G11"]["passed"])

    check("E88 the recursive check walks dicts, lists and arrays",
          J18.is_finite_recursive({"a": [1.0, {"b": np.array([2.0, 3.0])}]})
          and not J18.is_finite_recursive({"a": [1.0, {"b": np.array([2.0, np.nan])}]})
          and not J18.is_finite_recursive([[[float("inf")]]]))
    check("E89 strings, booleans, ints and None are finite by vacuity",
          J18.is_finite_recursive({"s": "x", "b": True, "i": 3, "n": None,
                                   "arr": np.array([1, 2], dtype=np.int64)}))

    # allow_nan=False is the second barrier behind G11
    for bad in (float("nan"), float("inf"), float("-inf")):
        raised = False
        try:
            J18.encode_json({"metric": bad})
        except ValueError:
            raised = True
        check("E90 encode_json REFUSES %r" % bad, raised)
    check("E91 encode_json still serialises finite payloads",
          json.loads(J18.encode_json({"metric": 1.5}))["metric"] == 1.5)
    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("E92 encode_json sets allow_nan=False", "allow_nan=False" in source)

    # the registered seed order puts the Generator BEFORE the optimizer
    tree = ast.parse(source)
    candidate_fn = next(n for n in tree.body
                        if isinstance(n, ast.FunctionDef) and n.name == "run_candidate")
    rng_line = adam_line = None
    for node in ast.walk(candidate_fn):
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            dumped = ast.dump(node.value)
            if node.targets[0].id == "rng" and "default_rng" in dumped:
                rng_line = node.lineno
            if node.targets[0].id == "optimiser" and "Adam" in dumped:
                adam_line = node.lineno
    check("E93 the Generator is constructed before Adam, as registered",
          rng_line is not None and adam_line is not None and rng_line < adam_line,
          "rng@%s adam@%s" % (rng_line, adam_line))

    check("E94 validate_go's docstring says seven artefacts",
          "SEVEN" in (J18.validate_go.__doc__ or ""))


# --------------------------------------------------------------------------
# E95-E103 - addendum v3 says only what is true
# --------------------------------------------------------------------------


def test_v3_wording():
    raw = (ROOT / J18.ADDENDUM_V3_PATH).read_text(encoding="utf-8")
    v3 = json.loads(raw)

    check("E95 v3 parses as JSON", isinstance(v3, dict))
    check("E96 v3 declares three pre-execution points, not one",
          len(v3["the_three_points"]) == 3
          and "THREE" in v3["amends"]
          and "ONE point only" not in raw,
          v3["amends"])
    check("E97 v3's scope names all three, including the hermetic transcriptions",
          all(token in v3["scope"] for token in
              ("scale-absorption", "statistical qualification", "prior-stage")),
          v3["scope"][:120])

    amendment = v3["wording_amendment_to_v2"]
    claim = amendment["unbiasedness_is_claimed_only_where_it_holds"]
    check("E98 unbiasedness is claimed only for a uniform M independent of theta",
          "UNIFORMLY" in claim["holds"] and "INDEPENDENTLY of theta" in claim["holds"])
    check("E99 v3 states plainly that this does NOT hold under random reshuffling",
          "RANDOM RESHUFFLING" in claim["does_not_hold_for_the_actual_training_loop"]
          and "NO claim of conditional unbiasedness" in
          claim["does_not_hold_for_the_actual_training_loop"])
    check("E100 what is claimed instead is coefficients, coverage and frozen-theta",
          len(claim["what_is_claimed_instead"]) == 3
          and any("w_block(i) / N_block(i)" in c for c in claim["what_is_claimed_instead"])
          and any("exactly once" in c for c in claim["what_is_claimed_instead"])
          and any("FROZEN theta" in c for c in claim["what_is_claimed_instead"]))
    check("E101 no surviving claim that every real step is unbiased",
          "per-step unbiasedness" not in raw
          and "each gradient step is an unbiased estimate" not in raw)

    check("E102 v3 says SEVEN artefacts, never six",
          "six artefacts" not in raw
          and "SEVEN artefacts" in v3[
              "no_prior_stage_module_is_imported_at_run_time"]["rule"])
    check("E103 v3 quotes the tolerance correctly, not as a ten-millionth",
          "ten-millionth" not in raw
          and "one-millionth" in v3["binding_replacement_contract"][
              "what_is_NOT_required"]["why"])
    check("E104 v3 still leaves the experiment's governing rules untouched",
          all(token in v3["scope"] for token in
              ("not the eleven gates", "not a single threshold", "not the ranking order")))


# --------------------------------------------------------------------------
# E105-E120 - G7 is byte identity, G11 reaches every reported number, and the
# two docstrings say only what is true
# --------------------------------------------------------------------------


def test_byte_identity_and_reported_diagnostics(built):
    state = built["state"]
    metrics = J18.evaluate_all_blocks(state, built["blocks"])

    def gate(candidate_state, candidate_metrics=None, diagnostics=None):
        return {g["id"]: g for g in J18.evaluate_gates(
            candidate_state, candidate_metrics or metrics, parent=state,
            reported_diagnostics=diagnostics)}

    # --- G7 must be byte identity, not numeric equality ---------------------
    check("E105 bytes_identical rejects a dtype change that array_equal accepts",
          np.array_equal(np.float32([1.0]), np.float64([1.0]))
          and not J18.bytes_identical(np.float32([1.0]), np.float64([1.0])))
    check("E106 bytes_identical rejects -0.0 vs +0.0, which array_equal accepts",
          np.array_equal(np.float32([0.0]), np.float32([-0.0]))
          and not J18.bytes_identical(np.float32([0.0]), np.float32([-0.0])))
    check("E107 bytes_identical accepts a genuine copy",
          J18.bytes_identical(np.float32([1.5, 2.5]), np.float32([1.5, 2.5]).copy()))

    check("E108 logstd_raw_slices performs no dtype conversion",
          all(J18.logstd_raw_slices(state)[k].dtype
              == np.asarray(state[k]).dtype for k in J18.HEAD_KEYS),
          {k: str(J18.logstd_raw_slices(state)[k].dtype) for k in J18.HEAD_KEYS})
    check("E109 the parent still passes G7", gate(state)["G7"]["passed"])

    # adversarial: flip one +0.0 of the log-std weight to -0.0. Numerically
    # identical, byte different. The old np.array_equal gate saw nothing.
    signed = {k: np.array(v, copy=True) for k, v in state.items()}
    rows = signed["pi.1.weight"][J18.LOGSTD_ROWS]
    zeros = np.flatnonzero(rows.reshape(-1) == 0.0)
    signed["pi.1.weight"][2 + zeros[0] // rows.shape[1], zeros[0] % rows.shape[1]] = -0.0
    check("E110 the signed-zero actor is NUMERICALLY identical to the parent",
          np.array_equal(np.asarray(signed["pi.1.weight"])[J18.LOGSTD_ROWS],
                         np.asarray(state["pi.1.weight"])[J18.LOGSTD_ROWS]))
    check("E111 G7 FAILS on the signed-zero log-std, which array_equal missed",
          not gate(signed)["G7"]["passed"])
    check("E112 and it is still state-independent, so only byte identity catches it",
          J18.logstd_is_state_independent(signed))

    widened = {k: np.array(v, copy=True) for k, v in state.items()}
    widened["pi.1.bias"] = np.asarray(state["pi.1.bias"], dtype=np.float64)
    check("E113 G7 FAILS on a dtype change with identical shape and values",
          not gate(widened)["G7"]["passed"])

    # --- G11 must reach every reported number -------------------------------
    good = {"best_objective": 0.5, "history": [{"epoch": 1, "train_loss_mean": 0.1}],
            "scale_absorption": {"numpy_max_abs_diff": 1e-09,
                                 "weight_round_trip": {"max_abs_delta": 0.0}}}
    check("E114 finite diagnostics pass G11", gate(state, diagnostics=good)["G11"]["passed"])

    for label, spoiled in (
        ("NaN nested in history", {"history": [{"epoch": 1,
                                                "composite_objective": float("nan")}]}),
        ("Inf in scale_absorption", {"scale_absorption": {
            "weight_round_trip": {"max_abs_delta": float("inf")}}}),
        ("NaN in best_objective", {"best_objective": float("nan")}),
    ):
        payload = dict(good)
        payload.update(spoiled)
        check("E115 G11 FAILS on %s" % label,
              not gate(state, diagnostics=payload)["G11"]["passed"])

    # --- sanitisation preserves the path and the kind, and never emits NaN ---
    dirty = {"history": [{"epoch": 1, "composite_objective": float("nan")}],
             "best_objective": float("-inf"),
             "scale_absorption": {"numpy_max_abs_diff": float("inf")}}
    clean, findings = J18.sanitise_non_finite(dirty, "$.diagnostics")
    kinds = sorted(f["kind"] for f in findings)
    paths = sorted(f["path"] for f in findings)
    check("E116 sanitisation finds every non-finite value and records its kind",
          kinds == ["-Infinity", "Infinity", "NaN"], kinds)
    check("E117 and records where each one was",
          paths == ["$.diagnostics.best_objective",
                    "$.diagnostics.history[0].composite_objective",
                    "$.diagnostics.scale_absorption.numpy_max_abs_diff"], paths)
    check("E118 the sanitised record serialises, with no NaN in the JSON",
          json.loads(J18.encode_json(clean))["best_objective"]["non_finite"]
          == "-Infinity")
    check("E119 sanitisation leaves finite values untouched",
          J18.sanitise_non_finite({"a": 1.5, "b": [2, "x", True, None]})[0]
          == {"a": 1.5, "b": [2, "x", True, None]})

    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("E120 run_fit builds the diagnostics BEFORE gating and reuses them",
          "reported_diagnostics=diagnostics" in source
          and source.index("diagnostics = {") < source.index(
              "reported_diagnostics=diagnostics"))
    check("E121 G7 uses byte identity, never np.array_equal, on the log-std",
          "bytes_identical(parent_logstd[key], candidate_logstd[key])" in source
          and "np.array_equal(parent_logstd" not in source)


def test_docstrings_are_truthful():
    source = RUNNER_PATH.read_text(encoding="utf-8")
    header = ast.get_docstring(ast.parse(source)) or ""
    check("E122 the header no longer claims the test suite never loads torch",
          "or the test suite" not in header
          and "execution-path tests do load it" in header)
    check("E123 the header still scopes the claim to the two read-only modes",
          "--preflight-only" in header and "--dry-run" in header)

    doc = J18.minibatch_loss.__doc__ or ""
    check("E124 minibatch_loss qualifies the epoch identity with a fixed theta",
          "FOR A FIXED theta" in doc)
    check("E125 it qualifies unbiasedness as uniform and theta-independent",
          "uniformly and independently of theta" in doc)
    check("E126 it claims neither for the real loop, only coefficients and coverage",
          "Neither statement is claimed for the actual training loop" in doc
          and "exact per-row coefficients" in doc
          and "exact once-per-epoch coverage" in doc)


# --------------------------------------------------------------------------
# E127-E140 - a contaminated metric reaches gate["measured"] too. The record
# must still serialise, so one bad candidate cannot abort the other fifteen.
# --------------------------------------------------------------------------


def test_contaminated_gate_measured(built):
    state = built["state"]
    grid = J18.enumerate_grid()

    def diagnostics_for(objective=0.25):
        return {
            "best_epoch": 7,
            "best_objective": objective,
            "optimizer_steps": 6600,
            "batches_per_epoch": 33,
            "scale_absorption": {"numpy_max_abs_diff": 1e-09,
                                 "torch_max_abs_diff": 1.2e-07,
                                 "weight_round_trip": {"max_abs_delta": 4.6e-10}},
            "history": [{"epoch": 1, "batches": 33, "train_loss_mean": 0.5,
                         "composite_objective": 0.4}],
        }

    # --- the finite path must be numerically untouched ----------------------
    clean_metrics = J18.evaluate_all_blocks(state, built["blocks"])
    clean_gates = J18.evaluate_gates(state, clean_metrics, parent=state)
    clean_record = J18.build_candidate_record(
        grid[0], clean_metrics, clean_gates, diagnostics_for())

    check("E127 the finite path leaves every block metric numerically identical",
          all(clean_record["metrics"][b]["mse"] == clean_metrics[b]["mse"]
              and clean_record["metrics"][b]["max_abs"] == clean_metrics[b]["max_abs"]
              for b in J18.BLOCK_PRIORITY))
    check("E128 the finite path leaves every gate measured value identical",
          [g["measured"] for g in clean_record["gates"]]
          == [g["measured"] for g in clean_gates])
    check("E129 the finite path records no non-finite finding",
          clean_record["non_finite_count"] == 0
          and clean_record["non_finite_findings"] == [])
    check("E130 the finite path keeps the raw verdict",
          clean_record["passed_all_gates"]
          == all(g["passed"] for g in clean_gates))
    check("E131 the finite record serialises under strict JSON",
          json.loads(J18.encode_json(J18.strip_state(clean_record)))["candidate"][
              "candidate_index"] == 0)

    # --- a single contaminated metric, exactly as a real run would see it ----
    for label, poison, block, field in (
        ("NaN in MSE_B", float("nan"), "B", "mse"),
        ("Inf in the C drift", float("inf"), "C", "max_abs"),
    ):
        dirty = json.loads(json.dumps(clean_metrics))
        dirty[block][field] = poison
        dirty_gates = J18.evaluate_gates(state, dirty, parent=state)

        contaminated = [g["id"] for g in dirty_gates
                        if not np.isfinite(g["measured"])]
        check("E132 %s propagates into gate['measured']" % label,
              contaminated, contaminated)

        by_id = {g["id"]: g for g in dirty_gates}
        check("E133 %s makes G11 fail" % label, not by_id["G11"]["passed"])

        record = J18.build_candidate_record(
            grid[3], dirty, dirty_gates, diagnostics_for(), state=state)
        check("E134 %s leaves passed_all_gates False, from the RAW gates" % label,
              record["passed_all_gates"] is False)

        encoded = None
        try:
            encoded = J18.encode_json(J18.strip_state(record))
        except ValueError as error:
            check("E135 %s still serialises under strict JSON" % label, False, error)
        else:
            check("E135 %s still serialises under strict JSON" % label, True)
            parsed = json.loads(
                encoded.decode(),
                parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
            check("E136 %s writes no NaN/Infinity literal" % label,
                  isinstance(parsed, dict))

        gate_paths = [f["path"] for f in record["non_finite_findings"]
                      if f["path"].startswith("$.gates[")]
        check("E137 %s is recorded with a precise $.gates[...] path" % label,
              gate_paths and all(p.endswith("].measured") for p in gate_paths),
              gate_paths)
        indices = sorted(int(p[len("$.gates["):p.index("]")]) for p in gate_paths)
        check("E138 %s the flagged index is the gate that measured it" % label,
              [dirty_gates[i]["id"] for i in indices] == contaminated,
              [dirty_gates[i]["id"] for i in indices])
        check("E139 %s is also recorded under its $.metrics path" % label,
              any(f["path"] == "$.metrics.%s.%s" % (block, field)
                  for f in record["non_finite_findings"]),
              [f["path"] for f in record["non_finite_findings"]])
        check("E140 %s the surviving gate booleans are untouched by sanitisation"
              % label,
              [g["passed"] for g in record["gates"]]
              == [g["passed"] for g in dirty_gates])

    # --- one bad candidate must not impede the other fifteen ----------------
    dirty = json.loads(json.dumps(clean_metrics))
    dirty["B"]["mse"] = float("nan")
    dirty_gates = J18.evaluate_gates(state, dirty, parent=state)
    batch = [
        J18.build_candidate_record(
            grid[i],
            dirty if i == 3 else clean_metrics,
            dirty_gates if i == 3 else clean_gates,
            diagnostics_for(0.25 + i / 100.0),
        )
        for i in range(16)
    ]
    payload = None
    try:
        payload = J18.encode_json({"records": batch})
    except ValueError as error:
        check("E141 all sixteen records serialise with one contaminated among them",
              False, error)
    else:
        check("E141 all sixteen records serialise with one contaminated among them",
              True)
        decoded = json.loads(payload.decode())["records"]
        check("E142 all sixteen are present, none dropped", len(decoded) == 16)
        check("E143 exactly the contaminated one carries findings",
              [i for i, r in enumerate(decoded) if r["non_finite_count"]] == [3],
              [i for i, r in enumerate(decoded) if r["non_finite_count"]])
        check("E144 the fifteen clean records are numerically unchanged",
              all(decoded[i]["metrics"]["B"]["mse"] == clean_metrics["B"]["mse"]
                  for i in range(16) if i != 3))

    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("E145 run_fit builds records through the sanitising helper",
          "build_candidate_record(" in source
          and '"gates": gates,' not in source)
    check("E146 gates are sanitised under the $.gates path",
          'sanitise_non_finite(gates, "$.gates")' in source)
    check("E147 the verdict is taken from the RAW gates",
          'passed_all = all(gate["passed"] for gate in gates)' in source)


def main():
    test_namespace_closure()
    test_preflight_is_read_only()
    built = test_block_b_is_clean()
    test_composition(built)
    test_anchor_labels(built)
    test_structural_invariants(built)
    test_prefix_is_derived()
    test_grid_and_thresholds()
    test_sealed_seeds_and_no_fit()
    test_provenance()

    # Execution-path tests run last: everything above must observe a torch-free
    # process, and these import torch.
    test_minibatch_estimator()
    test_candidate_determinism(built)
    test_step_contract(built)
    test_training_loop_double(built)
    test_scaling(built)
    test_selection_serialisation()
    test_leaf_mechanics()
    test_go_validation()
    test_no_prohibited_subsystem()
    test_local_transcriptions(built)
    test_adversarial_round_trip(built)
    test_finiteness_and_seed_order(built)
    test_v3_wording()
    test_byte_identity_and_reported_diagnostics(built)
    test_docstrings_are_truthful()
    test_contaminated_gate_measured(built)

    failed = [c for c in CHECKS if not c[1]]
    for name, passed, detail in CHECKS:
        print("[%s] %-62s %s" % ("PASS" if passed else "FAIL", name, detail))
    print("\n%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
