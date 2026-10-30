"""V26C J19A - single reproduction of the selected J18 candidate.

Stage 1 of a two-phase protocol. J19A refits ONE configuration - candidate 13
of the frozen J18 grid - checks it reproduces, applies the offline eligibility
rule, and commits at most ONE actor into ONE leaf. It performs NO rollout.

J19B, the binding closed-loop A-F requalification, is a SEPARATE stage. It is
described in the readiness report, is not implemented here, and cannot be
started from this module. It will be preregistered only after the architect has
audited the J19A leaf.

Unlike J18, this runner deliberately IMPORTS the frozen J18 runner and reuses
its training path unmodified, pinned by hash. J18 had to import no prior-stage
module because its behaviour had to follow from the artefacts its GO pinned;
J19A's purpose is the opposite - to reproduce a J18 result - and a re-transcribed
training loop would make any mismatch ambiguous between genuine irreproducibility
and a transcription error. The inversion is declared in the preregistration.

No historical runner is modified. J16 and the rest remain immutable evidence.

Usage:
    python v26c_j19a_single_reproduction.py --preflight-only
    python v26c_j19a_single_reproduction.py --dry-run
    python v26c_j19a_single_reproduction.py --execute --go-file <architect GO json>

``--preflight-only`` and ``--dry-run`` write nothing, create no directory, run
no fit and never import torch.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import pickle
import sys

import numpy as np

# The dynamic import of the frozen J18 runner would otherwise leave a
# __pycache__ entry in the validation root, so the preflight's "writes nothing"
# claim would hold only when the caller happened to set PYTHONDONTWRITEBYTECODE.
# Setting it here, before any such import can occur, makes the claim
# self-contained rather than conditional on the environment.
sys.dont_write_bytecode = True

VALIDATION_ROOT = pathlib.Path(__file__).resolve().parent

PREREG_PATH = "v26c_j19a_prereg_single_reproduction.json"

J18_RUNNER_NAME = "v26c_j18_b_only_update.py"
J18_RUNNER_SHA256 = "d3949631ddd135c50bbd91eec26d9f77b70bdf41692a4500e01d0d01a34e992b"
J18_DATASET_MANIFEST = "v26c_j18_dataset_manifest.json"
J18_DATASET_MANIFEST_SHA256 = (
    "ba9ebf9c77550595562d3047368483f8fab7d4c46a0032c177ffd278a07910d2"
)
J18_LEAF = "j18_runs/j18_b_only_update_v26c_2026-08-27_r1"
J18_LEAF_ARTEFACTS = {
    "v26c_j18_candidate_metrics.json":
        "f4b8688dd8d4d0c4f330064dce4ac9ccd021e9529669a04e97ea615f226f36f3",
    "v26c_j18_selection.json":
        "8ef47ef6343399dfded8ffe85ec109a6b420d0c804c32ba19d7b8906348aac30",
    "v26c_j18_b_only_update_receipt.json":
        "600c3c45a49d39da93e4e6e890a479e923e51d456cf92c4faa245031929452db",
    "history.json":
        "3eaba5128c1d0734a60127cddeb5f383d794fa59951a3c7cf2eb9e8dd4e46eee",
    "commit_verification.json":
        "15bd6e5d3510ce7974d5df0a4669077a3d5995332e8d56cfd22447a3d44d3247",
}

J2_MODULE_PATH = "j2_runs/j2_base_v26c_2026-08-26_r1/rl_module/module_state.pkl"
J2_MODULE_SHA256 = "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"

SELECTED_CANDIDATE_INDEX = 13
EXPECTED_BEST_EPOCH = 191
EXPECTED_OPTIMIZER_STEPS = 6600
EXPECTED_BATCHES_PER_EPOCH = 33

# Numerical tolerance for the reproduction, NOT a performance gate.
REPRO_REL_TOL = 1e-06
REPRO_ABS_TOL = 1e-09

# Binding ceilings measured from J8's own update, recomputed and asserted at run
# time. Not derived from any sigma; the criterion is not-worse-than-operational-J8.
J8_CEILING_BIAS_C = 0.0008992143952043874
J8_CEILING_BIAS_D = 0.0007264156156278919
J8_CEILING_RMSE_C = 0.00447396874790294
J8_CEILING_RMSE_D = 0.005262698258768814
CEILING_ASSERT_TOL = 1e-12

# Preregistered in J18 v1 and unchanged.
GATE_MSE_B_CEILING = 0.09546555953591015
GATE_MSE_A_CEILING = 0.004108013186414973

# Recorded, never binding.
DIAGNOSTIC_MAX_DRIFT_REFERENCE = 0.005

LEAF_ROOT = "j19a_runs"
LEAF_NAME = "j19a_single_reproduction_v26c_2026-08-27_r1"
RECEIPT_NAME = "v26c_j19a_single_reproduction_receipt.json"
INVALID_MARKER = "TECHNICAL_INVALID"

GO_REQUIRED_STAGE = "V26C_J19A_SINGLE_REPRODUCTION"
GO_REQUIRED_PINS = (
    PREREG_PATH,
    "v26c_j19a_single_reproduction.py",
    "test_v26c_j19a_single_reproduction.py",
    J18_RUNNER_NAME,
)


def sha256_file(path):
    """SHA-256 of a file's bytes."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def encode_json(payload):
    """Deterministic JSON bytes; non-finite values are refused."""
    return json.dumps(payload, indent=2, sort_keys=True, allow_nan=False).encode("utf-8")


def load_j18(root=VALIDATION_ROOT):
    """Import the frozen J18 runner, verifying its bytes before use."""
    path = root / J18_RUNNER_NAME
    actual = sha256_file(path)
    if actual != J18_RUNNER_SHA256:
        raise RuntimeError(
            "the pinned J18 runner changed: expected %s, found %s. A reproduction "
            "is meaningless against altered training code."
            % (J18_RUNNER_SHA256, actual)
        )
    sys.path.insert(0, str(root))
    try:
        import v26c_j18_b_only_update as j18
    finally:
        if sys.path and sys.path[0] == str(root):
            sys.path.pop(0)
    return j18


def frozen_record(root=VALIDATION_ROOT):
    """The selected candidate's frozen record, READ from the pinned J18 leaf.

    Never hardcoded: the leaf is pinned by hash and the numbers come from it.
    """
    leaf = root / J18_LEAF
    for name, expected in sorted(J18_LEAF_ARTEFACTS.items()):
        actual = sha256_file(leaf / name)
        if actual != expected:
            raise RuntimeError(
                "the J18 leaf artefact %s changed: expected %s, found %s"
                % (name, expected, actual)
            )
    with open(leaf / "v26c_j18_candidate_metrics.json", "r", encoding="utf-8") as handle:
        records = json.load(handle)["records"]
    for record in records:
        if record["candidate"]["candidate_index"] == SELECTED_CANDIDATE_INDEX:
            return record
    raise RuntimeError("candidate %d is absent from the J18 leaf" % SELECTED_CANDIDATE_INDEX)


def frozen_metrics(record):
    """The six frozen metrics compared under the reproducibility contract."""
    metrics = record["metrics"]
    return {
        "MSE_A": metrics["A"]["mse"],
        "MSE_B": metrics["B"]["mse"],
        "rmse_C": metrics["C"]["rmse"],
        "rmse_D": metrics["D"]["rmse"],
        "max_abs_C": metrics["C"]["max_abs"],
        "max_abs_D": metrics["D"]["max_abs"],
    }


def fresh_metrics(metrics):
    """The same six quantities from a freshly evaluated actor."""
    return {
        "MSE_A": metrics["A"]["mse"],
        "MSE_B": metrics["B"]["mse"],
        "rmse_C": metrics["C"]["rmse"],
        "rmse_D": metrics["D"]["rmse"],
        "max_abs_C": metrics["C"]["max_abs"],
        "max_abs_D": metrics["D"]["max_abs"],
    }


def compare_reproduction(fresh, frozen, exact_fresh, exact_frozen):
    """The reproducibility contract. Numerical tolerance, not a performance gate."""
    numeric = []
    for key in sorted(frozen):
        got, want = fresh[key], frozen[key]
        ok = math.isclose(got, want, rel_tol=REPRO_REL_TOL, abs_tol=REPRO_ABS_TOL)
        numeric.append({
            "field": key, "fresh": got, "frozen": want,
            "abs_delta": abs(got - want),
            "rel_delta": (abs(got - want) / abs(want)) if want else None,
            "rel_tol": REPRO_REL_TOL, "abs_tol": REPRO_ABS_TOL, "passed": bool(ok),
        })
    exact = []
    for key in sorted(exact_frozen):
        got, want = exact_fresh[key], exact_frozen[key]
        exact.append({"field": key, "fresh": got, "frozen": want, "passed": got == want})
    return {
        "numeric": numeric,
        "exact": exact,
        "comparison": "math.isclose(rel_tol=%r, abs_tol=%r)" % (REPRO_REL_TOL, REPRO_ABS_TOL),
        "is_numerical_tolerance_not_a_performance_gate": True,
        "ok": all(e["passed"] for e in numeric) and all(e["passed"] for e in exact),
    }


def compare_exact_fields(j18, built, reference, root=VALIDATION_ROOT):
    """The FULL field-by-field exact comparison required by D6.

    Every exact field the preregistration names is compared here and recorded in
    the result: the seed and fit protocol, the eight candidate/config fields, the
    four block row counts, the observation and label hashes of all four blocks,
    the parent's bytes and digest, and the pinned provenance. Nothing is left to
    an implicit reliance on the preflight.
    """
    with open(root / J18_DATASET_MANIFEST, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    with open(root / J18_LEAF / "v26c_j18_b_only_update_receipt.json", "r",
              encoding="utf-8") as handle:
        receipt = json.load(handle)

    fields = []

    def compare(name, fresh, frozen, note=None):
        entry = {"field": name, "fresh": fresh, "frozen": frozen,
                 "passed": bool(fresh == frozen)}
        if note:
            entry["note"] = note
        fields.append(entry)

    protocol = receipt["protocol"]
    compare("protocol.seed", j18.FIT_SEED, protocol["seed"])
    compare("protocol.epochs", j18.FIT_EPOCHS, protocol["epochs"])
    compare("protocol.batch_size", j18.FIT_BATCH_SIZE, protocol["batch_size"])
    compare("protocol.batches_per_epoch",
            j18.expected_batches_per_epoch(len(np.concatenate(
                [built["blocks"][b][0] for b in j18.BLOCK_PRIORITY]))),
            protocol["batches_per_epoch"])
    compare("protocol.optimizer_steps_per_candidate",
            EXPECTED_OPTIMIZER_STEPS, protocol["optimizer_steps_per_candidate"])

    grid = [c for c in j18.enumerate_grid()
            if c["candidate_index"] == SELECTED_CANDIDATE_INDEX]
    entry = grid[0] if grid else {}
    for key in ("candidate_index", "learning_rate", "on_policy_weight_beta",
                "preservation_weight_lambda", "w_A", "w_B", "w_C", "w_D"):
        compare("candidate.%s" % key, entry.get(key), reference["candidate"][key])

    for block in j18.BLOCK_PRIORITY:
        compare("rows.%s" % block, int(len(built["blocks"][block][0])),
                receipt["dataset"]["rows_by_block"][block])

    manifest_blocks = {b["id"]: b for b in manifest["block_composition"]["blocks"]}
    for block in j18.BLOCK_PRIORITY:
        observations, labels = built["blocks"][block]
        compare("observations_sha256.%s" % block, j18.sha256_array(observations),
                manifest_blocks[block]["observations_sha256"])
        note = None
        if not manifest_blocks[block]["labels_read_from_file_not_recomputed"]:
            note = ("HISTORICAL DISTINCTION: this block's labels are COMPUTED, not read "
                    "from a file. J18's contract was CROSS-PLATFORM and therefore bound "
                    "them by a 1e-06 tolerance rather than by hash, because a BLAS "
                    "difference can move the last bits. J19A is a SAME-PLATFORM "
                    "reproduction and requires the exact hash; a mismatch here means the "
                    "computation changed, not that the platform did.")
        compare("labels_sha256.%s" % block, j18.sha256_array(labels),
                manifest_blocks[block]["labels_sha256"], note)

    compare("parent_j8_module_sha256", sha256_file(root / j18.PARENT_MODULE_PATH),
            receipt["inputs"]["parent"]["actual_byte_sha256"])
    compare("parent_j8_actor_digest", j18.actor_digest(built["state"]),
            receipt["inputs"]["parent"]["actor_digest"])
    compare("parent_of_j8_module_sha256", sha256_file(root / J2_MODULE_PATH),
            J2_MODULE_SHA256)

    compare("provenance.j18_runner_sha256", sha256_file(root / J18_RUNNER_NAME),
            J18_RUNNER_SHA256)
    compare("provenance.dataset_manifest_sha256",
            sha256_file(root / J18_DATASET_MANIFEST), J18_DATASET_MANIFEST_SHA256)
    compare("provenance.dataset_manifest_matches_j18_receipt",
            sha256_file(root / J18_DATASET_MANIFEST),
            receipt["inputs"]["dataset_manifest_sha256"])
    for name, expected in sorted(J18_LEAF_ARTEFACTS.items()):
        compare("provenance.j18_leaf.%s" % name,
                sha256_file(root / J18_LEAF / name), expected)

    failed = [f["field"] for f in fields if not f["passed"]]
    return {
        "fields": fields,
        "count": len(fields),
        "failed_fields": failed,
        "relation": "exact equality, every field, fail-closed",
        "label_hash_policy": {
            "blocks_read_from_file": "exact hash required",
            "block_D_computed_labels": "exact hash required in THIS same-platform "
                                       "reproduction; J18's cross-platform contract used a "
                                       "1e-06 tolerance instead",
            "j18_cross_platform_tolerance": 1e-06,
        },
        "ok": not failed,
    }


def j8_reference_ceilings(j18, root=VALIDATION_ROOT):
    """Recompute J8's own drift from J2 on the identical blocks, then assert.

    Derived, then checked. The ceilings are never merely trusted.
    """
    built = j18.build_blocks(root)
    with open(root / J2_MODULE_PATH, "rb") as handle:
        parent_of_j8 = pickle.load(handle)
    j8 = built["state"]

    out = {}
    for name, expect_rmse, expect_bias in (
        ("C", J8_CEILING_RMSE_C, J8_CEILING_BIAS_C),
        ("D", J8_CEILING_RMSE_D, J8_CEILING_BIAS_D),
    ):
        observations = built["blocks"][name][0]
        delta = j18.forward_mean(j8, observations) - j18.forward_mean(
            parent_of_j8, observations
        )
        rmse = float(np.sqrt((delta ** 2).mean()))
        knee, ankle = float(delta[:, 0].mean()), float(delta[:, 1].mean())
        bias = max(abs(knee), abs(ankle))
        out[name] = {
            "rmse": rmse, "bias": bias,
            "bias_knee_signed": knee, "bias_ankle_signed": ankle,
            "max_abs": float(np.abs(delta).max()),
            "rmse_matches_prereg": abs(rmse - expect_rmse) <= CEILING_ASSERT_TOL,
            "bias_matches_prereg": abs(bias - expect_bias) <= CEILING_ASSERT_TOL,
        }
    out["ok"] = all(
        out[k]["rmse_matches_prereg"] and out[k]["bias_matches_prereg"] for k in ("C", "D")
    )
    out["origin"] = ("MEASURED from J8 against its own parent J2 on the identical "
                     "blocks; not-worse-than-operational-J8, not derived from sigma")
    return out, built


def post_485_diagnostic(j18, state, root=VALIDATION_ROOT):
    """Drift on the 485 post-mismatch rows of cell B. DIAGNOSTIC, never binding."""
    trace = j18.load_trace("B", root)
    rows = j18.trace_observations(trace)[15:]
    parent = j18.load_parent_state(root)
    delta = j18.forward_mean(state, rows) - j18.forward_mean(parent, rows)
    return {
        "rows": int(len(rows)),
        "max_abs": float(np.abs(delta).max()),
        "rmse": float(np.sqrt((delta ** 2).mean())),
        "bias_knee_signed": float(delta[:, 0].mean()),
        "bias_ankle_signed": float(delta[:, 1].mean()),
        "binding": False,
        "why_diagnostic": "these rows are the TARGET REGION of the correction; a "
                          "corrected actor is supposed to change there, so any "
                          "association with closed-loop failure is confounded by design",
    }


def evaluate_eligibility(j18, state, metrics, parent, ceilings, g11_payload):
    """The binding offline rule. G1/G2 are recorded as diagnostics only.

    ``g11_payload`` is the SINGLE explicit payload carrying every numeric family
    the result will publish other than the state and the block metrics, which
    are covered separately: ``candidate``, ``training_diagnostics`` (best_epoch,
    best_objective, optimizer_steps, batches_per_epoch, scale_absorption,
    post_485_diagnostic, history), ``j8_ceilings``,
    ``raw_vs_scaled_equivalence``, ``reproducibility`` and ``exact_fields``.

    G11's recursive finiteness runs over all of it. Passing a narrower payload
    would leave published numbers outside every check, which is exactly what the
    preregistration forbids.
    """
    if g11_payload is None:
        raise RuntimeError(
            "evaluate_eligibility requires g11_payload: G11 must cover every "
            "published numeric field, not only the block metrics"
        )
    missing = [f for f in G11_PAYLOAD_FAMILIES if f not in g11_payload]
    if missing:
        raise RuntimeError(
            "g11_payload is missing published numeric families %s; G11 would not "
            "cover them" % missing
        )
    reported_diagnostics = g11_payload
    structural = {g["id"]: g for g in j18.evaluate_gates(
        state, metrics, parent=parent, reported_diagnostics=reported_diagnostics)}

    def bias_of(block):
        return max(abs(metrics[block]["knee"]["signed_mean_shift"]),
                   abs(metrics[block]["ankle"]["signed_mean_shift"]))

    binding = [
        ("G5", "on_policy_improvement_B", metrics["B"]["mse"], GATE_MSE_B_CEILING),
        ("G6", "teacher_non_regression_A", metrics["A"]["mse"], GATE_MSE_A_CEILING),
        ("G3", "preservation_bias_C", bias_of("C"), ceilings["C"]["bias"]),
        ("G4", "preservation_bias_D", bias_of("D"), ceilings["D"]["bias"]),
        ("E-C", "preservation_rmse_C", metrics["C"]["rmse"], ceilings["C"]["rmse"]),
        ("E-D", "preservation_rmse_D", metrics["D"]["rmse"], ceilings["D"]["rmse"]),
    ]
    records = [
        {"id": i, "name": n, "measured": float(m), "threshold": float(t),
         "relation": "<=", "binding": True, "passed": bool(m <= t)}
        for i, n, m, t in binding
    ]
    for gate_id in ("G7", "G8", "G9", "G10", "G11"):
        gate = dict(structural[gate_id])
        gate["binding"] = True
        records.append(gate)

    diagnostics = [
        {"id": "G1", "name": "preservation_max_drift_C",
         "measured": metrics["C"]["max_abs"],
         "recorded_reference": DIAGNOSTIC_MAX_DRIFT_REFERENCE, "binding": False},
        {"id": "G2", "name": "preservation_max_drift_D",
         "measured": metrics["D"]["max_abs"],
         "recorded_reference": DIAGNOSTIC_MAX_DRIFT_REFERENCE, "binding": False},
    ]
    return {
        "binding": records,
        "diagnostic_only": diagnostics,
        "ok": all(r["passed"] for r in records),
    }


def commit_leaf(files, root=VALIDATION_ROOT, leaf_name=LEAF_NAME):
    """Write a leaf atomically, born invalid, verified after commit."""
    import os

    leaf_root = root / LEAF_ROOT
    final = leaf_root / leaf_name
    if final.exists():
        raise RuntimeError("refusing to clobber an existing leaf: %s" % final)

    digest = hashlib.sha256()
    for name in sorted(files):
        digest.update(name.encode("utf-8"))
        digest.update(hashlib.sha256(files[name]).hexdigest().encode("ascii"))
    aggregate = digest.hexdigest()

    staging = leaf_root / (".staging_%s" % aggregate[:16])
    if staging.exists():
        raise RuntimeError("refusing to reuse an existing staging directory: %s" % staging)
    staging.mkdir(parents=True)
    (staging / INVALID_MARKER).write_bytes(
        b"born invalid; removed only after post-commit verification succeeds\n"
    )
    for name in sorted(files):
        target = staging / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(files[name])
    os.rename(str(staging), str(final))

    problems = []
    with open(final / RECEIPT_NAME, "r", encoding="utf-8") as handle:
        committed = json.load(handle)
    for name, expected in sorted(committed["artefacts_sha256"].items()):
        path = final / name
        if not path.exists():
            problems.append("committed receipt names a missing file: %s" % name)
            continue
        actual = sha256_file(path)
        if actual != expected:
            problems.append("committed %s hashes %s, receipt says %s" % (name, actual, expected))
        if name in files and hashlib.sha256(files[name]).hexdigest() != actual:
            problems.append("committed %s differs from the staged bytes" % name)

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(final.relative_to(root)),
        "aggregate_digest": aggregate,
        "files_verified": len(committed["artefacts_sha256"]),
        "problems": problems,
        "ok": not problems,
        "method": "every leaf-relative path is re-resolved from the COMMITTED receipt, "
                  "re-hashed, and compared both to the receipt and to the staged bytes",
    }
    (final / "commit_verification.json").write_bytes(encode_json(verification))
    if problems:
        raise RuntimeError(
            "post-commit verification FAILED; the invalid marker is left in place: %s"
            % problems
        )
    (final / INVALID_MARKER).unlink()
    return {"leaf": str(final.relative_to(root)), "verification": verification}


def validate_go(payload, root=VALIDATION_ROOT):
    """Validate an architect GO payload. Pure: takes a dict, touches no GO file.

    A GO authorises J19A only. It never authorises a rollout: J19B is a separate
    stage, preregistered only after this leaf is audited.
    """
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"], "pins": {}}
    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append("stage is %r, expected %r" % (payload.get("stage"), GO_REQUIRED_STAGE))
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")
    if payload.get("authorises_rollout") is True:
        problems.append("a J19A GO must never authorise a rollout; J19B is a separate stage")

    pins = payload.get("pinned_artefacts_sha256")
    if not isinstance(pins, dict):
        problems.append("pinned_artefacts_sha256 is missing or is not an object")
        pins = {}
    for required in GO_REQUIRED_PINS:
        if required not in pins:
            problems.append("no pin for %s" % required)
    for name in sorted(pins):
        if name not in GO_REQUIRED_PINS:
            problems.append("pin for %s is outside the authorised scope" % name)
            continue
        path = root / name
        if not path.exists():
            problems.append("pinned artefact %s does not exist" % name)
            continue
        actual = sha256_file(path)
        if actual != pins[name]:
            problems.append("pinned hash for %s is stale: GO says %s, the file is %s"
                            % (name, pins[name], actual))
    return {"valid": not problems, "problems": problems, "pins": dict(pins)}


def load_go(go_file):
    """Read a GO file from disk and validate it."""
    path = pathlib.Path(go_file)
    if not path.exists():
        return {"valid": False, "problems": ["the GO file %s does not exist" % go_file],
                "pins": {}}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except (ValueError, OSError) as error:
        return {"valid": False, "problems": ["the GO file is unreadable: %s" % error],
                "pins": {}}
    return validate_go(payload)


def run_preflight(root=VALIDATION_ROOT, verbose=True):
    """Verify every precondition without training and without writing."""
    checks = []

    def record(name, passed, detail=""):
        checks.append({"check": name, "passed": bool(passed), "detail": str(detail)})

    actual = sha256_file(root / J18_RUNNER_NAME)
    record("j18_runner_pinned_bytes", actual == J18_RUNNER_SHA256, actual)
    for name, expected in sorted(J18_LEAF_ARTEFACTS.items()):
        got = sha256_file(root / J18_LEAF / name)
        record("j18_leaf:%s" % name, got == expected, got)
    record("j2_parent_bytes", sha256_file(root / J2_MODULE_PATH) == J2_MODULE_SHA256)

    j18 = load_j18(root)
    record("j18_import_after_hash_check", j18.__name__ == "v26c_j18_b_only_update")

    record("j18_preflight_still_passes", j18.run_preflight(root, verbose=False)["ok"])

    ceilings, built = j8_reference_ceilings(j18, root)
    record("j8_ceilings_recomputed_and_match_prereg", ceilings["ok"], {
        "C": {"rmse": ceilings["C"]["rmse"], "bias": ceilings["C"]["bias"]},
        "D": {"rmse": ceilings["D"]["rmse"], "bias": ceilings["D"]["bias"]},
    })

    record_c13 = frozen_record(root)
    record("selected_candidate_is_13",
           record_c13["candidate"]["candidate_index"] == SELECTED_CANDIDATE_INDEX)
    record("frozen_best_epoch_is_191", record_c13["best_epoch"] == EXPECTED_BEST_EPOCH,
           record_c13["best_epoch"])
    record("frozen_steps_are_6600",
           record_c13["optimizer_steps"] == EXPECTED_OPTIMIZER_STEPS)
    record("frozen_batches_per_epoch_are_33",
           record_c13["batches_per_epoch"] == EXPECTED_BATCHES_PER_EPOCH)

    grid = [c for c in j18.enumerate_grid()
            if c["candidate_index"] == SELECTED_CANDIDATE_INDEX]
    record("grid_entry_matches_the_frozen_candidate",
           len(grid) == 1 and grid[0] == record_c13["candidate"], grid)

    exact = compare_exact_fields(j18, built, record_c13, root)
    record("exact_field_comparison", exact["ok"],
           "%d fields, failed: %s" % (exact["count"], exact["failed_fields"]))
    for entry in exact["fields"]:
        record("exact:%s" % entry["field"], entry["passed"], entry["fresh"])

    frozen = frozen_metrics(record_c13)
    record("six_frozen_metrics_are_finite",
           all(math.isfinite(v) for v in frozen.values()), frozen)

    prereg = json.loads((root / PREREG_PATH).read_text(encoding="utf-8"))
    record("prereg_declares_no_rollout_in_this_stage",
           prereg["invariants"]["no_rollout_in_this_stage"] is True)
    record("prereg_declares_j19b_not_implemented",
           prereg["phase_separation"]["j19b_is_not_implemented_in_this_stage"] is True)

    record("no_leaf_yet", not (root / LEAF_ROOT / LEAF_NAME).exists())
    record("torch_not_imported_by_preflight", "torch" not in sys.modules)

    report = {
        "stage": GO_REQUIRED_STAGE,
        "mode": "PREFLIGHT",
        "trained_anything": False,
        "wrote_anything": False,
        "created_any_directory": False,
        "performed_any_rollout": False,
        "checks": checks,
        "passed": sum(1 for c in checks if c["passed"]),
        "total": len(checks),
        "ok": all(c["passed"] for c in checks),
        "selected_candidate": record_c13["candidate"],
        "frozen_metrics": frozen,
        "j8_ceilings": ceilings,
        "exact_fields": exact,
    }
    if verbose:
        for check in checks:
            print("[%s] %-46s %s" % ("PASS" if check["passed"] else "FAIL",
                                     check["check"], check["detail"]))
        print("\npreflight %s: %d/%d checks"
              % ("PASS" if report["ok"] else "FAIL", report["passed"], report["total"]))
        print("trained nothing, wrote nothing, created no directory, ran no rollout")
    return report


def run_reproduction(go_file, root=VALIDATION_ROOT, progress=False):
    """Refit the selected candidate ONCE and, if it qualifies, commit one actor."""
    go = load_go(go_file)
    if not go["valid"]:
        raise RuntimeError("refusing to execute: the architect GO is absent or invalid. %s"
                           % "; ".join(go["problems"]))
    preflight = run_preflight(root, verbose=False)
    if not preflight["ok"]:
        raise RuntimeError("refusing to execute: preflight did not pass")

    j18 = load_j18(root)
    ceilings, built = j8_reference_ceilings(j18, root)
    if not ceilings["ok"]:
        raise RuntimeError("refusing to execute: the J8 ceilings did not reproduce")

    reference = frozen_record(root)
    candidate = reference["candidate"]
    flat = j18.build_flat_dataset(built)
    parent = built["state"]
    scales = j18.july_scale_vector(built["teacher"]["feature_names"])

    import torch

    equivalence = j18.verify_raw_scaled_equivalence(
        parent, flat["observations"], scales, torch_module=torch)
    if not equivalence["ok"]:
        raise RuntimeError("refusing to execute: raw/scaled equivalence failed")

    trained = j18.run_candidate(candidate, flat, parent, scales, torch, progress=progress)
    metrics = j18.evaluate_all_blocks(trained["state"], built["blocks"])

    # POST-485 is computed BEFORE the gate so G11 can cover it. Reproducibility
    # and the exact-field comparison are computed before it too, so that ONE
    # explicit payload can carry every numeric family the result will publish.
    post485 = post_485_diagnostic(j18, trained["state"], root)
    repro = compare_reproduction(
        fresh_metrics(metrics), frozen_metrics(reference),
        {"best_epoch": trained["best_epoch"],
         "optimizer_steps": trained["optimizer_steps"],
         "batches_per_epoch": trained["batches_per_epoch"]},
        {"best_epoch": EXPECTED_BEST_EPOCH,
         "optimizer_steps": EXPECTED_OPTIMIZER_STEPS,
         "batches_per_epoch": EXPECTED_BATCHES_PER_EPOCH},
    )
    exact = compare_exact_fields(j18, built, reference, root)

    g11_payload = {
        "candidate": candidate,
        "training_diagnostics": {
            "best_epoch": trained["best_epoch"],
            "best_objective": trained["best_objective"],
            "optimizer_steps": trained["optimizer_steps"],
            "batches_per_epoch": trained["batches_per_epoch"],
            "scale_absorption": trained["scale_absorption"],
            "post_485_diagnostic": post485,
            "history": trained["history"],
        },
        "j8_ceilings": ceilings,
        "raw_vs_scaled_equivalence": equivalence,
        "reproducibility": repro,
        "exact_fields": exact,
    }
    eligibility = evaluate_eligibility(
        j18, trained["state"], metrics, parent, ceilings, g11_payload)

    promote = bool(repro["ok"] and exact["ok"] and eligibility["ok"])
    return finalise(root, j18, go, trained, metrics, g11_payload, eligibility,
                    parent, promote)


G11_PAYLOAD_FAMILIES = ("candidate", "training_diagnostics", "j8_ceilings",
                        "raw_vs_scaled_equivalence", "reproducibility", "exact_fields")


def finalise(root, j18, go, trained, metrics, g11_payload, eligibility, parent, promote):
    """Assemble and commit the leaf. At most one actor, only if it qualifies.

    Everything the result publishes is sanitised before it is written, and only
    the SAFE versions are used - including the epoch history, which is taken
    from the sanitised payload and never from the raw training output. A
    non-finite value anywhere makes promotion impossible, whatever the gates
    said: the record is still written, with each finding's path, but no actor
    accompanies it.

    The receipt carries only derived booleans, counters and hash strings, so no
    raw float can reach it.
    """
    files = {}
    safe_metrics, f1 = j18.sanitise_non_finite(metrics, "$.metrics")
    safe_payload, f2 = j18.sanitise_non_finite(g11_payload, "$.g11_payload")
    safe_elig, f3 = j18.sanitise_non_finite(eligibility, "$.eligibility")
    findings = f1 + f2 + f3

    # A non-finite value anywhere in the published record blocks promotion.
    promote = bool(promote and not findings)
    safe_training = safe_payload["training_diagnostics"]

    files["v26c_j19a_result.json"] = encode_json({
        "kind": "SINGLE REPRODUCTION RESULT",
        "stage": GO_REQUIRED_STAGE,
        "candidate": safe_payload["candidate"],
        "reproducibility": safe_payload["reproducibility"],
        "exact_fields": safe_payload["exact_fields"],
        "eligibility": safe_elig,
        "metrics": safe_metrics,
        "diagnostics": safe_training,
        "j8_ceilings": safe_payload["j8_ceilings"],
        "raw_vs_scaled_equivalence": safe_payload["raw_vs_scaled_equivalence"],
        "non_finite_findings": findings,
        "non_finite_count": len(findings),
        "promotion_blocked_by_non_finite": bool(findings),
        "actor_promoted": promote,
        "rollout_performed": False,
        "j19b_not_authorised_by_this_stage": True,
    })
    files["history.json"] = encode_json({"epoch_history": safe_training["history"]})

    if promote:
        state = trained["state"]
        module_bytes = pickle.dumps(state, protocol=4)
        files["rl_module/module_state.pkl"] = module_bytes
        source = root / j18.PARENT_MODULE_PATH.replace(
            "module_state.pkl", "class_and_ctor_args.pkl")
        files["rl_module/class_and_ctor_args.pkl"] = source.read_bytes()
        files["rl_module/actor_feature_manifest.json"] = encode_json({
            "actor_label": "J19A_SINGLE_REPRODUCTION_OF_J18_C13",
            "stage": GO_REQUIRED_STAGE,
            "status": "reproduced and offline-eligible; closed-loop A-F NOT run",
            "module_state_sha256": hashlib.sha256(module_bytes).hexdigest(),
            "actor_digest": j18.actor_digest(state),
            "source_actor_digest": j18.actor_digest(parent),
            "derived_from": {"parent": "J8",
                             "parent_module_state_sha256": j18.PARENT_MODULE_SHA256},
            "observation_width": j18.OBSERVATION_WIDTH,
            "input_convention": "RAW observations, unscaled",
            "clock_columns_exactly_zero": True,
            "truthful_about_its_own_module": True,
        })

    receipt = {
        "kind": "EXECUTION RECEIPT",
        "stage": GO_REQUIRED_STAGE,
        "verdict": "PASS" if promote else "FAIL_CLOSED_NO_ACTOR",
        "actors_persisted": 1 if promote else 0,
        "rollout_performed": False,
        "inputs": {
            "j18_runner_sha256": J18_RUNNER_SHA256,
            "j18_leaf": J18_LEAF,
            "j18_leaf_artefacts_sha256": J18_LEAF_ARTEFACTS,
            "parent_j8_sha256": j18.PARENT_MODULE_SHA256,
            "parent_of_j8_sha256": J2_MODULE_SHA256,
            "prereg_sha256": sha256_file(root / PREREG_PATH),
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "architect_go_pins": go["pins"],
        },
        # Derived booleans, counters and strings only. No raw float reaches the
        # receipt, so it cannot carry a non-finite value even in principle.
        "reproducibility_ok": bool(safe_payload["reproducibility"]["ok"]),
        "exact_fields_ok": bool(safe_payload["exact_fields"]["ok"]),
        "exact_fields_compared": int(safe_payload["exact_fields"]["count"]),
        "exact_fields_failed": list(safe_payload["exact_fields"]["failed_fields"]),
        "eligibility_ok": bool(safe_elig["ok"]),
        "non_finite_count": len(findings),
        "promotion_blocked_by_non_finite": bool(findings),
        "artefacts_sha256": {n: hashlib.sha256(p).hexdigest() for n, p in sorted(files.items())},
    }
    files[RECEIPT_NAME] = encode_json(receipt)
    committed = commit_leaf(files, root=root)
    return {"leaf": committed["leaf"], "verdict": receipt["verdict"],
            "verification": committed["verification"], "actor_promoted": promote}


def build_parser():
    """Command-line surface. Preflight is the default."""
    parser = argparse.ArgumentParser(description="V26C J19A - single reproduction")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--go-file", default=None)
    parser.add_argument("--progress", action="store_true")
    return parser


def main(argv=None):
    """Return 0 on success, 1 on failure. Writes nothing unless --execute."""
    args = build_parser().parse_args(argv)
    if args.execute:
        if not args.go_file:
            print("--execute requires --go-file", file=sys.stderr)
            return 1
        try:
            outcome = run_reproduction(args.go_file, progress=args.progress)
        except RuntimeError as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"], "verdict": outcome["verdict"]}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = run_preflight(verbose=True)
    if args.dry_run:
        print("\nplan: ONE refit of candidate %d, then reproducibility, then offline "
              "eligibility, then at most ONE actor." % SELECTED_CANDIDATE_INDEX)
        print("J19B closed-loop A-F is a SEPARATE stage and is not reachable from here.")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
