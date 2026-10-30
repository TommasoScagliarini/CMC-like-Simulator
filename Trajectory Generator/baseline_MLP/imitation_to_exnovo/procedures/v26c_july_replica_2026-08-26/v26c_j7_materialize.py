"""V26C J7 MATERIALIZE - write to disk, once, the dataset the J7 preflight already audited.

WHAT IT IS
    A separate layer. It does NOT reimplement the build: it calls assemble() from the FROZEN
    v26c_j7_markov_dataset.py. That file is hashed and compared BEFORE anything imports or
    executes it, then loaded from its exact path - never by name, which would search sys.path -
    and re-checked after loading. It recomputes nothing, filters nothing, reorders nothing.

    If a module of that name is ALREADY in sys.modules, this layer refuses outright - even when
    its __file__ and hash match the frozen builder. Those bytes executed before this layer
    authenticated them, so it cannot attest to what ran; identity is not provenance. There is no
    reuse branch. The materializer must run in a process that has not already imported it.

WHAT IT WRITES
    Exactly two files, exactly once, into exactly one leaf:
        <root>/j7_runs/j7_markov_dataset_v26c_2026-08-26_r1/
            v26c_j7_markov_recovery_dataset.npz   observations, actions, actor_feature_names
            v26c_j7_markov_dataset_receipt.json
    The relative leaf is NOT overridable. Only the root can be redirected, by an explicit
    in-process test override that stamps the receipt as non-authoritative.

HOW IT WRITES
    Atomic commit, no-clobber under an exclusive materializer lock: a sibling lock taken with
    O_CREAT|O_EXCL, a sibling staging directory, a round-trip check of the staged bytes, a
    re-check of the leaf, then the rename. The lock serialises materializer INSTANCES and claims
    nothing against a writer that ignores it - os.rename on POSIX silently replaces an existing
    empty directory, so the rename alone is not a no-clobber primitive. A pre-existing lock is a
    fail-closed stop; the run removes no lock it does not own.

    On failure it removes the exact staging directory it created, its own lock, and - only if it
    created the parent and only if the parent is empty - a non-recursive rmdir of the parent.
    Nothing else is ever removed: content that appeared concurrently in j7_runs survives.
    No autonomous retry, no partial leaf.

WHAT IT DOES NOT DO
    No fit, no environment, no rollout, no critic, no PPO. It touches no FSM, morphology, reward,
    SEA or C++ code, no J2 parent and no existing artefact.

Cross-platform: pathlib and os.rename only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


class MaterializeError(RuntimeError):
    pass


STAGE = "V26C-J7-MATERIALIZE"
AUTHORIZATION = HERE / "v26c_j7_materialize_authorization.json"
PIN_AUTHORIZATION = "a64cf7348705528cb84638143ca00f481b9c8ef432146a4989b09ad09a62383f"

# ------------------------------------------------------------------ the frozen inputs -----------
BUILDER = HERE / "v26c_j7_markov_dataset.py"
PREREG = HERE / "v26c_j7_prereg_markov_dataset.json"
BUILDER_TEST = HERE / "test_v26c_j7_markov_dataset.py"
BUILDER_MODULE_NAME = "v26c_j7_markov_dataset"
FROZEN = {
    "v26c_j7_prereg_markov_dataset.json":
        "bea97f268d2080968954cd694c7244cc3dc9d1f40ea4ae4ec79e9726fa9edf96",
    "v26c_j7_markov_dataset.py":
        "b3cc2b88997ccb7c23964123d70ac3fb524adf4fdf3e8882d26ce20ff87d1461",
    "test_v26c_j7_markov_dataset.py":
        "aca18c3f73f91d8be9fc3621a5e33b97d400ee7ef3d3b9c825db0db2bbd32924",
}


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


# ------------------------------------------------------------------ bootstrap: AUTH, THEN EXEC --
# Authentication that happens after the module has already run is not authentication. The builder's
# bytes are hashed and compared BEFORE anything imports or executes them, and it is then loaded
# from its exact path - never by name, which would search sys.path and could execute a namesake.
# BOOTSTRAP_TRACE records the order as it happens, so a test can prove verify precedes exec.
BOOTSTRAP_TRACE: list[str] = []


def load_verified_builder(path: Path | None = None, expected_sha: str | None = None,
                          module_name: str | None = None) -> Any:
    """Hash the file, refuse on mismatch, and only then import and execute it."""
    path = Path(path) if path is not None else BUILDER
    expected = expected_sha if expected_sha is not None else FROZEN["v26c_j7_markov_dataset.py"]
    name = module_name if module_name is not None else BUILDER_MODULE_NAME

    # (1) AUTHENTICATE. Nothing has been imported or executed at this point.
    if not path.is_file():
        raise MaterializeError(f"the frozen builder is missing: {path}")
    digest = _sha_file(path)
    BOOTSTRAP_TRACE.append(f"verify:{name}:{digest}")
    if digest != expected:
        raise MaterializeError(f"the frozen builder changed: {digest} != {expected}. Refusing to "
                               f"import or execute it.")

    # (2) FAIL CLOSED on ANY module of this name that is already loaded - a namesake from another
    # path, and equally one whose __file__ and hash match. Those bytes ran before this layer
    # authenticated them, so this layer cannot attest to what executed. Identity is not
    # provenance, and a matching hash of the file on disk today says nothing about what was
    # executed earlier. There is no reuse branch.
    existing = sys.modules.get(name)
    if existing is not None:
        got = getattr(existing, "__file__", None)
        BOOTSTRAP_TRACE.append(f"refuse-preloaded:{name}:{got}")
        raise MaterializeError(
            f"{name} is already present in sys.modules (loaded from {got}). This layer refuses to "
            f"reuse ANY pre-loaded module, including one whose path and hash match the frozen "
            f"builder: it was executed before this layer authenticated it, and identity is not "
            f"provenance. Run the materializer in a process that has not already imported it.")

    # (3) load from the EXACT path. Never `import <name>`: that searches sys.path.
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise MaterializeError(f"the frozen builder cannot be loaded from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    BOOTSTRAP_TRACE.append(f"exec:{name}:{path}")
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


J7B = load_verified_builder()       # the FROZEN builder; verified first, never reimplemented

# ------------------------------------------------------------------ the only destination --------
RELATIVE_LEAF_PARTS = ("j7_runs", "j7_markov_dataset_v26c_2026-08-26_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
DATASET_NAME = J7B.DATASET_NAME                     # v26c_j7_markov_recovery_dataset.npz
RECEIPT_NAME = J7B.RECEIPT_NAME                     # v26c_j7_markov_dataset_receipt.json
DATASET_KEYS = ("observations", "actions", "actor_feature_names")

# The ROOT of the destination, and nothing else, may be redirected. Production leaves this None.
# A test sets it to a temporary directory; the receipt then declares itself non-authoritative.
OUTPUT_ROOT_OVERRIDE: Path | None = None

# ------------------------------------------------------------------ the audited content ---------
EXPECTED_ROWS = 16713
EXPECTED_OBS_SHAPE = (EXPECTED_ROWS, 35)
EXPECTED_ACT_SHAPE = (EXPECTED_ROWS, 2)
EXPECTED_NAMES = 35
EXPECTED_OBS_SHA = "a59fb89b43a372162661ec8cf131b5abbf1e9ce32b16f98518af9010a4cf587a"
EXPECTED_ACT_SHA = "6e920c2f890dbdc2cfacf6a2c6842b34c6f33e24499d0d8716bd6347624bde4f"

FORBIDDEN_HERE = ("fit", "optimizer step", "training", "critic", "PPO", "ex-novo",
                  "environment construction", "rollout", "promotion", "recomputing the dataset",
                  "dedup", "downsampling", "reordering", "writing anywhere but the authorised leaf",
                  "clobbering an existing leaf", "an autonomous retry")


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(J7B.REPO))
    except ValueError:
        return str(Path(p).resolve())


# ================================================================ fail-closed verification =======

def verify_authorization() -> dict[str, Any]:
    """The authorisation, the three frozen inputs, and the identity of the imported builder."""
    if not AUTHORIZATION.is_file():
        raise MaterializeError("the materialisation authorisation is missing")
    auth_sha = _sha_file(AUTHORIZATION)
    if PIN_AUTHORIZATION != "PENDING" and auth_sha != PIN_AUTHORIZATION:
        raise MaterializeError(f"the authorisation changed: {auth_sha} != {PIN_AUTHORIZATION}")
    auth = json.loads(AUTHORIZATION.read_text())
    if auth.get("stage_authorised") != STAGE:
        raise MaterializeError(f"the authorisation names stage {auth.get('stage_authorised')!r}, "
                               f"not {STAGE}")

    # the three frozen files, byte for byte
    declared = auth["frozen_inputs"]
    frozen: dict[str, str] = {}
    for rel, pin in FROZEN.items():
        target = HERE / rel
        if not target.is_file():
            raise MaterializeError(f"the frozen input {rel} is missing")
        got = _sha_file(target)
        if got != pin:
            raise MaterializeError(f"the frozen input {rel} changed: {got} != {pin}")
        if declared.get(rel) != pin:
            raise MaterializeError(f"the authorisation pins {rel} at {declared.get(rel)}, the "
                                   f"materializer at {pin}")
        frozen[rel] = got

    # POST-LOAD re-check: the module in hand must still BE the frozen builder. The pre-exec
    # authentication above is what makes this a re-check rather than the only check.
    imported = Path(J7B.__file__).resolve()
    if imported != BUILDER.resolve():
        raise MaterializeError(f"the imported builder is {imported}, not the frozen {BUILDER}")
    if _sha_file(imported) != FROZEN["v26c_j7_markov_dataset.py"]:
        raise MaterializeError("the imported builder does not hash to the frozen value")
    if not BOOTSTRAP_TRACE or not BOOTSTRAP_TRACE[0].startswith("verify:"):
        raise MaterializeError(f"the builder was not authenticated before loading: "
                               f"{BOOTSTRAP_TRACE}")
    verify_at = next((i for i, s in enumerate(BOOTSTRAP_TRACE)
                      if s.startswith(f"verify:{BUILDER_MODULE_NAME}:")), None)
    exec_at = next((i for i, s in enumerate(BOOTSTRAP_TRACE)
                    if s.startswith(f"exec:{BUILDER_MODULE_NAME}:")), None)
    if verify_at is None or exec_at is None or verify_at > exec_at:
        raise MaterializeError(f"the builder was not verified before it was executed: "
                               f"{BOOTSTRAP_TRACE}")
    if J7B.PIN_PREREG != FROZEN["v26c_j7_prereg_markov_dataset.json"]:
        raise MaterializeError("the frozen builder pins a different preregistration")

    # the destination the authorisation fixes must be the one this module implements
    dest = auth["destination"]
    if dest["relative_leaf"] != RELATIVE_LEAF:
        raise MaterializeError(f"the authorisation fixes {dest['relative_leaf']!r}, the "
                               f"materializer {RELATIVE_LEAF!r}")
    outs = auth["authorised_outputs"]
    if outs["dataset"] != DATASET_NAME or outs["receipt"] != RECEIPT_NAME or outs["count"] != 2:
        raise MaterializeError("the authorisation and the materializer disagree on the outputs")
    contract = auth["dataset_contract"]
    if tuple(contract["keys"]) != DATASET_KEYS:
        raise MaterializeError("the authorisation and the materializer disagree on the keys")
    if contract["content_hash_observations"] != EXPECTED_OBS_SHA \
            or contract["content_hash_actions"] != EXPECTED_ACT_SHA:
        raise MaterializeError("the authorisation and the materializer disagree on the content "
                               "hashes")
    if tuple(contract["observations_shape"]) != EXPECTED_OBS_SHAPE \
            or tuple(contract["actions_shape"]) != EXPECTED_ACT_SHAPE:
        raise MaterializeError("the authorisation and the materializer disagree on the shapes")
    return {"file": _rel(AUTHORIZATION), "sha256": auth_sha, "stage": STAGE,
            "frozen_inputs": frozen, "imported_builder": _rel(imported),
            "relative_leaf": RELATIVE_LEAF,
            "bootstrap": {"authenticated_before_exec": True, "trace": list(BOOTSTRAP_TRACE),
                          "loaded_from": _rel(imported),
                          "rule": "the builder's bytes are hashed before any import or exec, and "
                                  "loaded from the exact path, never by name off sys.path",
                          "preloaded_module_policy": "FAIL CLOSED on any module of that name "
                                                     "already in sys.modules, including one whose "
                                                     "path and hash match: it ran before this "
                                                     "layer authenticated it. There is no reuse "
                                                     "branch.",
                          "reused_a_preloaded_module": False},
            "authorised_outputs": {"dataset": DATASET_NAME, "receipt": RECEIPT_NAME}}


def authorized_leaf() -> Path:
    """The one destination. The relative part is fixed; only the root can be redirected."""
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def _refuse_symlink(path: Path, root: Path) -> None:
    """No component from the root down to the leaf may be a symlink."""
    current = path
    while True:
        if current.is_symlink():
            raise MaterializeError(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_out(out_arg: str | None) -> Path:
    """--out must name exactly the authorised leaf, which must not already exist."""
    if out_arg is None:
        raise MaterializeError("--materialize requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise MaterializeError(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise MaterializeError(f"--out is {got}, which is not the authorised leaf {leaf}")
    # the structural check comes FIRST: a symlinked component is a violation whatever it points at
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise MaterializeError(f"the authorised leaf already exists; this stage is no-clobber and "
                               f"single-execution: {leaf}")
    return leaf


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise MaterializeError(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


# ================================================================ the audited payload =============

def build_payload() -> dict[str, Any]:
    """Call the FROZEN assemble() and prove the result is the one the preflight audited."""
    built = J7B.assemble()
    obs = built["observations"]
    act = built["actions"]
    names = built["actor_feature_names"]
    if obs.shape != EXPECTED_OBS_SHAPE or act.shape != EXPECTED_ACT_SHAPE:
        raise MaterializeError(f"the assembled shapes are {obs.shape}/{act.shape}, expected "
                               f"{EXPECTED_OBS_SHAPE}/{EXPECTED_ACT_SHAPE}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise MaterializeError(f"the assembled dtypes are {obs.dtype}/{act.dtype}, expected "
                               f"float32/float32")
    if len(names) != EXPECTED_NAMES:
        raise MaterializeError(f"the assembled dataset carries {len(names)} feature names, "
                               f"expected {EXPECTED_NAMES}")
    obs_sha = J7B._sha_array(obs)
    act_sha = J7B._sha_array(act)
    if obs_sha != EXPECTED_OBS_SHA:
        raise MaterializeError(f"the observations content hash is {obs_sha}, not the audited "
                               f"{EXPECTED_OBS_SHA}")
    if act_sha != EXPECTED_ACT_SHA:
        raise MaterializeError(f"the actions content hash is {act_sha}, not the audited "
                               f"{EXPECTED_ACT_SHA}")
    report = built["report"]
    if report["teacher_usage"]["observations_read"] is not False:
        raise MaterializeError("the build reports that the teacher's observations were read")
    return {"observations": obs, "actions": act, "actor_feature_names": names,
            "content_hashes": {"observations": obs_sha, "actions": act_sha},
            "report": report, "prereg": built["prereg"]}


def _verify_npz(path: Path, payload: dict[str, Any], where: str) -> dict[str, Any]:
    """Round-trip: reload from disk and prove the bytes carry the audited arrays."""
    with np.load(path, allow_pickle=False) as archive:
        keys = tuple(archive.files)
        if tuple(sorted(keys)) != tuple(sorted(DATASET_KEYS)):
            raise MaterializeError(f"the {where} dataset holds keys {keys}, expected "
                                   f"{DATASET_KEYS}")
        obs = np.asarray(archive["observations"])
        act = np.asarray(archive["actions"])
        names = [str(n) for n in np.asarray(archive["actor_feature_names"]).tolist()]
    if obs.shape != EXPECTED_OBS_SHAPE or act.shape != EXPECTED_ACT_SHAPE:
        raise MaterializeError(f"the {where} dataset is {obs.shape}/{act.shape}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise MaterializeError(f"the {where} dataset is {obs.dtype}/{act.dtype}, expected float32")
    if J7B._sha_array(obs) != EXPECTED_OBS_SHA or J7B._sha_array(act) != EXPECTED_ACT_SHA:
        raise MaterializeError(f"the {where} dataset does not round-trip to the audited content "
                               f"hashes")
    if names != [str(n) for n in payload["actor_feature_names"]]:
        raise MaterializeError(f"the {where} dataset carries different feature names")
    return {"keys": list(keys), "observations_shape": list(obs.shape),
            "actions_shape": list(act.shape), "dtype": "float32/float32",
            "actor_feature_names": len(names), "round_trip": "content hashes reproduced"}


def build_receipt(payload: dict[str, Any], auth: dict[str, Any], leaf: Path,
                  dataset_sha: str, round_trip: dict[str, Any]) -> dict[str, Any]:
    report = payload["report"]
    per_seed = {
        seed: {
            "retained_steps": v["retained_steps"],
            "penetration": {
                "series_sha256": v["penetration_band_coverage"]["series_sha256"],
                "bit_identical_to_trace": v["penetration_band_coverage"]["bit_identical_to_trace"],
                "evaluated_by": v["penetration_band_coverage"]["evaluated_by"],
                "contract_sha256": v["penetration_band_coverage"]["contract_sha256"],
                "max_penetration_m": v["penetration_band_coverage"]["max_penetration_m"],
                "band": v["penetration_band_coverage"]["band"],
                "counts": v["penetration_band_coverage"]["counts"],
                "fractions": v["penetration_band_coverage"]["fractions"],
                "binding_pass": v["penetration_band_coverage"]["binding_pass"],
                "binding_verdict": v["penetration_band_coverage"]["binding_verdict"],
                "is_a_gate_here": v["penetration_band_coverage"]["is_a_gate_here"],
            },
            "time_identity": v["time_identity"],
        }
        for seed, v in report["per_seed"].items()
    }
    overridden = OUTPUT_ROOT_OVERRIDE is not None
    return {
        "schema": "v26c_j7_materialize_receipt.1",
        "stage": STAGE,
        "authorization": auth,
        "preregistration": payload["prereg"],
        "output": {
            "leaf": _rel(leaf),
            "relative_leaf": RELATIVE_LEAF,
            "dataset": DATASET_NAME,
            "receipt": RECEIPT_NAME,
            "dataset_sha256": dataset_sha,
            "write_protocol": "atomic commit, no-clobber under exclusive materializer lock",
            "lock": {
                "file": LOCK_NAME,
                "acquisition": "os.open with O_CREAT | O_EXCL | O_WRONLY, sibling of the leaf",
                "serialises": "materializer instances, and only those",
                "does_not_guarantee": "nothing is claimed against a writer that ignores the lock. "
                                      "os.rename on POSIX silently replaces an existing EMPTY "
                                      "directory, so the rename alone is not a no-clobber "
                                      "primitive; the lock and the re-check immediately before it "
                                      "close that window between materializer instances only.",
                "if_it_pre_exists": "fail closed, removing no lock this run does not own",
                "released": "by this run, after the commit",
            },
            "cleanup_scope": {
                "staging": "recursive removal of the exact staging directory this run created",
                "lock": "unlinked only if this run created it",
                "parent": "non-recursive rmdir, only if this run created it and only if empty",
                "never": "no recursive removal of j7_runs; concurrent content survives",
            },
            "output_root_override": None if not overridden else str(OUTPUT_ROOT_OVERRIDE),
            "authoritative": not overridden,
            "authoritative_note": "an overridden root means this is a TEST materialisation and "
                                  "the leaf is not the authorised one" if overridden
                                  else "written to the authorised leaf",
        },
        "content_hashes": payload["content_hashes"],
        "round_trip_verification": round_trip,
        "schema_of_dataset": report["schema"],
        "composition": report["composition"],
        "ratios": report["ratios"],
        "column_policy": report["column_policy"],
        "nominal_label_identity": report["nominal_label_identity"],
        "penetration_authority": report["penetration_authority"],
        "penetration_per_seed": per_seed,
        "teacher_usage": report["teacher_usage"],
        "teacher_observations_read": report["teacher_usage"]["observations_read"],
        "inert": {"fit_executed": False, "environment_constructed": False,
                  "rollout_executed": False, "critic_touched": False, "ppo_updates": 0,
                  "dataset_recomputed_or_altered": False,
                  "note": "the arrays written are exactly the ones assemble() returned"},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "single_execution": True, "no_autonomous_retry": True},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ preflight (READ-ONLY) ==========

def preflight() -> dict[str, Any]:
    """Verify everything and write NOTHING. No staging, no directory, no file, no rename."""
    auth = verify_authorization()
    payload = build_payload()
    leaf = authorized_leaf()
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    staging = leaf.parent / STAGING_NAME
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    lock_path = leaf.parent / LOCK_NAME
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a materializer lock is already held or was left behind: {lock_path}. "
                        f"This stage removes no lock it does not own.")
    report = payload["report"]
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"leaf_created": False, "staging_created": False, "dataset_written": False,
                  "receipt_written": False, "rename_performed": False, "fit_executed": False,
                  "environment_constructed": False, "rollout_executed": False,
                  "critic_touched": False, "ppo_updates": 0,
                  "note": "the dataset is assembled in memory, audited and discarded"},
        "authorization": auth,
        "preregistration": payload["prereg"],
        "would_write": {
            "leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
            "leaf_exists": bool(leaf.exists()),
            "files": [DATASET_NAME, RECEIPT_NAME],
            "staging": str(staging.name), "lock": str(lock_path.name),
            "lock_held": bool(lock_path.exists()),
            "protocol": "acquire the exclusive sibling lock, stage, round-trip verify, re-check "
                        "the leaf, then commit by atomic rename",
            "protocol_limit": "the lock serialises materializer instances only; os.rename on POSIX "
                              "would replace an empty directory, so nothing is claimed against a "
                              "writer that ignores the lock",
            "output_root_override": None if OUTPUT_ROOT_OVERRIDE is None
            else str(OUTPUT_ROOT_OVERRIDE),
        },
        "payload_audit": {
            "observations_shape": list(EXPECTED_OBS_SHAPE),
            "actions_shape": list(EXPECTED_ACT_SHAPE),
            "dtype": "float32/float32", "actor_feature_names": EXPECTED_NAMES,
            "content_hashes": payload["content_hashes"],
            "content_hashes_match_audited": True,
            "keys": list(DATASET_KEYS),
        },
        "composition": report["composition"], "ratios": report["ratios"],
        "penetration_authority": report["penetration_authority"],
        "teacher_usage": report["teacher_usage"],
        "per_seed": report["per_seed"],
        "requires_to_write": {"flag": "--materialize",
                              "stage_token": STAGE,
                              "out": "must equal the authorised leaf exactly"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ materialisation ================

def materialize(out_arg: str | None, stage_token: str | None) -> dict[str, Any]:
    """Write the two authorised files, once, atomically. Clean up only what we created."""
    validate_stage(stage_token)
    auth = verify_authorization()
    leaf = validate_out(out_arg)
    payload = build_payload()                      # assembled BEFORE anything touches the disk

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise MaterializeError(f"a stale staging directory is in the way: {staging}")

    # Cleanup is scoped to what THIS run created, and to nothing else. A directory that already
    # existed, or content that appeared concurrently inside it, is never removed.
    parent_created: Path | None = None
    staging_created: Path | None = None
    lock_owned: Path | None = None
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent

        # Exclusive lock. It serialises MATERIALIZER instances and claims nothing about a writer
        # that ignores it: os.rename on POSIX would silently replace an empty directory, so the
        # rename alone is not a no-clobber primitive. The lock plus the re-check immediately
        # before the rename close that window between instances of this tool.
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError as exc:
            raise MaterializeError(
                f"the materializer lock already exists: {lock_path}. Another instance holds it, or "
                f"a previous run left it behind. This stage fails closed and removes no lock it "
                f"does not own - inspect it and remove it by hand.") from exc
        lock_owned = lock_path
        try:
            os.write(fd, json.dumps({"stage": STAGE, "pid": os.getpid(),
                                     "leaf": str(leaf)}).encode("utf-8"))
        finally:
            os.close(fd)

        staging.mkdir()
        staging_created = staging

        dataset_path = staging / DATASET_NAME
        np.savez(dataset_path, observations=payload["observations"], actions=payload["actions"],
                 actor_feature_names=payload["actor_feature_names"])
        if not dataset_path.is_file():                                  # np.savez may append .npz
            raise MaterializeError(f"the dataset was not written where expected: {dataset_path}")
        round_trip = _verify_npz(dataset_path, payload, "staged")
        dataset_sha = _sha_file(dataset_path)

        receipt = build_receipt(payload, auth, leaf, dataset_sha, round_trip)
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, sort_keys=False, allow_nan=False, default=str),
            encoding="utf-8")

        written = sorted(p.name for p in staging.iterdir())
        if written != sorted([DATASET_NAME, RECEIPT_NAME]):
            raise MaterializeError(f"the staging holds {written}, expected exactly the two "
                                   f"authorised files")

        # atomic commit, no-clobber under the lock: re-check immediately before the rename
        if leaf.exists() or leaf.is_symlink():
            raise MaterializeError(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None

        # verify from the FINAL location, still holding the lock. On failure the leaf is
        # preserved as evidence and never deleted.
        final_dataset = leaf / DATASET_NAME
        final_round_trip = _verify_npz(final_dataset, payload, "materialised")
        final_sha = _sha_file(final_dataset)
        if final_sha != dataset_sha:
            raise MaterializeError(f"the materialised dataset hashes {final_sha}, the staged one "
                                   f"{dataset_sha}; the leaf is preserved as evidence")
        present = sorted(p.name for p in leaf.iterdir())
        if present != sorted([DATASET_NAME, RECEIPT_NAME]):
            raise MaterializeError(f"the leaf holds {present}, expected exactly the two authorised "
                                   f"files; it is preserved as evidence")
    except BaseException:
        # ONLY the exact staging directory this run created is removed recursively.
        if staging_created is not None and staging_created.name == STAGING_NAME \
                and staging_created.is_dir() and not staging_created.is_symlink():
            shutil.rmtree(staging_created, ignore_errors=True)
        # then our own lock, so the parent can be empty again
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass
            lock_owned = None
        # the parent gets at most a NON-RECURSIVE rmdir, and only if this run created it. If
        # anything appeared inside it concurrently, rmdir fails and that content survives.
        if parent_created is not None:
            try:
                parent_created.rmdir()
            except OSError:
                pass
        raise
    finally:
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass

    return {
        "verdict": "MATERIALIZED", "stage": STAGE, "leaf": _rel(leaf),
        "files": {DATASET_NAME: final_sha,
                  RECEIPT_NAME: _sha_file(leaf / RECEIPT_NAME)},
        "content_hashes": payload["content_hashes"],
        "round_trip_verification": final_round_trip,
        "staging_removed": True, "clobbered_nothing": True,
        "lock_released": not lock_path.exists(),
        "commit": "atomic commit, no-clobber under exclusive materializer lock",
        "commit_limit": "serialises materializer instances only; nothing is claimed against a "
                        "writer that ignores the lock",
        "authoritative": OUTPUT_ROOT_OVERRIDE is None,
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False},
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J7 dataset materialisation")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--materialize", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    a = p.parse_args(argv)
    if a.materialize:
        r = materialize(a.out, a.authorized_stage)
        print(json.dumps(r, indent=2, default=str))
        return 0
    if a.out is not None:
        raise MaterializeError("--out is meaningless without --materialize; the preflight writes "
                               "nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
