"""V26C J20 R2 - the zero-iteration restore audit that closes gate G9.

A MECHANICAL COPY of v26c_j20_restore_audit.py, the R1 runner, which stays
byte-identical at 322b9abc0a2feae7b2e77f7a51946ae19de34e7013901b6e9425e6ee7c5210ae
because the R1 architect GO pins it and the R1 receipt names it. Nothing about
R1 is edited, reused or overwritten.

R1 failed closed at 4/13: every Ray worker died with
"ModuleNotFoundError: No module named 'train_ppo_mlp'" while deserialising the
runtime-env setup hook, so PPOConfig.build_algo never finished and the child
self-guard fired at 600 s. algo.restore_from_path was never reached.

rev5 is the governing amendment: it names R2 honestly as the SECOND CORRECTIVE
ATTEMPT of the audit, separates the direct evidence from the causal inference,
and replaces rev4's find_spec-only smoke check with a real serialise/deserialise
of the actual Ray setup hook. The 600 s timeout is deliberately unchanged.

rev4 authorises exactly ONE correction, and this file carries it: the child
environment gets the ABSOLUTE baseline_MLP directory prepended to PYTHONPATH.
See child_environment(). Six functions differ from the R1 runner and no others:
child_environment (new), launch_once, go_pin_targets, expected_pin_hashes,
check_entry_evidence and run_execution. The test suite proves that mechanically,
function by function.

Preregistered by v26c_j20_prereg_restore_audit.json, amended by rev1 (which
replaced R9 with a live in-process comparison and added R13), rev2 (which
resolved rev1's normalisation contradiction), rev3 (which fixed the token
counts, the wrapper's containment wording and the GO status rule, and tightened
R9), rev4 (which records the observed cause and the single environment
correction) and rev5 (which corrects the semantics and strengthens the smoke
test). All six are pinned by the GO and re-hashed here; precedence is
rev5 > rev4 > rev3 > rev2 > rev1 > base. The gate count is still 13 and no
predicate moved.

R2 is the SECOND CORRECTIVE ATTEMPT of this audit - a second attempt after R1
failed, never an automatic retry. It has its own stage identifier,
V26C_J20_RESTORE_AUDIT_R2, its own preregistration, its own GO and its own
leaf, and it runs exactly once, only after BOTH an explicit user authorisation
and an APPROVED architect GO.

The warm-up leaf carries AWAITING_RESTORE_AUDIT because G9's structural half
passed and its restore half was owed. This stage pays that debt:

  - it reloads checkpoint_last through the REAL production path,
    train_ppo_mlp.run -> algo.restore_from_path, never an emulation of it;
  - it runs ZERO new iterations, because iteration_start is derived as
    restored_logical_iteration + 1 = 2 while the target is 1, so
    range(2, 2) has no members;
  - it reads the LIVE optimizer out of the learner at the instant the restore
    returns, and compares it to the checkpoint byte for byte;
  - it repeats the module and optimizer restore hermetically, out of process,
    as an independent second opinion.

Nothing here trains, samples, rolls out, transplants, promotes or authorises a
next stage. --preflight-only and --dry-run write nothing and import neither
torch nor ray.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
REPO = HERE.parents[3]

STAGE = "V26C_J20_RESTORE_AUDIT_R2"
GO_REQUIRED_STAGE = STAGE

# ---------------------------------------------------------- this stage's files --
PREREG_NAME = "v26c_j20_prereg_restore_audit.json"
PIN_PREREG = "1326944edcbe368319409705e6ff2aeafb62f3def4503f467736ba6780c7d9e6"
PREREG_REV1_NAME = "v26c_j20_prereg_restore_audit_rev1.json"
PIN_PREREG_REV1 = "3fa1ae238cfe0c20865850951ee787cf528b1c17cc0ba6fe97a714d9e37c3ef1"
PREREG_REV2_NAME = "v26c_j20_prereg_restore_audit_rev2.json"
PIN_PREREG_REV2 = "59e2e5e04d167eaafd8283800d277ca8415fac8d57a880818a082dbdbb838cf7"
PREREG_REV3_NAME = "v26c_j20_prereg_restore_audit_rev3.json"
PIN_PREREG_REV3 = "bfb6ef9b4b9a176138adae837e622442e4727f1088c81ea124f3a91b6a7ed41e"
PREREG_REV4_NAME = "v26c_j20_prereg_restore_audit_rev4.json"
PIN_PREREG_REV4 = "71a6118f5555472dfc46be424d60010848ac1a80ba4f4f9c1cb970965db1ec77"
PREREG_REV5_NAME = "v26c_j20_prereg_restore_audit_rev5.json"
PIN_PREREG_REV5 = "3d732153fd51bb7956f91d194fc0f4b1e74c7b35cd451d7a24a0cc70255f675d"
PREREG_PRECEDENCE = ("rev5", "rev4", "rev3", "rev2", "rev1", "base")
GO_REQUIRED_STATUS = "APPROVED"
RUNNER_NAME = "v26c_j20_restore_audit_r2.py"
TEST_NAME = "test_v26c_j20_restore_audit_r2.py"
# The R1 runner and its suite: pinned so R2 cannot run if either moved.
R1_RUNNER_NAME = "v26c_j20_restore_audit.py"
PIN_R1_RUNNER = \
    "322b9abc0a2feae7b2e77f7a51946ae19de34e7013901b6e9425e6ee7c5210ae"
R1_TEST_NAME = "test_v26c_j20_restore_audit.py"
PIN_R1_TEST = \
    "07213d4d13c2ffb0965673ec7960ce8db3153fb9c3393006a77d0aaba0530860"
R1_LEAF_REL = "j20_runs/j20_restore_audit_v26c_2026-08-27_r1"
# The R1 attempt is immutable evidence. Every one of its files is pinned, so
# pins_unchanged - and therefore gate R11 - covers it without R11 being touched.
PIN_R1_LEAF = {
    "TECHNICAL_INVALID":
        "be4447cdad3a637c81e7462d99a2c26c055980428d8796b1e5cd9969f6df51a8",
    "v26c_j20_restore_audit_receipt.json":
        "de481f8d0d905e034b7d0c7c27e924a5bdf9ba21d87bd24e20fd7bf87a2e715f",
    "v26c_j20_restore_audit_result.json":
        "7e069ac65560f2a1e764a80eea1f3d607b476593419dbb326e00b78e41cfb1b0",
    "commit_verification.json":
        "5d850544d384167068b6f6b4a56eb31d821dbd30efe7d0fe3f8813ca7c0b52e4",
    "summary.json":
        "d6d9f8ed2a1e482d98f6fdd507991473a513af7e6e2ca15ff8003401eb5ef0e3",
    "training_cfg.resolved.yaml":
        "976f6c5050161f3edf610de1843c186186fdc56ea9d563cf0fc4c1958c14a810",
    "child_stdout_stderr.txt":
        "67159801e188d00c80b9e0e4ec3aa860d99d73f358e2f8f19f2b096dad106187",
    "watchdog_state.json":
        "a6ad152c3c9e4aa97be4c8ee70a02f47342a86e37983aafe3734d4559cbd8a7e",
}
R1_GO_NAME = "v26c_j20_restore_audit_architect_go.json"
PIN_R1_GO = \
    "29775d4265227a5ef3bfc0fca59313f1ffea56d9c1e34b0236440d2638c1c18a"
CHILD_NAME = "v26c_j20_restore_audit_child.py"

# ------------------------------------------------------- the warm-up under audit --
WARMUP_RUNNER_NAME = "v26c_j20_critic_warmup_execution.py"
PIN_WARMUP_RUNNER = \
    "ce70797ad6362298c63fccceeab127c760d5dd68bfe3e96ef8f92b76f8395114"
WARMUP_PREREG_NAME = "v26c_j20_prereg_critic_warmup_execution.json"
PIN_WARMUP_PREREG = \
    "3ecd96a18a9a6e1d475c823b4fbf37d176fd549cb3c1fe4f57e36518548213f2"
WARMUP_TEST_NAME = "test_v26c_j20_critic_warmup_execution.py"
PIN_WARMUP_TEST = \
    "5e90c3df90e2bdeccde4e1f92abcddcf1cf2595e116f00f1a40b7ea59a3e9990"
WARMUP_GO_NAME = "v26c_j20_critic_warmup_architect_go.json"
PIN_WARMUP_GO = \
    "651927d86fc53f71160c24e070c0177f6e9d3327490ecd3150b818f1296abfdc"

SOURCE_LEAF_REL = "j20_runs/j20_critic_warmup_v26c_2026-08-27_r1"
# The governance and checkpoint files of the leaf under audit. R11 additionally
# re-hashes the WHOLE tree before and after; these are the named anchors.
PIN_SOURCE_LEAF = {
    "v26c_j20_critic_warmup_receipt.json":
        "33b75add03a42d31d116ecb60f442685121adac5cbdf8995ad650d28c46e7294",
    "v26c_j20_critic_warmup_result.json":
        "6fb4200c588641a51f6db97afbe8a6a20b599a064e2eef06941c615c57088a49",
    "commit_verification.json":
        "177657e5da876df66c808e0b46fd4819d52ddee30d260e17a2bd9c783aa97699",
    "summary.json":
        "cc24f24819c686d3add51e8face61f35021eb48682454c8c1081193b7cfe115f",
    "train_iterations.jsonl":
        "667c1c963da00e085283ee297e2988547f4e8b662cb7e139f1ea756fed3562c8",
    "checkpoint_last_meta.json":
        "11c8dfd5a52adad850f63efa838eaa690b067a6af9fbdf54d1504a923ca13291",
    "actor_transplant_report.json":
        "d7ea9efbd1fbec4c847a14575cec85ab2db5cd253716157fbdd1adeeb4f4e11b",
    "training_cfg.resolved.yaml":
        "976f6c5050161f3edf610de1843c186186fdc56ea9d563cf0fc4c1958c14a810",
    "RESTORE_AUDIT_PENDING":
        "4807a882166ccea3ced9e0b088ff01a5253de83f2a2f127b4fb1ed449d581964",
    "checkpoint_last/learner_group/learner/rl_module/default_policy/"
    "module_state.pkl":
        "57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168",
    "checkpoint_last/learner_group/learner/rl_module/default_policy/"
    "class_and_ctor_args.pkl":
        "95c8014de9d609eb9be86630b2b340c43e4d6f49feb796584be9a361dd243180",
    "checkpoint_last/learner_group/learner/state.pkl":
        "51d97ef016aea2086201b55db807f3273b76784bbafdea6cc9e5c7fea90294be",
    "checkpoint_last/rllib_checkpoint.json":
        "80dcbfd86d311932527fe4d8121cd4cf27dc788f23f667e2467b3526967ff8dc",
    "checkpoint_last/algorithm_state.pkl":
        "22e699eddc980c1c0219eac840e4114c593d046195a92e11901de37b4cd11b69",
}
CHECKPOINT_REL = "checkpoint_last"
LEARNER_STATE_REL = CHECKPOINT_REL + "/learner_group/learner/state.pkl"
MODULE_STATE_REL = (CHECKPOINT_REL
                    + "/learner_group/learner/rl_module/default_policy"
                    + "/module_state.pkl")

# ------------------------------------------------------------ expected values --
EXPECTED_ACTOR_DIGEST = \
    "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96"
EXPECTED_CRITIC_DIGEST = \
    "2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410"
EXPECTED_MODULE_STATE_SHA = \
    "57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168"
EXPECTED_MODULE_KEYS = 16
EXPECTED_PARAM_COUNT = 12
EXPECTED_ACTOR_PARAM_INDICES = (0, 1, 2, 3, 4, 5)
EXPECTED_CRITIC_PARAM_INDICES = (6, 7, 8, 9, 10, 11)
EXPECTED_ADAM_STEP = 81.0
EXPECTED_LOGSTD_BIAS = -5.2983174324035645
EXPECTED_SIGMA = 0.005
EXPECTED_RESTORED_ITERATION = 1
EXPECTED_ITERATION_START = 2
EXPECTED_NEXT_ITERATION = 2

# rev3's tightened R9: the live evidence must identify itself and its source.
EVIDENCE_KIND = "LIVE OPTIMIZER RESTORE AUDIT"
EVIDENCE_STAGE_MARKER = "after_restore"
EXPECTED_SOURCE_STATE_SHA = \
    "51d97ef016aea2086201b55db807f3273b76784bbafdea6cc9e5c7fea90294be"
# Read off the warm-up's committed train_iterations.jsonl, not assumed here.
EXPECTED_OPTIMIZER_NAME = "default_optimizer"
EXPECTED_TOP_LEVEL_KEYS = ["param_groups", "state"]

# rev3's three token counts.
EXPECTED_DERIVED_TOKENS = 20
EXPECTED_DELEGATED_ARGV = 18
EXPECTED_CHILD_ARGV = 25

MODULE_CLASS = "AsymmetricActorCriticTorchRLModule"
ACTOR_WIDTH = 35
FULL_WIDTH = 84
ACTION_DIM = 2
HIDDENS = (256, 256)
ACTIVATION = "tanh"

CRITIC_KEYS = ("vf.bias", "vf.weight",
               "vf_encoder.0.bias", "vf_encoder.0.weight",
               "vf_encoder.2.bias", "vf_encoder.2.weight")
WARM_START_ACTOR_KEYS = (
    "pi_encoder.0.weight", "pi.0.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.bias",
    "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias",
)
EXPECTED_PARAM_ORDER = (
    "pi_encoder.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias",
    "pi.1.weight", "pi.1.bias",
    "vf_encoder.0.weight", "vf_encoder.0.bias",
    "vf_encoder.2.weight", "vf_encoder.2.bias",
    "vf.weight", "vf.bias",
)
LOGSTD_ROWS = slice(2, 4)

# ------------------------------------------------------------------- the leaf --
LEAF_ROOT = "j20_runs"
LEAF_NAME = "j20_restore_audit_v26c_2026-08-27_r2"
INVALID_MARKER = "TECHNICAL_INVALID"
PASSED_MARKER = "RESTORE_AUDIT_PASSED"
RESULT_NAME = "v26c_j20_restore_audit_result.json"
RECEIPT_NAME = "v26c_j20_restore_audit_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
CHILD_LOG_NAME = "child_stdout_stderr.txt"
EVIDENCE_NAME = "live_optimizer_audit.json"

VERDICT_PASS = "RESTORE_AUDIT_PASS"
VERDICT_FAILED = "FAIL_CLOSED"

# R10: what training would have left behind and an audit must not.
FORBIDDEN_ARTEFACTS = (
    "checkpoint_last", "checkpoint_best", "checkpoint_last_meta.json",
    "checkpoint_best_meta.json", "rl_module_last", "rl_module_best",
    "rl_module_initial_warm_start", "actor_transplant_report.json",
)
FORBIDDEN_PREFIX = "milestone_iteration_"

INTERPRETER = "/opt/anaconda3/envs/envCMC-rllib/bin/python"

WARM_START_TOKENS_TO_REMOVE = 5


class RestoreAuditError(RuntimeError):
    """Any refusal. The stage fails closed on all of them."""


# ------------------------------------------------------------------ helpers --

def sha256_file(path: pathlib.Path) -> str:
    """sha256 of a file's bytes."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encode_json(payload) -> bytes:
    """Canonical JSON bytes: sorted keys, indent 2, no NaN."""
    return json.dumps(payload, indent=2, sort_keys=True,
                      allow_nan=False).encode("utf-8")


def now_utc() -> str:
    """An ISO-8601 UTC timestamp."""
    import datetime
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def tensor_digest(array) -> str:
    """The project's canonical tensor digest: dtype, shape, C-order bytes."""
    import numpy as np

    contiguous = np.ascontiguousarray(array)
    digest = hashlib.sha256()
    digest.update(str(contiguous.dtype).encode("ascii"))
    digest.update(repr(tuple(int(d) for d in contiguous.shape)).encode("ascii"))
    digest.update(contiguous.tobytes(order="C"))
    return digest.hexdigest()


def group_digest(state, keys) -> str:
    """warm_start's group digest: key utf-8 then the ASCII tensor hexdigest."""
    digest = hashlib.sha256()
    for key in sorted(keys):
        if key not in state:
            raise RestoreAuditError("the state is missing %s" % key)
        digest.update(key.encode("utf-8"))
        digest.update(tensor_digest(state[key]).encode("ascii"))
    return digest.hexdigest()


def bytes_identical(left, right) -> bool:
    """Same dtype, same shape, same C-order bytes. NOT numpy.array_equal."""
    import numpy as np

    a = np.ascontiguousarray(left)
    b = np.ascontiguousarray(right)
    return bool(a.dtype == b.dtype and a.shape == b.shape
                and a.tobytes(order="C") == b.tobytes(order="C"))


def hash_tree(root: pathlib.Path) -> dict:
    """Every file under a directory, relative path -> sha256."""
    out = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            out[str(path.relative_to(root))] = sha256_file(path)
    return out


# --------------------------------------------------------------- the command --

def load_warmup():
    """Import the warm-up execution runner, verifying its bytes first.

    It is imported for its constants and for execution_command(), which itself
    reuses sealed_command() from the readiness runner. Chaining the derivation
    this way means there is exactly ONE source for the J20 invocation and this
    stage cannot silently disagree with it.
    """
    path = HERE / WARMUP_RUNNER_NAME
    actual = sha256_file(path)
    if actual != PIN_WARMUP_RUNNER:
        raise RestoreAuditError(
            "the pinned warm-up runner changed: expected %s, found %s"
            % (PIN_WARMUP_RUNNER, actual))
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    import v26c_j20_critic_warmup_execution as warmup

    return warmup


def restore_command(output_dir: str) -> dict:
    """The warm-up command transformed by FOUR auditable operations.

    Operation 1 - substitute the config - is performed by the warm-up runner's
    own execution_command(). Operations 2, 3 and 4 are performed here:

      2. remove the five warm-start tokens, so summary.json reports
         initialization_mode 'resume_from' rather than 'warm_start_raw' for a
         run that performs no transplant;
      3. append --resume-from <checkpoint_last>, which is the whole stage;
      4. substitute the script, train_ppo_mlp.py -> the validation-only
         wrapper, because R9's live capture has to exist inside the child
         before run() reaches the restore and cannot be installed from outside.

    --iteration-start is deliberately NOT passed, so the trainer derives it from
    the restored checkpoint instead of being told the number it must measure.
    """
    warmup = load_warmup()
    base = warmup.execution_command(output_dir)
    tokens = list(base["tokens"])
    before_count = len(tokens)

    # 2 - remove, verifying the exact window rather than deleting by name
    if "--warm-start-raw" not in tokens:
        raise RestoreAuditError("the warm-up command carries no --warm-start-raw")
    index = tokens.index("--warm-start-raw")
    window = tokens[index:index + WARM_START_TOKENS_TO_REMOVE]
    expected_window = [
        "--warm-start-raw",
        "--warm-start-raw-source", str(HERE / warmup.J19A_MODULE_REL),
        "--warm-start-raw-source-feature-manifest", str(HERE / warmup.OVERLAY_NAME),
    ]
    if window != expected_window:
        raise RestoreAuditError("the warm-start window is %r, expected %r"
                                % (window, expected_window))
    del tokens[index:index + WARM_START_TOKENS_TO_REMOVE]
    for leftover in ("--warm-start-raw", "--warm-start-raw-source",
                     "--warm-start-raw-source-feature-manifest",
                     "--warm-start", "--iteration-start"):
        if leftover in tokens:
            raise RestoreAuditError("%s survived the removal" % leftover)

    # 3 - append the resume path
    checkpoint = HERE / SOURCE_LEAF_REL / CHECKPOINT_REL
    tokens += ["--resume-from", str(checkpoint)]

    # 4 - substitute the script
    script_index = 1
    expected_script = str(BASELINE / "train_ppo_mlp.py")
    if tokens[script_index] != expected_script:
        raise RestoreAuditError("token 1 is %r, expected the trainer %r"
                                % (tokens[script_index], expected_script))
    derived = list(tokens)
    tokens[script_index] = str(HERE / CHILD_NAME)

    delegated = derived[2:]
    child_argv = [
        tokens[0], tokens[script_index],
        "--audit-source-state", str(HERE / SOURCE_LEAF_REL / LEARNER_STATE_REL),
        "--audit-evidence", str(pathlib.Path(output_dir) / EVIDENCE_NAME),
        "--",
    ] + list(delegated)

    # rev3 fixes the three counts and makes each of them a refusal, not a note.
    if len(derived) != EXPECTED_DERIVED_TOKENS:
        raise RestoreAuditError("the derived command holds %d tokens, expected %d"
                                % (len(derived), EXPECTED_DERIVED_TOKENS))
    if len(delegated) != EXPECTED_DELEGATED_ARGV:
        raise RestoreAuditError("the delegated trainer argv holds %d tokens, "
                                "expected %d" % (len(delegated),
                                                 EXPECTED_DELEGATED_ARGV))
    if len(child_argv) != EXPECTED_CHILD_ARGV:
        raise RestoreAuditError("the full child argv holds %d tokens, expected %d"
                                % (len(child_argv), EXPECTED_CHILD_ARGV))
    return {
        "warmup_tokens": tuple(base["tokens"]),
        "warmup_token_count": before_count,
        "derived_tokens": tuple(derived),
        "derived_token_count": len(derived),
        "delegated_argv": tuple(delegated),
        "delegated_argv_count": len(delegated),
        "child_argv": tuple(child_argv),
        "child_argv_count": len(child_argv),
        "operations": {
            "1_substituted_config": {
                "from": base["substituted_from"], "to": base["substituted_to"],
                "performed_by": "execution_command() of " + WARMUP_RUNNER_NAME},
            "2_removed": expected_window,
            "3_appended": ["--resume-from", str(checkpoint)],
            "4_substituted_script": {"from": expected_script,
                                     "to": str(HERE / CHILD_NAME)},
        },
        "source": "sealed_command() -> execution_command() -> restore_command()",
        "counting_note":
            "rev3 settles the three counts: 20 derived command tokens, 18 "
            "delegated to train_ppo_mlp (the derived list minus the "
            "interpreter and the script), 25 in the full child argv (18 + 7). "
            "rev1's note_on_counting called the delegated list 'twenty "
            "tokens', which was inaccurate by two; rev1 was not edited, rev3 "
            "corrects it additively. All three counts are enforced as "
            "refusals above, not merely recorded.",
    }


# ------------------------------------------------------------------- the pins --

def go_pin_targets() -> dict:
    """The CLOSED label -> path map a GO may pin. Paths resolved from constants."""
    warmup = load_warmup()
    targets = {
        PREREG_NAME: HERE / PREREG_NAME,
        PREREG_REV1_NAME: HERE / PREREG_REV1_NAME,
        PREREG_REV2_NAME: HERE / PREREG_REV2_NAME,
        PREREG_REV3_NAME: HERE / PREREG_REV3_NAME,
        PREREG_REV4_NAME: HERE / PREREG_REV4_NAME,
        PREREG_REV5_NAME: HERE / PREREG_REV5_NAME,
        R1_RUNNER_NAME: HERE / R1_RUNNER_NAME,
        R1_TEST_NAME: HERE / R1_TEST_NAME,
        R1_GO_NAME: HERE / R1_GO_NAME,
        RUNNER_NAME: HERE / RUNNER_NAME,
        TEST_NAME: HERE / TEST_NAME,
        CHILD_NAME: HERE / CHILD_NAME,
        WARMUP_RUNNER_NAME: HERE / WARMUP_RUNNER_NAME,
        WARMUP_PREREG_NAME: HERE / WARMUP_PREREG_NAME,
        WARMUP_TEST_NAME: HERE / WARMUP_TEST_NAME,
        WARMUP_GO_NAME: HERE / WARMUP_GO_NAME,
        warmup.CONFIG_NAME: HERE / warmup.CONFIG_NAME,
        warmup.J20_CONFIG_NAME: HERE / warmup.J20_CONFIG_NAME,
        warmup.OVERLAY_NAME: HERE / warmup.OVERLAY_NAME,
        warmup.DERIVER_NAME: HERE / warmup.DERIVER_NAME,
        warmup.READINESS_RUNNER_NAME: HERE / warmup.READINESS_RUNNER_NAME,
        warmup.READINESS_PREREG_NAME: HERE / warmup.READINESS_PREREG_NAME,
        warmup.READINESS_GO_NAME: HERE / warmup.READINESS_GO_NAME,
        warmup.K1R1_PREREG_NAME: HERE / warmup.K1R1_PREREG_NAME,
        warmup.K1R1_RUNNER_NAME: HERE / warmup.K1R1_RUNNER_NAME,
        warmup.K1R1_GO_NAME: HERE / warmup.K1R1_GO_NAME,
        warmup.TELEMETRY_TEST_LABEL: BASELINE / warmup.TELEMETRY_TEST_REL,
        warmup.RUNTIME_CONFIG_LABEL: REPO / warmup.RUNTIME_CONFIG_REL,
    }
    for label, name in warmup.BASELINE_MODULE_LABELS.items():
        targets[label] = BASELINE / name
    for name in warmup.PIN_J19A_LEAF:
        targets[warmup.J19A_LEAF_REL + "/" + name] = \
            HERE / warmup.J19A_LEAF_REL / name
    for name in warmup.PIN_K1_LEAF:
        targets[warmup.K1_LEAF_REL + "/" + name] = HERE / warmup.K1_LEAF_REL / name
    for name in warmup.PIN_K1R1_LEAF:
        targets[warmup.K1R1_LEAF_REL + "/" + name] = \
            HERE / warmup.K1R1_LEAF_REL / name
    for label in list(warmup.PIN_J19B) + list(warmup.PIN_J19C):
        targets[label] = HERE / label
    for name in PIN_SOURCE_LEAF:
        targets[SOURCE_LEAF_REL + "/" + name] = HERE / SOURCE_LEAF_REL / name
    for name in PIN_R1_LEAF:
        targets[R1_LEAF_REL + "/" + name] = HERE / R1_LEAF_REL / name
    return targets


def expected_pin_hashes() -> dict:
    """The pinned hash for every label the map knows."""
    warmup = load_warmup()
    pins = {
        PREREG_NAME: PIN_PREREG,
        PREREG_REV1_NAME: PIN_PREREG_REV1,
        PREREG_REV2_NAME: PIN_PREREG_REV2,
        PREREG_REV3_NAME: PIN_PREREG_REV3,
        PREREG_REV4_NAME: PIN_PREREG_REV4,
        PREREG_REV5_NAME: PIN_PREREG_REV5,
        R1_RUNNER_NAME: PIN_R1_RUNNER,
        R1_TEST_NAME: PIN_R1_TEST,
        R1_GO_NAME: PIN_R1_GO,
        RUNNER_NAME: None,          # this file: pinned by the GO, not by itself
        TEST_NAME: None,            # same
        CHILD_NAME: None,           # same
        WARMUP_RUNNER_NAME: PIN_WARMUP_RUNNER,
        WARMUP_PREREG_NAME: PIN_WARMUP_PREREG,
        WARMUP_TEST_NAME: PIN_WARMUP_TEST,
        WARMUP_GO_NAME: PIN_WARMUP_GO,
        warmup.CONFIG_NAME: warmup.PIN_CONFIG,
        warmup.J20_CONFIG_NAME: warmup.PIN_J20_CONFIG,
        warmup.OVERLAY_NAME: warmup.PIN_OVERLAY,
        warmup.DERIVER_NAME: warmup.PIN_DERIVER,
        warmup.READINESS_RUNNER_NAME: warmup.PIN_READINESS_RUNNER,
        warmup.READINESS_PREREG_NAME: warmup.PIN_READINESS_PREREG,
        warmup.READINESS_GO_NAME: warmup.PIN_READINESS_GO,
        warmup.K1R1_PREREG_NAME: warmup.PIN_K1R1_PREREG,
        warmup.K1R1_RUNNER_NAME: warmup.PIN_K1R1_RUNNER,
        warmup.K1R1_GO_NAME: warmup.PIN_K1R1_GO,
        warmup.TELEMETRY_TEST_LABEL: warmup.PIN_TELEMETRY_TEST,
        warmup.RUNTIME_CONFIG_LABEL: warmup.PIN_RUNTIME_CONFIG,
    }
    for label, name in warmup.BASELINE_MODULE_LABELS.items():
        pins[label] = warmup.BASELINE_PINS[name]
    for name, digest in warmup.PIN_J19A_LEAF.items():
        pins[warmup.J19A_LEAF_REL + "/" + name] = digest
    for name, digest in warmup.PIN_K1_LEAF.items():
        pins[warmup.K1_LEAF_REL + "/" + name] = digest
    for name, digest in warmup.PIN_K1R1_LEAF.items():
        pins[warmup.K1R1_LEAF_REL + "/" + name] = digest
    pins.update(warmup.PIN_J19B)
    pins.update(warmup.PIN_J19C)
    for name, digest in PIN_SOURCE_LEAF.items():
        pins[SOURCE_LEAF_REL + "/" + name] = digest
    for name, digest in PIN_R1_LEAF.items():
        pins[R1_LEAF_REL + "/" + name] = digest
    return pins


# ------------------------------------------------------------------ preflight --

def check_pins() -> list:
    """Every pinned artefact, re-hashed. Pure: reads files, writes nothing."""
    targets = go_pin_targets()
    expected = expected_pin_hashes()
    results = []
    for label in sorted(targets):
        path = targets[label]
        want = expected.get(label)
        if not path.is_file():
            results.append({"artefact": label, "ok": False, "why": "missing",
                            "expected": want, "actual": None})
            continue
        actual = sha256_file(path)
        results.append({
            "artefact": label,
            "ok": want is None or actual == want,
            "why": "present" if want is None else
                   ("matches" if actual == want else "HASH MISMATCH"),
            "expected": want,
            "actual": actual,
        })
    return results


def check_entry_evidence() -> dict:
    """The warm-up must read AWAITING_RESTORE_AUDIT 12/12 and owe exactly G9."""
    problems = []
    found = {}
    result = json.loads((HERE / SOURCE_LEAF_REL
                         / "v26c_j20_critic_warmup_result.json"
                         ).read_text(encoding="utf-8"))
    found["warmup_verdict"] = result.get("verdict")
    found["warmup_gates"] = "%s/%s" % (result.get("gates_passed"),
                                       result.get("gates_total"))
    if result.get("verdict") != "AWAITING_RESTORE_AUDIT":
        problems.append("the warm-up verdict is %r, expected "
                        "AWAITING_RESTORE_AUDIT" % result.get("verdict"))
    if result.get("gates_passed") != 12 or result.get("gates_total") != 12:
        problems.append("the warm-up is not 12/12: %s" % found["warmup_gates"])
    if (result.get("restore_audit") or {}).get("performed") is not False:
        problems.append("the warm-up does not record the restore as unperformed")

    marker = HERE / SOURCE_LEAF_REL / "RESTORE_AUDIT_PENDING"
    found["pending_marker"] = marker.is_file()
    if not marker.is_file():
        problems.append("the source leaf carries no RESTORE_AUDIT_PENDING marker")
    if (HERE / SOURCE_LEAF_REL / "TECHNICAL_INVALID").exists():
        problems.append("the source leaf still carries TECHNICAL_INVALID")

    # R2's own entry condition: R1 must exist, must read FAIL_CLOSED 4/13 and
    # must still carry TECHNICAL_INVALID. If R1 had passed there would be
    # nothing to correct; if it had been tidied away the evidence would be gone.
    r1 = json.loads((HERE / R1_LEAF_REL / "v26c_j20_restore_audit_result.json"
                     ).read_text(encoding="utf-8"))
    found["r1_verdict"] = r1.get("verdict")
    found["r1_gates"] = "%s/%s" % (r1.get("gates_passed"), r1.get("gates_total"))
    if r1.get("verdict") != VERDICT_FAILED or r1.get("gates_passed") != 4:
        problems.append("the R1 record is not the expected FAIL_CLOSED 4/13: %s"
                        % found["r1_gates"])
    if not (HERE / R1_LEAF_REL / INVALID_MARKER).is_file():
        problems.append("the R1 leaf no longer carries %s" % INVALID_MARKER)
    if (HERE / R1_LEAF_REL / PASSED_MARKER).exists():
        problems.append("the R1 leaf carries %s, which it never earned"
                        % PASSED_MARKER)
    found["r1_child_returncode"] = (r1.get("child") or {}).get("returncode")
    if found["r1_child_returncode"] != 124:
        problems.append("the R1 child returncode is %s, expected the 124 "
                        "self-timeout rev4 records"
                        % found["r1_child_returncode"])

    meta = json.loads((HERE / SOURCE_LEAF_REL / "checkpoint_last_meta.json"
                       ).read_text(encoding="utf-8"))
    found["logical_iteration"] = meta.get("logical_iteration")
    found["rllib_training_iteration"] = meta.get("rllib_training_iteration")
    if meta.get("logical_iteration") != EXPECTED_RESTORED_ITERATION \
            or meta.get("rllib_training_iteration") != EXPECTED_RESTORED_ITERATION:
        problems.append("checkpoint_last_meta.json does not read iteration 1: %s"
                        % meta)
    return {"ok": not problems, "problems": problems, "found": found}


def check_destination() -> dict:
    """The destination must be absent."""
    leaf = HERE / LEAF_ROOT / LEAF_NAME
    problems = []
    if leaf.exists():
        problems.append("the destination leaf already exists: %s" % leaf)
    return {"ok": not problems, "problems": problems,
            "leaf": LEAF_ROOT + "/" + LEAF_NAME, "leaf_exists": leaf.exists()}


def preflight(verbose: bool = True) -> dict:
    """Fail-closed readiness check. No torch, no ray, no child, no write."""
    pins = check_pins()
    entry = check_entry_evidence()
    destination = check_destination()
    command = restore_command(str(HERE / LEAF_ROOT / LEAF_NAME))

    problems = [item["artefact"] + ": " + item["why"]
                for item in pins if not item["ok"]]
    problems += entry["problems"] + destination["problems"]

    report = {
        "stage": STAGE,
        "closes": "gate G9 of V26C_J20_CRITIC_WARMUP",
        "ok": not problems,
        "problems": problems,
        "pins": pins,
        "pins_checked": len(pins),
        "pins_matching": sum(1 for item in pins if item["ok"]),
        "entry_evidence": entry,
        "destination": destination,
        "command": {k: v for k, v in command.items()
                    if k not in ("warmup_tokens", "derived_tokens",
                                 "delegated_argv", "child_argv")},
        "gates": 13,
        "best_available_verdict": VERDICT_PASS,
        "it_trains_nothing": True,
    }
    if verbose:
        print("stage              %s" % STAGE)
        print("pins               %d/%d match"
              % (report["pins_matching"], report["pins_checked"]))
        print("entry evidence     %s (warm-up %s, %s)"
              % ("OK" if entry["ok"] else "FAILED",
                 entry["found"].get("warmup_verdict"),
                 entry["found"].get("warmup_gates")))
        print("command            %d derived tokens, %d delegated to the trainer,"
              % (command["derived_token_count"], command["delegated_argv_count"]))
        print("                   %d in the full child argv, 4 operations"
              % command["child_argv_count"])
        print("destination        %s (%s)"
              % (destination["leaf"],
                 "absent" if not destination["leaf_exists"] else "PRESENT"))
        print("gates              13 (R1-R13; rev1 added R13, rev2 fixed "
              "normalisation,")
        print("                   rev3 fixed the counts and tightened R9)")
        print("declared env cost  num_env_runners 13, ray_num_cpus 14; the "
              "Algorithm build")
        print("                   constructs and may reset those environments. "
              "Zero algo.train,")
        print("                   zero sampling, zero rollouts.")
        print("verdict            %s" % ("READY" if report["ok"] else "BLOCKED"))
        for problem in problems:
            print("  problem: %s" % problem, file=sys.stderr)
    return report


# ---------------------------------------------------------------------- the GO --

def validate_go(payload) -> dict:
    """Validate an architect GO. It authorises ONE restore audit, nothing else."""
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"],
                "pins": {}}
    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append("stage is %r, expected %r"
                        % (payload.get("stage"), GO_REQUIRED_STAGE))
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")
    # rev3: status must be EXACTLY "APPROVED". Refusing only the two literals
    # DRAFT and PROPOSED let a missing status, a null, or any other string
    # through, so the rule is now a whitelist of one.
    status = payload.get("status")
    if status != GO_REQUIRED_STATUS:
        problems.append("status is %r, and only the exact string %r authorises "
                        "execution" % (status, GO_REQUIRED_STATUS))
    for forbidden in ("authorises_retry", "authorises_ppo", "authorises_ex_novo",
                      "authorises_promotion", "authorises_training",
                      "authorises_rewriting_the_warmup_leaf"):
        if payload.get(forbidden) is True:
            problems.append("a restore-audit GO must never set %s" % forbidden)
    pins = payload.get("pinned_artefacts_sha256")
    if not isinstance(pins, dict):
        problems.append("pinned_artefacts_sha256 is missing or is not an object")
        pins = {}
    targets = go_pin_targets()
    for required in sorted(targets):
        if required not in pins:
            problems.append("no pin for %s" % required)
    for label in sorted(pins):
        if label not in targets:
            problems.append("pin for %s is outside the authorised scope" % label)
            continue
        path = targets[label]
        if not path.is_file():
            problems.append("pinned artefact %s does not exist at %s"
                            % (label, path))
            continue
        actual = sha256_file(path)
        if actual != pins[label]:
            problems.append("pinned hash for %s is stale: GO says %s, the file "
                            "is %s" % (label, pins[label], actual))
    return {"valid": not problems, "problems": problems, "pins": dict(pins),
            "pin_labels_required": len(targets), "pin_labels_supplied": len(pins)}


def load_go(go_file: str) -> dict:
    """Read and validate a GO file."""
    path = pathlib.Path(go_file)
    if not path.exists():
        return {"valid": False, "pins": {},
                "problems": ["the GO file %s does not exist" % go_file]}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError) as error:
        return {"valid": False, "pins": {},
                "problems": ["the GO file is unreadable: %s" % error]}
    return validate_go(payload)


# ------------------------------------------------------------------ the audit --

def read_child_artefacts(leaf: pathlib.Path) -> dict:
    """Everything the child left behind, read defensively."""
    out = {"summary": None, "summary_error": None, "iteration_rows": None,
           "evidence": None, "evidence_error": None, "residents": [],
           "forbidden_present": [], "supervisor_state_present": False}
    out["residents"] = sorted(p.name for p in leaf.iterdir()) \
        if leaf.is_dir() else []
    try:
        out["summary"] = json.loads((leaf / "summary.json"
                                     ).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        out["summary_error"] = str(error)
    rows_path = leaf / "train_iterations.jsonl"
    if rows_path.is_file():
        out["iteration_rows"] = [line for line
                                 in rows_path.read_text(encoding="utf-8"
                                                        ).splitlines()
                                 if line.strip()]
    else:
        out["iteration_rows"] = []
    try:
        out["evidence"] = json.loads((leaf / EVIDENCE_NAME
                                      ).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        out["evidence_error"] = str(error)
    for name in FORBIDDEN_ARTEFACTS:
        if (leaf / name).exists():
            out["forbidden_present"].append(name)
    for path in leaf.iterdir() if leaf.is_dir() else []:
        if path.name.startswith(FORBIDDEN_PREFIX):
            out["forbidden_present"].append(path.name)
    out["supervisor_state_present"] = (leaf / "supervisor_state.json").exists()
    return out


def hermetic_restore() -> dict:
    """R8 and R13, out of process: no Ray, no environment, no wrapper.

    Builds ONE module at the pinned geometry, loads the sixteen-key state with
    strict=True, then rebuilds a torch Adam over the twelve parameters and
    load_state_dict()s the pickled optimizer state. Every moment tensor is
    compared BYTE for byte against the source, not merely by shape.
    """
    import pickle

    import numpy as np
    import torch
    import gymnasium as gym

    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    from asymmetric_rl_module import AsymmetricActorCriticTorchRLModule

    problems = []
    source = HERE / SOURCE_LEAF_REL
    with (source / MODULE_STATE_REL).open("rb") as handle:
        state = pickle.load(handle)

    torch.manual_seed(0)
    module = AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, (FULL_WIDTH,),
                                         np.float32),
        action_space=gym.spaces.Box(-1.0, 1.0, (ACTION_DIM,), np.float32),
        inference_only=False, learner_only=False, catalog_class=None,
        model_config={"vf_share_layers": False, "n_actor": ACTOR_WIDTH,
                      "n_full": FULL_WIDTH, "fcnet_hiddens": list(HIDDENS),
                      "fcnet_activation": ACTIVATION,
                      "freeze_logstd": False, "freeze_actor": False})
    if type(module).__name__ != MODULE_CLASS:
        problems.append("the module class is %s, expected %s"
                        % (type(module).__name__, MODULE_CLASS))

    missing, unexpected = module.load_state_dict(
        {k: torch.as_tensor(v) for k, v in state.items()}, strict=True)
    if list(missing) or list(unexpected):
        problems.append("strict load left missing=%s unexpected=%s"
                        % (list(missing), list(unexpected)))
    if len(state) != EXPECTED_MODULE_KEYS:
        problems.append("the module state holds %d keys, expected %d"
                        % (len(state), EXPECTED_MODULE_KEYS))

    loaded = {name: parameter for name, parameter in module.named_parameters()}
    buffers = dict(module.state_dict())
    per_key = {}
    for key in sorted(state):
        live = buffers.get(key)
        per_key[key] = live is not None and bytes_identical(
            state[key], live.detach().cpu().numpy())
    if not all(per_key.values()):
        problems.append("these keys are not byte-identical after the load: %s"
                        % sorted(k for k, ok in per_key.items() if not ok))

    actor = group_digest(state, WARM_START_ACTOR_KEYS)
    critic = group_digest(state, CRITIC_KEYS)
    if actor != EXPECTED_ACTOR_DIGEST:
        problems.append("the actor digest is %s, expected %s"
                        % (actor, EXPECTED_ACTOR_DIGEST))
    if critic != EXPECTED_CRITIC_DIGEST:
        problems.append("the critic digest is %s, expected %s"
                        % (critic, EXPECTED_CRITIC_DIGEST))

    weight = np.ascontiguousarray(state["pi.1.weight"])[LOGSTD_ROWS]
    bias = np.ascontiguousarray(state["pi.1.bias"])[LOGSTD_ROWS]
    logstd_weight_zero = bool((weight == 0).all())
    logstd_bias_exact = all(float(v) == EXPECTED_LOGSTD_BIAS for v in bias)
    sigma = [float(np.exp(np.float64(v))) for v in bias]
    sigma_ok = all(abs(s - EXPECTED_SIGMA) < 1e-9 for s in sigma)
    if not logstd_weight_zero:
        problems.append("the log-std weight rows are not exactly zero")
    if not logstd_bias_exact:
        problems.append("the log-std bias rows are not exactly %r"
                        % EXPECTED_LOGSTD_BIAS)
    if not sigma_ok:
        problems.append("sigma is %r, expected %r" % (sigma, EXPECTED_SIGMA))

    order = tuple(name for name, _ in module.named_parameters())
    if order != EXPECTED_PARAM_ORDER:
        problems.append("named_parameters order is %s, expected %s"
                        % (list(order), list(EXPECTED_PARAM_ORDER)))
    actor_names = tuple(order[i] for i in EXPECTED_ACTOR_PARAM_INDICES)
    critic_names = tuple(order[i] for i in EXPECTED_CRITIC_PARAM_INDICES)
    critic_indices_are_the_critic = set(critic_names) == set(CRITIC_KEYS)
    actor_indices_are_not_the_critic = not (set(actor_names) & set(CRITIC_KEYS))
    if not critic_indices_are_the_critic:
        problems.append("indices %s are %s, not the six critic tensors"
                        % (list(EXPECTED_CRITIC_PARAM_INDICES),
                           list(critic_names)))
    if not actor_indices_are_not_the_critic:
        problems.append("indices %s overlap the critic"
                        % list(EXPECTED_ACTOR_PARAM_INDICES))

    with (source / LEARNER_STATE_REL).open("rb") as handle:
        learner_state = pickle.load(handle)
    torch_state = learner_state["optimizer"]["default_policy_default_optimizer"
                                             ]["state"]

    def to_torch(obj):
        if isinstance(obj, dict):
            return {k: to_torch(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [to_torch(v) for v in obj]
        if isinstance(obj, np.ndarray):
            return torch.as_tensor(obj)
        return obj

    optimizer = torch.optim.Adam([p for _, p in module.named_parameters()],
                                 lr=1e-4)
    optimizer_loaded = True
    load_error = None
    try:
        optimizer.load_state_dict(to_torch(torch_state))
    except Exception as error:                          # noqa: BLE001
        optimizer_loaded = False
        load_error = "%s: %s" % (type(error).__name__, error)
        problems.append("the optimizer state did not load: %s" % load_error)

    restored_state = optimizer.state_dict().get("state", {}) \
        if optimizer_loaded else {}
    indices = tuple(sorted(int(k) for k in restored_state))
    if indices != EXPECTED_CRITIC_PARAM_INDICES:
        problems.append("restored Adam indices are %s, expected %s"
                        % (list(indices), list(EXPECTED_CRITIC_PARAM_INDICES)))

    moments_byte_identical = {}
    steps = []
    for index in indices:
        entry = restored_state[index]
        steps.append(float(np.asarray(
            entry["step"].detach().cpu().numpy()
            if hasattr(entry["step"], "detach") else entry["step"]).item()))
        for key in ("exp_avg", "exp_avg_sq"):
            live = entry[key]
            live_array = live.detach().cpu().numpy() \
                if hasattr(live, "detach") else live
            same = bytes_identical(torch_state["state"][index][key], live_array)
            moments_byte_identical["%d.%s" % (index, key)] = same
            shape_ok = tuple(np.shape(live_array)) == \
                tuple(loaded[order[index]].shape)
            if not shape_ok:
                problems.append("%d.%s shape %s does not match parameter %s"
                                % (index, key, tuple(np.shape(live_array)),
                                   tuple(loaded[order[index]].shape)))
    if moments_byte_identical and not all(moments_byte_identical.values()):
        problems.append("these moments are not byte-identical: %s"
                        % sorted(k for k, ok in moments_byte_identical.items()
                                 if not ok))
    if steps and not all(s == EXPECTED_ADAM_STEP for s in steps):
        problems.append("Adam steps are %s, expected all %r"
                        % (steps, EXPECTED_ADAM_STEP))

    return {
        "ok": not problems,
        "problems": problems,
        "module_state_sha256": sha256_file(source / MODULE_STATE_REL),
        "module_state_sha256_expected": EXPECTED_MODULE_STATE_SHA,
        "module_class": type(module).__name__,
        "state_keys": len(state),
        "strict_load_missing": list(missing),
        "strict_load_unexpected": list(unexpected),
        "per_key_byte_identical": per_key,
        "all_keys_byte_identical": all(per_key.values()),
        "actor_digest": actor,
        "critic_digest": critic,
        "logstd_weight_rows_zero": logstd_weight_zero,
        "logstd_bias_exact": logstd_bias_exact,
        "sigma": sigma,
        "sigma_exact": sigma_ok,
        "named_parameter_order": list(order),
        "actor_parameter_names": list(actor_names),
        "critic_parameter_names": list(critic_names),
        "critic_indices_are_the_critic": critic_indices_are_the_critic,
        "actor_indices_are_not_the_critic": actor_indices_are_not_the_critic,
        "optimizer_loaded": optimizer_loaded,
        "optimizer_load_error": load_error,
        "adam_indices": list(indices),
        "adam_steps": steps,
        "moments_byte_identical": moments_byte_identical,
        "all_moments_byte_identical": bool(moments_byte_identical)
        and all(moments_byte_identical.values()),
        "method": "module built at the pinned geometry, sixteen keys loaded "
                  "with strict=True, torch Adam rebuilt over the twelve "
                  "parameters in named_parameters() order, pickled optimizer "
                  "state converted to tensors and loaded; moments compared by "
                  "dtype, shape and C-order bytes",
    }


def evaluate_gates(child: dict, artefacts: dict, hermetic: dict,
                   integrity: dict) -> dict:
    """The thirteen gates of the base preregistration as amended by rev1."""
    summary = artefacts.get("summary") or {}
    evidence = artefacts.get("evidence") or {}
    rows = artefacts.get("iteration_rows") or []
    freeze = summary.get("actor_freeze_audit") or []
    critic = summary.get("critic_state_audit") or []
    lr_audit = summary.get("optimizer_lr_audit") or []
    checkpoint = str(HERE / SOURCE_LEAF_REL / CHECKPOINT_REL)

    gates = {
        "R1_zero_new_iterations":
            len(rows) == 0
            and summary.get("iterations_run") == 0
            and summary.get("iterations_completed_this_process") == 0,
        "R2_empty_history":
            summary.get("history") == [],
        "R3_restored_iteration_is_one":
            summary.get("restored_training_iteration")
            == EXPECTED_RESTORED_ITERATION
            and summary.get("restored_logical_iteration")
            == EXPECTED_RESTORED_ITERATION,
        "R4_next_and_start_are_two":
            summary.get("iteration_start") == EXPECTED_ITERATION_START
            and summary.get("next_iteration") == EXPECTED_NEXT_ITERATION,
        "R5_the_restore_path_was_taken":
            summary.get("resume_from") == checkpoint
            and summary.get("initialization_mode") == "resume_from"
            and not summary.get(
                "warm_start_raw_transplant_applied_this_process", False)
            and "actor_transplant_report.json" not in
            artefacts.get("forbidden_present", []),
        "R6_restored_actor_digest":
            len(freeze) == 1
            and freeze[0].get("stage") == "before_training"
            and freeze[0].get("actor_digest") == EXPECTED_ACTOR_DIGEST,
        "R7_restored_critic_digest":
            len(critic) == 1
            and critic[0].get("stage") == "before_training"
            and critic[0].get("critic_digest") == EXPECTED_CRITIC_DIGEST
            and sorted(critic[0].get("critic_keys") or []) == sorted(CRITIC_KEYS),
        "R8_hermetic_full_module_restore":
            bool(hermetic.get("all_keys_byte_identical"))
            and hermetic.get("state_keys") == EXPECTED_MODULE_KEYS
            and not hermetic.get("strict_load_missing")
            and not hermetic.get("strict_load_unexpected")
            and hermetic.get("actor_digest") == EXPECTED_ACTOR_DIGEST
            and hermetic.get("critic_digest") == EXPECTED_CRITIC_DIGEST
            and bool(hermetic.get("logstd_weight_rows_zero"))
            and bool(hermetic.get("logstd_bias_exact"))
            and bool(hermetic.get("sigma_exact")),
        # rev3 tightened this. A difference_count of zero can be reported by a
        # payload that computed nothing, so the predicate now demands the
        # evidence itself: the identity of the file, the identity of the source
        # it compared against, empty difference LISTS rather than counts, the
        # two whole-tree digests agreeing, and a param_groups verdict of its own.
        "R9_live_optimizer_is_byte_exact_after_restore":
            bool(evidence)
            and evidence.get("kind") == EVIDENCE_KIND
            and evidence.get("stage_marker") == EVIDENCE_STAGE_MARKER
            and evidence.get("source_state_sha256") == EXPECTED_SOURCE_STATE_SHA
            and evidence.get("exact") is True
            and evidence.get("differences") == []
            and evidence.get("difference_count") == 0
            and evidence.get("problems") == []
            and bool(evidence.get("normalised_digest_source"))
            and evidence.get("normalised_digest_live")
            == evidence.get("normalised_digest_source")
            and evidence.get("normalised_digests_match") is True
            and isinstance(evidence.get("param_groups"), dict)
            and evidence["param_groups"].get("exact") is True
            and evidence["param_groups"].get("differences") == []
            and evidence["param_groups"].get("digests_match") is True
            and evidence.get("top_level_keys_source") == EXPECTED_TOP_LEVEL_KEYS
            and evidence.get("top_level_keys_live") == EXPECTED_TOP_LEVEL_KEYS
            and evidence.get("state_indices_source")
            == list(EXPECTED_CRITIC_PARAM_INDICES)
            and evidence.get("state_indices_live")
            == list(EXPECTED_CRITIC_PARAM_INDICES)
            and evidence.get("learner_count") == 1
            and (evidence.get("optimizer_names") or [])
            == [EXPECTED_OPTIMIZER_NAME]
            and any(entry.get("stage") == EVIDENCE_STAGE_MARKER
                    for entry in lr_audit),
        "R10_nothing_was_trained":
            not artefacts.get("forbidden_present"),
        "R11_sources_unchanged":
            bool(integrity.get("source_tree_unchanged"))
            and bool(integrity.get("pins_unchanged")),
        "R12_single_clean_process":
            child.get("returncode") == 0
            and child.get("launched_once") is True
            and child.get("retried") is False
            and summary.get("stop_reason") == "completed"
            and summary.get("interrupted") is False
            and summary.get("timed_out") is False
            and artefacts.get("supervisor_state_present") is False,
        "R13_hermetic_second_opinion_with_byte_identity":
            bool(hermetic.get("ok"))
            and bool(hermetic.get("optimizer_loaded"))
            and bool(hermetic.get("all_moments_byte_identical"))
            and hermetic.get("adam_indices")
            == list(EXPECTED_CRITIC_PARAM_INDICES)
            and bool(hermetic.get("critic_indices_are_the_critic"))
            and bool(hermetic.get("actor_indices_are_not_the_critic"))
            and all(step == EXPECTED_ADAM_STEP
                    for step in (hermetic.get("adam_steps") or [None])),
    }
    failed = sorted(name for name, ok in gates.items() if not ok)
    return {
        "gates": gates,
        "passed": sum(1 for ok in gates.values() if ok),
        "total": len(gates),
        "failed": failed,
        "measurements": {
            "iteration_rows": len(rows),
            "iterations_run": summary.get("iterations_run"),
            "iterations_completed_this_process":
                summary.get("iterations_completed_this_process"),
            "iterations_completed_carried_forward":
                summary.get("iterations_completed"),
            "restored_training_iteration":
                summary.get("restored_training_iteration"),
            "restored_logical_iteration":
                summary.get("restored_logical_iteration"),
            "iteration_start": summary.get("iteration_start"),
            "next_iteration": summary.get("next_iteration"),
            "initialization_mode": summary.get("initialization_mode"),
            "resume_from": summary.get("resume_from"),
            "stop_reason": summary.get("stop_reason"),
            "child_returncode": child.get("returncode"),
            "live_exact": evidence.get("exact"),
            "live_difference_count": evidence.get("difference_count"),
            "live_optimizer_names": evidence.get("optimizer_names"),
            "live_source_state_sha256": evidence.get("source_state_sha256"),
            "live_normalised_digest_source":
                evidence.get("normalised_digest_source"),
            "live_normalised_digest_live":
                evidence.get("normalised_digest_live"),
            "live_normalised_digests_match":
                evidence.get("normalised_digests_match"),
            "live_param_groups_exact":
                (evidence.get("param_groups") or {}).get("exact"),
            "live_param_groups_digests_match":
                (evidence.get("param_groups") or {}).get("digests_match"),
            "live_state_indices": evidence.get("state_indices_live"),
            "hermetic_adam_indices": hermetic.get("adam_indices"),
            "hermetic_adam_steps": hermetic.get("adam_steps"),
            "hermetic_sigma": hermetic.get("sigma"),
            "actor_digest": hermetic.get("actor_digest"),
            "critic_digest": hermetic.get("critic_digest"),
        },
    }


# ------------------------------------------------------------------- execute --

def child_environment() -> dict:
    """The environment for the child. THE SINGLE CORRECTION R2 CARRIES.

    R1 died because every Ray worker raised
    ``ModuleNotFoundError: No module named 'train_ppo_mlp'`` while unpickling
    the runtime-env setup hook. Ray serialises that hook BY REFERENCE to the
    module that defines it, so each worker - a fresh process - has to import
    ``train_ppo_mlp``. Workers rebuild ``sys.path`` from the interpreter and
    from PYTHONPATH; they never see the driver's runtime ``sys.path.insert``.
    Operation 4 of the command derivation substitutes the script with the
    validation-only wrapper, which took ``baseline_MLP`` out of ``sys.path[0]``,
    and nothing put it back.

    So the absolute ``baseline_MLP`` directory is PREPENDED to PYTHONPATH:

      * absolute, derived from this file's resolved ``__file__``, never
        relative and never dependent on the working directory;
      * prepended, so it wins over anything already there;
      * any pre-existing value is preserved VERBATIM after the separator, and
        when there is none no separator is added;
      * joined with ``os.pathsep``, so the same code is correct on macOS and on
        Windows;
      * built on a COPY of ``os.environ`` - this process's own environment is
        never mutated.

    Fail-closed: it refuses to build an environment that points somewhere
    absurd, so a wrong path is an error here rather than 115 dead workers.
    """
    baseline = str(BASELINE)
    if not pathlib.Path(baseline).is_absolute():
        raise RestoreAuditError("the baseline directory %r is not absolute; "
                                "a relative PYTHONPATH entry would be resolved "
                                "against each worker's own working directory"
                                % baseline)
    if not (BASELINE / "train_ppo_mlp.py").is_file():
        raise RestoreAuditError("%s holds no train_ppo_mlp.py, so putting it on "
                                "PYTHONPATH would not make the module "
                                "importable" % baseline)

    environment = dict(os.environ)          # a copy; os.environ is untouched
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    existing = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (baseline + os.pathsep + existing) \
        if existing else baseline
    return environment


def launch_once(tokens, leaf: pathlib.Path) -> dict:
    """Start the child EXACTLY once and wait. No loop, no retry, no supervisor."""
    import subprocess

    started = now_utc()
    environment = child_environment()
    process = subprocess.Popen(
        list(tokens), cwd=str(BASELINE), env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    pid = int(process.pid)
    output, _ = process.communicate()
    finished = now_utc()
    (leaf / CHILD_LOG_NAME).write_text(output or "", encoding="utf-8")
    return {
        "command": list(tokens),
        "cwd": str(BASELINE),
        "pid": pid,
        "started_at": started,
        "finished_at": finished,
        "returncode": int(process.returncode),
        "stdout_stderr_file": CHILD_LOG_NAME,
        "stdout_stderr_sha256": hashlib.sha256(
            (output or "").encode("utf-8")).hexdigest(),
        "stdout_stderr_bytes": len((output or "").encode("utf-8")),
        "launched_once": True,
        "retried": False,
        "supervisor_used": False,
        "pythonpath": environment.get("PYTHONPATH"),
        "pythonpath_first_entry": (environment.get("PYTHONPATH") or "").split(
            os.pathsep)[0],
    }


def verify_commit(leaf: pathlib.Path, files: dict, gates_ok: bool) -> dict:
    """Post-commit verification, then the marker swap. Fail-closed."""
    problems = []
    committed = json.loads((leaf / RECEIPT_NAME).read_text(encoding="utf-8"))
    for name, expected in sorted(committed["artefacts_sha256"].items()):
        path = leaf / name
        if not path.exists():
            problems.append("committed receipt names a missing file: %s" % name)
            continue
        actual = sha256_file(path)
        if actual != expected:
            problems.append("committed %s hashes %s, receipt says %s"
                            % (name, actual, expected))
        if name in files and hashlib.sha256(files[name]).hexdigest() != actual:
            problems.append("committed %s differs from the written bytes" % name)

    targets = go_pin_targets()
    expected_pins = expected_pin_hashes()
    for label in sorted(targets):
        want = expected_pins.get(label)
        if want is None:
            continue
        path = targets[label]
        if not path.is_file() or sha256_file(path) != want:
            problems.append("the pinned artefact %s changed after the run"
                            % label)

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(leaf.relative_to(HERE)),
        "files_verified": len(committed["artefacts_sha256"]),
        "all_gates_passed": gates_ok,
        "problems": problems,
        "ok": not problems and gates_ok,
        "method": "every leaf-relative path named by the COMMITTED receipt is "
                  "re-hashed against it and against the written bytes, and "
                  "every pinned artefact, the source leaf included, is "
                  "re-hashed against its pin",
        "marker": PASSED_MARKER if (not problems and gates_ok)
        else INVALID_MARKER,
    }
    (leaf / COMMIT_VERIFICATION_NAME).write_bytes(encode_json(verification))
    if problems or not gates_ok:
        return verification
    (leaf / PASSED_MARKER).write_bytes(
        b"the zero-iteration restore audit passed all thirteen gates and gate "
        b"G9 of the critic-only warm-up is closed. Nothing here is promoted, "
        b"nothing is training-ready, and no next stage is authorised.\n")
    (leaf / INVALID_MARKER).unlink()
    return verification


def run_execution(go_file: str) -> dict:
    """Launch the child ONCE, audit, commit. No retry, whatever happens."""
    go = load_go(go_file)
    if not go["valid"]:
        raise RestoreAuditError("refusing to execute: the architect GO is "
                                "absent or invalid. %s" % "; ".join(go["problems"]))
    report = preflight(verbose=False)
    if not report["ok"]:
        raise RestoreAuditError("refusing to execute: preflight did not pass: %s"
                                % "; ".join(report["problems"]))

    leaf = HERE / LEAF_ROOT / LEAF_NAME
    if leaf.exists():
        raise RestoreAuditError("refusing to clobber an existing leaf: %s" % leaf)

    leaf.mkdir(parents=True)
    (leaf / INVALID_MARKER).write_bytes(
        b"born invalid; replaced by RESTORE_AUDIT_PASSED only after all "
        b"thirteen gates and post-commit verification pass\n")
    residents = sorted(p.name for p in leaf.iterdir())
    if residents != [INVALID_MARKER]:
        raise RestoreAuditError("the destination is not clean: it holds %s"
                                % residents)

    source = HERE / SOURCE_LEAF_REL
    before_tree = hash_tree(source)
    before_pins = {label: sha256_file(path)
                   for label, path in sorted(go_pin_targets().items())
                   if path.is_file()}

    command = restore_command(str(leaf))
    child = launch_once(command["child_argv"], leaf)

    after_tree = hash_tree(source)
    after_pins = {label: sha256_file(path)
                  for label, path in sorted(go_pin_targets().items())
                  if path.is_file()}
    integrity = {
        "source_tree_files": len(before_tree),
        "source_tree_unchanged": before_tree == after_tree,
        "source_tree_differences": sorted(
            name for name in set(before_tree) | set(after_tree)
            if before_tree.get(name) != after_tree.get(name)),
        "pins_unchanged": before_pins == after_pins,
        "pin_differences": sorted(
            label for label in set(before_pins) | set(after_pins)
            if before_pins.get(label) != after_pins.get(label)),
        "leaf_files": hash_tree(leaf),
    }

    artefacts = read_child_artefacts(leaf)
    try:
        hermetic = hermetic_restore()
    except Exception as error:                          # noqa: BLE001
        hermetic = {"ok": False, "problems": ["the hermetic restore raised: "
                                              "%s: %s" % (type(error).__name__,
                                                          error)]}
    graded = evaluate_gates(child, artefacts, hermetic, integrity)
    gates_ok = not graded["failed"]
    verdict = VERDICT_PASS if gates_ok else VERDICT_FAILED

    result = {
        "kind": "ZERO-ITERATION RESTORE AUDIT RESULT",
        "stage": STAGE,
        "verdict": verdict,
        "closes": "gate G9 of V26C_J20_CRITIC_WARMUP" if gates_ok
        else "nothing; at least one gate failed and there is no retry",
        "gates": graded["gates"],
        "gates_passed": graded["passed"],
        "gates_total": graded["total"],
        "gates_failed": graded["failed"],
        "measurements": graded["measurements"],
        "child": child,
        "command": {k: (list(v) if isinstance(v, tuple) else v)
                    for k, v in command.items()},
        "live_optimizer_audit": artefacts.get("evidence"),
        "hermetic_restore": hermetic,
        "integrity": integrity,
        "entry_evidence": report["entry_evidence"]["found"],
        "inert": {
            "retried": False,
            "child_processes_launched": 1,
            "supervisor_used": False,
            "trained": False,
            "sampled": False,
            "transplanted": False,
            "ppo_ex_novo": False,
            "promotion": "NONE",
        },
        "promotion": "NONE",
        "training_ready": False,
        "next_stage_authorized": False,
        "what_this_still_does_not_authorise": [
            "PPO", "an ex-novo run", "any promotion", "training readiness",
            "any subsequent stage",
        ],
    }

    files = {RESULT_NAME: encode_json(result)}
    receipt = {
        "kind": "RESTORE AUDIT RECEIPT",
        "stage": STAGE,
        "verdict": verdict,
        "gates_passed": graded["passed"],
        "gates_total": graded["total"],
        "gates_failed": graded["failed"],
        "g9_closed": gates_ok,
        "inputs": {
            "prereg_sha256": PIN_PREREG,
            "prereg_rev1_sha256": PIN_PREREG_REV1,
            "prereg_rev2_sha256": PIN_PREREG_REV2,
            "prereg_rev3_sha256": PIN_PREREG_REV3,
            "prereg_rev4_sha256": PIN_PREREG_REV4,
            "prereg_rev5_sha256": PIN_PREREG_REV5,
            "r1_leaf": R1_LEAF_REL,
            "r1_leaf_unchanged": True,
            "prereg_precedence": list(PREREG_PRECEDENCE),
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "child_wrapper_sha256": sha256_file(HERE / CHILD_NAME),
            "source_leaf": SOURCE_LEAF_REL,
            "source_checkpoint_module_state_sha256": EXPECTED_MODULE_STATE_SHA,
            "architect_go_pins": go["pins"],
        },
        "child_returncode": child["returncode"],
        "child_pid": child["pid"],
        "child_started_at": child["started_at"],
        "child_finished_at": child["finished_at"],
        "source_tree_unchanged": integrity["source_tree_unchanged"],
        "pins_unchanged": integrity["pins_unchanged"],
        "inert": result["inert"],
        "promotion": "NONE",
        "training_ready": False,
        "next_stage_authorized": False,
        "artefacts_sha256": {name: hashlib.sha256(payload).hexdigest()
                             for name, payload in sorted(files.items())},
    }
    files[RECEIPT_NAME] = encode_json(receipt)
    for name, payload in sorted(files.items()):
        (leaf / name).write_bytes(payload)

    verification = verify_commit(leaf, files, gates_ok)
    return {"leaf": str(leaf.relative_to(HERE)), "verdict": verdict,
            "gates": "%d/%d" % (graded["passed"], graded["total"]),
            "verification": verification}


# ------------------------------------------------------------------------ CLI --

def build_parser() -> argparse.ArgumentParser:
    """Command-line surface. Preflight is the default and writes nothing."""
    parser = argparse.ArgumentParser(
        description="V26C J20 - zero-iteration restore audit, closes G9")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--go-file", default=None)
    return parser


def main(argv: list | None = None) -> int:
    """Return 0 on success, 1 on failure. Writes nothing unless --execute."""
    args = build_parser().parse_args(argv)
    if args.execute:
        if not args.go_file:
            print("--execute requires --go-file", file=sys.stderr)
            return 1
        try:
            outcome = run_execution(args.go_file)
        except (RestoreAuditError, RuntimeError) as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"], "verdict": outcome["verdict"],
                          "gates": outcome["gates"]}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = preflight(verbose=True)
    if args.dry_run:
        command = restore_command(str(HERE / LEAF_ROOT / LEAF_NAME))
        print("\nplan")
        print("  create the leaf, write %s FIRST, assert it is the only"
              % INVALID_MARKER)
        print("  resident, launch ONE child, audit the live evidence and the")
        print("  hermetic restore, then replace the marker with %s only if"
              % PASSED_MARKER)
        print("  all thirteen gates and post-commit verification pass.")
        print("\n  the child runs ZERO iterations: iteration_start is derived as")
        print("  restored_logical_iteration + 1 = 2 while the target is 1, so")
        print("  range(2, 2) is empty. Nothing trains, samples or rolls out.")
        print("\nthe command, four operations from the warm-up invocation:")
        print("  cd '%s'" % BASELINE)
        print("  PYTHONDONTWRITEBYTECODE=1 " + " ".join(
            '"%s"' % t if " " in t else t for t in command["child_argv"]))
        print("\nthis stage authorises no PPO, no ex-novo and no promotion.")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
