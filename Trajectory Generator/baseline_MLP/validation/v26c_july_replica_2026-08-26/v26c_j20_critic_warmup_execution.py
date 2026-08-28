"""V26C J20 - the critic-only warm-up. Single execution, fail-closed.

This stage launches ONE child process - the sealed warm-up command - and then
audits what it wrote. It trains nothing itself: every gradient step happens
inside the child, under the frozen-actor contract, and this runner only starts
it, watches it, and grades it.

WHAT IT IS NOT. It is not a retry of anything. It performs no restore: the exact
restore audit that gate G9 demands is a SEPARATE stage that this runner cannot
reach. Consequently the best outcome available here is NOT a final PASS but
AWAITING_RESTORE_AUDIT: every immediate gate passed, and one gate is still owed.
Saying PASS would claim evidence that does not exist yet.

THE CONFIG. The child consumes v26c_j20_warmup_critic_only_cfg.yaml, an ADDITIVE
amendment of the J20 config with exactly ONE semantic difference:
supervision.max_minibatch_mean_kl_loss goes from 0.01 to null. The J20 config is
neither modified nor overwritten, and train_ppo_mlp.py is not touched.

Why that one difference is necessary, measured rather than predicted:
_enforce_kl_update_guard (train_ppo_mlp.py:716-773) runs on EVERY iteration and
requires the four kl_update/* metrics; those exist only when the custom learner
is installed, which happens only under the exact-start-sampling contract
(train_ppo_mlp.py:1380-1394). This warm-up must keep exact_start_sampling false,
because that contract requires num_epochs 1 and three start offsets while July's
mechanics are 10 epochs and one start. With the guard enabled and the custom
learner absent, all four metrics are None and 7 of 7 guard checks fail, so the
run would abort at iteration 1. The July warm-ups that succeeded carried no such
guard at all. And in a critic-only warm-up the guard cannot bind on anything:
the actor and the log-std are frozen, the policy ratio is identically 1 and the
KL is identically 0. The direct evidence is kept and gated instead - actor
byte-identical, log-std byte-identical, standard policy mean KL <= 1e-09.

THE COMMAND has ONE source. It is sealed_command() from the readiness runner,
with exactly one token substituted - the --config path - and the substitution is
verified token by token before use. A second, divergent command string would be
a second thing to keep in agreement, and this chain has learned what that costs.

Usage:
    python v26c_j20_critic_warmup_execution.py --preflight-only
    python v26c_j20_critic_warmup_execution.py --dry-run
    python v26c_j20_critic_warmup_execution.py --execute --go-file <architect GO json>

``--preflight-only`` and ``--dry-run`` write nothing, create no directory, launch
no process and never import torch or ray.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import pathlib
import sys

sys.dont_write_bytecode = True

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
REPO = HERE.parents[3]

STAGE = "V26C_J20_CRITIC_WARMUP"
READINESS_STAGE = "V26C_J20_CRITIC_WARMUP_READINESS"

PREREG_NAME = "v26c_j20_prereg_critic_warmup_execution.json"
PIN_PREREG = "3ecd96a18a9a6e1d475c823b4fbf37d176fd549cb3c1fe4f57e36518548213f2"
RUNNER_NAME = "v26c_j20_critic_warmup_execution.py"
TEST_NAME = "test_v26c_j20_critic_warmup_execution.py"

# --------------------------------------------------------------------------
# The readiness stage this one executes. Imported for ONE reason: sealed_command.
# --------------------------------------------------------------------------
READINESS_RUNNER_NAME = "v26c_j20_critic_warmup_readiness.py"
PIN_READINESS_RUNNER = \
    "d33e0709ca4682ef857e07a85c7015d2d857aba2b04f454afb3bc1aa0c57858b"
READINESS_PREREG_NAME = "v26c_j20_prereg_critic_warmup_readiness.json"
PIN_READINESS_PREREG = \
    "f4ec06d77641a460e26df554f4c5c0f10ebceacb4534bbd0368773294706130c"
READINESS_GO_NAME = "v26c_j20_architect_go_k1.json"
PIN_READINESS_GO = \
    "aca75c7c12585b5b1ee50c52ebabf3030facf8c83e4ece668e70e421cd3f44f0"
DERIVER_NAME = "v26c_j20_derive_warmup_config.py"
PIN_DERIVER = "f21654ce315827e11bf3a7a8c48331454b71a0c9f439c75454f9652ede81aaaf"

# --------------------------------------------------------------------------
# The two configs. The J20 one is READ, never written; the critic-only one is
# what the child consumes.
# --------------------------------------------------------------------------
J20_CONFIG_NAME = "v26c_j20_warmup_cfg.yaml"
PIN_J20_CONFIG = "d8bcfd39f6015eaf7a41deda2edec201e9dbe89df6c425a67f3bf14bc5e4f525"
CONFIG_NAME = "v26c_j20_warmup_critic_only_cfg.yaml"
PIN_CONFIG = "e98bde4eb3b1d339d4f94dd5b0883d874db0c8971c19adb55404026584bf0c20"
OVERLAY_NAME = "v26c_j20_actor_feature_manifest_overlay.json"
PIN_OVERLAY = "b0f9035413be0a56216e391cd27c4282c3cb708b7733a67ad32e506d281d39ca"

# The ONE authorised semantic difference between the two configs.
CONFIG_DIFF_SECTION = "supervision"
CONFIG_DIFF_KEY = "max_minibatch_mean_kl_loss"
CONFIG_DIFF_FROM = 0.01
CONFIG_DIFF_TO = None
CONFIG_SECTIONS_MUST_BE_IDENTICAL = (
    "model", "ppo", "parallelism", "simulation", "grf", "logging", "reward")

# --------------------------------------------------------------------------
# The actor. Read, never edited.
# --------------------------------------------------------------------------
J19A_LEAF_REL = "j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1"
J19A_MODULE_REL = J19A_LEAF_REL + "/rl_module"
PIN_J19A_LEAF = {
    "commit_verification.json":
        "4d526dc119a10983e0186257132fbff58569d90ab0d1a793e0afae5ba2f32c61",
    "history.json":
        "0a00cb6be58945d525201f76aa425ad713addbd4946d108e02cf4bc2c4111af0",
    "rl_module/actor_feature_manifest.json":
        "2c01067e9a569354cc4099537a3a556ab50d55aa8baa1d2127324dafeee27c54",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/module_state.pkl":
        "8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530",
    "v26c_j19a_result.json":
        "8982b2fc40e514bed903af0c33e0a4ab3737ebf849a44a232912a816002dc254",
    "v26c_j19a_single_reproduction_receipt.json":
        "235a117fc849bbe137dfd7ea29621390a6d1aa71aa9f9b4d95ca0e7a5dd50dad",
}
PIN_J19A_MODULE_STATE = PIN_J19A_LEAF["rl_module/module_state.pkl"]
PIN_J19A_ACTOR_DIGEST = \
    "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96"

# Entry evidence, all four required to be intact and to read what they read.
PIN_J19B = {
    "j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1/v26c_j19b_closed_loop_receipt.json":
        "789c57ca293ae7d5f578d4b3cae4e9aec599fa3edf4ce87239ad5fc8a7b1b509",
    "j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1/commit_verification.json":
        "0ded736553a55eba6ac600340750d40aa82f7ebf4689bcdd4717dc3636907776",
}
PIN_J19C = {
    "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/v26c_j19c_heldout_g_i_receipt.json":
        "4224f0201eeccc8ccec911635b5bc18a82e38fb27eeb878457d004d28cb17ffc",
    "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/commit_verification.json":
        "11096edec7e259a2fe3f6dd2691c26cee0ce0e82b96864bd85a52a00a9e6ef2f",
}
K1_LEAF_REL = "j20_runs/j20_critic_warmup_readiness_v26c_2026-08-27_r1"
PIN_K1_LEAF = {
    "commit_verification.json":
        "06b60b92af40b852cdbff77551421189c2235f8b64165d972f7e4784e6eeb063",
    "probe_actor_transplant_report.json":
        "73bb4c78f9e3910d552308250bf6dda29028bf6182a9760d0965db3c45ff0f80",
    "probe_full_module_state.pkl":
        "b5aef9b22a60ce664e8d5fc9415621448b24cd90fe7c2a27fc732f5b275edcd2",
    "probe_module_manifest.json":
        "eab14deedfe88449bac9ab37f5f56959d3213a3d0830ce7f22b71bb3aedfceaa",
    "v26c_j20_critic_warmup_readiness_receipt.json":
        "8d2a4ba9a8e286194eb239665b0d1d71179cf52160b4bed93a4a99cddb18d128",
    "v26c_j20_probe_result.json":
        "51b5f999f22fbd937b754159fc7a44f2fc3efa72463ea137154829b9d71d2452",
}
K1R1_LEAF_REL = "j20_runs/j20_k1r1_nondegenerate_gradient_v26c_2026-08-27_r1"
PIN_K1R1_LEAF = {
    "commit_verification.json":
        "8a2718089ecbb79f19d2f6554c51173925353b7fd01ea7193c8e0c346ffb63be",
    "k1r1_stimulus.npy":
        "0a246c04524d31ac1c0e6fbb418bc1d1c1c75a82eb38c8c21248297d32b89591",
    "k1r1_stimulus_manifest.json":
        "a06967633a2075c11762aca98a50b2e1a22a973ed0c98f0ab8af6f7ea4b370e1",
    "v26c_j20_k1r1_gradient_amendment_receipt.json":
        "25b499764f371e5822b5cfd58faf4ebccf920cc2c6e18f5c479b7b14eb737d06",
    "v26c_j20_k1r1_result.json":
        "46fd00b1e4dd5cb824b30edb1e03a35e5c6990d3aca89701251439cdbb44a3bc",
}
K1R1_PREREG_NAME = "v26c_j20_k1r1_prereg_nondegenerate_gradient_amendment.json"
PIN_K1R1_PREREG = \
    "d770477cf31988d0f464b157a258b42fa16a1bd8de03c90d24dc8767fb9fbac5"
K1R1_RUNNER_NAME = "v26c_j20_k1r1_gradient_amendment.py"
PIN_K1R1_RUNNER = \
    "572e17e7c3e6599eb3c3a325b4f4b525ca71e8f13bcfc1c6a4d6dca22287adb9"
K1R1_GO_NAME = "v26c_j20_k1r1_architect_go.json"
PIN_K1R1_GO = "5323b448c0ee37e7c15175745fd2306cd4beeccfbf7a4404cd1896018a4bf07d"

# Production code the CHILD executes.
BASELINE_PINS = {
    "train_ppo_mlp.py":
        "cbd278c39c4fe9efcfb52b7a95eb3e85f24020eb6aedd1892f7832b1472be375",
    "warm_start.py":
        "84706218dcc4c5cb7f97a8f3f67ef40ba9e064ba5aef25cb6559d1c8a506c34c",
    "asymmetric_rl_module.py":
        "5084786c8e6312de2d37744bf327b907ed52ff92cc6e3686b36bd1bde6d21a0f",
    "tb_logging.py":
        "b5543e286410aa9476b5e91614b3d1028516696f1a5c743ec858d1e2c609ce26",
}
TELEMETRY_TEST_REL = "validation/test_training_health_telemetry.py"
PIN_TELEMETRY_TEST = \
    "608cbe0ac927506553bee52b6a1eeb815d8d288f61a4a67f6410fbebc486206b"
RUNTIME_CONFIG_REL = (
    "Trajectory Generator/runs/training/"
    "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml")
PIN_RUNTIME_CONFIG = \
    "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"

# --------------------------------------------------------------------------
# The contract the child must honour.
# --------------------------------------------------------------------------
EXPECTED_ITERATIONS = 1
EXPECTED_SAMPLED_STEPS = 4096.0
SIGMA = 0.005
KL_TOLERANCE = 1e-09
LOGSTD_ROWS = slice(2, 4)
ACTOR_KEYS = (
    "pi.0.0.bias", "pi.0.0.weight", "pi.0.2.bias", "pi.0.2.weight",
    "pi.1.bias", "pi.1.weight",
    "pi_encoder.0.bias", "pi_encoder.0.weight",
    "pi_encoder.2.bias", "pi_encoder.2.weight",
)
CRITIC_KEYS = (
    "vf.bias", "vf.weight",
    "vf_encoder.0.bias", "vf_encoder.0.weight",
    "vf_encoder.2.bias", "vf_encoder.2.weight",
)
WARM_START_ACTOR_KEYS = (
    "pi_encoder.0.weight", "pi.0.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.bias",
    "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias",
)
TRAINING_HEALTH_FIELDS = (
    "observed_rows", "missing_telemetry_rows", "phase_timeout_stance_rows",
    "morphology_causal_contract_failure_rows", "resync_event_rows",
    "resync_count_max", "hs_cancelled_count_max",
    "timeout_side_disagreement_rows",
)
# The four the architect wants RECORDED without a behavioural bound.
TRAINING_HEALTH_INCIDENT_FIELDS = (
    "phase_timeout_stance_rows", "morphology_causal_contract_failure_rows",
    "resync_event_rows", "resync_count_max", "hs_cancelled_count_max",
)

# --------------------------------------------------------------------------
# The real checkpoint layout, READ FROM COMMITTED JULY ARTEFACTS, not assumed.
# Two of these paths are traps: learner_group/state.pkl and
# learner_group/learner/rl_module/module_state.pkl are five-byte EMPTY dicts.
# The real optimizer and the real sixteen-key state live one level deeper.
# --------------------------------------------------------------------------
CHECKPOINT_DIR = "checkpoint_last"
OPTIMIZER_REL = "learner_group/learner/state.pkl"
LEARNER_MODULE_REL = "learner_group/learner/rl_module/default_policy/module_state.pkl"
EMPTY_DECOY_PATHS = (
    "learner_group/state.pkl",
    "learner_group/learner/rl_module/module_state.pkl",
)
OPTIMIZER_TOP_KEYS = ("metrics_logger", "optimizer", "should_module_be_updated",
                      "weights_seq_no")
OPTIMIZER_ENTRY = "default_policy_default_optimizer"
EXPECTED_PARAM_GROUP_SIZE = 12
EXPECTED_ADAM_STATE_ENTRIES = 6
# Which twelve parameters may carry Adam moments: ONLY the six critic tensors.
# named_parameters() orders the actor first (indices 0-5, the six unique actor
# tensors) and the value tower second (6-11). A frozen actor takes no gradient,
# so it can never acquire a moment - and an entry at any index below 6 would mean
# the freeze leaked. Read from July's own optimizer, not assumed.
EXPECTED_ADAM_STATE_INDICES = (6, 7, 8, 9, 10, 11)
MILESTONE_DIR = "milestone_iteration_000001"

# Metric keys, literal, as they appear in the JSONL row's learner_metrics blob.
METRIC_MEAN_KL = "learners/default_policy/mean_kl_loss"
METRIC_VF_LOSS = "learners/default_policy/vf_loss"
METRIC_VF_EV = "learners/default_policy/vf_explained_var"
# The PER-ITERATION sampled-steps figure the trainer actually emits. It is not
# the lifetime counter: in the August five-iteration run this key reads 4608 on
# every iteration while num_env_steps_sampled_lifetime climbs 4608, 9216, 13824,
# 18432, 23040. G2 checks BOTH, because on a single fresh iteration they happen
# to coincide and a check that saw only the lifetime would not notice if they
# ever stopped coinciding.
METRIC_STEPS_THIS_ITERATION = (
    "learners/__all_modules__/learner_connector_sum_episodes_length_in")

# --------------------------------------------------------------------------
# The only destination.
# --------------------------------------------------------------------------
LEAF_ROOT = "j20_runs"
LEAF_NAME = "j20_critic_warmup_v26c_2026-08-27_r1"
RECEIPT_NAME = "v26c_j20_critic_warmup_receipt.json"
RESULT_NAME = "v26c_j20_critic_warmup_result.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
CHILD_LOG_NAME = "child_stdout_stderr.txt"
INVALID_MARKER = "TECHNICAL_INVALID"
PENDING_MARKER = "RESTORE_AUDIT_PENDING"

VERDICT_AWAITING = "AWAITING_RESTORE_AUDIT"
VERDICT_FAILED = "FAIL_CLOSED"

GO_REQUIRED_STAGE = STAGE
BASELINE_MODULE_LABELS = {
    "baseline_MLP/train_ppo_mlp.py": "train_ppo_mlp.py",
    "baseline_MLP/warm_start.py": "warm_start.py",
    "baseline_MLP/asymmetric_rl_module.py": "asymmetric_rl_module.py",
    "baseline_MLP/tb_logging.py": "tb_logging.py",
}
TELEMETRY_TEST_LABEL = "baseline_MLP/validation/test_training_health_telemetry.py"
RUNTIME_CONFIG_LABEL = "runs/training/MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml"

# Calls this file must never make. It starts ONE child and audits files; it does
# not train, sample, build an environment, start Ray or restore anything.
FORBIDDEN_CALLS = (
    "train", "build_algo", "PPOConfig", "make_cmc_env", "make_env",
    "save_to_path", "restore_from_path", "from_checkpoint", "sync_weights",
    "backward", "step", "sample", "call", "check_output", "system",
)
FORBIDDEN_IMPORTS = (
    "torch", "ray", "gymnasium", "torch.optim", "env_factory", "rollout_eval",
    "train_ppo_mlp", "warm_start", "asymmetric_rl_module",
)


class WarmupError(RuntimeError):
    """The stage refused to proceed, or the run failed its gates."""


# ------------------------------------------------------------------ helpers --

def sha256_file(path: pathlib.Path) -> str:
    """SHA-256 of a file's bytes."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def encode_json(payload) -> bytes:
    """Deterministic JSON bytes; non-finite values are refused."""
    return json.dumps(payload, indent=2, sort_keys=True,
                      allow_nan=False).encode("utf-8")


def now_utc() -> str:
    """An ISO-8601 UTC timestamp."""
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


def actor_digest(state) -> str:
    """warm_start.actor_state_digest, transcribed. Must reproduce d4a13ff7..."""
    digest = hashlib.sha256()
    for key in sorted(WARM_START_ACTOR_KEYS):
        if key not in state:
            raise WarmupError("actor state is missing %s" % key)
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


def finite(value) -> bool:
    """True only for a real, finite number. None and NaN are not."""
    import math

    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


# --------------------------------------------------------------- the command --

def load_readiness():
    """Import the readiness runner, verifying its bytes first.

    It is imported for ONE symbol, sealed_command. Depending on code a GO does
    not pin would make this stage's behaviour hostage to a file nobody checked.
    """
    path = HERE / READINESS_RUNNER_NAME
    actual = sha256_file(path)
    if actual != PIN_READINESS_RUNNER:
        raise WarmupError(
            "the pinned readiness runner changed: expected %s, found %s"
            % (PIN_READINESS_RUNNER, actual))
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    import v26c_j20_critic_warmup_readiness as readiness

    return readiness


def execution_command(output_dir: str) -> dict:
    """The sealed command with EXACTLY ONE token substituted: the config.

    There is one source for this command - sealed_command() in the readiness
    runner - and this function does not restate it. It takes that tuple,
    verifies the token it is about to replace really is the J20 config path,
    replaces it with the critic-only config, and returns both tuples so the
    substitution is auditable rather than asserted.
    """
    readiness = load_readiness()
    sealed = tuple(readiness.sealed_command(output_dir))
    if "--config" not in sealed:
        raise WarmupError("the sealed command carries no --config token")
    index = sealed.index("--config") + 1
    expected = str(HERE / J20_CONFIG_NAME)
    if sealed[index] != expected:
        raise WarmupError("the sealed command's config is %r, expected %r"
                          % (sealed[index], expected))
    tokens = list(sealed)
    tokens[index] = str(HERE / CONFIG_NAME)
    differing = [i for i, (a, b) in enumerate(zip(sealed, tokens)) if a != b]
    if differing != [index] or len(tokens) != len(sealed):
        raise WarmupError("the substitution touched %s, expected only the "
                          "config token at %d" % (differing, index))
    return {
        "tokens": tuple(tokens),
        "sealed_tokens": sealed,
        "substituted_index": index,
        "substituted_from": sealed[index],
        "substituted_to": tokens[index],
        "token_count": len(tokens),
        "source": "sealed_command() of " + READINESS_RUNNER_NAME,
        "differences_from_sealed": 1,
    }


# ------------------------------------------------------------------- the pins --

def go_pin_targets() -> dict:
    """The CLOSED label -> path map a GO may pin.

    Paths are resolved HERE, from constants, never from the GO payload. Labels
    outside this directory - the production modules, the telemetry suite, the
    pinned runtime config - are resolved from internal constants, which is the
    reason the map exists at all.
    """
    targets = {
        PREREG_NAME: HERE / PREREG_NAME,
        RUNNER_NAME: HERE / RUNNER_NAME,
        TEST_NAME: HERE / TEST_NAME,
        CONFIG_NAME: HERE / CONFIG_NAME,
        J20_CONFIG_NAME: HERE / J20_CONFIG_NAME,
        OVERLAY_NAME: HERE / OVERLAY_NAME,
        DERIVER_NAME: HERE / DERIVER_NAME,
        READINESS_RUNNER_NAME: HERE / READINESS_RUNNER_NAME,
        READINESS_PREREG_NAME: HERE / READINESS_PREREG_NAME,
        READINESS_GO_NAME: HERE / READINESS_GO_NAME,
        K1R1_PREREG_NAME: HERE / K1R1_PREREG_NAME,
        K1R1_RUNNER_NAME: HERE / K1R1_RUNNER_NAME,
        K1R1_GO_NAME: HERE / K1R1_GO_NAME,
        TELEMETRY_TEST_LABEL: BASELINE / TELEMETRY_TEST_REL,
        RUNTIME_CONFIG_LABEL: REPO / RUNTIME_CONFIG_REL,
    }
    for label, name in BASELINE_MODULE_LABELS.items():
        targets[label] = BASELINE / name
    for name in PIN_J19A_LEAF:
        targets[J19A_LEAF_REL + "/" + name] = HERE / J19A_LEAF_REL / name
    for name in PIN_K1_LEAF:
        targets[K1_LEAF_REL + "/" + name] = HERE / K1_LEAF_REL / name
    for name in PIN_K1R1_LEAF:
        targets[K1R1_LEAF_REL + "/" + name] = HERE / K1R1_LEAF_REL / name
    for label in list(PIN_J19B) + list(PIN_J19C):
        targets[label] = HERE / label
    return targets


GO_REQUIRED_PINS = tuple(sorted(go_pin_targets()))


def expected_pin_hashes() -> dict:
    """The pinned hash for every label the map knows, from this file's constants."""
    pins = {
        PREREG_NAME: PIN_PREREG,
        RUNNER_NAME: None,          # this file: pinned by the GO, not by itself
        TEST_NAME: None,            # same
        CONFIG_NAME: PIN_CONFIG,
        J20_CONFIG_NAME: PIN_J20_CONFIG,
        OVERLAY_NAME: PIN_OVERLAY,
        DERIVER_NAME: PIN_DERIVER,
        READINESS_RUNNER_NAME: PIN_READINESS_RUNNER,
        READINESS_PREREG_NAME: PIN_READINESS_PREREG,
        READINESS_GO_NAME: PIN_READINESS_GO,
        K1R1_PREREG_NAME: PIN_K1R1_PREREG,
        K1R1_RUNNER_NAME: PIN_K1R1_RUNNER,
        K1R1_GO_NAME: PIN_K1R1_GO,
        TELEMETRY_TEST_LABEL: PIN_TELEMETRY_TEST,
        RUNTIME_CONFIG_LABEL: PIN_RUNTIME_CONFIG,
    }
    for label, name in BASELINE_MODULE_LABELS.items():
        pins[label] = BASELINE_PINS[name]
    for name, digest in PIN_J19A_LEAF.items():
        pins[J19A_LEAF_REL + "/" + name] = digest
    for name, digest in PIN_K1_LEAF.items():
        pins[K1_LEAF_REL + "/" + name] = digest
    for name, digest in PIN_K1R1_LEAF.items():
        pins[K1R1_LEAF_REL + "/" + name] = digest
    pins.update(PIN_J19B)
    pins.update(PIN_J19C)
    return pins


# ------------------------------------------------------------------ preflight --

def check_pins() -> list[dict]:
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


def check_config_amendment() -> dict:
    """The critic-only config differs from the J20 config in EXACTLY one key."""
    import yaml

    base = yaml.safe_load((HERE / J20_CONFIG_NAME).read_text(encoding="utf-8"))
    amended = yaml.safe_load((HERE / CONFIG_NAME).read_text(encoding="utf-8"))
    differences = []
    for section in sorted(set(base) | set(amended)):
        left = base.get(section) or {}
        right = amended.get(section) or {}
        for key in sorted(set(left) | set(right)):
            if left.get(key) != right.get(key):
                differences.append({"section": section, "key": key,
                                    "from": left.get(key), "to": right.get(key)})
    problems = []
    if len(differences) != 1:
        problems.append("the amended config differs in %d keys, expected exactly "
                        "one: %s" % (len(differences), differences))
    elif (differences[0]["section"] != CONFIG_DIFF_SECTION
          or differences[0]["key"] != CONFIG_DIFF_KEY
          or differences[0]["from"] != CONFIG_DIFF_FROM
          or differences[0]["to"] is not CONFIG_DIFF_TO):
        problems.append("the single difference is %s, expected %s.%s %r -> None"
                        % (differences[0], CONFIG_DIFF_SECTION, CONFIG_DIFF_KEY,
                           CONFIG_DIFF_FROM))
    for section in CONFIG_SECTIONS_MUST_BE_IDENTICAL:
        if base.get(section) != amended.get(section):
            problems.append("section %r is not identical between the two configs"
                            % section)
    guard = (amended.get("supervision") or {}).get(CONFIG_DIFF_KEY, "absent")
    if guard is not None:
        problems.append("the amended config still enables the KL update guard: %r"
                        % guard)
    exact = (amended.get("parallelism") or {}).get("exact_start_sampling")
    if exact is not False:
        problems.append("exact_start_sampling must stay false, it is %r" % exact)
    return {
        "ok": not problems,
        "problems": problems,
        "semantic_differences": differences,
        "difference_count": len(differences),
        "sections_identical": {s: base.get(s) == amended.get(s)
                               for s in CONFIG_SECTIONS_MUST_BE_IDENTICAL},
        "guard_disabled": guard is None,
        "exact_start_sampling": exact,
        "why_the_guard_must_be_off":
            "_enforce_kl_update_guard runs every iteration and needs the four "
            "kl_update/* metrics; those exist only when the custom learner is "
            "installed, which happens only under exact-start sampling. With the "
            "guard on and that learner absent, 7 of 7 checks fail and the child "
            "aborts at iteration 1. Measured by executing the guard as a pure "
            "function on the stock learner's metric set.",
        "what_replaces_it":
            "the direct evidence, all gated here: actor byte-identical, log-std "
            "byte-identical, and the standard policy mean KL within 1e-09",
    }


def check_entry_evidence() -> dict:
    """The qualification and probe evidence must read what it is supposed to read."""
    problems = []
    found = {}
    for label, expected_verdict in (
            ("j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1/"
             "v26c_j19b_closed_loop_receipt.json", "PASS"),
            ("j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/"
             "v26c_j19c_heldout_g_i_receipt.json", "PASS")):
        payload = json.loads((HERE / label).read_text(encoding="utf-8"))
        found[label] = payload.get("verdict")
        if payload.get("verdict") != expected_verdict:
            problems.append("%s reads %r, expected %r"
                            % (label, payload.get("verdict"), expected_verdict))

    k1 = json.loads((HERE / K1_LEAF_REL
                     / "v26c_j20_probe_result.json").read_text(encoding="utf-8"))
    found["k1_verdict"] = k1.get("verdict")
    found["k1_checks"] = "%s/%s" % (k1.get("checks_passed"), k1.get("checks_total"))
    if k1.get("verdict") != "FAIL_CLOSED" or k1.get("checks_passed") != 28:
        problems.append("the K1 record is not the expected FAIL_CLOSED 28/29")

    k1r1 = json.loads((HERE / K1R1_LEAF_REL
                       / "v26c_j20_k1r1_result.json").read_text(encoding="utf-8"))
    found["k1r1_verdict"] = k1r1.get("verdict")
    composite = k1r1.get("composite") or {}
    found["k1r1_composite"] = "%s/%s" % (composite.get("composite_passed"),
                                         composite.get("composite_total"))
    if k1r1.get("verdict") != "PASS" or composite.get("composite_passed") != 29:
        problems.append("the K1R1 amendment does not read PASS 29/29")
    return {"ok": not problems, "problems": problems, "found": found}


def check_destination() -> dict:
    """The destination must be absent, and j20_runs must hold only K1 and K1R1."""
    leaf = HERE / LEAF_ROOT / LEAF_NAME
    contents = sorted(p.name for p in (HERE / LEAF_ROOT).iterdir()) \
        if (HERE / LEAF_ROOT).is_dir() else []
    expected = sorted([K1_LEAF_REL.split("/")[-1], K1R1_LEAF_REL.split("/")[-1]])
    problems = []
    if leaf.exists():
        problems.append("the destination leaf already exists: %s" % leaf)
    if contents != expected:
        problems.append("j20_runs holds %s, expected %s" % (contents, expected))
    return {"ok": not problems, "problems": problems,
            "leaf": LEAF_ROOT + "/" + LEAF_NAME,
            "leaf_exists": leaf.exists(), "j20_runs_contents": contents}


def preflight(verbose: bool = True) -> dict:
    """Fail-closed readiness check. No torch, no ray, no child, no write."""
    pins = check_pins()
    config = check_config_amendment()
    entry = check_entry_evidence()
    destination = check_destination()
    command = execution_command(str(HERE / LEAF_ROOT / LEAF_NAME))

    problems = [entry_["artefact"] + ": " + entry_["why"]
                for entry_ in pins if not entry_["ok"]]
    problems += config["problems"] + entry["problems"] + destination["problems"]

    report = {
        "stage": STAGE,
        "executes": "the sealed warm-up command of " + READINESS_STAGE,
        "ok": not problems,
        "problems": problems,
        "pins": pins,
        "pins_checked": len(pins),
        "pins_matching": sum(1 for e in pins if e["ok"]),
        "config_amendment": config,
        "entry_evidence": entry,
        "destination": destination,
        "command": {k: v for k, v in command.items()
                    if k not in ("tokens", "sealed_tokens")},
        "it_performs_no_restore": True,
        "best_available_verdict": VERDICT_AWAITING,
    }
    if verbose:
        print("stage              %s" % STAGE)
        print("pins               %d/%d match"
              % (report["pins_matching"], report["pins_checked"]))
        print("config amendment   %s (%d semantic difference)"
              % ("OK" if config["ok"] else "FAILED", config["difference_count"]))
        print("  the difference   %s.%s: %r -> %r"
              % (CONFIG_DIFF_SECTION, CONFIG_DIFF_KEY, CONFIG_DIFF_FROM,
                 CONFIG_DIFF_TO))
        print("entry evidence     %s (%s)"
              % ("OK" if entry["ok"] else "FAILED",
                 ", ".join("%s=%s" % (k.split("/")[0], v)
                           for k, v in sorted(entry["found"].items())
                           if not k.endswith(".json"))))
        print("command            %d tokens, %d substitution from sealed_command()"
              % (command["token_count"], command["differences_from_sealed"]))
        print("destination        %s (%s)"
              % (destination["leaf"], "absent" if destination["leaf_exists"] is False
                 else "PRESENT"))
        print("verdict            %s" % ("READY" if report["ok"] else "BLOCKED"))
        for problem in problems:
            print("  problem: %s" % problem, file=sys.stderr)
    return report


# ---------------------------------------------------------------------- the GO --

def validate_go(payload) -> dict:
    """Validate an architect GO. It authorises ONE warm-up and nothing else."""
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"],
                "pins": {}}
    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append("stage is %r, expected %r"
                        % (payload.get("stage"), GO_REQUIRED_STAGE))
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")
    if payload.get("status") in ("DRAFT", "PROPOSED"):
        problems.append("this GO is a %s and does not authorise execution"
                        % payload.get("status"))
    for forbidden in ("authorises_restore", "authorises_retry",
                      "authorises_ppo", "authorises_ex_novo",
                      "authorises_promotion", "authorises_rewriting_k1"):
        if payload.get(forbidden) is True:
            problems.append("a warm-up GO must never set %s" % forbidden)
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
            problems.append("pinned artefact %s does not exist at %s" % (label, path))
            continue
        actual = sha256_file(path)
        if actual != pins[label]:
            problems.append("pinned hash for %s is stale: GO says %s, the file is %s"
                            % (label, pins[label], actual))
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
    """Read what the child wrote. Pure: opens files, writes nothing."""
    summary_path = leaf / "summary.json"
    rows_path = leaf / "train_iterations.jsonl"
    summary = json.loads(summary_path.read_text(encoding="utf-8")) \
        if summary_path.is_file() else None
    rows = []
    if rows_path.is_file():
        for line in rows_path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                rows.append(json.loads(line))
    return {"summary": summary, "rows": rows,
            "summary_present": summary_path.is_file(),
            "rows_present": rows_path.is_file()}


def audit_checkpoint(leaf: pathlib.Path) -> dict:
    """G9, structural half. The layout is READ from committed July artefacts.

    Two paths inside the checkpoint are decoys: learner_group/state.pkl and
    learner_group/learner/rl_module/module_state.pkl are five-byte EMPTY dicts.
    An auditor that opened those would conclude the optimizer and the module were
    missing. The real ones live one level deeper, and both are checked here.
    """
    import pickle

    root = leaf / CHECKPOINT_DIR
    report = {"checkpoint_dir": CHECKPOINT_DIR, "present": root.is_dir()}
    problems = []
    if not root.is_dir():
        return {"ok": False, "problems": ["no %s directory" % CHECKPOINT_DIR],
                **report}

    optimizer_path = root / OPTIMIZER_REL
    module_path = root / LEARNER_MODULE_REL
    report["optimizer_path"] = OPTIMIZER_REL
    report["learner_module_path"] = LEARNER_MODULE_REL
    for label, path in (("optimizer", optimizer_path),
                        ("learner module state", module_path)):
        if not path.is_file():
            problems.append("the %s is missing at %s" % (label, path))
    report["decoys_checked"] = {}
    for decoy in EMPTY_DECOY_PATHS:
        path = root / decoy
        empty = None
        if path.is_file():
            try:
                empty = len(pickle.loads(path.read_bytes())) == 0
            except Exception:  # noqa: BLE001 - a decoy that is unreadable is fine
                empty = None
        report["decoys_checked"][decoy] = {"exists": path.is_file(),
                                           "is_empty_dict": empty}
    if problems:
        return {"ok": False, "problems": problems, **report}

    optimizer = pickle.loads(optimizer_path.read_bytes())
    report["optimizer_top_keys"] = sorted(optimizer)
    if tuple(sorted(optimizer)) != OPTIMIZER_TOP_KEYS:
        problems.append("the optimizer state has keys %s, expected %s"
                        % (sorted(optimizer), list(OPTIMIZER_TOP_KEYS)))
    entry = (optimizer.get("optimizer") or {}).get(OPTIMIZER_ENTRY)
    if not isinstance(entry, dict) or "state" not in entry:
        problems.append("no %s entry with a torch state_dict" % OPTIMIZER_ENTRY)
        return {"ok": False, "problems": problems, **report}
    torch_state = entry["state"]
    groups = torch_state.get("param_groups") or []
    per_param = torch_state.get("state") or {}
    report["optimizer_param_group_size"] = \
        len(groups[0]["params"]) if groups else 0
    report["optimizer_adam_state_entries"] = len(per_param)
    report["optimizer_adam_state_indices"] = sorted(per_param)
    report["optimizer_learning_rate"] = groups[0].get("lr") if groups else None
    if report["optimizer_param_group_size"] != EXPECTED_PARAM_GROUP_SIZE:
        problems.append("the optimizer group holds %d parameters, expected %d"
                        % (report["optimizer_param_group_size"],
                           EXPECTED_PARAM_GROUP_SIZE))
    # Only the six critic tensors may carry Adam moments. A frozen actor takes no
    # gradient, so it can never acquire one. This is an INDEPENDENT reading of
    # the freeze, from the optimizer rather than from the digests.
    indices = tuple(sorted(int(i) for i in per_param))
    report["optimizer_adam_state_indices_expected"] = list(EXPECTED_ADAM_STATE_INDICES)
    report["only_the_critic_has_adam_moments"] = \
        indices == EXPECTED_ADAM_STATE_INDICES
    if len(per_param) != EXPECTED_ADAM_STATE_ENTRIES:
        problems.append("the optimizer holds %d per-parameter entries, expected "
                        "%d - one per critic tensor and none for the frozen actor"
                        % (len(per_param), EXPECTED_ADAM_STATE_ENTRIES))
    if indices != EXPECTED_ADAM_STATE_INDICES:
        problems.append("the optimizer carries Adam moments at indices %s, "
                        "expected exactly %s. Six entries at the wrong indices "
                        "would mean the actor took a gradient and some critic "
                        "tensor did not; the count alone cannot tell."
                        % (list(indices), list(EXPECTED_ADAM_STATE_INDICES)))

    module = pickle.loads(module_path.read_bytes())
    report["learner_module_keys"] = sorted(module)
    report["learner_module_key_count"] = len(module)
    missing_actor = [k for k in ACTOR_KEYS if k not in module]
    missing_critic = [k for k in CRITIC_KEYS if k not in module]
    if len(module) != 16 or missing_actor or missing_critic:
        problems.append("the learner module state holds %d keys (missing actor "
                        "%s, missing critic %s), expected the sixteen"
                        % (len(module), missing_actor, missing_critic))
    report["milestone_present"] = (leaf / MILESTONE_DIR).is_dir()
    return {"ok": not problems, "problems": problems, **report}


def audit_actor(leaf: pathlib.Path) -> dict:
    """G5, G6, G8: the actor and the log-std did not move; the file hash did."""
    import pickle

    import numpy as np

    module_path = leaf / CHECKPOINT_DIR / LEARNER_MODULE_REL
    on_disk = {k: np.asarray(v) for k, v in pickle.loads(
        (HERE / J19A_MODULE_REL / "module_state.pkl").read_bytes()).items()}
    trained = {k: np.asarray(v)
               for k, v in pickle.loads(module_path.read_bytes()).items()}
    per_key = {k: bytes_identical(trained[k], on_disk[k])
               for k in ACTOR_KEYS if k in trained}
    digest = actor_digest(trained) if all(k in trained for k in ACTOR_KEYS) else None
    head_w, head_b = trained.get("pi.1.weight"), trained.get("pi.1.bias")
    logstd_ok = bool(
        head_w is not None and head_b is not None
        and bytes_identical(np.asarray(head_w)[LOGSTD_ROWS],
                            np.asarray(on_disk["pi.1.weight"])[LOGSTD_ROWS])
        and bytes_identical(np.asarray(head_b)[LOGSTD_ROWS],
                            np.asarray(on_disk["pi.1.bias"])[LOGSTD_ROWS]))
    sigma = [float(x) for x in np.exp(
        np.asarray(head_b)[LOGSTD_ROWS].astype(np.float64))] if head_b is not None \
        else None
    file_sha = sha256_file(module_path)
    return {
        "actor_per_key_byte_identical": per_key,
        "actor_all_byte_identical": bool(per_key) and all(per_key.values()),
        "actor_digest": digest,
        "actor_digest_expected": PIN_J19A_ACTOR_DIGEST,
        "actor_digest_matches": digest == PIN_J19A_ACTOR_DIGEST,
        "logstd_rows_byte_identical": logstd_ok,
        "sigma": sigma,
        "sigma_expected": SIGMA,
        "trained_module_sha256": file_sha,
        "j19a_module_sha256": PIN_J19A_MODULE_STATE,
        "file_hashes_differ": file_sha != PIN_J19A_MODULE_STATE,
        "method": "same dtype, same shape, same C-order bytes; NOT array_equal",
    }


def evaluate_gates(child: dict, artefacts: dict, checkpoint: dict, actor: dict,
                   leaf_hashes: dict) -> dict:
    """The immediate gates. G9's restore half is NOT evaluated here."""
    summary = artefacts["summary"] or {}
    rows = artefacts["rows"]
    row = rows[0] if len(rows) == 1 else None
    metrics = (row or {}).get("learner_metrics") or {}
    health = (row or {}).get("training_health") or {}

    mean_kl = row.get("mean_kl_loss") if row else None
    if mean_kl is None:
        mean_kl = metrics.get(METRIC_MEAN_KL)
    # vf_loss is hoisted to the row by the trainer, but only when the suffix
    # match found it. Fall back to the literal learner_metrics key rather than
    # let a None row value read as "not finite".
    vf_loss = row.get("vf_loss") if row else None
    if vf_loss is None:
        vf_loss = metrics.get(METRIC_VF_LOSS)
    # vf_explained_var is NEVER hoisted; it exists only inside learner_metrics.
    vf_ev = metrics.get(METRIC_VF_EV)
    lifetime = row.get("num_env_steps_sampled_lifetime") if row else None
    this_iteration = metrics.get(METRIC_STEPS_THIS_ITERATION)

    freeze = summary.get("actor_freeze_audit") or []
    critic = summary.get("critic_state_audit") or []
    digests = [e.get("critic_digest") for e in critic]

    health_present = all(health.get(f) is not None for f in TRAINING_HEALTH_FIELDS)
    observed = health.get("observed_rows")
    missing = health.get("missing_telemetry_rows")

    gates = {
        "G1_exactly_one_logical_iteration":
            len(rows) == 1 and row is not None and int(row.get("iteration", -1)) == 1
            and summary.get("iterations_completed_this_process") == 1,
        "G2_sampled_lifetime_AND_per_iteration_delta_are_exactly_the_batch":
            lifetime is not None and float(lifetime) == EXPECTED_SAMPLED_STEPS
            and this_iteration is not None
            and float(this_iteration) == EXPECTED_SAMPLED_STEPS,
        "G3_critic_metrics_are_finite":
            finite(vf_loss) and finite(vf_ev),
        "G4_mean_kl_is_zero":
            finite(mean_kl) and abs(float(mean_kl)) <= KL_TOLERANCE,
        "G5_actor_bit_exact_at_every_audit":
            len(freeze) >= 2
            and all(e.get("exact") is True and float(e.get("max_abs_diff", 1)) == 0.0
                    for e in freeze)
            and all(e.get("actor_digest") == PIN_J19A_ACTOR_DIGEST for e in freeze)
            and bool(actor["actor_all_byte_identical"])
            and bool(actor["actor_digest_matches"]),
        "G6_logstd_bit_exact_and_sigma_unchanged":
            bool(actor["logstd_rows_byte_identical"])
            and actor["sigma"] is not None
            and all(abs(s - SIGMA) <= 1e-09 for s in actor["sigma"]),
        "G7_critic_digest_changed":
            len(digests) >= 2 and all(digests) and digests[0] != digests[-1],
        "G8_full_module_hash_differs_from_j19a":
            bool(actor["file_hashes_differ"]),
        "G9_structural_checkpoint_and_optimizer":
            bool(checkpoint["ok"]),
        "G10_no_retry_no_crash_no_timeout":
            child["returncode"] == 0 and summary.get("ok") is True
            and summary.get("stop_reason") == "completed"
            and summary.get("timed_out") is False
            and summary.get("interrupted") is False
            and not (leaf_hashes["leaf_files"].get("supervisor_state.json")),
        "G11_training_health_complete":
            health_present
            and finite(observed) and finite(missing)
            and float(missing) == 0.0
            and float(observed) + float(missing) == EXPECTED_SAMPLED_STEPS,
        "G12_j19a_leaf_byte_unchanged":
            leaf_hashes["j19a_unchanged"],
    }
    measurements = {
        "iterations_completed_this_process":
            summary.get("iterations_completed_this_process"),
        "num_env_steps_sampled_lifetime": lifetime,
        "steps_sampled_this_iteration": this_iteration,
        "steps_metric_key_this_iteration": METRIC_STEPS_THIS_ITERATION,
        "vf_loss": vf_loss,
        "vf_loss_source": "row" if (row or {}).get("vf_loss") is not None
        else METRIC_VF_LOSS,
        "vf_explained_var": vf_ev,
        "mean_kl_loss": mean_kl,
        "actor_freeze_audit_entries": len(freeze),
        "critic_digest_before": digests[0] if digests else None,
        "critic_digest_after": digests[-1] if digests else None,
        "child_returncode": child["returncode"],
        "summary_ok": summary.get("ok"),
        "summary_stop_reason": summary.get("stop_reason"),
        # RECORDED, NOT BOUNDED. Their presence is the gate, their value is data.
        "training_health": {f: health.get(f) for f in TRAINING_HEALTH_FIELDS},
        "training_health_incident_fields_are_recorded_not_bounded":
            list(TRAINING_HEALTH_INCIDENT_FIELDS),
    }
    return {"gates": gates, "passed": sum(1 for v in gates.values() if v),
            "total": len(gates),
            "failed": sorted(k for k, v in gates.items() if not v),
            "measurements": measurements}


# ------------------------------------------------------------------- execute --

def hash_tree(root: pathlib.Path) -> dict:
    """Every file under a directory, relative path -> sha256."""
    out = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            out[str(path.relative_to(root))] = sha256_file(path)
    return out


def run_execution(go_file: str) -> dict:
    """Launch the child ONCE, audit, commit. No retry, whatever happens."""
    go = load_go(go_file)
    if not go["valid"]:
        raise WarmupError("refusing to execute: the architect GO is absent or "
                          "invalid. %s" % "; ".join(go["problems"]))
    report = preflight(verbose=False)
    if not report["ok"]:
        raise WarmupError("refusing to execute: preflight did not pass: %s"
                          % "; ".join(report["problems"]))

    leaf = HERE / LEAF_ROOT / LEAF_NAME
    if leaf.exists():
        raise WarmupError("refusing to clobber an existing leaf: %s" % leaf)

    # Born invalid, and the marker goes in FIRST. The child will write into this
    # same directory, so there is no staging-then-rename here; the marker is what
    # makes the leaf invalid from its first byte until the audit clears it.
    leaf.mkdir(parents=True)
    (leaf / INVALID_MARKER).write_bytes(
        b"born invalid; replaced by RESTORE_AUDIT_PENDING only after every "
        b"immediate gate passes\n")
    residents = sorted(p.name for p in leaf.iterdir())
    if residents != [INVALID_MARKER]:
        raise WarmupError("the destination is not clean: it holds %s. An "
                          "inherited train_iterations.jsonl or checkpoint would "
                          "be silently reused by the child." % residents)

    before = {
        "j19a": {n: sha256_file(HERE / J19A_LEAF_REL / n) for n in PIN_J19A_LEAF},
        "k1": {n: sha256_file(HERE / K1_LEAF_REL / n) for n in PIN_K1_LEAF},
        "k1r1": {n: sha256_file(HERE / K1R1_LEAF_REL / n) for n in PIN_K1R1_LEAF},
        "config": sha256_file(HERE / CONFIG_NAME),
        "j20_config": sha256_file(HERE / J20_CONFIG_NAME),
        "overlay": sha256_file(HERE / OVERLAY_NAME),
    }

    command = execution_command(str(leaf))
    child = launch_once(command["tokens"], leaf)

    after = {
        "j19a": {n: sha256_file(HERE / J19A_LEAF_REL / n) for n in PIN_J19A_LEAF},
        "k1": {n: sha256_file(HERE / K1_LEAF_REL / n) for n in PIN_K1_LEAF},
        "k1r1": {n: sha256_file(HERE / K1R1_LEAF_REL / n) for n in PIN_K1R1_LEAF},
        "config": sha256_file(HERE / CONFIG_NAME),
        "j20_config": sha256_file(HERE / J20_CONFIG_NAME),
        "overlay": sha256_file(HERE / OVERLAY_NAME),
    }
    leaf_hashes = {
        "leaf_files": hash_tree(leaf),
        "inputs_before": before,
        "inputs_after": after,
        "inputs_unchanged": before == after,
        "j19a_unchanged": after["j19a"] == dict(PIN_J19A_LEAF)
        and before["j19a"] == after["j19a"],
        "k1_unchanged": after["k1"] == dict(PIN_K1_LEAF),
        "k1r1_unchanged": after["k1r1"] == dict(PIN_K1R1_LEAF),
    }

    artefacts = read_child_artefacts(leaf)
    checkpoint = audit_checkpoint(leaf)
    actor = audit_actor(leaf) if checkpoint["ok"] else {
        "actor_all_byte_identical": False, "actor_digest_matches": False,
        "logstd_rows_byte_identical": False, "sigma": None,
        "file_hashes_differ": False, "actor_per_key_byte_identical": {},
        "actor_digest": None, "actor_digest_expected": PIN_J19A_ACTOR_DIGEST,
        "trained_module_sha256": None, "j19a_module_sha256": PIN_J19A_MODULE_STATE,
        "method": "not evaluated: the checkpoint audit failed first",
    }
    graded = evaluate_gates(child, artefacts, checkpoint, actor, leaf_hashes)

    immediate_ok = bool(
        all(graded["gates"].values())
        and leaf_hashes["inputs_unchanged"]
        and leaf_hashes["j19a_unchanged"]
        and leaf_hashes["k1_unchanged"]
        and leaf_hashes["k1r1_unchanged"])
    verdict = VERDICT_AWAITING if immediate_ok else VERDICT_FAILED

    result = {
        "kind": "CRITIC-ONLY WARM-UP RESULT",
        "stage": STAGE,
        "verdict": verdict,
        "why_not_PASS":
            "the exact restore audit that gate G9 demands is a SEPARATE stage "
            "this runner cannot reach. Every immediate gate passed and one gate "
            "is still owed, so the honest verdict is AWAITING_RESTORE_AUDIT. "
            "Calling it PASS would claim evidence that does not exist yet."
            if immediate_ok else
            "at least one immediate gate failed; the evidence is preserved and "
            "there is no retry",
        "gates": graded["gates"],
        "gates_passed": graded["passed"],
        "gates_total": graded["total"],
        "gates_failed": graded["failed"],
        "measurements": graded["measurements"],
        "child": child,
        "command": {"tokens": list(command["tokens"]),
                    "token_count": command["token_count"],
                    "source": command["source"],
                    "substituted_index": command["substituted_index"],
                    "substituted_from": command["substituted_from"],
                    "substituted_to": command["substituted_to"],
                    "differences_from_sealed_command": 1},
        "config_amendment": report["config_amendment"],
        "checkpoint": checkpoint,
        "actor": actor,
        "integrity": leaf_hashes,
        "entry_evidence": report["entry_evidence"]["found"],
        "restore_audit": {
            "performed": False,
            "performable_here": False,
            "why": "a restore would require rebuilding the algorithm and calling "
                   "restore_from_path, which this stage is structurally "
                   "forbidden from doing",
            "owed_gate": "G9, exact half: reload the checkpoint and reproduce "
                         "both the actor digest and the critic digest",
            "marker": PENDING_MARKER,
        },
        "inert": {
            "retried": False,
            "child_processes_launched": 1,
            "supervisor_used": False,
            "restore_performed": False,
            "ppo_ex_novo": False,
            "promotion": "NONE",
        },
        "promotion": "NONE",
        "training_ready": False,
        "next_stage_authorized": False,
    }

    files = {
        RESULT_NAME: encode_json(result),
    }
    receipt = {
        "kind": "EXECUTION RECEIPT",
        "stage": STAGE,
        "verdict": verdict,
        "gates_passed": graded["passed"],
        "gates_total": graded["total"],
        "gates_failed": graded["failed"],
        "restore_audit_pending": True,
        "inputs": {
            "prereg_sha256": sha256_file(HERE / PREREG_NAME),
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "config_sha256": PIN_CONFIG,
            "j20_config_sha256": PIN_J20_CONFIG,
            "overlay_sha256": PIN_OVERLAY,
            "j19a_actor_digest": PIN_J19A_ACTOR_DIGEST,
            "baseline_pins_sha256": dict(sorted(BASELINE_PINS.items())),
            "architect_go_pins": go["pins"],
        },
        "child_returncode": child["returncode"],
        "child_pid": child["pid"],
        "child_started_at": child["started_at"],
        "child_finished_at": child["finished_at"],
        "inputs_unchanged": leaf_hashes["inputs_unchanged"],
        "j19a_leaf_unchanged": leaf_hashes["j19a_unchanged"],
        "k1_leaf_unchanged": leaf_hashes["k1_unchanged"],
        "k1r1_leaf_unchanged": leaf_hashes["k1r1_unchanged"],
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

    verification = verify_commit(leaf, files, immediate_ok)
    return {"leaf": str(leaf.relative_to(HERE)), "verdict": verdict,
            "gates": "%d/%d" % (graded["passed"], graded["total"]),
            "verification": verification}


def launch_once(tokens, leaf: pathlib.Path) -> dict:
    """Start the child EXACTLY once and wait. No loop, no retry, no supervisor.

    subprocess.Popen, not run(), because the PID has to be on the record and
    run() does not expose it. There is one Popen in this file and no code path
    that reaches it twice.
    """
    import subprocess

    started = now_utc()
    environment = dict(os.environ)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
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
    }


def verify_commit(leaf: pathlib.Path, files: dict, immediate_ok: bool) -> dict:
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
    for label, pinned, root in (("J19A", PIN_J19A_LEAF, HERE / J19A_LEAF_REL),
                                ("K1", PIN_K1_LEAF, HERE / K1_LEAF_REL),
                                ("K1R1", PIN_K1R1_LEAF, HERE / K1R1_LEAF_REL)):
        for name, expected in sorted(pinned.items()):
            actual = sha256_file(root / name)
            if actual != expected:
                problems.append("the %s leaf file %s changed: expected %s, found %s"
                                % (label, name, expected, actual))

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(leaf.relative_to(HERE)),
        "files_verified": len(committed["artefacts_sha256"]),
        "immediate_gates_passed": immediate_ok,
        "problems": problems,
        "ok": not problems and immediate_ok,
        "method": "every leaf-relative path named by the COMMITTED receipt is "
                  "re-hashed against it and against the written bytes, and the "
                  "J19A, K1 and K1R1 leaves are re-hashed against their pins",
        "marker": PENDING_MARKER if (not problems and immediate_ok)
        else INVALID_MARKER,
    }
    (leaf / COMMIT_VERIFICATION_NAME).write_bytes(encode_json(verification))
    if problems or not immediate_ok:
        return verification
    (leaf / PENDING_MARKER).write_bytes(
        b"the immediate gates passed. The exact restore audit that gate G9 "
        b"demands is a SEPARATE stage and has NOT been performed. Nothing here "
        b"is promoted and nothing is training-ready.\n")
    (leaf / INVALID_MARKER).unlink()
    return verification


# ------------------------------------------------------------------------ CLI --

def build_parser() -> argparse.ArgumentParser:
    """Command-line surface. Preflight is the default and writes nothing."""
    parser = argparse.ArgumentParser(
        description="V26C J20 - critic-only warm-up, single execution")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--go-file", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Return 0 on success, 1 on failure. Writes nothing unless --execute."""
    args = build_parser().parse_args(argv)
    if args.execute:
        if not args.go_file:
            print("--execute requires --go-file", file=sys.stderr)
            return 1
        try:
            outcome = run_execution(args.go_file)
        except (WarmupError, RuntimeError) as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"], "verdict": outcome["verdict"],
                          "gates": outcome["gates"]}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = preflight(verbose=True)
    if args.dry_run:
        command = execution_command(str(HERE / LEAF_ROOT / LEAF_NAME))
        print("\nplan")
        print("  create the leaf, write %s FIRST, assert it is the only" % INVALID_MARKER)
        print("  resident, launch ONE child, audit, then replace the marker with")
        print("  %s only if every immediate gate passed." % PENDING_MARKER)
        print("\n  best available verdict: %s. NOT a final PASS: the exact"
              % VERDICT_AWAITING)
        print("  restore audit of gate G9 is a separate stage.")
        print("\nthe command, one substitution from sealed_command():")
        print("  cd '%s'" % BASELINE)
        print("  PYTHONDONTWRITEBYTECODE=1 " + " ".join(
            '"%s"' % t if " " in t else t for t in command["tokens"]))
        print("\nthis stage authorises no restore, no PPO and no promotion.")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
