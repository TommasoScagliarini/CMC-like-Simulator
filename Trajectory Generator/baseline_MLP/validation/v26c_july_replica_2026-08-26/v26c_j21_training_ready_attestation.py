"""V26C J21 - the evidence-aggregation attestation that may set training_ready.

READ-ONLY BY CONSTRUCTION. This file imports the standard library and nothing
else: hashlib, json, pathlib, argparse, ast, sys, os, datetime. It does not
import train_ppo_mlp, ray, torch, numpy, gymnasium, opensim or any simulation
module; it starts no Ray, builds no Algorithm, constructs no environment,
samples nothing, rolls out nothing, launches no subprocess and trains nothing.
Gate T14 audits this file's own source at run time and fails the stage if any of
that stops being true.

What it does
------------
It reads the already-committed evidence of J19A, J19B, J19C, K1R1, the J20
critic-only warm-up and the J20 R3 restore audit, re-hashes every pinned byte,
and evaluates eighteen binding gates. If all eighteen pass it attests that the
EXISTING J20 warm-up checkpoint is TRAINING_READY as an INPUT checkpoint for a
future conservative PPO pilot.

What that attestation is, exactly
---------------------------------
It is a statement about an ARTEFACT, not an authorisation of an ACTION. On a
full pass the result records training_ready true and promotion
TRAINING_INPUT_ONLY, and it records launch_authorized false, ppo_authorized
false, ex_novo_authorized false, next_stage_authorized false, training_started
false, training_iterations_run 0, sampling false, rollout false, ray_started
false, environment_constructed false and deployable false. Every one of those is
written on success as well as on failure.

The pilot protocol is NOT sealed
--------------------------------
At the time this file was written no pilot configuration and no pilot launch
command existed, were pinned or were approved. This stage therefore attests
CHECKPOINT readiness, not PILOT readiness, and it is forbidden from inventing
either. The preregistration's readiness_scope section is the closed list of what
training_ready does and does not assert.

The warm-up leaf is byte-immutable
----------------------------------
G9's closure is read from the R3 receipt. It is NEVER written into the warm-up
leaf, which keeps its RESTORE_AUDIT_PENDING marker at its pinned hash. Gate T11
measures that rather than assuming it.

Preregistered by v26c_j21_prereg_training_ready_attestation.json.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
REPO = HERE.parents[3]

STAGE = "V26C_J21_TRAINING_READY_ATTESTATION"
GO_REQUIRED_STAGE = STAGE
GO_REQUIRED_STATUS = "APPROVED"
GO_USER_AUTH_SCOPE = "J21 evidence aggregation only"
GO_USER_AUTH_ATTESTATION_QUOTE = \
    "per questo ultimo passaggio non usare claude. arriva training ready"
GO_USER_AUTH_STOP_QUOTE = "fermati prima di lanciare un training"

PREREG_NAME = "v26c_j21_prereg_training_ready_attestation.json"
PIN_PREREG = "df178ffae521aaf46837258ce2036789c6f8ae9f6b4cfb27b4cd11b85d06da79"
PREREG_REV1_NAME = "v26c_j21_prereg_training_ready_attestation_rev1.json"
PIN_PREREG_REV1 = \
    "9fa87c120de56d4ef0ae063ce3a4126b94a37ca629fe30710387958d9bcdfdfa"
PREREG_PRECEDENCE = ("rev1", "base")
RUNNER_NAME = "v26c_j21_training_ready_attestation.py"
TEST_NAME = "test_v26c_j21_training_ready_attestation.py"

# ------------------------------------------------------------- upstream anchors --
# The R3 architect GO carries a CLOSED map of 92 pinned artefacts covering the
# production modules, the configs, every preregistration from base through rev7,
# and the J19A/J19B/J19C/K1R1/warm-up/R1/R2 leaves. Pinning the GO and then
# re-verifying every label it names inherits that whole closure without copying
# 92 literals and without the chance of a transcription error.
R3_GO_NAME = "v26c_j20_restore_audit_r3_architect_go.json"
PIN_R3_GO = "fe090682f046a1ba74a3cc4f7d8c6b319c243920a375b8c19265e107bb9ca583"
R3_GO_PIN_COUNT = 92

WARMUP_LEAF_REL = "j20_runs/j20_critic_warmup_v26c_2026-08-27_r1"
R3_LEAF_REL = "j20_runs/j20_restore_audit_v26c_2026-08-28_r3"
R1_LEAF_REL = "j20_runs/j20_restore_audit_v26c_2026-08-27_r1"
R2_LEAF_REL = "j20_runs/j20_restore_audit_v26c_2026-08-27_r2"
J19A_LEAF_REL = "j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1"
J19B_LEAF_REL = "j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1"
J19C_LEAF_REL = "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1"
K1R1_LEAF_REL = "j20_runs/j20_k1r1_nondegenerate_gradient_v26c_2026-08-27_r1"
J18_LEAF_REL = "j18_runs/j18_b_only_update_v26c_2026-08-27_r1"
CHECKPOINT_REL = WARMUP_LEAF_REL + "/checkpoint_last"

# Artefacts the R3 GO does not name. Forty-five literals, hashed from disk at
# preparation time and re-hashed here. The final nine close the measured
# August lineage introduced by rev1.
PIN_ADDITIONAL = {
    "v26c_penetration_contract_2026-08-26.json":
        "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461",
    "v26c_penetration_contract.py":
        "9257e9b8cdf54d9a59bfe2ee25526b283408d325a900156a69d64dbf196298dc",
    "v26c_penetration_contract_reconciliation.json":
        "ef74cb55efcd084cfddf8f4bd9ec8c944d2fddd10316dafdb1897255838be145",
    "v26c_j20_restore_audit_r3_architect_go.json":
        "fe090682f046a1ba74a3cc4f7d8c6b319c243920a375b8c19265e107bb9ca583",
    "v26c_j20_restore_audit_r3_go_DRAFT_rev7.json":
        "07a5bc4302f7618857511bd0105771de5025df8d6909ac87e8d652ad91d07943",
    "v26c_j20_restore_audit_r3_go_DRAFT.json":
        "dce5a521fb21a5e192d341aff0fbc977da87cf71b8a3b0e81bc522399fe2d908",
    R3_LEAF_REL + "/RESTORE_AUDIT_PASSED":
        "56bac0767c80d4a861c28f97d9aafbdf6c1e7061783b0ca5bba317b121705f2e",
    R3_LEAF_REL + "/child_stdout_stderr.txt":
        "8163d3cc5d0aa0a94b9dd07034a2346c4438952b6f8d2807a47599b3d6f28133",
    R3_LEAF_REL + "/commit_verification.json":
        "e53c48a9d7a59da5ada3bac0bebbd7b42c8e8437fa1915ead5dbdca73c5b58fb",
    R3_LEAF_REL + "/faulthandler.log":
        "5db53236c7f04fecdd1c18eafe871c7560ffa21851667df0e4b038c0a10e8e10",
    R3_LEAF_REL + "/live_optimizer_audit.json":
        "9e6f2acc48f4a3f6d024b0ecfda5b57ff9643d9873d192349bc90b1d7af81c3c",
    R3_LEAF_REL + "/summary.json":
        "748e25725beb264524e9f14170d9832bcf08ba3c0af96f4265ab3c5f91027f21",
    R3_LEAF_REL + "/tensorboard/"
    "events.out.tfevents.1787873944.MacBook-Pro.local.42763.0":
        "2ee863c757eac46e3ab15a09ad34cb501d2835587f39fc0646432720ee59938c",
    R3_LEAF_REL + "/training_cfg.resolved.yaml":
        "976f6c5050161f3edf610de1843c186186fdc56ea9d563cf0fc4c1958c14a810",
    R3_LEAF_REL + "/v26c_j20_restore_audit_receipt.json":
        "0426774a88b3793acc7a639a4817f8d008b730b835a63bc70dc4dbba7248c51c",
    R3_LEAF_REL + "/v26c_j20_restore_audit_result.json":
        "dbd1837a0271c831578e05bf18dc5f4badc83fdd5446632b8e9ac20e584ababb",
    R3_LEAF_REL + "/watchdog_state.json":
        "90e6ea90a06325469da7246a8dbf1f3bafc0aab366298cccc0cae924c0349a48",
    CHECKPOINT_REL + "/class_and_ctor_args.pkl":
        "896224b5c03d2dd50008890537b109ea63d93d9cb28a2eff84b7e92da1558b4a",
    CHECKPOINT_REL + "/env_runner/class_and_ctor_args.pkl":
        "5b871fd6771d0e7e5663ab9992a117f2e93d476004009669a8257760b3b213f6",
    CHECKPOINT_REL + "/env_runner/env_to_module_connector/"
    "class_and_ctor_args.pkl":
        "c5d4891c43a0b130b65e204262da163eec512a47176f12810cc0e5ccff884bc3",
    CHECKPOINT_REL + "/env_runner/env_to_module_connector/metadata.json":
        "96ece4363f8b7069198bf7a7bee7954f599110723552317892af0a20abb8a79e",
    CHECKPOINT_REL + "/env_runner/env_to_module_connector/state.pkl":
        "bfce13fc2f13ad950016bb48ccbbbd04ce13be07c748476dde1f940fcdfa0d52",
    CHECKPOINT_REL + "/env_runner/metadata.json":
        "96ece4363f8b7069198bf7a7bee7954f599110723552317892af0a20abb8a79e",
    CHECKPOINT_REL + "/env_runner/module_to_env_connector/"
    "class_and_ctor_args.pkl":
        "5b28d5edb328605254859b8ad42d42498be344d7f272c12646a5a3e0c807f28f",
    CHECKPOINT_REL + "/env_runner/module_to_env_connector/metadata.json":
        "96ece4363f8b7069198bf7a7bee7954f599110723552317892af0a20abb8a79e",
    CHECKPOINT_REL + "/env_runner/module_to_env_connector/state.pkl":
        "bfce13fc2f13ad950016bb48ccbbbd04ce13be07c748476dde1f940fcdfa0d52",
    CHECKPOINT_REL + "/env_runner/state.pkl":
        "036fc526315c6d82c1e5e19b910aeb870fef040bb9c5ef0c38c3cfa3857c963e",
    CHECKPOINT_REL + "/learner_group/class_and_ctor_args.pkl":
        "cbaa031d19f1884842c4b8b3c89ad9cf4eb7b97391e883f74936c1db76871027",
    CHECKPOINT_REL + "/learner_group/learner/class_and_ctor_args.pkl":
        "bd81d0133935d45fe0d0a6c7cca64874d4d58fc6413199c86b168aebf4924547",
    CHECKPOINT_REL + "/learner_group/learner/metadata.json":
        "96ece4363f8b7069198bf7a7bee7954f599110723552317892af0a20abb8a79e",
    CHECKPOINT_REL + "/learner_group/learner/rl_module/class_and_ctor_args.pkl":
        "fd8c8dcc637c08b9ee32317e5305cd1ec35db9c3033e88def0e71db21a995c7a",
    CHECKPOINT_REL + "/learner_group/learner/rl_module/default_policy/"
    "metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    CHECKPOINT_REL + "/learner_group/learner/rl_module/metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    CHECKPOINT_REL + "/learner_group/learner/rl_module/module_state.pkl":
        "bfce13fc2f13ad950016bb48ccbbbd04ce13be07c748476dde1f940fcdfa0d52",
    CHECKPOINT_REL + "/learner_group/metadata.json":
        "96ece4363f8b7069198bf7a7bee7954f599110723552317892af0a20abb8a79e",
    CHECKPOINT_REL + "/learner_group/state.pkl":
        "bfce13fc2f13ad950016bb48ccbbbd04ce13be07c748476dde1f940fcdfa0d52",
    # rev1: the artefacts that make the August lineage MEASURED rather than a
    # constant. These are the final nine of the 45 additional pins.
    "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/"
    "rl_module_best/module_state.pkl":
        "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd",
    "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/"
    "rl_module_best/actor_feature_manifest.json":
        "2837779ceb5953b18cf9be0e62836c8cc56d3fe871c48b869a5cedb6b0d81945",
    "j2_runs/j2_base_v26c_2026-08-26_r1/rl_module/module_state.pkl":
        "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130",
    "j2_runs/j2_base_v26c_2026-08-26_r1/rl_module/actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
    "j2_runs/j2_base_v26c_2026-08-26_r1/v26c_j2_fit_receipt.json":
        "dd997dc9d465184de6e25a3488c9752799fb60feca304ac876f35dfbdcd9871e",
    "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1/rl_module/module_state.pkl":
        "9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82",
    "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1/"
    "v26c_j8_recovery_fit_receipt.json":
        "13adc58b81ea372edef43a3d57f62b5f971e6be87c5242a8a0e234e993de6995",
    "j18_runs/j18_b_only_update_v26c_2026-08-27_r1/"
    "v26c_j18_b_only_update_receipt.json":
        "600c3c45a49d39da93e4e6e890a479e923e51d456cf92c4faa245031929452db",
    "v26c_j18_provenance_overlay_j8_2026-08-27.json":
        "dc4fd169ec95e4bcec1d472ab94f176b980f059ffdd98e28d009e83db0db9128",
}

# ------------------------------------------------------------ expected values --
EXPECTED_ACTOR_DIGEST = \
    "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96"
EXPECTED_CRITIC_DIGEST = \
    "2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410"
EXPECTED_MODULE_STATE_SHA = \
    "57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168"
EXPECTED_LEARNER_STATE_SHA = \
    "51d97ef016aea2086201b55db807f3273b76784bbafdea6cc9e5c7fea90294be"
EXPECTED_PENETRATION_CONTRACT_SHA = \
    "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461"
EXPECTED_SIGMA = 0.004999999670722372
EXPECTED_SAMPLED_STEPS = 4096.0
EXPECTED_WEIGHTS_SEQ_NO = 1.0
EXPECTED_CHECKPOINT_FILES = 24
EXPECTED_MODULE_KEYS = 16
EXPECTED_ADAM_INDICES = [6, 7, 8, 9, 10, 11]
EXPECTED_PARAM_GROUP_SIZE = 12
BANDS_M = {"soft_diagnostic": 0.02, "july_legacy": 0.025, "hard_binding": 0.028}
# The contract names its middle band july_legacy_hard_reference; the J19 cells
# name the same threshold july_legacy. Both names are checked against the same
# number rather than one being assumed to imply the other.
CONTRACT_BAND_KEYS = {"soft_diagnostic": "soft_diagnostic",
                      "july_legacy": "july_legacy_hard_reference",
                      "hard_binding": "hard_binding"}
# Only the hard band may fail a stage. The other two are declared non-binding by
# the contract itself, and that declaration is verified rather than trusted.
CONTRACT_BINDING = {"soft_diagnostic": False, "july_legacy": False,
                    "hard_binding": True}

J19A_BINDING_IDS = ("G5", "G6", "G3", "G4", "E-C", "E-D", "G7", "G8", "G9",
                    "G10", "G11")
J19A_DIAGNOSTIC_IDS = ("G1", "G2")
J19B_CELLS = ("A", "B", "C", "D", "E", "F")
J19C_CELLS = ("G", "H", "I")
JULY_LEGACY_CELLS = ("F", "H")           # the two disclosed >= 25 mm cells
ZERO_CELL_COUNTERS = ("phase_timeout_stance", "phase_timeout_swing",
                      "morphology_causal_contract_failure", "resync_count",
                      "hs_cancelled_count")
ZERO_HEALTH_FIELDS = ("phase_timeout_stance_rows",
                      "morphology_causal_contract_failure_rows",
                      "resync_event_rows", "resync_count_max",
                      "hs_cancelled_count_max", "missing_telemetry_rows",
                      "timeout_side_disagreement_rows")
WARMUP_GATE_NAMES = (
    "G1_exactly_one_logical_iteration",
    "G2_sampled_lifetime_AND_per_iteration_delta_are_exactly_the_batch",
    "G3_critic_metrics_are_finite", "G4_mean_kl_is_zero",
    "G5_actor_bit_exact_at_every_audit",
    "G6_logstd_bit_exact_and_sigma_unchanged", "G7_critic_digest_changed",
    "G8_full_module_hash_differs_from_j19a",
    "G9_structural_checkpoint_and_optimizer",
    "G10_no_retry_no_crash_no_timeout", "G11_training_health_complete",
    "G12_j19a_leaf_byte_unchanged")

OPERATIONAL_LINEAGE = (
    "August V26 imitation -> J2 35D -> J8 -> J18 c13 -> J19A -> "
    "J19B / J19C -> J20 critic-only warm-up -> J20 R3 restore audit")

# ------------------------------------------------- rev1: the measured lineage --
# Every operational parent is identified by the BYTES its child's receipt
# records, not by a label. A stage may be called July-faithful and still have an
# August parent - J8 is exactly that case - so a label is never evidence here.
AUGUST_SOURCE_REL = ("runs/training/"
                     "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/"
                     "rl_module_best")
SHA_AUGUST_MODULE = \
    "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
SHA_AUGUST_MANIFEST39 = \
    "2837779ceb5953b18cf9be0e62836c8cc56d3fe871c48b869a5cedb6b0d81945"
SHA_J2_MODULE = \
    "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
SHA_J2_MANIFEST35 = \
    "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81"
SHA_J8_MODULE = \
    "9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82"
DIGEST_J8_ACTOR = \
    "6a879714044ba8321fedf8e554d0f2ec448c1f1177e1648e4e4aa72195031207"
SHA_J19A_MODULE = \
    "8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530"
AUGUST_WIDTH = 39
STUDENT_WIDTH = 35
# What J8 must declare it does NOT take from July. A J8 claiming July's parent
# or actor would fail; a J8 replicating July's ALGORITHM is exactly correct.
JULY_MUST_NOT_REPLICATE = ("July's dataset", "July's parent", "July's actor",
                           "July's output")
JULY_FORBIDDEN_AS_INPUT = ("checkpoint", "actor weights", "dataset", "trace",
                           "labels")
JULY_ALLOWED_AS_INFORMATIONAL = (
    "July-derived methodological constants, such as the hyperparameter "
    "provenance the J2 receipt cites",
    "July-derived physical scaling code and the documented seeding and split "
    "protocol",
    "the 25 mm July legacy reference band, diagnostic and never binding",
    "stage and directory LABELS containing the word July, including the "
    "containing directory v26c_july_replica_2026-08-26 and the JULY_FAITHFUL "
    "label of J8, whose actual parent bytes are August V26",
)

# ------------------------------------- rev1: worker sync, the real proof --
TRANSPLANT_REPORT_REL = WARMUP_LEAF_REL + "/actor_transplant_report.json"
EXPECTED_ENV_RUNNERS = 14

# ------------------------------------- rev1: exact leaf file sets --
EXPECTED_COMMITTED_MAP_SIZES = {"j19b": 132, "j19c": 66}
EXPECTED_LEAF_FILE_COUNTS = {"j19a": 7, "j19b": 134, "j19c": 68, "r3": 11}
J19A_EXPECTED_FILES = (
    "commit_verification.json", "history.json",
    "rl_module/actor_feature_manifest.json",
    "rl_module/class_and_ctor_args.pkl", "rl_module/module_state.pkl",
    "v26c_j19a_result.json", "v26c_j19a_single_reproduction_receipt.json")
R3_EXPECTED_FILES = (
    "RESTORE_AUDIT_PASSED", "child_stdout_stderr.txt",
    "commit_verification.json", "faulthandler.log",
    "live_optimizer_audit.json", "summary.json",
    "tensorboard/events.out.tfevents.1787873944.MacBook-Pro.local.42763.0",
    "training_cfg.resolved.yaml", "v26c_j20_restore_audit_receipt.json",
    "v26c_j20_restore_audit_result.json", "watchdog_state.json")

# ------------------------------------- rev1: the truthful training counters --
CRITIC_WARMUP_ITERATIONS_COMPLETED = 1
CHECKPOINT_LOGICAL_ITERATION = 1
NEXT_LOGICAL_ITERATION = 2

# T14: this file must import none of these and call none of these.
FORBIDDEN_IMPORTS = ("train_ppo_mlp", "ray", "torch", "numpy", "gymnasium",
                     "opensim", "simulation_runner", "model_loader",
                     "rollout_eval", "warm_start", "asymmetric_rl_module",
                     "subprocess", "multiprocessing", "socket")
FORBIDDEN_CALL_NAMES = ("init", "build_algo", "restore_from_path", "train",
                        "sample", "rollout", "Popen", "run", "system",
                        "spawn", "fork", "execv")

# ------------------------------------------------------------------- the leaf --
LEAF_ROOT = "j21_runs"
LEAF_NAME = "j21_training_ready_attestation_v26c_2026-08-28_r1"
INVALID_MARKER = "TECHNICAL_INVALID"
ATTESTED_MARKER = "TRAINING_READY_ATTESTED"
RESULT_NAME = "v26c_j21_training_ready_result.json"
RECEIPT_NAME = "v26c_j21_training_ready_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"

VERDICT_PASS = "TRAINING_READY_ATTESTED"
VERDICT_FAILED = "FAIL_CLOSED"
PROMOTION_PASS = "TRAINING_INPUT_ONLY"
PROMOTION_FAIL = "NONE"


class AttestationError(RuntimeError):
    """Any refusal. The stage fails closed on all of them."""


# ------------------------------------------------------------------ helpers --

def sha256_file(path: pathlib.Path) -> str:
    """sha256 of a file's bytes, streamed so a large artefact is not held."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def encode_json(payload) -> bytes:
    """Canonical JSON bytes: sorted keys, indent 2, no NaN."""
    return json.dumps(payload, indent=2, sort_keys=True,
                      allow_nan=False).encode("utf-8")


def now_utc() -> str:
    """An ISO-8601 UTC timestamp."""
    import datetime
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def read_json(path: pathlib.Path):
    """Read a JSON artefact. A missing or unreadable file is a refusal."""
    if not path.is_file():
        raise AttestationError("required evidence is missing: %s" % path)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError as error:
        raise AttestationError("unreadable evidence %s: %s" % (path, error))


def need(payload, path: str, where: str, problems: list):
    """Fetch a dotted field. A MISSING field is a failure, never a default.

    This is the single most important helper in the file. Every gate reads its
    inputs through it, so an evidence file that simply does not carry a field
    can never be silently treated as if it carried a falsy one.
    """
    node = payload
    for part in path.split("."):
        if isinstance(node, dict) and part in node:
            node = node[part]
        else:
            problems.append("%s: missing field %s" % (where, path))
            return None
    return node


def expect(actual, wanted, label: str, problems: list) -> bool:
    """Exact equality, recorded on failure."""
    if actual != wanted:
        problems.append("%s: found %r, expected %r" % (label, actual, wanted))
        return False
    return True


# --------------------------------------------------------------------- pins --

def r3_go_pins() -> dict:
    """The 92 upstream labels the R3 architect GO pins, read from the GO.

    The GO itself is pinned by hash, so reading its map is not trust: a changed
    GO fails before its contents are used.
    """
    path = HERE / R3_GO_NAME
    if not path.is_file():
        raise AttestationError("the R3 architect GO is missing: %s" % path)
    actual = sha256_file(path)
    if actual != PIN_R3_GO:
        raise AttestationError("the R3 architect GO changed: expected %s, "
                               "found %s" % (PIN_R3_GO, actual))
    payload = json.loads(path.read_text(encoding="utf-8"))
    pins = payload.get("pinned_artefacts_sha256")
    if not isinstance(pins, dict) or len(pins) != R3_GO_PIN_COUNT:
        raise AttestationError("the R3 GO pins %s artefacts, expected %d"
                               % (len(pins) if isinstance(pins, dict) else None,
                                  R3_GO_PIN_COUNT))
    return dict(pins)


def resolve(label: str) -> pathlib.Path:
    """A pinned label to a path. baseline_MLP/ and runs/ are repo-anchored."""
    if label.startswith("baseline_MLP/"):
        return BASELINE.parent / label
    if label.startswith("runs/"):
        return REPO / "Trajectory Generator" / label
    return HERE / label


def all_pins() -> dict:
    """Every label this stage verifies: the R3 GO's 92 plus 45 additional."""
    pins = r3_go_pins()
    for label, digest in PIN_ADDITIONAL.items():
        if label in pins and pins[label] != digest:
            raise AttestationError("pin conflict for %s: the R3 GO says %s, "
                                   "this stage says %s"
                                   % (label, pins[label], digest))
        pins[label] = digest
    pins[PREREG_NAME] = PIN_PREREG
    pins[PREREG_REV1_NAME] = PIN_PREREG_REV1
    return pins


def go_required_labels() -> list:
    """Every label an architect GO must pin.

    This is all_pins() PLUS this file and its suite. Those two cannot carry
    their own hash - a file cannot contain its own digest - so they are pinned
    by the GO alone, exactly as the R3 package pins its runner and suite. Their
    absence from the GO would leave the two files that decide the verdict free
    to change between approval and execution, which is precisely the hole this
    closes.
    """
    return sorted(set(all_pins()) | {RUNNER_NAME, TEST_NAME})


def check_pins() -> dict:
    """Re-hash every pinned artefact. Pure: reads files, writes nothing."""
    pins = all_pins()
    results = []
    for label in sorted(pins):
        path = resolve(label)
        if not path.is_file():
            results.append({"artefact": label, "ok": False, "why": "missing"})
            continue
        actual = sha256_file(path)
        results.append({"artefact": label,
                        "ok": actual == pins[label],
                        "why": "matches" if actual == pins[label]
                        else "HASH MISMATCH",
                        "expected": pins[label], "actual": actual})
    return {"checked": len(results),
            "matching": sum(1 for item in results if item["ok"]),
            "results": results,
            "problems": ["%s: %s" % (item["artefact"], item["why"])
                         for item in results if not item["ok"]]}


def verify_committed_map(leaf_rel: str, receipt_name: str, key: str) -> dict:
    """Re-hash every file a J19 receipt content-addresses in its own leaf.

    The receipt is itself pinned, so its map is trustworthy once its hash
    matches. This turns 'the receipt says the cells are these bytes' into a
    measurement over all of them, without embedding 198 more literals.
    """
    receipt = read_json(HERE / leaf_rel / receipt_name)
    committed = receipt.get(key) or {}
    problems = []
    for name in sorted(committed):
        path = HERE / leaf_rel / name
        if not path.is_file():
            problems.append("%s/%s is missing" % (leaf_rel, name))
            continue
        if sha256_file(path) != committed[name]:
            problems.append("%s/%s changed" % (leaf_rel, name))
    return {"leaf": leaf_rel, "files": len(committed),
            "ok": not problems, "problems": problems}


# ------------------------------------------------------------------ the gates --

def evidence() -> dict:
    """Every committed artefact this stage reads, loaded once."""
    return {
        "j19a_result": read_json(HERE / J19A_LEAF_REL / "v26c_j19a_result.json"),
        "j19a_receipt": read_json(
            HERE / J19A_LEAF_REL / "v26c_j19a_single_reproduction_receipt.json"),
        "j19b": read_json(
            HERE / J19B_LEAF_REL / "v26c_j19b_closed_loop_receipt.json"),
        "j19c": read_json(
            HERE / J19C_LEAF_REL / "v26c_j19c_heldout_g_i_receipt.json"),
        "k1r1": read_json(HERE / K1R1_LEAF_REL / "v26c_j20_k1r1_result.json"),
        "warmup_result": read_json(
            HERE / WARMUP_LEAF_REL / "v26c_j20_critic_warmup_result.json"),
        "warmup_receipt": read_json(
            HERE / WARMUP_LEAF_REL / "v26c_j20_critic_warmup_receipt.json"),
        "warmup_meta": read_json(
            HERE / WARMUP_LEAF_REL / "checkpoint_last_meta.json"),
        "warmup_rows": [json.loads(line) for line in
                        (HERE / WARMUP_LEAF_REL / "train_iterations.jsonl"
                         ).read_text(encoding="utf-8").splitlines()
                        if line.strip()],
        "r3_result": read_json(
            HERE / R3_LEAF_REL / "v26c_j20_restore_audit_result.json"),
        "r3_receipt": read_json(
            HERE / R3_LEAF_REL / "v26c_j20_restore_audit_receipt.json"),
        "r3_commit": read_json(HERE / R3_LEAF_REL / "commit_verification.json"),
        "r3_evidence": read_json(
            HERE / R3_LEAF_REL / "live_optimizer_audit.json"),
        "contract": read_json(HERE / "v26c_penetration_contract_2026-08-26.json"),
        # rev1: the artefacts the measured lineage and the worker-sync proof
        # are read from.
        "j2_receipt": read_json(
            HERE / "j2_runs/j2_base_v26c_2026-08-26_r1/v26c_j2_fit_receipt.json"),
        "j8_receipt": read_json(
            HERE / "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1"
            / "v26c_j8_recovery_fit_receipt.json"),
        "j18_receipt": read_json(
            HERE / J18_LEAF_REL / "v26c_j18_b_only_update_receipt.json"),
        "j19a_manifest": read_json(
            HERE / J19A_LEAF_REL / "rl_module/actor_feature_manifest.json"),
        "transplant_report": read_json(HERE / TRANSPLANT_REPORT_REL),
    }


def cells_of(receipt, expected_ids, where, problems) -> list:
    """The cell list of a J19 receipt, order-checked against its ids."""
    cells = need(receipt, "cells", where, problems) or []
    found = tuple(cell.get("id") for cell in cells)
    if found != tuple(expected_ids):
        problems.append("%s: cell ids are %s, expected %s"
                        % (where, list(found), list(expected_ids)))
    return cells


def gate_t1_lineage(ev, pins, problems) -> dict:
    """July is never an operational parent or training input. rev1.

    The base gated this with a substring scan of pin labels for the word July.
    That is unsound here: the containing directory is literally called
    v26c_july_replica_2026-08-26, so every label contains it. Worse, a label is
    not a lineage - J8 is labelled July-faithful and its parent bytes are the
    August V26 J2 module.

    So this gate no longer reads labels. It requires POSITIVE evidence from J8's
    own receipt that July's dataset, parent, actor and output are NOT
    replicated, and that its operational lineage names the August V26 J2 parent.
    The byte-level chain itself is measured by T17.
    """
    parent = need(ev["j19a_receipt"], "inputs.j18_leaf", "T1", problems)
    expect(parent, J18_LEAF_REL, "T1 J19A parent leaf", problems)
    manifest_label = J19A_LEAF_REL + "/rl_module/actor_feature_manifest.json"
    if manifest_label not in pins:
        problems.append("T1: the 35D actor feature manifest is not pinned")

    faithfulness = need(ev["j8_receipt"], "july_faithfulness", "T1", problems) \
        or {}
    not_replicated = faithfulness.get("does_not_replicate")
    if not isinstance(not_replicated, list):
        problems.append("T1: the J8 receipt records no does_not_replicate list")
        not_replicated = []
    for item in JULY_MUST_NOT_REPLICATE:
        if item not in not_replicated:
            problems.append("T1: J8 does not declare %r among the things it "
                            "does NOT replicate from July" % item)
    operational = faithfulness.get("operational_lineage") or ""
    if "August V26" not in operational or "J2 parent" not in operational:
        problems.append("T1: J8's operational_lineage does not name the August "
                        "V26 J2 parent: %r" % operational)

    width = need(ev["j19a_manifest"], "observation_width", "T1", problems)
    expect(width, STUDENT_WIDTH, "T1 student observation width", problems)
    return {
        "operational_lineage": OPERATIONAL_LINEAGE,
        "student": "one single native 35D actor",
        "student_observation_width": width,
        "j19a_parent_leaf": parent,
        "july_role": "INFORMATIONAL ONLY",
        "july_forbidden_as_operational_parent_or_training_input":
            list(JULY_FORBIDDEN_AS_INPUT),
        "july_allowed_as_disclosed_informational_sources":
            list(JULY_ALLOWED_AS_INFORMATIONAL),
        "j8_does_not_replicate_from_july": not_replicated,
        "j8_operational_lineage": operational,
        "label_is_not_lineage": "J8 carries a July-faithful label and an August "
                                "V26 parent. What decides is the parent bytes, "
                                "which T17 measures.",
    }


def gate_t16_worker_sync(ev, problems) -> dict:
    """The workers really received the intended actor. rev1.

    The base proved this from learners/default_policy/weights_seq_no == 1, a
    counter of how many times the LEARNER published weights. It says nothing
    about whether the EnvRunners received them, how many were checked, or what
    they hold. The real proof is the J20 transplant report's integration
    validation, which checked every runner's actor digest.
    """
    report = ev["transplant_report"]
    iv = need(report, "integration_validation", "T16", problems) or {}
    expect(iv.get("weights_synced_before_first_sample"), True,
           "T16 weights synced before first sample", problems)
    expect(iv.get("env_runner_count_checked"), EXPECTED_ENV_RUNNERS,
           "T16 env runner count checked", problems)
    expect(iv.get("env_runner_actors_exact"), True,
           "T16 env runner actors exact", problems)
    digests = iv.get("env_runner_actor_digests")
    if not isinstance(digests, list):
        problems.append("T16: env_runner_actor_digests is not a list")
        digests = []
    if len(digests) != EXPECTED_ENV_RUNNERS:
        problems.append("T16: %d env runner actor digests, expected %d"
                        % (len(digests), EXPECTED_ENV_RUNNERS))
    wrong = sorted({d for d in digests if d != EXPECTED_ACTOR_DIGEST})
    if wrong:
        problems.append("T16: these env runner actor digests are not the "
                        "expected actor: %s" % wrong)
    for block in ("learner_actor", "saved_initial_actor"):
        entry = iv.get(block)
        if not isinstance(entry, dict):
            problems.append("T16: the transplant report has no %s block"
                            % block)
            continue
        expect(entry.get("exact"), True, "T16 %s exact" % block, problems)
        expect(entry.get("max_abs_diff"), 0.0, "T16 %s max_abs_diff" % block,
               problems)
        expect(entry.get("expected_digest"), EXPECTED_ACTOR_DIGEST,
               "T16 %s expected digest" % block, problems)
        expect(entry.get("actual_digest"), EXPECTED_ACTOR_DIGEST,
               "T16 %s actual digest" % block, problems)
        expect(entry.get("missing_keys"), [], "T16 %s missing keys" % block,
               problems)
    rows = ev["warmup_rows"]
    seq = (rows[0].get("learner_metrics") or {}).get(
        "learners/default_policy/weights_seq_no") if rows else None
    return {
        "weights_synced_before_first_sample":
            iv.get("weights_synced_before_first_sample"),
        "env_runner_count_checked": iv.get("env_runner_count_checked"),
        "env_runner_actors_exact": iv.get("env_runner_actors_exact"),
        "env_runner_actor_digests_count": len(digests),
        "env_runner_actor_digests_all_expected": not wrong,
        "learner_actor": {k: (iv.get("learner_actor") or {}).get(k)
                          for k in ("exact", "max_abs_diff", "expected_digest",
                                    "actual_digest")},
        "saved_initial_actor": {k: (iv.get("saved_initial_actor") or {}).get(k)
                                for k in ("exact", "max_abs_diff",
                                          "expected_digest", "actual_digest")},
        "weights_seq_no_supporting_telemetry": seq,
        "weights_seq_no_is_not_the_proof": "rev1 demotes it. It is recorded "
                                           "because it corroborates a single "
                                           "publication, and it gates nothing.",
    }


def gate_t17_measured_lineage(ev, problems) -> dict:
    """Every operational parent, by BYTES, from August V26 to the checkpoint."""
    links = {}

    # August V26 imitation -> J2, recorded by the J2 receipt.
    j2 = ev["j2_receipt"]
    expect(need(j2, "derivation.parent.module_state_sha256", "T17", problems),
           SHA_AUGUST_MODULE, "T17 August source module", problems)
    expect(need(j2, "derivation.parent.manifest39_sha256", "T17", problems),
           SHA_AUGUST_MANIFEST39, "T17 August source manifest", problems)
    expect(need(j2, "derivation.parent.width", "T17", problems), AUGUST_WIDTH,
           "T17 August source width", problems)
    expect(need(j2, "derivation.parent.path", "T17", problems),
           "Trajectory Generator/" + AUGUST_SOURCE_REL,
           "T17 August source path", problems)
    # The output map is keyed by file name, and those names contain dots, so
    # they are fetched directly rather than through the dotted-path helper.
    j2_outputs = need(j2, "output_files_sha256", "T17", problems) or {}
    expect(j2_outputs.get("module_state.pkl"), SHA_J2_MODULE,
           "T17 J2 output module", problems)
    expect(j2_outputs.get("actor_feature_manifest.json"), SHA_J2_MANIFEST35,
           "T17 J2 output 35D manifest", problems)
    links["august_to_j2"] = {"parent": SHA_AUGUST_MODULE,
                             "child": SHA_J2_MODULE,
                             "width": "%d -> %d" % (AUGUST_WIDTH,
                                                    STUDENT_WIDTH)}

    # J2 -> J8, recorded by the J8 receipt.
    j8 = ev["j8_receipt"]
    expect(need(j8, "inputs.parent.module_state_sha256", "T17", problems),
           SHA_J2_MODULE, "T17 J8 parent module", problems)
    expect((j8.get("output") or {}).get("files", {}).get(
        "rl_module/module_state.pkl"), SHA_J8_MODULE, "T17 J8 output module",
        problems)
    links["j2_to_j8"] = {"parent": SHA_J2_MODULE, "child": SHA_J8_MODULE}

    # J8 -> J18, recorded by the J18 receipt.
    j18 = ev["j18_receipt"]
    expect(need(j18, "inputs.parent.actual_byte_sha256", "T17", problems),
           SHA_J8_MODULE, "T17 J18 parent module", problems)
    expect(need(j18, "inputs.parent.actor_digest", "T17", problems),
           DIGEST_J8_ACTOR, "T17 J18 parent actor digest", problems)
    links["j8_to_j18"] = {"parent": SHA_J8_MODULE,
                          "parent_actor_digest": DIGEST_J8_ACTOR,
                          "note": "J18 persisted zero actors; its selection was "
                                  "fail-closed. It contributes the constrained "
                                  "update protocol, not an actor."}

    # J18 / J8 -> J19A, recorded by the J19A receipt and its manifest.
    receipt = ev["j19a_receipt"]
    expect(need(receipt, "inputs.parent_j8_sha256", "T17", problems),
           SHA_J8_MODULE, "T17 J19A parent J8", problems)
    expect(need(receipt, "inputs.parent_of_j8_sha256", "T17", problems),
           SHA_J2_MODULE, "T17 J19A grandparent J2", problems)
    manifest = ev["j19a_manifest"]
    expect(need(manifest, "derived_from.parent", "T17", problems), "J8",
           "T17 J19A manifest parent", problems)
    expect(need(manifest, "derived_from.parent_module_state_sha256", "T17",
                problems), SHA_J8_MODULE, "T17 J19A manifest parent module",
           problems)
    expect(need(manifest, "source_actor_digest", "T17", problems),
           DIGEST_J8_ACTOR, "T17 J19A source actor digest", problems)
    expect(need(manifest, "actor_digest", "T17", problems),
           EXPECTED_ACTOR_DIGEST, "T17 J19A output actor digest", problems)
    expect(need(manifest, "module_state_sha256", "T17", problems),
           SHA_J19A_MODULE, "T17 J19A output module", problems)
    links["j18_j8_to_j19a"] = {"parent": SHA_J8_MODULE,
                               "parent_actor_digest": DIGEST_J8_ACTOR,
                               "child_module": SHA_J19A_MODULE,
                               "child_actor_digest": EXPECTED_ACTOR_DIGEST}

    # J19A -> J20 warm-up, recorded by the transplant report.
    report = ev["transplant_report"]
    expect(need(report, "source_actor_digest", "T17", problems),
           EXPECTED_ACTOR_DIGEST, "T17 warm-up source actor", problems)
    expect(need(report, "target_actor_digest_after", "T17", problems),
           EXPECTED_ACTOR_DIGEST, "T17 warm-up target actor", problems)
    expect(need(report,
                "source_actor_feature_manifest.validated_actor_digest", "T17",
                problems), EXPECTED_ACTOR_DIGEST,
           "T17 warm-up manifest validated digest", problems)
    links["j19a_to_warmup"] = {"source_actor_digest": EXPECTED_ACTOR_DIGEST,
                               "target_actor_digest": EXPECTED_ACTOR_DIGEST}
    return {"links": links, "measured_not_declared": True,
            "chain": OPERATIONAL_LINEAGE}


def file_set(root: pathlib.Path) -> list:
    """Every regular file under a directory, as sorted relative paths."""
    if not root.is_dir():
        return []
    return sorted(str(p.relative_to(root)) for p in root.rglob("*")
                  if p.is_file())


def check_leaf_set(label: str, actual: list, expected: list,
                   problems: list) -> dict:
    """The file set of a leaf must be EXACTLY the expected one.

    Taken as arguments rather than read inside, so a test can supply a
    synthetic set and never has to create, delete or rename anything under a
    real leaf.
    """
    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    if missing:
        problems.append("T18: %s is missing %s" % (label, missing))
    if extra:
        problems.append("T18: %s holds unexpected files %s" % (label, extra))
    if len(actual) != len(expected):
        problems.append("T18: %s holds %d files, expected %d"
                        % (label, len(actual), len(expected)))
    return {"files": len(actual), "missing": missing, "extra": extra,
            "exact": not missing and not extra}


def check_committed_map(label: str, receipt, expected_size: int,
                        problems: list):
    """The committed map must be a PRESENT dict of the exact expected size.

    The base re-hashed whatever the map held. An emptied or missing map would
    have verified vacuously, which is the difference between checking evidence
    and checking that a loop ran.
    """
    committed = receipt.get("committed_files_sha256") if isinstance(
        receipt, dict) else None
    if committed is None:
        problems.append("T18: %s has no committed_files_sha256 map" % label)
        return None
    if not isinstance(committed, dict):
        problems.append("T18: %s committed_files_sha256 is a %s, not a map"
                        % (label, type(committed).__name__))
        return None
    if not committed:
        problems.append("T18: %s committed_files_sha256 is EMPTY" % label)
        return committed
    if len(committed) != expected_size:
        problems.append("T18: %s committed_files_sha256 holds %d entries, "
                        "expected %d" % (label, len(committed), expected_size))
    return committed


def gate_t18_leaf_integrity(ev, problems) -> dict:
    """Exact file sets and non-vacuous committed maps. rev1."""
    findings = {}
    for label, leaf_rel, receipt in (
            ("j19b", J19B_LEAF_REL, ev["j19b"]),
            ("j19c", J19C_LEAF_REL, ev["j19c"])):
        committed = check_committed_map(label, receipt,
                                        EXPECTED_COMMITTED_MAP_SIZES[label],
                                        problems)
        expected = sorted(list(committed or {})
                          + ["commit_verification.json",
                             "v26c_j19b_closed_loop_receipt.json"
                             if label == "j19b"
                             else "v26c_j19c_heldout_g_i_receipt.json"])
        actual = file_set(HERE / leaf_rel)
        findings[label] = check_leaf_set(label, actual, expected, problems)
        findings[label]["committed_map_entries"] = len(committed or {})
        expect(len(actual), EXPECTED_LEAF_FILE_COUNTS[label],
               "T18 %s leaf file count" % label, problems)
    findings["j19a"] = check_leaf_set(
        "j19a", file_set(HERE / J19A_LEAF_REL), list(J19A_EXPECTED_FILES),
        problems)
    findings["r3"] = check_leaf_set(
        "r3", file_set(HERE / R3_LEAF_REL), list(R3_EXPECTED_FILES), problems)
    for label in ("j19a", "r3"):
        expect(findings[label]["files"], EXPECTED_LEAF_FILE_COUNTS[label],
               "T18 %s leaf file count" % label, problems)
    return findings


def gate_t2_j19a(ev, problems) -> dict:
    """Reproducibility and offline eligibility."""
    result, receipt = ev["j19a_result"], ev["j19a_receipt"]
    expect(need(receipt, "verdict", "T2", problems), "PASS", "T2 J19A verdict",
           problems)
    for field in ("reproducibility_ok", "eligibility_ok", "exact_fields_ok"):
        expect(need(receipt, field, "T2", problems), True, "T2 " + field,
               problems)
    expect(need(receipt, "exact_fields_compared", "T2", problems), 36,
           "T2 exact fields compared", problems)
    expect(need(receipt, "exact_fields_failed", "T2", problems), [],
           "T2 exact fields failed", problems)
    expect(need(receipt, "non_finite_count", "T2", problems), 0,
           "T2 non finite count", problems)
    expect(need(receipt, "promotion_blocked_by_non_finite", "T2", problems),
           False, "T2 promotion blocked by non finite", problems)
    expect(need(receipt, "rollout_performed", "T2", problems), False,
           "T2 J19A rollout performed", problems)
    expect(need(result, "actor_promoted", "T2", problems), True,
           "T2 actor promoted", problems)
    expect(need(result, "eligibility.ok", "T2", problems), True,
           "T2 eligibility ok", problems)
    expect(need(result, "exact_fields.ok", "T2", problems), True,
           "T2 exact fields ok", problems)
    binding = need(result, "eligibility.binding", "T2", problems) or []
    ids = tuple(item.get("id") for item in binding)
    if sorted(ids) != sorted(J19A_BINDING_IDS):
        problems.append("T2: binding criteria are %s, expected %s"
                        % (list(ids), list(J19A_BINDING_IDS)))
    for item in binding:
        if item.get("binding") is not True or item.get("passed") is not True:
            problems.append("T2: binding criterion %s did not pass: %r"
                            % (item.get("id"), item))
    diagnostic = need(result, "eligibility.diagnostic_only", "T2", problems) or []
    for item in diagnostic:
        if item.get("binding") is not False:
            problems.append("T2: %s is marked binding but belongs to the "
                            "diagnostic set" % item.get("id"))
    return {
        "verdict": receipt.get("verdict"),
        "binding_criteria": {item.get("id"): item.get("measured")
                             for item in binding},
        "binding_all_passed": all(item.get("passed") is True
                                  for item in binding),
        "diagnostic_only_recorded_not_gated": {
            item.get("id"): item.get("measured") for item in diagnostic},
        "exact_fields": receipt.get("exact_fields_compared"),
        "rollout_performed": receipt.get("rollout_performed"),
    }


def gate_j19_cells(receipt, expected_ids, total, where, problems) -> dict:
    """Shared body of T3 and T4: N of N behavioural and telemetry-valid."""
    expect(need(receipt, "verdict", where, problems), "PASS",
           where + " verdict", problems)
    expect(need(receipt, "aggregate_pass", where, problems), True,
           where + " aggregate pass", problems)
    expect(need(receipt, "actor_unchanged", where, problems), True,
           where + " actor unchanged", problems)
    expect(need(receipt, "cells_total", where, problems), total,
           where + " cells total", problems)
    expect(need(receipt, "cells_behavioural_pass", where, problems), total,
           where + " behavioural pass", problems)
    expect(need(receipt, "cells_telemetry_valid", where, problems), total,
           where + " telemetry valid", problems)
    cells = cells_of(receipt, expected_ids, where, problems)
    per_cell = {}
    for cell in cells:
        cid = cell.get("id")
        for field, wanted in (("verdict", "PASS"), ("behavioural_pass", True),
                              ("telemetry_valid", True)):
            expect(cell.get(field), wanted, "%s cell %s %s" % (where, cid, field),
                   problems)
        per_cell[cid] = {"verdict": cell.get("verdict"),
                         "behavioural_pass": cell.get("behavioural_pass"),
                         "telemetry_valid": cell.get("telemetry_valid")}
    return {"verdict": receipt.get("verdict"), "cells": per_cell,
            "actor_unchanged": receipt.get("actor_unchanged")}


def gate_t5_penetration(ev, problems) -> dict:
    """The current contract, and no hard-binding breach anywhere."""
    contract = ev["contract"]
    actual = sha256_file(HERE / "v26c_penetration_contract_2026-08-26.json")
    expect(actual, EXPECTED_PENETRATION_CONTRACT_SHA, "T5 contract sha256",
           problems)
    for band, wanted in BANDS_M.items():
        key = CONTRACT_BAND_KEYS[band]
        expect(need(contract, "bands.%s.value_m" % key, "T5", problems), wanted,
               "T5 band " + band, problems)
        expect(need(contract, "bands.%s.binding" % key, "T5", problems),
               CONTRACT_BINDING[band], "T5 band %s binding role" % band,
               problems)
    per_cell = {}
    for receipt, ids, where in ((ev["j19b"], J19B_CELLS, "T5/J19B"),
                                (ev["j19c"], J19C_CELLS, "T5/J19C")):
        expect(need(receipt, "penetration_authority.contract_sha256", where,
                    problems), EXPECTED_PENETRATION_CONTRACT_SHA,
               where + " penetration authority", problems)
        for cell in cells_of(receipt, ids, where, problems):
            cid = cell.get("id")
            pen = cell.get("penetration") or {}
            counts = pen.get("counts") or {}
            flags = pen.get("flags") or {}
            expect(pen.get("binding_pass"), True,
                   "T5 cell %s binding pass" % cid, problems)
            expect(counts.get("above_hard_binding"), 0,
                   "T5 cell %s samples above the 28 mm hard band" % cid,
                   problems)
            expect(flags.get("above_hard_binding"), False,
                   "T5 cell %s above hard binding flag" % cid, problems)
            per_cell[cid] = {
                "max_penetration_m": pen.get("max_penetration_m"),
                "band": pen.get("band"),
                "samples": pen.get("samples"),
                "above_soft_diagnostic": counts.get("above_soft_diagnostic"),
                "at_or_above_july_legacy": counts.get("at_or_above_july_legacy"),
                "above_hard_binding": counts.get("above_hard_binding"),
                "binding_pass": pen.get("binding_pass"),
            }
    # The disclosed diagnostics, reproduced rather than asserted.
    above_soft = sorted(cid for cid, c in per_cell.items()
                        if (c["above_soft_diagnostic"] or 0) > 0)
    at_july = sorted(cid for cid, c in per_cell.items()
                     if (c["at_or_above_july_legacy"] or 0) > 0)
    if sorted(at_july) != sorted(JULY_LEGACY_CELLS):
        problems.append("T5: the cells at or above the 25 mm July band are %s, "
                        "and the preregistration discloses %s"
                        % (at_july, list(JULY_LEGACY_CELLS)))
    return {
        "bands_m": BANDS_M,
        "contract_sha256": actual,
        "cells": per_cell,
        "accepted_diagnostics": {
            "cells_above_soft_20mm": above_soft,
            "cells_at_or_above_july_25mm": at_july,
            "cells_above_hard_28mm": sorted(
                cid for cid, c in per_cell.items()
                if (c["above_hard_binding"] or 0) > 0),
            "soft_and_july_are_diagnostic_only": True,
            "sole_binding_threshold_m": BANDS_M["hard_binding"],
        },
        "no_hard_breach": all((c["above_hard_binding"] or 0) == 0
                              for c in per_cell.values()),
    }


def gate_t6_counters(ev, problems) -> dict:
    """FSM v3 and morphology incident counters, all zero."""
    per_cell = {}
    for receipt, ids, where in ((ev["j19b"], J19B_CELLS, "T6/J19B"),
                                (ev["j19c"], J19C_CELLS, "T6/J19C")):
        for cell in cells_of(receipt, ids, where, problems):
            cid = cell.get("id")
            telemetry = cell.get("telemetry") or {}
            row = {}
            for counter in ZERO_CELL_COUNTERS:
                if counter not in telemetry:
                    problems.append("T6: cell %s has no %s counter"
                                    % (cid, counter))
                    continue
                row[counter] = telemetry[counter]
                expect(telemetry[counter], 0, "T6 cell %s %s" % (cid, counter),
                       problems)
            per_cell[cid] = row
    health = need(ev["warmup_result"], "measurements.training_health", "T6",
                  problems) or {}
    for field in ZERO_HEALTH_FIELDS:
        if field not in health:
            problems.append("T6: the warm-up training health has no %s" % field)
            continue
        expect(health[field], 0.0, "T6 warm-up " + field, problems)
    expect(health.get("observed_rows"), EXPECTED_SAMPLED_STEPS,
           "T6 warm-up observed rows", problems)
    return {"cells": per_cell, "warmup_training_health": health,
            "all_zero": not problems}


def gate_t7_k1r1(ev, problems) -> dict:
    """The non-degenerate gradient amendment."""
    k = ev["k1r1"]
    expect(need(k, "verdict", "T7", problems), "PASS", "T7 K1R1 verdict",
           problems)
    expect(need(k, "amended_checks_passed", "T7", problems), 12,
           "T7 K1R1 checks passed", problems)
    expect(need(k, "amended_checks_total", "T7", problems), 12,
           "T7 K1R1 checks total", problems)
    expect(need(k, "promotion", "T7", problems), "NONE", "T7 K1R1 promotion",
           problems)
    expect(need(k, "next_stage_authorized", "T7", problems), False,
           "T7 K1R1 next stage", problems)
    return {"verdict": k.get("verdict"),
            "checks": "%s/%s" % (k.get("amended_checks_passed"),
                                 k.get("amended_checks_total"))}


def gate_t8_warmup(ev, problems) -> dict:
    """The critic-only warm-up: 12/12, actor and log-std bit exact."""
    result = ev["warmup_result"]
    expect(need(result, "verdict", "T8", problems), "AWAITING_RESTORE_AUDIT",
           "T8 warm-up verdict", problems)
    expect(need(result, "gates_passed", "T8", problems), 12,
           "T8 warm-up gates passed", problems)
    expect(need(result, "gates_total", "T8", problems), 12,
           "T8 warm-up gates total", problems)
    expect(need(result, "gates_failed", "T8", problems), [],
           "T8 warm-up gates failed", problems)
    gates = need(result, "gates", "T8", problems) or {}
    for name in WARMUP_GATE_NAMES:
        if name not in gates:
            problems.append("T8: the warm-up records no gate %s" % name)
        elif gates[name] is not True:
            problems.append("T8: warm-up gate %s did not pass" % name)
    actor = need(result, "actor", "T8", problems) or {}
    expect(actor.get("actor_all_byte_identical"), True,
           "T8 actor byte identical", problems)
    expect(actor.get("actor_digest"), EXPECTED_ACTOR_DIGEST, "T8 actor digest",
           problems)
    expect(actor.get("actor_digest_matches"), True, "T8 actor digest matches",
           problems)
    expect(actor.get("logstd_rows_byte_identical"), True,
           "T8 log-std byte identical", problems)
    expect(actor.get("sigma"), [EXPECTED_SIGMA, EXPECTED_SIGMA], "T8 sigma",
           problems)
    expect(actor.get("trained_module_sha256"), EXPECTED_MODULE_STATE_SHA,
           "T8 trained module sha256", problems)
    per_key = actor.get("actor_per_key_byte_identical") or {}
    if not per_key or not all(per_key.values()):
        problems.append("T8: not every actor key is byte-identical: %r"
                        % sorted(k for k, v in per_key.items() if not v))
    measurements = need(result, "measurements", "T8", problems) or {}
    before = measurements.get("critic_digest_before")
    after = measurements.get("critic_digest_after")
    if before is None or after is None or before == after:
        problems.append("T8: the critic digest did not change: %r -> %r"
                        % (before, after))
    expect(after, EXPECTED_CRITIC_DIGEST, "T8 critic digest after", problems)
    expect(measurements.get("num_env_steps_sampled_lifetime"),
           EXPECTED_SAMPLED_STEPS, "T8 sampled lifetime", problems)
    expect(measurements.get("steps_sampled_this_iteration"),
           EXPECTED_SAMPLED_STEPS, "T8 steps this iteration", problems)
    expect(measurements.get("mean_kl_loss"), 0.0, "T8 mean kl loss", problems)
    expect(measurements.get("iterations_completed_this_process"), 1,
           "T8 iterations completed", problems)
    expect(measurements.get("child_returncode"), 0, "T8 child returncode",
           problems)
    checkpoint = need(result, "checkpoint", "T8", problems) or {}
    for field in ("present", "ok", "only_the_critic_has_adam_moments"):
        expect(checkpoint.get(field), True, "T8 checkpoint " + field, problems)
    expect(checkpoint.get("learner_module_key_count"), EXPECTED_MODULE_KEYS,
           "T8 module key count", problems)
    expect(checkpoint.get("optimizer_adam_state_indices"),
           EXPECTED_ADAM_INDICES, "T8 Adam indices", problems)
    expect(checkpoint.get("optimizer_param_group_size"),
           EXPECTED_PARAM_GROUP_SIZE, "T8 param group size", problems)
    # rev1: weights_seq_no is SUPPORTING TELEMETRY, not the worker-sync proof.
    # It counts how many times the LEARNER published weights and says nothing
    # about what the EnvRunners received. The binding proof is T16, which reads
    # the transplant report's per-runner digest validation. Only the row count
    # and the iteration number remain binding here.
    rows = ev["warmup_rows"]
    if len(rows) != 1:
        problems.append("T8: the warm-up committed %d iteration rows, expected 1"
                        % len(rows))
    seq = None
    if rows:
        seq = (rows[0].get("learner_metrics")
               or {}).get("learners/default_policy/weights_seq_no")
        expect(rows[0].get("iteration"), 1, "T8 committed iteration", problems)
    return {
        "verdict": result.get("verdict"),
        "gates": "%s/%s" % (result.get("gates_passed"),
                            result.get("gates_total")),
        "actor_digest": actor.get("actor_digest"),
        "logstd_byte_identical": actor.get("logstd_rows_byte_identical"),
        "sigma": actor.get("sigma"),
        "critic_digest_before": before, "critic_digest_after": after,
        "critic_changed": before != after,
        "sampled_steps": measurements.get("num_env_steps_sampled_lifetime"),
        "worker_weights_seq_no_supporting_telemetry_only": seq,
        "worker_sync_binding_proof_is": "T16, from the transplant report",
        "checkpoint": {k: checkpoint.get(k) for k in
                       ("learner_module_key_count",
                        "optimizer_adam_state_indices",
                        "optimizer_param_group_size",
                        "only_the_critic_has_adam_moments")},
    }


def gate_t9_r3(ev, problems) -> dict:
    """The restore audit: 13/13, G9 closed, markers correct."""
    result, receipt = ev["r3_result"], ev["r3_receipt"]
    expect(need(result, "stage", "T9", problems), "V26C_J20_RESTORE_AUDIT_R3",
           "T9 R3 stage", problems)
    expect(need(result, "verdict", "T9", problems), "RESTORE_AUDIT_PASS",
           "T9 R3 verdict", problems)
    expect(need(result, "gates_passed", "T9", problems), 13,
           "T9 R3 gates passed", problems)
    expect(need(result, "gates_total", "T9", problems), 13,
           "T9 R3 gates total", problems)
    expect(need(result, "gates_failed", "T9", problems), [],
           "T9 R3 gates failed", problems)
    expect(need(result, "gates_not_evaluated", "T9", problems), [],
           "T9 R3 gates not evaluated", problems)
    expect(need(result, "live_restore.completed", "T9", problems), True,
           "T9 R3 live restore completed", problems)
    gates = need(result, "gates", "T9", problems) or {}
    if len(gates) != 13 or not all(gates.values()):
        problems.append("T9: the thirteen R3 gates are not all true: %r"
                        % sorted(k for k, v in gates.items() if not v))
    expect(need(receipt, "g9_closed", "T9", problems), True, "T9 g9 closed",
           problems)
    expect(need(receipt, "child_returncode", "T9", problems), 0,
           "T9 R3 child returncode", problems)
    expect(need(receipt, "source_tree_unchanged", "T9", problems), True,
           "T9 R3 source tree unchanged", problems)
    expect(need(receipt, "pins_unchanged", "T9", problems), True,
           "T9 R3 pins unchanged", problems)
    commit = ev["r3_commit"]
    expect(need(commit, "ok", "T9", problems), True, "T9 R3 commit ok", problems)
    expect(need(commit, "problems", "T9", problems), [], "T9 R3 commit problems",
           problems)
    expect(need(commit, "marker", "T9", problems), "RESTORE_AUDIT_PASSED",
           "T9 R3 commit marker", problems)
    if not (HERE / R3_LEAF_REL / "RESTORE_AUDIT_PASSED").is_file():
        problems.append("T9: the R3 leaf does not carry RESTORE_AUDIT_PASSED")
    if (HERE / R3_LEAF_REL / INVALID_MARKER).exists():
        problems.append("T9: the R3 leaf still carries %s" % INVALID_MARKER)
    for leaf in (R1_LEAF_REL, R2_LEAF_REL):
        if not (HERE / leaf / INVALID_MARKER).is_file():
            problems.append("T9: %s no longer carries %s" % (leaf,
                                                             INVALID_MARKER))
    return {"verdict": result.get("verdict"),
            "gates": "%s/%s" % (result.get("gates_passed"),
                                result.get("gates_total")),
            "g9_closed": receipt.get("g9_closed"),
            "live_restore_completed": (result.get("live_restore")
                                       or {}).get("completed")}


def gate_t10_live_optimizer(ev, problems) -> dict:
    """The live optimizer evidence, including rev7's learning-rate gate."""
    e = ev["r3_evidence"]
    expect(need(e, "kind", "T10", problems),
           "LIVE OPTIMIZER RESTORE AUDIT, CANONICALISED", "T10 evidence kind",
           problems)
    expect(need(e, "stage_marker", "T10", problems), "after_restore",
           "T10 stage marker", problems)
    expect(need(e, "exact", "T10", problems), True, "T10 exact", problems)
    expect(need(e, "problems", "T10", problems), [], "T10 problems", problems)
    expect(need(e, "source_state_sha256", "T10", problems),
           EXPECTED_LEARNER_STATE_SHA, "T10 source state sha256", problems)
    expect(need(e, "canonical.difference_count", "T10", problems), 0,
           "T10 canonical difference count", problems)
    expect(need(e, "canonical.differences", "T10", problems), [],
           "T10 canonical differences", problems)
    expect(need(e, "canonical.digests_match", "T10", problems), True,
           "T10 canonical digests match", problems)
    expect(need(e, "canonical.param_groups.exact", "T10", problems), True,
           "T10 canonical param groups exact", problems)
    expect(need(e, "accepted_equivalences_are_exactly_the_expected_eight",
                "T10", problems), True, "T10 accepted equivalences", problems)
    expect(need(e, "unexplained_canonical_paths", "T10", problems), [],
           "T10 unexplained canonical paths", problems)
    expect(need(e, "moment_digests_match", "T10", problems), True,
           "T10 moment digests match", problems)
    for side in ("source", "live"):
        expect(need(e, "moments_unchanged_by_canonicalisation_" + side, "T10",
                    problems), True, "T10 moments unchanged " + side, problems)
    expect(need(e, "conversion.call_site_verified", "T10", problems), True,
           "T10 conversion call site", problems)
    for side in ("source", "live"):
        expect(need(e, "conversion_fixed_point_" + side, "T10", problems), True,
               "T10 fixed point " + side, problems)
    lr = need(e, "learning_rate_observation", "T10", problems)
    if not isinstance(lr, dict):
        problems.append("T10: the rev7 learning-rate observation is missing")
        lr = {}
    expect(lr.get("matches"), True, "T10 learning rate matches", problems)
    expect(lr.get("gated"), True, "T10 learning rate gated", problems)
    expect(lr.get("canonical_dtype"), "float32", "T10 learning rate dtype",
           problems)
    canonical_source = lr.get("canonical_source_lr")
    if not isinstance(canonical_source, str) \
            or not canonical_source.startswith("('t'"):
        problems.append("T10: canonical_source_lr is not a populated tensor "
                        "node: %r" % canonical_source)
    expect(lr.get("canonical_live_lr_before_reapply"), canonical_source,
           "T10 canonical lr agreement", problems)
    if lr.get("live_lr_before_reapply") is None:
        problems.append("T10: no pre-reapply learning rate was recorded")
    return {
        "exact": e.get("exact"),
        "canonical_digest": (e.get("canonical") or {}).get("digest_source"),
        "canonical_differences": (e.get("canonical") or {}).get(
            "difference_count"),
        "raw_differences_reproduce_r2_eight": (e.get("raw") or {}).get(
            "reproduces_r2_eight"),
        "accepted_equivalence_paths": e.get("accepted_equivalence_paths"),
        "learning_rate_observation": lr,
        "conversion": {k: (e.get("conversion") or {}).get(k)
                       for k in ("ray_version", "torch_version",
                                 "call_site_verified")},
    }


def gate_t11_warmup_leaf_immutable(pins, problems) -> dict:
    """The warm-up leaf keeps RESTORE_AUDIT_PENDING and every pinned byte."""
    marker_label = WARMUP_LEAF_REL + "/RESTORE_AUDIT_PENDING"
    if marker_label not in pins:
        problems.append("T11: the pending marker is not pinned")
    marker = HERE / WARMUP_LEAF_REL / "RESTORE_AUDIT_PENDING"
    if not marker.is_file():
        problems.append("T11: the warm-up leaf lost RESTORE_AUDIT_PENDING")
    elif marker_label in pins and sha256_file(marker) != pins[marker_label]:
        problems.append("T11: RESTORE_AUDIT_PENDING changed")
    for forbidden in (ATTESTED_MARKER, "RESTORE_AUDIT_PASSED", INVALID_MARKER):
        if (HERE / WARMUP_LEAF_REL / forbidden).exists():
            problems.append("T11: the warm-up leaf carries %s, which it must "
                            "not" % forbidden)
    return {"leaf": WARMUP_LEAF_REL, "marker": "RESTORE_AUDIT_PENDING",
            "marker_present": marker.is_file(),
            "g9_closure_source": "the R3 receipt, never this leaf"}


def gate_t12_checkpoint(ev, pins, problems) -> dict:
    """All 24 checkpoint files present and pinned; meta reads iteration 1."""
    root = HERE / CHECKPOINT_REL
    if not root.is_dir():
        problems.append("T12: the checkpoint directory is missing")
        found = []
    else:
        found = sorted(p for p in root.rglob("*") if p.is_file())
    if len(found) != EXPECTED_CHECKPOINT_FILES:
        problems.append("T12: the checkpoint holds %d files, expected %d"
                        % (len(found), EXPECTED_CHECKPOINT_FILES))
    unpinned = [str(p.relative_to(HERE)) for p in found
                if str(p.relative_to(HERE)) not in pins]
    if unpinned:
        problems.append("T12: these checkpoint files are not pinned: %s"
                        % unpinned)
    meta = ev["warmup_meta"]
    expect(need(meta, "logical_iteration", "T12", problems), 1,
           "T12 logical iteration", problems)
    expect(need(meta, "rllib_training_iteration", "T12", problems), 1,
           "T12 rllib training iteration", problems)
    return {"directory": CHECKPOINT_REL, "files": len(found),
            "module_state_sha256": EXPECTED_MODULE_STATE_SHA,
            "learner_state_sha256": EXPECTED_LEARNER_STATE_SHA,
            "logical_iteration": meta.get("logical_iteration")}


def gate_t14_self_audit() -> dict:
    """This file constructs no runtime and trains nothing. Measured, not claimed.

    The source is parsed and every import and every call is inspected. A
    forbidden import at ANY level - module or function - or a forbidden call
    name fails the stage. Docstrings and string literals are irrelevant here
    because the walk is over the AST, not over the text.
    """
    source = pathlib.Path(__file__).resolve().read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported = set()
    called = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
        elif isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute):
                called.add(func.attr)
            elif isinstance(func, ast.Name):
                called.add(func.id)
    forbidden_imports = sorted(imported & set(FORBIDDEN_IMPORTS))
    forbidden_calls = sorted(called & set(FORBIDDEN_CALL_NAMES))
    problems = []
    if forbidden_imports:
        problems.append("T14: forbidden imports present: %s"
                        % forbidden_imports)
    if forbidden_calls:
        problems.append("T14: forbidden calls present: %s" % forbidden_calls)
    return {
        "imports": sorted(imported),
        "forbidden_imports_found": forbidden_imports,
        "forbidden_calls_found": forbidden_calls,
        "standard_library_only": not forbidden_imports,
        "constructs_runtime": False,
        "starts_ray": False,
        "constructs_environment": False,
        "launches_subprocess": False,
        "trains": False,
        "problems": problems,
    }


def gate_t15_no_upstream_checkpoint_readiness(ev, problems) -> dict:
    """No upstream stage promoted THIS checkpoint or authorised a launch. rev1.

    The base said 'no upstream stage ever claimed a promotion'. That was
    factually WRONG: J19A legitimately records actor_promoted true. It promoted
    an ACTOR, at its own stage, after eleven binding eligibility criteria.

    What matters is narrower and true: no upstream stage promoted THIS J20
    checkpoint to TRAINING_INPUT_ONLY, none claimed CHECKPOINT training
    readiness, and none authorised a launch or a next stage. J19A's actor
    promotion is a different object and a different claim, and it is recorded
    here as COMPATIBLE - indeed required, since an actor with no eligibility
    behind it would be worse, not better.
    """
    claims = {}
    checks = (
        ("j19b", ev["j19b"], "outcome.promotion", "outcome.deployable",
         "outcome.next_stage_authorized"),
        ("j19c", ev["j19c"], "outcome.promotion", "outcome.deployable",
         "outcome.next_stage_authorized"),
        ("k1r1", ev["k1r1"], "promotion", None, "next_stage_authorized"),
        ("warmup_result", ev["warmup_result"], "promotion", None,
         "next_stage_authorized"),
        ("warmup_receipt", ev["warmup_receipt"], "promotion", None,
         "next_stage_authorized"),
        ("r3_result", ev["r3_result"], "promotion", None,
         "next_stage_authorized"),
        ("r3_receipt", ev["r3_receipt"], "promotion", None,
         "next_stage_authorized"),
    )
    for name, payload, promo, deployable, nextstage in checks:
        value = need(payload, promo, "T15/" + name, problems)
        expect(value, "NONE", "T15 %s promotion" % name, problems)
        claims[name] = {"promotion": value}
        if deployable is not None:
            claims[name]["deployable"] = need(payload, deployable,
                                              "T15/" + name, problems)
            expect(claims[name]["deployable"], False,
                   "T15 %s deployable" % name, problems)
        claims[name]["next_stage_authorized"] = need(payload, nextstage,
                                                     "T15/" + name, problems)
        expect(claims[name]["next_stage_authorized"], False,
               "T15 %s next stage" % name, problems)
    for name, payload in (("warmup_result", ev["warmup_result"]),
                          ("warmup_receipt", ev["warmup_receipt"]),
                          ("r3_result", ev["r3_result"]),
                          ("r3_receipt", ev["r3_receipt"])):
        value = need(payload, "training_ready", "T15/" + name, problems)
        expect(value, False, "T15 %s training_ready" % name, problems)
        claims[name]["training_ready"] = value

    # The compatible, distinct, stage-local promotion. Required to be TRUE.
    actor_promoted = need(ev["j19a_result"], "actor_promoted", "T15/j19a",
                          problems)
    expect(actor_promoted, True, "T15 J19A actor promotion", problems)
    for name, payload in (("j19b", ev["j19b"]), ("j19c", ev["j19c"])):
        promo = (payload.get("outcome") or {}).get("promotion")
        if promo == PROMOTION_PASS:
            problems.append("T15: %s already promoted to %s, which only J21 "
                            "may do" % (name, PROMOTION_PASS))
    for name, payload in (("warmup_result", ev["warmup_result"]),
                          ("r3_result", ev["r3_result"])):
        if payload.get("promotion") == PROMOTION_PASS:
            problems.append("T15: %s already promoted this checkpoint to %s"
                            % (name, PROMOTION_PASS))

    # rev2 packaging correction: prove the absence of a downstream PPO/ex-novo
    # continuation from explicit, pinned fields rather than inferring it only
    # from promotion and next_stage_authorized.
    continuation_evidence = {}
    for name, payload in (("j19b", ev["j19b"]), ("j19c", ev["j19c"])):
        ppo_updates = need(payload, "inert.ppo_updates", "T15/" + name,
                           problems)
        note = need(payload, "outcome.note", "T15/" + name, problems)
        expect(ppo_updates, 0, "T15 %s PPO updates" % name, problems)
        expect(note, "a closed-loop PASS here authorises nothing further",
               "T15 %s no-continuation note" % name, problems)
        continuation_evidence[name] = {
            "ppo_updates": ppo_updates,
            "no_continuation_note": note,
        }

    k1_inert = need(ev["k1r1"], "inert", "T15/k1r1", problems) or {}
    for field, wanted in (("ppo_updates", 0), ("algo_train_called", False),
                          ("warmup_executed", False)):
        expect(k1_inert.get(field), wanted, "T15 K1R1 " + field, problems)
    continuation_evidence["k1r1"] = {
        field: k1_inert.get(field)
        for field in ("ppo_updates", "algo_train_called", "warmup_executed")
    }

    for name, payload in (("warmup_result", ev["warmup_result"]),
                          ("warmup_receipt", ev["warmup_receipt"]),
                          ("r3_result", ev["r3_result"]),
                          ("r3_receipt", ev["r3_receipt"])):
        ppo_ex_novo = need(payload, "inert.ppo_ex_novo", "T15/" + name,
                           problems)
        expect(ppo_ex_novo, False, "T15 %s PPO/ex-novo" % name, problems)
        continuation_evidence.setdefault(name, {})["ppo_ex_novo"] = \
            ppo_ex_novo

    r3_denials = need(ev["r3_result"], "what_this_still_does_not_authorise",
                      "T15/r3_result", problems)
    if not isinstance(r3_denials, list):
        problems.append("T15: R3 no-authorisation disclosure is not a list")
        r3_denials = []
    required_denials = ("PPO", "an ex-novo run", "any promotion",
                        "training readiness", "any subsequent stage")
    for denial in required_denials:
        if denial not in r3_denials:
            problems.append("T15: R3 does not explicitly deny %r" % denial)
    continuation_evidence["r3_explicit_denials"] = list(r3_denials)
    return {
        "no_upstream_checkpoint_promotion": claims,
        "gated": ["no upstream stage promoted THIS J20 checkpoint to "
                  "TRAINING_INPUT_ONLY",
                  "no upstream stage claimed CHECKPOINT training readiness",
                  "no upstream stage authorised a downstream PPO/ex-novo "
                  "continuation or a next stage"],
        "measured_no_training_continuation": continuation_evidence,
        "compatible_and_distinct": {
            "j19a_actor_promoted": actor_promoted,
            "stage": "V26C_J19A_SINGLE_REPRODUCTION",
            "scope": "STAGE-LOCAL ACTOR PROMOTION",
            "what_it_means": "an actor passed J19A's eleven binding offline "
                             "eligibility criteria and was persisted",
            "why_it_does_not_conflict": "TRAINING_INPUT_ONLY is a "
                                        "CHECKPOINT-level status for the J20 "
                                        "warm-up checkpoint, which did not "
                                        "exist when J19A ran. Different "
                                        "object, different claim.",
            "it_is_required_not_merely_tolerated": "a J19A that had NOT "
                                                   "promoted its actor would "
                                                   "fail this gate: the actor "
                                                   "inside the checkpoint "
                                                   "would then have no "
                                                   "eligibility behind it.",
        },
        "what_j21_may_claim": "J21 is the first CURRENT V26C checkpoint-level "
                              "TRAINING_INPUT_ONLY attestation after J20.",
        "what_j21_may_NOT_claim": "that it is the first training_ready in the "
                                  "project's history. That is unverifiable "
                                  "here and is not asserted anywhere.",
    }


def evaluate() -> dict:
    """Every gate, read-only. Returns the whole record; writes nothing."""
    pin_report = check_pins()
    pins = all_pins()
    ev = evidence()
    problems: dict = {}
    findings: dict = {}
    local: list = []
    findings["T1_lineage_is_august_operational_july_informational"] = \
        gate_t1_lineage(ev, pins, local)
    problems["T1_lineage_is_august_operational_july_informational"] = list(local)

    local = []
    findings["T2_j19a_reproducibility_and_offline_eligibility"] = \
        gate_t2_j19a(ev, local)
    problems["T2_j19a_reproducibility_and_offline_eligibility"] = list(local)

    local = []
    findings["T3_j19b_six_of_six"] = gate_j19_cells(
        ev["j19b"], J19B_CELLS, 6, "T3", local)
    problems["T3_j19b_six_of_six"] = list(local)

    local = []
    findings["T4_j19c_three_of_three_heldout"] = gate_j19_cells(
        ev["j19c"], J19C_CELLS, 3, "T4", local)
    problems["T4_j19c_three_of_three_heldout"] = list(local)

    local = []
    findings["T5_penetration_contract_and_no_hard_breach"] = \
        gate_t5_penetration(ev, local)
    problems["T5_penetration_contract_and_no_hard_breach"] = list(local)

    local = []
    findings["T6_fsm_v3_and_morphology_counters_are_zero"] = \
        gate_t6_counters(ev, local)
    problems["T6_fsm_v3_and_morphology_counters_are_zero"] = list(local)

    local = []
    findings["T7_k1r1_nondegenerate_gradient"] = gate_t7_k1r1(ev, local)
    problems["T7_k1r1_nondegenerate_gradient"] = list(local)

    local = []
    findings["T8_j20_critic_warmup_twelve_of_twelve"] = gate_t8_warmup(ev, local)
    problems["T8_j20_critic_warmup_twelve_of_twelve"] = list(local)

    local = []
    findings["T9_r3_restore_audit_thirteen_of_thirteen"] = gate_t9_r3(ev, local)
    problems["T9_r3_restore_audit_thirteen_of_thirteen"] = list(local)

    local = []
    findings["T10_r3_live_optimizer_is_exact_under_rev7"] = \
        gate_t10_live_optimizer(ev, local)
    problems["T10_r3_live_optimizer_is_exact_under_rev7"] = list(local)

    local = []
    findings["T11_warmup_leaf_is_byte_immutable"] = \
        gate_t11_warmup_leaf_immutable(pins, local)
    problems["T11_warmup_leaf_is_byte_immutable"] = list(local)

    local = []
    findings["T12_checkpoint_files_are_pinned_and_present"] = \
        gate_t12_checkpoint(ev, pins, local)
    problems["T12_checkpoint_files_are_pinned_and_present"] = list(local)

    local = list(pin_report["problems"])
    deep = [verify_committed_map(J19B_LEAF_REL,
                                 "v26c_j19b_closed_loop_receipt.json",
                                 "committed_files_sha256"),
            verify_committed_map(J19C_LEAF_REL,
                                 "v26c_j19c_heldout_g_i_receipt.json",
                                 "committed_files_sha256")]
    for item in deep:
        local += item["problems"]
    findings["T13_runtime_config_and_source_are_pinned"] = {
        "pins_checked": pin_report["checked"],
        "pins_matching": pin_report["matching"],
        "r3_go_labels": R3_GO_PIN_COUNT,
        "additional_labels": len(PIN_ADDITIONAL),
        "deep_leaf_verification": [{"leaf": d["leaf"], "files": d["files"],
                                    "ok": d["ok"]} for d in deep],
    }
    problems["T13_runtime_config_and_source_are_pinned"] = list(local)

    self_audit = gate_t14_self_audit()
    findings["T14_j21_itself_constructs_no_runtime_and_trains_nothing"] = \
        self_audit
    problems["T14_j21_itself_constructs_no_runtime_and_trains_nothing"] = \
        list(self_audit["problems"])

    local = []
    findings["T15_no_upstream_checkpoint_readiness_or_launch"] = \
        gate_t15_no_upstream_checkpoint_readiness(ev, local)
    problems["T15_no_upstream_checkpoint_readiness_or_launch"] = list(local)

    local = []
    findings["T16_worker_sync_proved_by_the_transplant_report"] = \
        gate_t16_worker_sync(ev, local)
    problems["T16_worker_sync_proved_by_the_transplant_report"] = list(local)

    local = []
    findings["T17_august_lineage_measured_byte_for_byte"] = \
        gate_t17_measured_lineage(ev, local)
    problems["T17_august_lineage_measured_byte_for_byte"] = list(local)

    local = []
    findings["T18_leaf_file_sets_and_committed_maps_are_exact"] = \
        gate_t18_leaf_integrity(ev, local)
    problems["T18_leaf_file_sets_and_committed_maps_are_exact"] = list(local)

    gates = {name: not issues for name, issues in problems.items()}
    passed = sum(1 for ok in gates.values() if ok)
    return {
        "gates": gates,
        "gate_problems": problems,
        "findings": findings,
        "passed": passed,
        "total": len(gates),
        "failed": sorted(name for name, ok in gates.items() if not ok),
        "pin_report": pin_report,
    }


def final_fields(gates_ok: bool) -> dict:
    """The mandatory final semantics. Written on success AND on failure.

    Every one of these is false or zero whatever happens, except
    training_ready and promotion, which move only on a full pass - and even
    then to TRAINING_INPUT_ONLY, which authorises no action at all.

    rev1 SCOPES the two training fields. training_started and
    training_iterations_run refer to, and only to, (a) downstream post-J20
    actor-updating PPO or ex-novo pilot training and (b) the execution of J21
    itself. They do NOT deny the J20 critic warm-up, which really did complete
    one training iteration and is recorded truthfully in training_history_facts.
    Counts are integers, never booleans.
    """
    return {
        "training_ready": bool(gates_ok),
        "promotion": PROMOTION_PASS if gates_ok else PROMOTION_FAIL,
        "training_started": False,
        "training_iterations_run": 0,
        "sampling": False,
        "rollout": False,
        "ray_started": False,
        "environment_constructed": False,
        "launch_authorized": False,
        "next_stage_authorized": False,
        "ppo_authorized": False,
        "ex_novo_authorized": False,
        "deployable": False,
    }


def field_scopes() -> dict:
    """What the false flags and zero counts actually range over. rev1."""
    return {
        "training_started": "downstream post-J20 actor-updating PPO or "
                            "ex-novo pilot training, and the execution of J21 "
                            "itself. NOT the J20 critic warm-up.",
        "training_iterations_run": "the same scope, as an integer count. NOT "
                                   "the J20 critic warm-up's one iteration.",
        "environment_constructed": "J21 ITSELF constructs no environment. "
                                   "Prior stages did: J19B, J19C and R3 all "
                                   "constructed environments, and R3 declared "
                                   "it openly.",
        "ray_started": "J21 itself starts no Ray. Prior stages did.",
        "sampling": "J21 itself samples nothing. Prior stages did.",
        "rollout": "J21 itself rolls out nothing. J19B and J19C did.",
        "deployable": "always false, in every outcome, for every object.",
    }


def training_history_facts() -> dict:
    """What WAS trained, recorded truthfully alongside the scoped zeros. rev1."""
    return {
        "critic_warmup_iterations_completed": CRITIC_WARMUP_ITERATIONS_COMPLETED,
        "checkpoint_logical_iteration": CHECKPOINT_LOGICAL_ITERATION,
        "next_logical_iteration": NEXT_LOGICAL_ITERATION,
        "j21_execution_training_iterations": 0,
        "downstream_actor_training_started": False,
        "what_the_warmup_trained": "the critic only. The actor and the log-std "
                                   "stayed bit-exact, which gates G5 and G6 of "
                                   "the warm-up measured.",
        "note": "these are integers and are recorded because saying 'nothing "
                "was trained' would be false. One critic-only iteration of "
                "4096 sampled steps did happen.",
    }


def readiness_scope() -> dict:
    """What training_ready true asserts, and what it does not."""
    return {
        "object": CHECKPOINT_REL,
        "identified_by": {
            "module_state_sha256": EXPECTED_MODULE_STATE_SHA,
            "learner_state_sha256": EXPECTED_LEARNER_STATE_SHA,
            "actor_digest": EXPECTED_ACTOR_DIGEST,
            "critic_digest": EXPECTED_CRITIC_DIGEST,
        },
        "asserts": "this checkpoint is a sound, audited INPUT for a future "
                   "conservative PPO pilot: it restores exactly through the "
                   "real production path, it carries the qualified J19A actor "
                   "bit-exact, and every closed gate above holds.",
        "does_not_assert": [
            "that any pilot configuration is correct, safe or sealed",
            "that any launch command is correct or approved",
            "that pilot hyperparameters, iteration count, stopping rule, "
            "safety monitors or rollback plan exist",
            "that the resulting policy is deployable or fit for a subject",
            "anything about July as an operational baseline",
        ],
        "pilot_protocol_sealed": False,
        "pilot_protocol_note": "NO pilot configuration file and NO pilot launch "
                               "command exist, are pinned or are approved. This "
                               "stage attests CHECKPOINT readiness, not PILOT "
                               "readiness, and deliberately invents neither.",
        "why_it_is_still_meaningful": "the assertion is a closed list of "
                                      "properties of a hash-identified "
                                      "artefact, each measured from committed "
                                      "evidence. A future pilot still needs its "
                                      "own preregistration, GO and user "
                                      "authorisation; what it will not need is "
                                      "to re-litigate this checkpoint.",
    }


# ------------------------------------------------------------------ preflight --

def preflight(verbose: bool = True) -> dict:
    """Fail-closed readiness check. Reads only; writes nothing, launches nothing."""
    leaf = HERE / LEAF_ROOT / LEAF_NAME
    graded = evaluate()
    problems = [p for issues in graded["gate_problems"].values() for p in issues]
    if leaf.exists():
        problems.append("the destination leaf already exists: %s" % leaf)
    for name, pin in ((PREREG_NAME, PIN_PREREG),
                      (PREREG_REV1_NAME, PIN_PREREG_REV1)):
        actual = sha256_file(HERE / name) if (HERE / name).is_file() else None
        if actual != pin:
            problems.append("the preregistration %s changed: %s"
                            % (name, actual))

    report = {
        "stage": STAGE,
        "ok": not problems,
        "problems": problems,
        "gates": graded["total"],
        "gates_passed": graded["passed"],
        "gates_failed": graded["failed"],
        "pins_checked": graded["pin_report"]["checked"],
        "pins_matching": graded["pin_report"]["matching"],
        "destination": LEAF_ROOT + "/" + LEAF_NAME,
        "destination_exists": leaf.exists(),
        "readiness_scope": readiness_scope(),
        "final_fields_if_it_passed": final_fields(not graded["failed"]),
        "field_scopes": field_scopes(),
        "training_history_facts": training_history_facts(),
        "preregistration_precedence": list(PREREG_PRECEDENCE),
        "it_trains_nothing": True,
        "self_audit": graded["findings"][
            "T14_j21_itself_constructs_no_runtime_and_trains_nothing"],
    }
    if verbose:
        print("stage              %s" % STAGE)
        print("pins               %d/%d match"
              % (report["pins_matching"], report["pins_checked"]))
        print("gates              %d/%d pass"
              % (report["gates_passed"], report["gates"]))
        for name in sorted(graded["gates"]):
            print("   %-58s %s" % (name,
                                   "PASS" if graded["gates"][name] else "FAIL"))
        print("destination        %s (%s)"
              % (report["destination"],
                 "absent" if not report["destination_exists"] else "PRESENT"))
        print("self audit         standard library only: %s; trains: %s"
              % (report["self_audit"]["standard_library_only"],
                 report["self_audit"]["trains"]))
        print("prereg             %s (rev1 governs)"
              % " > ".join(PREREG_PRECEDENCE))
        print("pilot protocol     NOT SEALED - this stage attests CHECKPOINT")
        print("                   readiness only, and invents no pilot config")
        print("                   and no launch command")
        print("promotion if PASS  %s (never deployable)" % PROMOTION_PASS)
        print("verdict            %s" % ("READY" if report["ok"] else "BLOCKED"))
        for problem in problems:
            print("  problem: %s" % problem, file=sys.stderr)
    return report


# ---------------------------------------------------------------------- the GO --

def validate_go(payload) -> dict:
    """Validate an architect GO. It authorises ONE attestation, nothing else."""
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"],
                "pins": {}}
    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append("stage is %r, expected %r"
                        % (payload.get("stage"), GO_REQUIRED_STAGE))
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")
    if payload.get("status") != GO_REQUIRED_STATUS:
        problems.append("status is %r, and only the exact string %r authorises "
                        "execution" % (payload.get("status"),
                                       GO_REQUIRED_STATUS))
    user_auth = payload.get("user_authorisation")
    if not isinstance(user_auth, dict):
        problems.append("user_authorisation is missing or is not an object")
        user_auth = {}
    if user_auth.get("explicit") is not True:
        problems.append("user_authorisation.explicit is not exactly true")
    if user_auth.get("training_launch_authorised") is not False:
        problems.append("user_authorisation.training_launch_authorised is "
                        "not exactly false")
    if user_auth.get("scope") != GO_USER_AUTH_SCOPE:
        problems.append("user_authorisation.scope is %r, expected %r"
                        % (user_auth.get("scope"), GO_USER_AUTH_SCOPE))
    if user_auth.get("attestation_quote_verbatim") != \
            GO_USER_AUTH_ATTESTATION_QUOTE:
        problems.append("the attestation user-authorisation quote is absent "
                        "or not verbatim")
    if user_auth.get("stop_quote_verbatim") != GO_USER_AUTH_STOP_QUOTE:
        problems.append("the stop-before-training quote is absent or not "
                        "verbatim")
    for forbidden in ("authorises_retry", "authorises_ppo", "authorises_ex_novo",
                      "authorises_promotion", "authorises_training",
                      "authorises_launch", "authorises_deployment",
                      "authorises_rewriting_the_warmup_leaf"):
        if payload.get(forbidden) is True:
            problems.append("an attestation GO must never set %s" % forbidden)
    pins = payload.get("pinned_artefacts_sha256")
    if not isinstance(pins, dict):
        problems.append("pinned_artefacts_sha256 is missing or is not an object")
        pins = {}
    required = go_required_labels()
    for label in required:
        if label not in pins:
            problems.append("no pin for %s" % label)
    for label in sorted(pins):
        if label not in required:
            problems.append("pin for %s is outside the authorised scope" % label)
            continue
        path = resolve(label)
        if not path.is_file():
            problems.append("pinned artefact %s does not exist" % label)
            continue
        if sha256_file(path) != pins[label]:
            problems.append("pinned hash for %s is stale" % label)
    return {"valid": not problems, "problems": problems, "pins": dict(pins),
            "pin_labels_required": len(required),
            "pin_labels_supplied": len(pins)}


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


# ------------------------------------------------------------------- execute --

def run_execution(go_file: str) -> dict:
    """Evaluate once and commit. No subprocess, no retry, no runtime."""
    go = load_go(go_file)
    if not go["valid"]:
        raise AttestationError("refusing to execute: the architect GO is "
                               "absent or invalid. %s" % "; ".join(go["problems"]))
    report = preflight(verbose=False)
    if not report["ok"]:
        raise AttestationError("refusing to execute: preflight did not pass: %s"
                               % "; ".join(report["problems"]))

    leaf = HERE / LEAF_ROOT / LEAF_NAME
    if leaf.exists():
        raise AttestationError("refusing to clobber an existing leaf: %s" % leaf)
    leaf.mkdir(parents=True)
    (leaf / INVALID_MARKER).write_bytes(
        b"born invalid; replaced by TRAINING_READY_ATTESTED only after all "
        b"eighteen gates and post-commit verification pass\n")
    residents = sorted(p.name for p in leaf.iterdir())
    if residents != [INVALID_MARKER]:
        raise AttestationError("the destination is not clean: it holds %s"
                               % residents)

    graded = evaluate()
    gates_ok = not graded["failed"] and graded["passed"] == graded["total"]
    verdict = VERDICT_PASS if gates_ok else VERDICT_FAILED
    fields = final_fields(gates_ok)

    result = {
        "kind": "TRAINING READY ATTESTATION RESULT",
        "stage": STAGE,
        "verdict": verdict,
        "attested_at": now_utc(),
        "lineage": {"operational": OPERATIONAL_LINEAGE,
                    "july": "informational only; never an operational input"},
        "readiness_scope": readiness_scope(),
        "field_scopes": field_scopes(),
        "training_history_facts": training_history_facts(),
        "preregistration": {"base": PREREG_NAME, "base_sha256": PIN_PREREG,
                            "rev1": PREREG_REV1_NAME,
                            "rev1_sha256": PIN_PREREG_REV1,
                            "precedence": list(PREREG_PRECEDENCE)},
        "gates": graded["gates"],
        "gates_passed": graded["passed"],
        "gates_total": graded["total"],
        "gates_failed": graded["failed"],
        "gate_problems": graded["gate_problems"],
        "findings": graded["findings"],
        "accepted_diagnostics": graded["findings"][
            "T5_penetration_contract_and_no_hard_breach"][
                "accepted_diagnostics"],
        "warmup_leaf_marker": "RESTORE_AUDIT_PENDING, byte-immutable; G9 "
                              "closure is read from the R3 receipt",
        "what_this_still_does_not_authorise": [
            "launching a training run of any kind", "PPO", "an ex-novo run",
            "a pilot", "any deployment", "any subsequent stage",
        ],
    }
    result.update(fields)

    files = {RESULT_NAME: encode_json(result)}
    receipt = {
        "kind": "TRAINING READY ATTESTATION RECEIPT",
        "stage": STAGE,
        "verdict": verdict,
        "gates_passed": graded["passed"],
        "gates_total": graded["total"],
        "gates_failed": graded["failed"],
        "pins_checked": graded["pin_report"]["checked"],
        "pins_matching": graded["pin_report"]["matching"],
        "inputs": {
            "prereg_sha256": PIN_PREREG,
            "prereg_rev1_sha256": PIN_PREREG_REV1,
            "r3_architect_go_sha256": PIN_R3_GO,
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "checkpoint": CHECKPOINT_REL,
            "module_state_sha256": EXPECTED_MODULE_STATE_SHA,
            "learner_state_sha256": EXPECTED_LEARNER_STATE_SHA,
            "architect_go_pins": go["pins"],
        },
        "inert": {
            "trained": False, "sampled": False, "rolled_out": False,
            "ray_started": False, "environment_constructed": False,
            "child_processes_launched": 0, "retried": False,
            "imports_beyond_the_standard_library": 0,
        },
        "artefacts_sha256": {name: hashlib.sha256(payload).hexdigest()
                             for name, payload in sorted(files.items())},
    }
    receipt.update(fields)
    receipt["training_history_facts"] = training_history_facts()
    receipt["field_scopes"] = field_scopes()
    files[RECEIPT_NAME] = encode_json(receipt)
    for name, payload in sorted(files.items()):
        (leaf / name).write_bytes(payload)

    verification = verify_commit(leaf, files, gates_ok)
    return {"leaf": str(leaf.relative_to(HERE)), "verdict": verdict,
            "gates": "%d/%d" % (graded["passed"], graded["total"]),
            "training_ready": fields["training_ready"],
            "promotion": fields["promotion"],
            "verification": verification}


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
    for item in check_pins()["results"]:
        if not item["ok"]:
            problems.append("the pinned artefact %s changed after the run"
                            % item["artefact"])

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(leaf.relative_to(HERE)),
        "files_verified": len(committed["artefacts_sha256"]),
        "all_gates_passed": gates_ok,
        "problems": problems,
        "ok": not problems and gates_ok,
        "marker": ATTESTED_MARKER if (not problems and gates_ok)
        else INVALID_MARKER,
    }
    (leaf / COMMIT_VERIFICATION_NAME).write_bytes(encode_json(verification))
    if problems or not gates_ok:
        return verification
    (leaf / ATTESTED_MARKER).write_bytes(
        b"the aggregation passed all eighteen gates. The J20 warm-up checkpoint "
        b"is TRAINING_READY as an INPUT for a future conservative PPO pilot, "
        b"promotion TRAINING_INPUT_ONLY. J21 executed zero training iterations "
        b"and no downstream post-J20 actor-updating PPO or ex-novo training "
        b"started. The J20 critic-only warm-up had already completed exactly "
        b"one iteration. Nothing is launched or authorised and nothing is "
        b"deployable.\n")
    (leaf / INVALID_MARKER).unlink()
    return verification


# ------------------------------------------------------------------------ CLI --

def build_parser() -> argparse.ArgumentParser:
    """Command-line surface. Preflight is the default and writes nothing."""
    parser = argparse.ArgumentParser(
        description="V26C J21 - read-only training-ready evidence attestation")
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
        except (AttestationError, RuntimeError) as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"],
                          "verdict": outcome["verdict"],
                          "gates": outcome["gates"],
                          "training_ready": outcome["training_ready"],
                          "promotion": outcome["promotion"]}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = preflight(verbose=True)
    if args.dry_run:
        print("\nplan")
        print("  create the leaf, write %s FIRST, assert it is the only"
              % INVALID_MARKER)
        print("  resident, evaluate the eighteen gates READ-ONLY, then replace")
        print("  the marker with %s only if all eighteen and" % ATTESTED_MARKER)
        print("  post-commit verification pass.")
        print("\n  no subprocess, no Ray, no Algorithm, no environment, no")
        print("  sampling, no rollout, no training. Standard library only.")
        print("\n  on PASS: training_ready true, promotion %s," % PROMOTION_PASS)
        print("  and launch_authorized, ppo_authorized, ex_novo_authorized,")
        print("  next_stage_authorized, training_started, sampling, rollout,")
        print("  ray_started, environment_constructed and deployable ALL false,")
        print("  training_iterations_run 0.")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
