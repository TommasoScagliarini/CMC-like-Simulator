"""V26C J20 - readiness for the critic-only warm-up. K0 audit and K1 probe.

This stage does NOT warm up anything. It establishes, before any GO for the
warm-up itself can be issued, that:

  K0  every input the warm-up would consume is present at its pinned bytes -
      the J19A actor, the derived warm-up config, the feature-name overlay, the
      module class, the transplant module and the trainer that would run it;

  K1  the module the warm-up would build actually behaves as claimed, exercised
      through the REAL production path: an inference_only=False construction of
      the SAME pinned class and ctor arguments carries a live 84-wide critic;
      warm_start.transplant_actor_state - the function train_ppo_mlp itself
      calls - driven by warm_start.resolve_source_actor_features against the
      explicit overlay, lands the ten J19A actor tensors BYTE-IDENTICALLY,
      leaves the critic at the target's own fresh initialisation, zeroes only
      the two gait-clock columns (a provable no-op) and keeps the
      controller-memory block 25:35 alive; the frozen sigma is exactly 0.005;
      and the freeze mechanism really does cut the actor from the gradient graph.

The authoritative path is that production transplant. A bare
load_state_dict(strict=False) would exercise a path production does not take and
would prove nothing about the column algebra, the gait-clock zeroing or the
transplant's refusal to touch the critic; it is kept only as a SECONDARY
diagnostic, because the silent-fresh-critic behaviour is worth having on record.

K1 constructs a module and pushes one synthetic batch through it to check where
gradients flow. It NEVER calls algo.train(), never constructs an optimizer,
never takes an optimizer step, never builds Ray, never builds an environment and
never steps one. The absence of those is asserted structurally by the test suite
against this file's own source, and the actor tensors are re-verified
byte-identical AFTER the gradient probe, so nothing can have been written.

The warm-up execution itself is a SEPARATE stage. It is preregistered here, with
its gates and its exact one-shot command, and it is not reachable from this
module.

Usage:
    python v26c_j20_critic_warmup_readiness.py --preflight-only
    python v26c_j20_critic_warmup_readiness.py --dry-run
    python v26c_j20_critic_warmup_readiness.py --probe --go-file <architect GO json>

``--preflight-only`` and ``--dry-run`` write nothing, create no directory and
never import torch.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import pickletools
import sys

sys.dont_write_bytecode = True

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
REPO = HERE.parents[3]

STAGE = "V26C_J20_CRITIC_WARMUP_READINESS"
WARMUP_STAGE = "V26C_J20_CRITIC_WARMUP"

PREREG_NAME = "v26c_j20_prereg_critic_warmup_readiness.json"
PIN_PREREG = "f4ec06d77641a460e26df554f4c5c0f10ebceacb4534bbd0368773294706130c"

# --------------------------------------------------------------------------
# The one actor. Read, never edited, never copied into any promotable place.
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

# Entry evidence: the warm-up runs behind a closed-loop qualification that passed.
PIN_J19B_RECEIPT = "789c57ca293ae7d5f578d4b3cae4e9aec599fa3edf4ce87239ad5fc8a7b1b509"
PIN_J19C_RECEIPT = "4224f0201eeccc8ccec911635b5bc18a82e38fb27eeb878457d004d28cb17ffc"
PIN_J19C_COMMIT = "11096edec7e259a2fe3f6dd2691c26cee0ce0e82b96864bd85a52a00a9e6ef2f"

# --------------------------------------------------------------------------
# The stage's own additive artefacts.
# --------------------------------------------------------------------------
CONFIG_NAME = "v26c_j20_warmup_cfg.yaml"
PIN_CONFIG = "d8bcfd39f6015eaf7a41deda2edec201e9dbe89df6c425a67f3bf14bc5e4f525"
DERIVATION_NAME = "v26c_j20_derive_warmup_config.py"
OVERLAY_NAME = "v26c_j20_actor_feature_manifest_overlay.json"
PIN_OVERLAY = "b0f9035413be0a56216e391cd27c4282c3cb708b7733a67ad32e506d281d39ca"

# The pinned runtime config the warm-up config is derived FROM.
SOURCE_CONFIG_REL = (
    "Trajectory Generator/runs/training/"
    "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml"
)
PIN_SOURCE_CONFIG = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"

# --------------------------------------------------------------------------
# The production code the warm-up would execute. Pinned, so a GO authorises a
# warm-up against THESE bytes and not against whatever they later become.
# --------------------------------------------------------------------------
BASELINE_PINS = {
    "asymmetric_rl_module.py":
        "5084786c8e6312de2d37744bf327b907ed52ff92cc6e3686b36bd1bde6d21a0f",
    "warm_start.py":
        "84706218dcc4c5cb7f97a8f3f67ef40ba9e064ba5aef25cb6559d1c8a506c34c",
    "train_ppo_mlp.py":
        "cbd278c39c4fe9efcfb52b7a95eb3e85f24020eb6aedd1892f7832b1472be375",
    "tb_logging.py":
        "b5543e286410aa9476b5e91614b3d1028516696f1a5c743ec858d1e2c609ce26",
}
TELEMETRY_TEST_REL = "validation/test_training_health_telemetry.py"
PIN_TELEMETRY_TEST = \
    "608cbe0ac927506553bee52b6a1eeb815d8d288f61a4a67f6410fbebc486206b"

# The eight training-health fields, asserted textually so the preflight needs no
# import of tb_logging and therefore no ray import.
TRAINING_HEALTH_FIELDS = (
    "observed_rows",
    "missing_telemetry_rows",
    "phase_timeout_stance_rows",
    "morphology_causal_contract_failure_rows",
    "resync_event_rows",
    "resync_count_max",
    "hs_cancelled_count_max",
    "timeout_side_disagreement_rows",
)

# --------------------------------------------------------------------------
# The module contract, measured not assumed.
# --------------------------------------------------------------------------
MODULE_CLASS = "AsymmetricActorCriticTorchRLModule"
MODULE_MODULE = "asymmetric_rl_module"
ACTOR_WIDTH = 35
FULL_WIDTH = 84
ACTION_DIM = 2
HIDDENS = (256, 256)
ACTIVATION = "tanh"

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
CRITIC_SHAPES = {
    "vf_encoder.0.weight": (256, FULL_WIDTH),
    "vf_encoder.0.bias": (256,),
    "vf_encoder.2.weight": (256, 256),
    "vf_encoder.2.bias": (256,),
    "vf.weight": (1, 256),
    "vf.bias": (1,),
}
# warm_start.actor_state_digest key order, transcribed.
WARM_START_ACTOR_KEYS = (
    "pi_encoder.0.weight", "pi.0.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.bias",
    "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias",
)
ENCODER_ALIASES = {
    "pi_encoder.0.weight": "pi.0.0.weight",
    "pi_encoder.0.bias": "pi.0.0.bias",
    "pi_encoder.2.weight": "pi.0.2.weight",
    "pi_encoder.2.bias": "pi.0.2.bias",
}
LOGSTD_ROWS = slice(2, 4)
CLOCK_COLUMNS = slice(0, 2)
CONTROLLER_COLUMNS = slice(25, 35)
# The LIVE environment's own recorded actor contract, from a committed J19C
# cell. The target's feature names must not come from the same manifest that
# describes the source, or the contract would agree with itself by construction.
ENV_FEATURE_NAMES_REL = ("j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/"
                         "j19c_cell_G_kinematics.npz")
SIGMA = 0.005
SIGMA_TOLERANCE = 1e-9
PROBE_SEED = 123
PROBE_BATCH_ROWS = 8

# --------------------------------------------------------------------------
# The only destination.
# --------------------------------------------------------------------------
LEAF_ROOT = "j20_runs"
LEAF_NAME = "j20_critic_warmup_readiness_v26c_2026-08-27_r1"
RECEIPT_NAME = "v26c_j20_critic_warmup_readiness_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
INVALID_MARKER = "TECHNICAL_INVALID"

GO_REQUIRED_STAGE = STAGE
GO_REQUIRED_PINS = (
    PREREG_NAME,
    "v26c_j20_critic_warmup_readiness.py",
    "test_v26c_j20_critic_warmup_readiness.py",
    DERIVATION_NAME,
    CONFIG_NAME,
    OVERLAY_NAME,
    J19A_MODULE_REL + "/module_state.pkl",
)

# Calls this file must never make. The probe has to be structurally unable to
# train, to sample or to build an environment, and the test suite asserts that by
# walking this file's own AST - not its text, because several of these words
# legitimately appear in prose and in receipt fields that record their ABSENCE.
FORBIDDEN_CALLS = (
    "train", "build_algo", "PPOConfig", "init", "make_cmc_env", "make_env",
    "save_to_path", "restore_from_path", "from_checkpoint", "sync_weights",
    "reset", "sample",
)
# Modules this file must never import.
FORBIDDEN_IMPORTS = ("torch.optim", "env_factory", "rollout_eval", "train_ppo_mlp")

INTERPRETER = "/opt/anaconda3/envs/envCMC-rllib/bin/python"


class J20Error(RuntimeError):
    """The stage refused to proceed."""


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
            raise J20Error("actor state is missing %s" % key)
        digest.update(key.encode("utf-8"))
        digest.update(tensor_digest(state[key]).encode("ascii"))
    return digest.hexdigest()


def bytes_identical(left, right) -> bool:
    """True only for the same dtype, the same shape and the same C-order bytes.

    Deliberately NOT numpy's array_equal, which is numeric: it would call a
    float32 tensor equal to a float64 one and -0.0 equal to +0.0. An immutability
    claim made with array_equal would be weaker than it sounds.
    """
    import numpy as np

    a = np.ascontiguousarray(left)
    b = np.ascontiguousarray(right)
    return bool(a.dtype == b.dtype and a.shape == b.shape
                and a.tobytes(order="C") == b.tobytes(order="C"))


def decode_ctor_args(raw: bytes) -> dict:
    """Read class_and_ctor_args.pkl WITHOUT importing the class it names.

    A pickle that names asymmetric_rl_module cannot be loaded without importing
    it, and the preflight must not import production code to decide whether that
    code is the pinned one. Disassembling the opcodes reads the declared values
    with no import at all.
    """
    import io

    buffer = io.StringIO()
    pickletools.dis(raw, out=buffer)
    text = buffer.getvalue()

    def has(literal: str) -> bool:
        return ("SHORT_BINUNICODE %r" % literal) in text or (
            "BINUNICODE %r" % literal) in text

    def int_after(name: str):
        marker = "SHORT_BINUNICODE %r" % name
        index = text.find(marker)
        if index < 0:
            return None
        window = text[index:index + 400].splitlines()
        for line in window[1:]:
            if "BININT" in line:
                return int(line.rsplit(None, 1)[1])
        return None

    def bool_after(name: str):
        marker = "SHORT_BINUNICODE %r" % name
        index = text.find(marker)
        if index < 0:
            return None
        window = text[index:index + 200].splitlines()
        for line in window[1:]:
            if "NEWTRUE" in line:
                return True
            if "NEWFALSE" in line:
                return False
        return None

    return {
        "declares_module": has(MODULE_MODULE),
        "declares_class": has(MODULE_CLASS),
        "inference_only": bool_after("inference_only"),
        "learner_only": bool_after("learner_only"),
        "vf_share_layers": bool_after("vf_share_layers"),
        "freeze_logstd": bool_after("freeze_logstd"),
        "freeze_actor": bool_after("freeze_actor"),
        "n_actor": int_after("n_actor"),
        "n_full": int_after("n_full"),
        "activation_is_tanh": has(ACTIVATION),
        "observation_shape_declared": int_after("_shape"),
    }


# ------------------------------------------------------------- sealed command --

def sealed_command(output_dir: str) -> tuple[str, ...]:
    """The EXACT one-shot warm-up invocation this stage preregisters.

    One process, no supervisor, no retry: ``--worker-process`` is what makes
    main() fall through to run() instead of entering run_supervised()'s restart
    loop, so a crash is a failure and never a retry.

    The scientific parameters are NOT on this command line. They live in the
    derived config, so the command cannot silently disagree with the document
    that was preregistered and hashed.
    """
    return (
        INTERPRETER,
        str(BASELINE / "train_ppo_mlp.py"),
        "--worker-process",
        "--config", str(HERE / CONFIG_NAME),
        "--output-dir", output_dir,
        "--warm-start-raw",
        "--warm-start-raw-source", str(HERE / J19A_MODULE_REL),
        "--warm-start-raw-source-feature-manifest", str(HERE / OVERLAY_NAME),
        "--asymmetric-actor-critic",
        "--freeze-actor",
        "--freeze-logstd",
        "--iterations", "1",
        "--checkpoint-every", "1",
        "--retain-iteration-checkpoints",
        "--tensorboard",
        "--no-progress",
        "--no-update-history",
    )


def sealed_command_text(output_dir: str) -> str:
    """The sealed command as a single copy-pasteable line."""
    parts = []
    for token in sealed_command(output_dir):
        parts.append('"%s"' % token if " " in token else token)
    return "PYTHONDONTWRITEBYTECODE=1 " + " ".join(parts)


# ------------------------------------------------------------------ preflight --

def check_pins() -> list[dict]:
    """Every pinned artefact, re-hashed. Pure: reads files, writes nothing."""
    results = []

    def record(label: str, path: pathlib.Path, expected: str | None) -> None:
        if not path.is_file():
            results.append({"artefact": label, "ok": False, "why": "missing",
                            "expected": expected, "actual": None})
            return
        actual = sha256_file(path)
        results.append({
            "artefact": label,
            "ok": expected is None or actual == expected,
            "why": "present" if expected is None else
                   ("matches" if actual == expected else "HASH MISMATCH"),
            "expected": expected,
            "actual": actual,
        })

    # The preregistration is sealed. The check is UNCONDITIONAL: there is no
    # branch that accepts a placeholder, so a runner carrying an unsealed pin
    # cannot pass its own preflight.
    record(PREREG_NAME, HERE / PREREG_NAME, PIN_PREREG)
    for name, expected in sorted(PIN_J19A_LEAF.items()):
        record(J19A_LEAF_REL + "/" + name, HERE / J19A_LEAF_REL / name, expected)
    record(CONFIG_NAME, HERE / CONFIG_NAME, PIN_CONFIG)
    record(OVERLAY_NAME, HERE / OVERLAY_NAME, PIN_OVERLAY)
    record(DERIVATION_NAME, HERE / DERIVATION_NAME, None)
    record(SOURCE_CONFIG_REL, REPO / SOURCE_CONFIG_REL, PIN_SOURCE_CONFIG)
    for name, expected in sorted(BASELINE_PINS.items()):
        record("baseline_MLP/" + name, BASELINE / name, expected)
    record("baseline_MLP/" + TELEMETRY_TEST_REL, BASELINE / TELEMETRY_TEST_REL,
           PIN_TELEMETRY_TEST)
    record("j19b receipt",
           HERE / "j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1"
                / "v26c_j19b_closed_loop_receipt.json", PIN_J19B_RECEIPT)
    record("j19c receipt",
           HERE / "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1"
                / "v26c_j19c_heldout_g_i_receipt.json", PIN_J19C_RECEIPT)
    record("j19c commit verification",
           HERE / "j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1"
                / COMMIT_VERIFICATION_NAME, PIN_J19C_COMMIT)
    return results


def check_config() -> dict:
    """The derived config says what the preregistration says it says."""
    import yaml

    path = HERE / CONFIG_NAME
    if not path.is_file():
        raise J20Error("the derived warm-up config is missing: %s" % path)
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    expected = {
        "model.freeze_actor": True,
        "model.freeze_logstd": True,
        "model.asymmetric_actor_critic": True,
        "model.seed": 123,
        "ppo.train_batch_size": 4096,
        "ppo.minibatch_size": 512,
        "ppo.num_epochs": 10,
        "ppo.lr": 1e-04,
        "ppo.gamma": 0.99,
        "ppo.lam": 0.9,
        "ppo.clip_param": 0.2,
        "ppo.kl_coeff": 0.2,
        "ppo.kl_target": 0.01,
        "ppo.vf_clip_param": 10.0,
        "ppo.vf_loss_coeff": 1.0,
        "parallelism.num_env_runners": 13,
        "parallelism.ray_num_cpus": 14,
        "parallelism.exact_start_sampling": False,
        "simulation.iterations": 1,
        "simulation.episode_start_offset_s": 1.956870983805102,
        "simulation.episode_start_offset_choices_s": [],
    }
    problems = []
    observed = {}
    for dotted, want in sorted(expected.items()):
        section, key = dotted.split(".", 1)
        got = (payload.get(section) or {}).get(key)
        observed[dotted] = got
        if got != want:
            problems.append("%s is %r, the preregistration says %r"
                            % (dotted, got, want))

    # Everything the stage must NOT have touched, compared against the source.
    source = yaml.safe_load((REPO / SOURCE_CONFIG_REL).read_text(encoding="utf-8"))
    untouched = {}
    for section in ("grf", "supervision", "logging", "reward"):
        same = payload.get(section) == source.get(section)
        untouched[section] = bool(same)
        if not same:
            problems.append("section %r differs from the pinned runtime config; "
                            "the physical runtime, FSM v3, the detector, the "
                            "morphology corridor and the reward must be "
                            "inherited unchanged" % section)
    return {"ok": not problems, "problems": problems, "values": observed,
            "sections_byte_inherited": untouched,
            "reward_keys": len(payload.get("reward") or {})}


def check_overlay() -> dict:
    """The overlay corrects exactly one field and invents no name.

    Every one of the thirteen committed sources is re-read here. The overlay's
    list is transcription, and this is what holds the transcription honest.
    """
    import numpy as np

    path = HERE / OVERLAY_NAME
    payload = json.loads(path.read_text(encoding="utf-8"))
    problems = []
    # The three keys warm_start actually reads must be at the TOP level. A list
    # nested under a documentation key is not consumable, however well
    # documented - measured, not assumed: the first draft of this overlay was
    # rejected at runtime for exactly that.
    for required in ("actor_feature_names", "actor_feature_count", "actor_digest"):
        if required not in payload:
            problems.append("the overlay has no top-level %s, so warm_start "
                            "cannot consume it" % required)
    names = tuple(payload.get("actor_feature_names") or ())
    if len(names) != ACTOR_WIDTH:
        problems.append("the overlay lists %d names, expected %d"
                        % (len(names), ACTOR_WIDTH))
    if len(set(names)) != len(names):
        problems.append("the overlay lists duplicate names")
    if payload.get("actor_feature_count") != ACTOR_WIDTH:
        problems.append("the overlay declares actor_feature_count %r, expected %d"
                        % (payload.get("actor_feature_count"), ACTOR_WIDTH))
    if payload.get("actor_digest") != PIN_J19A_ACTOR_DIGEST:
        problems.append("the overlay declares an actor_digest that is not J19A's")

    joined = hashlib.sha256("\n".join(names).encode("utf-8")).hexdigest()
    if joined != payload["the_feature_list"]["names_join_sha256"]:
        problems.append("the overlay's own names_join_sha256 does not match its list")

    sources = {}
    for relative in payload["provenance_of_the_names"]["sources"]:
        target, _, field = relative.partition("#")
        full = HERE / target
        if not full.is_file():
            problems.append("overlay source is missing: %s" % target)
            continue
        if target.endswith(".npz"):
            with np.load(full) as bundle:
                found = tuple(str(x) for x in bundle["actor_feature_names"])
        else:
            document = json.loads(full.read_text(encoding="utf-8"))
            node = document
            for part in field.split("."):
                node = node[part]
            if isinstance(node, dict):
                candidates = [v for v in node.values()
                              if isinstance(v, list) and len(v) == ACTOR_WIDTH]
                node = candidates[0] if candidates else []
            found = tuple(str(x) for x in node)
        sources[relative] = found == names
        if found != names:
            problems.append("overlay source disagrees with the overlay: %s" % relative)

    if payload["overlays"]["sha256"] != PIN_J19A_LEAF[
            "rl_module/actor_feature_manifest.json"]:
        problems.append("the overlay names a manifest hash that is not J19A's")
    leaf_manifest = json.loads(
        (HERE / J19A_MODULE_REL / "actor_feature_manifest.json").read_text("utf-8"))
    if "actor_feature_names" in leaf_manifest:
        problems.append("the J19A manifest now HAS actor_feature_names; the "
                        "overlay's reason for existing has changed and it must "
                        "be re-audited rather than silently kept")
    return {"ok": not problems, "problems": problems,
            "sources_checked": len(sources),
            "sources_agreeing": sum(1 for v in sources.values() if v),
            "names_join_sha256": joined}


def check_telemetry_source() -> dict:
    """The eight training-health fields are declared, without importing ray."""
    source = (BASELINE / "tb_logging.py").read_text(encoding="utf-8")
    problems = []
    if "TRAINING_HEALTH_FIELDS" not in source:
        problems.append("tb_logging declares no TRAINING_HEALTH_FIELDS")
    for field in TRAINING_HEALTH_FIELDS:
        if '"%s"' % field not in source:
            problems.append("tb_logging does not declare the field %s" % field)
    if 'TRAINING_HEALTH_PREFIX = "training_health"' not in source:
        problems.append("the TensorBoard section is not training_health")
    trainer = (BASELINE / "train_ppo_mlp.py").read_text(encoding="utf-8")
    if '"training_health": _training_health_metrics(env_metrics),' not in trainer:
        problems.append("the trainer does not publish training_health in the "
                        "iteration row")
    if "_training_health_metrics" not in trainer:
        problems.append("the trainer has no _training_health_metrics helper")
    return {"ok": not problems, "problems": problems,
            "fields": list(TRAINING_HEALTH_FIELDS)}


def check_no_leaf() -> dict:
    """The destination must not exist yet."""
    root = HERE / LEAF_ROOT
    leaf = root / LEAF_NAME
    return {"ok": not leaf.exists(),
            "leaf": LEAF_ROOT + "/" + LEAF_NAME,
            "leaf_exists": leaf.exists(),
            "root_exists": root.exists(),
            "root_contents": sorted(p.name for p in root.iterdir())
            if root.is_dir() else []}


def preflight(verbose: bool = True) -> dict:
    """Fail-closed readiness check. No torch, no ray, no env, no write."""
    pins = check_pins()
    config = check_config()
    overlay = check_overlay()
    telemetry = check_telemetry_source()
    destination = check_no_leaf()

    ctor_raw = (HERE / J19A_MODULE_REL / "class_and_ctor_args.pkl").read_bytes()
    ctor = decode_ctor_args(ctor_raw)
    ctor_problems = []
    for field, want in (("declares_module", True), ("declares_class", True),
                        ("inference_only", True), ("learner_only", False),
                        ("vf_share_layers", False), ("n_actor", ACTOR_WIDTH),
                        ("n_full", FULL_WIDTH), ("activation_is_tanh", True),
                        ("observation_shape_declared", FULL_WIDTH)):
        if ctor.get(field) != want:
            ctor_problems.append("ctor %s is %r, expected %r"
                                 % (field, ctor.get(field), want))

    problems = [entry["artefact"] + ": " + entry["why"]
                for entry in pins if not entry["ok"]]
    problems += config["problems"] + overlay["problems"] + telemetry["problems"]
    problems += ctor_problems
    if not destination["ok"]:
        problems.append("the destination leaf already exists: " + destination["leaf"])

    report = {
        "stage": STAGE,
        "ok": not problems,
        "problems": problems,
        "pins": pins,
        "pins_checked": len(pins),
        "pins_matching": sum(1 for entry in pins if entry["ok"]),
        "config": config,
        "overlay": overlay,
        "telemetry": telemetry,
        "ctor": ctor,
        "destination": destination,
        "warmup_is_not_authorised_by_this_stage": True,
    }
    if verbose:
        print("stage              %s" % STAGE)
        print("pins               %d/%d match"
              % (report["pins_matching"], report["pins_checked"]))
        print("derived config     %s (%d reward keys inherited)"
              % ("OK" if config["ok"] else "FAILED", config["reward_keys"]))
        print("overlay            %s (%d/%d committed sources agree)"
              % ("OK" if overlay["ok"] else "FAILED",
                 overlay["sources_agreeing"], overlay["sources_checked"]))
        print("telemetry source   %s (%d fields)"
              % ("OK" if telemetry["ok"] else "FAILED", len(TRAINING_HEALTH_FIELDS)))
        print("module ctor        n_actor=%s n_full=%s inference_only=%s"
              % (ctor.get("n_actor"), ctor.get("n_full"), ctor.get("inference_only")))
        print("destination        %s (%s)"
              % (destination["leaf"], "absent" if destination["ok"] else "PRESENT"))
        print("verdict            %s" % ("READY" if report["ok"] else "BLOCKED"))
        for problem in problems:
            print("  problem: %s" % problem, file=sys.stderr)
    return report


# ---------------------------------------------------------------------- the GO --

def validate_go(payload, root: pathlib.Path = HERE) -> dict:
    """Validate an architect GO. It authorises the K1 probe and nothing else."""
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"],
                "pins": {}}
    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append("stage is %r, expected %r"
                        % (payload.get("stage"), GO_REQUIRED_STAGE))
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")
    for forbidden in ("authorises_warmup", "authorises_training",
                      "authorises_rollout", "authorises_ppo"):
        if payload.get(forbidden) is True:
            problems.append("a J20 readiness GO must never set %s" % forbidden)
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


# ------------------------------------------------------------------ K1 probe --

def build_module(torch_module, gym_module, module_cls, *, freeze: bool):
    """Construct ONE trainable module at the pinned geometry.

    ``inference_only=False`` is the ONLY difference from the pinned J19A ctor
    arguments other than the two freeze flags, and both differences are named in
    the preregistration. Nothing else about the class or its geometry moves.
    """
    import numpy as np

    torch_module.manual_seed(PROBE_SEED)
    return module_cls(
        observation_space=gym_module.spaces.Box(
            -np.inf, np.inf, (FULL_WIDTH,), np.float32),
        action_space=gym_module.spaces.Box(-1.0, 1.0, (ACTION_DIM,), np.float32),
        inference_only=False,
        learner_only=False,
        catalog_class=None,
        model_config={
            "vf_share_layers": False,
            "n_actor": ACTOR_WIDTH,
            "n_full": FULL_WIDTH,
            "fcnet_hiddens": list(HIDDENS),
            "fcnet_activation": ACTIVATION,
            "freeze_logstd": bool(freeze),
            "freeze_actor": bool(freeze),
        },
    )


def gradient_reach(torch_module, module, on_disk) -> dict:
    """Where do gradients reach? One synthetic batch, NO optimizer, NO step.

    This is the only place the probe touches autograd. It constructs no
    optimizer, calls no step, and writes no parameter: it reads ``.grad`` after a
    single backward pass and then the caller re-verifies that every actor tensor
    is still byte-identical to J19A's on disk, so the claim that nothing was
    written is measured rather than asserted.

    The frozen module is compared against an UNFROZEN control built the same way.
    Without the control the check would be vacuous: a probe that finds no actor
    gradient proves nothing unless it can show it would have found one.
    """
    from ray.rllib.core.columns import Columns

    batch = {Columns.OBS: torch_module.zeros(
        (PROBE_BATCH_ROWS, FULL_WIDTH), dtype=torch_module.float32)}
    for parameter in module.parameters():
        parameter.grad = None
    outputs = module._forward_train(batch)
    logits = outputs[Columns.ACTION_DIST_INPUTS]
    values = module.compute_values(batch)
    (logits.sum() + values.sum()).backward()

    reached = {}
    for name, parameter in module.named_parameters():
        grad = parameter.grad
        reached[name] = bool(grad is not None
                             and float(grad.abs().sum().item()) > 0.0)
    actor_reached = sorted(name for name, hit in reached.items()
                           if hit and not name.startswith(("vf.", "vf_encoder.")))
    critic_reached = sorted(name for name, hit in reached.items()
                            if hit and name.startswith(("vf.", "vf_encoder.")))
    state = {k: v.detach().cpu().numpy() for k, v in module.state_dict().items()}
    unchanged = all(bytes_identical(state[key], on_disk[key]) for key in ACTOR_KEYS)
    return {
        "actor_parameters_receiving_gradient": actor_reached,
        "critic_parameters_receiving_gradient": critic_reached,
        "actor_bytes_unchanged_after_backward": bool(unchanged),
        "no_optimizer_was_constructed": True,
        "no_optimizer_step_was_taken": True,
    }


def run_probe(go_file: str) -> dict:
    """K1. Build the target, transplant through the REAL warm-start path, measure.

    The authoritative path is warm_start.transplant_actor_state - the same
    function train_ppo_mlp calls - driven by the real feature resolver against
    the explicit overlay. A bare load_state_dict would exercise a path production
    does not take, and would prove nothing about the transplant's column
    algebra, its gait-clock zeroing or its refusal to touch the critic. The bare
    load is kept, but only as a SECONDARY diagnostic.

    No algo.train(), no optimizer, no step, no Ray, no environment, no rollout.
    """
    go = load_go(go_file)
    if not go["valid"]:
        raise J20Error("refusing to run the probe: the architect GO is absent or "
                       "invalid. %s" % "; ".join(go["problems"]))
    report = preflight(verbose=False)
    if not report["ok"]:
        raise J20Error("refusing to run the probe: preflight did not pass: %s"
                       % "; ".join(report["problems"]))

    import pickle

    import numpy as np

    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import gymnasium
    import torch
    import warm_start
    from asymmetric_rl_module import AsymmetricActorCriticTorchRLModule as module_cls

    if module_cls.__name__ != MODULE_CLASS:
        raise J20Error("the imported class is %s, expected %s"
                       % (module_cls.__name__, MODULE_CLASS))

    source_dir = HERE / J19A_MODULE_REL
    overlay_path = HERE / OVERLAY_NAME
    on_disk = {k: np.asarray(v)
               for k, v in pickle.loads(
                   (source_dir / "module_state.pkl").read_bytes()).items()}
    if tuple(sorted(on_disk)) != tuple(sorted(ACTOR_KEYS)):
        raise J20Error("the J19A module holds %s, expected the ten actor keys"
                       % sorted(on_disk))

    # ---------------------------------------------------------------- the target
    frozen = build_module(torch, gymnasium, module_cls, freeze=True)
    control = build_module(torch, gymnasium, module_cls, freeze=False)
    built_keys = tuple(sorted(frozen.state_dict()))
    target_state = {k: np.array(v.detach().cpu().numpy(), copy=True)
                    for k, v in frozen.state_dict().items()}
    critic_present = all(key in built_keys for key in CRITIC_KEYS)
    critic_shapes_ok = all(
        tuple(int(d) for d in target_state[key].shape) == shape
        for key, shape in CRITIC_SHAPES.items()) if critic_present else False

    def critic_digest(state) -> str:
        digest = hashlib.sha256()
        for key in sorted(CRITIC_KEYS):
            digest.update(key.encode("utf-8"))
            digest.update(tensor_digest(state[key]).encode("ascii"))
        return digest.hexdigest()

    fresh_critic_digest = critic_digest(target_state) if critic_present else None

    # ------------------------------------------- the target's feature contract
    # Taken from the LIVE environment's own record, not from the same manifest
    # that describes the source. Using the source manifest for both sides would
    # make the contract agree with itself by construction and prove nothing.
    with np.load(HERE / ENV_FEATURE_NAMES_REL) as bundle:
        env_names = tuple(str(name) for name in bundle["actor_feature_names"])

    # ------------------------------------------------------ the REAL resolver
    source_state = warm_start.load_module_state(source_dir)
    source_names, manifest_report = warm_start.resolve_source_actor_features(
        source_checkpoint=source_dir,
        source_state=source_state,
        source_actor_feature_manifest=overlay_path,
    )

    # ----------------------------------------------------- the REAL transplant
    transplanted, transplant_report = warm_start.transplant_actor_state(
        target_state=target_state,
        target_actor_feature_names=env_names,
        source_checkpoint=source_dir,
        source_actor_feature_manifest=overlay_path,
        mode=warm_start.DEFAULT_WARM_START_MODE,
        zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES,
    )
    critic_comparison = warm_start.compare_non_actor_states(target_state, transplanted)
    actor_comparison = warm_start.compare_actor_states(on_disk, transplanted)

    # SECONDARY DIAGNOSTIC ONLY: what a bare load of the actor-only export does.
    # It is recorded because the silent-fresh-critic behaviour is worth having on
    # the record, but it is NOT the path the warm-up takes.
    bare_missing, bare_unexpected = control.load_state_dict(
        {k: torch.as_tensor(v) for k, v in on_disk.items()}, strict=False)

    # The transplanted state is complete, so the production load is strict.
    strict_missing, strict_unexpected = frozen.load_state_dict(
        {k: torch.as_tensor(v) for k, v in transplanted.items()}, strict=True)
    control.load_state_dict(
        {k: torch.as_tensor(v) for k, v in transplanted.items()}, strict=True)

    after = {k: v.detach().cpu().numpy() for k, v in frozen.state_dict().items()}
    per_key = {key: bytes_identical(after[key], on_disk[key]) for key in ACTOR_KEYS}
    alias_ok = {alias: bytes_identical(after[alias], after[source])
                for alias, source in ENCODER_ALIASES.items()}
    digest_after = actor_digest(after)

    # The pinned runtime has gait_clock_enable false, so the transplant zeroes
    # the target columns for gait_phase_sin and gait_phase_cos. Those columns are
    # already exactly zero in J19A, so the zeroing is a no-op - but that has to be
    # MEASURED, because if it were not a no-op the warm-up would perturb the
    # actor before the first gradient.
    first_layer = np.asarray(after["pi.0.0.weight"])
    encoder_layer = np.asarray(after["pi_encoder.0.weight"])
    clock_zero = bool(
        bytes_identical(first_layer[:, CLOCK_COLUMNS],
                        np.zeros_like(first_layer[:, CLOCK_COLUMNS]))
        and bytes_identical(encoder_layer[:, CLOCK_COLUMNS],
                            np.zeros_like(encoder_layer[:, CLOCK_COLUMNS])))
    # The controller-memory block must survive the transplant ALIVE: copied
    # byte-for-byte and not silently zeroed along with the clock columns.
    controller_identical = bool(
        bytes_identical(first_layer[:, CONTROLLER_COLUMNS],
                        np.asarray(on_disk["pi.0.0.weight"])[:, CONTROLLER_COLUMNS])
        and bytes_identical(encoder_layer[:, CONTROLLER_COLUMNS],
                            np.asarray(on_disk["pi_encoder.0.weight"])[:, CONTROLLER_COLUMNS]))
    controller_alive = bool(np.abs(first_layer[:, CONTROLLER_COLUMNS]).max() > 0.0)

    head_weight = np.asarray(after["pi.1.weight"])
    head_bias = np.asarray(after["pi.1.bias"])
    logstd_weight_max = float(np.abs(head_weight[LOGSTD_ROWS]).max())
    sigma = np.exp(head_bias[LOGSTD_ROWS].astype(np.float64))
    sigma_ok = bool(np.all(np.abs(sigma - SIGMA) <= SIGMA_TOLERANCE))
    logstd_bytes_ok = bool(
        bytes_identical(head_weight[LOGSTD_ROWS],
                        np.asarray(on_disk["pi.1.weight"])[LOGSTD_ROWS])
        and bytes_identical(head_bias[LOGSTD_ROWS],
                            np.asarray(on_disk["pi.1.bias"])[LOGSTD_ROWS]))

    frozen_reach = gradient_reach(torch, frozen, on_disk)
    control_reach = gradient_reach(torch, control, on_disk)

    # A full-module pickle, so gate G8's semantics can be demonstrated NOW: the
    # file hash necessarily differs from J19A's because the file carries six
    # tensors J19A does not have, while every actor tensor inside it is
    # byte-identical. This is EVIDENCE, not an actor, and nothing is promoted.
    full_state = {k: np.asarray(v.detach().cpu().numpy())
                  for k, v in frozen.state_dict().items()}
    full_bytes = pickle.dumps(full_state, protocol=4)
    full_sha = hashlib.sha256(full_bytes).hexdigest()

    leaf_after = {name: sha256_file(HERE / J19A_LEAF_REL / name)
                  for name in sorted(PIN_J19A_LEAF)}
    leaf_unchanged = leaf_after == dict(PIN_J19A_LEAF)

    expected_zeroed = list(warm_start.DISABLED_GAIT_CLOCK_FEATURES)
    checks = {
        "P01_module_class_is_the_pinned_one": module_cls.__name__ == MODULE_CLASS,
        "P02_inference_only_false_builds_sixteen_keys": len(built_keys) == 16,
        "P03_critic_keys_present": bool(critic_present),
        "P04_critic_shapes_exact": bool(critic_shapes_ok),
        "P05_real_resolver_accepted_the_explicit_overlay":
            manifest_report["resolution"] == "explicit_manifest"
            and str(manifest_report["path"]) == str(overlay_path),
        "P06_real_resolver_validated_the_actor_digest":
            manifest_report["declared_actor_digest"] == PIN_J19A_ACTOR_DIGEST
            and manifest_report["validated_actor_digest"] == PIN_J19A_ACTOR_DIGEST,
        "P07_target_contract_comes_from_the_live_env_and_matches_the_source":
            env_names == tuple(source_names) and len(env_names) == ACTOR_WIDTH,
        "P08_real_transplant_copied_the_controller_block_and_zeroed_only_the_clock":
            sorted(transplant_report["copied_features"])
            == sorted(set(env_names) - set(expected_zeroed))
            and transplant_report["shared_features_zeroed"] == expected_zeroed
            and transplant_report["target_only_features_zeroed"] == []
            and transplant_report["source_only_features_dropped"] == [],
        "P09_real_transplant_reports_the_source_is_actor_only":
            transplant_report["source_state_is_actor_only"] is True
            and transplant_report["source_non_actor_keys"] == [],
        "P10_real_transplant_left_the_critic_untouched":
            transplant_report["target_non_actor_state_unchanged"] is True
            and transplant_report["critic_init_mode"] == "fresh_target_untouched",
        "P11_critic_is_the_TARGET_init_not_imported_from_the_source":
            bool(critic_comparison["exact"])
            and sorted(critic_comparison["keys"]) == sorted(CRITIC_KEYS)
            and critic_digest(transplanted) == fresh_critic_digest,
        "P12_production_load_is_strict_and_complete":
            tuple(strict_missing) == () and tuple(strict_unexpected) == (),
        "P13_all_ten_actor_tensors_byte_identical": all(per_key.values()),
        "P14_actor_digest_reproduces_j19a": digest_after == PIN_J19A_ACTOR_DIGEST,
        "P15_warm_start_own_comparator_calls_the_actor_exact":
            bool(actor_comparison["exact"])
            and float(actor_comparison["max_abs_diff"]) == 0.0,
        "P16_encoder_aliases_still_byte_identical": all(alias_ok.values()),
        "P17_clock_columns_are_already_exactly_zero": clock_zero,
        "P18_controller_memory_25_35_survives_alive":
            controller_identical and controller_alive,
        "P19_logstd_rows_are_byte_identical": logstd_bytes_ok,
        "P20_logstd_is_state_independent": logstd_weight_max == 0.0,
        "P21_sigma_is_exactly_the_qualified_value": sigma_ok,
        "P22_freeze_flags_live_on_the_module":
            bool(getattr(frozen, "_freeze_actor", None) is True
                 and getattr(frozen, "_freeze_logstd", None) is True),
        "P23_frozen_module_gives_the_actor_no_gradient":
            frozen_reach["actor_parameters_receiving_gradient"] == [],
        "P24_frozen_module_still_gives_the_critic_gradient":
            len(frozen_reach["critic_parameters_receiving_gradient"]) == len(CRITIC_KEYS),
        "P25_control_proves_the_probe_can_see_an_unfrozen_actor":
            len(control_reach["actor_parameters_receiving_gradient"]) > 0,
        "P26_actor_bytes_survive_the_gradient_probe":
            bool(frozen_reach["actor_bytes_unchanged_after_backward"]),
        "P27_full_module_hash_differs_from_j19a": full_sha != PIN_J19A_MODULE_STATE,
        "P28_j19a_leaf_is_byte_unchanged": bool(leaf_unchanged),
        "P29_bare_load_diagnostic_shows_the_silent_fresh_critic":
            tuple(sorted(bare_missing)) == tuple(sorted(CRITIC_KEYS))
            and tuple(bare_unexpected) == (),
    }

    result = {
        "kind": "K1 PROBE RESULT",
        "stage": STAGE,
        "verdict": "PASS" if all(checks.values()) else "FAIL_CLOSED",
        "checks": checks,
        "checks_passed": sum(1 for v in checks.values() if v),
        "checks_total": len(checks),
        "authoritative_path": {
            "what": "warm_start.transplant_actor_state, the function "
                    "train_ppo_mlp calls, driven by warm_start."
                    "resolve_source_actor_features against the explicit overlay",
            "why_not_a_bare_load": "a bare load_state_dict exercises a path "
                                   "production does not take. It would prove "
                                   "nothing about the column algebra, the "
                                   "gait-clock zeroing, or the transplant's "
                                   "refusal to touch the critic.",
            "mode": warm_start.DEFAULT_WARM_START_MODE,
            "zero_target_features": expected_zeroed,
            "source_checkpoint": str(source_dir),
            "source_actor_feature_manifest": str(overlay_path),
        },
        "module": {
            "class": module_cls.__name__,
            "state_keys_built": list(built_keys),
            "state_keys_built_count": len(built_keys),
            "critic_keys": list(CRITIC_KEYS),
            "critic_shapes": {k: list(v) for k, v in sorted(CRITIC_SHAPES.items())},
            "critic_parameters": 87809,
            "actor_parameters_independent": 76036,
        },
        "feature_contract": {
            "target_names_source": ENV_FEATURE_NAMES_REL,
            "target_names_provenance": "the LIVE environment's own recorded "
                                       "contract from a committed J19C cell, not "
                                       "the manifest that describes the source",
            "target_width": len(env_names),
            "source_width": len(source_names),
            "identical": env_names == tuple(source_names),
            "no_widening_no_25d_no_contralateral_feature": True,
        },
        "resolver_report": manifest_report,
        "transplant_report": {
            key: transplant_report[key] for key in sorted((
                "warm_start_name", "source_checkpoint", "source_config",
                "source_config_exists", "removed_feature_mode",
                "critic_init_mode", "copied_features",
                "source_only_features_dropped", "target_only_features_zeroed",
                "shared_features_zeroed", "zeroed_target_features",
                "zero_target_feature_reason", "source_state_is_actor_only",
                "source_non_actor_keys", "target_non_actor_keys_preserved",
                "target_non_actor_state_unchanged", "source_actor_digest",
                "target_actor_digest_after")) if key in transplant_report
        },
        "critic_comparison": {
            "exact": bool(critic_comparison["exact"]),
            "keys": list(critic_comparison["keys"]),
            "max_abs_diff": float(critic_comparison["max_abs_diff"]),
            "expected_digest": critic_comparison["expected_digest"],
            "actual_digest": critic_comparison["actual_digest"],
            "meaning": "the critic after the transplant IS the target's own fresh "
                       "initialisation. The source carries no critic, and none was "
                       "invented for it.",
        },
        "actor_comparison": {
            "exact": bool(actor_comparison["exact"]),
            "max_abs_diff": float(actor_comparison["max_abs_diff"]),
            "expected_digest": actor_comparison.get("expected_digest"),
            "actual_digest": actor_comparison.get("actual_digest"),
        },
        "byte_identity": {
            "per_key": per_key,
            "encoder_aliases": alias_ok,
            "method": "same dtype, same shape, same C-order bytes. Deliberately "
                      "NOT numpy.array_equal, which is numeric and would call a "
                      "float32 tensor equal to a float64 one and -0.0 equal to +0.0.",
        },
        "gait_clock_columns": {
            "columns": [0, 1],
            "features": expected_zeroed,
            "already_exactly_zero_bitwise": clock_zero,
            "why_it_matters": "gait_clock_enable is false in the pinned runtime, "
                              "so the transplant zeroes these target columns. "
                              "They are already zero, so the zeroing is a "
                              "provable no-op and cannot perturb the actor "
                              "before the first gradient.",
            "checked_on": ["pi.0.0.weight", "pi_encoder.0.weight"],
        },
        "controller_memory_columns": {
            "columns": [25, 35],
            "byte_identical_to_source": controller_identical,
            "alive_not_zeroed": controller_alive,
            "why_it_matters": "the controller-memory block is the part of the "
                              "actor contract the July lineage did not have. If "
                              "the transplant zeroed it along with the clock "
                              "columns, the warm-up would sample from a different "
                              "policy than the one J19B and J19C qualified.",
        },
        "sigma": {
            "value": [float(x) for x in sigma],
            "expected": SIGMA,
            "tolerance": SIGMA_TOLERANCE,
            "logstd_weight_abs_max": logstd_weight_max,
            "state_independent": logstd_weight_max == 0.0,
            "logstd_rows_byte_identical": logstd_bytes_ok,
        },
        "gradient_reach": {
            "frozen": frozen_reach,
            "unfrozen_control": control_reach,
            "why_a_control": "a probe that finds no actor gradient proves nothing "
                             "unless it can show it would have found one",
        },
        "bare_load_secondary_diagnostic": {
            "missing_keys": sorted(str(k) for k in bare_missing),
            "unexpected_keys": sorted(str(k) for k in bare_unexpected),
            "what_it_shows": "loading the actor-only export into a trainable "
                             "module leaves the six critic tensors at their fresh "
                             "initialisation, with NO exception and NO warning. "
                             "Recorded because it is worth knowing; it is NOT the "
                             "path the warm-up takes.",
            "is_not_the_authoritative_path": True,
        },
        "fresh_critic_digest": fresh_critic_digest,
        "fresh_critic_digest_is_diagnostic_only":
            "recorded, never pinned: this stage does not claim to reproduce any "
            "prior fresh-critic digest and invents no threshold on it",
        "full_module_state": {
            "sha256": full_sha,
            "j19a_module_state_sha256": PIN_J19A_MODULE_STATE,
            "hashes_differ": full_sha != PIN_J19A_MODULE_STATE,
            "why_they_must_differ": "the full module carries six critic tensors "
                                    "J19A does not have, so the FILE hash cannot "
                                    "match. The binding comparison is on the "
                                    "actor tensors, and those are byte-identical.",
            "it_is_not_an_actor_and_is_not_promoted": True,
        },
        "j19a_leaf_after": leaf_after,
        "j19a_leaf_unchanged": bool(leaf_unchanged),
        "inert": {
            "algo_train_called": False,
            "optimizer_constructed": False,
            "optimizer_step_taken": False,
            "ray_cluster_started": False,
            "environment_constructed": False,
            "environment_stepped": False,
            "rollout_performed": False,
            "actor_edited": False,
            "warmup_executed": False,
            "ppo_updates": 0,
        },
        "promotion": "NONE",
        "next_stage_authorized": False,
    }

    files = {
        "v26c_j20_probe_result.json": encode_json(result),
        "probe_full_module_state.pkl": full_bytes,
        "probe_actor_transplant_report.json": encode_json(
            {"note": "the FULL report returned by warm_start.transplant_actor_state, "
                     "the production function, recorded verbatim except for the "
                     "float norm/shape blocks that are summarised in the result.",
             "report": {k: v for k, v in transplant_report.items()
                        if k not in ("weight_norms",)}}),
        "probe_module_manifest.json": encode_json({
            "what_this_is": "the FULL 16-tensor module the warm-up would train: "
                            "the ten J19A actor tensors, byte-identical, plus six "
                            "freshly-initialised critic tensors.",
            "what_this_is_NOT": "an actor, a candidate, a checkpoint, a promotable "
                                "artefact or anything deployable. It carries no "
                                "optimizer state and was never trained.",
            "sha256": full_sha,
            "actor_digest": digest_after,
            "actor_tensors_byte_identical_to_j19a": all(per_key.values()),
            "critic_initialisation": "fresh, torch default, seed %d" % PROBE_SEED,
            "state_keys": sorted(full_state),
            "stage": STAGE,
            "promotion": "NONE",
        }),
    }
    receipt = {
        "kind": "EXECUTION RECEIPT",
        "stage": STAGE,
        "verdict": result["verdict"],
        "checks_passed": result["checks_passed"],
        "checks_total": result["checks_total"],
        "failed_checks": sorted(k for k, v in checks.items() if not v),
        "authoritative_path": "warm_start.transplant_actor_state",
        "inputs": {
            "prereg_sha256": sha256_file(HERE / PREREG_NAME),
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "config_sha256": sha256_file(HERE / CONFIG_NAME),
            "overlay_sha256": sha256_file(HERE / OVERLAY_NAME),
            "j19a_module_state_sha256": PIN_J19A_MODULE_STATE,
            "j19a_actor_digest": PIN_J19A_ACTOR_DIGEST,
            "baseline_pins_sha256": dict(sorted(BASELINE_PINS.items())),
            "architect_go_pins": go["pins"],
        },
        "j19a_leaf_unchanged": bool(leaf_unchanged),
        "warmup_command_preregistered": list(sealed_command("<OUTPUT_DIR>")),
        "warmup_not_executed_by_this_stage": True,
        "inert": result["inert"],
        "promotion": "NONE",
        "next_stage_authorized": False,
        "artefacts_sha256": {name: hashlib.sha256(payload).hexdigest()
                             for name, payload in sorted(files.items())},
    }
    files[RECEIPT_NAME] = encode_json(receipt)
    committed = commit_leaf(files)
    return {"leaf": committed["leaf"], "verdict": receipt["verdict"],
            "verification": committed["verification"],
            "checks_passed": result["checks_passed"],
            "checks_total": result["checks_total"]}


# ----------------------------------------------------------------- the commit --

def commit_leaf(files: dict) -> dict:
    """Write the leaf atomically, born invalid, verified after commit."""
    import os

    leaf_root = HERE / LEAF_ROOT
    final = leaf_root / LEAF_NAME
    if final.exists():
        raise J20Error("refusing to clobber an existing leaf: %s" % final)

    digest = hashlib.sha256()
    for name in sorted(files):
        digest.update(name.encode("utf-8"))
        digest.update(hashlib.sha256(files[name]).hexdigest().encode("ascii"))
    aggregate = digest.hexdigest()

    staging = leaf_root / (".staging_%s" % aggregate[:16])
    if staging.exists():
        raise J20Error("refusing to reuse an existing staging directory: %s" % staging)
    staging.mkdir(parents=True)
    (staging / INVALID_MARKER).write_bytes(
        b"born invalid; removed only after post-commit verification succeeds\n")
    for name in sorted(files):
        target = staging / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(files[name])
    os.rename(str(staging), str(final))

    problems = []
    committed = json.loads((final / RECEIPT_NAME).read_text(encoding="utf-8"))
    for name, expected in sorted(committed["artefacts_sha256"].items()):
        path = final / name
        if not path.exists():
            problems.append("committed receipt names a missing file: %s" % name)
            continue
        actual = sha256_file(path)
        if actual != expected:
            problems.append("committed %s hashes %s, receipt says %s"
                            % (name, actual, expected))
        if name in files and hashlib.sha256(files[name]).hexdigest() != actual:
            problems.append("committed %s differs from the staged bytes" % name)

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(final.relative_to(HERE)),
        "aggregate_digest": aggregate,
        "files_verified": len(committed["artefacts_sha256"]),
        "problems": problems,
        "ok": not problems,
        "method": "every leaf-relative path is re-resolved from the COMMITTED "
                  "receipt, re-hashed, and compared both to the receipt and to "
                  "the staged bytes",
    }
    (final / COMMIT_VERIFICATION_NAME).write_bytes(encode_json(verification))
    if problems:
        raise J20Error("post-commit verification FAILED; the invalid marker is "
                       "left in place: %s" % problems)
    (final / INVALID_MARKER).unlink()
    return {"leaf": str(final.relative_to(HERE)), "verification": verification}


# ------------------------------------------------------------------------ CLI --

def build_parser() -> argparse.ArgumentParser:
    """Command-line surface. Preflight is the default and writes nothing."""
    parser = argparse.ArgumentParser(
        description="V26C J20 - critic-only warm-up readiness (K0 audit, K1 probe)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--probe", action="store_true")
    parser.add_argument("--go-file", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Return 0 on success, 1 on failure. Writes nothing unless --probe."""
    args = build_parser().parse_args(argv)
    if args.probe:
        if not args.go_file:
            print("--probe requires --go-file", file=sys.stderr)
            return 1
        try:
            outcome = run_probe(args.go_file)
        except (J20Error, RuntimeError) as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"], "verdict": outcome["verdict"],
                          "checks": "%d/%d" % (outcome["checks_passed"],
                                               outcome["checks_total"])}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = preflight(verbose=True)
    if args.dry_run:
        print("\nplan")
        print("  K1 probe: build ONE module at the pinned geometry with")
        print("            inference_only=False, transplant the ten J19A actor")
        print("            tensors, and measure. No algo.train(), no optimizer,")
        print("            no Ray, no environment, no rollout.")
        print("\nthe warm-up command this stage PREREGISTERS and does NOT run:")
        print("  cd '%s'" % BASELINE)
        print("  " + sealed_command_text("<OUTPUT_DIR>"))
        print("\nthis stage authorises no warm-up. That is a separate GO.")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
