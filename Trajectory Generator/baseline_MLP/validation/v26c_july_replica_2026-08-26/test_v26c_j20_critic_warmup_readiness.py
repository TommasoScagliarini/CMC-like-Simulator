"""Static test suite for V26C J20 - critic-only warm-up readiness.

Runs no warm-up, no training, no rollout, no PPO, and builds no environment. It
checks four things:

  * that the readiness runner is STRUCTURALLY unable to train - asserted by
    walking its own AST, not by reading its docstring;
  * that the derived warm-up config is a mechanical transformation of the pinned
    runtime config and nothing else;
  * that the overlay is really consumable by the real warm_start loader, and
    that its 35 names come from committed evidence rather than from an author;
  * that the preregistered gates and the sealed command say what the runner says.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j20_critic_warmup_readiness.py
"""

from __future__ import annotations

import ast
import hashlib
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(BASELINE) not in sys.path:
    sys.path.insert(0, str(BASELINE))

sys.dont_write_bytecode = True

import v26c_j20_critic_warmup_readiness as R  # noqa: E402
import v26c_j20_derive_warmup_config as D  # noqa: E402

RUNNER_PATH = HERE / "v26c_j20_critic_warmup_readiness.py"
RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")
RUNNER_TREE = ast.parse(RUNNER_SOURCE)
PREREG = json.loads((HERE / R.PREREG_NAME).read_text(encoding="utf-8"))

CHECKS: list[tuple[str, bool, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(condition), detail))


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# ---------------------------------------------- the runner cannot train, at all

def called_names(tree: ast.AST) -> set[str]:
    """Every attribute or bare name that appears in call position."""
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        target = node.func
        if isinstance(target, ast.Attribute):
            names.add(target.attr)
        elif isinstance(target, ast.Name):
            names.add(target.id)
    return names


def imported_modules(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def test_structurally_inert() -> None:
    calls = called_names(RUNNER_TREE)
    for forbidden in R.FORBIDDEN_CALLS:
        check("A01 the runner never calls %s()" % forbidden,
              forbidden not in calls,
              "found in call position")
    imports = imported_modules(RUNNER_TREE)
    for forbidden in R.FORBIDDEN_IMPORTS:
        check("A02 the runner never imports %s" % forbidden,
              forbidden not in imports and not any(
                  name.startswith(forbidden + ".") for name in imports))

    # Exactly ONE backward pass exists, and it lives in the one function that
    # is documented to hold it. Anywhere else it would be a second, unaudited
    # place where the module could be pushed.
    backward_functions = set()
    for node in ast.walk(RUNNER_TREE):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for inner in ast.walk(node):
                if (isinstance(inner, ast.Call)
                        and isinstance(inner.func, ast.Attribute)
                        and inner.func.attr == "backward"):
                    backward_functions.add(node.name)
    check("A03 the only backward pass is inside gradient_reach",
          backward_functions == {"gradient_reach"}, str(sorted(backward_functions)))
    check("A04 there is exactly one backward call in the whole file",
          sum(1 for node in ast.walk(RUNNER_TREE)
              if isinstance(node, ast.Call)
              and isinstance(node.func, ast.Attribute)
              and node.func.attr == "backward") == 1)

    # No optimizer may be constructed anywhere.
    check("A05 no optimizer is constructed",
          not any(name in calls for name in ("Adam", "SGD", "AdamW", "RMSprop")))
    optim_attribute = any(
        isinstance(node, ast.Attribute) and node.attr == "optim"
        and isinstance(node.value, ast.Name) and node.value.id == "torch"
        for node in ast.walk(RUNNER_TREE))
    check("A06 torch.optim is never accessed as an attribute either",
          not optim_attribute)

    # The probe must not be reachable without a GO.
    probe_source = RUNNER_SOURCE[RUNNER_SOURCE.index("def run_probe"):]
    probe_source = probe_source[:probe_source.index("\ndef ")]
    check("A07 run_probe refuses to start without a valid GO",
          'if not go["valid"]' in probe_source)
    check("A08 run_probe refuses to start without a passing preflight",
          'if not report["ok"]' in probe_source)

    # C: the probe must exercise the REAL production transplant, not a bare load.
    check("A09a the runner calls warm_start.transplant_actor_state",
          "transplant_actor_state" in calls)
    check("A09b the runner calls the real feature resolver",
          "resolve_source_actor_features" in calls)
    check("A09c the runner calls warm_start's own critic comparator",
          "compare_non_actor_states" in calls)
    check("A09d the runner calls warm_start's own actor comparator",
          "compare_actor_states" in calls)
    check("A09e the runner imports warm_start",
          "warm_start" in imports)
    check("A09f the transplant is driven with the production zeroing policy",
          "DISABLED_GAIT_CLOCK_FEATURES" in RUNNER_SOURCE
          and "DEFAULT_WARM_START_MODE" in RUNNER_SOURCE)
    check("A09g the target contract is read from the live env record, not the "
          "source manifest",
          "ENV_FEATURE_NAMES_REL" in RUNNER_SOURCE
          and "kinematics.npz" in R.ENV_FEATURE_NAMES_REL)

    check("A09 the runner declares the warm-up stage but does not implement it",
          R.WARMUP_STAGE not in calls and "def run_warmup" not in RUNNER_SOURCE
          and "def warmup" not in RUNNER_SOURCE)


def test_preflight_is_pure() -> None:
    """--preflight-only must write nothing and must not import torch."""
    before = sorted((str(p.relative_to(HERE)), p.stat().st_size)
                    for p in HERE.rglob("*") if p.is_file())
    completed = subprocess.run(
        [sys.executable, str(RUNNER_PATH), "--preflight-only"],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    after = sorted((str(p.relative_to(HERE)), p.stat().st_size)
                   for p in HERE.rglob("*") if p.is_file())
    check("B01 --preflight-only exits 0", completed.returncode == 0,
          completed.stderr[-400:])
    check("B02 --preflight-only writes nothing", before == after,
          str(set(after) ^ set(before)))
    check("B03 --preflight-only reports READY", "READY" in completed.stdout,
          completed.stdout[-300:])

    probe = subprocess.run(
        [sys.executable, "-c",
         "import sys; sys.path.insert(0, %r); sys.dont_write_bytecode = True;\n"
         "import v26c_j20_critic_warmup_readiness as R;\n"
         "R.preflight(verbose=False);\n"
         "print('torch' in sys.modules, 'ray' in sys.modules)" % str(HERE)],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    check("B04 the preflight imports neither torch nor ray",
          probe.stdout.strip() == "False False",
          probe.stdout.strip() + probe.stderr[-300:])


def test_preflight_passes() -> None:
    report = R.preflight(verbose=False)
    check("B05 the preflight verdict is READY", report["ok"],
          "; ".join(report["problems"]))
    check("B06 every pin matches",
          report["pins_matching"] == report["pins_checked"]
          and report["pins_checked"] >= 20,
          "%d/%d" % (report["pins_matching"], report["pins_checked"]))
    check("B07 the destination leaf does not exist yet",
          report["destination"]["ok"])
    check("B08 the ctor decodes to the pinned geometry, with no import",
          report["ctor"]["n_actor"] == 35 and report["ctor"]["n_full"] == 84
          and report["ctor"]["inference_only"] is True
          and report["ctor"]["vf_share_layers"] is False)
    check("B09 the derived config inherits all four untouchable sections",
          all(report["config"]["sections_byte_inherited"].values()),
          str(report["config"]["sections_byte_inherited"]))
    check("B10 all thirteen overlay sources agree",
          report["overlay"]["sources_agreeing"] == 13
          and report["overlay"]["sources_checked"] == 13,
          "%d/%d" % (report["overlay"]["sources_agreeing"],
                     report["overlay"]["sources_checked"]))


# ------------------------------------------------------------ the sealed pins

def test_pins() -> None:
    check("C01 the preregistration is sealed to a real hash, not a placeholder",
          len(R.PIN_PREREG) == 64 and all(c in "0123456789abcdef" for c in R.PIN_PREREG),
          R.PIN_PREREG)
    check("C02 the sealed prereg hash is the prereg's actual hash",
          R.PIN_PREREG == sha256_file(HERE / R.PREREG_NAME))
    check("C03 the prereg check is UNCONDITIONAL - no placeholder branch",
          "PENDING" not in RUNNER_SOURCE and "SEALED_BY_THE_ARCHITECT" not in RUNNER_SOURCE)
    check("C04 the prereg contains no self-hash",
          R.PIN_PREREG not in (HERE / R.PREREG_NAME).read_text(encoding="utf-8"))
    check("C05 the prereg declares contains_no_self_hash",
          PREREG["contains_no_self_hash"] is True)

    for name, expected in sorted(R.PIN_J19A_LEAF.items()):
        path = HERE / R.J19A_LEAF_REL / name
        check("C06 J19A pin matches: %s" % name,
              path.is_file() and sha256_file(path) == expected)
    check("C07 the J19A leaf holds exactly seven files",
          sum(1 for p in (HERE / R.J19A_LEAF_REL).rglob("*") if p.is_file()) == 7)
    check("C08 the config pin matches",
          sha256_file(HERE / R.CONFIG_NAME) == R.PIN_CONFIG)
    check("C09 the overlay pin matches",
          sha256_file(HERE / R.OVERLAY_NAME) == R.PIN_OVERLAY)
    for name, expected in sorted(R.BASELINE_PINS.items()):
        check("C10 baseline pin matches: %s" % name,
              sha256_file(BASELINE / name) == expected)
    check("C11 the telemetry test pin matches",
          sha256_file(BASELINE / R.TELEMETRY_TEST_REL) == R.PIN_TELEMETRY_TEST)
    for name in R.GO_REQUIRED_PINS:
        check("C12 the GO pin target exists: %s" % name, (HERE / name).exists())
    check("C13 the GO pins the prereg, the runner and this suite",
          {R.PREREG_NAME, "v26c_j20_critic_warmup_readiness.py",
           "test_v26c_j20_critic_warmup_readiness.py"} <= set(R.GO_REQUIRED_PINS))


# ------------------------------------------------------- the config derivation

def test_derivation() -> None:
    report = D.check()
    check("D01 the derived config matches the derivation exactly",
          report["matches"] and not report["problems"],
          str(report.get("problems")))
    check("D02 the source config is the pinned runtime config",
          report["source_sha256"] == R.PIN_SOURCE_CONFIG)
    check("D03 the derivation is a closed map of eleven substitutions",
          report["substitutions"] == 11)
    check("D04 fourteen source lines are substituted",
          report["source_lines_substituted"] == 14)
    check("D05 the remaining 228 source lines are inherited verbatim",
          report["source_lines_inherited_verbatim"] == 228)
    check("D06 fifteen invariant lines are asserted",
          report["invariants_asserted"] == 15)
    check("D07 the derived hash is the pinned config hash",
          report["sha256"] == R.PIN_CONFIG)

    # The map may not touch a line inside an untouchable section.
    source_lines = (D.REPO / D.SOURCE_REL).read_text(encoding="utf-8").splitlines()
    for entry in D.SUBSTITUTIONS:
        for number in entry["lines"]:
            section = D.section_of_line(source_lines, number - 1)
            check("D08 %s does not touch an untouchable section" % entry["id"],
                  section not in D.UNTOUCHABLE_SECTIONS, section)

    # The derivation must REFUSE a source that drifted, not follow it.
    mutated = list(source_lines)
    mutated[16] = "  train_batch_size: 9999"
    validation = D.validate_map(mutated)
    check("D09 the derivation refuses a source whose pinned line changed",
          not validation["ok"] and any("S2" in p for p in validation["problems"]),
          str(validation["problems"][:2]))
    mutated = list(source_lines)
    mutated[20] = "  gamma: 0.5"
    validation = D.validate_map(mutated)
    check("D10 the derivation refuses a source whose INVARIANT line changed",
          not validation["ok"] and any("invariant line 21" in p
                                       for p in validation["problems"]),
          str(validation["problems"][:2]))

    # The substantive claim is about the file the derivation actually OPENS,
    # not about which names appear in its prose. source_path() is the single
    # place it resolves a source, and it re-hashes what it opens.
    opened = D.source_path()
    check("D11 the derivation opens exactly the pinned runtime config",
          opened == (D.REPO / D.SOURCE_REL).resolve()
          or opened == D.REPO / D.SOURCE_REL,
          str(opened))
    check("D11b what it opens hashes to the pinned source",
          sha256_file(opened) == R.PIN_SOURCE_CONFIG)
    check("D11c training_exnovo_cfg.yaml is never on any path it builds",
          D.FORBIDDEN_SOURCE not in str(opened)
          and D.FORBIDDEN_SOURCE not in D.SOURCE_REL
          and D.FORBIDDEN_SOURCE not in D.TARGET_NAME)
    # And it must refuse a source whose bytes are not the pinned ones at all.
    original = D.SOURCE_SHA256
    try:
        D.SOURCE_SHA256 = "0" * 64
        try:
            D.source_path()
            refused = False
        except D.J20DerivationError:
            refused = True
    finally:
        D.SOURCE_SHA256 = original
    check("D11d the derivation refuses a source whose hash is not the pinned one",
          refused)

    import yaml

    derived = yaml.safe_load((HERE / R.CONFIG_NAME).read_text(encoding="utf-8"))
    source = yaml.safe_load((D.REPO / D.SOURCE_REL).read_text(encoding="utf-8"))
    for section in D.UNTOUCHABLE_SECTIONS:
        check("D12 section %s is inherited unchanged" % section,
              derived[section] == source[section])
    check("D13 the reward section carries all 125 keys",
          len(derived["reward"]) == 125, str(len(derived["reward"])))
    check("D14 freeze_actor and freeze_logstd are both true",
          derived["model"]["freeze_actor"] is True
          and derived["model"]["freeze_logstd"] is True)
    check("D15 the July mechanics are in the config, all eleven values",
          derived["ppo"]["train_batch_size"] == 4096
          and derived["ppo"]["minibatch_size"] == 512
          and derived["ppo"]["num_epochs"] == 10
          and derived["ppo"]["lr"] == 1e-04
          and derived["ppo"]["gamma"] == 0.99
          and derived["ppo"]["lam"] == 0.9
          and derived["ppo"]["clip_param"] == 0.2
          and derived["ppo"]["kl_coeff"] == 0.2
          and derived["ppo"]["kl_target"] == 0.01
          and derived["ppo"]["vf_clip_param"] == 10.0
          and derived["ppo"]["vf_loss_coeff"] == 1.0,
          json.dumps(derived["ppo"]))
    check("D16 thirteen runners, fourteen Ray CPUs, exact-start OFF",
          derived["parallelism"] == {"num_env_runners": 13, "ray_num_cpus": 14,
                                     "exact_start_sampling": False},
          json.dumps(derived["parallelism"]))
    check("D17 exactly one iteration",
          derived["simulation"]["iterations"] == 1)
    check("D18 a single nominal start and an EMPTY choices list",
          derived["simulation"]["episode_start_offset_s"] == 1.956870983805102
          and derived["simulation"]["episode_start_offset_choices_s"] == [])
    check("D19 the penetration thresholds are inherited, not restated",
          derived["simulation"]["grf_penetration_termination_m"]
          == source["simulation"]["grf_penetration_termination_m"] == 0.028)


# ------------------------------------------------------------------ the overlay

def test_overlay() -> None:
    import numpy as np
    import warm_start

    path = HERE / R.OVERLAY_NAME
    payload = json.loads(path.read_text(encoding="utf-8"))

    # The decisive check: the REAL loader must accept it. The previous draft of
    # this overlay was well documented and unusable; only this check tells the
    # two apart.
    try:
        names, loaded = warm_start._load_actor_feature_manifest(path)
        loaded_ok, why = True, ""
    except Exception as error:  # noqa: BLE001 - that is what is being tested
        names, loaded, loaded_ok, why = (), {}, False, repr(error)
    check("E01 the REAL warm_start loader accepts the overlay", loaded_ok, why)
    check("E02 it yields exactly 35 names in order",
          len(names) == 35 and names[0] == "gait_phase_sin"
          and names[-1] == "pros_ankle_angle_sea_u")
    check("E03 the three consumable keys are at the TOP level",
          all(key in payload for key in
              ("actor_feature_names", "actor_feature_count", "actor_digest")))
    check("E04 the declared count matches the list",
          payload["actor_feature_count"] == len(payload["actor_feature_names"]))
    check("E05 the declared actor_digest is J19A's",
          payload["actor_digest"] == R.PIN_J19A_ACTOR_DIGEST)
    check("E06 the names_join hash is self-consistent",
          hashlib.sha256("\n".join(names).encode("utf-8")).hexdigest()
          == payload["the_feature_list"]["names_join_sha256"])

    # Provenance: the names must come from committed evidence.
    agreeing = 0
    for relative in payload["provenance_of_the_names"]["sources"]:
        target, _, field = relative.partition("#")
        full = HERE / target
        if not full.is_file():
            continue
        if target.endswith(".npz"):
            with np.load(full) as bundle:
                found = tuple(str(x) for x in bundle["actor_feature_names"])
        else:
            node = json.loads(full.read_text(encoding="utf-8"))
            for part in field.split("."):
                node = node[part]
            if isinstance(node, dict):
                node = next(v for v in node.values()
                            if isinstance(v, list) and len(v) == 35)
            found = tuple(str(x) for x in node)
        if found == tuple(names):
            agreeing += 1
    check("E07 all thirteen committed sources agree with the overlay",
          agreeing == 13, str(agreeing))

    # And the reason it exists must still be true.
    leaf = json.loads((HERE / R.J19A_MODULE_REL
                       / "actor_feature_manifest.json").read_text("utf-8"))
    check("E08 the J19A leaf manifest still lacks actor_feature_names",
          "actor_feature_names" not in leaf)
    check("E09 the overlay names the J19A manifest by its real hash",
          payload["overlays"]["sha256"]
          == R.PIN_J19A_LEAF["rl_module/actor_feature_manifest.json"])
    check("E10 the overlay does not claim deployability",
          "deployable" not in payload
          and payload["what_this_overlay_does_not_do"][
              "does_not_confer_deployability"] is True)
    check("E11 the overlay declares the stale status field",
          any(entry["field"] == "status"
              for entry in payload["fields_of_the_underlying_manifest_declared_STALE"]))

    # The gait-clock zeroing must really be a no-op.
    import pickle

    state = pickle.loads((HERE / R.J19A_MODULE_REL
                          / "module_state.pkl").read_bytes())
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        block = np.ascontiguousarray(np.asarray(state[key])[:, R.CLOCK_COLUMNS])
        check("E12 the clock columns of %s are exactly zero" % key,
              block.tobytes(order="C")
              == np.zeros_like(block).tobytes(order="C"))


# ------------------------------------------------------------ the sealed command

def test_sealed_command() -> None:
    tokens = list(R.sealed_command("<OUTPUT_DIR>"))
    documented = list(PREREG["the_sealed_warmup_command"]["tokens"])
    restored = [
        token.replace("<BASELINE>", str(BASELINE)).replace("<J20_DIR>", str(HERE))
        for token in documented
    ]
    check("F01 the runner and the preregistration agree token by token",
          tokens == restored,
          str([a for a, b in zip(tokens, restored) if a != b][:3]))
    check("F02 the command is one-shot: --worker-process is present",
          "--worker-process" in tokens)
    check("F03 the command carries no --resume-from",
          "--resume-from" not in tokens)
    check("F04 the command asks for exactly one iteration",
          tokens[tokens.index("--iterations") + 1] == "1")
    check("F05 the command names the pinned config",
          tokens[tokens.index("--config") + 1] == str(HERE / R.CONFIG_NAME))
    check("F06 the command names the overlay, not the leaf manifest",
          tokens[tokens.index("--warm-start-raw-source-feature-manifest") + 1]
          == str(HERE / R.OVERLAY_NAME))
    check("F07 the command names the J19A module directory as the source",
          tokens[tokens.index("--warm-start-raw-source") + 1]
          == str(HERE / R.J19A_MODULE_REL))
    check("F08 the command freezes both the actor and the log-std",
          "--freeze-actor" in tokens and "--freeze-logstd" in tokens)
    check("F09 the command declares the asymmetric actor-critic",
          "--asymmetric-actor-critic" in tokens)
    check("F10 the command does not touch the run registry",
          "--no-update-history" in tokens)
    check("F11 the command retains the iteration milestone, for gate G9",
          "--retain-iteration-checkpoints" in tokens)
    check("F12 no scientific parameter is on the command line",
          not any(token.startswith(("--lr", "--gamma", "--train-batch-size",
                                    "--minibatch-size", "--num-epochs",
                                    "--clip-param", "--kl-coeff"))
                  for token in tokens))
    check("F13 this stage does not run that command",
          "subprocess" not in RUNNER_SOURCE and "os.system" not in RUNNER_SOURCE)


# ---------------------------------------------------------------------- the GO

def valid_go() -> dict:
    return {
        "kind": "ARCHITECT GO - SINGLE EXECUTION",
        "stage": R.GO_REQUIRED_STAGE,
        "authorises_execution": True,
        "pinned_artefacts_sha256": {
            name: sha256_file(HERE / name) for name in R.GO_REQUIRED_PINS
        },
    }


def test_go_validation() -> None:
    check("G01 a well-formed GO validates", R.validate_go(valid_go())["valid"])

    bad = valid_go()
    bad["stage"] = "V26C_J20_CRITIC_WARMUP"
    check("G02 a GO for the WARM-UP stage is refused by the readiness runner",
          not R.validate_go(bad)["valid"])

    bad = valid_go()
    bad["authorises_execution"] = "yes"
    check("G03 authorises_execution must be exactly true",
          not R.validate_go(bad)["valid"])

    for forbidden in ("authorises_warmup", "authorises_training",
                      "authorises_rollout", "authorises_ppo"):
        bad = valid_go()
        bad[forbidden] = True
        check("G04 a readiness GO may not set %s" % forbidden,
              not R.validate_go(bad)["valid"])

    bad = valid_go()
    bad["pinned_artefacts_sha256"].pop(R.PREREG_NAME)
    check("G05 a GO missing a required pin is refused",
          not R.validate_go(bad)["valid"])

    bad = valid_go()
    bad["pinned_artefacts_sha256"][R.PREREG_NAME] = "0" * 64
    check("G06 a GO with a stale pin is refused",
          not R.validate_go(bad)["valid"])

    bad = valid_go()
    bad["pinned_artefacts_sha256"]["v26c_j19c_heldout_g_i.py"] = "0" * 64
    check("G07 a GO pinning something outside the scope is refused",
          not R.validate_go(bad)["valid"])

    check("G08 an absent GO file is refused, not defaulted",
          not R.load_go(str(HERE / "no_such_go.json"))["valid"])
    check("G09 validate_go is pure: it opens no GO file itself",
          "open(" not in RUNNER_SOURCE[RUNNER_SOURCE.index("def validate_go"):
                                       RUNNER_SOURCE.index("def load_go")])


# ---------------------------------------------------- the preregistered gates

def test_gates() -> None:
    gates = PREREG["preregistered_gates_for_the_single_warmup_execution"]["gates"]
    ids = [gate["id"] for gate in gates]
    check("H01 exactly twelve gates are preregistered", len(gates) == 12, str(len(gates)))
    check("H02 the gates are G1..G12 in order",
          ids == ["G%d" % n for n in range(1, 13)], str(ids))
    check("H03 no threshold is invented by this stage",
          PREREG["preregistered_gates_for_the_single_warmup_execution"][
              "thresholds_invented_in_this_stage"] == 0)

    by_id = {gate["id"]: gate for gate in gates}
    check("H04 G1 requires exactly one logical iteration",
          "exactly one row" in by_id["G1"]["criterion"])
    check("H05 G2 pins BOTH the total and the per-iteration delta at 4096",
          "4096" in by_id["G2"]["criterion"] and "delta" in by_id["G2"]["criterion"])
    check("H06 G3 explicitly sets NO explained-variance threshold",
          "NO threshold" in by_id["G3"]["explicitly_not_a_performance_gate"])
    check("H07 G3's criterion contains no numeric bar",
          not any(ch.isdigit() for ch in by_id["G3"]["criterion"]),
          by_id["G3"]["criterion"])
    check("H08 G4 requires a zero or numerically-null mean KL",
          "0.0" in by_id["G4"]["criterion"] and "1e-09" in by_id["G4"]["criterion"])
    check("H09 G5 pins the actor digest itself",
          R.PIN_J19A_ACTOR_DIGEST in by_id["G5"]["criterion"])
    check("H10 G6 checks the log-std separately from the actor",
          "byte-identical" in by_id["G6"]["criterion"]
          and "0.005" in by_id["G6"]["criterion"])
    check("H11 G7 requires the critic digest to CHANGE",
          "differ" in by_id["G7"]["criterion"])
    check("H12 G8 requires the full module hash NOT to equal J19A's",
          "MUST NOT equal" in by_id["G8"]["criterion"]
          and R.PIN_J19A_MODULE_STATE in by_id["G8"]["criterion"])
    check("H13 G8 makes the actor TENSORS the binding comparison",
          "actor TENSORS" in by_id["G8"]["why"])
    check("H14 G9 requires a restorable optimizer",
          "optimizer" in by_id["G9"]["criterion"]
          and "state.pkl" in by_id["G9"]["criterion"])
    check("H15 G10 forbids retry, crash and timeout",
          all(word in by_id["G10"]["criterion"]
              for word in ("retry", "restart", "timeout")))
    check("H16 G11 asserts the telemetry ARRIVED and bounds nothing",
          "missing_telemetry_rows" in by_id["G11"]["criterion"]
          and "not bounded" in by_id["G11"]["explicitly_not_a_behavioural_gate"])
    check("H17 G12 requires the J19A leaf to be byte-unchanged",
          "seven J19A leaf files" in by_id["G12"]["criterion"])

    probe = PREREG["K1_probe"]
    checks_listed = probe["the_preregistered_checks"]
    check("H18 the probe preregisters twenty-nine checks",
          len(checks_listed) == 29, str(len(checks_listed)))
    check("H18b the preregistered check ids are P01..P29 in order",
          [entry.split()[0] for entry in checks_listed]
          == ["P%02d" % n for n in range(1, 30)])
    check("H19 the probe declares it never calls algo.train()",
          "call algo.train()" in probe["what_it_never_does"])
    check("H19b the probe declares the REAL production path as authoritative",
          "warm_start.transplant_actor_state" in probe["authoritative_path"]["what"])
    check("H19c the bare load is declared a SECONDARY diagnostic only",
          probe["bare_load_is_secondary_only"]["is_not_the_authoritative_path"] is True)
    autograd = probe["the_one_place_it_touches_autograd"]
    check("H20 the probe declares no optimizer and no step",
          autograd["no_optimizer_is_constructed_and_no_step_is_taken"] is True)
    check("H20b the probe declares an UNFROZEN control arm",
          "UNFROZEN" in autograd["control_arm"], autograd["control_arm"][:80])
    check("H20c the probe declares the post-backward byte re-verification",
          "byte-identical" in autograd["safety"]
          and "re-hashed" in autograd["safety"], autograd["safety"][:80])
    check("H20d the runner really builds both arms",
          "freeze=True" in RUNNER_SOURCE and "freeze=False" in RUNNER_SOURCE)


def test_prereg_consistency() -> None:
    check("I01 the prereg's config hash is the real one",
          PREREG["config_derivation"]["target_sha256"] == R.PIN_CONFIG)
    check("I02 the prereg's source hash is the real one",
          PREREG["config_derivation"]["source_sha256"] == R.PIN_SOURCE_CONFIG)
    check("I03 the prereg's overlay hash is the real one",
          PREREG["manifest_overlay"]["overlay_sha256"] == R.PIN_OVERLAY)
    check("I04 the prereg's actor hashes are the real ones",
          PREREG["actor_under_warmup"]["module_state_sha256"]
          == R.PIN_J19A_MODULE_STATE
          and PREREG["actor_under_warmup"]["actor_digest"]
          == R.PIN_J19A_ACTOR_DIGEST)
    check("I05 the prereg's entry evidence matches the pins",
          PREREG["entry_evidence_pinned"]["j19b_receipt_sha256"] == R.PIN_J19B_RECEIPT
          and PREREG["entry_evidence_pinned"]["j19c_receipt_sha256"] == R.PIN_J19C_RECEIPT)
    check("I06 the prereg lists the eight telemetry fields the code declares",
          tuple(PREREG["training_health_telemetry"]["fields"])
          == R.TRAINING_HEALTH_FIELDS)
    partition = PREREG["training_health_telemetry"]["observed_and_missing_partition"]
    check("I06b the prereg states the partition rule",
          "sum to 1" in partition["rule"]
          and partition["observed_rows_is_not_a_row_counter"] is True)
    check("I06c the prereg denies that zero-over-zero is health",
          "silence" in partition["zero_incidents_over_zero_observed_rows"])
    containment = PREREG["training_health_telemetry"]["containment"]
    check("I06d the prereg enumerates the four guarded boundaries",
          sorted(containment["guarded_boundaries"])
          == ["extraction", "jsonl", "metrics_logger", "tensorboard"],
          str(containment["guarded_boundaries"]))
    check("I06e the prereg records that non-health TB failures still propagate",
          containment["non_health_tensorboard_failures_still_propagate"] is True)
    check("I07 the prereg's GO pin list matches the runner's",
          tuple(PREREG["execution_discipline"]["go_must_pin"]) == R.GO_REQUIRED_PINS)
    july = PREREG["lineage"]["july_is_informative_only"]
    check("I08 the prereg denies July any operational input",
          "no July checkpoint" in july and "operational input" in july, july[:90])
    check("I08b the prereg names the operational lineage exactly",
          PREREG["lineage"]["operational"]
          == "August V26 imitation -> J2 35D -> J8 -> J18 c13 -> J19A")
    check("I08c the prereg holds the one-actor-35D invariant",
          "35-wide" in PREREG["lineage"]["one_actor_35d"]
          and "no widening" in PREREG["lineage"]["one_actor_35d"])
    b0820 = PREREG["actor_under_warmup"]["b0820_39d_is_explicitly_NOT_the_lineage"]
    check("I09 the prereg demotes B0820 to methodological precedent",
          "methodological precedent only" in b0820
          and "39-wide" in b0820 and "never closed-loop qualified" in b0820,
          b0820[:90])
    check("I10 the prereg declares the telemetry as observation only",
          "OBSERVATION ONLY" in PREREG["training_health_telemetry"]["nature"])
    check("I11 the prereg carries a deferred TODO list",
          len(PREREG["deferred_todo"]) >= 8)
    check("I12 the prereg promotes nothing",
          PREREG["what_this_stage_cannot_establish"]["no_promotion"].startswith(
              "nothing is promoted"))
    check("I13 the prereg does not modify training_exnovo_cfg.yaml",
          "training_exnovo_cfg.yaml" in PREREG["invariants"]["not_modified"])


# ------------------------------------------------- the telemetry it depends on

def test_telemetry_suite_passes() -> None:
    completed = subprocess.run(
        [sys.executable, str(BASELINE / R.TELEMETRY_TEST_REL)],
        cwd=str(BASELINE / "validation"), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    check("J01 the training-health telemetry suite passes",
          completed.returncode == 0,
          completed.stdout[-300:] + completed.stderr[-300:])
    check("J02 it reports every check green",
          "checks passed" in completed.stdout
          and completed.stdout.strip().split("/")[0].split()[-1]
          == completed.stdout.strip().split("/")[1].split()[0],
          completed.stdout.strip()[-120:])


def main() -> int:
    test_structurally_inert()
    test_preflight_is_pure()
    test_preflight_passes()
    test_pins()
    test_derivation()
    test_overlay()
    test_sealed_command()
    test_go_validation()
    test_gates()
    test_prereg_consistency()
    test_telemetry_suite_passes()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
