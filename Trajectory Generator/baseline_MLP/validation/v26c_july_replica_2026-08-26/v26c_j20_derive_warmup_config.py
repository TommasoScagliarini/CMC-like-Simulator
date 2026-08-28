"""V26C J20 - mechanical derivation of the critic-only warm-up config.

The warm-up config is NOT hand-written. It is derived by this tool from the
runtime config that the whole V26C chain already pins - the resolved snapshot of
MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter - by applying a CLOSED, PINNED
substitution map. The transformation is the auditable artefact: everything the
map does not touch is byte-inherited, and that is provable line by line.

Why a tool and not a hand-written YAML: the architect's instruction is to keep
the physical runtime, FSM v3, the detector, the morphology corridor, the reward
and every current threshold INTEGRALLY unchanged, and to change only the July
warm-up mechanics. A hand-written config would make "unchanged" a claim; a
derivation makes it a measurement. It is the same discipline used for J19B (from
J16) and J19C (from J19B).

``training_exnovo_cfg.yaml`` is NOT read and NOT modified. The pinned source is
a committed run artefact, not an editable project config.

Usage:
    python v26c_j20_derive_warmup_config.py --check
    python v26c_j20_derive_warmup_config.py --emit

``--check`` writes nothing. ``--emit`` refuses to clobber a file whose bytes
already differ from the derivation.
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import sys

sys.dont_write_bytecode = True

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]

# --------------------------------------------------------------------------
# The pinned source. A committed run artefact, never an editable config.
# --------------------------------------------------------------------------
SOURCE_REL = (
    "Trajectory Generator/runs/training/"
    "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml"
)
SOURCE_SHA256 = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"
SOURCE_LINES = 242

TARGET_NAME = "v26c_j20_warmup_cfg.yaml"

# The config this stage must NOT touch. Named so a test can assert it is never
# opened, and so the intent is on the record rather than merely implied.
FORBIDDEN_SOURCE = "training_exnovo_cfg.yaml"

# --------------------------------------------------------------------------
# THE CLOSED SUBSTITUTION MAP.
#
# Each entry pins BOTH the 1-based line number and the exact expected text.
# Pinning only the text would be ambiguous where two lines coincide; pinning
# only the line number would silently follow source drift. Pinning both makes
# the derivation refuse to run against anything but the exact pinned bytes.
#
# `new` is a tuple of replacement lines, so one source line may become several
# or several may collapse into one.
# --------------------------------------------------------------------------
SUBSTITUTIONS = (
    {
        "id": "S1",
        "section": "model",
        "lines": (14,),
        "old": ("  freeze_actor: false",),
        "new": ("  freeze_actor: true",),
        "why": "critic-only warm-up: the actor tower takes no gradient. This is "
               "the single flag that makes the stage what it is.",
    },
    {
        "id": "S2",
        "section": "ppo",
        "lines": (17,),
        "old": ("  train_batch_size: 4608",),
        "new": ("  train_batch_size: 4096",),
        "why": "July's warm-up batch. 4096 is TIMESTEPS PER ITERATION, not a "
               "number of environments.",
    },
    {
        "id": "S3",
        "section": "ppo",
        "lines": (19,),
        "old": ("  num_epochs: 1",),
        "new": ("  num_epochs: 10",),
        "why": "July's warm-up used 10 epochs over the single sampled batch.",
    },
    {
        "id": "S4",
        "section": "ppo",
        "lines": (20,),
        "old": ("  lr: 5.0e-07",),
        "new": ("  lr: 1.0e-04",),
        "why": "July's warm-up learning rate. The ex-novo 5e-07 is a "
               "conservative-actor-update rate and does not apply here.",
    },
    {
        "id": "S5",
        "section": "ppo",
        "lines": (23,),
        "old": ("  clip_param: 0.05",),
        "new": ("  clip_param: 0.2",),
        "why": "July's warm-up value. Inert for a frozen actor - the policy "
               "ratio is identically 1 - but pinned because it was pinned then.",
    },
    {
        "id": "S6",
        "section": "ppo",
        "lines": (24,),
        "old": ("  kl_coeff: 1.0",),
        "new": ("  kl_coeff: 0.2",),
        "why": "July's warm-up value. Also inert for a frozen actor, and "
               "recorded for the same reason.",
    },
    {
        "id": "S7",
        "section": "parallelism",
        "lines": (29,),
        "old": ("  num_env_runners: 12",),
        "new": ("  num_env_runners: 13",),
        "why": "July's warm-up parallelism.",
    },
    {
        "id": "S8",
        "section": "parallelism",
        "lines": (30,),
        "old": ("  ray_num_cpus: 13",),
        "new": ("  ray_num_cpus: 14",),
        "why": "July's warm-up Ray CPU budget.",
    },
    {
        "id": "S9",
        "section": "parallelism",
        "lines": (31,),
        "old": ("  exact_start_sampling: true",),
        "new": ("  exact_start_sampling: false",),
        "why": "with the exact-start contract OFF, rollout_fragment_length is "
               "the literal string 'auto' (train_ppo_mlp.py:1342-1347), which "
               "is what July used. It also removes the divisibility requirement "
               "that 4096 could not satisfy against 13 runners.",
    },
    {
        "id": "S10",
        "section": "simulation",
        "lines": (36,),
        "old": ("  iterations: 55",),
        "new": ("  iterations: 1",),
        "why": "exactly one logical iteration. July's warm-up was one; this is "
               "also the preregistered gate.",
    },
    {
        "id": "S11",
        "section": "simulation",
        "lines": (40, 41, 42, 43),
        "old": (
            "  episode_start_offset_choices_s:",
            "  - 1.756870983805102",
            "  - 1.956870983805102",
            "  - 2.156870983805102",
        ),
        "new": ("  episode_start_offset_choices_s: []",),
        "why": "single nominal start. With the choices list empty the sampler "
               "falls back to episode_start_offset_s, which line 39 already "
               "holds at the nominal 1.956870983805102 and which S-nothing "
               "touches.",
    },
)

# --------------------------------------------------------------------------
# Values the map must NOT change, asserted present at their pinned line.
# These are the fields the architect enumerated as "keep July's" where July and
# the pinned runtime already agree, plus the invariants that make the stage
# scientifically meaningful. Asserting them is what turns "we did not touch it"
# into a check.
# --------------------------------------------------------------------------
INVARIANT_LINES = (
    (7, "  asymmetric_actor_critic: true"),
    (13, "  freeze_logstd: true"),
    (15, "  seed: 123"),
    (18, "  minibatch_size: 512"),
    (21, "  gamma: 0.99"),
    (22, "  lam: 0.9"),
    (25, "  kl_target: 0.01"),
    (26, "  vf_clip_param: 10.0"),
    (27, "  vf_loss_coeff: 1.0"),
    (34, "  segment_duration: 0.01"),
    (35, "  episode_duration: 5.0"),
    (38, "  random_init: false"),
    (39, "  episode_start_offset_s: 1.956870983805102"),
    (45, "  grf_penetration_penalty_threshold_m: 0.02"),
    (46, "  grf_penetration_termination_m: 0.028"),
)

# Sections whose every line must be byte-inherited. The substitution map may not
# name a line inside one of these, and the check enforces it.
UNTOUCHABLE_SECTIONS = ("grf", "supervision", "logging", "reward")

HEADER = (
    "# V26C J20 - critic-only warm-up config.",
    "#",
    "# DERIVED, NOT WRITTEN. Produced by v26c_j20_derive_warmup_config.py from",
    "#   " + SOURCE_REL,
    "#   sha256 " + SOURCE_SHA256,
    "# by a closed map of 11 substitutions. Every other byte is inherited, so",
    "# the physical runtime, FSM v3, the detector, the morphology corridor, the",
    "# reward and all current thresholds are unchanged by construction.",
    "#",
    "# training_exnovo_cfg.yaml is neither read nor modified by this stage.",
    "#",
    "# This file is a READINESS artefact. It authorises nothing on its own.",
)


class J20DerivationError(RuntimeError):
    """The derivation refused to proceed."""


def sha256_bytes(payload: bytes) -> str:
    """SHA-256 of a byte string."""
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: pathlib.Path) -> str:
    """SHA-256 of a file's bytes."""
    return sha256_bytes(path.read_bytes())


def source_path() -> pathlib.Path:
    """The pinned runtime config, verified before it is used."""
    path = REPO / SOURCE_REL
    if not path.is_file():
        raise J20DerivationError("the pinned runtime config is missing: %s" % path)
    actual = sha256_file(path)
    if actual != SOURCE_SHA256:
        raise J20DerivationError(
            "the pinned runtime config changed: expected %s, found %s. A derivation "
            "from altered bytes would prove nothing." % (SOURCE_SHA256, actual)
        )
    return path


def section_of_line(lines: list[str], index0: int) -> str:
    """The top-level YAML section a 0-based line index falls in."""
    for probe in range(index0, -1, -1):
        text = lines[probe]
        if text and not text.startswith((" ", "-", "#")) and text.endswith(":"):
            return text[:-1]
    return ""


def validate_map(lines: list[str]) -> dict:
    """Check the substitution map against the pinned source. Pure, writes nothing."""
    problems: list[str] = []
    claimed: list[int] = []
    for entry in SUBSTITUTIONS:
        numbers = entry["lines"]
        if len(numbers) != len(entry["old"]):
            problems.append("%s: %d line numbers for %d expected lines"
                            % (entry["id"], len(numbers), len(entry["old"])))
            continue
        for number, expected in zip(numbers, entry["old"]):
            if not 1 <= number <= len(lines):
                problems.append("%s: line %d is outside the source" % (entry["id"], number))
                continue
            actual = lines[number - 1]
            if actual != expected:
                problems.append("%s: line %d is %r, the map expects %r"
                                % (entry["id"], number, actual, expected))
            section = section_of_line(lines, number - 1)
            if section != entry["section"]:
                problems.append("%s: line %d is in section %r, the map says %r"
                                % (entry["id"], number, section, entry["section"]))
            if section in UNTOUCHABLE_SECTIONS:
                problems.append("%s: line %d is inside untouchable section %r"
                                % (entry["id"], number, section))
            claimed.append(number)
    duplicates = sorted({n for n in claimed if claimed.count(n) > 1})
    if duplicates:
        problems.append("the map claims these lines more than once: %s" % duplicates)

    for number, expected in INVARIANT_LINES:
        if not 1 <= number <= len(lines):
            problems.append("invariant line %d is outside the source" % number)
            continue
        if lines[number - 1] != expected:
            problems.append("invariant line %d is %r, expected %r"
                            % (number, lines[number - 1], expected))
        if number in claimed:
            problems.append("line %d is both an invariant and a substitution" % number)

    return {"ok": not problems, "problems": problems,
            "substituted_lines": sorted(claimed),
            "invariant_lines": [n for n, _ in INVARIANT_LINES]}


def derive() -> dict:
    """Produce the derived config bytes. Pure: reads the source, writes nothing."""
    path = source_path()
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    if len(lines) != SOURCE_LINES:
        raise J20DerivationError(
            "the pinned source has %d lines, expected %d" % (len(lines), SOURCE_LINES))

    validation = validate_map(lines)
    if not validation["ok"]:
        raise J20DerivationError(
            "the substitution map does not fit the pinned source: %s"
            % "; ".join(validation["problems"]))

    replacement: dict[int, tuple[str, ...]] = {}
    consumed: set[int] = set()
    for entry in SUBSTITUTIONS:
        first = entry["lines"][0]
        replacement[first] = tuple(entry["new"])
        consumed.update(entry["lines"][1:])

    out: list[str] = list(HEADER)
    inherited = 0
    for number, line in enumerate(lines, start=1):
        if number in consumed:
            continue
        if number in replacement:
            out.extend(replacement[number])
            continue
        out.append(line)
        inherited += 1

    payload = ("\n".join(out) + "\n").encode("utf-8")
    return {
        "bytes": payload,
        "sha256": sha256_bytes(payload),
        "source_sha256": SOURCE_SHA256,
        "source_lines": len(lines),
        "derived_lines": len(out),
        "header_lines": len(HEADER),
        "substitutions": len(SUBSTITUTIONS),
        "source_lines_substituted": len(validation["substituted_lines"]),
        "source_lines_inherited_verbatim": inherited,
        "invariants_asserted": len(INVARIANT_LINES),
        "validation": validation,
    }


def check(root: pathlib.Path = HERE) -> dict:
    """Compare the on-disk config against the derivation. Writes nothing."""
    result = derive()
    target = root / TARGET_NAME
    if not target.is_file():
        return dict(result, target_exists=False, matches=False,
                    target_sha256=None,
                    problems=["%s has not been emitted yet" % TARGET_NAME])
    actual = target.read_bytes()
    matches = actual == result["bytes"]
    return dict(result, target_exists=True, matches=matches,
                target_sha256=sha256_bytes(actual),
                problems=[] if matches else
                ["%s differs from the derivation" % TARGET_NAME])


def emit(root: pathlib.Path = HERE) -> dict:
    """Write the derived config. Refuses to clobber differing bytes."""
    report = check(root)
    target = root / TARGET_NAME
    if report["target_exists"] and not report["matches"]:
        raise J20DerivationError(
            "refusing to clobber %s: it exists and differs from the derivation. "
            "Delete it deliberately or fix the map." % target)
    if report["target_exists"]:
        return dict(report, written=False, reason="already byte-identical")
    target.write_bytes(report["bytes"])
    # check() reported "not emitted yet"; that finding is now spent. Carrying it
    # forward would make a successful emission exit non-zero.
    return dict(report, written=True, target_sha256=report["sha256"],
                matches=True, problems=[])


def build_parser() -> argparse.ArgumentParser:
    """Command-line surface. --check is the default and writes nothing."""
    parser = argparse.ArgumentParser(description="V26C J20 warm-up config derivation")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--emit", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Return 0 on success, 1 on failure."""
    args = build_parser().parse_args(argv)
    try:
        report = emit() if args.emit else check()
    except J20DerivationError as error:
        print(str(error), file=sys.stderr)
        return 1
    print("source            %s" % SOURCE_REL)
    print("source sha256     %s" % report["source_sha256"])
    print("source lines      %d" % report["source_lines"])
    print("substitutions     %d, covering %d source lines"
          % (report["substitutions"], report["source_lines_substituted"]))
    print("inherited verbatim %d source lines" % report["source_lines_inherited_verbatim"])
    print("invariants asserted %d" % report["invariants_asserted"])
    print("derived sha256    %s" % report["sha256"])
    print("target            %s (%s)"
          % (TARGET_NAME, "written" if report.get("written") else
             ("matches" if report.get("matches") else "MISSING/DIFFERS")))
    for problem in report.get("problems", []):
        print("  problem: %s" % problem, file=sys.stderr)
    return 0 if not report.get("problems") else 1


if __name__ == "__main__":
    sys.exit(main())
