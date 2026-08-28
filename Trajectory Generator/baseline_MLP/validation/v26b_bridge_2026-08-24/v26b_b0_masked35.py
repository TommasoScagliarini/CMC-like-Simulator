"""V26B stage B0 - single 35D student, controller/Markov channel HARD-MASKED. TOOLING ONLY.

The final student is 35D from creation.  There is ONE contract, ONE manifest and ONE feature
list: the v3 canonical 35D contract, unmodified.  The ten controller/Markov features stay in
the vector and in the manifest, but during the BASE phase they are neutralised twice over:

  1. INPUT MASK      every observation column in CONTROLLER_COLUMNS is overwritten with
                     MASK_VALUE before the network sees it;
  2. ZERO COLUMNS    the corresponding first-layer columns are exactly zero, frozen, and
                     restored to zero after every optimizer step.

Either guarantee alone already removes the channel; together they make a leak detectable.
With masked inputs the gradient on those columns is identically zero, so the restore is a
second, independent guard rather than the primary mechanism - and the test proves both.

Because the columns are already zero, REMOVING THE MASK IS BIT-EXACT: the unmasked network
computes the same function until the Markov phase deliberately adapts those columns.

Parent: EXCLUSIVELY the August V26 imitation actor 39D, digest 5bbc6cbd...
S0D, S1A and every REV4*/V2_* artifact are forbidden as parent or data.  No July artifact.
The superseded 25D branch is never imported.

No fit, no training, no rollout, no collection.  Cross-platform: pathlib only, no shell.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_student as VS  # noqa: E402   (frozen V1 recipe: pinned names and the 39->35 transplant)
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402
import f1_dataset as DS  # noqa: E402


class B0Error(RuntimeError):
    pass


STAGE = "V26B-B0-MASKED35-TRANSPLANT"

PIN_V26_ACTOR_DIGEST = "5bbc6cbd3c7e3ec37524b7b6b69ca017af48057cac5207cf755d3b2f72c2709e"
PIN_V1_RECIPE_DIGEST = "ae846220a6f7f1ac1289ccc9636e3ad2e5bc7842ba7ece0b62bb9d7590e7f587"
PIN_RUNTIME_CONFIG = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"
PIN_MANIFEST35 = "c6f8602816059b5c3508c626e5ce08d3ba922710f8b48c9557af9056f1155004"

CONTROLLER_SUFFIXES = ("_previous_endpoint", "_served_ref", "_served_ref_vel",
                       "_served_ref_accel", "_sea_u")
EXPECTED_CONTROLLER_COLUMNS = (25, 26, 27, 28, 29, 30, 31, 32, 33, 34)
CLOCK_COLUMNS = tuple(R.CLOCK_COLUMNS)
MASK_VALUE = np.float32(0.0)
ACTOR_WIDTH = R.ENV_ACTOR_WIDTH          # 35, unchanged
SIGMA_PLACEHOLDER = VS.SIGMA_PLACEHOLDER
SIGMA_STATEMENT = ("sigma is a SERIALISATION PLACEHOLDER and is NOT a decision: the exploration "
                   "scale must be MEASURED on the base actor before any stochastic recovery")

FORBIDDEN_SOURCE_MARKERS = ("S0D_35D_DISTILLED", "S1A_", "S1B_", "S1C1_", "S1C2Z_",
                            "REV4B", "REV4C", "REV4D", "REV4E", "V2_", "2026-07",
                            "target_domain_dagger_", "target_domain_imitation_2026")
SUPERSEDED_MODULES = ("v26b_a0_transplant25", "v26b_a1_ik_imitation")

OUT_DIR = VA.OUT_ROOT / "student" / "B0_35D_MASKED"
RECEIPT_NAME = "v26b_b0_receipt.json"


def assert_not_forbidden(path: Path | str, where: str) -> None:
    s = str(path)
    for mark in FORBIDDEN_SOURCE_MARKERS:
        if mark in s:
            raise B0Error(f"{where}: {s} uses a forbidden source ({mark!r}); this branch takes "
                          "only the August V26 parent")


def controller_columns() -> tuple[int, ...]:
    """Derived from the pinned 35D manifest BY NAME, then checked against the expected tuple."""
    names35, _, shas = VS.pinned_names()
    if len(names35) != ACTOR_WIDTH:
        raise B0Error(f"pinned manifest is {len(names35)} wide, expected {ACTOR_WIDTH}")
    if shas["manifest35_sha256"] != PIN_MANIFEST35:
        raise B0Error("35D manifest sha does not match the pin")
    cols = tuple(i for i, n in enumerate(names35)
                 if any(n.endswith(s) for s in CONTROLLER_SUFFIXES))
    if cols != EXPECTED_CONTROLLER_COLUMNS:
        raise B0Error(f"controller columns {cols} != expected {EXPECTED_CONTROLLER_COLUMNS}")
    if set(cols) & set(CLOCK_COLUMNS):
        raise B0Error("controller and clock column sets overlap")
    return cols


def kept_columns() -> tuple[int, ...]:
    ctrl = set(controller_columns())
    return tuple(i for i in range(ACTOR_WIDTH) if i not in ctrl)


# ---------------------------------------------------------------- mask primitives ---------------

def apply_input_mask(obs: np.ndarray) -> np.ndarray:
    """Overwrite the controller columns with MASK_VALUE. Returns a copy; never in place."""
    a = np.array(obs, dtype=np.float32, copy=True)
    if a.ndim != 2 or a.shape[1] != ACTOR_WIDTH:
        raise B0Error(f"observations must be (N,{ACTOR_WIDTH}), got {a.shape}")
    a[:, list(controller_columns())] = MASK_VALUE
    return a


def assert_masked_columns_zero(state: Mapping[str, Any], where: str = "state") -> None:
    """Fail closed: every first-layer column of the masked channel must be EXACTLY zero."""
    cols = list(controller_columns())
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        W = np.asarray(state[key])
        if W.shape[1] != ACTOR_WIDTH:
            raise B0Error(f"{where}.{key} is {W.shape[1]} wide, expected {ACTOR_WIDTH}")
        bad = np.asarray(W[:, cols])
        if not np.all(bad == 0.0):
            raise B0Error(f"{where}.{key}: masked columns are not exactly zero "
                          f"(max abs {float(np.abs(bad).max())!r})")


def assert_clock_columns_zero(state: Mapping[str, Any], where: str = "state") -> None:
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        if not np.all(np.asarray(state[key])[:, list(CLOCK_COLUMNS)] == 0.0):
            raise B0Error(f"{where}.{key}: clock columns are not zero")


def restore_masked_columns(state: dict[str, Any]) -> dict[str, Any]:
    """Re-zero the masked (and clock) first-layer columns. Called after EVERY optimizer step."""
    cols = list(controller_columns()) + list(CLOCK_COLUMNS)
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        W = np.array(state[key], copy=True)
        W[:, cols] = np.float32(0.0)
        state[key] = W
    return state


def assert_no_masked_update(before: Mapping[str, Any], after: Mapping[str, Any]) -> dict[str, Any]:
    """Fail closed: the masked columns must be bit-identical before and after, and still zero."""
    cols = list(controller_columns())
    out: dict[str, Any] = {}
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        b = np.asarray(before[key])[:, cols]
        a = np.asarray(after[key])[:, cols]
        if not np.array_equal(a, b):
            raise B0Error(f"{key}: the masked columns changed across the step")
        if not np.all(a == 0.0):
            raise B0Error(f"{key}: the masked columns are not zero after the step")
        out[key] = {"bit_identical": True, "all_zero": True, "columns": cols}
    return out


# ---------------------------------------------------------------- equivalence proofs -------------

def _subnet_25(state: Mapping[str, Any]) -> dict[str, np.ndarray]:
    """The same actor with the masked columns physically dropped: a genuine 25-column network."""
    keep = list(kept_columns())
    out = {k: np.array(v, copy=True) for k, v in state.items()}
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        out[key] = np.ascontiguousarray(np.asarray(state[key])[:, keep])
    return out


def functional_equivalence_25(state: Mapping[str, Any], obs: np.ndarray) -> dict[str, Any]:
    """BIT-EXACT: masked-35D output == the 25-column subnetwork on the kept columns."""
    o35 = apply_input_mask(obs)
    o25 = np.asarray(obs, dtype=np.float32)[:, list(kept_columns())]
    m35 = RF.numpy_mean(dict(state), o35)
    m25 = RF.numpy_mean(_subnet_25(state), o25)
    equal = bool(np.array_equal(m35, m25))
    if not equal:
        raise B0Error("the masked 35D actor is NOT bit-identical to its 25-column subnetwork "
                      f"(max abs difference {float(np.max(np.abs(m35 - m25)))!r})")
    return {"rows": int(np.asarray(obs).shape[0]), "bit_identical": True,
            "max_abs_difference": 0.0,
            "meaning": "the base phase provably operates on the 25-feature subspace; the ten "
                       "controller columns contribute nothing to any output"}


def bit_exact_unmask_transition(state: Mapping[str, Any], obs: np.ndarray) -> dict[str, Any]:
    """BIT-EXACT: with the columns at zero, feeding the RAW observation gives the same output as
    feeding the masked one. Removing the mask is therefore a no-op until those columns change."""
    assert_masked_columns_zero(state, "unmask transition")
    raw = np.asarray(obs, dtype=np.float32)
    masked = apply_input_mask(obs)
    if np.array_equal(raw, masked):
        raise B0Error("the probe observations do not differ on the masked columns, so the "
                      "transition test would be vacuous")
    m_masked = RF.numpy_mean(dict(state), masked)
    m_raw = RF.numpy_mean(dict(state), raw)
    if not np.array_equal(m_masked, m_raw):
        raise B0Error("removing the input mask changed the output while the columns were zero "
                      f"(max abs difference {float(np.max(np.abs(m_masked - m_raw)))!r})")
    return {"rows": int(raw.shape[0]), "bit_identical": True, "max_abs_difference": 0.0,
            "probe_columns_differ": True,
            "meaning": "the base -> Markov transition needs no contract switch and no re-export: "
                       "it is the identity until the Markov phase adapts those columns"}


# ---------------------------------------------------------------- transplant ---------------------

def load_v26_source() -> dict[str, np.ndarray]:
    src_module = Path(VS.SOURCE_MODULE) if hasattr(VS, "SOURCE_MODULE") else None
    if src_module is None:
        src_module = (R.BASELINE_DIR.parent / "runs" / "training"
                      / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter" / "rl_module_best")
    assert_not_forbidden(src_module, "B0 parent")
    with open(Path(src_module) / "module_state.pkl", "rb") as fh:
        st = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    d = RF.actor_state_digest(st)
    if d != PIN_V26_ACTOR_DIGEST:
        raise B0Error(f"V26 parent digest {d} != pinned")
    return st


def build_b0_state() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """V1 recipe from V26 (39->35, healthy targets mean-compensated, clock zeroed) and then the
    ten controller columns zeroed.  Those ten are NOT mean-compensated on purpose: they return in
    the Markov phase and must start from a clean zero, exactly as the July expansion did."""
    state, reports = VS.build_v1_state()
    if RF.actor_state_digest(state) != PIN_V1_RECIPE_DIGEST:
        raise B0Error("the 39->35 recipe did not reproduce its pinned digest")
    before = {k: np.array(v, copy=True) for k, v in state.items()}
    masked = {k: np.array(v, copy=True) for k, v in state.items()}
    cols = list(controller_columns())
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        W = np.array(masked[key], copy=True)
        W[:, cols] = np.float32(0.0)
        masked[key] = np.ascontiguousarray(W)
    assert_masked_columns_zero(masked, "B0")
    assert_clock_columns_zero(masked, "B0")
    keep = list(kept_columns())
    for key in ("pi.0.0.weight", "pi_encoder.0.weight"):
        if not np.array_equal(np.asarray(masked[key])[:, keep], np.asarray(before[key])[:, keep]):
            raise B0Error(f"{key}: a kept column changed while masking")
    for key in ("pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias",
                "pi_encoder.0.bias", "pi_encoder.2.weight", "pi_encoder.2.bias"):
        if not np.array_equal(np.asarray(masked[key]), np.asarray(before[key])):
            raise B0Error(f"{key} changed while masking; only first-layer columns may change")
    RF.validate_init_state(masked, expected_actor_digest=None, width=ACTOR_WIDTH)
    report = {
        "recipe": "V26B / B0: the frozen V1 39->35 transplant, then the ten controller columns zeroed",
        "parent_actor_digest": PIN_V26_ACTOR_DIGEST,
        "v1_recipe_digest_reproduced": PIN_V1_RECIPE_DIGEST,
        "masked_columns": cols,
        "clock_columns": list(CLOCK_COLUMNS),
        "mean_compensation": {
            "healthy_targets": "COMPENSATED, exactly as the frozen V1 recipe: those four columns "
                               "leave the vector for good, so their mean is folded into the bias",
            "controller_columns": "NOT COMPENSATED, deliberately: these columns stay in the vector "
                                  "and come back in the Markov phase. Folding their mean into the "
                                  "bias would pre-load a constant that the adaptation would then "
                                  "have to unlearn. July expanded from a clean zero for the same reason",
        },
        "b0_actor_digest": RF.actor_state_digest(masked),
        "logstd_placeholder": {"statement": SIGMA_STATEMENT},
    }
    return masked, {"transplant": report, "v1_reports": reports}


# ---------------------------------------------------------------- preflight ----------------------

def probe_observations(rows: int = 256) -> np.ndarray:
    """Real observations from the pinned V26 anchor traces; never S0D/S1A/REV."""
    out: list[np.ndarray] = []
    for start, spec in R.ANCHORS.items():
        job = Path(spec["job_dir"])
        assert_not_forbidden(job, "probe anchor")
        o = np.asarray(DS.trajectory_from_job(job, expected_width=ACTOR_WIDTH)["obs35"],
                       dtype=np.float32)
        out.append(o)
    obs = np.concatenate(out, axis=0)
    if obs.shape[0] < rows:
        raise B0Error("not enough anchor rows for the probe")
    step = max(1, obs.shape[0] // rows)
    return np.ascontiguousarray(obs[::step][:rows])


def preflight() -> dict[str, Any]:
    """No-write, fail-closed."""
    if any(m in sys.modules for m in SUPERSEDED_MODULES):
        raise B0Error("a superseded 25D module is imported; this branch must not touch it")
    cols = controller_columns()
    state, reports = build_b0_state()
    obs = probe_observations()
    equiv = functional_equivalence_25(state, obs)
    unmask = bit_exact_unmask_transition(state, obs)
    names35, _, shas = VS.pinned_names()
    if OUT_DIR.exists():
        raise B0Error(f"no-clobber: {OUT_DIR} already exists")
    return {"verdict": "GO", "stage": STAGE,
            "contract": {"actor_width": ACTOR_WIDTH, "manifest35_sha256": shas["manifest35_sha256"],
                         "runtime_config_sha256": PIN_RUNTIME_CONFIG,
                         "single_contract": "the v3 canonical 35D contract, unmodified; no second "
                                            "observation contract and no second runtime pin exists",
                         "masked_columns": list(cols),
                         "masked_feature_names": [names35[i] for i in cols],
                         "mask_value": float(MASK_VALUE),
                         "clock_columns": list(CLOCK_COLUMNS),
                         "contralateral_features_in_actor": 0},
            "transplant": reports["transplant"],
            "functional_equivalence_25": equiv,
            "bit_exact_unmask_transition": unmask,
            "probe": {"rows": int(obs.shape[0]),
                      "source": "the three pinned V26 anchor traces (nominal, -0.20 s, +0.20 s)"},
            "sigma": SIGMA_STATEMENT}


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B0: masked 35D transplant (preflight only)")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    a = p.parse_args(argv)
    if a.authorized_stage is not None:
        raise B0Error("B0 is TOOLING ONLY in this stage: no materialisation is authorised")
    print(json.dumps(preflight(), indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
