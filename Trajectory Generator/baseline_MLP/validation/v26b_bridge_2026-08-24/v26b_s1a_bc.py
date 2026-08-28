"""V26B rev3q — S1A mono-role BC: materialisation of a NON-DEPLOYABLE intermediate candidate.

Token V26B-S1A-BC-FIT.  The computation is IDENTICAL to rev3o (init = pinned S0D, the 380
task rows of steps 1-190 and 311-500, same-time AB06 u_IK labels, frozen numerics) and is
performed by CALLING the unmodified rev3o fit function, so the reproduction is exact by
construction.  What changes is only the publication rule, per the architect's decision after
the documentary review:

  * Q1 on the rev3m source holdout is DIAGNOSTIC, never binding (July's first BC had no
    preservation DATA role; preserving raw S0D outputs against conflicting u_IK inside one
    optimisation is proven impossible - R0 feasibility diagnostic and rev3p).
  * A fail-closed P0 gate requires bit-exact reproduction of the five rev3o quantities,
    including the candidate actor digest, before anything is materialised.
  * The artefact is published OUTSIDE student/ and carries deployable:false,
    rollout_pending:true, sigma_unresolved:true and promotion_requires.

The deterministic nominal rollout is a SEPARATE token and is the entry gate to any later
July-style source==init preservation stage.  This module never executes a closed-loop episode.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1p_multirole as P  # noqa: E402   (rev3p lineage, unmodified)
import v26b_s1_fit as FIT  # noqa: E402        (rev3o computation, unmodified)
import v26b_r2g as G  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1AError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1A-BC-FIT"
AMENDMENT_REV3Q = HERE / "v26b_amendment_rev3q_s1a_bc_intermediate.json"
PIN_AMENDMENT_REV3Q = "74b23e6f5a202b260320718e43dd70f483dc898dc5805d91a28eeb52546bc667"

# P0: the five quantities recorded inside the immutable rev3o REJECTED artifact a559bd69...
P0_G_TASK = [0.13421327340983133, 0.12155383791334523]
P0_Q1_DIAGNOSTIC = [0.49825443853542484, 0.3461596068119536]
P0_T1 = 7.398672874359136e-08
P0_T2 = 9.561586078454809e-08
P0_ACTOR_DIGEST = "8f3e0ce17eff7c741dcf72de6d0fec0c372f9dbc7a9b3119d41c155ec8603e35"
PIN_REV3P_TOOL = "22f0e3554023ef03c8c82674d48200000508d9aa197e0d2b2176d02138c75f23"

G_TASK_MAX = FIT.G_TASK_MAX          # 0.15 by reference, never redefined
Q1_REFERENCE = FIT.Q1_MAX            # 0.10, kept ONLY as a reported reference (non-binding here)
OUT_S1A = VA.OUT_ROOT / "candidates" / "S1A_IK_AB06_35D_NONDEPLOYABLE"
RECEIPT_NAME = "v26b_s1a_bc_receipt.json"
MANDATORY_FLAGS = {"deployable": False, "rollout_pending": True, "sigma_unresolved": True,
                   "promotion_requires": "closed-loop nominal rollout under a separate token"}
CONTRACT_STRING = "intermediate_candidate_state_NON_DEPLOYABLE"


# --- guards --------------------------------------------------------------------------------------

def assert_not_under_student(path: Path) -> None:
    if "student" in Path(path).resolve().parts:
        raise S1AError(f"rev3q forbids publishing under student/: {path}")


def assert_no_deployable_marking(obj: Any, where: str = "root") -> None:
    """No deployable marking or alias: every occurrence of the word must be a NON_DEPLOYABLE one,
    and a key literally named 'deployable' must carry False."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            if str(k).lower() == "deployable" and v is not False:
                raise S1AError(f"{where}: key 'deployable' must be False, got {v!r}")
            assert_no_deployable_marking(v, f"{where}.{k}")
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_no_deployable_marking(v, f"{where}[{i}]")
    elif isinstance(obj, str):
        norm = obj.lower().replace("-", "_")
        if "deployable" in norm.replace("non_deployable", "").replace("nondeployable", ""):
            raise S1AError(f"{where}: forbidden deployable marking/alias in {obj!r}")


def verify_lineage_s1a() -> dict[str, Any]:
    lin = P.verify_lineage_s1p()      # rev3l/m/n/o/p + REJECTED artifacts + S0D chain + additivity
    got = C.sha256_file(AMENDMENT_REV3Q)
    if got != PIN_AMENDMENT_REV3Q:
        raise S1AError(f"rev3q sha {got} != pinned")
    lin["amendment_rev3q"] = got
    got = C.sha256_file(HERE / "v26b_s1p_multirole.py")
    if got != PIN_REV3P_TOOL:
        raise S1AError(f"rev3p tool sha {got} != pinned (rev3q must stay additive)")
    lin["rev3p_tool"] = got
    if P.OUT_S1P.exists():
        raise S1AError(f"rev3p published a checkpoint at {P.OUT_S1P}: contradicts the REJECTED record")
    lin["rev3p_no_checkpoint_published"] = True
    assert_not_under_student(OUT_S1A)
    lin["output_outside_student"] = True
    return lin


# --- P0 -------------------------------------------------------------------------------------------

def p0_check(gates: Mapping[str, Any]) -> dict[str, Any]:
    """Fail-closed bit-exact reproduction of the five rev3o quantities."""
    got = {"G_task": list(gates["G_task"]["per_joint"]),
           "Q1_diagnostic": list(gates["Q1_source_holdout_rev3m"]["per_joint"]),
           "T1_maxabs": float(gates["function_preservation"]["T1_maxabs"]),
           "T2_maxabs": float(gates["function_preservation"]["T2_maxabs"]),
           "candidate_actor_digest": str(gates["actor_digest_new"])}
    want = {"G_task": P0_G_TASK, "Q1_diagnostic": P0_Q1_DIAGNOSTIC, "T1_maxabs": P0_T1,
            "T2_maxabs": P0_T2, "candidate_actor_digest": P0_ACTOR_DIGEST}
    mismatch = {k: {"got": got[k], "expected": want[k]} for k in want if got[k] != want[k]}
    return {"expected": want, "observed": got, "mismatch": mismatch, "pass": bool(not mismatch),
            "rule": "BIT-EXACT equality on all five items (source: immutable rev3o REJECTED artifact a559bd69...)",
            "binding": True}


def materialisation_decision(gates: Mapping[str, Any], p0: Mapping[str, Any]) -> dict[str, Any]:
    """Binding: G_task, Q3, T1/T2, P0.  Q1 is DIAGNOSTIC and can never block."""
    binding = {"G_task": bool(gates["G_task"]["pass"]), "Q3_invariants": bool(gates["Q3_invariants"]["pass"]),
               "function_preservation": bool(gates["function_preservation"]["pass"]), "P0": bool(p0["pass"])}
    failed = [k for k, v in binding.items() if not v]
    return {"binding_gates": binding, "failed_binding_gates": failed,
            "Q1_is_binding": False,
            "Q1_observed": list(gates["Q1_source_holdout_rev3m"]["per_joint"]),
            "Q1_reference_threshold_non_binding": Q1_REFERENCE,
            "materialise": bool(not failed)}


def rev3q_gate_view(gates: Mapping[str, Any]) -> dict[str, Any]:
    """A rev3q view of the rev3o gate block: Q1 re-labelled as diagnostic, nothing recomputed."""
    out = json.loads(json.dumps(gates, default=str))
    q1 = out["Q1_source_holdout_rev3m"]
    q1["binding"] = False
    q1["role"] = ("DIAGNOSTIC only under rev3q: July's first BC had no preservation DATA role; "
                  "preserving raw S0D outputs against conflicting u_IK in one optimisation is proven impossible")
    out.pop("pass_all", None)
    return out


# --- run --------------------------------------------------------------------------------------------

def run_s1a(*, authorized_stage: str | None, out_dir: Path = OUT_S1A, reject_dir: Path = VA.OUT_ROOT,
            progress: bool = True, runtime_status: dict[str, Any] | None = None,
            _fit_fn=None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1AError(f"the S1A materialisation requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    out_dir = Path(out_dir)
    assert_not_under_student(out_dir)
    lineage = verify_lineage_s1a()
    vec, scale_table = G.scale_vector()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(FIT.S0D_MODULE).items()}
    struct0 = RF.validate_init_state(init_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR)
    if struct0["actor_digest"] != FIT.PIN_S0D_ACTOR:
        raise S1AError("init is not the pinned S0D actor")
    view = FIT.build_s1_task()
    src = FIT.build_source_holdout(view, init_raw)
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, view["obs"], vec)
    if t1 > G.PRESERVATION_TOL:
        raise S1AError(f"prefit T1 FAILED: {t1:.3e}")
    scaled_fit, fit_report = (_fit_fn or FIT.fit_s1)(init_scaled, view, vec, progress=progress)
    export_raw = G.export_state_from_scaled(scaled_fit, vec)
    gates = FIT.evaluate(init_raw, export_raw, scaled_fit, view, src, vec, t1)
    p0 = p0_check(gates)
    view_gates = rev3q_gate_view(gates)
    decision = materialisation_decision(gates, p0)
    if not decision["materialise"]:
        rej = R.unique_artifact_path(Path(reject_dir), f"v26b_s1a_bc_REJECTED_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_s1a_bc_rejected.1", "REJECTED": True,
                                                  "amendment_rev3q": PIN_AMENDMENT_REV3Q,
                                                  "authorized_stage": AUTHORIZED_STAGE, "lineage": lineage,
                                                  "split": view["records"], "source_holdout_rev3m": src["records"],
                                                  "P0": p0, "gates": view_gates, "decision": decision,
                                                  "loss_history_full": fit_report.get("history"),
                                                  "policy": "no candidate materialised; thresholds/metrics/labels unchanged; prior artifacts untouched; STOP",
                                                  **MANDATORY_FLAGS,
                                                  "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}, indent=2, default=str) + "\n")
        raise S1AError(f"materialisation gates FAILED {decision['failed_binding_gates']}: nothing published; diagnosis at {C.rel(rej)}")
    s0d_pre = VS.source_files_table(FIT.S0D_MODULE)
    if out_dir.exists():
        raise FileExistsError(f"no-clobber: {out_dir} exists")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    lock, token = RF.acquire_export_lock(out_dir)
    staging = None; promoted = False
    try:
        if out_dir.exists():
            raise FileExistsError("final path exists (under lock)")
        staging = RF._staging_dir_for(out_dir)
        stage_module = staging / "rl_module"; stage_module.mkdir(parents=True, exist_ok=False)
        shutil.copy2(FIT.S0D_MODULE / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(FIT.S0D_MODULE / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded)
        if not cmp.get("exact"):
            raise S1AError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=P0_ACTOR_DIGEST)
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"),
                    "manifest35_sha256": manifest_shas["manifest35_sha256"],
                    "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE,
                    "derived_from": C.rel(FIT.S0D_MODULE), "source_actor_digest": FIT.PIN_S0D_ACTOR,
                    "contract": CONTRACT_STRING, **MANDATORY_FLAGS,
                    "status": "INTERMEDIATE CANDIDATE (rev3q): mono-role BC towards AB06 u_IK; no closed-loop evidence exists for it",
                    "only_actor_with_closed_loop_evidence": "S0D 481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a (500/500 nominal under v3)"}
        assert_no_deployable_marking(manifest, "manifest")
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(FIT.S0D_MODULE) != s0d_pre:
            raise S1AError("SOURCE S0D changed during export")
        hist = fit_report["history"]
        receipt = {"schema": "v26b_s1a_bc.1", "amendment_rev3q": PIN_AMENDMENT_REV3Q, "authorized_stage": AUTHORIZED_STAGE,
                   **MANDATORY_FLAGS,
                   "lineage": lineage,
                   "init": {"module": C.rel(FIT.S0D_MODULE), "actor_digest": FIT.PIN_S0D_ACTOR,
                            "module_state_sha256": FIT.PIN_S0D_MODULE_STATE, "files_sha256": s0d_pre,
                            "enforcement": "positive digest pin; JUL_H0/R0a/R1/R2G/R2I forbidden as init and labels"},
                   "computation": {"identical_to_rev3o": True, "fit_function": "v26b_s1_fit.fit_s1 (unmodified rev3o tool a6d7163e...)",
                                   "anchor_semantics": "July-11-faithful PARAMETRIC regulariser 1e-5 towards theta_S0D; NOT a preservation mechanism"},
                   "split": view["records"], "source_holdout_rev3m": src["records"],
                   "scaling": {"table": scale_table, "T1_maxabs": t1, "T2_maxabs": gates["function_preservation"]["T2_maxabs"], "tol": G.PRESERVATION_TOL},
                   "fit": {k: fit_report[k] for k in ("tool", "rows_train", "budget", "anchor", "loss", "sigma_note")},
                   "loss_curve_essential": {"first": hist[0], "epoch_50": hist[49], "epoch_150": hist[149], "epoch_300": hist[-1]},
                   "loss_history_full": hist,
                   "P0_reproducibility": p0, "gates": view_gates, "decision": decision,
                   "structure": struct, "save_reload_exact": True,
                   "sigma": {"placeholder_value": VS.SIGMA_PLACEHOLDER, "statement": G.SIGMA_NOTE,
                             "question": "UNRESOLVED: July selected sigma per stage (0.05 -> 0.03 -> 0.003 -> 0.005); the 11/07 BC distilled the source log-std. Do not assume 0.005"},
                   "honest_extrapolation": ("requested behaviour change ~0.38 (rev3o knee drift) vs the closest July analogue (13/07) "
                                            "nominal shift RMS 0.004175; every chain actor trained towards u_IK failed closed loop (493/242/197). "
                                            "The offline gates are NOT evidence of closed-loop viability"),
                   "next_stage_locked": {"token": "V26B-S1A-NOMINAL-ROLLOUT (not granted)",
                                         "role": "entry gate to any July-style source==init preservation stage; the 13/07 anchor construction needs this actor's own COMPLETE nominal trace"},
                   "output_module": C.rel(out_dir / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
                   "code_digests": {"v26b_s1a_bc.py": C.sha256_file(Path(__file__).resolve()),
                                    "test_v26b_s1a_bc.py": C.sha256_file(HERE / "test_v26b_s1a_bc.py") if (HERE / "test_v26b_s1a_bc.py").is_file() else None,
                                    "v26b_s1_fit.py": C.sha256_file(HERE / "v26b_s1_fit.py"),
                                    "v26b_s1p_multirole.py": C.sha256_file(HERE / "v26b_s1p_multirole.py"),
                                    "v26b_s1_pregate_rev3n.py": C.sha256_file(HERE / "v26b_s1_pregate_rev3n.py"),
                                    "v26b_s1_prereg.py": C.sha256_file(HERE / "v26b_s1_prereg.py")},
                   "scope": "S1A offline materialisation ONLY: no closed-loop evaluation, no DAgger/PPO/critic/multistart/sigma sweep/morphology fit",
                   "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename, else F2R fallback", "completion_marker": RECEIPT_NAME},
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        assert_no_deployable_marking(receipt, "receipt")
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise S1AError("canonical receipt differs from memory")
        canonical_sha = C.sha256_file(staging / RECEIPT_NAME)
        if out_dir.exists():
            raise FileExistsError("final path appeared during export")
        promotion = RF.promote_staging(staging, out_dir)
        promoted = True
    except BaseException:
        if staging is not None and not promoted:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        released = RF.release_export_lock(lock, token)
    if runtime_status is not None:
        runtime_status.update({"promotion": promotion, "lock_released": bool(released),
                               "canonical_receipt_sha256": canonical_sha, "final_path": C.rel(out_dir)})
    return canonical


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3q S1A mono-role BC materialisation (token-gated; offline only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_s1a()
        print(json.dumps({"mode": "dry", "lineage_ok": True}, indent=2))
        return 0
    runtime = {}
    canonical = run_s1a(authorized_stage=args.authorized_stage, runtime_status=runtime)
    g = canonical["gates"]
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"),
                      "P0_pass": canonical["P0_reproducibility"]["pass"], "materialised": canonical["decision"]["materialise"],
                      "actor_digest": g["actor_digest_new"], "G_task": g["G_task"]["per_joint"],
                      "Q1_diagnostic": g["Q1_source_holdout_rev3m"]["per_joint"],
                      "deployable": canonical["deployable"], "rollout_pending": canonical["rollout_pending"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
