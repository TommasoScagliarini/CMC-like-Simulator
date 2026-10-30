"""V26B rev3v — S1C: sign/target audit, causal diagnosis and quality-gate definitions (validation only).

This module RE-DERIVES from the frozen artifacts every number the rev3v amendment freezes, so the
audit is reproducible rather than asserted, and it defines the preregistered S1C quality gates as
pure functions.  It NEVER fits, NEVER launches an episode and NEVER collects data.

Key conventions, fixed here once and for all:
  * the coordinate under test is the PROSTHETIC ankle (obs35[4]); NEGATIVE = plantarflexion.
  * the optimisation/quality reference is the PROSTHETIC IK target, decoded q = 0.7*a (ankle) and
    q = 0.75*a - 0.75 (knee).
  * the healthy contralateral signal (cache targets) is a SYMMETRY diagnostic only and may never be
    used for sign agreement, because it is sign-degenerate.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1b_rollout as SB  # noqa: E402      (rev3u lineage + A2 job, unmodified)
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402


class S1CError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1C-PROTOCOL"
AMENDMENT_REV3V = HERE / "v26b_amendment_rev3v_s1c_protocol.json"
PIN_AMENDMENT_REV3V = "889f1068a9ddb8db8241b62b569a85fbc6baf3d7673ae626c4df5e51902ed984"
FUTURE_PROBE_TOKEN = "V26B-S1C-FEASIBILITY-PROBE"
FUTURE_FIT_TOKEN = "V26B-S1C-FIT"
FUTURE_ROLLOUT_TOKEN = "V26B-S1C-NOMINAL-ROLLOUT"

# decode of the normalised action into the prosthetic joint coordinate (PROTOCOL_F2R sec.3)
KNEE_DECODE = (0.75, -0.75)
ANKLE_DECODE = (0.70, 0.0)
NAMES = R.FEATURE_NAMES_35
IDX = {n: i for i, n in enumerate(NAMES)}
B3_THRESHOLD = -0.03
OUT_DIR = VA.OUT_ROOT / "s1c_protocol"


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV3V)
    if got != PIN_AMENDMENT_REV3V:
        raise S1CError(f"rev3v sha {got} != pinned")
    return json.loads(AMENDMENT_REV3V.read_text(encoding="utf-8"))


def decode_action(a: np.ndarray) -> np.ndarray:
    """Normalised action -> prosthetic joint target [knee, ankle], PROTOCOL_F2R sec.3."""
    a = np.asarray(a, dtype=np.float64)
    return np.stack([KNEE_DECODE[0] * a[:, 0] + KNEE_DECODE[1], ANKLE_DECODE[0] * a[:, 1] + ANKLE_DECODE[1]], axis=1)


def verify_lineage_s1c() -> dict[str, Any]:
    lin = SB.verify_lineage_rollout()
    lin["amendment_rev3v"] = C.sha256_file(AMENDMENT_REV3V)
    if lin["amendment_rev3v"] != PIN_AMENDMENT_REV3V:
        raise S1CError("rev3v sha != pinned")
    rec = json.loads((SB.JOB_DIR / SB.RECEIPT_NAME).read_text(encoding="utf-8"))
    if rec["status"] != SB.PASS_STATUS:
        raise S1CError("the A2 rollout receipt no longer records the eligibility status")
    lin["a2_rollout_receipt"] = C.sha256_file(SB.JOB_DIR / SB.RECEIPT_NAME)
    lin["a2_promotion_refused_by_architect"] = "qualitative promotion REFUSED; A2 stays NON-DEPLOYABLE"
    return lin


# --- audit -------------------------------------------------------------------------------------------

def sign_degeneracy(x: np.ndarray) -> dict[str, Any]:
    """A reference is sign-degenerate when it never changes sign: sign agreement against it is void."""
    x = np.asarray(x, dtype=np.float64)
    neg = float(np.mean(x < 0.0))
    return {"frac_negative": neg, "degenerate": bool(neg == 0.0 or neg == 1.0),
            "min": float(x.min()), "max": float(x.max())}


def sign_agreement(x: np.ndarray, ref: np.ndarray) -> dict[str, Any]:
    """Sign agreement, VOID when the reference is sign-degenerate (the rev3u receipt bug)."""
    deg = sign_degeneracy(ref)
    if deg["degenerate"]:
        return {"value": None, "verdict": "VOID_degenerate_reference", "reference_sign_degeneracy": deg}
    return {"value": float(np.mean(np.sign(np.asarray(x)) == np.sign(np.asarray(ref)))),
            "verdict": "valid", "reference_sign_degeneracy": deg}


def audit_targets() -> dict[str, Any]:
    """Re-derive the two references and their distance from the frozen cache."""
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    healthy = np.asarray(cc.targets)[:, [0, 2]]
    q_ik = decode_action(np.asarray(cc.ik_action))
    rmse = [float(np.sqrt(np.mean((q_ik[:, j] - healthy[:, j]) ** 2))) for j in (0, 1)]
    return {"cache_digest": cc.digest(),
            "prosthetic_IK_target_decoded": {"knee": sign_degeneracy(q_ik[:, 0]), "ankle": sign_degeneracy(q_ik[:, 1]),
                                             "decode": {"knee": list(KNEE_DECODE), "ankle": list(ANKLE_DECODE)}},
            "healthy_contralateral_reference": {"knee": sign_degeneracy(healthy[:, 0]), "ankle": sign_degeneracy(healthy[:, 1])},
            "rmse_between_the_two_references": {"knee": rmse[0], "ankle": rmse[1]},
            "rule": "the prosthetic IK target is the optimisation and quality reference; the healthy signal is a symmetry diagnostic only"}


def trace_view(job_dir: Path) -> dict[str, Any]:
    tr = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.asarray(tr["obs35"], dtype=np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(np.asarray(tr["t_pre"], dtype=np.float64))
    return {"obs": obs, "act": np.asarray(tr["b_raw_action"], dtype=np.float64),
            "q_ik": decode_action(np.asarray(cc.ik_action))[idx], "healthy": np.asarray(cc.targets)[idx][:, [0, 2]],
            "trace_sha256": tr["trace_sha256"]}


def command_chain_audit(job_dir: Path) -> dict[str, Any]:
    """Command -> served reference -> realised joint, for the ankle: where the plantarflexion is lost."""
    v = trace_view(job_dir)
    obs, act = v["obs"], v["act"]
    cmd = decode_action(act)[:, 1]
    ref = obs[:, IDX["pros_ankle_angle_served_ref"]]
    real = obs[:, IDX["pros_ankle_angle"]]
    seau = obs[:, IDX["pros_ankle_angle_sea_u"]]
    tgt_neg = v["q_ik"][:, 1] < 0.0
    deep = cmd < -0.05
    out = {"trace_sha256": v["trace_sha256"], "rows": int(obs.shape[0]),
           "commanded_q_ankle": sign_degeneracy(cmd), "served_reference": sign_degeneracy(ref),
           "realised_q_ankle": sign_degeneracy(real),
           "sea_u_ankle": {"min": float(seau.min()), "max": float(seau.max()),
                           "frac_abs_gt_0p99": float(np.mean(np.abs(seau) > 0.99))},
           "reference_tracking_error": {"mean_abs": float(np.abs(real - ref).mean()), "max_abs": float(np.abs(real - ref).max())},
           "rows_with_negative_IK_target": int(tgt_neg.sum()),
           "on_those_rows": {"commanded_mean": float(cmd[tgt_neg].mean()), "served_ref_mean": float(ref[tgt_neg].mean()),
                             "realised_mean": float(real[tgt_neg].mean()), "realised_min": float(real[tgt_neg].min())} if tgt_neg.any() else None,
           "rows_commanding_below_minus_0p05": int(deep.sum()),
           "on_deep_command_rows": {"served_ref_mean": float(ref[deep].mean()), "realised_mean": float(real[deep].mean()),
                                    "realised_min": float(real[deep].min())} if deep.any() else None}
    out["verdict"] = ("the negative command never reaches the joint: the served reference is sign-degenerate positive "
                      "while the policy commands plantarflexion") if (out["served_reference"]["frac_negative"] == 0.0
                                                                      and out["commanded_q_ankle"]["frac_negative"] > 0.0) else "no command/realisation mismatch detected"
    return out


def quality_metrics(job_dir: Path) -> dict[str, Any]:
    """Prosthetic-vs-prosthetic-IK quality, plus the healthy signal as a symmetry diagnostic."""
    v = trace_view(job_dir)
    q = np.stack([v["obs"][:, RO.IDX_KNEE_Q], v["obs"][:, RO.IDX_ANKLE_Q]], axis=1)
    out: dict[str, Any] = {"trace_sha256": v["trace_sha256"]}
    for tag, ref in (("vs_prosthetic_IK_target", v["q_ik"]), ("vs_healthy_symmetry_diagnostic", v["healthy"])):
        block = {}
        for j, jn in ((0, "knee"), (1, "ankle")):
            x, y = q[:, j], ref[:, j]
            sx, sy = float(x.std()), float(y.std())
            block[jn] = {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                         "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                         "amplitude_ratio": (sx / sy if sy > 0 else None),
                         "sign_agreement": sign_agreement(x, y)}
        block["role"] = ("BINDING quality reference" if tag.startswith("vs_prosthetic")
                         else "symmetry diagnostic only; never an optimisation or sign reference")
        out[tag] = block
    out["realised_ankle"] = sign_degeneracy(q[:, 1])
    return out


# --- preregistered S1C gates (pure; evaluated by future stages) -------------------------------------------

def quality_gates(metrics: Mapping[str, Any], *, s0d_baseline: Mapping[str, Any],
                  ankle_negative_reachable: bool | None = None) -> dict[str, Any]:
    """G2..G5 from rev3v.  G1 (the seven FSM/safety gates) is evaluated by the rollout driver."""
    am = _amendment()["S1C_PROTOCOL"]["PREREGISTERED_QUALITY_GATES"]
    ik = metrics["vs_prosthetic_IK_target"]; hz = metrics["vs_healthy_symmetry_diagnostic"]
    g2b = am["G2_non_regression_vs_S0D_on_the_IK_target"]["baseline_S0D"]
    g3b = am["G3_non_regression_vs_S0D_on_the_symmetry_reference"]["baseline_S0D"]
    g: dict[str, Any] = {}
    g["G2_non_regression_vs_IK"] = {"observed": [ik["knee"]["rmse"], ik["ankle"]["rmse"]], "baseline": g2b,
                                    "rule": "<= baseline per joint", "binding": True,
                                    "pass": bool(ik["knee"]["rmse"] <= g2b[0] and ik["ankle"]["rmse"] <= g2b[1])}
    g["G3_non_regression_symmetry"] = {"observed": [hz["knee"]["rmse"], hz["ankle"]["rmse"]], "baseline": g3b,
                                       "tolerance": 1.05, "rule": "<= 1.05 x baseline per joint", "binding": True,
                                       "pass": bool(hz["knee"]["rmse"] <= 1.05 * g3b[0] and hz["ankle"]["rmse"] <= 1.05 * g3b[1])}
    pr = [ik["knee"]["pearson_r"], ik["ankle"]["pearson_r"]]
    ar = [ik["knee"]["amplitude_ratio"], ik["ankle"]["amplitude_ratio"]]
    base_r = s0d_baseline["pearson_vs_IK"]
    g["G4_shape"] = {"pearson_observed": pr, "pearson_baseline": base_r, "pearson_rule": ">= baseline - 0.02 per joint",
                     "amplitude_ratio_observed": ar, "amplitude_ratio_window": [0.80, 1.25], "binding": True,
                     "pass": bool(all(p is not None and p >= b - 0.02 for p, b in zip(pr, base_r))
                                  and all(a is not None and 0.80 <= a <= 1.25 for a in ar))}
    reach = ankle_negative_reachable
    g["G5_ankle_negative_excursion"] = {"threshold": B3_THRESHOLD, "window": [0.55, 0.80],
                                        "reachable_per_S1C0": reach,
                                        "binding": bool(reach is True),
                                        "status": ("evaluated" if reach is True else
                                                   "OPEN BLOCKER: not binding until the S1C-0 probe returns REACHABLE"),
                                        "pass": None if reach is not True else bool(metrics.get("ankle_window_min", 0.0) <= B3_THRESHOLD)}
    binding = [k for k, v in g.items() if v.get("binding")]
    failed = [k for k in binding if g[k]["pass"] is not True]
    return {"gates": g, "binding": binding, "failed": failed, "all_binding_pass": bool(not failed),
            "no_walking_claim": am["no_walking_claim"]}


# --- guards ------------------------------------------------------------------------------------------------

def run_probe(*_, **kw):
    raise S1CError(f"the S1C-0 feasibility probe requires --authorized-stage {FUTURE_PROBE_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def run_fit(*_, **kw):
    raise S1CError(f"the S1C fit requires --authorized-stage {FUTURE_FIT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def run_rollout(*_, **kw):
    raise S1CError(f"the S1C rollout requires --authorized-stage {FUTURE_ROLLOUT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


# --- audit report -------------------------------------------------------------------------------------------

def build_audit(*, authorized_stage: str | None, out_dir: Path = OUT_DIR) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1CError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_s1c()
    tgt = audit_targets()
    s0d_chain = command_chain_audit(SR.JOB_DIR)
    a2_chain = command_chain_audit(SB.JOB_DIR)
    s0d_q = quality_metrics(SR.JOB_DIR)
    a2_q = quality_metrics(SB.JOB_DIR)
    am = _amendment()
    frozen = am["SIGN_AND_TARGET_AUDIT"]
    checks = {
        "healthy_ankle_is_sign_degenerate": tgt["healthy_contralateral_reference"]["ankle"]["degenerate"] is True,
        "IK_ankle_has_a_negative_tract": tgt["prosthetic_IK_target_decoded"]["ankle"]["frac_negative"] > 0.0,
        "s0d_commands_plantarflexion": s0d_chain["commanded_q_ankle"]["frac_negative"] > 0.0,
        "served_reference_never_negative": s0d_chain["served_reference"]["frac_negative"] == 0.0
                                            and a2_chain["served_reference"]["frac_negative"] == 0.0,
        "sea_not_saturated": s0d_chain["sea_u_ankle"]["frac_abs_gt_0p99"] == 0.0,
        "a2_ankle_regressed_vs_IK": a2_q["vs_prosthetic_IK_target"]["ankle"]["rmse"] > s0d_q["vs_prosthetic_IK_target"]["ankle"]["rmse"],
        "sign_agreement_is_void_on_the_healthy_reference": a2_q["vs_healthy_symmetry_diagnostic"]["ankle"]["sign_agreement"]["verdict"] == "VOID_degenerate_reference",
    }
    if not all(checks.values()):
        raise S1CError(f"the re-derived audit contradicts the frozen findings: {checks}")
    receipt = {"schema": "v26b_s1c_audit.1", "authorized_stage": AUTHORIZED_STAGE, "amendment_rev3v": PIN_AMENDMENT_REV3V,
               "lineage": lineage, "target_and_sign_audit": tgt,
               "command_chain": {"S0D": s0d_chain, "A2": a2_chain},
               "quality_metrics": {"S0D": s0d_q, "A2": a2_q},
               "rederived_checks": checks,
               "frozen_findings_reproduced": True,
               "causal_diagnosis": am["CAUSAL_DIAGNOSIS"],
               "s1c_protocol": am["S1C_PROTOCOL"],
               "executed_in_this_stage": {"fit": False, "rollout": False, "collection": False, "probe": False, "promotion": False},
               "sigma": "UNRESOLVED and not operational",
               "code_digests": {"v26b_s1c_protocol.py": C.sha256_file(Path(__file__).resolve()),
                                "test_v26b_s1c_protocol.py": C.sha256_file(HERE / "test_v26b_s1c_protocol.py") if (HERE / "test_v26b_s1c_protocol.py").is_file() else None},
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    path = R.unique_artifact_path(out_dir, f"v26b_s1c_audit_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "receipt": receipt}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3v S1C sign audit + protocol (validation only)")
    parser.add_argument("--audit", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.audit:
        verify_lineage_s1c()
        print(json.dumps({"mode": "lineage-only", "ok": True}, indent=2))
        return 0
    out = build_audit(authorized_stage=args.authorized_stage)
    r = out["receipt"]
    print(json.dumps({"receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"],
                      "rederived_checks": r["rederived_checks"], "executed": r["executed_in_this_stage"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
