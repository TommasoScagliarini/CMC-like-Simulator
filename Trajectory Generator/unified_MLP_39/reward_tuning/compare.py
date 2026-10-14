"""Paired physical screening with separate completion and common-prefix gates."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .io import resolve, write_json
from .metrics import load_dataset, measure

DEFAULTS = dict(tracking_relative_increase=.10, tracking_absolute_increase_deg=.5,
                excursion_min_ratio=.90, impulse_min_ratio=.90,
                nominal_penetration_limit_mm=20., penetration_increase_limit_mm=2.,
                ankle_area_min_reduction=.10, regularity_max_increase=.10)


def compare_metrics(candidate, reference, criteria=None, *, nominal=False):
    """Compare already aligned metrics; None means unknown, never a pass."""
    limits = {**DEFAULTS, **(criteria or {})}
    checks = []
    def check(name, actual, bound, sense="max"):
        available = actual is not None and bound is not None
        passed = None if not available else bool(actual <= bound + 1e-9 if sense == "max" else actual >= bound - 1e-9)
        checks.append(dict(name=name, actual=actual, bound=bound, passed=passed))
    ce, re = candidate["episode"], reference["episode"]
    check("completion_of_reference_scenario", int(ce["completed"]), int(re["completed"]), "min")
    check("valid_cycles", ce["valid_cycles"], re["valid_cycles"], "min")
    check("no_new_hard_termination", int("hard" in str(ce["end_reason"])), int("hard" in str(re["end_reason"])))
    p = ce["penetration_max_mm"]
    rp = re["penetration_max_mm"]
    check("penetration_increase_mm", p, None if rp is None else rp + limits["penetration_increase_limit_mm"])
    if nominal:
        # Strictly below the soft threshold, as stipulated in the protocol.
        check("nominal_penetration_mm", p, limits["nominal_penetration_limit_mm"] - 1e-6)
    for joint in ("knee", "ankle"):
        ct, rt = candidate["tracking"][joint], reference["tracking"][joint]
        for label, c, r in [("global", ct, rt), *[(f"phase_{b}", ct["phase_bins"].get(str(b), {}), rt["phase_bins"].get(str(b), {})) for b in range(10)]]:
            for kind in ("actual", "served"):
                key = kind + "_rmse_deg"
                value = r.get(key)
                bound = None if value is None else value + min(limits["tracking_relative_increase"] * value, limits["tracking_absolute_increase_deg"])
                check(f"tracking_{joint}_{kind}_{label}", c.get(key), bound)
        check(f"excursion_{joint}", ct["excursion_deg"], rt["excursion_deg"] * limits["excursion_min_ratio"], "min")
    cc, rc = candidate.get("contact") or {}, reference.get("contact") or {}
    impulse = rc.get("vertical_impulse_n_s")
    check("support_impulse", cc.get("vertical_impulse_n_s"), None if impulse is None else limits["impulse_min_ratio"] * impulse, "min")
    for key in ("rate_rms_nm_s", "hp10_rms_nm", "hp20_rms_nm"):
        c, r = (candidate.get("knee") or {}).get(key), (reference.get("knee") or {}).get(key)
        check("knee_regularity_" + key, c, None if r is None else (1 + limits["regularity_max_increase"]) * r)
    for joint in ("knee", "ankle"):
        for channel in ("actual", "served"):
            key = joint + "_" + channel
            for metric in ("jerk_rms_rad_s3", "position_hp10_rms_deg"):
                c, r = candidate["kinematic_smoothness"][key][metric], reference["kinematic_smoothness"][key][metric]
                check("kinematic_regularity_" + key + "_" + metric, c, None if r is None else (1 + limits["regularity_max_increase"]) * r)
    return checks


def _paired_receipts(paths):
    receipts = {}
    for role, path in paths.items():
        receipt = path / "paired_noise_receipt.json"
        if not receipt.is_file():
            return {"passed": None, "reason": f"Missing paired-noise receipt: {role}"}
        receipts[role] = json.loads(receipt.read_text(encoding="utf-8"))
    if len({r["full_draws_sha256"] for r in receipts.values()}) != 1:
        return {"passed": False, "reason": "Gaussian streams differ"}
    if len({r["sigma_sha256"] for r in receipts.values()}) != 1 or any(not r["sigma_sha256"] for r in receipts.values()):
        return {"passed": False, "reason": "Frozen sigma schedules differ or are missing"}
    n = min(len(r["records"]) for r in receipts.values())
    if not n:
        return {"passed": None, "reason": "No shared perturbation samples"}
    baseline = receipts["best80"]["records"][:n]
    for role, receipt in receipts.items():
        for before, after in zip(baseline, receipt["records"][:n]):
            if before["standard_normal"] != after["standard_normal"] or before["sigma"] != after["sigma"]:
                return {"passed": False, "reason": f"Actual perturbation inputs differ for {role}"}
    return {"passed": True, "common_draws": n}


def compare(config, output):
    output = Path(output)
    pilot = output / "pilot"
    manifest = json.loads((pilot / "pilot_manifest.json").read_text(encoding="utf-8"))
    state = json.loads((pilot / "pilot_state.json").read_text(encoding="utf-8"))
    from .pilot import verify_hashes, sha256
    if state["manifest_sha256"] != sha256(pilot / "pilot_manifest.json"):
        raise ValueError("Prepared pilot manifest changed")
    verify_hashes(Path(manifest["repo_root"]), manifest["input_hashes"])
    candidate_role = manifest.get("candidate_role", "ankle_10")
    if not isinstance(candidate_role, str) or not candidate_role or candidate_role in {"best80", "control"}:
        raise ValueError("Pilot candidate_role must identify a distinct candidate")
    cases = {}
    missing = []
    for case in manifest["evaluation_cases"]:
        role, scenario = case["role"], case["scenario"]
        path = resolve(state.get("evaluation", {}).get(role, {}).get(scenario, case["output_dir"]))
        if not (path / "evaluation_complete.json").is_file():
            missing.append(f"{role}/{scenario}")
        else:
            receipt = json.loads((path / "evaluation_complete.json").read_text(encoding="utf-8"))
            hashes = receipt.get("artifact_hashes", {})
            required = {"rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json"}
            # Any physical data used by load_dataset must belong to the sealed
            # evaluation too; absent STO files remain explicit missing metrics.
            required.update(f"sim_outputs/rollout_episode_{name}.sto"
                            for name in ("sea_torques", "kinematics", "kinematics_reference", "online_grf")
                            if (path / "sim_outputs" / f"rollout_episode_{name}.sto").is_file())
            if not isinstance(hashes, dict) or not required.issubset(hashes):
                missing.append(f"{role}/{scenario}: incomplete artifact receipt")
                continue
            if receipt.get("role", role) != role or receipt.get("scenario", scenario) != scenario:
                raise ValueError(f"Evaluation receipt identifies a different case: {path}")
            verify_hashes(path, hashes)
            cases.setdefault(scenario, {})[role] = path
    result = {"schema": "reward_tuning_comparison_v1", "status": "incomplete", "missing_cases": missing,
              "candidate_role": candidate_role,
              "criteria": {**DEFAULTS, **config.get("criteria", {})}, "scenarios": {}, "input_hashes": {},
              "interpretation": "Exploratory paired screening; not statistical robustness certification"}
    all_checks, nominal_area_checks, noisy_area_delta = [], [], {"best80": [], "control": []}
    for scenario, paths in cases.items():
        if set(paths) != {"best80", "control", candidate_role}:
            continue
        data = {role: load_dataset(path, identity=role, scenario=scenario) for role, path in paths.items()}
        full = {role: measure(d) for role, d in data.items()}
        for d in data.values():
            result["input_hashes"].update(d["input_hashes"])
        interval = config.get("analysis_window_s", [13.15, 17.95])
        lo = max(interval[0], *(m["episode"]["start_s"] for m in full.values()))
        hi = min(interval[1], *(m["episode"]["stop_s"] for m in full.values()))
        if hi <= lo:
            missing.append(f"{scenario}: no common complete segment")
            continue
        aligned = {role: measure(d, [lo, hi], filter_stop=hi) for role, d in data.items()}
        entry = {"full": full, "common_window": aligned, "comparisons": {}}
        if scenario != "nominal":
            receipt = _paired_receipts(paths)
            entry["pairing"] = receipt
            all_checks.append(receipt)
        for ref in ("best80", "control"):
            checks = compare_metrics(aligned[candidate_role], aligned[ref], result["criteria"], nominal=scenario == "nominal")
            # Global RMSE is also checked over the whole episode when durations
            # match. Otherwise the common-prefix result is explicitly restricted.
            if all(m["episode"]["completed"] for m in full.values()):
                extra = compare_metrics(full[candidate_role], full[ref], result["criteria"], nominal=scenario == "nominal")
                checks += [{**c, "name": "full_episode_" + c["name"]} for c in extra]
            all_checks += checks
            entry["comparisons"][ref] = checks
            ca, ra = full[candidate_role]["ankle"], full[ref]["ankle"]
            # Shorter failed trajectories must never win via smaller torque area.
            eligible = full[candidate_role]["episode"]["completed"] and full[ref]["episode"]["completed"] and ca is not None and ra is not None
            delta = ca["positive_area_nm_s"] - ra["positive_area_nm_s"] if eligible else None
            if scenario == "nominal":
                passed = None if delta is None else bool(ca["positive_area_nm_s"] <= (1 - result["criteria"]["ankle_area_min_reduction"]) * ra["positive_area_nm_s"])
                nominal_area_checks.append({"reference": ref, "passed": passed, "delta_nm_s": delta})
            else:
                noisy_area_delta[ref].append(delta)
        result["scenarios"][scenario] = entry
    result["nominal_ankle_area"] = nominal_area_checks
    result["noisy_ankle_area"] = {}
    for ref, deltas in noisy_area_delta.items():
        complete = len(deltas) == 3 and all(v is not None for v in deltas)
        median = float(np.median(deltas)) if complete else None
        result["noisy_ankle_area"][ref] = {"paired_deltas_nm_s": deltas, "median_delta_nm_s": median,
                                          "passed": None if median is None else median <= 0}
    area_checks = nominal_area_checks + list(result["noisy_ankle_area"].values())
    decisions = [c.get("passed") for c in all_checks + area_checks]
    if not missing and len(result["scenarios"]) == 4:
        if any(value is False for value in [c.get("passed") for c in all_checks]):
            result["status"] = "rejected_constraints"
        elif decisions and all(value is True for value in decisions):
            result["status"] = "promising"
        else:
            result["status"] = "inconclusive"
    write_json(output / "comparison.json", result)
    lines = ["# Screening fisico del pilot LEFT", "", f"Stato: **{result['status']}**.", "",
             f"Candidata: `{candidate_role}`.", "",
             result["interpretation"], "", "Priorità: area positiva ankle, regolarità coppia knee, regolarità cinematica.", ""]
    if missing:
        lines += ["Valutazioni mancanti; nessun candidato è dichiarato migliore:", "", *[f"- {name}" for name in missing]]
    for scenario, entry in result["scenarios"].items():
        lines += ["", f"## {scenario}", ""]
        for ref, checks in entry["comparisons"].items():
            failed = [c["name"] for c in checks if c["passed"] is False]
            unknown = [c["name"] for c in checks if c["passed"] is None]
            lines += [f"- Contro {ref}: {len(failed)} vincoli falliti, {len(unknown)} non valutabili."]
    lines += ["", "Le aree dell'intero episodio non classificano rollout interrotti. Il dettaglio "
              "JSON mantiene esiti, metriche complete, prefissi comuni e tutti i limiti verificati.", ""]
    (output / "comparison.md").write_text("\n".join(lines), encoding="utf-8")
    return result
