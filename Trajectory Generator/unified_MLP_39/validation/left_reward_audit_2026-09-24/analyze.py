"""Offline LEFT reward decomposition; no simulator, policy or learner is run.

All source traces are preserved. Additive imitation score deficits are allocated
in proportion to weighted loss components: exact accounting, not a causal
attribution or a PPO gradient. Clipping is accounted for separately per step.
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[name] = "1"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
sys.path.insert(0, str(TG))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from unified_MLP_39.reward_function import RewardConfig, compute_reward

RUNS = PACKAGE / "runs/rollout"
PLOTS = REPO / "plot/09_24_2026_left_reward_audit"
SOURCES = {
    "LEFT25": RUNS / "MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25",
    "LEFT40": RUNS / "MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "LEFT60": RUNS / "MLP39_completion_2026-09-22_iter060/deterministic",
    "LEFT72": RUNS / "MLP39_completion_2026-09-22_best072/deterministic",
    "LEFT80": RUNS / "MLP39_completion_2026-09-22_iter080/deterministic",
    "LEFT100": RUNS / "MLP39_completion_2026-09-22_iter100/deterministic",
    "ORACLE_LEFT": RUNS / "causal_difference_2026-09-21_oracle_left",
    "ORACLE_RIGHT": RUNS / "causal_difference_2026-09-21_oracle_right",
    "RIGHT60": RUNS / "MLP39_right_v26_60iter_2026-09-23_last60",
    "V26": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
DEFICITS = [f"deficit_{j}_{k}" for j in ("knee", "ankle") for k in ("position", "velocity")]
HASHES = {}


def read(path):
    raw = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(raw).hexdigest()
    return json.loads(raw)


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def csv_write(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def analyze_row(row, cfg):
    terms = row["reward_terms"]
    reward, comp = compute_reward(terms, cfg, reference=row["policy_segment_values"])
    actual = cfg.blend_imitation * comp["imitation_score"]
    served = cfg.blend_served_imitation * comp["served_imitation_score"]
    assert cfg.blend_imitation_tracking == 0
    before = actual + served - comp["penalty"]
    adjustment = comp["reward_base"] - before
    extras = {k: v for k, v in comp.items() if k.endswith("_term")}
    np.testing.assert_allclose(comp["reward_base"] - sum(extras.values()), reward, atol=1e-12)
    out = {
        "step": row["step"], "time": row["time"],
        "phase": terms["pros_knee_angle_imitation_target_phase"] % 1,
        "reward": reward, "actual_score_contribution": actual,
        "served_score_contribution": served, "base_before_clip": before,
        "base_after_clip": comp["reward_base"], "clip_adjustment": adjustment,
        "clip_low": int(before < cfg.clip_low), "clip_high": int(before > cfg.clip_high),
        "base_penalty": comp["penalty"],
        "command_rate_cost": cfg.command_rate_weight * terms["command_rate_loss"],
        "other_base_cost": comp["penalty"] - cfg.command_rate_weight * terms["command_rate_loss"],
        "penetration_cost": comp["grf_penetration_term"],
        "moment_flip_cost": comp["grf_ankle_moment_flip_term"],
        "other_postclip_cost": sum(extras.values()) - comp["grf_penetration_term"] - comp["grf_ankle_moment_flip_term"],
        "penetration_mm": 1000 * terms["grf_penetration_m"],
        "force_n": terms["grf_ankle_moment_flip_force_n"],
        "moment_nm": terms["grf_ankle_moment_flip_tau_nm"],
        "command_rate_loss": terms["command_rate_loss"],
        "imitation_actual_loss": terms["sound_imitation_loss"],
        "imitation_served_loss": terms["served_imitation_loss"],
        "fsm_state": row["phase_fsm"]["state_name"],
        "jerk_limit_fraction": row["reference_governor_diagnostics"].get("jerk_limit_fraction", 0),
    }
    for joint, short in zip(JOINTS, ("knee", "ankle")):
        for what in ("position", "velocity"):
            w = terms[f"{joint}_imitation_{what}_weight"]
            la = w * terms[f"{joint}_imitation_{what}_loss"]
            ls = w * terms[f"{joint}_served_imitation_{what}_loss"]
            out[f"weighted_loss_actual_{short}_{what}"] = la
            out[f"weighted_loss_served_{short}_{what}"] = ls
            out[f"deficit_{short}_{what}"] = (
                cfg.blend_imitation * cfg.imitation_weight * la / (1 + cfg.imitation_weight * terms["sound_imitation_loss"])
                + cfg.blend_served_imitation * cfg.served_imitation_weight * ls / (1 + cfg.served_imitation_weight * terms["served_imitation_loss"])
            )
        q = row["prosthetic_state"][joint]
        ref = row["imitation_target_q"][joint]
        qs = row["prosthetic_state"][joint + "_served_ref"]
        out[f"{short}_actual_deg"] = float(np.degrees(q))
        out[f"{short}_target_deg"] = float(np.degrees(ref))
        out[f"{short}_served_deg"] = float(np.degrees(qs))
        out[f"{short}_endpoint_deg"] = float(np.degrees(row["policy_segment_values"][-1][JOINTS.index(joint)]))
        out[f"{short}_error_deg"] = float(np.degrees(q - ref))
        out[f"{short}_served_error_deg"] = float(np.degrees(qs - ref))
        out[f"{short}_tracking_error_deg"] = float(np.degrees(q - qs))
    for kind, key in (("actual", "sound_imitation_loss"), ("served", "served_imitation_loss")):
        expected = sum(out[k] for k in out if k.startswith(f"weighted_loss_{kind}_"))
        np.testing.assert_allclose(expected, terms[key], atol=1e-12)
    np.testing.assert_allclose(sum(out[k] for k in DEFICITS), 1 - actual - served, atol=1e-12)
    np.testing.assert_allclose(
        1 - sum(out[k] for k in DEFICITS) - out["base_penalty"] + adjustment
        - out["penetration_cost"] - out["moment_flip_cost"] - out["other_postclip_cost"], reward, atol=1e-12)
    return out


def aggregate(rows):
    totals = {key: float(sum(r[key] for r in rows)) for key in [
        "reward", "actual_score_contribution", "served_score_contribution", *DEFICITS,
        "command_rate_cost", "other_base_cost", "base_penalty", "clip_adjustment",
        "penetration_cost", "moment_flip_cost", "other_postclip_cost"]}
    means = {k: v / len(rows) for k, v in totals.items()}
    rms = {f"{j}_{err}_rmse_deg": float(np.sqrt(np.mean([r[f"{j}_{err}_deg"] ** 2 for r in rows])))
           for j in ("knee", "ankle") for err in ("error", "served_error", "tracking_error")}
    return {"steps": len(rows), "sum": totals, "mean": means, **rms,
            "lower_clipped_steps": sum(r["clip_low"] for r in rows),
            "upper_clipped_steps": sum(r["clip_high"] for r in rows),
            "moment_penalized_steps": sum(r["moment_flip_cost"] > 0 for r in rows),
            "penetration_penalized_steps": sum(r["penetration_cost"] > 0 for r in rows),
            "penetration_max_mm": max(r["penetration_mm"] for r in rows),
            "force_max_n": max(r["force_n"] for r in rows),
            "knee_min_deg": min(r["knee_actual_deg"] for r in rows),
            "ankle_min_deg": min(r["ankle_actual_deg"] for r in rows)}


def main():
    PLOTS.mkdir(parents=True, exist_ok=True)
    table, datasets, all_rows = [], {}, {}
    left_reference = None
    for name, folder in SOURCES.items():
        summary = read(folder / "rollout_summary.json")
        trace = read(folder / "rollout_policy_trace.json")
        cfg = RewardConfig.from_mapping(summary["reward_config"])
        assert cfg.reward_mode == "imitation" and len(trace) == 500
        assert all(b["time"] > a["time"] for a, b in zip(trace, trace[1:]))
        records = [analyze_row(r, cfg) for r in trace]
        measured = sum(r["reward"] for r in records)
        error = abs(measured - summary["episode_return"])
        assert error < 1e-8, (name, error)
        if name.startswith("LEFT") or name == "ORACLE_LEFT":
            refs = np.array([[r["imitation_target_q"][j] for j in JOINTS] for r in trace])
            if left_reference is None:
                left_reference = refs
            np.testing.assert_allclose(refs, left_reference, atol=1e-9, rtol=0)
        a = aggregate(records)
        phases = []
        for i in range(10):
            subset = [r for r in records if int(r["phase"] * 10) == i]
            phases.append({"phase_start": i / 10, "phase_end": (i + 1) / 10, **aggregate(subset)})
        datasets[name] = {"folder": str(folder.relative_to(REPO)), "return_reconstruction_error": error,
            "phase_binning": "10 bins of prescribed target knee phase; physical FSM stored separately; RIGHT bins not phase-matched to LEFT",
            "end_reason": summary["end_reason"], "reward_config": summary["reward_config"],
            "whole": a, "phase_bins": phases,
            "contact_groups": {label: aggregate(group) if group else None for label, group in (
                ("force_ge50N", [r for r in records if r["force_n"] >= 50]),
                ("force_lt50N", [r for r in records if r["force_n"] < 50]),
                ("ankle_target_negative", [r for r in records if r["ankle_target_deg"] < 0]),
                ("moment_penalized", [r for r in records if r["moment_flip_cost"] > 0]),
                ("penetration_penalized", [r for r in records if r["penetration_cost"] > 0]))}}
        all_rows[name] = records
        csv_write(HERE / f"steps_{name}.csv", records)
        table.append({"dataset": name, "return": measured, "knee_rmse_deg": a["knee_error_rmse_deg"],
            "ankle_rmse_deg": a["ankle_error_rmse_deg"], "clip_steps": a["lower_clipped_steps"], **a["sum"]})
    save(HERE / "metrics.json", {"datasets": datasets, "new_training": 0, "new_simulations": 0,
        "imitation_deficit_allocation": "Exact proportional allocation of each score deficit to four weighted loss components BEFORE base clipping; not independent causal effects.",
        "limitations": ["One deterministic episode per checkpoint, one fixed start, one training seed.",
            "Rows are repeated correlated samples, not independent trials.",
            "RIGHT/V26 returns concern different target tasks and are descriptive context only.",
            "Direct oracle has 10ms preview and different control behavior, not an executable learned checkpoint.",
            "LEFT25-to40 optimizer reset confounds later historical checkpoint progression."]})
    csv_write(HERE / "summary.csv", table)
    save(HERE / "sources.json", {k: str(v.relative_to(REPO)) for k, v in SOURCES.items()})
    csv_write(HERE / "phase_summary.csv", [{"dataset": name, "bin_start": p["phase_start"], "n": p["steps"],
        **p["mean"], "knee_rmse_deg": p["knee_error_rmse_deg"], "ankle_rmse_deg": p["ankle_error_rmse_deg"]}
        for name, d in datasets.items() for p in d["phase_bins"]])
    plot(all_rows, datasets)
    for path in (PACKAGE / "reward_function.py", TG / "osim_trj_cmc_like.py", HERE / "analyze.py"):
        HASHES[str(path.relative_to(REPO))] = hashlib.sha256(path.read_bytes()).hexdigest()
    assert all(hashlib.sha256((REPO / path).read_bytes()).hexdigest() == h for path, h in HASHES.items())
    save(HERE / "input_hashes.json", HASHES)
    save(HERE / "verification.json", {"passed": True, "sources_unchanged": True,
        "deterministic_episodes": len(datasets), "rows": sum(len(r) for r in all_rows.values()),
        "return_error_max": max(d["return_reconstruction_error"] for d in datasets.values()),
        "left_q_targets_identical": True, "decomposition_verified_each_step": True,
        "raw_env_scores_not_used": True, "new_training": 0, "new_simulations": 0})
    print(json.dumps(table, indent=2))


def plot(rows, metrics):
    names = ["LEFT25", "LEFT40", "LEFT60", "LEFT72", "LEFT80", "LEFT100", "ORACLE_LEFT"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), layout="constrained")
    keys = DEFICITS + ["command_rate_cost", "other_base_cost", "moment_flip_cost", "penetration_cost", "other_postclip_cost"]
    labels = ["Errore posizione knee", "Errore velocità knee", "Errore posizione ankle", "Errore velocità ankle",
              "Variazione comandi", "Altri costi base", "Momento GRF", "Penetrazione", "Altri costi fisici"]
    bottom = np.zeros(len(names))
    for key, label in zip(keys, labels):
        values = np.array([metrics[n]["whole"]["sum"][key] for n in names])
        axes[0].bar(names, values, bottom=bottom, label=label)
        bottom += values
    axes[0].set(title="Deficit di imitazione e costi cumulativi — 500 step, stesso target LEFT", ylabel="Deficit/costi prima della correzione clipping [punti]")
    axes[0].legend(ncol=3, fontsize=8)
    for name in ("LEFT40", "LEFT72", "ORACLE_LEFT"):
        phase = metrics[name]["phase_bins"]
        axes[1].plot(np.arange(5, 100, 10), [p["mean"]["reward"] for p in phase], "o-", label=name)
    axes[1].set(xlabel="Fase prescribed sinistra [%]", ylabel="Reward media per step", title="Reward per fase: media su campioni correlati, non stima statistica")
    axes[1].legend(); axes[1].grid(alpha=.25)
    fig.suptitle("Audit offline LEFT: scomposizione della reward effettiva\nDeficit non lineari ripartiti proporzionalmente; clipping contabilizzato separatamente nel CSV", fontsize=12)
    fig.savefig(PLOTS / "reward_decomposition.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
    for name in ("LEFT40", "LEFT72", "ORACLE_LEFT"):
        phase = metrics[name]["phase_bins"]
        for ax, key, title in zip(axes.flat,
            ("command_rate_cost", "moment_flip_cost", "deficit_ankle_position", "deficit_knee_position"),
            ("Costo variazione comandi", "Penalità momento GRF", "Deficit score: posizione caviglia", "Deficit score: posizione ginocchio")):
            ax.plot(np.arange(5,100,10), [p["mean"][key] for p in phase], "o-", label=name)
            ax.set(title=title, xlabel="Fase prescribed sinistra [%]", ylabel="Punti / step"); ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.savefig(PLOTS / "phase_contributions.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, layout="constrained")
    for name in ("LEFT72", "ORACLE_LEFT"):
        rs = rows[name]; t = np.arange(1, len(rs)+1) * .01
        axes[0].plot(t, [r["ankle_actual_deg"] for r in rs], label=name)
        axes[1].plot(t, [r["reward"] for r in rs], label=name)
        axes[2].plot(t, [r["command_rate_cost"] for r in rs], label=name)
        axes[3].plot(t, [r["moment_nm"] if r["force_n"] >=50 else np.nan for r in rs], label=name)
    axes[0].plot(t, [r["ankle_target_deg"] for r in rows["LEFT72"]], "k--", label="Target LEFT")
    for ax, y in zip(axes, ["Angolo ankle [°]", "Reward / step", "Costo comandi / step", "Momento GRF ankle [Nm]"]):
        ax.set_ylabel(y); ax.grid(alpha=.25); ax.legend(loc="upper left", ncol=3, fontsize=8)
    axes[3].axhline(-8, ls="--", c="gray", label="Soglia anti-flip")
    axes[3].set_xlabel("Tempo dall'inizio [s]")
    fig.suptitle("LEFT72 e comando diretto LEFT: stessa reward, comportamenti diversi\nComando diretto: preview 10 ms, singolo episodio; fattibilità nominale, non robustezza")
    fig.savefig(PLOTS / "left72_vs_direct_reference.png", dpi=170); plt.close(fig)


if __name__ == "__main__":
    main()
