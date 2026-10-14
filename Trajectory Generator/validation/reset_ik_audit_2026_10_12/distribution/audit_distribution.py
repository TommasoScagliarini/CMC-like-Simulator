"""Offline audit of recorded best47 sampling; never imports OpenSim/Ray/Torch.

Run from any directory with a Python environment providing NumPy and SciPy.
The reconstructed IK values describe current files, not recorded PPO resets.
"""
from __future__ import annotations

import ast
import contextlib
import csv
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import scipy
from scipy.interpolate import CubicSpline
from scipy.signal import butter, sosfiltfilt

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
RUN = ROOT / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path):
    return str(path.relative_to(ROOT))


def main():
    summary = json.loads((RUN / "summary.json").read_text())
    rows = [json.loads(line) for line in (RUN / "train_iterations.jsonl").read_text().splitlines() if line.strip()]
    best = json.loads((RUN / "checkpoint_best_meta.json").read_text())
    config_text = (RUN / "training_cfg.resolved.yaml").read_text()
    assert "random_init: false" in config_text
    assert "imitation_initialize_to_target: false" in config_text
    offsets = summary["episode_start_offset_choices_s"]
    assert [r["iteration"] for r in rows] == list(range(2, 52))
    setup = ROOT / summary["setup_xml_path"]
    xml = ET.parse(setup).getroot()
    t_start = float(xml.findtext(".//t_start"))
    t_end = float(xml.findtext(".//t_end"))
    ik_path = ROOT / xml.findtext(".//kinematics_file")
    labels = ["offset_" + f"{v:.6f}".replace(".", "p") + "s" for v in offsets]
    per_offset = []
    iteration_table = []
    for offset, label in zip(offsets, labels):
        actual = [r["exact_start_balance"]["actual_steps"][label] for r in rows]
        sampled = [r["per_start_metrics"][label]["sampled_steps"] for r in rows]
        assert actual == sampled == [1536.0] * 50
        assert all(r["exact_start_balance"]["pass"] and r["exact_start_balance"]["learner_batch_pass"] for r in rows)
        episodes = sum(r["per_start_metrics"][label]["completed_episodes"] for r in rows)
        per_offset.append({
            "offset_s": offset, "metric_label": label,
            "absolute_reset_time_s_inferred_from_current_setup": t_start + offset,
            "new_ppo_transitions": int(sum(actual)), "transitions_per_update": 1536,
            "observed_completed_episode_count": int(episodes),
            "runners_assigned_from_current_sampler": [i for i in range(1, 13) if (i - 1) % 3 == labels.index(label)],
            "final_lifetime_transitions": rows[-1]["start_coverage_metrics"][f"env_runners/episode_start_steps/{label}"],
        })
    for row in rows:
        for item in per_offset:
            label = item["metric_label"]
            iteration_table.append({
                "logical_iteration": row["iteration"], "offset_s": item["offset_s"],
                "actual_transitions": row["exact_start_balance"]["actual_steps"][label],
                "completed_episodes": row["per_start_metrics"][label]["completed_episodes"],
                "balance_pass": row["exact_start_balance"]["pass"],
                "learner_batch_pass": row["exact_start_balance"]["learner_batch_pass"],
                "max_start_run_length": row["exact_start_balance"]["max_start_run_length"],
            })
    # Evaluate the existing pure NumPy/SciPy interpolation code without importing
    # any production module or simulator/plugin. All other module statements are excluded.
    kin_source = ROOT / "kinematics_interpolator.py"
    tree = ast.parse(kin_source.read_text())
    kept = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in {"KinematicsInterpolator", "_read_sto"}]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, *kept], type_ignores=[]))
    ns = {"np": np, "Path": Path, "CubicSpline": CubicSpline, "butter": butter, "sosfiltfilt": sosfiltfilt,
          "resolve_simulator_paths": lambda cfg: SimpleNamespace(kinematics_path=ik_path)}
    exec(compile(module, str(kin_source), "exec"), ns)
    cfg = SimpleNamespace(t_start=t_start, t_end=t_end, translation_coords=["pelvis_tx", "pelvis_ty", "pelvis_tz"],
                          enable_kinematics_lowpass_filter=True, kinematics_lowpass_cutoff_hz=6.0,
                          kinematics_lowpass_order=4, kinematics_resample_dt=0.001)
    with contextlib.redirect_stdout(io.StringIO()):
        kin = ns["KinematicsInterpolator"](cfg)
    q_rows = []
    for item in per_offset:
        q, qdot, _ = kin.get(item["absolute_reset_time_s_inferred_from_current_setup"])
        item["current_ik_reconstruction_prosthetic"] = {
            name: {"q_rad": q[name], "qdot_rad_s": qdot[name]} for name in ["pros_knee_angle", "pros_ankle_angle"]
        }
        for name in q:
            q_rows.append({"offset_s": item["offset_s"], "absolute_time_s": t_start + item["offset_s"],
                           "coordinate": name, "q": q[name], "qdot": qdot[name],
                           "position_unit": "m" if name in cfg.translation_coords else "rad",
                           "velocity_unit": "m/s" if name in cfg.translation_coords else "rad/s",
                           "evidence_kind": "current_files_offline_reconstruction_not_recorded_training_reset"})
    predecessors = []
    jroot = ROOT / "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
    jfiles = sorted(jroot.glob("j19[bc]_runs/*/*receipt.json"))
    for file in jfiles:
        d = json.loads(file.read_text())
        predecessors.append({"source": rel(file), "sha256": sha(file), "aggregate_pass": d["aggregate_pass"],
                             "cells": [{"id": c["id"], "mode": c["mode"], "seed": c["seed"],
                                        "offset_s": c["episode_start_offset_s"], "actual_reset_time_s": c["reset_check"]["actual_reset_time_s"],
                                        "steps": c["summary"]["steps"], "behavioural_pass": c["behavioural_pass"],
                                        "sigma": c["action_semantics"].get("sigma")} for c in d["cells"]],
                             "scope": "historical J19 actor qualification, not a present best47 reset test"})
    source_files = [RUN / "summary.json", RUN / "train_iterations.jsonl", RUN / "training_cfg.resolved.yaml",
                    RUN / "checkpoint_best_meta.json", setup, ik_path, kin_source, ROOT / "config.py",
                    ROOT / "simulation_runner.py", ROOT / "Trajectory Generator/osim_trj_cmc_like.py",
                    ROOT / "Trajectory Generator/baseline_MLP/env_factory.py", ROOT / "Trajectory Generator/baseline_MLP/tb_logging.py",
                    ROOT / "Trajectory Generator/baseline_MLP/train_ppo_mlp.py", *jfiles]
    limitations = [
        "The training run contains aggregated offset/episode statistics, not a persisted per-reset q/qdot/seed/time series. Reset counts and state equality across successive episodes cannot be proved from transition counts alone.",
        "Recorded completed episodes are end-callback counts, not the count of reset() calls: initial and final unfinished episodes differ, and historical per-runner episode IDs are absent.",
        "No runtime_manifest/progress.csv/result.json/reset telemetry file was found under the run. Summary/history, resolved YAML, and per-iteration guards are the authoritative available evidence.",
        "Current setup/IK/config and current sampler/reset source are SHA-pinned here, but their identity with files executed in September is not independently pinned by the inspected run metadata. Reconstructed q/qdot are current-file expectations, not historical measured reset states.",
        "Sampling interleaving decorrelates condition order in minibatches, not the physical reset state. Three fixed IK times span 0.4s of one AB06 trajectory; this is not broad initial-state randomization or statistical independence.",
        "Historical J19 qualification used an earlier actor and sigma0.005/held noise where declared; it does not establish robustness of the PPO best47 actor today.",
    ]
    result = {
        "audit_date": "2026-10-12", "scope": "offline historical artifact audit and current IK reconstruction only",
        "run": rel(RUN), "best_logical_iteration": best["logical_iteration"], "new_ppo_updates": len(rows),
        "logical_iterations": [2, 51], "restored_logical_iteration": summary["restored_logical_iteration"],
        "new_ppo_transitions_total": sum(x["new_ppo_transitions"] for x in per_offset),
        "best47_prefix_new_updates": 46, "best47_prefix_transitions_per_offset": 46 * 1536,
        "random_init": False, "imitation_initialize_to_target": False, "per_offset": per_offset,
        "summary_history_equal_to_jsonl": summary["history"] == rows,
        "condition_balance_passes": sum(r["exact_start_balance"]["pass"] for r in rows),
        "learner_batch_balance_passes": sum(r["exact_start_balance"]["learner_batch_pass"] for r in rows),
        "interleaved_max_start_run_length_values": sorted(set(r["exact_start_balance"]["max_start_run_length"] for r in rows)),
        "inherited_nominal_lifetime_steps": 4096,
        "reset_semantics": "Current reset draws a uniform time only if random_init; otherwise fixed assigned offset is clamped to t_end-duration. It sets coordinate q/qdot from IK, SEA motor angle/speed from joint q/qdot, resets controller/SO/FSM memory, and applies no reset q/qdot noise.",
        "runtime_file_scan": {"runtime_manifest_files": [rel(p) for p in RUN.rglob("*runtime_manifest*")],
                              "progress_csv_files": [rel(p) for p in RUN.rglob("progress.csv")]},
        "current_ik_preprocessing": {"filter": "zero-phase Butterworth", "cutoff_hz": 6.0, "order": 4, "resample_dt_s": 0.001,
                                     "numpy": np.__version__, "scipy": scipy.__version__, "historically_attested": False},
        "historical_j19": predecessors, "limitations": limitations,
        "sources": [{"path": rel(p), "sha256": sha(p)} for p in source_files],
    }
    (OUT / "distribution_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    for name, table in [("per_iteration_offsets.csv", iteration_table), ("current_ik_reset_q_qdot.csv", q_rows)]:
        with (OUT / name).open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(table[0]))
            writer.writeheader()
            writer.writerows(table)
    lines = ["# Distribuzione reset IK — audit offline best47", "", "Data: 2026-10-12. Nessun training, rollout, test nativo o cambiamento alla produzione.", "",
             f"Run: `{rel(RUN)}`. Il best è logical47; il run ha 50 nuovi update (logical2–51), riprendendo logical1.", "",
             "Tutte le 50 righe registrano 1.536 transizioni reali per offset e PASS sia sul campionamento sia sul batch Learner: 76.800 per offset, 230.400 totali. Fino a best47: 46 update, 70.656 transizioni per offset.", "",
             "| Offset [s] | Tempo IK assoluto [s], ricostruito | Runner assegnati | Nuove transizioni | Episodi completati registrati |", "|---:|---:|---|---:|---:|"]
    for x in per_offset:
        lines.append(f"| {x['offset_s']:.12f} | {x['absolute_reset_time_s_inferred_from_current_setup']:.12f} | {x['runners_assigned_from_current_sampler']} | {x['new_ppo_transitions']} | {x['observed_completed_episode_count']} |")
    lines += ["", "La lifetime nominale finale è 80.896: include 4.096 step ereditati dal rewarm critic precedente; non indica uno sbilanciamento dei nuovi update. Le altre lifetime sono 76.800.", "",
              "Il sampler assegna deterministicamente `(worker_index-1+vector_index)%3`; quattro runner per offset. `random_init:false`: ogni runner riusa lo stesso tempo IK a ogni reset. Non è presente jitter su q/qdot. L'interleaving ha run-length massimo1 per la condizione, ma non rende indipendenti gli stati iniziali.", "",
              "q/qdot riportati nel CSV sono calcolati offline usando il codice d'interpolazione corrente, IK corrente, filtro6Hz ordine4/dt1ms. Sono aspettative dai file attuali, non stati reset misurati durante il run storico. SEA motor angle/speed sono inizializzati alla posizione/velocità del giunto dal reset corrente.", "",
              "| Offset [s] | Knee q [rad] | Knee qdot [rad/s] | Ankle q [rad] | Ankle qdot [rad/s] |", "|---:|---:|---:|---:|---:|"]
    for x in per_offset:
        k = x["current_ik_reconstruction_prosthetic"]["pros_knee_angle"]
        a = x["current_ik_reconstruction_prosthetic"]["pros_ankle_angle"]
        lines.append(f"| {x['offset_s']:.6f} | {k['q_rad']:.8f} | {k['qdot_rad_s']:.8f} | {a['q_rad']:.8f} | {a['qdot_rad_s']:.8f} |")
    lines += ["", "Precedenti J19B 6/6 e J19C 3/3 sono registrati separatamente nel JSON: i tre start deterministici e i seed stocastici nominali qualificavano l'actor J19 precedente, non best47 e non una prova odierna.", "", "Limiti:", ""]
    lines += ["- " + text for text in limitations]
    lines += ["", "Fonti: SHA256 e path completi nel JSON; righe50×3 nel CSV. Riferimenti codice:", "",
              "- `env_factory.py:52–110`: assegnazione deterministica e callback prima del reset.",
              "- `osim_trj_cmc_like.py:1786`, `:2540–2609`: seed/reset, tempo e inizializzazione episodio.",
              "- `simulation_runner.py:337–390`, `:1796–1843`: IK q/qdot e SEA reset.",
              "- `tb_logging.py:332–364`, `:458–505`: contatori transizioni corrente/lifetime ed episodi completati.",
              "- `train_ppo_mlp.py:1134–1195`: per_start_metrics aggregati.", ""]
    (OUT / "distribution_audit.md").write_text("\n".join(lines))
    print(json.dumps({"updates": len(rows), "transitions": result["new_ppo_transitions_total"],
                      "per_offset": per_offset, "summary_history_equal": result["summary_history_equal_to_jsonl"]}, indent=2))


if __name__ == "__main__":
    main()
