"""Offline 3-policy x 2-target comparison and receipt-verified heldout critics.

No new optimizer updates, sampling, Ray cluster or physical simulation.
Incomplete rollout cases remain explicitly pending; rerun after receipts arrive.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import learner_analysis as analysis
import numpy as np
import torch

DEFAULT_REPLAY = HERE / "boundary_replay_v2"
DEFAULT_OUTPUT = HERE / "replay_comparison"


def verify_probe(path):
    manifest = json.loads((path / "manifest.json").read_text())
    if not manifest.get("complete"):
        raise ValueError(f"Incomplete probe: {path}")
    hashes = json.loads((path / "artifact_hashes.json").read_text())
    inputs = {}
    for name, expected in hashes.items():
        file = (path / name).resolve()
        if analysis.digest(file) != expected:
            raise ValueError(f"Probe artifact changed: {file}")
        inputs[str(file)] = expected
    inputs[str((path / "artifact_hashes.json").resolve())] = analysis.digest(path / "artifact_hashes.json")
    return manifest, inputs


def evaluation_cases(manifests):
    cases, inputs = [], {}
    for path in manifests:
        path = Path(path).resolve()
        if not path.exists():
            continue
        inputs[str(path)] = analysis.digest(path)
        data = json.loads(path.read_text())
        cases.extend(data["cases"])
    return cases, inputs


def heldout_comparison(cases, modules, config):
    output, inputs = [], {}
    for case in cases:
        folder = Path(case["output_dir"])
        row = {"case": folder.name, "requested_role": case.get("comparison_role", case.get("role")),
               "scenario": case["scenario"], "noise_seed": case.get("noise_seed"), "folder": str(folder)}
        receipt_path = folder / "evaluation_complete.json"
        transition_path = folder / "critic_transitions.npz"
        if not receipt_path.exists():
            output.append({**row, "available": False, "pending": True, "reason": "completion receipt absent"})
            continue
        inputs[str(receipt_path)] = analysis.digest(receipt_path)
        if not transition_path.exists():
            output.append({**row, "available": False, "pending": False,
                           "reason": "Completed rollout lacks full critic observations (e.g. reused actor-only archive)"})
            continue
        result = analysis.analyze_heldout([transition_path], modules, config)[0]
        generator = {"after": "control"}.get(row["requested_role"], row["requested_role"])
        result["generator_policy"] = generator
        result["critic_checkpoint_matches_generator_checkpoint"] = {name: name == generator for name in modules}
        result["own_stochastic_behavior"] = {
            name: row["scenario"] == "noisy" and name == generator for name in modules
        }
        result["off_policy_comparison_note"] = (
            "Observed rewards/returns belong to the generating behavior. A matching checkpoint "
            "only matches the critic's own stochastic behavior in a noisy rollout. Deterministic "
            "nominal actions differ from the stochastic policy even at the same checkpoint. "
            "Cross-critic scores do not establish each critic's own-policy value accuracy."
        )
        output.append({**result, **row, "pending": False})
        for name in ("critic_transitions.npz", "rollout_summary.json"):
            inputs[str(folder / name)] = analysis.digest(folder / name)
    return output, inputs


def make_plot(path, table):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.dpi": 190, "figure.facecolor": "white"})
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8), constrained_layout=True)
    policies = ["before", "control", "corrected"]
    x = np.arange(3)
    for index, (target, color) in enumerate((("stock", "#b4544e"), ("corrected_real_boundary", "#247b95"))):
        rows = [next(row for row in table if row["policy"] == policy and row["target"] == target) for policy in policies]
        offset = -.19 if index == 0 else .19
        for ax, key in zip(axes[:2], ("vf_mse_raw", "policy_loss")):
            ax.bar(x + offset, [row[key] for row in rows], .38, color=color, label=target.replace("_", " "))
    kl_rows = [next(row for row in table if row["policy"] == policy) for policy in policies]
    axes[2].bar(x, [row["kl_mean"] for row in kl_rows], color=["#777777", "#b4544e", "#247b95"])
    for ax, title in zip(axes, ("Critic MSE on each fixed target", "PPO surrogate on each fixed advantage", "KL(old || policy), same observations")):
        ax.set_title(title)
        ax.set_xticks(x, policies)
        ax.grid(axis="y", alpha=.2)
    axes[0].legend(fontsize=9)
    fig.suptitle("One captured batch, exact control replay, isolated boundary correction\nCompare policies within each target color; physical outcomes require separate evaluation")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def fragment_boundary_plot(path, output, source_probe, stock, corrected):
    """Last 50 real decisions of the first open fragment; all values are saved."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    episodes = json.loads((source_probe / "episodes_pre_gae.json").read_text())
    position = 0
    selected = None
    for episode in episodes:
        if not episode["terminated"] and not episode["truncated"]:
            selected = episode
            break
        position += episode["length"] + 1
    if selected is None:
        raise ValueError("No unfinished fragment is available for the boundary figure")
    boundary = position + selected["length"]
    start = max(position, boundary - 50)
    index = np.arange(start, boundary)
    # This audit uses fixed 10 ms decisions. Label decisions instead of deriving
    # physical timestamps from an unverified dt inside this plotting helper.
    distance = index - boundary
    values = analysis.array(analysis.load_pickle(source_probe / "gae_values_and_input.pkl")["values"]["default_policy"])
    old_targets = analysis.array(stock["value_targets"])[index]
    new_targets = analysis.array(corrected["value_targets"])[index]
    old_adv = analysis.array(stock["advantages"])[index]
    new_adv = analysis.array(corrected["advantages"])[index]
    last_reward = float(analysis.array(stock["rewards"])[boundary - 1])
    payload = {"batch_index": index, "decisions_before_boundary": distance,
               "value_before": values[index], "target_stock": old_targets,
               "target_corrected_real_boundary": new_targets, "advantage_stock_normalized": old_adv,
               "advantage_corrected_normalized": new_adv, "bootstrap_value_before": np.array(values[boundary]),
               "last_reward": np.array(last_reward), "bootstrap_batch_row": np.array(boundary)}
    np.savez_compressed(output / "fragment_boundary.npz", **payload)
    metadata = {"original_episode": selected, "first_shown_batch_row": int(start),
                "last_real_batch_row": int(boundary - 1), "bootstrap_batch_row": int(boundary),
                "bootstrap_value_before": float(values[boundary]), "last_reward": last_reward,
                "last_target_stock": float(old_targets[-1]), "last_target_corrected": float(new_targets[-1]),
                "last_target_stock_equals_reward": bool(old_targets[-1] == last_reward),
                "advantage_convention": "Both normalized over all elongated rows; artificial stock targets retained in corrected branch",
                "scope": "Recorded learner batch, no new policy forward, optimizer update or rollout"}
    analysis.write_json(output / "fragment_boundary.json", metadata)
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 8), sharex=True, constrained_layout=True)
    axes[0].plot(np.r_[distance, 0], values[start:boundary + 1], color="#666666", ls="--", lw=1.8, label="V before update (including bootstrap)")
    axes[0].plot(distance, old_targets, color="#b4544e", lw=2.2, label="Stock value target")
    axes[0].plot(distance, new_targets, color="#247b95", lw=2.2, label="Corrected real-row target")
    axes[0].scatter([-1, -1], [old_targets[-1], new_targets[-1]], color=["#b4544e", "#247b95"], zorder=5)
    axes[0].annotate(f"Stock: {old_targets[-1]:.4f} = reward", xy=(-1, old_targets[-1]),
                     xytext=(-27, 4), arrowprops={"arrowstyle": "->", "color": "#b4544e"}, color="#9b3b36")
    axes[0].annotate(f"Corrected: {new_targets[-1]:.4f}\nreward + 0.99 × {values[boundary]:.4f}",
                     xy=(-1, new_targets[-1]), xytext=(-27, 31),
                     arrowprops={"arrowstyle": "->", "color": "#247b95"}, color="#17677d")
    axes[0].set(ylabel="Value / TD-lambda target", title="An unfinished sampling fragment loses its bootstrap in the installed stock pipeline")
    axes[0].legend(loc="upper left", fontsize=9)
    axes[1].plot(distance, old_adv, color="#b4544e", lw=2.2, label="Stock normalized advantage")
    axes[1].plot(distance, new_adv, color="#247b95", lw=2.2, label="Corrected normalized advantage")
    axes[1].axhline(0, color="#888888", lw=1)
    axes[1].set(ylabel="Normalized advantage", xlabel="Decision index relative to the sampling boundary (0 = next observation)")
    axes[1].legend(loc="lower left", fontsize=9)
    for ax in axes:
        ax.axvline(0, color="#555555", ls=":", lw=1.3)
        ax.grid(alpha=.2)
        ax.set_xlim(distance[0] - 1, 1)
    fig.suptitle("First open fragment: last 50 recorded decisions\nSame rewards, observations and V estimates; only real-row boundary targets differ")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(fig)
    return metadata


def report(metrics, table):
    pending = [row["case"] for row in metrics["heldout"] if row.get("pending")]
    available = [row for row in metrics["heldout"] if row.get("available")]
    unavailable = [row["case"] for row in metrics["heldout"] if not row.get("available") and not row.get("pending")]
    correction = metrics["boundary_correction"]
    lines = ["# A/B sullo stesso batch: target stock e correzione boundary", "",
             "Il controllo riproduce esattamente l'update originale: pesi actor/critic, Adam, scalari e input/ordine/forward di tutti gli 81 minibatch. Il ramo corretto riparte dallo stesso stato e dagli stessi episodi, senza raccogliere nuovi dati. Cambiano soltanto i target TD-lambda reali e le advantages che ne derivano; target artificiali e convenzione di normalizzazione sono preservati.", "",
             f"La correzione cambia il segno delle advantages normalizzate nel {100*correction['real_advantage_sign_change_fraction']:.3f}% delle 4096 transizioni. Non significa che ogni target corrispondente sia gravemente errato: la centratura e la scala cambiano anche sulle righe lontane dal confine. La quota dei target stock distanti oltre 1 dal target corretto è 9,5215% nel batch originale.", "",
             "## Griglia a target fisso", "",
             "Ogni coppia target/advantages è congelata. Confrontare **before, control e corrected dentro la medesima coppia**; le loss di righe appartenenti a target diversi non misurano la stessa funzione obiettivo.", "",
             "| Target | Policy/critic | VF MSE raw | VF capped | EV | Policy loss | KL |", "|---|---|---:|---:|---:|---:|---:|"]
    for row in table:
        ev = row["vf_explained_variance"]
        ev_text = f"{ev:.4f}" if ev is not None else "n/d"
        lines.append(f"| {row['target']} | {row['policy']} | {row['vf_mse_raw']:.5f} | {row['vf_mse_capped']:.5f} | {ev_text} | {row['policy_loss']:.6f} | {row['kl_mean']:.6f} |")
    lines += ["", "La tabella verifica adattamento al segnale di training e mostra se l'update stock o corretto ottimizza meglio ciascun segnale. Non misura ancora la qualità delle traiettorie né attribuisce tutta la difficoltà LEFT a questo errore.", "",
              "## Critic su traiettorie separate dal batch", "",
              f"Casi con osservazioni complete disponibili: {len(available)}. Casi ancora in attesa di receipt: {len(pending)}. Casi completati senza osservazioni complete del critic: {len(unavailable)}.", "",
              "Tutti e tre i critic sono valutati sulle stesse osservazioni all'interno di ciascun rollout. Solo una vera terminazione rende il ritorno osservato fino alla fine un campione Monte Carlo dell'episodio della policy generatrice. Per truncation o fine della registrazione sono distinti il ritorno finito senza coda e il ritorno con bootstrap del critic before: sono convenzioni esplicite, non verità indipendenti sul valore continuativo. Il target TD a un passo usa anch'esso V_before fisso, mentre la residual TD con il valore di ciascun critic è soltanto diagnostica.", "",
              "**Limite fuori-policy:** azioni, reward e ritorni appartengono al comportamento che ha generato il rollout (before, control, corrected o first_pass). Il nominale usa azioni deterministiche: anche quando il checkpoint coincide, non rappresenta il rendimento atteso della policy stocastica appresa dal critic. I campi `critic_checkpoint_matches_generator_checkpoint` e `own_stochastic_behavior` distinguono le due condizioni; il secondo vale soltanto per un rollout noisy del medesimo checkpoint. I confronti degli altri critic sullo stesso esito sono fuori-policy: neppure un ritorno terminale è una verità comune per tre policy diverse. Questi punteggi descrivono gli scarti sul medesimo dato, senza certificare l'accuratezza di ogni V rispetto al rendimento atteso della propria policy o un ranking assoluto dei critic. Le traiettorie sono separate dal batch di training; le transizioni successive sono correlate e il loro numero non equivale al numero di repliche statisticamente indipendenti."]
    if available:
        lines += ["", "| Rollout | Termine | Critic | RMSE su ritorno osservato | RMSE con bootstrap before |", "|---|---|---|---:|---:|"]
        for row in available:
            ending = "terminale / MC" if row["terminal_return_observed"] else "orizzonte finito"
            for name, values in row["snapshots"].items():
                lines.append(f"| {row['case']} | {ending} | {name} | {values['vs_finite_horizon_observed_rewards']['rmse']:.4f} | {values['vs_fixed_before_bootstrap_return']['rmse']:.4f} |")
    if pending:
        lines += ["", "In attesa: " + ", ".join(pending) + "."]
    if unavailable:
        lines += ["", "Osservazioni critic assenti: " + ", ".join(unavailable) + "."]
    lines += ["", "Sono confronti su un batch e pochi rollout, non una dimostrazione di convergenza, stabilità globale o sufficienza della correzione. Nessun codice di produzione o ambiente Python globale è stato modificato; questa correzione isolata non è un backport completo della modifica upstream.", "",
              "Provenienza: `input_hashes.json`; verifiche: `verification.json`; dati: `metrics.json`, `cross_target_metrics.csv`, `heldout_critic_metrics.csv`. Figura nella cartella root dei plot del learner."]
    return "\n".join(lines) + "\n"


def run(source_probe, replay, output, manifests):
    from ray.rllib.core.rl_module.rl_module import RLModule
    source_probe, replay, output = (Path(path).resolve() for path in (source_probe, replay, output))
    replay_receipt = json.loads((replay / "replay_complete.json").read_text())
    if not replay_receipt.get("complete") or not replay_receipt["control_reproduction"]["pass"]:
        raise ValueError("Exact replay gate has not passed")
    source_manifest, inputs = verify_probe(source_probe)
    corrected_probe = replay / "corrected_boundary/ppo_audit"
    corrected_manifest, corrected_inputs = verify_probe(corrected_probe)
    inputs.update(corrected_inputs)
    for path in (replay / "replay_complete.json", replay / "control_verification.json", replay / "corrected_boundary/boundary_correction.json",
                 Path(__file__), HERE / "learner_analysis.py", HERE / "replay_batch.py"):
        inputs[str(path)] = analysis.digest(path)
    config_path = source_probe.parent / "effective_rllib_configuration.json"
    config = json.loads(config_path.read_text())
    inputs[str(config_path)] = analysis.digest(config_path)
    batches = {"stock": analysis.load_pickle(source_probe / "batch_post_gae.pkl")["default_policy"],
               "corrected_real_boundary": analysis.load_pickle(corrected_probe / "batch_post_gae.pkl")["default_policy"]}
    observations = analysis.array(batches["stock"]["obs"])
    mask = analysis.array(batches["stock"]["loss_mask"]).astype(bool)
    for column in ("obs", "actions", "action_logp", "action_dist_inputs", "loss_mask", "rewards"):
        if not np.array_equal(analysis.array(batches["stock"][column]), analysis.array(batches["corrected_real_boundary"][column])):
            raise ValueError(f"Non-target input differs: {column}")
    paths = {"before": source_probe / "snapshots/before/rl_module_default_policy",
             "control": source_probe / "snapshots/final/rl_module_default_policy",
             "corrected": corrected_probe / "snapshots/final/rl_module_default_policy"}
    modules, predictions = {}, {}
    torch.set_num_threads(1)
    for name, path in paths.items():
        module = RLModule.from_checkpoint(str(path))
        module.eval()
        modules[name] = module
        predictions[name] = analysis.infer(module, observations)
    scalars = source_manifest["snapshots"]["before"]["scalars"]
    comparison, table = {}, []
    for target, batch in batches.items():
        comparison[target] = {}
        for name, module in modules.items():
            logits, values = predictions[name]
            metrics, _ = analysis.snapshot_metrics(module, logits, values, batch, mask, config,
                                                  kl_coeff=scalars["kl_coeff"]["default_policy"],
                                                  entropy_coeff=scalars["entropy_coeff"]["default_policy"])
            comparison[target][name] = metrics
            vf = metrics["critic_fixed_gae_targets"]
            table.append({"target": target, "policy": name, "vf_mse_raw": vf["squared_error"]["mean"],
                          "vf_mse_capped": vf["capped_mse"], "vf_cap_fraction": vf["cap_fraction"],
                          "vf_explained_variance": vf["explained_variance"], "policy_loss": metrics["policy_loss_fixed_advantages"],
                          "kl_mean": metrics["kl_old_to_current"]["mean"],
                          "surrogate_clipped_fraction": metrics["surrogate_actually_clipped_fraction"],
                          "advantage_delta_logp_mean": metrics["advantage_probability_alignment"]["mean_advantage_times_delta_logp"]})
    cases, manifest_inputs = evaluation_cases(manifests)
    heldout, heldout_inputs = heldout_comparison(cases, modules, config)
    inputs.update(manifest_inputs)
    inputs.update(heldout_inputs)
    mean_delta = predictions["corrected"][0][mask, :2] - predictions["control"][0][mask, :2]
    metrics = {"schema": "rl_learning_audit_cross_target_v1", "complete_offline_comparison": True,
               "all_requested_heldout_complete": not any(row.get("pending") for row in heldout),
               "control_bit_exact": True, "real_rows": int(mask.sum()), "target_grid": comparison,
               "boundary_correction": replay_receipt["corrected_boundary"]["boundary_correction"],
               "corrected_vs_control_mean_delta_rms_per_joint": np.sqrt(np.mean(mean_delta ** 2, axis=0)),
               "corrected_vs_control_mean_delta_max_abs_per_joint": np.max(np.abs(mean_delta), axis=0),
               "heldout": heldout, "new_optimizer_updates": 0, "new_sampling": False, "physical_simulations": False,
               "comparison_rule": "Compare policies within a fixed target/advantage set; cross-target loss magnitudes have different semantics"}
    output.mkdir(parents=True, exist_ok=True)
    analysis.write_json(output / "metrics.json", metrics)
    analysis.write_csv(output / "cross_target_metrics.csv", table)
    heldout_rows = []
    for dataset in heldout:
        if not dataset.get("available"):
            continue
        for name, data in dataset["snapshots"].items():
            for target, key in (("observed_terminal_return" if dataset["terminal_return_observed"] else "observed_finite_horizon_rewards", "vs_finite_horizon_observed_rewards"),
                                ("fixed_before_bootstrap_return", "vs_fixed_before_bootstrap_return"),
                                ("fixed_before_one_step_td_target", "vs_fixed_before_one_step_td_target")):
                row = data[key]
                heldout_rows.append({"case": dataset["case"], "critic": name, "target": target, "rows": dataset["rows"],
                                     "generator_policy": dataset["generator_policy"],
                                     "critic_checkpoint_matches_generator_checkpoint": dataset["critic_checkpoint_matches_generator_checkpoint"][name],
                                     "own_stochastic_behavior": dataset["own_stochastic_behavior"][name],
                                     "rmse": row["rmse"], "mse": row["squared_error"]["mean"],
                                     "explained_variance": row["explained_variance"], "bias": row["error"]["mean"]})
    analysis.write_csv(output / "heldout_critic_metrics.csv", heldout_rows)
    (output / "report.md").write_text(report(metrics, table))
    plot = analysis.DEFAULT_PLOTS / "04_cross_target_comparison.png"
    make_plot(plot, table)
    boundary_plot = analysis.DEFAULT_PLOTS / "05_fragment_boundary.png"
    fragment_boundary_plot(boundary_plot, output, source_probe, batches["stock"], batches["corrected_real_boundary"])
    analysis.write_json(output / "input_hashes.json", inputs)
    changed = [path for path, expected in inputs.items() if analysis.digest(path) != expected]
    if changed:
        raise RuntimeError(f"Comparison inputs changed: {changed}")
    outputs = [path for path in output.iterdir() if path.is_file() and path.name != "verification.json"] + [plot, boundary_plot]
    analysis.write_json(output / "verification.json", {
        "all_inputs_unchanged": True, "input_count": len(inputs), "probe_hashes_valid": True,
        "output_hashes": {str(path.resolve()): analysis.digest(path) for path in outputs},
        "pending_heldout_cases": [row["case"] for row in heldout if row.get("pending")],
        "no_new_optimizer_update": True})
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-probe", type=Path, default=analysis.DEFAULT_PROBE)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--evaluation-manifests", nargs="*", type=Path,
                        default=[HERE / "evaluation_manifest.json", HERE / "corrected_evaluation_manifest.json"])
    args = parser.parse_args()
    result = run(args.source_probe, args.replay, args.output, args.evaluation_manifests)
    print(json.dumps(analysis.clean({"complete_offline_comparison": True,
                                   "all_requested_heldout_complete": result["all_requested_heldout_complete"],
                                   "available_heldout": sum(bool(row.get("available")) for row in result["heldout"]),
                                   "pending_heldout": [row["case"] for row in result["heldout"] if row.get("pending")]}), indent=2))


if __name__ == "__main__":
    main()
