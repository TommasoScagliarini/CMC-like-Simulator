"""Frozen-trajectory reward sensitivity, never a prediction of policy learning."""
from __future__ import annotations

import csv
from dataclasses import replace
from pathlib import Path

import numpy as np

from unified_MLP_39.reward_function import RewardConfig, compute_reward
from unified_MLP_39.tuning_terms import TuningHistory, enrich_tuning_terms
from .io import relative, resolve, sha256, write_json
from .metrics import completed_row, load_dataset, measure

FAMILIES = {
    "ankle": ("ankle_positive_torque",),
    "knee": ("knee_torque_rate",),
    "kinematics": ("knee_kinematic_jerk", "ankle_kinematic_jerk"),
}


def enrich_trace(dataset, config):
    """Replay diagnostics at [segment start, segment end), as in step_until."""
    history = TuningHistory()
    first = dataset["trace"][0]
    history.reset(time_s=float(first["policy_segment_times"][0]),
                  kinematics=first.get("actor_observation_before", {}))
    torque = dataset["tables"].get("sea_torques")
    enriched, errors = [], []
    for row in dataset["trace"]:
        terms = dict(row["reward_terms"])
        complete = completed_row(row)
        if complete and torque is not None:
            lo, hi = row["policy_segment_times"][0], row["policy_segment_times"][-1]
            mask = (torque["time"] >= lo - 1e-7) & (torque["time"] < hi - 1e-7)
            samples = torque["SEA_Ankle_tau_spring"][mask]
            expected = terms.get("pros_ankle_angle_sea_sample_count")
            if not len(samples) or (expected is not None and len(samples) != int(expected)):
                raise ValueError(f"{dataset['id']}: STO does not cover segment at {lo}: {len(samples)} vs {expected}")
            old_rms = terms.get("pros_ankle_angle_sea_tau_spring_rms_nm")
            if old_rms is not None:
                error = abs(float(np.sqrt(np.mean(samples ** 2))) - old_rms)
                errors.append(error)
                if error > 1e-5:
                    raise ValueError(f"{dataset['id']}: STO/online SEA mismatch {error}")
            positive = np.maximum(samples, 0)
            prefix = "pros_ankle_angle_sea_tau_spring_positive_"
            terms.update({prefix + "sum_nm": float(positive.sum()),
                          prefix + "mean_nm": float(positive.mean()),
                          prefix + "max_nm": float(positive.max()),
                          prefix + "sample_count": float(np.count_nonzero(samples > 0))})
        terms.update(enrich_tuning_terms(terms, config, time_s=float(row["time"]),
                                        kinematics=row.get("prosthetic_state", {}),
                                        history=history, segment_completed=complete))
        terms["tuning_segment_completed"] = float(complete)
        enriched.append(terms)
    return enriched, max(errors) if errors else None


def source_replay(dataset):
    cfg = RewardConfig.from_mapping(dataset["summary"]["reward_config"])
    pairs = [compute_reward(r["reward_terms"], cfg, reference=r["policy_segment_values"])
             for r in dataset["trace"]]
    delta = float(sum(r for r, _ in pairs) - dataset["summary"]["episode_return"])
    if abs(delta) > 1e-5:
        raise ValueError(f"{dataset['id']}: original return replay failed: {delta}")
    return cfg, pairs, delta


def family_available(prefixes, terms):
    # Warmup/incomplete segments deliberately contribute no motion penalty;
    # absent physical inputs on a completed usable segment do not.
    return all(all(float(t.get(p + "_available", 0)) > 0 or
                       float(t.get(p + "_warmup", 0)) > 0 or
                       float(t.get("tuning_segment_completed", 1)) == 0
                       for p in prefixes) for t in terms)


def analyze(config, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    datasets, internal, hashes = {}, {}, {}
    for entry in config["corpus"]:
        path = resolve(entry["path"])
        dataset = load_dataset(path, identity=entry["id"], group=entry.get("group", "left"),
                               scenario=entry.get("scenario", "nominal"))
        source_cfg, replay, discrepancy = source_replay(dataset)
        enriched, segment_error = enrich_trace(dataset, source_cfg)
        internal[entry["id"]] = (dataset, source_cfg, enriched, replay)
        hashes.update(dataset["input_hashes"])
        datasets[entry["id"]] = {
            "path": dataset["path"], "group": dataset["group"], "scenario": dataset["scenario"],
            "source_reward_config": dataset["summary"]["reward_config"],
            "replay_return_error": discrepancy, "sea_segment_rms_replay_max_error_nm": segment_error,
            "missing_files": dataset["missing_files"], "full": measure(dataset),
            "window": measure(dataset, config.get("analysis_window_s", [13.15, 17.95])),
            "sensitivities": {},
        }
    reference_id = config.get("reference_id", "best80")
    if reference_id not in internal:
        raise ValueError("Reference dataset absent from corpus")
    reference, ref_cfg, ref_terms, ref_pairs = internal[reference_id]
    positive_score = float(sum(ref_cfg.blend_served_imitation * c["served_imitation_score"] +
                               ref_cfg.blend_imitation * c["imitation_score"] for _, c in ref_pairs))
    if not np.isfinite(positive_score) or positive_score <= 0:
        raise ValueError("No positive imitation score for calibration")
    candidates = {}
    for family, prefixes in FAMILIES.items():
        if not family_available(prefixes, ref_terms):
            raise ValueError(f"Reference lacks required diagnostics for {family}")
        totals = {p: sum(t[p + "_loss"] for t in ref_terms) for p in prefixes}
        if any(not np.isfinite(v) or v <= 0 for v in totals.values()):
            raise ValueError(f"Cannot calibrate zero/nonfinite {family} loss")
        for fraction in config.get("intensities", [.05, .10, .20]):
            if not 0 < fraction < 1:
                raise ValueError("Calibration fractions must be between zero and one")
            key = f"{family}_{round(100 * fraction)}"
            overrides = {p + "_weight": positive_score * fraction / len(prefixes) / totals[p]
                         for p in prefixes}
            candidates[key] = {"family": family, "fraction_of_source_positive_score": fraction,
                               "calibration_score": positive_score, "overrides": overrides,
                               "source_loss_totals": totals, "learned_effect": "not_measured"}
    csv_rows = []
    for identity, (dataset, source_cfg, terms, original_pairs) in internal.items():
        original = np.asarray([r for r, _ in original_pairs])
        for name, candidate in candidates.items():
            prefixes = FAMILIES[candidate["family"]]
            if not family_available(prefixes, terms):
                datasets[identity]["sensitivities"][name] = {
                    "available": False, "reason": "Missing physical diagnostics; no zero imputation"}
                continue
            cfg = replace(source_cfg, **candidate["overrides"])
            counterfactual = np.asarray([compute_reward(t, cfg, reference=r["policy_segment_values"])[0]
                                         for t, r in zip(terms, dataset["trace"])])
            difference = original - counterfactual
            phase_cost = []
            phase = np.asarray([r["reward_terms"].get("pros_ankle_angle_imitation_target_phase", np.nan)
                                for r in dataset["trace"]])
            for b in range(10):
                pm = (phase >= b / 10) & (phase < (b + 1) / 10)
                phase_cost.append({"bin": b, "samples": int(pm.sum()), "added_cost": float(difference[pm].sum())})
            result = {"available": True, "source_return": float(original.sum()),
                      "return_on_frozen_trace": float(counterfactual.sum()),
                      "added_cost": float(difference.sum()), "changed_steps": int(np.count_nonzero(abs(difference) > 1e-10)),
                      "cost_by_prescribed_phase": phase_cost}
            datasets[identity]["sensitivities"][name] = result
            csv_rows.append({"dataset": identity, "group": dataset["group"], "candidate": name,
                             **{k: v for k, v in result.items() if k != "cost_by_prescribed_phase"}})
    hashes[relative(config["_config_path"])] = sha256(config["_config_path"])
    result = {"schema": "reward_tuning_analysis_v1", "reference_id": reference_id,
              "scope": "Frozen trajectory arithmetic; learning effects not measured",
              "priorities": ["ankle_positive_area", "knee_torque_regularity", "kinematic_regularity"],
              "scales": {"ankle_positive_torque_scale_nm": ref_cfg.ankle_positive_torque_scale_nm,
                         "knee_torque_rate_scale_nm_s": ref_cfg.knee_torque_rate_scale_nm_s,
                         "kinematic_jerk_scale_rad_s3": ref_cfg.kinematic_jerk_scale_rad_s3},
              "input_hashes": hashes, "candidates": candidates, "datasets": datasets}
    write_json(output / "analysis.json", result)
    with (output / "sensitivities.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
    write_json(output / "input_hashes.json", hashes)
    _report(result, output)
    _plot(result, resolve(config["plot_dir"]))
    return result


def _report(result, output):
    lines = ["# Banco offline di tuning della reward LEFT", "",
             "Analisi di traiettorie congelate: i costi cambiano, i comportamenti restano quelli registrati. "
             "Nessun risultato di apprendimento è stato misurato e nessun training è stato avviato.", "",
             "## Confronto fisico nella finestra comune", "",
             "| Dataset | Tracking knee/ankle [°] | Area positiva ankle [Nm·s] | Knee HP10/20 [Nm] | Completo / cicli |",
             "|---|---:|---:|---:|---|"]
    def fmt(value):
        return "N/D" if value is None else f"{value:.3f}"
    for identity, d in result["datasets"].items():
        m = d["window"]
        ankle, knee = m["ankle"] or {}, m["knee"] or {}
        tr = m["tracking"]
        label = identity + (" (descrittivo)" if d["group"] != "left" else "")
        lines.append(f"| {label} | {fmt(tr['knee']['actual_rmse_deg'])} / {fmt(tr['ankle']['actual_rmse_deg'])} | "
                     f"{fmt(ankle.get('positive_area_nm_s'))} | {fmt(knee.get('hp10_rms_nm'))} / {fmt(knee.get('hp20_rms_nm'))} | "
                     f"{m['episode']['completed']} / {m['episode']['valid_cycles']} |")
    lines += ["", "I fallimenti hanno finestre più corte: le aree non sono confrontabili con episodi completi. "
              "Il comando compare ricalcola i prefissi comuni e mantiene separato il giudizio di completamento.", "",
              "## Candidate calibrate sul best80", "",
              "| Candidata | Quota score positivo | Pesi |", "|---|---:|---|"]
    for name, c in result["candidates"].items():
        weights = "; ".join(f"{k}={v:.8g}" for k, v in c["overrides"].items())
        lines.append(f"| {name} | {100*c['fraction_of_source_positive_score']:.0f}% | {weights} |")
    lines += ["", "Il pilot predisposto usa soltanto ankle_10 contro reward invariata. "
              "Questa scelta fissa un'intensità iniziale, non identifica un peso ottimale.", "",
              "## Copertura e verifiche", ""]
    for identity, d in result["datasets"].items():
        missing = ", ".join(Path(p).name for p in d["missing_files"]) or "nessuno"
        lines.append(f"- {identity}: errore replay {d['replay_return_error']:.3g}; file mancanti: {missing}.")
    lines += ["", "Area ankle e metriche spettrali knee richiedono STO ad alta frequenza. La sensibilità del costo knee usa "
              "gli RMS per segmento presenti nei trace. Le famiglie prive dei dati necessari "
              "sono indisponibili, mai valutate a costo zero. Il jerk è una differenza causale delle velocità a 100 Hz, "
              "con warm-up dichiarato. Il rateo knee della reward esclude i confini fra segmenti come la diagnostica "
              "originale; la metrica fisica dell'intero STO include anche quei confini.", "",
              "I ritorni storici usano la propria configurazione originale. V26/RIGHT restano descrittivi. "
              "Le scansioni non simulano PPO né prevedono il comportamento successivo.", "",
              "## Prossimo passo predisposto", "",
              "Un aggiornamento del solo critic e tre PPO per ramo, con prescribed visibili. "
              "Nominale più seed 123/125/127; sigma di best80 congelata per indice decisionale. "
              "Il pilot richiede il comando run esplicito. La verifica live Windows resta separata.", ""]
    (Path(output) / "report.md").write_text("\n".join(lines), encoding="utf-8")


def _plot(result, directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    directory.mkdir(parents=True, exist_ok=True)
    names = [k for k, d in result["datasets"].items() if d["group"] == "left" and
             d["scenario"] == "nominal" and d["window"]["ankle"] is not None]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), layout="constrained")
    for ax, label, getter in (
        (axes[0], "Area positiva ankle [Nm·s]", lambda m: m["ankle"]["positive_area_nm_s"]),
        (axes[1], "Coppia knee HP10 RMS [Nm]", lambda m: m["knee"]["hp10_rms_nm"]),
        (axes[2], "Tracking ankle RMSE [°]", lambda m: m["tracking"]["ankle"]["actual_rmse_deg"]),
    ):
        ax.bar(names, [getter(result["datasets"][n]["window"]) for n in names])
        ax.set_ylabel(label)
        ax.tick_params(axis="x", rotation=40)
        ax.grid(axis="y", alpha=.2)
    fig.suptitle("Rollout esistenti — nessun effetto di nuovo training misurato")
    fig.savefig(directory / "offline_physical_comparison.png", dpi=160)
    plt.close(fig)
    ref = result["datasets"][result["reference_id"]]
    fig, ax = plt.subplots(figsize=(10, 5), layout="constrained")
    for family in FAMILIES:
        name = family + "_10"
        values = ref["sensitivities"][name]["cost_by_prescribed_phase"]
        ax.plot([v["bin"] / 10 + .05 for v in values], [v["added_cost"] for v in values],
                marker="o", label=name)
    ax.set(xlabel="Fase prescribed", ylabel="Costo aggiuntivo sui trace best80",
           title="Stesso costo totale, diversa distribuzione temporale")
    ax.legend(); ax.grid(alpha=.2)
    fig.savefig(directory / "offline_reward_sensitivity.png", dpi=160)
    plt.close(fig)
