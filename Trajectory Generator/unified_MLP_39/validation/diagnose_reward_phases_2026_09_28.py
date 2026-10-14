"""Offline phase/event diagnosis of frozen LEFT pilots; never starts simulation.

Use --rollout LABEL=PATH repeatedly to diagnose additional saved rollouts. All
high-rate samples belong to their recorded pre-integration [start, end) segment;
the prescribed clock at its start is held without looking at a future phase.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
TG = PACKAGE.parent
REPO = TG.parent
sys.path.insert(0, str(TG))
from unified_MLP_39.reward_tuning.metrics import completed_row, read_table

DEFAULT_OUTPUT = PACKAGE / "runs/reward_tuning/left_pilot_diagnosis_2026-09-28/phase_analysis"
DEFAULT_PLOTS = REPO / "plot/09_28_2026_LEFT_pilot_diagnosis/phase_analysis"
DEFAULT_ROLLOUTS = {
    "best80": "left_best80_2026-09-25_best80_nominal",
    "control84": "left_best80_2026-09-25_control_nominal",
    "ankle5_84": "left_best80_ankle5_2026-09-25_ankle_5_nominal",
    "ankle10_84": "left_best80_2026-09-25_ankle_10_nominal",
}
HASHES = {}


def relative(path):
    path = Path(path).resolve()
    return str(path.relative_to(REPO)) if path.is_relative_to(REPO) else str(path)


def record_hash(path):
    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    HASHES[relative(path)] = digest
    return digest


def load_json(path):
    record_hash(path)
    return json.loads(Path(path).read_text(encoding="utf-8"))


def finite(values, name):
    values = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Missing/nonfinite {name}")
    return values


def segment_membership(times, starts, ends):
    """Match STO's eight-decimal time precision; never clip an unmatched sample."""
    times, starts, ends = [np.round(finite(x, "segment time"), 8) for x in (times, starts, ends)]
    if (np.any(np.diff(times) <= 0) or np.any(np.diff(starts) <= 0)
            or np.any(ends <= starts) or np.any(starts[1:] < ends[:-1])):
        raise ValueError("Unordered times or overlapping segments")
    index = np.searchsorted(starts, times, side="right") - 1
    if np.any(index < 0) or np.any(times >= ends[np.maximum(index, 0)]):
        raise ValueError("High-rate sample has no completed [start,end) segment")
    return index


def interval_weights(times, dt, lo, hi):
    """Constant pre-integration samples, clipped to the requested half-open window."""
    return np.maximum(0.0, np.minimum(times + dt, hi) - np.maximum(times, lo))


def integrate(values, weights):
    return float(np.dot(values, weights))


def rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def load_rollout(label, path):
    path = Path(path).resolve()
    summary = load_json(path / "rollout_summary.json")
    raw = load_json(path / "rollout_policy_trace.json")
    receipt = load_json(path / "evaluation_complete.json")
    for name, digest in receipt["artifact_hashes"].items():
        if record_hash(path / name) != digest:
            raise ValueError(f"Evaluation artifact changed: {path / name}")
    rows = [r for r in raw if completed_row(r)]
    if len(rows) != len(raw):
        raise ValueError("This nominal diagnostic requires complete segments; inspect failures separately")
    starts = finite([r["policy_segment_times"][0] for r in rows], "starts")
    ends = finite([r["policy_segment_times"][-1] for r in rows], "ends")
    if not np.allclose(starts[1:], ends[:-1], atol=1e-8, rtol=0):
        raise ValueError("Policy trace has gaps")
    tables = {}
    for name in ("sea_torques", "online_grf", "kinematics", "kinematics_reference"):
        file = path / f"sim_outputs/rollout_episode_{name}.sto"
        record_hash(file)
        tables[name] = read_table(file)
    times = tables["sea_torques"]["time"]
    dt = float(np.median(np.diff(times)))
    if not np.allclose(np.diff(times), dt, atol=1e-8, rtol=0):
        raise ValueError("Irregular high-rate samples")
    for name, table in tables.items():
        if table["time"].shape != times.shape or not np.allclose(table["time"], times, atol=1e-8, rtol=0):
            raise ValueError(f"Asynchronous {name}: no interpolation is used")
    idx = segment_membership(times, starts, ends)
    obs = [r["actor_observation_before"] for r in rows]
    sin = finite([r["gait_phase_sin"] for r in obs], "prescribed sin")
    cos = finite([r["gait_phase_cos"] for r in obs], "prescribed cos")
    if not np.allclose(sin * sin + cos * cos, 1, atol=1e-6, rtol=0):
        raise ValueError("Masked/invalid prescribed clock; no phase fallback")
    phase_start = np.mod(np.arctan2(sin, cos) / (2 * np.pi), 1)
    phase_end = finite([r["reward_terms"]["pros_ankle_angle_imitation_target_phase"] for r in rows], "endpoint phase")
    knee_phase = finite([r["reward_terms"]["pros_knee_angle_imitation_target_phase"] for r in rows], "knee endpoint phase")
    if not np.allclose(phase_end, knee_phase, atol=1e-10):
        raise ValueError("LEFT knee and ankle prescribed phases differ")
    # Next segment's beginning is the same physical instant as this segment's
    # endpoint. This validates the clock without using that future phase for bins.
    circular = np.mod(phase_end[:-1] - phase_start[1:] + .5, 1) - .5
    if np.max(np.abs(circular)) > 1e-6:
        raise ValueError("Recorded actor clock and target phase disagree")
    torque = tables["sea_torques"]["SEA_Ankle_tau_spring"]
    positive = np.maximum(torque, 0)
    counts = np.bincount(idx, minlength=len(rows))
    sums = np.bincount(idx, weights=positive, minlength=len(rows))
    expected_counts = finite([r["reward_terms"]["pros_ankle_angle_sea_sample_count"] for r in rows], "SEA count")
    expected_sum = finite([r["reward_terms"]["pros_ankle_angle_sea_tau_spring_positive_sum_nm"] for r in rows], "positive SEA sum")
    if not np.array_equal(counts, expected_counts) or np.max(np.abs(sums - expected_sum)) > 1e-6:
        raise ValueError("Offline [start,end) SEA statistics do not match recorded online statistics")
    force = tables["online_grf"]["left_normal_force"]
    before_fsm = finite([r["phase_fsm_swing_after_to"] for r in obs], "before FSM swing")
    result = dict(label=label, path=relative(path), summary=summary, rows=rows,
                  tables=tables, starts=starts, ends=ends, time=times, dt=dt,
                  sample_segment=idx, phase_start=phase_start, phase_end=phase_end,
                  phase_bin=np.floor(phase_start[idx] * 10).astype(int),
                  positive=positive, torque=torque, force=force,
                  swing=before_fsm[idx] == 1,
                  reconciliation={"segments": len(rows), "samples": len(times),
                      "sample_hz": 1 / dt, "counts_match": True,
                      "max_positive_sum_error_nm": float(np.max(np.abs(sums - expected_sum))),
                      "positive_area_from_trace_nm_s": float(expected_sum.sum() * dt),
                      "positive_area_from_sto_nm_s": float(positive.sum() * dt),
                      "clock_endpoint_start_max_error_cycles": float(np.max(np.abs(circular)))})
    return result


def physical_metrics(data, lo, hi, selection=None):
    available_lo = float(data["time"][0])
    available_hi = float(data["time"][-1] + data["dt"])
    covered_lo, covered_hi = max(lo, available_lo), min(hi, available_hi)
    coverage = {"requested_window_s": [lo, hi],
                "covered_window_s": [covered_lo, covered_hi] if covered_hi > covered_lo else None,
                "requested_duration_s": hi - lo,
                "covered_window_duration_s": max(0.0, covered_hi - covered_lo),
                "window_fully_covered": lo >= available_lo - 1e-8 and hi <= available_hi + 1e-8}
    weights = interval_weights(data["time"], data["dt"], lo, hi)
    if selection is not None:
        weights = weights * selection
    mask = weights > 1e-10
    force, positive, torque = data["force"], data["positive"], data["torque"]
    if not np.any(mask):
        return {**coverage, "samples": 0, "duration_s": 0.0, "positive_area_nm_s": None,
                "vertical_impulse_n_s": None, "reason": "no samples in window/bin"}
    return {**coverage, "samples": int(mask.sum()), "duration_s": float(weights.sum()),
            "positive_area_nm_s": integrate(positive, weights),
            "vertical_impulse_n_s": integrate(force, weights),
            "positive_loaded_area_nm_s": integrate(positive * (force >= 50), weights),
            "positive_unloaded_area_nm_s": integrate(positive * (force < 50), weights),
            "positive_fsm_swing_area_nm_s": integrate(positive * data["swing"], weights),
            "positive_duration_s": integrate((torque > 0).astype(float), weights),
            "loaded_duration_s": integrate((force >= 50).astype(float), weights),
            "positive_peak_nm": float(positive[mask].max()),
            "force_peak_n": float(force[mask].max()),
            "force_mean_n": integrate(force, weights) / float(weights.sum()),
            "penetration_max_mm": float(data["tables"]["online_grf"]["left_penetration"][mask].max() * 1000)}


def kinematics_window(data, lo, hi):
    mask = (data["time"] >= lo - 1e-8) & (data["time"] < hi - 1e-8)
    result = {}
    for joint in ("knee", "ankle"):
        coord = f"pros_{joint}_angle"
        for kind, name in (("actual", "kinematics"), ("served", "kinematics_reference")):
            column = coord if kind == "actual" else coord + "_q_ref"
            values = np.rad2deg(data["tables"][name][column][mask])
            result[f"{joint}_{kind}_min_deg"] = float(values.min()) if len(values) else None
            result[f"{joint}_{kind}_max_deg"] = float(values.max()) if len(values) else None
            result[f"{joint}_{kind}_excursion_deg"] = float(np.ptp(values)) if len(values) else None
    return result


def phase_analysis(data):
    records, totals = [], {}
    windows = {"full": (float(data["starts"][0]), float(data["ends"][-1])),
               "historical": (13.15, 17.95)}
    for window, (lo, hi) in windows.items():
        if lo < data["starts"][0] - 1e-8 or hi > data["ends"][-1] + 1e-8:
            raise ValueError(f"{window} not fully covered")
        total = physical_metrics(data, lo, hi)
        total["window_s"] = [lo, hi]
        if window == "historical":
            # The older comparison included the sample at its upper bound;
            # report this explicitly instead of disguising a one-sample change.
            inclusive = (data["time"] >= lo - 1e-8) & (data["time"] <= hi + 1e-8)
            total["legacy_inclusive_positive_area_nm_s"] = float(data["positive"][inclusive].sum() * data["dt"])
            total["legacy_inclusive_vertical_impulse_n_s"] = float(data["force"][inclusive].sum() * data["dt"])
        totals[window] = total
        window_records = []
        for phase in range(10):
            values = physical_metrics(data, lo, hi, data["phase_bin"] == phase)
            row = {"policy": data["label"], "window": window, "phase_bin": phase,
                   "phase_low": phase / 10, "phase_high": (phase + 1) / 10, **values}
            selected = [r for r, ep in zip(data["rows"], data["phase_end"])
                        if lo - 1e-8 <= r["time"] < hi - 1e-8 and phase / 10 <= ep < (phase + 1) / 10]
            row["tracking_endpoint_samples"] = len(selected)
            for joint in ("knee", "ankle"):
                coord = f"pros_{joint}_angle"
                for kind, suffix in (("actual", ""), ("served", "_served_ref")):
                    error = [math.degrees(r["prosthetic_state"][coord + suffix] - r["imitation_target_q"][coord]) for r in selected]
                    row[f"{joint}_{kind}_rmse_deg"] = rms(error)
            records.append(row)
            window_records.append(row)
        for key in ("positive_area_nm_s", "vertical_impulse_n_s", "duration_s",
                    "positive_loaded_area_nm_s", "positive_unloaded_area_nm_s",
                    "positive_fsm_swing_area_nm_s", "positive_duration_s", "loaded_duration_s"):
            if any(r.get(key) is None for r in window_records):
                raise ValueError(f"Missing phase data for {key}")
            if not np.isclose(sum(r[key] for r in window_records), total[key], atol=1e-8, rtol=1e-10):
                raise AssertionError(f"Phase sum does not reconcile: {key}")
        total["phase_totals_reconcile"] = True
    return records, totals


def event_analysis(data):
    events, cycles = [], []
    counters = ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count",
                "resync_count", "hs_cancelled_count")
    for row in data["rows"]:
        fsm = row["phase_fsm"]
        for transition in fsm["accepted_transitions_this_step"]:
            item = {"policy": data["label"], "kind": "accepted_transition",
                    "recorded_boundary_time_s": row["time"], **transition,
                    "valid_cycle_count_after": fsm["valid_cycle_count"],
                    "prescribed_phase_at_delivery": row["reward_terms"]["pros_ankle_angle_imitation_target_phase"]}
            if transition["segment_start_time_s"] is not None:
                start, end = transition["segment_start_time_s"], transition["event_time_s"]
                if start >= 0 and end > start:
                    item["closed_segment_duration_s"] = end - start
                    item["physical_closed_segment"] = physical_metrics(data, start, end)
                    item["kinematics_closed_segment"] = kinematics_window(data, start, end)
            if "resync" in transition["event"]:
                item["primary_grf_during_resync_dwell"] = physical_metrics(data, transition["event_time_s"], transition["delivered_time_s"])
            events.append(item)
        if fsm["invalid_event_this_step"]:
            events.append({"policy": data["label"], "kind": "invalid_event",
                           "recorded_boundary_time_s": row["time"],
                           "event_time_s": None, "event": fsm["invalid_event_type"],
                           "cycle_reject_reason": fsm["cycle_reject_reason"],
                           "last_valid_to_time_s": fsm["last_valid_to_time_s"],
                           "boundary_minus_last_to_s": row["time"] - fsm["last_valid_to_time_s"],
                           "valid_cycle_count_after": fsm["valid_cycle_count"],
                           "note": "Rejected raw detector timestamp not retained in this trace; boundary is not the event onset."})
        if fsm["cycle_completed_this_step"]:
            end, period = fsm["last_valid_hs_time_s"], fsm["last_period_s"]
            cycles.append({"start_hs_s": end - period, "end_hs_s": end, "period_s": period,
                           "stance_fraction": fsm["last_stance_fraction"],
                           "physical": physical_metrics(data, end - period, end),
                           "kinematics": kinematics_window(data, end - period, end)})
    final = {key: data["rows"][-1]["phase_fsm"][key] for key in counters}
    if len(cycles) != final["valid_cycle_count"] or final["valid_cycle_count"] != data["summary"]["phase_valid_cycle_count"]:
        raise AssertionError("Accepted cycle events do not reconcile with summary")
    # Wrap boundaries have at most one policy interval of quantization and are
    # diagnostic common windows, not asserted physical heel strikes.
    wraps = (np.flatnonzero(np.diff(data["phase_start"]) < -.5) + 1).tolist()
    prescribed_cycles, prescribed_windows = [], []
    for number, (first, last) in enumerate(zip(wraps[:-1], wraps[1:]), 1):
        lo, hi = float(data["starts"][first]), float(data["starts"][last])
        prescribed_cycles.append({"cycle": number, "window_s": [lo, hi],
                                  **physical_metrics(data, lo, hi), **kinematics_window(data, lo, hi)})
    boundaries = [float(data["starts"][0]), *[float(data["starts"][index]) for index in wraps], float(data["ends"][-1])]
    for number, (lo, hi) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        prescribed_windows.append({"window_index": number, "complete_prescribed_cycle": 0 < number < len(boundaries) - 2,
                                   "window_s": [lo, hi], **physical_metrics(data, lo, hi),
                                   **kinematics_window(data, lo, hi)})
    for key in ("positive_area_nm_s", "vertical_impulse_n_s", "duration_s"):
        total = physical_metrics(data, boundaries[0], boundaries[-1])[key]
        if not np.isclose(sum(r[key] for r in prescribed_windows), total, atol=1e-8, rtol=1e-10):
            raise AssertionError(f"Prescribed windows do not reconcile: {key}")
    return {"final_counters": final, "events": events, "valid_cycles": cycles,
            "prescribed_complete_cycle_windows": prescribed_cycles,
            "all_prescribed_windows": prescribed_windows}


def save_csv(path, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, sort_keys=True) if isinstance(v, (dict, list)) else v for k, v in row.items()})


def plots(data, phase_rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    path.mkdir(parents=True, exist_ok=True)
    figures = []
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    colors = ["#333333", "#778899", "#137f6c", "#c06435"]
    for i, (label, item) in enumerate(data.items()):
        records = [r for r in phase_rows if r["policy"] == label and r["window"] == "historical"]
        x = np.arange(10) + (i - (len(data) - 1) / 2) * .18
        for ax, key in zip(axes, ("positive_area_nm_s", "vertical_impulse_n_s")):
            ax.bar(x, [r[key] for r in records], width=.17, label=label, color=colors[i % 4])
    axes[0].set(ylabel="Area ankle+ [Nm·s]", title="Finestra [13.15, 17.95) s — fase prescribed mantenuta dal segmento corrente")
    axes[1].set(ylabel="Impulso verticale [N·s]", xlabel="Decile della fase prescribed", xticks=np.arange(10), xticklabels=[f"{i/10:.1f}–{(i+1)/10:.1f}" for i in range(10)])
    axes[0].legend(ncol=len(data), fontsize=9)
    fig.tight_layout()
    dest = path / "area_and_support_by_prescribed_phase.png"
    fig.savefig(dest, dpi=160)
    plt.close(fig)
    figures.append(relative(dest))
    fig, axes = plt.subplots(len(data), 2, figsize=(14, 3 * len(data)), sharex=True, squeeze=False)
    for i, (label, item) in enumerate(data.items()):
        time = item["time"]
        axes[i, 0].plot(time, item["torque"], color=colors[i % 4], linewidth=.8)
        axes[i, 0].fill_between(time, 0, item["positive"], alpha=.18, color=colors[i % 4])
        axes[i, 1].plot(time, item["force"], color=colors[i % 4], linewidth=.8)
        for event in item["event_analysis"]["events"]:
            if event["kind"] != "accepted_transition":
                continue
            t = event["event_time_s"]
            style = ":" if "resync" in event["event"] else "--"
            for ax in axes[i]:
                ax.axvline(t, color="#444444", alpha=.5, linestyle=style, linewidth=.6)
        axes[i, 0].set(ylabel=f"{label}\nAnkle [Nm]")
        axes[i, 1].set(ylabel="Fz [N]")
    axes[0, 0].set_title("Coppia SEA ankle; linee: transizioni accettate, puntini: resync")
    axes[0, 1].set_title("Forza verticale applicata, distinta dal latch binario")
    for ax in axes[-1]:
        ax.set_xlabel("Tempo assoluto [s]")
    fig.tight_layout()
    dest = path / "torque_contact_event_timeline.png"
    fig.savefig(dest, dpi=160)
    plt.close(fig)
    figures.append(relative(dest))
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for i, (label, item) in enumerate(data.items()):
        for ax, joint in zip(axes[:2], ("knee", "ankle")):
            ax.plot(item["time"], np.rad2deg(item["tables"]["kinematics"][f"pros_{joint}_angle"]), label=label, color=colors[i % 4])
        axes[2].plot(item["time"], item["force"], label=label, color=colors[i % 4])
    axes[0].set(ylabel="Knee actual [°]", title="Primo ciclo prescribed: movimento e appoggio durante il problema di qualificazione")
    axes[1].set(ylabel="Ankle actual [°]")
    axes[2].set(xlabel="Tempo assoluto [s]", ylabel="Fz [N]", xlim=(13.85, 15.7))
    axes[0].legend(ncol=len(data))
    fig.tight_layout()
    dest = path / "first_cycle_kinematics_and_contact.png"
    fig.savefig(dest, dpi=160)
    plt.close(fig)
    figures.append(relative(dest))
    return figures


def self_check():
    assert segment_membership([0, .5, 1, 1.5], [0, 1], [1, 2]).tolist() == [0, 0, 1, 1]
    for times, starts, ends in (([0, 1], [0], [1]), ([.5], [0, .7], [.4, 1]), ([0, .1], [0, .05], [.2, .3])):
        try:
            segment_membership(times, starts, ends)
        except ValueError:
            pass
        else:
            raise AssertionError("Expected uncovered/overlap rejection")
    weights = interval_weights(np.array([0, .5, 1, 1.5]), .5, .25, 1.25)
    assert np.array_equal(weights, [.25, .5, .25, 0])
    assert integrate(np.array([2, 4, 8, 16]), weights) == 4.5
    return {"exact_segment_boundaries": "passed", "uncovered_and_overlap_rejected": "passed",
            "partial_window_integration": "passed"}


def report(result, path):
    lines = ["# Diagnosi offline delle fasi e dei cicli — LEFT, 2026-09-28", "",
             "Analisi dei quattro nominali salvati; nessun training, nessuna simulazione e nessuna modifica alla FSM.", "",
             "## Risultati fisici", "",
             "| Policy | Cicli FSM validi | Invalidi / resync | Ankle+ intero [Nm·s] | Ankle+ storico [Nm·s] | Impulso storico [N·s] |",
             "| --- | --- | --- | --- | --- | --- |"]
    for label, item in result["rollouts"].items():
        f, full, hist = item["events"]["final_counters"], item["totals"]["full"], item["totals"]["historical"]
        lines.append(f"| {label} | {f['valid_cycle_count']:g} | {f['invalid_event_count']:g} / {f['resync_count']:g} | {full['positive_area_nm_s']:.6f} | {hist['positive_area_nm_s']:.6f} | {hist['vertical_impulse_n_s']:.6f} |")
    lines += ["", "## Eventi e qualificazione dei cicli", "",
              "HS richiede contatto geometrico heel/BOTH stabile per 5 ms; TOE isolato in swing non apre stance. TO richiede AIR stabile. La FSM actor rifiuta HS prima di 0.20 s dal TO. In v3 una contraddizione con il latch funzionale binario mantenuta per 0.08 s provoca resync: segmento non valido, ciclo degradato e nessun credito. Il successivo HS scarta il ciclo degradato e avvia una nuova ancora. La forza verticale applicata e i sensori legacy non sono il latch funzionale che guida questa logica.", "",
              "Le righe di eventi invalidi riportano il confine del segmento: il timestamp del candidato rifiutato non è conservato nei trace. Gli eventi accettati conservano onset, conferma e consegna. L'onset del resync indica l'inizio della contraddizione osservata, non necessariamente l'istante di un HS fisico."]
    for label, item in result["rollouts"].items():
        lines += ["", f"### {label}", "", "| Evento | Onset [s] | Confine trace [s] | Ciclo valido | Motivo |",
                  "| --- | --- | --- | --- | --- |"]
        for event in item["events"]["events"]:
            onset = "non disponibile" if event["event_time_s"] is None else f"{event['event_time_s']:.3f}"
            valid = event.get("cycle_valid")
            validity = "—" if valid is None or valid < 0 else ("sì" if valid else "no")
            lines.append(f"| {event['event']} | {onset} | {event['recorded_boundary_time_s']:.3f} | {validity} | {event.get('cycle_reject_reason', '')} |")
        periods = ", ".join(f"{c['start_hs_s']:.3f}–{c['end_hs_s']:.3f} ({c['period_s']:.3f} s)" for c in item["events"]["valid_cycles"])
        lines += ["", f"Cicli accettati: {periods or 'nessuno'}."]
    if "ankle5_84" in result["rollouts"] and "best80" in result["rollouts"]:
        lines += ["", "### Interpretazione del ciclo mancante di ankle_5", "",
                  "Nel nominale ankle_5 il TO a 14.538 s è seguito da `hs_too_early_after_to`, osservato al confine 14.570 s: l'onset preciso rifiutato non è disponibile. Il resync HS consegnato a 14.650 s usa l'ancora 14.570 s e chiude il ciclo come `resync`; l'HS a 15.605 s rifiuta poi `cycle_degraded`. Solo 15.605–17.150 s viene contato come ciclo valido. Nessun HS viene cancellato e l'episodio arriva a 5 s.", "",
                  "Best80 conta invece anche il breve ciclo 13.926–14.695 s, di 0.769 s; il TO successivo è rifiutato come troppo precoce, seguito da TO resync con ancora 14.720 s e consegna 14.800 s. L'HS a 15.593 s scarta il ciclo degradato. I conteggi 2 contro 1 restano un vincolo del pilot, ma non dimostrano da soli la perdita di un'intera oscillazione cinematica. I CSV separano segmenti accepted, cicli FSM e finestre di ciclo prescribed."]
        first = result["rollouts"]["ankle5_84"]["events"]["prescribed_complete_cycle_windows"][0]
        baseline_first = result["rollouts"]["best80"]["events"]["prescribed_complete_cycle_windows"][0]
        lines += ["", f"Nella prima finestra prescribed comune 13.95–15.62 s, ankle_5 conserva un'escursione knee di {first['knee_actual_excursion_deg']:.2f}° e ankle di {first['ankle_actual_excursion_deg']:.2f}°, contro {baseline_first['knee_actual_excursion_deg']:.2f}° / {baseline_first['ankle_actual_excursion_deg']:.2f}° di best80. L'impulso è {first['vertical_impulse_n_s']:.2f} contro {baseline_first['vertical_impulse_n_s']:.2f} N·s. Questi dati documentano movimento e carico presenti, pur con qualificazione FSM diversa; non sostituiscono i vincoli di tracking per fase."]
    lines += ["", "## Movimento nelle finestre comuni di ciclo prescribed", "",
              "Finestre individuate dai wrap del clock al principio dei segmenti; quantizzazione massima 10 ms. Sono finestre diagnostiche comuni, non nuovi cicli FSM validati.", "",
              "| Policy | Finestra [s] | Escursione knee actual [°] | Escursione ankle actual [°] | Ankle+ [Nm·s] | Impulso [N·s] |",
              "| --- | --- | --- | --- | --- | --- |"]
    for label, item in result["rollouts"].items():
        for c in item["events"]["prescribed_complete_cycle_windows"]:
            lines.append(f"| {label} | {c['window_s'][0]:.3f}–{c['window_s'][1]:.3f} | {c['knee_actual_excursion_deg']:.3f} | {c['ankle_actual_excursion_deg']:.3f} | {c['positive_area_nm_s']:.3f} | {c['vertical_impulse_n_s']:.3f} |")
    lines += ["", "## Distribuzione temporale, incluse le porzioni di ciclo ai bordi", "",
              "Le porzioni iniziale e finale sono riportate separatamente: non vengono spacciate per cicli completi. La loro somma con i due cicli completi ricostruisce esattamente l'episodio.", "",
              "| Policy | Finestra [s] | Ciclo prescribed completo | Ankle+ [Nm·s] | Impulso [N·s] |",
              "| --- | --- | --- | --- | --- |"]
    for label, item in result["rollouts"].items():
        for c in item["events"]["all_prescribed_windows"]:
            complete = "sì" if c["complete_prescribed_cycle"] else "no, porzione al bordo"
            lines.append(f"| {label} | {c['window_s'][0]:.3f}–{c['window_s'][1]:.3f} | {complete} | {c['positive_area_nm_s']:.3f} | {c['vertical_impulse_n_s']:.3f} |")
    if "ankle10_84" in result["rollouts"] and "best80" in result["rollouts"]:
        baseline, candidate = [result["rollouts"][label] for label in ("best80", "ankle10_84")]
        last_base, last_candidate = [item["events"]["all_prescribed_windows"][-1] for item in (baseline, candidate)]
        total_reduction = baseline["totals"]["full"]["positive_area_nm_s"] - candidate["totals"]["full"]["positive_area_nm_s"]
        last_reduction = last_base["positive_area_nm_s"] - last_candidate["positive_area_nm_s"]
        lines += ["", f"Ankle_10: nell'ultima porzione 17.17–17.99 s l'area passa da {last_base['positive_area_nm_s']:.3f} a {last_candidate['positive_area_nm_s']:.3f} Nm·s, contribuendo al {100*last_reduction/total_reduction:.1f}% della riduzione totale osservata. Nella stessa porzione l'impulso scende da {last_base['vertical_impulse_n_s']:.3f} a {last_candidate['vertical_impulse_n_s']:.3f} N·s. Nei due cicli prescribed completi precedenti, invece, l'impulso di ankle_10 non diminuisce rispetto a best80. È una concentrazione temporale del risultato nominale; non prova un legame causale diretto fra segno della coppia e forza verticale."]
    lines += ["", "## Differenze per fase rispetto a best80", "",
              "| Policy | Decile prescribed | Δ ankle+ [Nm·s] | Δ impulso [N·s] |",
              "| --- | --- | --- | --- |"]
    for row in result["phase_deltas_vs_best80"]:
        lines.append(f"| {row['policy']} | {row['phase_bin']/10:.1f}–{(row['phase_bin']+1)/10:.1f} | {row['delta_positive_area_nm_s']:+.4f} | {row['delta_vertical_impulse_n_s']:+.3f} |")
    if "ankle10_84" in result["rollouts"]:
        deltas = [row for row in result["phase_deltas_vs_best80"] if row["policy"] == "ankle10_84"]
        if deltas:
            mid_area = sum(row["delta_positive_area_nm_s"] for row in deltas if row["phase_bin"] in (1, 2, 3))
            late_support = sum(row["delta_vertical_impulse_n_s"] for row in deltas if row["phase_bin"] in (2, 3, 4))
            early_support = next(row["delta_vertical_impulse_n_s"] for row in deltas if row["phase_bin"] == 0)
            base = result["rollouts"]["best80"]["totals"]["historical"]
            cand = result["rollouts"]["ankle10_84"]["totals"]["historical"]
            lines += ["", f"Per ankle_10 la variazione dell'area nella fase prescribed 0.1–0.4 è {mid_area:+.3f} Nm·s. L'impulso nella fase 0.2–0.5 varia di {late_support:+.3f} N·s, in parte compensato da {early_support:+.3f} N·s nella fase 0.0–0.1. Il tempo sotto carico ≥50 N non diminuisce ({base['loaded_duration_s']:.3f}→{cand['loaded_duration_s']:.3f} s nella finestra storica): il calo dell'impulso totale riguarda distribuzione e intensità dell'appoggio, non semplicemente meno campioni caricati."]
    lines += ["", "## Metodo e verifiche", "",
              "Campioni torque, forza e cinematica a 1 kHz rigorosamente allineati. Ogni campione appartiene al segmento policy [start,end); il clock actor presente all'inizio del segmento viene mantenuto costante entro quei 10 ms. Nessuna interpolazione dalla fase futura, nessuna fase o forza mancante sostituita con zero. Le RMSE per fase nei CSV usano separatamente la fase target allo stesso endpoint del campione di tracking, come la metrica storica.", "",
              "Le integrazioni usano intervalli semiaperti e pesano l'eventuale intersezione parziale con la finestra. La finestra storica è [13.15,17.95) s: la precedente metrica di confronto includeva anche il campione a 17.950 s. I valori inclusivi sono conservati nel JSON, evitando di confondere tale differenza di un campione con una variazione fisica. L'episodio intero resta 12.99–17.99 s.", "",
              "Per tutti i segmenti, conteggi SEA esatti e somme positive coerenti con le statistiche online entro 1e-6 Nm; somme per decile riconciliate con gli integrali totali entro 1e-8. Test sintetici coprono bordi esatti, campioni senza segmento, sovrapposizioni e integrazione di una finestra parziale. I contatori dei cicli coincidono con gli eventi accettati e con i summary. Tutti gli artefatti dei rollout sono verificati contro le ricevute, con manifest degli hash degli input e del codice interpretativo.", "",
              "Ogni misura di segmento/resync conserva finestra richiesta, finestra coperta, durata coperta e flag `window_fully_covered`. Un intervallo eventualmente tagliato ai bordi dell'episodio è esplicito; campioni assenti non sono inventati. La penetrazione è misurata sui campioni GRF e può differire dal massimo nel summary, che campiona confini diversi.", "",
              "Il report è descrittivo e non modifica i vincoli originali. Non ricostruisce i campioni binari grezzi, assenti da questi output, e non identifica causalmente quale perturbazione cinematica abbia provocato il candidato HS rifiutato. Un solo seed di training e traiettorie dei worker differenti non consentono di attribuire tutte le differenze al peso della reward.", "", "## Figure", ""]
    for figure in result["figures"]:
        lines.append(f"- [{Path(figure).name}](<{REPO / figure}>)")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout", action="append", help="LABEL=path; replaces default four nominal rollouts")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--plots", type=Path, default=DEFAULT_PLOTS)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    supplied = dict(pair.split("=", 1) for pair in args.rollout) if args.rollout else {
        label: PACKAGE / "runs/rollout" / name for label, name in DEFAULT_ROLLOUTS.items()}
    checks = self_check()
    all_data, all_phase, all_events, cycle_rows, rollouts = {}, [], [], [], {}
    for label, path in supplied.items():
        item = load_rollout(label, path)
        phases, totals = phase_analysis(item)
        events = event_analysis(item)
        item["event_analysis"] = events
        all_data[label] = item
        all_phase.extend(phases)
        all_events.extend(events["events"])
        cycle_rows.extend({"policy": label, **cycle} for cycle in events["all_prescribed_windows"])
        rollouts[label] = {"path": item["path"], "end_reason": item["summary"]["end_reason"],
                           "totals": totals, "reconciliation": item["reconciliation"], "events": events}
    deltas = []
    if "best80" in rollouts:
        baseline = {r["phase_bin"]: r for r in all_phase if r["policy"] == "best80" and r["window"] == "historical"}
        for row in all_phase:
            if row["policy"] == "best80" or row["window"] != "historical":
                continue
            deltas.append({"policy": row["policy"], "phase_bin": row["phase_bin"],
                           **{f"delta_{key}": row[key] - baseline[row["phase_bin"]][key]
                              for key in ("positive_area_nm_s", "vertical_impulse_n_s")}})
    figures = plots(all_data, all_phase, args.plots.resolve())
    result = {"schema": "left_pilot_phase_diagnosis_v1", "training_started": False,
              "simulation_started": False, "self_checks": checks, "rollouts": rollouts,
              "phase_metrics": all_phase, "phase_deltas_vs_best80": deltas, "figures": figures,
              "method": "Causal held start-phase; exact pre-integration [start,end) membership; half-open weighted integration; no missing-data fallback."}
    for source in (Path(__file__), TG / "prosthetic_phase_fsm.py", TG / "binary_phase_adapter.py",
                   TG / "binary_phase_adapter_v26.py", TG / "binary_phase_fsm_v26.py",
                   PACKAGE / "reward_tuning/metrics.py", PACKAGE / "phase_target.py"):
        record_hash(source)
    (output / "diagnosis.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output / "input_hashes.json").write_text(json.dumps(HASHES, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    save_csv(output / "phase_metrics.csv", all_phase)
    save_csv(output / "events.csv", all_events)
    save_csv(output / "prescribed_cycle_windows.csv", cycle_rows)
    save_csv(output / "phase_deltas_vs_best80.csv", deltas)
    report(result, output / "report.md")
    print(json.dumps({"report": str(output / "report.md"), "hashes": len(HASHES),
                      "checks": checks, "rollouts": {label: item["reconciliation"] for label, item in rollouts.items()}}, indent=2))


if __name__ == "__main__":
    main()
