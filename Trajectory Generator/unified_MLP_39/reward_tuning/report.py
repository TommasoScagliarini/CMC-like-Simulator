"""Italian physical pilot reports from sealed comparison metrics, without simulation."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .io import REPO, resolve, sha256, write_json

SCENARIOS = ("nominal", "noise_123", "noise_125", "noise_127")
STATUS = {"promising": "promettente", "rejected_constraints": "respinta dai vincoli",
          "inconclusive": "inconclusiva", "incomplete": "incompleta"}


def _get(value, *keys):
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _fmt(value, digits=3):
    if value is None:
        return "n.d."
    if isinstance(value, bool):
        return "sì" if value else "no"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}"
    return str(value).replace("|", "/").replace("\n", " ")


def _table(headers, rows):
    return ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |",
            *["| " + " | ".join(_fmt(v) for v in row) + " |" for row in rows], ""]


def _flatten(value, prefix=""):
    result = {}
    for name, child in value.items():
        key = f"{prefix}.{name}" if prefix else name
        if isinstance(child, dict):
            result.update(_flatten(child, key))
        elif isinstance(child, list):
            result[key] = json.dumps(child)
        else:
            result[key] = child
    return result


def _write_csv(path, rows, base_columns):
    columns = list(base_columns) + sorted({k for row in rows for k in row} - set(base_columns))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def _critical(name):
    return any(key in name for key in ("completion", "valid_cycles", "hard_termination", "penetration"))


def _constraint_rows(comparison):
    rows = []
    for scenario, entry in comparison.get("scenarios", {}).items():
        for reference, checks in entry.get("comparisons", {}).items():
            for check in checks:
                row = {"scenario": scenario, "reference": reference, **check}
                name = check["name"]
                minimum = any(key in name for key in ("completion_of_reference", "valid_cycles", "excursion", "support_impulse"))
                row["direction"] = "min" if minimum else "max"
                actual, bound = check.get("actual"), check.get("bound")
                row["relative_violation"] = None if actual is None or bound is None else max(
                    0., (bound - actual if minimum else actual - bound) / max(abs(bound), 1e-12))
                rows.append(row)
    return rows


def _plots(plot_dir, entries):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    roles = list(entries)
    colors = ["#48556b", "#5796b0", "#d87027", "#b697bd"]
    width = .8 / len(roles)
    paths = []

    def grouped(ax, categories, extractor, ylabel, title, *, mask_failures=False):
        x = np.arange(len(categories))
        for index, role in enumerate(roles):
            values = []
            unavailable = []
            for category in categories:
                metric, complete = extractor(entries[role], category)
                masked = metric is None or (mask_failures and not complete)
                values.append(np.nan if masked else metric)
                unavailable.append("interrotto" if mask_failures and complete is False else "n.d.")
            positions = x + (index - (len(roles) - 1) / 2) * width
            bars = ax.bar(positions, values, width, label=role, color=colors[index % len(colors)])
            if entries[role]["descriptive"]:
                for bar in bars:
                    bar.set_hatch("//")
            for pos, value, reason in zip(positions, values, unavailable):
                if not np.isfinite(value):
                    ax.text(pos, .03, reason, rotation=90, ha="center", va="bottom", fontsize=7,
                            transform=ax.get_xaxis_transform())
        ax.set_xticks(x, categories)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)

    def save(fig, name):
        path = plot_dir / name
        fig.tight_layout()
        fig.savefig(path, dpi=160, bbox_inches="tight")
        plt.close(fig)
        paths.append(path)

    fig, ax = plt.subplots(figsize=(11, 4.6))
    def area(entry, scenario):
        metric = _get(entry, "scenarios", scenario, "full")
        return _get(metric, "ankle", "positive_area_nm_s"), _get(metric, "episode", "completed")
    grouped(ax, SCENARIOS, area, "Area positiva ankle [Nm·s]", "Episodi completi; aree dei rollout interrotti escluse", mask_failures=True)
    ax.legend(fontsize=8)
    save(fig, "ankle_positive_area_scenarios.png")

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    channels = ("knee actual", "knee served", "ankle actual", "ankle served")
    for ax, view in zip(axes, ("common_window", "full")):
        def tracking(entry, category, view=view):
            joint, kind = category.split()
            metric = _get(entry, "scenarios", "nominal", view)
            return _get(metric, "tracking", joint, kind + "_rmse_deg"), _get(metric, "episode", "completed")
        grouped(ax, channels, tracking, "RMSE [°]", "Nominale: " + ("finestra comune" if view == "common_window" else "episodio intero"))
        ax.tick_params(axis="x", rotation=20)
    axes[0].legend(fontsize=8)
    save(fig, "nominal_tracking.png")

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.4))
    for ax, key, label in zip(axes, ("rate_rms_nm_s", "hp10_rms_nm", "hp20_rms_nm"),
                              ("Rateo RMS [Nm/s]", "Passa-alto 10 Hz RMS [Nm]", "Passa-alto 20 Hz RMS [Nm]")):
        def knee(entry, _category, key=key):
            metric = _get(entry, "scenarios", "nominal", "common_window")
            return _get(metric, "knee", key), _get(metric, "episode", "completed")
        grouped(ax, ("knee",), knee, label, "Coppia nominale, finestra comune")
    axes[0].legend(fontsize=8)
    save(fig, "nominal_knee_torque_regularity.png")

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    for ax, key, label in zip(axes, ("jerk_rms_rad_s3", "position_hp10_rms_deg"),
                              ("Jerk RMS [rad/s³]", "Posizione sopra 10 Hz RMS [°]")):
        def smoothness(entry, category, key=key):
            metric = _get(entry, "scenarios", "nominal", "common_window")
            return _get(metric, "kinematic_smoothness", category.replace(" ", "_"), key), _get(metric, "episode", "completed")
        grouped(ax, channels, smoothness, label, "Cinematica nominale, finestra comune")
        ax.tick_params(axis="x", rotation=20)
    axes[0].legend(fontsize=8)
    save(fig, "nominal_kinematic_regularity.png")
    return paths


def build_report(config, output, previous_comparison=None):
    """Generate report/CSV/figures from comparison.json; never launches training.

    Missing scenarios remain explicitly incomplete. A previous candidate is a
    descriptive overlay only and never participates in the current verdict.
    """
    root = Path(config.get("_repo_root", REPO)).resolve()
    output = resolve(output, root)
    source = output / "comparison.json"
    comparison = json.loads(source.read_text(encoding="utf-8"))
    candidate = comparison.get("candidate_role", "ankle_10")
    manifest_path = output / "pilot/pilot_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    inputs = {str(source): sha256(source)}
    if manifest_path.is_file():
        inputs[str(manifest_path)] = sha256(manifest_path)
    entries = {}
    for role in ("best80", "control", candidate):
        entries[role] = {"descriptive": False, "source": str(source), "scenarios": {
            scenario: {view: values[view][role] for view in ("full", "common_window")
                       if role in values.get(view, {})}
            for scenario, values in comparison.get("scenarios", {}).items()}}
    if previous_comparison is not None:
        previous_path = resolve(previous_comparison, root)
        previous = json.loads(previous_path.read_text(encoding="utf-8"))
        inputs[str(previous_path)] = sha256(previous_path)
        previous_role = previous.get("candidate_role", "ankle_10")
        label = previous_role + " (precedente)"
        entries[label] = {"descriptive": True, "source": str(previous_path), "scenarios": {
            scenario: {view: values[view][previous_role] for view in ("full", "common_window")
                       if previous_role in values.get(view, {})}
            for scenario, values in previous.get("scenarios", {}).items()}}
    plot_dir = resolve(config["plot_dir"], root) / "pilot_comparison"
    if root / "plot" not in plot_dir.parents:
        raise ValueError("Pilot report figures must live under root plot/")
    plot_dir.mkdir(parents=True, exist_ok=True)
    metric_rows = [{"role": role, "descriptive": entry["descriptive"], "source_comparison": entry["source"],
                    "scenario": scenario, "view": view, **_flatten(metric)}
                   for role, entry in entries.items() for scenario, views in entry["scenarios"].items()
                   for view, metric in views.items()]
    constraints = _constraint_rows(comparison)
    metrics_path, constraints_path = output / "physical_metrics.csv", output / "physical_constraints.csv"
    _write_csv(metrics_path, metric_rows, ("role", "descriptive", "scenario", "view", "source_comparison"))
    _write_csv(constraints_path, constraints, ("scenario", "reference", "name", "passed", "actual", "bound", "direction"))
    status = comparison.get("status", "incomplete")
    lines = [f"# Pilot LEFT — {candidate}", "", f"Esito: **{STATUS.get(status, status)}** (`{status}`).", "",
             "Il giudizio deriva dai vincoli fisici contro best80 e controllo a reward invariata. "
             "I ritorni di reward differenti non vengono usati per ordinare le policy.", ""]
    if status == "incomplete" or comparison.get("missing_cases"):
        lines += ["**Analisi incompleta: nessuna selezione favorevole è giustificata.**", "",
                  *[f"- {case}" for case in comparison.get("missing_cases", [])], ""]
    if manifest.get("reference_reuse"):
        lines += ["Best80 e controllo riutilizzano checkpoint e otto valutazioni del pilot precedente, "
                  "verificati nella preparazione del nuovo manifest. Il nuovo training riguarda soltanto "
                  f"`{candidate}`; questi riferimenti non costituiscono nuove repliche indipendenti.", ""]
    else:
        lines += ["Best80 è il checkpoint sorgente; controllo è il ramo a reward invariata del protocollo corrente.", ""]
    lines += ["La calibrazione offline del peso descrive un costo sui trace esistenti. Le tabelle seguenti "
              "misurano l'effetto appreso nei rollout del pilot. Tre seed rumorosi sono uno screening operativo, "
              "non una prova di robustezza statistica.", "",
              "## Obiettivo ankle", ""]
    area_rows = []
    for reference in ("best80", "control"):
        nominal = next((c for c in comparison.get("nominal_ankle_area", []) if c["reference"] == reference), {})
        noisy = comparison.get("noisy_ankle_area", {}).get(reference, {})
        area_rows.append((reference, nominal.get("delta_nm_s"), nominal.get("passed"),
                          noisy.get("median_delta_nm_s"), noisy.get("passed")))
    lines += _table(("Riferimento", "Δ area nominale [Nm·s]", "Riduzione richiesta", "Mediana Δ rumore [Nm·s]", "Non aumenta"), area_rows)
    lines += ["Delta negativi indicano riduzione. Una mediana mancante include fallimenti o dati insufficienti; "
              "nessun rollout interrotto beneficia dell'area ridotta per durata inferiore.", ""]
    for view, title in (("common_window", "Nominale — finestra storica comune"), ("full", "Nominale — episodio intero")):
        lines += ["## " + title, ""]
        rows = []
        for role, entry in entries.items():
            metric = _get(entry, "scenarios", "nominal", view)
            interval = _get(metric, "window_s")
            window = "n.d." if interval is None else "–".join(f"{v:.2f}" for v in interval)
            rows.append((role, window, _get(metric, "ankle", "positive_area_nm_s"),
                         _get(metric, "tracking", "knee", "actual_rmse_deg"),
                         _get(metric, "tracking", "ankle", "actual_rmse_deg"),
                         _get(metric, "contact", "vertical_impulse_n_s"), _get(metric, "knee", "hp10_rms_nm")))
        lines += _table(("Policy", "Finestra [s]", "Area ankle+ [Nm·s]", "RMSE knee [°]", "RMSE ankle [°]", "Impulso [N·s]", "Knee HP10 [Nm]"), rows)
    lines += ["## Scenari — intero episodio", ""]
    for scenario in SCENARIOS:
        rows = []
        for role, entry in entries.items():
            m = _get(entry, "scenarios", scenario, "full")
            rows.append((role, _get(m, "episode", "completed"), _get(m, "episode", "duration_s"),
                         _get(m, "episode", "valid_cycles"), _get(m, "episode", "penetration_max_mm"),
                         _get(m, "ankle", "positive_area_nm_s"), _get(m, "tracking", "knee", "actual_rmse_deg"),
                         _get(m, "tracking", "ankle", "actual_rmse_deg")))
        lines += [f"### {scenario}", ""] + _table(
            ("Policy", "Completo", "Durata [s]", "Cicli", "Penetrazione [mm]", "Area ankle+ [Nm·s]", "RMSE knee [°]", "RMSE ankle [°]"), rows)
        for role, entry in entries.items():
            episode = _get(entry, "scenarios", scenario, "full", "episode")
            if episode and not episode.get("completed"):
                lines += [f"- `{role}`: `{episode.get('end_reason', 'n.d.')}`; area completa non confrontabile."]
        pairing = _get(comparison, "scenarios", scenario, "pairing")
        if pairing:
            lines += [f"Rumore appaiato: {_fmt(pairing.get('passed'))}; campioni comuni: {_fmt(pairing.get('common_draws'), 0)}."]
        lines += [""]
    lines += ["La penetrazione delle tabelle è il massimo riportato dall'episodio e usato nello screening. "
              "Il CSV conserva separatamente il massimo nei campioni GRF (anche nella finestra); "
              "i due valori possono differire ai confini del reset.", "", "## Vincoli falliti", ""]
    summary_rows = []
    for scenario in SCENARIOS:
        for ref in ("best80", "control"):
            values = [r for r in constraints if r["scenario"] == scenario and r["reference"] == ref]
            summary_rows.append((scenario, ref, sum(r.get("passed") is False for r in values),
                                 sum(r.get("passed") is None for r in values), len(values)))
    lines += _table(("Scenario", "Riferimento", "Falliti", "Non valutabili", "Totale"), summary_rows)
    failed = sorted((r for r in constraints if r.get("passed") is False),
                    key=lambda r: (_critical(r["name"]), r["relative_violation"] or 0), reverse=True)
    lines += ["Violazioni principali (prima completamento, cicli e contatto; poi scostamento relativo dal limite):", ""]
    lines += _table(("Scenario", "Riferimento", "Vincolo", "Misurato", "Limite", "Direzione"),
                    [(r["scenario"], r["reference"], r["name"], r["actual"], r["bound"], r["direction"]) for r in failed[:20]])
    lines += ["Il CSV dei vincoli include tutte le fasi prescribed, actual/served, escursioni e regolarità. "
              "`full_episode_` identifica controlli sull'intero episodio; gli altri usano il prefisso comune "
              "con filtri applicati prima del ritaglio.", ""]
    if previous_comparison is not None:
        lines += ["## Confronto con il pilot precedente", "",
                  "Le righe e barre «precedente» sono descrittive: non modificano il verdetto corrente, "
                  "non aggiungono repliche e non sostituiscono i vincoli contro best80 e controllo. "
                  "Le finestre comuni possono differire quando un rollout termina prima.", ""]
    figures = _plots(plot_dir, entries)
    lines += ["## Figure e dati", "", *[f"- [{path.name}]({path.as_posix()})" for path in figures],
              f"- [Metriche complete CSV]({metrics_path.as_posix()})",
              f"- [Vincoli completi CSV]({constraints_path.as_posix()})", ""]
    report_path = output / "report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    paths = {"report": str(report_path), "metrics_csv": str(metrics_path), "constraints_csv": str(constraints_path),
             "figures": [str(path) for path in figures]}
    receipt_path = output / "report_artifacts.json"
    write_json(receipt_path, {"schema": "reward_tuning_report_v1", "candidate_role": candidate,
                             "status": status, "input_hashes": inputs, "files": paths,
                             "artifact_hashes": {str(path): sha256(path) for path in
                                                 [report_path, metrics_path, constraints_path, *figures]}})
    return {**paths, "receipt": str(receipt_path)}
