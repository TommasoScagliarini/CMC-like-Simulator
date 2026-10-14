"""Read-only nominal screening of existing LEFT PPO milestones 82, 83 and 84.

Consumes experiment/nominal/pilot_manifest.json. Required evaluation_cases are
best80 and control/ankle_5/ankle_10 at iterations 82 and 83. endpoint_cases reuse
iteration84 with role aliases (e.g. control_84) and their original receipt_role.
No training, rollout or noisy evaluation is launched by this script.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from unified_MLP_39.reward_tuning.compare import DEFAULTS, compare_metrics
from unified_MLP_39.reward_tuning.io import REPO, resolve, sha256, write_json
from unified_MLP_39.reward_tuning.metrics import load_dataset, measure
from unified_MLP_39.reward_tuning.pilot import verify_hashes
from unified_MLP_39.reward_tuning.report import _flatten, _fmt, _get, _table, _write_csv

FAMILIES = ("control", "ankle_5", "ankle_10")
EXPECTED = ("best80", *(f"{family}_{iteration}" for iteration in (82, 83, 84) for family in FAMILIES))
HISTORICAL_WINDOW = [13.15, 17.95]


def _read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _load_case(case, root, input_hashes):
    """Authenticate measurements and their checkpoint before loading signals."""
    folder = resolve(case["output_dir"], root)
    receipt_path = folder / "evaluation_complete.json"
    if not receipt_path.is_file():
        return None, "evaluation_complete.json mancante"
    receipt = _read_json(receipt_path)
    role = case.get("receipt_role", case["role"])
    if receipt.get("role") != role or receipt.get("scenario") != "nominal":
        raise ValueError(f"Evaluation receipt identity mismatch: {folder}")
    artifacts = receipt.get("artifact_hashes", {})
    required = {"rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json"}
    required.update(f"sim_outputs/rollout_episode_{name}.sto"
                    for name in ("sea_torques", "kinematics", "kinematics_reference", "online_grf")
                    if (folder / "sim_outputs" / f"rollout_episode_{name}.sto").is_file())
    if not isinstance(artifacts, dict) or not required.issubset(artifacts):
        return None, "ricevuta artifact incompleta"
    for filename in artifacts:
        if not resolve(filename, folder).is_relative_to(folder):
            raise ValueError(f"Evaluation artifact escapes its folder: {filename}")
    verify_hashes(folder, artifacts)
    input_hashes[str(receipt_path)] = sha256(receipt_path)
    input_hashes.update({str(resolve(filename, folder)): digest for filename, digest in artifacts.items()})
    summary = _read_json(folder / "rollout_summary.json")
    noise = _read_json(folder / "paired_noise_receipt.json")
    checkpoint = resolve(case["checkpoint"], root)
    if resolve(summary["checkpoint"], root) != checkpoint:
        raise ValueError(f"Rollout checkpoint mismatch: {folder}")
    training_state_path = checkpoint / "unified_training_state.json"
    training_state = _read_json(training_state_path)
    expected_iteration = 80 if case["role"] == "best80" else int(case["role"].rsplit("_", 1)[1])
    if training_state.get("last_completed_iteration") != expected_iteration:
        raise ValueError(f"Role does not identify its measured checkpoint iteration: {case['role']}")
    input_hashes[str(training_state_path)] = sha256(training_state_path)
    protocol = noise.get("protocol", {})
    if protocol.get("role") != role or protocol.get("scenario") != "nominal":
        raise ValueError(f"Evaluation protocol identity mismatch: {folder}")
    for key in ("noise_seed", "environment_seed", "noise_scale", "maximum_steps"):
        if protocol.get(key) != case.get(key) or summary.get("evaluation_protocol", {}).get(key) != case.get(key):
            raise ValueError(f"Evaluation protocol {key} mismatch: {folder}")
    if resolve(protocol["checkpoint"], root) != checkpoint:
        raise ValueError(f"Noise protocol checkpoint mismatch: {folder}")
    sigma_path = resolve(case["sigma_path"], root)
    sigma_digest = sha256(sigma_path)
    if noise.get("sigma_sha256") != sigma_digest:
        raise ValueError(f"Frozen sigma receipt mismatch: {folder}")
    input_hashes[str(sigma_path)] = sigma_digest
    if (summary.get("ok") is not True or receipt.get("steps") != summary.get("steps")
            or receipt.get("steps") != noise.get("draws_used")
            or receipt.get("completed_full_episode") != summary.get("completed_full_episode")):
        raise ValueError(f"Evaluation completion metadata mismatch: {folder}")
    dataset = load_dataset(folder, identity=case["role"], scenario="nominal")
    input_hashes.update(dataset["input_hashes"])
    return dataset, None


def _completed_comparable(metrics):
    episodes = [value["episode"] for value in metrics]
    return (all(value.get("completed") is True for value in episodes)
            and max(value["duration_s"] for value in episodes) - min(value["duration_s"] for value in episodes) <= 1e-7)


def _valid_check(check):
    return (check.get("passed") is True and all(value is not None and math.isfinite(float(value))
                                               for value in (check.get("actual"), check.get("bound"))))


def _screen(role, datasets, measurements, criteria):
    """Each candidate is compared with best80 and the same PPO-update control."""
    family, number = role.rsplit("_", 1)
    refs = ["best80"] if family == "control" else ["best80", f"control_{number}"]
    entry = {"role": role, "references": refs, "comparisons": {}, "nominal_screen_passed": False,
             "eligible_for_new_noise_evaluation": False, "is_candidate": family != "control",
             "noise_stage": "previously_evaluated_endpoint" if number == "84" else "not_evaluated"}
    missing = [name for name in (role, *refs) if name not in measurements]
    if missing:
        entry["missing_cases"] = missing
        return entry
    full = {name: measurements[name]["full"] for name in (role, *refs)}
    lo = max(HISTORICAL_WINDOW[0], *(m["episode"]["start_s"] for m in full.values()))
    hi = min(HISTORICAL_WINDOW[1], *(m["episode"]["stop_s"] for m in full.values()))
    if hi <= lo:
        entry["missing_cases"] = ["nessun segmento completo nella finestra comune"]
        return entry
    aligned = {name: measure(datasets[name], [lo, hi], filter_stop=hi) for name in full}
    entry["common_window_s"] = [lo, hi]
    entry["common_window_metrics"] = aligned
    comparable = _completed_comparable(list(full.values()))
    entry["completed_equal_duration"] = comparable
    all_checks = []
    for reference in refs:
        checks = [{**check, "view": "historical_common_window"}
                  for check in compare_metrics(aligned[role], aligned[reference], criteria, nominal=True)]
        if comparable:
            checks += [{**check, "view": "full_episode"}
                       for check in compare_metrics(full[role], full[reference], criteria, nominal=True)]
        else:
            checks.append({"name": "full_episode_completed_equal_duration", "view": "full_episode",
                           "actual": 0, "bound": 1, "passed": False})
        area, reference_area = _get(full[role], "ankle", "positive_area_nm_s"), _get(full[reference], "ankle", "positive_area_nm_s")
        available = comparable and area is not None and reference_area is not None
        area_check = {"name": "ankle_positive_area_min_reduction", "view": "full_episode", "actual": area,
                      "bound": None if reference_area is None else (1 - criteria["ankle_area_min_reduction"]) * reference_area,
                      "passed": None if not available else area <= (1 - criteria["ankle_area_min_reduction"]) * reference_area,
                      "reduction_fraction": None if not available or reference_area == 0 else 1 - area / reference_area}
        # Unchanged-reward controls are diagnostic references, never reward candidates.
        if family != "control":
            checks.append(area_check)
        entry["comparisons"][reference] = {"checks": checks, "ankle_area": area_check,
                                          "physical_passed": all(_valid_check(c) for c in checks if c["name"] != area_check["name"])}
        all_checks += checks
    entry["nominal_screen_passed"] = bool(comparable and all_checks and all(_valid_check(c) for c in all_checks))
    entry["eligible_for_new_noise_evaluation"] = bool(entry["is_candidate"] and number in {"82", "83"}
                                                     and entry["nominal_screen_passed"])
    return entry


def _category(name):
    if name.startswith("tracking_"):
        return "tracking_phase" if "_phase_" in name else "tracking_global"
    if name.startswith("excursion_"):
        return "range_of_motion"
    if name.startswith(("knee_regularity_", "kinematic_regularity_")):
        return "regularity"
    if name.startswith("ankle_positive_area"):
        return "ankle_objective"
    return "contact_completion"


def _curves(result, plot_dir):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    plot_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    labels = ["best80\n0 PPO", "82\n1 PPO", "83\n2 PPO", "84\n3 PPO"]

    def plot_axis(ax, view, key, title, unit):
        for family, color in zip(FAMILIES, ("#546983", "#dd8b31", "#a14d78")):
            values = []
            for role in ("best80", *(f"{family}_{iteration}" for iteration in (82, 83, 84))):
                metrics = result["measurements"].get(role, {})
                value = _get(metrics.get(view), *key)
                completed = _get(metrics, "full", "episode", "completed")
                covered = view == "full" or metrics.get("historical_window_complete", False)
                values.append(value if completed and covered and value is not None else np.nan)
            ax.plot(range(4), values, "o-", label=family, color=color, linewidth=1.8)
        ax.set_xticks(range(4), labels)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(unit)
        ax.grid(alpha=.25)

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    specs = [("full", ("ankle", "positive_area_nm_s"), "Area positiva ankle — episodio", "Nm·s"),
             ("historical_window", ("tracking", "knee", "actual_rmse_deg"), "Tracking knee — finestra storica", "RMSE °"),
             ("historical_window", ("tracking", "ankle", "actual_rmse_deg"), "Tracking ankle — finestra storica", "RMSE °"),
             ("historical_window", ("contact", "vertical_impulse_n_s"), "Impulso verticale — finestra storica", "N·s"),
             ("full", ("episode", "penetration_max_mm"), "Penetrazione massima episodio", "mm"),
             ("historical_window", ("knee", "hp10_rms_nm"), "Coppia knee passa-alto 10 Hz", "RMS Nm")]
    for ax, spec in zip(axes.flat, specs):
        plot_axis(ax, *spec)
    axes[0, 0].legend()
    fig.suptitle("Nominale LEFT: evoluzione sui checkpoint esistenti; nessun nuovo training")
    fig.tight_layout()
    path = plot_dir / "physical_evolution.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    specs = [("historical_window", ("tracking", joint, "excursion_deg"), f"Escursione {joint}", "°") for joint in ("knee", "ankle")]
    specs += [("historical_window", ("kinematic_smoothness", f"{joint}_{channel}", "jerk_rms_rad_s3"),
               f"Jerk {joint} {channel}", "RMS rad/s³") for joint in ("knee", "ankle") for channel in ("actual", "served")]
    for ax, spec in zip(axes.flat, specs):
        plot_axis(ax, *spec)
    axes[0, 0].legend()
    fig.suptitle("Nominale LEFT: escursioni e regolarità actual/served nella finestra storica")
    fig.tight_layout()
    path = plot_dir / "kinematic_evolution.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)
    return [str(path) for path in paths]


def analyze(experiment):
    experiment = Path(experiment).resolve()
    manifest_path = experiment / "nominal/pilot_manifest.json"
    manifest = _read_json(manifest_path)
    root = Path(manifest.get("repo_root", REPO)).resolve()
    verify_hashes(root, manifest.get("input_hashes", {}))
    state_path = experiment / "nominal/pilot_state.json"
    state = _read_json(state_path) if state_path.is_file() else {}
    if state.get("manifest_sha256") is not None and state["manifest_sha256"] != sha256(manifest_path):
        raise ValueError("Nominal diagnosis manifest changed")
    criteria = {**DEFAULTS, **manifest.get("criteria", {})}
    if criteria != DEFAULTS or manifest.get("analysis_window_s", HISTORICAL_WINDOW) != HISTORICAL_WINDOW:
        raise ValueError("This diagnostic must preserve the original physical criteria and historical window")
    input_hashes = {str(manifest_path): sha256(manifest_path)}
    result = {"schema": "left_pilot_intermediate_nominal_diagnosis_v1", "status": "incomplete",
              "criteria": criteria, "historical_window_s": HISTORICAL_WINDOW, "input_hashes": input_hashes,
              "measurements": {}, "screening": {}, "missing_cases": [], "eligible": [],
              "interpretation": "Nominal screening only; eligibility requires subsequent paired noisy evaluation, never global promising status",
              "checkpoint_axis": {"80": "shared source, zero PPO updates", "81": "critic-only; no synthetic rollout or metric",
                                  "82": "1 PPO update", "83": "2 PPO updates", "84": "3 PPO updates; reused measured endpoint"}}
    datasets, cases = {}, {}
    for case in manifest.get("evaluation_cases", []) + manifest.get("endpoint_cases", []):
        if case.get("scenario") != "nominal":
            raise ValueError("This diagnostic accepts nominal cases only")
        role = case["role"]
        if role in cases or role not in EXPECTED:
            raise ValueError(f"Duplicate or unexpected nominal role: {role}")
        case = dict(case)
        case["output_dir"] = state.get("evaluation", {}).get(role, {}).get("nominal", case["output_dir"])
        cases[role] = case
    for role in EXPECTED:
        if role not in cases:
            result["missing_cases"].append({"role": role, "reason": "caso assente dal manifest"})
            continue
        data, reason = _load_case(cases[role], root, input_hashes)
        if data is None:
            result["missing_cases"].append({"role": role, "reason": reason})
            continue
        datasets[role] = data
        full = measure(data)
        lo, hi = max(HISTORICAL_WINDOW[0], full["episode"]["start_s"]), min(HISTORICAL_WINDOW[1], full["episode"]["stop_s"])
        historical = measure(data, [lo, hi], filter_stop=hi) if hi > lo else None
        result["measurements"][role] = {"full": full, "historical_window": historical,
                                       "historical_window_complete": lo <= HISTORICAL_WINDOW[0] + 1e-7 and hi >= HISTORICAL_WINDOW[1] - 1e-7,
                                       "missing_files": data["missing_files"], "rollout": cases[role]["output_dir"],
                                       "checkpoint": cases[role]["checkpoint"], "reused": role == "best80" or role.endswith("_84")}
    for role in EXPECTED[1:]:
        result["screening"][role] = _screen(role, datasets, result["measurements"], criteria)
    if not result["missing_cases"]:
        result["status"] = "nominal_screening_complete"
        result["eligible"] = [role for role, value in result["screening"].items() if value["eligible_for_new_noise_evaluation"]]
    rows, constraints = [], []
    for role, entry in result["measurements"].items():
        for view in ("full", "historical_window"):
            if entry.get(view) is not None:
                rows.append({"role": role, "view": view, "reused": entry["reused"],
                             "ppo_updates": 0 if role == "best80" else int(role.rsplit("_", 1)[1]) - 81,
                             **_flatten(entry[view])})
    for role, screening in result["screening"].items():
        for reference, entry in screening["comparisons"].items():
            constraints += [{"role": role, "reference": reference, "category": _category(check["name"]), **check}
                            for check in entry["checks"]]
    output = experiment / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    metrics_path, constraints_path = output / "nominal_metrics.csv", output / "nominal_constraints.csv"
    _write_csv(metrics_path, rows, ("role", "view", "ppo_updates", "reused"))
    _write_csv(constraints_path, constraints, ("role", "reference", "view", "category", "name", "passed", "actual", "bound"))
    plot_dir = resolve(manifest.get("plot_dir", "plot/09_28_2026_LEFT_pilot_diagnosis"), root) / "intermediate_comparison"
    if root / "plot" not in plot_dir.parents:
        raise ValueError("Comparison figures must live below root plot/")
    result["figures"] = _curves(result, plot_dir)
    lines = ["# Diagnosi LEFT — checkpoint intermedi del pilot", "", f"Stato: **{result['status']}**.", "",
             "Protocollo predisposto per sei valutazioni nominali dei checkpoint 82/83 già addestrati; best80 e i tre endpoint 84 "
             "sono riutilizzati con verifica delle ricevute e degli hash. Nessun nuovo training.", "",
             "Il passo 81 è il solo riadattamento del critic: non viene simulato e non riceve metriche sintetiche. "
             "Le curve partono dall'unica misura best80 (0 PPO), poi 82 (1 PPO), 83 (2 PPO), 84 (3 PPO). "
             "Lo stesso punto best80 compare nelle tre linee come riferimento comune, senza nuove repliche.", "",
             "## Accesso alle prove rumorose", "",
             "Intermedi ammessi: **" + (", ".join(result["eligible"]) or "nessuno") + "**.", "",
             "Sono necessari tutti i vincoli fisici sull'intero episodio e sulla finestra comune contro best80 "
             "e il controllo allo stesso aggiornamento, più almeno 10% di riduzione dell'area positiva ankle "
             "sull'intero episodio contro entrambi. Dati mancanti, episodi interrotti o durate diverse escludono "
             "l'ammissione. L'ammissibilità nominale richiede ancora le tre prove rumorose appaiate; non equivale "
             "a un candidato globalmente promettente. I controlli sono riferimenti fisici, non candidate ankle.", ""]
    if result["missing_cases"]:
        lines += ["**Diagnosi incompleta: lista degli ammessi sospesa.**", "",
                  *[f"- {item['role']}: {item['reason']}" for item in result["missing_cases"]], ""]
    for view, title in (("historical_window", "Finestra storica 13,15–17,95 s"), ("full", "Intero episodio")):
        lines += ["## " + title, ""]
        table = []
        for role in EXPECTED:
            m = _get(result, "measurements", role, view)
            table.append((role, _get(m, "episode", "completed"), _get(m, "ankle", "positive_area_nm_s"),
                          _get(m, "tracking", "knee", "actual_rmse_deg"), _get(m, "tracking", "ankle", "actual_rmse_deg"),
                          _get(m, "tracking", "knee", "excursion_deg"), _get(m, "tracking", "ankle", "excursion_deg"),
                          _get(m, "contact", "vertical_impulse_n_s"), _get(m, "episode", "penetration_max_mm")))
        lines += _table(("Policy", "Completo", "Area ankle+ Nm·s", "RMSE knee °", "RMSE ankle °", "ROM knee °", "ROM ankle °", "Impulso N·s", "Pen. ep. mm"), table)
    lines += ["La penetrazione sopra è il massimo dell'episodio usato dai vincoli; il CSV conserva anche il "
              "massimo dei campioni GRF nella finestra. I numeri dei rollout interrotti restano diagnostici: "
              "le curve escludono punti incompleti, le tabelle/CSV li mantengono con lo stato esplicito.", "",
              "## Vincoli globali e per fase, riferimenti affiancati", ""]
    for role in EXPECTED[1:]:
        screening = result["screening"][role]
        lines += [f"### {role}", ""]
        lines += _table(("Categoria", "Falliti vs best80", "Non valutabili vs best80", "Falliti vs controllo pari", "Non valutabili vs controllo pari"),
                        [(category, *[(sum(c["passed"] is outcome for c in constraints if c["role"] == role
                                         and c["reference"] == ref and c["category"] == category)
                                        if ref in screening["comparisons"] else None)
                                      for ref in ("best80", f"control_{role.rsplit('_', 1)[1]}") for outcome in (False, None)])
                         for category in ("tracking_global", "tracking_phase", "range_of_motion", "contact_completion", "regularity", "ankle_objective")])
        for reference, item in screening["comparisons"].items():
            area = item["ankle_area"]
            reduction = area["reduction_fraction"]
            lines += [f"- Area vs {reference}: riduzione {_fmt(None if reduction is None else 100 * reduction)}%; "
                      f"vincoli fisici: {_fmt(item['physical_passed'])}."]
        failures = [c for c in constraints if c["role"] == role and c["passed"] is False]
        if failures:
            lines += ["", "<details><summary>Tutti i vincoli falliti, inclusi quelli per fase</summary>", ""]
            lines += _table(("Riferimento", "Finestra", "Vincolo", "Misurato", "Limite"),
                            [(c["reference"], c["view"], c["name"], c["actual"], c["bound"]) for c in failures])
            lines += ["</details>", ""]
        lines += [""]
    lines += ["## Figure e dati", "", *[f"- [{Path(path).name}]({path})" for path in result["figures"]],
              f"- [Metriche CSV]({metrics_path.as_posix()})", f"- [Vincoli CSV]({constraints_path.as_posix()})", "",
              "Lo screening usa metriche fisiche indipendenti dal ritorno della reward. Confrontare i seed "
              "rumorosi resta necessario prima di qualunque decisione di selezione; le soglie sono operative, "
              "non attestazioni di robustezza statistica.", ""]
    report_path = output / "report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    result["files"] = {"report": str(report_path), "metrics_csv": str(metrics_path), "constraints_csv": str(constraints_path),
                       "json": str(output / "nominal_screening.json")}
    # Ensure the result represents one unchanged evidence snapshot.
    verify_hashes(root, input_hashes)
    result["artifact_hashes"] = {str(path): sha256(path) for path in [report_path, metrics_path, constraints_path,
                                                                    *map(Path, result["figures"])]}
    write_json(output / "nominal_screening.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.experiment)
    print(json.dumps({"status": result["status"], "eligible": result["eligible"], "files": result["files"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
