"""Publish the requested user report only after the complete iteration-50 evaluation.

Standard library only; this module never starts training or rollouts. The receipt
is committed last, after both the user report and its daily TODO propagation.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile


REPO_ROOT = Path(__file__).resolve().parents[3]
STARTS = ("early", "center", "late")
TITLE = "pretraining_unificata_valutazione_intermedia_50"
TODO = (
    "Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale 5/50/100.",
    "Svolgere la fase 5: variante 39, isolamento, smoke, confronto A/B/C e trasferimento ex-novo B/C secondo il protocollo.",
    "Completare la diagnostica dell'osservabilità e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime.",
    "Completare il benchmark di parità con la baseline, includendo costi e variabilità tra seed.",
    "Eseguire le verifiche live Windows; mantenere aperti i debiti storici non risolti.",
)
# These exact existing tasks are discharged by the validated milestone, not by
# passing the gait screen. Other historical tasks remain open without inference.
DONE_50 = (
    "Riprendere la V1 a 35 feature dal checkpoint 5 fino a 50 iterazioni totali, conservando configurazione e stato completo del training.",
    "Fissare i criteri e svolgere la valutazione intermedia a 50 sui tre start, includendo tracking, cicli, primo periodo online, terminazioni e penetrazione.",
    "Completare V1 fino a50 e valutarla sui tre start.",
    "Completare V1 fino a50 e verificarne i risultati intermedi.",
)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def relative(path: Path, root: Path) -> str:
    return Path(os.path.relpath(path, root)).as_posix()


def link(path: Path, report: Path, label: str | None = None) -> str:
    return f"[{label or path.name}](<{relative(path, report.parent)}>)"


def number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"Missing or nonfinite metric: {label}")
    return float(value)


def tracking_cells(metrics: dict) -> list[str]:
    cells = []
    for measure, unit in (("position", "deg"), ("velocity", "deg_s")):
        for joint in ("knee", "ankle"):
            tracking = metrics["tracking"][joint]
            cells.append(" / ".join(f"{number(tracking[source + '_vs_prescribed'][measure]['rmse_' + unit], joint + measure):.3f}"
                                    for source in ("actual", "served")))
    return cells


def validate_sources(evaluation_dir: Path, plan_dir: Path) -> tuple[dict, dict, dict[str, str]]:
    evaluation_path = evaluation_dir / "milestone_evaluation.json"
    evaluation = read_json(evaluation_path)
    selection = evaluation.get("selection", {})
    if evaluation.get("status") != "complete" or selection.get("milestone") != 50:
        raise ValueError("Iteration-50 evaluation is not complete")
    plan = evaluation.get("plan", {})
    if not plan.get("run_dir") or plan.get("pending_rollouts"):
        raise ValueError("Evaluation run is missing or rollouts are pending")
    run = Path(plan["run_dir"]).resolve()
    state = read_json(plan_dir / "execution_state.json")
    if not state.get("run_dir") or Path(state["run_dir"]).resolve() != run:
        raise ValueError("Evaluation belongs to a different plan run")
    summary_path = plan_dir / "training_summary_50.json"
    summary = read_json(summary_path)
    if (summary.get("ok") is not True or summary.get("stop_reason") != "completed"
            or summary.get("iterations_completed") != 50 or summary.get("skipped_iterations")):
        raise ValueError("Training summary does not confirm all 50 iterations")
    if summary.get("output_dir") and Path(summary["output_dir"]).resolve() != run:
        raise ValueError("Training summary belongs to a different run")
    history = summary.get("history", [])
    if [row.get("iteration") for row in history] != list(range(1, 51)):
        raise ValueError("Training history has missing, duplicate or skipped iterations")
    if any(row.get("unified_phase") != "imitation" or row.get("exact_start_balance", {}).get("pass") is not True for row in history):
        raise ValueError("Training history does not establish balanced imitation")
    roles = selection.get("roles", {})
    candidates = selection.get("candidates", [])
    indices = [candidate.get("iteration") for candidate in candidates]
    if (roles.get("last") != 50 or not isinstance(roles.get("best_training_return"), int)
            or not 1 <= roles["best_training_return"] <= 50
            or len(indices) != len(set(indices)) or set(indices) != set(roles.values())):
        raise ValueError("Missing or inconsistent last/best checkpoint selection")
    cases = evaluation.get("cases", [])
    expected_indices = set(indices) | {5}
    if len(cases) != 3 * len(expected_indices):
        raise ValueError("Incomplete or duplicate selected checkpoint/CP5 cases")
    for iteration in expected_indices:
        selected = [case for case in cases if case.get("checkpoint_iteration") == iteration]
        if len(selected) != 3 or {case.get("start") for case in selected} != set(STARTS):
            raise ValueError(f"CP{iteration}: missing or duplicate start")
        gate = evaluation.get("quality_gates", {}).get(str(iteration), {})
        if not isinstance(gate.get("pass"), bool) or not isinstance(gate.get("failures"), list):
            raise ValueError(f"CP{iteration}: missing quality gate")
        for case in selected:
            metrics = case["full_episode"]
            if number(metrics["duration_s"], "duration_s") <= 0:
                raise ValueError("Empty evaluated episode")
            tracking_cells(metrics)
            for key in ("valid_hs_count", "valid_to_count", "valid_cycle_count"):
                number(metrics["phase"][key], key)
            number(metrics["safety"]["penetration_max_mm"], "penetration_max_mm")
            if not metrics["termination"].get("end_reason"):
                raise ValueError("Missing episode termination")
            if case["first_online_period_available_elapsed_s"] is not None:
                number(case["first_online_period_available_elapsed_s"], "first online period")
    for start in STARTS:
        pairs = [pair for pair in evaluation.get("common_prefix_comparisons", [])
                 if pair.get("start") == start and set(pair.get("iterations", [])) == {5, 50}]
        if len(pairs) != 1 or set(pairs[0].get("metrics", {})) != {"5", "50"}:
            raise ValueError(f"Missing or duplicate CP5/CP50 common prefix: {start}")
        durations = []
        for metrics in pairs[0]["metrics"].values():
            tracking_cells(metrics)
            durations.append(number(metrics["duration_s"], "common prefix duration"))
        if min(durations) <= 0 or abs(durations[0] - durations[1]) > 1e-8:
            raise ValueError("Mismatched common-prefix durations")
    figures = evaluation.get("figures", [])
    if not figures:
        raise ValueError("Evaluation figures are not complete")
    sources = [evaluation_path, summary_path, evaluation_dir / "metrics.csv", evaluation_dir / "milestone_evaluation.md"]
    for name in figures:
        path = (evaluation_dir / name).resolve()
        path.relative_to(evaluation_dir)
        sources.append(path)
    hashes = {}
    for path in sources:
        if not path.is_file() or not path.stat().st_size:
            raise ValueError(f"Missing evaluation artifact: {path}")
        hashes[str(path)] = sha256(path)
    return evaluation, summary, hashes


def checkbox_blocks(text: str, *, checked: bool = False) -> list[str]:
    """Keep checkbox items and their indented continuation lines, not prose."""
    result, current = [], []
    pattern = r"^\s*[-*] \[[xX]\] " if checked else r"^\s*[-*] \[ \] "
    for line in text.splitlines() + [""]:
        if re.match(pattern, line):
            if current:
                result.append("\n".join(current))
            current = [line]
        elif current and line.startswith(("  ", "\t")) and line.strip():
            current.append(line)
        else:
            if current:
                result.append("\n".join(current))
            current = []
    return result


def checkbox_key(block: str) -> str:
    return " ".join(re.sub(r"^\s*[-*] \[[ xX]\] ", "", block).split())


def open_todos(previous: Path | None, existing_daily: str) -> list[str]:
    blocks = checkbox_blocks(previous.read_text(encoding="utf-8")) if previous else []
    blocks += checkbox_blocks(existing_daily)
    closed = set()
    for source in (previous.read_text(encoding="utf-8") if previous else "", existing_daily):
        closed.update(checkbox_key(block) for block in checkbox_blocks(source, checked=True))
    result, seen = [], set(DONE_50) | closed
    for block in blocks + ["- [ ] " + item for item in TODO]:
        key = checkbox_key(block)
        if key not in seen:
            result.append(block)
            seen.add(key)
    return result


def render_report(evaluation: dict, summary: dict, report: Path, evaluation_dir: Path,
                  plan_dir: Path, timestamp: datetime, todos: list[str], previous: Path | None) -> str:
    roles = evaluation["selection"]["roles"]
    lines = ["# Pretraining unificata: valutazione intermedia a 50 iterazioni", "",
             f"Redatto: {timestamp.isoformat()}. Instruction check token: `CMC_AGENT_OK_2026`.", "",
             "## Problema", "",
             "Verificare se la V1 a 35 feature, con reward imitativa e clock prescribed disabilitato, "
             "migliora dopo il resume dal checkpoint 5 e sviluppa cicli validi ai tre start. "
             "Il periodo online resta da distinguere dalla stabilità del cammino.", "",
             "## Soluzione e risultato intermedio", "",
             f"Training completato a **50/50 iterazioni**; **45 aggiornamenti dopo CP5**. "
             f"La valutazione è completa per {len(evaluation['cases'])} casi: CP5, ultimo CP50 "
             f"e best di training CP{roles['best_training_return']} (deduplicati se coincidono). "
             "Il best è selezionato dal return, senza implicare qualificazione.", ""]
    for iteration, gate in sorted(evaluation["quality_gates"].items(), key=lambda item: int(item[0])):
        detail = ", ".join(gate["failures"]) or "nessuna violazione dei criteri dello screening"
        lines.append(f"- CP{iteration}: screening **{'superato' if gate['pass'] else 'non superato'}**; {detail}.")
    lines += ["", "Lo screening è una condizione necessaria di fattibilità deterministica. "
              "Non dimostra parità con la baseline, robustezza statistica o qualità del tracking. "
              "La sincronizzazione e l'osservabilità rimangono oggetto della fase 5.", "",
              "## Strategia", "",
              "Confronto per start e giunto con seed 123, azioni deterministiche, massimo 500 passi a 10 ms "
              "(5 s). Actual e served sono confrontati con il prescribed al timestamp allineato. "
              "Le finestre complete riportano l'intero episodio osservato; CP5↔CP50 usa il prefisso "
              "temporale comune, senza estrapolazione o aggregazione degli start sopravvissuti.", "",
              "### Episodi completi", "",
              "| Start | CP / ruolo | Durata [s] | HS netti / annullati / TO / cicli | Primo periodo disponibile [s] | Pen max [mm] | Fine |",
              "|---|---|---:|---:|---:|---:|---|"]
    ordered = sorted(evaluation["cases"], key=lambda case: (STARTS.index(case["start"]), case["checkpoint_iteration"]))
    for case in ordered:
        m, first = case["full_episode"], case["first_online_period_available_elapsed_s"]
        p = m["phase"]
        role = ", ".join(case["roles"]) or "riferimento CP5"
        lines.append(f"| {case['start']} | {case['checkpoint_iteration']} / {role} | {m['duration_s']:.3f} | "
                     f"{p['valid_hs_count']} / {p.get('hs_cancelled_count', 'n.d.')} / {p['valid_to_count']} / {p['valid_cycle_count']} | "
                     f"{first if first is not None else 'non disponibile'} | {m['safety']['penetration_max_mm']:.3f} | "
                     f"{m['termination']['end_reason']} |")
    lines += ["", "HS è il conteggio netto della FSM. La versione v3 può annullare un appoggio riconosciuto come rimbalzo; "
              "il valutatore ammette tali decrementi solo se giustificati da cancellazioni registrate. "
              "Nelle sottofinestre il saldo HS può essere negativo se viene annullato un appoggio precedente. "
              "Le cancellazioni sono esposte separatamente, senza contarle come appoggi validi.", "",
              "Il primo periodo è il tempo trascorso dal reset alla prima osservazione pre-step con periodo > 0; "
              "non è la durata del ciclo. Startup e regime restano separati nel JSON; un regime nullo indica "
              "che il periodo non è diventato disponibile.", "", "### RMSE sugli episodi completi", "",
              "Ogni cella riporta **actual / served** rispetto al prescribed.", "",
              "| Start | CP | Ginocchio q [°] | Caviglia q [°] | Ginocchio qdot [°/s] | Caviglia qdot [°/s] |",
              "|---|---:|---:|---:|---:|---:|"]
    for case in ordered:
        lines.append(f"| {case['start']} | {case['checkpoint_iteration']} | " + " | ".join(tracking_cells(case["full_episode"])) + " |")
    lines += ["", "### CP5 ↔ CP50: prefisso comune per start", "",
              "| Start | CP | Finestra [s] | Ginocchio q actual/served [°] | Caviglia q actual/served [°] | Ginocchio qdot actual/served [°/s] | Caviglia qdot actual/served [°/s] |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for start in STARTS:
        pair = next(pair for pair in evaluation["common_prefix_comparisons"] if pair["start"] == start and set(pair["iterations"]) == {5, 50})
        for iteration in ("5", "50"):
            m = pair["metrics"][iteration]
            lines.append(f"| {start} | {iteration} | {m['duration_s']:.3f} | " + " | ".join(tracking_cells(m)) + " |")
    lines += ["", "Ulteriori confronti con il best, SEA, saturazione e penetrazione sopra 20/25/28 mm: "
              + link(evaluation_dir / "milestone_evaluation.md", report, "analisi completa") + " e "
              + link(evaluation_dir / "metrics.csv", report) + ".", "", "### Fonti e figure", "",
              "- " + link(evaluation_dir / "milestone_evaluation.json", report),
              "- " + link(plan_dir / "training_summary_50.json", report),
              "- " + link(plan_dir / "STATUS.md", report, "stato del piano successivo")]
    lines += ["- " + link(evaluation_dir / name, report) for name in evaluation["figures"]]
    if previous:
        lines.append("- " + link(previous, report, "daily precedente e TODO storici"))
    for filename in ("2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md", "2026-09-14_pilot_unificata_strategia_50_100_e_test_39_feature.md"):
        source = report.parent / filename
        if source.exists():
            lines.append("- " + link(source, report))
    lines += ["", "## File modificati", "",
              "Questa redazione genera il presente report, aggiunge la sintesi e i TODO al daily della "
              "stessa data e salva `create_report_50_receipt.json` nella directory del piano. "
              "L'automazione è in `Trajectory Generator/unified_MLP/validation/create_milestone_report.py`; "
              "`validation/compare_pretraining_checkpoints.py` e `validation/evaluate_training_milestone.py` "
              "gestiscono i contatori netti e le cancellazioni HS, con verifiche nei rispettivi test. "
              "valutatore e coordinatore sono documentati nel report di avvio. "
              "Le sorgenti del training e il plugin SEA non vengono modificati dalla redazione.", "",
              "## Test e verifiche eseguite", "",
              "- Validati status completo, milestone 50, identità del run, history 1–50 senza salti, "
              "reward imitativa e bilanciamento dei tre start.",
              "- Verificati tutti i casi last/best/CP5, metriche finite, tre prefissi comuni CP5/CP50 "
              "e presenza di JSON, CSV, Markdown e figure; hash delle fonti nella ricevuta.",
              f"- Summary archiviato: `ok={summary['ok']}`, `stop_reason={summary['stop_reason']}`, "
              f"`iterations_completed={summary['iterations_completed']}`; "
              f"restart totali: {summary.get('restart_count', 'non riportati')}; "
              f"crash restart: {summary.get('crash_restart_count', 'non riportati')}; "
              f"iterazioni saltate: {summary.get('skipped_iterations', [])}.",
              "- I conteggi di restart appartengono al summary archiviato del resume completato. "
              "Il fallimento iniziale sandbox `psutil/sysctl` resta un tentativo precedente, "
              "documentato nel report di avvio; non viene inferito dai file di stato mutabili."]
    if summary.get("elapsed_wall_time_s") is not None:
        lines.append(f"- Tempo wall dichiarato dal summary: {number(summary['elapsed_wall_time_s'], 'elapsed_wall_time_s'):.1f} s, "
                     "relativo all'ambito di quel processo/resume; non viene presentato come costo totale CP1–50.")
    lines += ["- Questa redazione non esegue nuovi test, training o rollout: riporta i risultati runtime "
              "già conclusi. La verifica live Windows rimane aperta.", "", "## TODO aperti", "",
              "Le fasi fino a 50 e la valutazione intermedia sono concluse come esecuzione. "
              "Gli obiettivi di qualità non sono risolti automaticamente da questo completamento. "
              "I TODO storici seguenti sono propagati senza riavviare campagne precedenti.", ""]
    lines += todos
    return "\n".join(lines) + "\n"


def create_report(evaluation_dir: Path, plan_dir: Path, *, repo_root: Path | None = None,
                  now: datetime | None = None) -> dict:
    """Publish once; reject incomplete evidence and changed sources on replay."""
    evaluation_dir, plan_dir = Path(evaluation_dir).resolve(), Path(plan_dir).resolve()
    root = Path(repo_root or REPO_ROOT).resolve()
    evaluation, summary, absolute_hashes = validate_sources(evaluation_dir, plan_dir)
    hashes = {relative(Path(path), root): digest for path, digest in absolute_hashes.items()}
    receipt_path = plan_dir / "create_report_50_receipt.json"
    intent_path = plan_dir / "create_report_50_intent.json"
    lock = plan_dir / "create_report_50.lock"
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as exc:
        raise RuntimeError("Iteration-50 report publication is already running") from exc
    os.close(fd)
    try:
        if receipt_path.exists():
            receipt = read_json(receipt_path)
            if receipt.get("status") != "complete" or receipt.get("source_sha256") != hashes:
                raise ValueError("Report receipt conflicts with the completed evaluation sources")
            report = root / receipt["report_path"]
            daily = root / receipt["daily_path"]
            if sha256(report) != receipt["report_sha256"] or receipt["daily_marker"] not in daily.read_text(encoding="utf-8"):
                raise ValueError("Published report or daily receipt evidence changed")
            intent_path.unlink(missing_ok=True)
            return receipt
        if intent_path.exists():
            intent = read_json(intent_path)
            if intent.get("source_sha256") != hashes:
                raise ValueError("Unfinished report intent conflicts with evaluation sources")
            timestamp = datetime.fromisoformat(intent["created_at"])
        else:
            timestamp = now or datetime.now().astimezone()
        date = timestamp.date().isoformat()
        report = root / "reports" / "user" / f"{date}_{TITLE}.md"
        daily = root / "reports" / "daily" / f"{date}_daily-report.md"
        previous_daily = sorted(path for path in daily.parent.glob("????-??-??_daily-report.md") if path.name[:10] < date)
        previous = previous_daily[-1] if previous_daily else None
        existing_daily = daily.read_text(encoding="utf-8") if daily.exists() else f"# Daily Report - {date}\n"
        todos = open_todos(previous, existing_daily)
        content = render_report(evaluation, summary, report, evaluation_dir, plan_dir, timestamp, todos, previous)
        if report.exists() and report.read_text(encoding="utf-8") != content:
            raise ValueError(f"Refusing to overwrite an existing user report: {report}")
        marker = "<!-- create_report_50:" + hashlib.sha256(str(plan_dir).encode()).hexdigest()[:16] + " -->"
        block = "\n\n" + marker + "\n\n## Valutazione intermedia V1 a 50 iterazioni\n\n"
        block += link(report, daily, "Report utente della valutazione") + ". Training 50/50 e valutazione last/best/CP5 "
        block += "sui tre start completati. Screening e parità restano distinti; dati e figure nel report.\n\n"
        block += "- [x] Completare V1 fino a50 e valutarla sui tre start.\n\n"
        block += "I precedenti TODO relativi esclusivamente al training e alla valutazione a 50 sono esauriti. "
        block += "Le altre sezioni di questo daily sono conservate.\n\n### TODO aperti propagati\n\n"
        block += "\n".join(todos) + "\n"
        # Persist the publication date before the first document write, so a
        # process interruption can resume even after local midnight.
        if not intent_path.exists():
            atomic_write(intent_path, json.dumps({"created_at": timestamp.isoformat(), "source_sha256": hashes}, indent=2) + "\n")
        atomic_write(report, content)
        if marker not in existing_daily:
            atomic_write(daily, existing_daily.rstrip() + block)
        receipt = {"schema_version": 1, "status": "complete", "milestone": 50,
                   "created_at": timestamp.isoformat(), "report_date": date,
                   "evaluation_dir": relative(evaluation_dir, root), "run_dir": evaluation["plan"]["run_dir"],
                   "report_path": relative(report, root), "daily_path": relative(daily, root),
                   "report_sha256": sha256(report), "daily_marker": marker, "source_sha256": hashes,
                   "todo_source": relative(previous, root) if previous else None}
        atomic_write(receipt_path, json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        intent_path.unlink(missing_ok=True)
        return receipt
    finally:
        lock.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation-dir", type=Path, required=True)
    parser.add_argument("--plan-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(create_report(args.evaluation_dir, args.plan_dir), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
