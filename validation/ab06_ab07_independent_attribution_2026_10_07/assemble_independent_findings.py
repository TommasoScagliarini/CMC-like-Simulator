"""Assemble the blind local audit from reviewed numerical artifacts.

No Claude response is read here. Operational inputs and prior evidence are
read-only; findings are created exclusively in this isolated audit directory.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT / "validation/ab06_ab07_causal_investigation_2026_10_07"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"),
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def fp(path: Path) -> dict:
    return {"path": path.relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main() -> None:
    paths = {
        "geometry": HERE / "geometry/functional_ankle_geometry.json",
        "common_rebuild_contact": HERE / "rebuild/contact_replay_common_procedure.json",
        "fresh_healthy_control": HERE / "healthy_ik/supplemental_healthy_geometry.json",
        "preparation_provenance": HERE / "provenance/preparation_provenance_protocol.json",
        "protocol_review": HERE / "review/preparation_protocol_review.json",
        "common_rebuild_review": HERE / "review/common_rebuild_review.json",
        "common_rebuild_model_delta": HERE / "review/common_rebuild_model_delta.json",
        "prior_causal_evidence": OLD / "causal_findings.json",
        "common_rebuild_protocol": HERE / "rebuild/protocol.json",
    }
    evidence = {key: read(path) for key, path in paths.items()}
    geo, contact, fresh, prior = (evidence[key] for key in (
        "geometry", "common_rebuild_contact", "fresh_healthy_control", "prior_causal_evidence"))
    protected = evidence["common_rebuild_protocol"]["inputs_before"]
    changed = [p for p, digest in protected.items()
               if not (ROOT/p).is_file() or fp(ROOT/p)["sha256"] != digest]
    if changed:
        raise AssertionError({"changed_original_inputs": changed})
    if not fresh["inputs_integrity"]["unchanged"] or not contact["inputs_integrity"]["all_unchanged"]:
        raise AssertionError("Evidence source integrity failed")
    original = geo["exact_subject_difference_budget"]["stance"]["joint_center_Y_budget_mm"]
    uniform = fresh["subject_difference_budgets"]["stance"]["fresh_healthy"]["joint_center_Y_budget_mm"]
    pair = contact["pair_comparison"]
    if pair["old_case"] != pair["common_procedure_case"]:
        # World transform reconstruction may differ below floating roundoff.
        for key in pair["old_case"]:
            if abs(pair["old_case"][key]-pair["common_procedure_case"][key]) > 1e-12:
                raise AssertionError("Common-procedure contact endpoint changed")
    builds = {}
    for subject in ("AB06", "AB07"):
        report_path = HERE / "rebuild/models" / f"{subject}_SEASEA_Threadmill/pipeline_report.json"
        report = read(report_path)
        builds[subject] = {
            "report": fp(report_path), "status": report["status"],
            "fresh_full_marker_ik": report["ik_provenance"],
            "frozen_reference_inputs": report["preparation_references"]["inputs"],
            "mounting_fit": report["prosthesis_fit"],
            "registration_adjustments": report["ground_registration"]["adjustments"],
            "source_healthy_mass_kg": report["mass_before_kg"],
            "prepared_mass_kg": report["mass_after_kg"],
        }
        if report["status"] != "complete" or report["ik_method"] != "opensim_marker_ik":
            raise AssertionError("Complete fresh marker IK required")
        if contact["subjects"][subject]["pose_changes"]["total_force_new_minus_old_max_abs_N"] != 0:
            raise AssertionError("Unexpected contact endpoint change")
    for key in builds["AB06"]["frozen_reference_inputs"]:
        if builds["AB06"]["frozen_reference_inputs"][key]["sha256"] != builds["AB07"]["frozen_reference_inputs"][key]["sha256"]:
            raise AssertionError("Reference snapshots differ")
    old_cycle = [c["stance"]["joint_center_Y_budget_mm"] for c in geo["per_cycle_subject_difference_budget"]]
    fresh_cycles = []
    for i in range(3):
        a, b = (fresh["subjects"][s]["cases"]["healthy_fresh"]["cycles"][i]["stance"]["joint_center_Y_time_weighted_mean_m"]
                for s in ("AB06", "AB07"))
        fresh_cycles.append((b-a)*1000)
    text = (
        "AUDIT INDIPENDENTE CODEX — AB06 / AB07 — 2026-10-07\n\n"
        "Obiettivo: distinguere anatomia/marcia originale, procedura diversa di preparazione, "
        "e risposta della costruzione protesica al soggetto e al contatto. "
        "Le conclusioni sono congelate prima di leggere il nuovo audit Claude.\n\n"
        "Conclusione: la spiegazione sostenuta è la risposta congiunta di costruzione/montaggio, "
        "calibrazione dei marker e nuova IK, valutata con un contatto empirico calibrato su AB06. "
        "Uniformare la procedura riproduce esattamente il divario; non emerge un difetto di esecuzione "
        "specifico di AB07. Non è dimostrata una causa anatomica esclusiva né una preparazione fisicamente errata.\n\n"
        f"Nel modello healthy, la differenza media in appoggio del centro caviglia AB07−AB06 è "
        f"{original['healthy_AB07_minus_AB06']:+.6f} mm con IK originale e "
        f"{uniform['healthy_AB07_minus_AB06']:+.6f} mm con nuova IK uniforme. "
        f"Dopo la costruzione protesica diventa {uniform['prepared_AB07_minus_AB06']:+.6f} mm. "
        f"La risposta differenziale costruzione+IK rispetto al controllo healthy uniforme è "
        f"{uniform['differential_construction_response_AB07_minus_AB06']:+.6f} mm. "
        "È un budget osservato tra meccanismi diversi, non una quota causale dell'anatomia.\n\n"
        f"Rispetto alla propria caviglia healthy con solver comune, la caviglia protesica viene rialzata "
        f"di {uniform['prepared_minus_healthy_AB06']:.6f} mm per AB06 e "
        f"{uniform['prepared_minus_healthy_AB07']:.6f} mm per AB07. "
        "AB07 è quindi rialzato meno di AB06; non viene abbassato rispetto al suo originale.\n\n"
        "Rebuild completo: stessi byte donor, healthy reference, static reference, plugin e profili; "
        "dati raw propri, anatomical fit, ground registration, marker calibration, nuova full-marker IK, "
        "nessuno ScaleTool, RRA, rollout dinamico, training o refit del contatto. "
        "Entrambe le nuove IK sono byte-identiche alle precedenti; il modello AB07 è byte-identico, "
        "AB06 differisce solo per ordine delle forze e arrotondamenti documentati. "
        "Tutti i q/qdot e tutte le forze del replay risultano identici.\n\n"
        f"Replay coerente dei riferimenti filtrati nei tre cicli: AB06 "
        f"{contact['subjects']['AB06']['common_procedure_case']['aggregate']['mean_force_N']:.6f} N, "
        f"AB07 {contact['subjects']['AB07']['common_procedure_case']['aggregate']['mean_force_N']:.6f} N; "
        f"divario {pair['common_procedure_case']['AB07_minus_AB06_mean_force_N']:.6f} N. "
        "Sono forze del replay FK, non delle traiettorie di un nuovo rollout. "
        "La componente left_basis_07, sulla posa filtrata congelata, spiega algebricamente "
        "+81.66 N del divario, mentre le altre sette danno −1.03 N; non è una prova di rimozione fisica.\n\n"
        "I precedenti controlli coerenti AB07 prima/dopo ground-registration modificano la forza media "
        "da 22.24 a 356.56 N. Identificano una forte sensibilità della preparazione, "
        "ma la vecchia preparazione era gravemente sottocaricata, quindi non provano che la correzione sia sbagliata. "
        "Il bilancio verticale dei rollout precedenti chiude grazie alle reserve (+60.28 N AB06, −131.76 N AB07), "
        "senza bisogno di una nuova causa nascosta nel carico destro o nel peso.\n\n"
        "Limiti: anatomy e gait originali restano accoppiati; il montaggio è funzione dell'anatomia. "
        "Il graft sostituisce articolazioni e hardware, calibra 12 marker e risolve una nuova IK; "
        "il centro caviglia è funzionalmente omologo, gli assi e la suola non sono fisicamente equivalenti. "
        "La registrazione replica una convenzione statica del donor, non un pavimento protesico misurato. "
        "Il confronto GRF healthy/protesi virtuale è un benchmark, non una misura dell'hardware. "
        "Il callback finale storico AB06 resta fallito; i tre cicli analizzati precedono quel bordo.\n\n"
        "Direzione: mantenere l'anatomia del soggetto; validare la relazione montaggio–marker–IK–terreno "
        "e il trasferimento della legge di contatto. Occorrono vincoli geometrici indipendenti o una qualificazione "
        "dinamica disgiunta per stabilire quale parametro sia fisicamente sbagliato, senza correggere il contatto "
        "solo mediante reserve o un guadagno comune. Nessuna correzione operativa applicata nell'audit.\n"
    )
    findings = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026", "branch": "Codex",
        "status": "local_independent_audit_complete_before_new_Claude_read",
        "new_Claude_assistant_content_read": False,
        "conclusion_it": text,
        "stance_joint_center_budget_mm": {"original_healthy_IK": original, "uniform_healthy_IK": uniform},
        "uniform_solver_healthy_subject_delta_minus_original_mm": uniform["healthy_AB07_minus_AB06"]-original["healthy_AB07_minus_AB06"],
        "coherent_contact_pair": pair,
        "new_builds": builds,
        "prior_observed_basis_force_gaps": prior["basis_mean_force_gaps"],
        "prior_registration_effects": prior["registration_effects"],
        "qualification_it": "La procedura comune non implica contatto qualificato sui due soggetti. L'effetto della costruzione non equivale a errore né a esclusione dell'anatomia mediata dal montaggio.",
        "verification": {"original_input_count": len(protected), "original_inputs_unchanged": True,
                         "common_contact_inputs_unchanged": contact["inputs_integrity"]["all_unchanged"],
                         "common_contact_consumed_count": contact["inputs_integrity"]["consumed_count"],
                         "supplemental_healthy_inputs_unchanged": fresh["inputs_integrity"]["unchanged"]},
        "evidence_artifacts": {key: fp(path) for key, path in paths.items()}, "helper": fp(Path(__file__)),
        "plot_facts": {
            "stance_joint_center_AB07_minus_AB06_mm": [original["healthy_AB07_minus_AB06"], uniform["healthy_AB07_minus_AB06"], uniform["prepared_AB07_minus_AB06"]],
            "per_cycle_stance_joint_center_AB07_minus_AB06_mm": [[c["healthy_AB07_minus_AB06"] for c in old_cycle], fresh_cycles, [c["prepared_AB07_minus_AB06"] for c in old_cycle]],
            "reference_FK_mean_force_N": {s: [contact["subjects"][s][case]["aggregate"]["mean_force_N"] for case in ("old_case", "common_procedure_case")] for s in ("AB06", "AB07")},
        },
    }
    for filename, payload in (("codex_independent_findings.json", json.dumps(findings, indent=2, ensure_ascii=False, allow_nan=False)+"\n"), ("codex_independent_findings.txt", text)):
        with (HERE/filename).open("x", encoding="utf-8") as stream:
            stream.write(payload)
    print(json.dumps({"output": fp(HERE/"codex_independent_findings.json"), "original_inputs_unchanged": True}, indent=2))


if __name__ == "__main__":
    main()
