"""Assemble reviewed causal evidence and recheck explicit input registries.

This reads saved diagnostics only. It performs no FK, dynamics, calibration or
operational file edit. Run after all three external-review turns have completed.
"""
from __future__ import annotations

import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[1]


def read(path):
    def reject(value):
        raise ValueError("Nonfinite JSON token: " + value)
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    files = {
        "geometry": "model/factorial_geometry_fk.json",
        "balance": "alternative/vertical_impulse_audit.json",
        "scalar": "alternative/contact_scale_transfer.json",
        "phase": "alternative/phase_contact_budget.json",
        "markers": "model/signed_marker_vertical_audit.json",
        "pose": "model/foot_pose_decomposition.json",
        "basis": "contact_basis_ablation.json",
        "review_geometry_balance": "review/contact_peer_review.json",
        "review_phase_markers": "review/phase_marker_peer_review.json",
        "review_pose": "review/foot_pose_peer_review.json",
        "review_claude_claims": "review/claude_claims_review.json",
    }
    evidence = {key: read(BASE / rel) for key, rel in files.items()}
    for key, row in evidence.items():
        critique_complete = (key == "review_claude_claims" and
                             row["status"] == "completed_scientific_review_with_material_corrections")
        if not row["status"].startswith("PASS") and not critique_complete:
            raise AssertionError("Unresolved evidence/review: " + key)

    # Only explicitly declared current-input registries are combined. Historical
    # hashes embedded in narrative/source-lineage metadata are not substituted.
    protected = {}

    def add(registry):
        for path, expected in registry.items():
            if path in protected and protected[path] != expected:
                raise AssertionError("Conflicting protected input: " + path)
            protected[path] = expected

    baseline = read(BASE / "baseline_sources_before.json")
    add({row["path"]: row["before_sha256"] for row in baseline["sources"]})
    for key in ("geometry", "balance", "scalar", "phase", "markers", "pose"):
        integrity = evidence[key]["inputs_integrity"]
        if not integrity["all_unchanged"]:
            raise AssertionError("Recorded input mutation: " + key)
        add(integrity.get("before", integrity.get("fingerprints_before")))
    add({row["path"]: row["sha256"] for row in evidence["basis"]["inputs_integrity"]["before"]})
    current = {path: digest(ROOT / path) for path in protected}
    changed = [path for path in protected if protected[path] != current[path]]
    if changed:
        raise AssertionError("Protected sources changed: " + repr(changed))

    metadata = {}
    for turn in (1, 2, 3):
        rows = read(BASE / f"claude/turn{turn}_metadata.json")
        init = next(row for row in rows if row.get("subtype") == "init")
        result = next(row for row in rows if row.get("type") == "result")
        if (init["model"] != "claude-fable-5-1" or init["tools"] or
                init["mcp_servers"] or result["is_error"] or result["subtype"] != "success"):
            raise AssertionError("External collaboration contract failed")
        metadata[str(turn)] = {
            "model_verified": init["model"], "session_id": init["session_id"],
            "tools": init["tools"], "mcp_servers": init["mcp_servers"],
            "status": result["subtype"], "effort_flag": "max",
            "visible_response": f"claude/turn{turn}_response.txt",
        }
    if len({row["session_id"] for row in metadata.values()}) != 1:
        raise AssertionError("External reviews did not resume the same chat")
    consent = read(BASE / "claude/capsule_scope_receipt.json")
    if not consent["user_consent"]["received"] or consent["broad_context_sent"]:
        raise AssertionError("External payload consent/scope mismatch")

    summaries = {}
    for subject in ("AB06", "AB07"):
        balance = evidence["balance"]["subjects"][subject]["cases"]
        actual = balance["virtual_actual_saved"]["aggregate"]
        healthy = balance["healthy_original_IK_filtered"]["aggregate"]
        virtual = balance["virtual_prepared_IK_filtered"]["aggregate"]
        markers = evidence["markers"]["subjects"][subject]["dynamic"]
        phase = evidence["phase"]["subjects"][subject]
        summaries[subject] = {
            "actual_vertical_balance": actual,
            "healthy_to_virtual_required_left_impulse_change_Ns":
                virtual["left_required_with_measured_right_and_no_pelvis_support_Ns"] -
                healthy["left_required_with_measured_right_and_no_pelvis_support_Ns"],
            "filtered_reference_phase": phase["filtered_IK"]["aggregate"],
            "actual_phase": phase["actual_saved"]["aggregate"],
            "signed_marker_stance_mean_mm": {
                case: value["residuals"]["aggregate"]["measured_stance"]["five_marker_mean_signed_Y_mm"]
                for case, value in markers.items()},
            "static_force_qualification": evidence["markers"]["subjects"][subject]["static_force_qualification"],
        }

    findings = [
        {
            "id": "contact_reference_mechanism", "kind": "confirmed_observed_mechanism",
            "text_it": "Il divario esiste già con i riferimenti IK, senza rete: AB07 ha maggiore intensità di contatto durante l'appoggio, pur con appoggio misurato più breve. Sulle pose congelate la differenza assoluta passa dalla componente left_basis_07 della base empirica condivisa.",
            "evidence": ["basis", "phase", "pose"],
            "limit_it": "La componente07 è una base matematica che condivide centro/raggio con06, con materiale diverso; non è un sito hardware fisico distinto. L'ablazione non è una soluzione o un nuovo rollout.",
        },
        {
            "id": "preparation_stage", "kind": "confirmed_conditional_FK_effect",
            "text_it": "La registrazione AB07 modifica fortemente il contatto: le coppie modello/IK coerenti archiviate e correnti passano da22.240 a356.561N medi nel replay. Il vecchio banco aveva scarso appoggio; l'incremento non dimostra un errore nella correzione.",
            "evidence": ["geometry", "markers"],
            "limit_it": "Le celle ibride quantificano sensibilità della funzione, non risultati dinamici di preparazioni valide. Non stimano la preparazione totale dal modello sano.",
        },
        {
            "id": "anatomy_scope", "kind": "limited_exclusion",
            "text_it": "La sola differenza conservata di offset dell'anca AB07, a montaggio e movimento fissati, riduce il contatto. Non spiega da sola l'aumento. Il confronto healthy/virtual cambia l'impulso richiesto dal bilancio di meno di1Ns, senza una grande deriva netta del COM.",
            "evidence": ["geometry", "balance"],
            "limit_it": "Non è esclusa l'anatomia tramite marcia, masse/inertie, recruitment o effetti intra-ciclo. Il confronto healthy/virtual non è un intervento mass-only.",
        },
        {
            "id": "marker_registration", "kind": "negative_discriminant",
            "text_it": "I cinque marker non mostrano un bias uniforme AB07 verso il basso. Il fit statico è su dati di calibrazione e manca una GRF statica indipendente; non qualifica la forza dell'hardware.",
            "evidence": ["markers", "review_phase_markers"],
            "limit_it": "I marker virtuali rimappati e calibrati non vincolano indipendentemente la relazione hardware-terreno-base di forza. Un RMS piccolo non prova la correttezza fisica.",
        },
        {
            "id": "pelvis_reserve", "kind": "confirmed_operational_compensation",
            "text_it": "Le reserve verticali del bacino aggiungono60.28N medi inAB06 e sottraggono131.76N inAB07. Con esse il bilancio verticale chiude entro0.066Ns: il buon tracking può nascondere un contatto incompatibile con il carico richiesto.",
            "evidence": ["balance"],
            "limit_it": "Il bilancio localizza la compensazione, non identifica l'origine esclusiva della diversa penetrazione.",
        },
        {
            "id": "common_scalar", "kind": "rejected_within_frozen_pose_test",
            "text_it": "Un solo guadagno di forza fittato sul primo ciclo AB06 non trasferisce: migliora i successivi AB06 ma porta AB07 a circa+67..70% di scostamento d'impulso. La semplice correzione scalare comune è insufficiente.",
            "evidence": ["scalar"],
            "limit_it": "Test posthoc su pose fissate, non validazione prospettica o prova dell'esito di una nuova dinamica.",
        },
        {
            "id": "exclusive_attribution", "kind": "not_identifiable_with_current_evidence",
            "text_it": "La diagnosi sostenuta è un problema di compatibilità/trasferimento fra posa del piede preparato, riferimento cinematico e base empirica di contatto, con compensazione delle reserve. Non è provata una causa anatomica esclusiva AB07 né un errore software specifico di preparazione.",
            "evidence": ["geometry", "phase", "markers", "pose", "balance", "review_claude_claims"],
            "limit_it": "Movimento, anatomia e registrazione restano accoppiati; non esiste una quota causale unica. La GRF di un soggetto sano è un benchmark per l'arto virtuale, non una misura dell'hardware protesico.",
        },
    ]
    artifacts = [{"path": (BASE / rel).relative_to(ROOT).as_posix(), "sha256": digest(BASE / rel)}
                 for rel in files.values()]
    for turn in (1, 2, 3):
        for name in (f"turn{turn}_response.txt", f"turn{turn}_metadata.json", f"numerical_capsule_turn{turn}.txt"):
            path = BASE / "claude" / name
            artifacts.append({"path": path.relative_to(ROOT).as_posix(), "sha256": digest(path)})
    figures = []
    for name in ("figure_rendering.json", "phase_figure_rendering.json"):
        manifest = BASE / name
        if manifest.exists():
            row = read(manifest)
            for item in row["outputs"]:
                if digest(ROOT / item["path"]) != item["sha256"]:
                    raise AssertionError("Figure changed: " + item["path"])
                figures.append(item)
    python_files = sorted(BASE.rglob("*.py"))
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    result = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "causal_investigation_complete_mechanism_identified_exclusive_attribution_not_identifiable",
        "scope": "Saved-data/FK diagnostics, conditional interventions, independent scientific review and three external Claude turns. No new dynamic rollout, training, operational model/profile/plugin/checkpoint edit.",
        "findings": findings, "subject_summaries": summaries,
        "basis_mean_force_gaps": evidence["basis"]["cases"],
        "registration_effects": evidence["geometry"]["registration_lineage_conditional_effects_n"],
        "pose_budget": evidence["pose"]["aggregate"],
        "external_collaboration": {"turns": metadata, "user_limited_consent": True,
                                   "scope_receipt": "claude/capsule_scope_receipt.json",
                                   "first_turn_claims_corrected_by_peer_review": "review/claude_claims_review.json",
                                   "second_turn_precision_corrected_in_turn3": [
                                       "Raw31mm late-stance gap is not31mm of positive penetration; last bin positive gap0.111mm.",
                                       "No demonstrated preparation error is not equivalent to proof that all preparation is correct.",
                                       "Tracking/reserves do not eliminate every indirect mass effect in integrated dynamics."]},
        "evidence_artifacts": artifacts, "figures": figures,
        "verification": {"independent_local_experiment_reviews_pass": True,
                         "external_claims_review_complete_with_corrections": True,
                         "python_AST_checked_count": len(python_files),
                         "previous_audit_source_count": len(baseline["sources"]),
                         "protected_union_count": len(protected), "all_inputs_unchanged": True,
                         "explicit_source_fingerprints": current},
        "limits": [
            "Raw signed depth and its origin/orientation decomposition are not force shares. Contact clips negative depth; a large late-stance raw difference can have negligible positive-contact effect.",
            "Phase normalization is descriptive only; original impulses use each subject's original times and measured nominal windows.",
            "Tiny interpolation/TO-knot differences do not replace the official paired values: AB06-13.802088%, AB07+39.988463% left vertical impulse versus own measured reference.",
            "The original AB06 paired receipt retains its terminal adapter-grid failure; physical primary cycles are fully observed. This audit does not revise the receipt.",
            "No physically independently qualified hardware force law or static hardware-force recording was available; matching healthy GRF alone would not establish hardware realism.",
        ],
        "practical_qualification_it": "Prima di attribuire il divario alla rete o all'anatomia, qualificare insieme registrazione e contatto con un vincolo indipendente di geometria/carico e rendere visibile il supporto esterno delle reserve. Non correggere AB07 adattando soltanto il guadagno alle sue GRF sane.",
        "helper": {"path": Path(__file__).resolve().relative_to(ROOT).as_posix(), "sha256": digest(Path(__file__))},
    }
    target = BASE / "causal_findings.json"
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "protected_union_count": len(protected),
                      "all_inputs_unchanged": True, "output": target.relative_to(ROOT).as_posix()}, indent=2))


if __name__ == "__main__":
    main()
