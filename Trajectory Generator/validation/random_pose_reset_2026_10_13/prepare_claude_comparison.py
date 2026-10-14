"""Build a compact, reviewable comparison capsule from completed saved evidence."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent


def read(name):
    return json.loads((HERE / name).read_text())


def short_case(r):
    post = r.get("tracking_rmse_deg", {})
    cycles = r.get("complete_valid_cycles", [])
    pen = r.get("left_primary_peak_penetration_m")
    return {"id": r["id"], "initial_pose_rad": r["case"]["pose"],
            "classification": r["classification"], "failed_gates": r["failed_gates"],
            "elapsed_logical_s": r.get("duration_s"), "returned_steps": r.get("returned_policy_steps"),
            "cycles": r.get("valid_cycles"), "qualifying_cycles": r.get("qualifying_cycle_count"),
            "first_hs_confirmed_elapsed_s": r.get("first_hs_confirmed_elapsed_s"),
            "peak_pen_mm": None if pen is None else pen*1000,
            "post050_rmse_deg": {k: v.get("post050") for k, v in post.items()},
            "cycle_excursion_rad": [c["excursion_rad"] for c in cycles],
            "returned_reward_sum": r.get("returned_reward_sum"),
            "measurement_error": r.get("measurement_error"),
            "terminal_witness": r.get("returned_journal_evidence", {}).get("measured_terminal_contacts", []),
            "positive_duration_segments": r.get("positive_duration_returned_segments"),
            "zero_duration_terminal_returns": r.get("witnessed_zero_duration_terminal_returns")}


def main():
    aggregate = read("aggregate_results.json")
    verification = read("independent_saved_output_verification.json")
    diagnostics = read("compact_diagnostics.json")
    capsule = {"scope": aggregate["scope"], "geometry_pool": aggregate["geometry_pool"],
               "N": aggregate["selected_random_n"], "successes": aggregate["successes"],
               "unresolved": aggregate["unresolved_n"], "classes": aggregate["classification_counts"],
               "cp95": aggregate["success_probability_cp95"],
               "completion_vs_primary": aggregate["completion_vs_strict_frontier"],
               "control": [short_case(r) for r in aggregate["controls"]],
               "random": [short_case(r) for r in aggregate["random_cases"]]}
    capsule["descriptive_diagnostics"] = []
    for r in diagnostics["cases"]:
        late = (r.get("same_clock_actual_vs_coherent_nominal") or {}).get("late_400_500", {})
        reserve = (r.get("biological_reserves") or {}).get("startup_000_050", {})
        capsule["descriptive_diagnostics"].append({
            "id": r["id"], "clipped_returned_steps": r.get("raw_action_clipping", {}).get("any_joint_raw_clipped_steps"),
            "late_actual_vs_nominal_rmse_deg": {j: v["rmse"] for j, v in late.get("prosthetic_q_rmse", {}).items()},
            "startup_biological_rotational_reserve_norm_rms_Nm": reserve.get("biological_rotational_reserve_norm_Nm", {}).get("rms"),
            "startup_pelvis_translation_residual_norm_rms_N": reserve.get("pelvis_translation_residual_force_norm_N", {}).get("rms"),
            "initial_primary_pen_mm": (r.get("initial", {}).get("primary_contact", {}).get("max_penetration_m") or 0)*1000,
            "initial_detector_word": r.get("initial", {}).get("detector", {}).get("word")})
    # Detailed diagnostics and raw STO remain local; this capsule carries the
    # scientific outcomes and their hashes rather than thousands of samples.
    evidence_hashes = {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                       for name in ["aggregate_results.json", "compact_diagnostics.json",
                                    "independent_saved_output_verification.json",
                                    "source_protocol_audit.md", "classifier_seal_v2.json"]}
    prompt = """Confronto finale richiesto dall'utente: sintetizza e critica i due audit
e i test di reset da pose casuali PROTESICHE, in italiano, massimo1600 parole.
Modello richiesto Opus5.5 effortmax. Nessun tool o nuovo test: ragiona SOLO su
questa capsula e dichiara i limiti. Non proporre come eseguita nessuna prova
aggiuntiva, training, Windows o replay open-loop.

Codex ha sigillato le conclusioni indipendenti PRIMA di leggere il tuo audit
source. Campagna: best47 AB06 actor35/critic84 frozen, t0=13.946870983805102,
5s,500step,dtinteg1ms,seed123 deterministic,zeroPPOupdate. Knee[-1.20,-.03],
ankle[-.55,+.55]rad,random.Random(20261013),pool400uniformIID,first20ammessi
in ordine; nessun failure sostituito. Biologia19DOF q/qdot IK al reset poi
integrazione libera muscle/reserve-driven su stesse referenceIK/GRFdestra;
non e standing o biologia prescribed. 76 file congelati,73precedenti invariati.

Bootstrap: actual/reference/SEA/governor/endpoint stessa qrandom,v=a0,PI0;
FSMprime dopo reset,0HS/TO/cycles,loadobs0 originale fino primasample. Wrapper
governor originale persistent,mai IKfallbackprotesico. Controlcoherentnominal
stessi v=a0, sola qnominale diversa; legacy12oct riusato separatamente. 401
staticresets senza policy/integration e4nativefirstsegmentprobes verificati;
11bootstraptest,20readerV2test,5diagnosticstest offlinePASS. Setteplot percaso
sotto rootplot,17STO percaso verificati numericamente e con hash indipendenti.

Confronto source Codex: bootstrapcoerente necessario; cambiare soloq fisiche
nonbasta. Physicalfailures nelN20, tecnici distinti; nonattribuirecapacitanei
singoli componenti o al feedback senza replay. Concordiamo suprimaryvsdetector
plane eN20conditionalCIlimit. Sourcefinalcode corregge rischi del tuo audit
intermedio: probeFlatAction viaoriginalwrapper(1,2)clip; count20,pool400gate27,
selectionhash401receipts,incrementaljournals,censoredreturns,batchcontinues,
postprocessingseparato,walltimeouttecnico. Nativeprobes confermano ilfixprobe.

Correzione readerV2, criteri NONmutati: un guard hard30mm puo essere mascherato
da end_reason morphology_causal_contract_failure nel wrapperrewardV26. Il
journalreturnedfailure+terminal_contact testimonia la causa e un samplefisico
successivo all'ultimoboundarylogico; ultima risposta puo restituirereward senza
avanzare tempo. Tenere rewardreale, nonduplicareeventiFSM, separaterecordSTO
parziali; physicalfailure rimanefailure. V1code/seal/test immutabili conservati.
Nessunrerun, sogliarelax o rescue; questoe correttalettura, nonnuovocriterio.

Criteri frozen:500step/5s,time_limit,>=2actorcycles,firstacceptedHSconfirmed
<=3s,0invalid/rejected/degraded/timeout/resync/cancelled,pen1ms<28mm,
actual-servedRMSEpost.5<=2deg/joint,>=2completecyclesEACH kneeexc>=.60rad
eankleexc>=.30rad. Return/clipping/reserves sono diagnostiche separatamente.
Ilcontrolmisurato verifica se2cicli in5s siano fattibili: il periodo GRF1.669s
non impone lo stesso periodo agli eventi actor. Non cambiamo orizzonte dopoesiti.

Dettaglioimportante random025: initialknee−.04547945rad,terminalactualknee
+.0004368696902rad (=+.02503deg),servedknee−.003312332496rad,trackingerror
circa.2148deg. Termina a.14s oltre operatingboundknee0rad, dentro ROMmodello
+/-pi/2.13su14 rawkneeactionsoltre+1,clippati. È un failure della guardia
operativa: NONun crash o una dimostrazione di instabilità catastrofica/fall.
random006:initialpen23.556mm, terminalguard30.129935mm asamplefisico.319s;
lastlogicalboundary.310s,32returnincl1zerotimeterminal,31positivewindows.

Valuta: quali conclusioni precedenti confermate/corrette? Quanto e dimostrata
gestione di posecasuali e quanto restalimitato? Analizza failurecauses ecriteri
completionvsrecovery; noninferirecausalita sullearningPPO. Distingui gaitrecuperato
nel plant da merito esclusivo rete, standing, interobox o affidabilita95%.
Verifica CP e denominatori. Fornisci una sintesi comune chiara e futureprove
necessarie, senza trasformarle in prerequisiti retroattivi o lavoro giaeseguito.

IL TUO AUDIT SOURCE ORIGINALE (interimcode):
""" + (HERE / "claude/source_response.md").read_text() + "\nRISULTATI NUMERICI COMPLETI COMPATTI:\n" + json.dumps(capsule, ensure_ascii=False, separators=(",", ":")) + "\nHASH EVIDENZE:\n" + json.dumps(evidence_hashes)
    assert len(prompt.encode()) < 40000, "Comparison capsule became too large"
    (HERE / "claude/comparison_prompt.txt").write_text(prompt, encoding="utf-8")
    print(json.dumps({"prompt_bytes": len(prompt.encode()), "case_count": len(capsule["random"]),
                      "verification_loaded": bool(verification), "diagnostics_loaded": bool(diagnostics)}))


if __name__ == "__main__":
    main()
