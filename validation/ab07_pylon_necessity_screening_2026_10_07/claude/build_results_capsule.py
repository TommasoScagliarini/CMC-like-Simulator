"""Export only aggregate observations to the independently blinded reviewer."""
from datetime import datetime, timezone
import csv
import hashlib
import io
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]


def read(p):
    return json.loads(Path(p).read_text())


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rounded(v):
    return format(v, ".8g") if isinstance(v, (float, int)) else v


def main():
    aggregate_path = AUDIT / "dynamics_analysis/aggregate_metrics.json"
    aggregate = read(aggregate_path)
    protocol = read(AUDIT / "dynamic_protocol.json")
    if len(aggregate["cases"]) != 18 or set(aggregate["cases"]) != set(protocol["cases"]):
        raise ValueError("All eighteen finalized cases required")
    rows = []
    for case_id in protocol["execution_order"]:
        item = aggregate["cases"][case_id]
        metrics_path = ROOT / item["metrics"]["path"]
        if sha(metrics_path) != item["metrics"]["sha256"]:
            raise ValueError("Aggregated source changed")
        m = read(metrics_path)
        p = m["windows"]["primary_all"]
        runtime = m["runtime"]
        duration = runtime["final_clocks_s"].get("environment", 0) - runtime["actual_episode_range_s"][0]
        complete_cycles = sum(m["windows"][f"cycle_{i}"]["coverage"] == "complete" for i in (1, 2, 3))
        recorded_pen = m["contact_all_recorded"]["per_side"]["left"]["penetration_m"]["max"] * 1000
        row = [case_id, item["case"]["length_mm"], item["case"]["static_ankle_offset_deg"],
               runtime["status"], runtime["end_reason"], duration, complete_cycles, p["coverage"], recorded_pen]
        if p["coverage"] == "complete":
            r = p["reserves"]["per_coordinate"]["pelvis_ty"]
            balance = p["vertical_balance"]
            tracking = p["tracking"]["coordinates"]
            swing = sum(m["windows"][f"cycle_{i}_swing"]["forces"]["left_online"]["impulse_Ns"] for i in (1, 2, 3))
            jleft = p["forces"]["left_online"]["impulse_Ns"]
            row += [p["forces"]["left_online"]["mean_N"], balance["implied_left_required_minus_reserve_Ns"] / p["requested_duration_s"],
                    r["force_or_torque"]["mean"], r["force_or_torque"]["rms"], r["impulse"]["positive"], r["impulse"]["negative"],
                    r["impulse"]["absolute"], r["work_J"]["absolute"],
                    tracking["pros_knee_angle"]["actual_minus_served"]["position"]["rms"],
                    tracking["pros_ankle_angle"]["actual_minus_served"]["position"]["rms"],
                    tracking["pelvis_ty"]["actual_minus_served"]["position"]["rms"] * 1000,
                    jleft, p["forces"]["left_measured"]["impulse_Ns"], swing / jleft if jleft else None,
                    balance["closure_residual_Ns"]]
        else:
            row += ["NA"] * 15
        rows.append([rounded(v) for v in row])
    stream = io.StringIO()
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(["case", "mount_mm", "static_reference_offset_deg", "status", "end_reason", "observed_episode_s", "fully_observed_primary_cycles", "three_cycle_coverage", "max_recorded_penetration_mm", "left_mean_N", "left_required_by_momentum_N", "reserve_y_mean_N", "reserve_y_RMS_N", "reserve_y_positive_Ns", "reserve_y_negative_Ns", "reserve_y_absolute_Ns", "reserve_y_absolute_work_J", "knee_tracking_RMS_deg", "ankle_tracking_RMS_deg", "pelvis_y_tracking_RMS_mm", "left_impulse_Ns", "healthy_measured_left_impulse_Ns", "swing_impulse_fraction", "momentum_closure_Ns"])
    writer.writerows(rows)
    text = """Osservazioni aggregate finali, senza conclusioni del team locale.
Continua il tuo audit indipendente iniziato in questa sessione. Il team non ha ancora letto alcun tuo testo visibile. Non ci sono sorgenti, dati per campione, credenziali o accesso ai file in questa capsule.

Ambito: verificare la necessita' dell'allungamento del montaggio distale AB07, distinguendo requisito anatomico, requisito condizionato dalla preparazione, comportamento del banco e identificazione del montaggio fisico reale.
Proporzione dai modelli: AB06 shank sano465.480598772mm, mount430mm; AB07 shank sano417.741015827mm. L0=385.89929908923726mm; L1 attuale403.4784289483052mm; delta17.57912985906795mm.
I17 modelli AB07 differiscono solo nelle12posizioni dei marker calibrati e nel soloY del frame parent della caviglia protesica. Tutti3049altri campi XML sono ESATTAMENTE identici: masse, localCOM, inerzie, jointframes/axes, defaults/ranges, ForceSet/SEA, mesh/rendering. Varia la distanza di montaggio; non si ricostruisce massa/inerzia di hardware diverso. WorldCOM puo' cambiare con mount/IK.
Ogni cella usa nuova calibrazione dei12marker e propria fullIK28612righe10.36-153.415s, stesso solver. Quattro endpoint riprendono byte-esattamente le preparazioni precedenti. Le tre pose aggiuntive -2/-5/-10deg aL0 sono congelate prima dei loro output, motivate dalla verifica geometrica. Nessun adattamento dei profili o dei guadagni ai risultati.
Offsetstatico significa convenzione di calibrazione dei marker del piede e proprio riferimento IK. Non e' modifica di jointzero/default/range. Non identifica di per se' un montaggio fisico osservato: i dati sono del soggetto sano, non marker hardware indipendenti su una protesi realmente montata.

Geometria isolata: su436pose statiche, L0 con ginocchio raw invariato raggiunge la frontiera di contatto delle sfere primarie in436/436, con variazione caviglia mediana-1.655589deg (range-1.841047..-1.578260deg), entro limiti originali del modello. Clearance raw sfere primarie L0+1.622242mm, L1-15.861616mm. L1 rimane gia' intersecante questa frontiera per tutto il range caviglia con ginocchio raw. Toccare la frontiera non significa sostenere il peso.
Distinguere: geometriaSTL, sfereforza virtuali, punti detector. Piano primaryY=-9.503954760mm; piano binaryY=+14.8208231mm; COPdataset codificatoY=0. RawCOPY e' gia'0, conversione preservaXYZ/scaling. Non e' stata identificata nei file analizzati una calibrazione indipendente piano/piattaforma->frame dei marker; questo non dimostra che Y0 sia errato. Non trattare la mesh ossea destra come suola/marker hardware omologo.
Registrazione attuale copia due target del donorAB06: pitchstatico7.050422788deg e clearanceSTL-2.136415973mm rispetto allo stesso piano benchmark. AB07pre pitch2.319774287deg e clearance23.418364525mm. Perturbando solo il targetdonor clearance di+/-5mm, l'estensione calcolata diventa circa12.551819/22.606581mm; pitchdonor+/-2deg produce circa14.215478/20.994493mm. Traslare lo STESSO piano per entrambi cancella, non e' prova di confusione fra piani.
Anklecenter meanstance: sanoAB06 51.957585699mm, sanoAB07 61.230211526mm; prostheticAB06 79.863156401mm. AB07proporzionaleoffset0 89.138814963mm, attuale71.813162275mm. Il contrasto sano include anatomia e marcia/posa, non una misura isolata dell'anatomia.

Contratto dinamico uguale per tutti: forward integration rk4_bypass1ms, azione originale prescribed_action derivata dalla propriaIK filtrata produzione6Hz/order4, muscoli/reserve e cascade esistenti, SEA plugin NONimpedance con molle reali, nessuna rete caricata. LeftGRF virtuale calcolata dallo stato reale; rightGRF misurata prescritta. Kknee321/Kankle500Nm/rad, Fopt100/250Nm, stessi guadagni e clamp motore500Nm. Non sono posizioni imposte tramite vincoli.
Finestre: AB07 reset12.591740519774012s, trecicli14.087704688172044->18.69078019047619s, durata4.603075502304145s. AB06 reset15.615960838108348s, primary17.163967985977223->21.822001050884474s. AB07 episode6.11s (emendamento prima di ogni output:6.10nonbrackettava l'ultimoHS nel recorderprestep); AB06episode6.21s. Setuporiginaleorizzonte9s preservato, nessuna finestra sostituita daultimooutput.
Le medie e gli impulsi confrontabili qui sono sui trecicli congelati, non sull'interoepisode. ReserveY positiva spinge versoalto, negativa tiraversobasso. Absoluteimpulse evita cancellazioni; generalizedwork=integrale reserve_y*qdot_pelvis_y. Il carico sinistro richiesto e' mgT+mdeltaVcom-Iright, non la GRF sperimentale sinistra come groundtruth di una protesi reale. Bilancio: Ileft+Iright+Ireserve_y-mgT=mdeltaVcom. Measuredleft e' del sano ed e' solo diagnostica.
NA indica nessun aggregato dei trecicli, anche se esiste osservazione parziale; le interruzioni sono risultati, senza retune/rerun. La soglia grf_penetration e' quella gia' catturata28mm, non un nuovo criterio clinico. max_recorded_penetration e' un massimo di campioni esportati, non necessariamente lo snapshot terminale.

""" + stream.getvalue()
    text += "\nRiferimento cinematico puroFK (forza sulla IK, distinto dai rollout effettivi sopra):\n"
    fk = read(AUDIT / "preparation/contact_replay_screening.json")
    for case_id, values in fk["scalar_comparison"].items():
        text += f"{case_id}: FKmeanN={values['contact_mean_force_N']:.8g}; left12markerRMSmm={1000*values['left12_marker_mean_stance_RMS_m']:.8g}; anklecenterYmm={1000*values['ankle_center_Y_mean_stance_m']:.8g}\n"
    text += "\nControlli SEA (solo3cicli completi, frazioni percampione):\n"
    for case_id, item in aggregate["cases"].items():
        m = read(ROOT / item["metrics"]["path"])
        if m["windows"]["primary_all"]["coverage"] != "complete":
            continue
        parts = []
        for name, values in m["SEA_primary_native_samples"].items():
            parts.append(f"{name}:identityPASS={values['all_identity_checks_pass']},u_saturation={values['normalized_control_saturation']['fraction']:.6g},motor500clamp={values['motor_torque_clamp']['fraction']:.6g}")
        text += case_id + ": " + "; ".join(parts) + "\n"
    phase_path = AUDIT / "phase_timing/aggregate_phase_metrics_v2.json"
    phase = read(phase_path)
    if set(phase["cases"]) != set(aggregate["cases"]):
        raise ValueError("All eighteen finalized phase cases required")
    text += "\nFase: originale matchingdiagnostico(HSonset120ms/TOonset180ms), quattroHS/treTO nominalifissi. UltimoHS rightcensored anchenei completi (salvati solo11/3ms dopochiusura). Missing countsphysiological nonpromossi se manca supporto; matchedclosing ancora osservazione ma guardincompleto. BinarygeometryFSM eforza20N sono canali distinti. Segue conteggi aggregati e bias del subset matched originale, inclosingseosservato; biasNAquandozero matches. Missingguardfull e censored sono separati.\n"
    text += "case,channel,event,status,predicted,matched,extra,unmatched_fully_observed,censored_unmatched,bias_ms\n"
    for case_id, item in phase["cases"].items():
        for channel in ("binary_geometry_FSM", "primary_force_20N_crossings"):
            ch = item["primary_channels"][channel]
            if not isinstance(ch.get("comparison"), dict):
                text += f"{case_id},{channel},all,{ch['status']},NA,NA,NA,NA,NA,NA\n"
                continue
            for event in ("heel_strike", "toe_off"):
                onset = ch["comparison"][event]["channels"]["onset"]
                match = onset["matching"]
                bias = match["timing_bias_s"]
                values = [case_id, channel, event, ch["status"], match["predicted_count"], match["matched_count"],
                          match["false_positives"], len(onset["unmatched_reference_fully_observed_matching_guard_s"]),
                          len(onset["unmatched_reference_censored_or_unknown_s"]), 1000*bias if bias is not None else "NA"]
                text += ",".join(str(rounded(v)) for v in values) + "\n"
    text += "\nP0_M10: tre onsetforza20N, uno durante ciascuno dei primi trecicli, sono tardivi rispetto al rispettivoHSsano (media149.925ms, range138.681-166.069ms); originale matcher120ms li lascia unmatched. Sono eventi osservati tardivi, non assenza di contatto. Tutti3controlliL0negativi completano la fisica ma nessun ciclo actorFSM valido: M2/M5 emettono solo1HS/1TO, M10zeroHS/TO. Questo e' un tradeoff di timing/percezione, non un nuovo gate o nuova dichiarazionePASS.\n"
    text += """
Validazione: nessuna modifica ai683input originali/protocollati. P4_A1 tutti17STO decodificati esattamenteuguali al prefisso del vecchio paired. AB06 stesso prefisso comune6200righe, scarti ultimi10campioni di controlli/muscoli e9di stati per endpoint-0.700ps che causa ceil dei substeps governor11->10. Il test di esattaidentita' globale restaFALSE; primarydeltaIleft=0.0Ns, deltaIreserve_y=-1.42379099e-6Ns, deltaWsigned=+1.48110054e-7J. Nessuna modifica runtime per renderloPASS.
Tutte le identita' SEA/finitezza vengono verificate sui campioni registrati. Le simulazioni completate possono comunque essere sostenute dalle reserve; questo non valida il montaggio reale.

Fornisci ora, indipendentemente dal team locale, il tuo verdetto in italiano (massimo1100parole): cosa e' dimostrato circa l'esatto+17.579mm, cosa circa qualsiasi allungamento aposa attuale fissata, cosa circa alternative aL0, e cosa non e' identificabile sul montaggio fisico/anatomia. Usa numeri e limiti dei test, conserva tutte le interruzioni. Non scegliere una lunghezza clinicamente corretta o normale senza misure indipendenti. Indica separatamente eventuali cause ancora non identificate e la correzione del metodo di preparazione giustificata da questi dati.
"""
    destination = HERE / "independent_results_capsule.txt"
    with destination.open("x", encoding="utf-8") as output:
        output.write(text)
    receipt = {"created_utc": datetime.now(timezone.utc).isoformat(), "capsule_bytes": destination.stat().st_size,
               "capsule_sha256": sha(destination), "aggregate_sha256": sha(aggregate_path), "phase_aggregate_sha256": sha(phase_path),
               "builder_sha256": sha(Path(__file__)), "only_aggregate_observations": True,
               "local_conclusions_sent": False, "source_code_sent": False, "raw_timeseries_sent": False}
    with (HERE / "results_capsule_receipt.json").open("x") as output:
        json.dump(receipt, output, indent=2)
    print(json.dumps(receipt))


if __name__ == "__main__":
    main()
