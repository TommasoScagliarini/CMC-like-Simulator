# Ricomparsa della coppia positiva SEA ankle — 22 settembre 2026

## Risultato

Il grande burst positivo nel ciclo finale del rollout `last40` è reale e riproduce il meccanismo diagnosticato a giugno: la contact patch online produce un momento negativo rispetto all'asse della caviglia, e il SEA applica una coppia positiva quasi uguale per seguire la reference. Non emerge un'inversione di segno del controller o del grafico. La correzione storica nella reward è già attiva; era un incentivo appreso, non una modifica del controller che vietasse per sempre le coppie positive.

## Evidenza numerica attuale

File letti: `rollout_episode_sea_torques.sto`, `rollout_episode_sea_diagnostics.sto`, `rollout_episode_online_grf.sto`, `rollout_policy_trace.json`, `rollout_summary.json` del last40 e dei confronti.

- Picco positivo dopo il transiente: **+37,222 Nm a 17,532 s**. Nello stesso campione `tau_ref=+37,210 Nm`: il SEA esegue il comando, con errore −0,012 Nm.
- Il controller cascade produce **P=+1,407 Nm**, **I=+35,802 Nm**. L'integrale compensa il carico persistente; nel burst finale non risultano saturazione del comando motore, clamp integrale o anti-windup attivo. `tau_ref=P+I` verificato entro 1e−8 Nm.
- A 17,530 s: `tau_spring=+37,131 Nm`, `tau_ref=+37,188 Nm`, momento GRF nel trace **−36,359 Nm**, COPz−anklez **+65,29 mm**. Il ricalcolo simultaneo della probe statica precedente dava **−36,750 Nm**.
- Il carico normale simultaneo a 17,530 s è **539,19 N** e la penetrazione **17,37 mm**, sotto la soglia di penalizzazione per penetrazione di 20 mm. Un contatto può dunque avere una leva sfavorevole anche senza superare quella soglia; il guard sul momento lo rileva separatamente.
- Su tutti i campioni caricati ≥50 N dopo i primi 0,2 s: correlazione fra `tau_spring` e **meno** il momento GRF **0,999765**; RMS della somma **0,788 Nm**. È evidenza molto forte della compensazione meccanica, senza pretendere di avere ripetuto l'inverse dynamics completa fatta a giugno.
- Burst oltre +8 Nm: **15,630–16,274 s** e **17,183–17,808 s**. Nell'ultima finestra d'appoggio 17,2–17,85 s la coppia è positiva in tutti i campioni e vale mediamente +19,35 Nm. Nell'ultimissimo campione, invece, scende già a +0,691 Nm: il problema è il burst dell'ultimo ciclo, non una divergenza al termine del file.

L'errore RMS `tau_ref−tau_spring` dell'intero tratto dopo 0,2 s è **0,428 Nm**. Il problema è quindi il contatto indotto dalla traiettoria generata, non un SEA incapace di seguire il comando.

Nota di allineamento: il trace ogni 10 ms usa l'ultimo wrench GRF, campionato 1 ms prima del timestamp di stato. Le correlazioni usano questo trace già disponibile; i valori di torque/carico ad alta frequenza provengono dagli STO a 1 kHz. Il picco di 17,532 s usa il trace più vicino, 17,530 s. Tale discretizzazione non cambia la conclusione sul segno/meccanismo.

## Evoluzione nei checkpoint

Metriche calcolate dopo i primi 0,2 s; aree in Nm·s.

| Checkpoint | Picco positivo | Area positiva | Area negativa | Costo totale anti-flip |
|---|---:|---:|---:|---:|
| 39D iter25 | +9,15 Nm | +3,45 | −24,26 | 0,000022 |
| 39D best29 | +37,32 Nm | +41,18 | −7,39 | 69,85 |
| 39D last40 | +37,22 Nm | +27,52 | −6,70 | 40,43 |
| V26 best87 | +10,19 Nm | +5,29 | −6,00 | 0,01045 |

La 40 ha ridotto il costo e l'area positiva rispetto alla 29, ma non ha eliminato il picco né recuperato la robustezza della 25. Il fenomeno non migliora monotonicamente con le iterazioni. Piccole coppie positive non sono tutte proibite: anche il V26 presenta transitori positivi. Il guard opera sul momento di contatto inferiore a −8 Nm in presenza di carico, non su ogni campione SEA positivo.

## Cosa era stato corretto a giugno

1. **19 giugno** — `reports/user/2026-06-19_confronto_sym60_asym100_tau_spring_ankle.md`: posizione simile non implica coppia simile; velocità/fase e integrale cascade possono produrre comandi di segno diverso. Diagnosi del percorso q/reference → cascade → SEA, non correzione del segno nel plugin.
2. **22 giugno, diagnosi fisica** — `reports/user/2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md`: inverse dynamics e geometria identificarono la causa diretta nel momento della GRF/COP. Picco `asym100`: `tau_SEA=+21,83 Nm`, `tau_GRF=−21,31 Nm`, carico559 N, penetrazione13,73 mm. La nuova ricomparsa ha lo stesso bilancio di segni, ma ampiezza maggiore.
3. **22 giugno, implementazione** — `reports/user/2026-06-22_reward_guard_momento_grf_caviglia.md`: aggiunta loss normalizzata `max(0,−tau_GRF−8)^2/8^2` attiva con carico≥50 N; penalità sottratta dopo il clip. L'implementazione attuale limita la loss a25, limite non raggiunto nel picco analizzato. Peso iniziale0,25. Il pacchetto completo includeva penalità di penetrazione5,0. Non fu imposto un target di coppia e non furono invertiti segni o cambiati i SEA.
4. **23 giugno** — il guard forte migliorò la caviglia, ma ridusse il carico sinistro e peggiorò la cinematica del ginocchio; vedi `2026-06-23_rollout_contact_validity_knee_chattering.md` e `2026-06-23_diagnosi_grf_penalty_tracking_knee_stance.md`.
5. **24 giugno** — `2026-06-24_analisi_rollout_grf_soft.md`: il training da zero con pesi più morbidi **penetrazione1,0 / anti-flip0,10** mantenne il beneficio sulla coppia con un compromesso migliore. Dati ricalcolati: giugno17 senza guard area positiva+9,06/negativa−6,01; giugno22 hard+2,36/−5,95; giugno23 soft+1,71/−11,30. Il successivo `2026-06-24_config_grf_soft_knee_jerk2000.md` portò il config al soft e predispose un'ablation del jerk **solo ginocchio**: non era un fix specifico del segno ankle.

**Stato attuale verificato:** last40 e V26 hanno entrambi anti-flip **0,10**, tolleranza **8 Nm**, carico minimo **50 N**, penetrazione **1,0**. La penalità è calcolata e sottrae **40,427 punti** al rollout40. Non è stata persa o disattivata la correzione storica; la nuova policy non ha ancora imparato a evitarne l'attivazione.

## Collegamento con la probe causale del 21 settembre

`validation/causal_difference_2026-09-21/target_audit/static_contact_probe.json` ha verificato nella stessa posa17,53 s che correggere **solo la caviglia da +14,62° al target +9,33°**, mantenendo tutto il resto fisso, cambia il momento da −36,75 a −4,18 Nm e azzera il guard. La penetrazione scende da17,37 a13,54 mm. Quindi al picco identificato non serve cambiare il segno SEA: serve correggere la traiettoria/postura che induce il carico sfavorevole. La prova è statica e locale, non una prova di eseguibilità di tutto il ciclo.

## Implicazione operativa

Non è motivato correggere controller/plugin o forzare torque negativo. L'incentivo corretto esiste già. Proseguire l'apprendimento può migliorare la traiettoria, ma **non è dimostrato che basti attendere**: fra25,29,40 la metrica è non monotona e l'errore ankle sotto carico resta evidente. Un eventuale proseguimento va giudicato su errore ankle durante stance, durata/completamento, costo anti-flip e area positiva, non solo sul ritorno complessivo. Non aumentare automaticamente il guard a0,25: lo storico ha già mostrato la possibilità di sacrificare ginocchio e carico per evitare la penalità.

## Artefatti e verifiche

- `analyze.py`, `metrics.json`: ricalcolo offline su sette rollout.
- `ankle_torque_vs_grf.png`: confronto 25/29/40/V26 e corrispondenza di segno.
- `last_stance_cascade.png`: dettaglio torque/reference/momento, P/I e penetrazione nell'ultimo appoggio.
- Nessun training, simulazione o modifica runtime eseguiti.
