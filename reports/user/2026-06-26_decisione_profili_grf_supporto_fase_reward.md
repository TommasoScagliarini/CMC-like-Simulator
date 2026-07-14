# Decisione profili GRF: supporto dinamico e fase reward

## Problema

Dopo la validazione degli eventi GRF online e il confronto A/B tra profili, andava chiarita la decisione architetturale per la reward task-based ex-novo:

- quale profilo usare per sostenere fisicamente la simulazione;
- quale segnale usare per rilevare heel strike, toe off e fase protesica;
- come evitare che la reward dipenda da una misura GRF completa non disponibile nel deployment reale.

Il punto centrale e separare il ruolo dinamico della GRF dal ruolo sensoriale/event-based della fase.

## Decisione

La scelta operativa e:

1. usare `grf_correct_magnitude` come profilo GRF online applicato alla dinamica della simulazione;
2. usare `grf_detector_HS-TO`, o il detector event-tuned equivalente, come sorgente di HS/TO e fase protesica per reward e osservazioni actor.

Quindi:

- `grf_correct_magnitude` sostiene la protesi nella simulazione CMC-like ibrida;
- `grf_detector_HS-TO` non viene trattato come forza fisica da ottimizzare in magnitudo;
- il segnale event-tuned viene usato come proxy di sensore di contatto/fase.

## Motivazione

Il confronto A/B su finestra `11.99-13.99 s` ha mostrato che `grf_detector_HS-TO` e migliore per il timing:

- TO error: `2 ms` contro `53 ms` di `grf_correct_magnitude`;
- contact IoU: `0.980` contro `0.925`.

Pero `grf_correct_magnitude` resta migliore come supporto dinamico:

- impulso verticale sinistro piu alto: `659.5 Ns` contro `488.4 Ns`;
- rapporto impulso/prescribed piu alto: `0.869` contro `0.643`;
- reserve piu basse: tau reserve norm p95 `325.6` contro `433.3`;
- penetration massima piu bassa: `0.0175 m` contro `0.0394 m`.

La conclusione e che i due profili rispondono a due esigenze diverse:

- supporto fisico della simulazione;
- rilevamento robusto degli eventi di gait.

## Implicazione per la reward

La reward task-based deve usare la fase protesica come gate, non inseguire una wrench GRF completa.

Segnali ammessi/consigliati per actor e reward:

- `left_in_contact`;
- pulse/eventi `HS` e `TO`;
- `phi_pros` stimata online;
- maschera stance/swing;
- eventualmente normal force o segnale di contatto binarizzato, con cautela.

Segnali da non usare come target forte:

- magnitudo assoluta della GRF event-tuned;
- COP/momenti come obiettivi prescritti;
- reserve actuator come termine realistico di reward;
- pelvis height o altre grandezze non misurabili sul paziente come reward primaria.

I termini reward 1-2 della proposta originale vanno quindi reinterpretati cosi:

- contact/load reward: premiare il fatto che in stance la protesi sia caricata in modo non nullo e coerente, usando il gate di fase;
- swing unloading: penalizzare contatto/carico protesico durante swing, sempre usando fase/eventi e non una GRF prescribed completa.

## Strategia implementativa proposta

La pipeline dovrebbe esporre due livelli:

1. livello dinamico simulativo:
   - `online_grf_profile_file = grf_correct_magnitude`;
   - lato sinistro online applicato;
   - lato destro prescribed o altra modalita coerente con il setup sperimentale.

2. livello sensoriale per fase:
   - calcolo HS/TO da `grf_detector_HS-TO` o detector equivalente;
   - smoothing causale `0.10 s`;
   - low threshold `15 N`;
   - HS confirmation threshold `120 N`;
   - min contact duration `0.03 s`;
   - min cycle duration `0.30 s`.

La policy non dovrebbe ricevere la full wrench GRF, ma solo variabili compatibili con deployment:

- contatto;
- eventi;
- fase protesica;
- eventualmente tempo dall'ultimo HS/TO.

## File modificati

Creato questo report:

- `reports/user/2026-06-26_decisione_profili_grf_supporto_fase_reward.md`

Nessun file di codice, reward o training e stato modificato da questa decisione.

## Test e verifiche collegate

La decisione si basa sui seguenti artifact gia generati:

- `results/online_grf_event_validation_preliminary_fine/report.md`;
- `results/online_grf_event_validation_forward_states_recommended/`;
- `results/hybrid_profile_ab_comparison/report.md`;
- `results/hybrid_profile_ab_comparison/summary.json`;
- `reports/user/2026-06-26_validazione_eventi_grf_fase_protesica_reward.md`;
- `reports/user/2026-06-26_confronto_ibrido_profili_grf_online.md`.

Verifiche principali:

- validazione eventi HS/TO su IK replay e forward states;
- confronto A/B CMC-like ibrido tra `grf_correct_magnitude` e `grf_detector_HS-TO`;
- controllo di reserve, impulse, penetration, moment flip e timing eventi.

## Ex-Novo TODO

- Implementare in modo esplicito la separazione tra profilo dinamico e detector di fase nella pipeline RL.
- Esporre all'actor solo feature realistiche di contatto/fase.
- Aggiornare la reward task-based affinche i termini di load/unloading siano phase-gated e non basati su target GRF prescribed.
- Mantenere per il primo pass `T_nom` fisso, model-based e specifico per AB06 treadmill, stimato offline dalla reference/condizione sperimentale e salvato in config come parametro dichiarato.
- Preparare, per un secondo step, una stima dinamica di `T_nom` basata sulla velocita di cammino stimata da sensori disponibili sulla protesi. La stima dovra essere lenta, filtrata e clampata in un range clinico/biomeccanico, e non dovra dipendere direttamente dal periodo HS->HS prodotto dalla policy nel ciclo corrente.
- Aggiungere in un secondo momento il punto 7 della reward: `prosthetic kinematic morphology`. Questo termine dovra incentivare una forma biomeccanicamente plausibile della traiettoria senza introdurre imitazione prescribed diretta, per esempio knee flexion minima in swing, knee extension prima di HS, ankle stance non collassata, toe clearance, smoothness e assenza di oscillazioni ad alta frequenza.
