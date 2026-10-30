# Training con oscuramento da CP90 e rollout di CP112 e CP184 — 2026-10-01

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


## Risultato

Completato il pilot di 100 aggiornamenti aggiuntivi da CP90, fino all’iterazione globale 190. Il best per ritorno medio di training nell’intervallo 91–190 è **CP112**; il best delle ultime 25 iterazioni è **CP184**. Eseguiti due rollout deterministici con oscuramento completo e prodotti **sette plot canonici per checkpoint**, tutti sotto `plot/` del repository.

Entrambi completano i 5 secondi previsti. Nel confronto a ingressi oscurati CP184 ottiene ritorno 210,4209, contro 138,1906 di CP112. È un risultato sul singolo start e seed valutato, non una dimostrazione di robustezza. **Best90 fresh resta la baseline corrente**, come deciso dall’utente.

## Problema e strategia

Dopo aver ottenuto una configurazione imitativa accettabile con best90, occorreva misurare la capacità della policy di operare senza i canali prescribed, in preparazione al successivo training ex-novo. L’utente ha scelto un primo esperimento semplice: continuazione dal checkpoint completo CP90, start singolo, quattro blocchi fissi di 25 aggiornamenti. Non sono stati introdotti i tre start differenziati.

La reward resta imitativa durante tutto il pilot. L’oscuramento riguarda gli ingressi della policy; il target resta disponibile per calcolare reward e diagnostiche. Il successivo training ex-novo non è stato avviato.

| Iterazioni globali | Aggiornamenti | Probabilità di oscuramento episodico | Disponibilità target |
|---|---:|---:|---:|
| 91–115 | 25 | 25% | 0,75 |
| 116–140 | 25 | 50% | 0,50 |
| 141–165 | 25 | 75% | 0,25 |
| 166–190 | 25 | 100% | 0 |

Le percentuali indicano la probabilità di mascherare un episodio, non un’attenuazione dell’ampiezza dei segnali. Ogni blocco viene eseguito in un nuovo processo dal checkpoint completo finale del precedente, mantenendo lo stato di apprendimento e iniziando nuovi episodi fisici. Questo evita che episodi visibili attraversino il confine verso il blocco finale.

La preparazione ha introdotto un fork esplicito del curriculum, con offset persistente 90 per far decorrere correttamente la nuova progressione. Restore di actor, critic, Adam e KL; nessun critic rewarm o cambio di reward. Avvio autorizzato dall’utente il 30 settembre alle 11:48:42 CEST.

## Esito del training e selezione

Il [registro di avanzamento — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-c855f3acc71e824f>) attesta quattro blocchi completati, ciascuno con 25 aggiornamenti. Le summary dei blocchi riportano completamento regolare, nessuna iterazione saltata e nessun crash restart.

| Blocco | Best globale del blocco | Ritorno medio di training |
|---|---:|---:|
| Oscuramento 25% | 112 | 249,3247 |
| Oscuramento 50% | 123 | 205,5802 |
| Oscuramento 75% | 165 | 161,7012 |
| Oscuramento 100% | 184 | 158,7107 |

La selezione usa `episode_return_mean` delle nuove iterazioni 91–190; il checkpoint di partenza CP90 non partecipa a questa graduatoria. Poiché CP112 non appartiene alle ultime 25 iterazioni (166–190), è stato valutato anche CP184, come richiesto.

I ritorni di training provengono da regimi di visibilità differenti: la graduatoria complessiva non identifica automaticamente la migliore policy in oscuramento completo. Per questo entrambi i rollout sono stati eseguiti nelle stesse condizioni completamente oscurate.

## Protocollo e risultati dei rollout

Azioni deterministiche, seed 123, start offset 1,0 s, durata 5 s, massimo 500 step, disponibilità target 0. Registrati output fisici e tracce della policy. Usati rispettivamente `stage01_mask25/rl_module_best` e `stage04_mask100/rl_module_best` del run `MLP39_LEFT_best90_masking_100iter_2026-09-30`.

| Metrica | CP112 — best complessivo | CP184 — best ultime 25 |
|---|---:|---:|
| Step | 500 | 500 |
| Durata | 5 s | 5 s |
| Ritorno rollout | 138,1906 | 210,4209 |
| Cicli FSM validi | 2 | 2 |
| Eventi invalidi | 0 | 0 |
| Penetrazione massima | 17,632 mm | 19,181 mm |
| Motivo fine | `episode_time_limit` | `episode_time_limit` |

Entrambi riportano `terminated=false` e `truncated=true`: fine della finestra temporale, senza arresto anticipato. CP184 migliora il ritorno in questa valutazione, ma registra una penetrazione massima maggiore. L’ispezione della figura 7 mostra ancora scostamenti dal target, soprattutto nei picchi del ginocchio. Non è stato eseguito in questa attività un confronto CP90 completamente oscurato, né una valutazione multi-start o multi-seed.

## Plot e dati

- [Set completo di 7 plot CP112](../../plot/10_01_2026_unified39_LEFT_masking_best112_overall_hidden/).
- [Set completo di 7 plot CP184](../../plot/10_01_2026_unified39_LEFT_masking_best184_last25_hidden/).
- [Rollout CP112: summary, traccia e output fisici — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-5ca9412348973bdd>).
- [Rollout CP184: summary, traccia e output fisici — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-f4c878ae727c60e3>).
- [Selezione e hash dei checkpoint — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-76f04ca86d294ee1>).
- [Risultati e verifiche aggregate — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-ed02cbcefbb9a2a1>).
- [Piano eseguibile, configurazioni e log del training — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-79357e1c36826f1c>).

Ogni set contiene le figure canoniche 01–07: controllo SEA/reserve; stati articolari e motore; coppia/angolo/potenza sul ciclo; velocità/potenza sul ciclo; coppia motore ed errore; riferimento articolare ed errore SEA; policy rispetto al target imitativo. Nella figura 7 il target è il template LEFT registrato (`imitation_target_q`); la gamba sana grezza nella riga centrale è solo contesto.

## File modificati o creati

Durante la preparazione del pilot, sotto `Trajectory Generator/unified_MLP_39/`:

- `target_availability.py`: fork esplicito del curriculum e offset persistente.
- `training_config.py`, `train_ppo_mlp.py`, `training_runtime.py`: configurazione, vincoli del resume e registrazione del fork in un output separato.
- `test_curriculum_fork.py`, `test_masking_pilot_plan.py`: verifiche del fork e del piano.
- `runs/plans/left_best90_masking_100iter_2026-09-30/`: launcher, quattro configurazioni, manifest, preflight e ricevute di esecuzione.

Durante la valutazione del 1° ottobre:

- `validation/masking_results_2026-10-01/render_and_verify.py`: rendering dei due set, verifica oscuramento, PNG e integrità dei checkpoint.
- Nella stessa directory: `selection.json`, `results.json`, `README.md`, ricevute di lancio/esecuzione e log dei rollout/render.
- Le due directory di rollout e le due directory di plot collegate sopra; ricevute `plot_receipt.json` e `verification.json` per la tracciabilità.
- Questo report e l’aggiornamento parziale del daily del 1° ottobre per propagare i TODO.

Riutilizzato il renderer canonico esistente `validation/render_canonical_rollout.py`. Nessuna modifica al simulatore, al controller o al plugin SEA in questa valutazione.

## Test e verifiche eseguite

Preparazione del pilot, già documentata nel daily del 30 settembre: 228 test di suite e 7 del launcher superati; preflight nativo macOS con restore esatto CP90, 12 stati Adam, KL 0,45000001788139343 e quattro passi OpenSim. Il preflight non conteneva aggiornamenti PPO e non era un test completo a 13 worker; l’avvio reale ha poi verificato il restore e l’ingresso nell’iterazione 91.

Valutazione conclusa il 1° ottobre:

- Completamento dei quattro blocchi e selezione dei due checkpoint verificati dai metadati e dai risultati di training.
- Entrambi i processi di rollout terminati con exit code 0; 500 righe di traccia per ciascuna summary.
- Sei ingressi prescribed, indici 0, 1, 35, 36, 37, 38, nulli in tutti gli step delle due tracce.
- Checkpoint corrispondenti alla selezione; hash sorgente invariati dopo la valutazione.
- Esattamente 7 PNG leggibili per checkpoint, hash registrati, nessun canale mancante.
- Figura 7 di entrambi i checkpoint ispezionata visivamente.

Le esecuzioni native attestano macOS arm64; la verifica live Windows resta aperta. I test della preparazione sono riportati come evidenza pregressa e non sono stati rieseguiti per la sola redazione del report.

## TODO e decisioni aperte

- [ ] Correggere in un’attività separata il bug bootstrap PPO di Ray/RLlib 2.55.1: bootstrap eliminato ai confini artificiali dei frammenti e propagazione errata prima di una vera terminazione. La correzione è stata esplicitamente rinviata dall’utente; non è applicata a questo pilot. Dettagli nell’[audit del 29 settembre](2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).
- [ ] Analizzare quantitativamente l’effetto dell’oscuramento prima di adattare la progressione o scegliere il checkpoint per il successivo training ex-novo. Le due valutazioni richieste sono complete; un confronto comune con CP90 e con i checkpoint di fine blocco resta da svolgere se si prosegue l’analisi.
- [ ] Verificare robustezza su condizioni e seed ulteriori; i tre start restano esclusi dal pilot corrente per scelta dell’utente.
- [ ] Eseguire la verifica live Windows della pipeline scelta.

Il backlog storico resta nel daily; queste voci non avviano nuovi esperimenti né modificano automaticamente la baseline.
