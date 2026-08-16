# Preflight full-environment H0/V25 A-B-C — `ERROR_H0_REFERENCE`

Data: 2026-08-05

## Esito

Il preflight H0/V25 A-B-C zero-update è stato autorizzato, congelato ed
eseguito. Il protocollo si è arrestato correttamente al primo rollout, caso A
`det_minus020`, con stato terminale:

```text
ERROR_H0_REFERENCE
```

La baseline H0 ha superato il limite di penetrazione della GRF primaria al
passo 53:

- limite preregistrato: `grf_penetration_max_m < 0.025 m`;
- passo 52: `0.02409746674941412 m`;
- passo 53: `0.026628085231843266 m`;
- superamento del limite: circa `1.628 mm`;
- `end_reason=grf_penetration`;
- `safety_stop_count=1`.

Il rollout ha quindi completato 53/500 step, non ha raggiunto due cicli e non
ha potuto costituire una baseline A valida per il confronto successivo.

Questo non è un fallimento del detector V25 o della FSM V20. Nel caso A:

- `binary_phase_fsm_mode=disabled`;
- V20 non è stata eseguita;
- gli eventi legacy erano ancora autorevoli;
- V25 è stato soltanto campionato per preparare il futuro confronto bit-exact
  con B.

Di conseguenza non sono stati eseguiti B `binary_shadow` o C
`binary_active`, e il preflight non produce ancora una conclusione sulla
compatibilità H0-V25.

## Problema verificato

Il protocollo richiedeva che ogni riferimento A completasse 500 step, finisse
per time limit, contenesse almeno due cicli completi e restasse sotto 25 mm di
penetrazione. La prima condizione era la partenza deterministica anticipata di
0,20 s:

```text
condition_id = det_minus020
episode_start_offset_s = 1.756870983805102
seed = 123
```

La penetrazione è passata da 24,097 mm al passo 52 a 26,628 mm al passo 53.
Questo ha causato direttamente la terminazione di sicurezza. Gli altri check
falliti nel receipt sono conseguenze dello stop precoce:

- 53 anziché 500 step;
- terminazione anziché truncation al time limit;
- zero cicli completi;
- 53 anziché 500 campioni episode-level per reserve e residual.

I controlli indipendenti dal completamento dell'episodio sono invece risultati
corretti:

- layout actor/full `35/84`;
- osservazione `float32`;
- zero action clipping;
- zero timeout;
- zero fallback;
- zero hard-invalid e non-finiti;
- zero saturazioni SEA;
- `morphology_weight=0`.

## Confronto con il riferimento storico

Il riferimento H0 storico del 2026-07-14, con lo stesso checkpoint, offset e
seed, aveva completato 500 step e due cicli con penetrazione massima
`0.02460202939096984 m`.

Il confronto read-only dei trace ha isolato la causa della divergenza:

- al reset, tutte le 35 feature actor e la prima azione H0 sono bit-identiche;
- dopo il primo step, 33/35 feature actor restano bit-identiche;
- divergono soltanto `online_left_normal_grf_bw` e
  `online_left_in_contact`;
- nel trace storico valgono rispettivamente `0.013752441853284836` e `1.0`;
- nel nuovo caso A valgono `1.2345267919045e-7` e `0.0`.

Il vecchio valore di carico coincide, entro circa `4.1e-10 BW`, con i
`10.532831120172203 N` del detector analogico. Il nuovo valore proviene invece
dai `9.455093274727161e-05 N` della GRF primaria. Di conseguenza già la seconda
azione diverge:

- storico: `[0.2726323902606964, 0.004133209586143494]`;
- nuovo A: `[0.2739918828010559, 0.023434802889823914]`.

Il commit `4be9f2afcf2b4034ce81eccd3b6972d2ec5e6947` del 2026-08-04 ha corretto
questa semantica: prima `_phase_grf_sides()` preferiva il detector analogico;
ora `_physical_online_grf_sides()` usa esclusivamente la GRF primaria per
carico continuo e contatto. Questa è la separazione `primary_grf_split_v1`
richiesta dal piano e non deve essere annullata.

L'actor H0 è invece rimasto bit-identico al modello pre-split: il warm-up del
critic aveva `freeze_actor=true`, digest actor invariato e differenza massima
zero. La causa a monte del fallimento è quindi l'incompatibilità del vecchio
actor H0 con la nuova semantica delle due feature continue
`primary_grf_split_v1`. La divergenza delle azioni porta poi alla divergenza
della dinamica e alla terminazione per penetrazione.

Questo non è attribuibile a V25 o V20: V20 era disabilitata e V25 non entra
nell'osservazione actor. Il caso A ha svolto esattamente il suo compito,
scoprendo che il riferimento H0 corrente non è valido prima di aprire B e C.

Riferimento storico:

`Trajectory Generator/runs/rollout/validation/robust_reference_runs/2026-07-14_h0_schema2/deterministic_minus020/rollout_summary.json`

Trace storico usato per il confronto delle prime transizioni:

`validation/controller_memory_ablation/2026-07-13_markov35_corrected_full_short_minus020/rollout_policy_trace.json`

## Strategia fail-closed applicata

Il supervisor ha applicato la sequenza preregistrata:

1. esecuzione di A `det_minus020`;
2. valutazione del gate comune;
3. classificazione `ERROR_H0_REFERENCE`;
4. arresto immediato dell'intera matrice.

Non sono stati avviati retry, rescue, retuning, modifiche della soglia o
sostituzioni di seed. Le altre 17 destinazioni A/B/C e la directory dei gate
sono rimaste vuote.

Il ledger terminale registra:

- `rollout_count_completed=1`;
- `protocol_unit_count_completed=0`;
- `pair_results={}`;
- `c_results={}`;
- `next_stage=STOP_WITHOUT_RETRY_RETUNING_OR_FALLBACK`;
- `ppo_updates=0`;
- `training_performed=false`;
- `protected_trials_opened=[]`;
- `runtime_promoted=false`.

H0_sep non è autorizzato: il piano lo rende valutabile soltanto dopo un PASS
completo A/B seguito da un fallimento scientifico di C. Qui è fallito A.

## File implementati per il preflight

Il percorso preparatorio, congelato prima dell'esecuzione, comprende:

- `validation/run_h0_v25_abc_preflight.py`, supervisor e worker A/B/C;
- `validation/compare_h0_v25_abc.py`, gate comuni, parità A/B e
  non-regressione C/A;
- `validation/freeze_h0_v25_abc_execution.py`, freeze dell'execution-unlock;
- `validation/test_h0_v25_abc_preflight.py`, test del protocollo;
- `validation/test_binary_phase_fsm_env_v20.py`, test del sampled-disabled A;
- `Trajectory Generator/osim_trj_cmc_like.py`, esposizione del journal V25 in
  A senza eseguire V20;
- `simulation_runner.py`, statistiche SEA e conteggi fallback richiesti;
- `static_optimization.py`, diagnostica dei fallback del solver;
- `Trajectory Generator/baseline_MLP/rollout_eval.py`, compatibilità CLI con
  `binary_active`.

Nessuno di questi file è stato modificato dopo il freeze di esecuzione.

## Artefatti prodotti

Freeze e receipt:

- `validation/h0_v25_abc_protocol_corrected_lock.json`;
- `validation/h0_v25_abc_layout_preflight_receipt_v3.json`;
- `validation/h0_v25_abc_preflight_test_receipt_v3.json`;
- `validation/h0_v25_abc_execution_unlock.json`.

Esecuzione terminale:

- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/execution_ledger.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/run_start.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/action_tape.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/v25_raw_journal.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/trace.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/summary.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/manifest.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/receipt.json`;
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/A_det_minus020/failure.json`.

## Test e verifiche

- Layout preflight A/B/C: PASS per `84 float32`, proiezione actor 35 e azione
  `(2,) float32`; nessun campo binario nell'osservazione.
- Suite mirata del protocollo: 69 test PASS prima del freeze.
- Rollout reale A `det_minus020`: FAIL al gate comune per penetrazione GRF.
- Audit indipendente post-mortem: classificazione
  `ERROR_H0_REFERENCE` corretta.
- Audit di integrità: tutti i JSON strict, finiti e senza chiavi duplicate;
  tutti i path/hash/size congelati corrispondono.
- Cardinalità degli artefatti coerente con lo stop: 53 righe di trace e action
  tape, 530 campioni V25 a 1 ms più baseline t0.
- Le altre 17 directory e `gates/` sono vuote.
- Zero PPO, training, trial protetti, promozione runtime o modifica H0.

Limite documentale noto: `run_start.json`, `failure.json` e il ledger sono
no-clobber ma non possiedono una receipt crittografica esterna completa. Gli
artefatti scientifici principali sono invece legati dal manifest e dal
receipt.

## Invarianti preservate

- GRF primaria e `online_grf.py` non modificati;
- profilo/materiale/geometria e legge di contatto primaria non modificati;
- plugin C++ e semantica SEA non modificati;
- checkpoint H0 invariato, SHA `44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b`;
- morphology reward a peso zero;
- trial protetti 05/06 e reserve 03/07 mai aperti;
- nessun PPO o H0_sep;
- nessuna promozione di V25/V20.

## TODO

- Non rilanciare o modificare la matrice A/B/C appena consumata.
- Non ripristinare il detector analogico come sorgente delle feature continue:
  violerebbe `primary_grf_split_v1` e la separazione primaria/detector.
- Revisionare e congelare con nuova autorità il percorso H0: la necessità ora
  evidenziata è adattare l'actor pre-split alla semantica della GRF primaria,
  prima di poter valutare separatamente l'attivazione V25/V20.
- Definire un nuovo gate baseline A per il candidato adattato e un nuovo
  percorso A/B/C soltanto dopo avere ottenuto una baseline valida.
- Mantenere B, C, H0_sep, trial 05/06, corridor positivo e PPO chiusi fino ai
  rispettivi gate.
