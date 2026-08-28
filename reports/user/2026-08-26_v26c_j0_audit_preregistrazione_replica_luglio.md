# V26C J0 — audit e preflight della replica July-faithful su parent V26 agosto e runtime v3

**Data**: 2026-08-26
**Fase**: audit + preregistrazione + preflight. **Nessun fit, nessun rollout, nessuna collection, nessun training.**
**Verdetto**: **GO-FOR-ARCHITECT-REVIEW** — mi fermo al preflight e attendo il tuo GO.

Autorità: **l'utente** decide in via esclusiva ogni scelta su dataset, split, gate, architettura, iperparametri e ordine. I valori recuperati verbatim dagli artefatti di luglio non sono deviazioni. Nessun default è stato inventato.

---

## 1. Due stadi di luglio, non uno

Ricostruiti da **codice e artefatti**, non dai report.

### Stadio BASE — `target_domain_imitation_no_controller_memory_2026-07-13`

| voce | valore |
|---|---|
| parent | `validation/controller_memory_ablation/2026-07-13_zero_iter_port/rl_module_initial_warm_start` |
| forma parent | **W1 (256, 25)** — rete realmente 25-wide, colonne clock 0,1 a zero |
| dataset | **500 coppie** `observation → PRESCRIBED TEACHER`, da un rollout del teacher prescritto di 500 step |
| artefatti | `teacher_dataset.npz`, `teacher_trace.json` |
| gate del rollout teacher | 500 step, `end_reason = episode_time_limit`, penetrazione **0,022944 < 0,025**, 2 cicli validi, `gate_pass = true` |
| scope del fit | **l'intera mean actor** |
| split | **casuale 80/20** → 400 train / 100 val |
| iperparametri | seed 123, batch **64**, lr **3e-4**, patience 60, clip 1.0, logstd_weight **0.1**, anchor **1e-5**, `freeze_logstd = False`, val 0.2 |
| epoche | max **400**, run 243, **best 183** |
| esito | RMSE iniziale 0,167339 → **adattata 0,011782** |

**Nota importante**: questi iperparametri sono **esattamente** quelli che il ramo V26B aveva pinnato come «protocollo luglio». Quel pin era dunque **fedele allo stadio base**. Ritiro la mia affermazione contraria nell'analisi in 800 parole del 25/08: era basata sullo stadio Markov, che usa parametri diversi.

### Stadio MARKOV — `..._phase_aligned_scaled_full_r32_alt8_2026-07-13` (selezionato)

| voce | valore |
|---|---|
| parent | `2026-07-13_markov35_zero_iter_port/rl_module_initial_warm_start`, W1 (256, 35), colonne zero **[0,1,25..34]** |
| dataset | **16000 self-anchor** + **712 recovery phase-aligned** + **8000 teacher multistart** = **24712** |
| label nominali | il `policy_action_mean` **dell'attore stesso** (`_trace_arrays`, `target_domain_markov_adaptation.py`) — **non** un teacher |
| label recovery | teacher prescritto time-aligned, troncato al primo discrete mismatch |
| scope del fit | **`train_full_actor = true`** — l'intera mean-network, **non** le sole 10 colonne |
| iperparametri | batch 128, lr 5e-5, anchor 0.01, logstd_weight 0.0, `freeze_logstd = True`, epoche 400 (best 392) |
| digest | `a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21` |

## 2. Riconciliazione dei due livelli di gate

Avevo commesso l'errore che hai segnalato: classificare luglio sul solo `run_summary.gate_pass`. Correzione recepita e ora **codificata nel tool**, che avverte esplicitamente contro quell'uso.

| livello | criterio | `full_r32_alt8` | `robust_conservative_r96` |
|---|---|---|---|
| **1 — gate offline di adattamento** | shift nominale entro `max_nominal_mean_shift` | **FAIL** solo sul criterio single-sample stretto: max_abs 0,031594 > 0,005, con rms 0,004175 e p95 0,007956 | **PASS** (max_abs 0,004909) |
| **2 — selection gate (closed-loop)** | robustezza start ed esplorazione | **PASS** → **SELEZIONATO** | rifiutato dinamicamente |

**Decisione registrata**: `SELECT_MARKOV35_FOR_H0_H1_WITH_WARM_CRITIC`, `training_ready = true`.

**Evidenza closed-loop del selezionato**: **3/3 start deterministici** (nominal, −0.20, +0.20) e **3/3 seed stocastici σ 0.005**, tutti 500 step, penetrazione massima 0,0233–0,0246 **sotto la guardia 0,025**, **zero clipping**.

**Rationale verbatim di luglio**:

> «Closed-loop start and exploration robustness are the primary warm-start gate. A stricter nominal refinement reduced offline shift but failed dynamically, so it was rejected instead of weakening the safety threshold.»

Due criteri interni al selection gate sono `false` senza bloccare: `strict_offline_single_sample_shift_gate` e `nominal_return_not_regressed_vs_25_feature_baseline`.

**Root cause registrata da luglio**: `architecture_rejected = false`, `adaptation_protocol_failure = **true**`.

**Conseguenza per V26B**: il ramo B è stato bloccato **interamente su gate offline di ricostruzione** e non ha **mai** raggiunto una misura closed-loop. Sotto il precedente di luglio quell'ordine è invertito. `r96` è un candidato intermedio offline, non il risultato finale.

## 3. Trasferibilità al parent V26 agosto

| attore | W1 | colonne a zero | SHA-256 |
|---|---|---|---|
| V26 agosto 39D (parent esclusivo) | (256, 39) | — | `0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd` |
| V1 35D transplant | (256, 35) | 0, 1 | `16c2d1ae9fb4e77fffa092d74d37e78f54ba24d990774e91bf1d412c551bb031` |
| **B0 35D masked** | (256, 35) | **0,1,25..34** | `aa7ea0fa1bbef8bb6ef2a33ee8ebe5defeeb4959148a589b81ff994cf291171f` |
| parent markov di luglio | (256, 35) | **0,1,25..34** | — |

**Verificato**: B0 è esattamente V1 con le colonne 25..34 azzerate, tutti gli altri tensori identici; e il suo insieme di colonne a zero **coincide** con quello del parent markov di luglio.

**Deviazione dichiarata**: lo stadio base di luglio usava una rete **realmente 25-wide**; il replay usa **un solo attore 35D** con 25..34 hard-zero durante lo stadio base e riattivate **nello stesso attore** dopo. Una colonna a zero non contribuisce al forward, quindi lo stadio base è funzionalmente equivalente, ed è ciò che rende possibile la riattivazione senza una seconda rete. Come da tua indicazione: nessuna rete 25D separata, nessun widening.

## 4. Vincolo tecnico sul collector — controllo negativo

Verificato nel codice, come da tua segnalazione. `target_domain_imitation.build_target_env_config` **non inoltra nessuna** delle sette chiavi v3:

`binary_phase_detector_profile_file` · `binary_phase_invalid_event_policy` · **`event_contract_id`** · **`binary_phase_event_contract_id`** · `binary_phase_fsm_mode` · `binary_phase_actor_fsm_version` · `binary_phase_debounce_s` · `phase_fsm_input_mode` · `phase_sensor_on_threshold_n` · `phase_sensor_off_threshold_n` · `phase_sensor_dwell_s` · `detector_sample_dt_s`

e non inoltra **né morphology né corridor**. Userebbe un teacher **non-v3** pur leggendo il config pinnato `a870cc38…`. I builder di riferimento (`train_ppo_mlp.py`, `rollout_eval.py`) le portano tutte e dodici: il buco è specifico del builder storico.

**Correzione mia sui nomi — seconda passata.** Due delle tre voci che avevo messo in tabella di rename erano **sbagliate**. Verificato in produzione: `rollout_eval.py` righe 523 e 530 e `train_ppo_mlp.py` righe 1301 e 1308 contengono **entrambe** le chiavi contract, distinte e affiancate.

**I due contract ID sono chiavi distinte con valori distinti**, entrambe richieste:

| chiave | valore nel config pinnato |
|---|---|
| `event_contract_id` | **`legacy_events_v1`** |
| `binary_phase_event_contract_id` | **`binary_point_v25+heel_qualified_fsm_v2`** |

Non vanno mai fuse, scambiate o trattate come rename l'una dell'altra.

**L'unico rename config→env reale** è il detector profile:

| config | env |
|---|---|
| `binary_phase_detector_profile` | **`binary_phase_detector_profile_file`** |

`binary_phase_invalid_event_policy` **non è un rename**: il config la usa già con quel nome (valore `reject_continue`), e nessuna chiave `invalid_event_policy` nuda esiste nel config.

**Test negativi fail-closed indipendenti**: (a) omissione di ciascuno dei due contract ID è identificabile esattamente; (b) lo scambio dei due valori produce una configurazione diversa da quella pinnata, verificato leggendo il config con un parser **ancorato a inizio riga** — un confronto per sottostringa sarebbe stato ingannato da `morphology_causal_event_contract_id`, che condivide il suffisso con `event_contract_id`. Il test verifica anche quel caso specifico.

Il collector additivo dovrà quindi costruire il **FULL env_config** semanticamente identico a `rollout_eval.run` / `train_ppo_mlp.build_config`, partendo dal config pinnato, includendo reward e morphology, e **assertare nel proprio receipt**: detector profile, `binary_phase_actor_fsm_version == v3`, `phase_fsm_input_mode`, `binary_phase_fsm_mode`, `invalid_event_policy`, `event_contract_id`, debounce e dwell, corridor profile con alpha e weight, start, soglie correnti **0,020 m e 0,028 m** (entrambe), i criteri del gate J1/J3 pertinente, la semantica di collection pinnata (§4-ter), timeout, reward, morphology. **Fail-closed**: un campo mancante o difforme aborta prima di qualunque step d'ambiente. È vietato chiamare `build_target_env_config` o `target_domain_imitation.main`.

Il test include il controllo negativo: misura che le sette chiavi mancano dal builder storico, che i builder di riferimento le hanno tutte, e che l'audit stesso non importa né chiama il builder.

## 4-ter. Semantica di collection di luglio, pinnata

Recuperata **verbatim** dagli artefatti dello stadio base, non assunta come default:

| parametro | valore | fonte |
|---|---|---|
| `seed` | **123** | `adaptation_report.hyperparameters.seed` |
| `teacher_lookahead_s` | **0.0** | `run_summary.teacher`, `teacher_summary.json` |
| `action_noise_sigma` | **[0.0, 0.0]** | `teacher_summary.json` |
| `action_noise_hold_steps` | **1** | `teacher_summary.json` |
| `action_noise_hold_duration_s` | 0.01 | `teacher_summary.json` |

`action_noise_realized_rms = [0.0, 0.0]`: il rollout teacher di luglio era **strettamente senza rumore**. Il test confronta i valori pinnati campo per campo con `teacher_summary.json`, non con una costante riscritta.

## 4-bis. Gate vincolante: quello V26B corrente, non quello di luglio

Correzione recepita. **0,025 m non è la guardia operativa.** Il runtime pinnato `a870cc38…` ha soglia soft GRF **0,020 m** e terminazione hard **0,028 m**; il collector deve registrare **entrambi**.

**Due gate distinti**, non uno solo. Correzione recepita:

- **J1 — teacher collection**: gate **comune** di integrità, runtime, sicurezza e contratto. È ciò che serve per **accettare il dataset**. La qualità cinematica **non** si applica: qualifica il passo che un *attore* produce, non l'integrità di una raccolta.
- **J3 — actor closed-loop**: gate comune **+ qualità cinematica completa**.

Gate **comune** (J1 e J3), da `v26b_b1_base_fit.declared_closed_loop_gates`:

| criterio | soglia |
|---|---|
| penetrazione massima | **≤ 0,020 m** |
| step | **500/500**, `end_reason = episode_time_limit` |
| cicli validi | **≥ 2** |
| `phase_timeout_stance` | **0** |
| `phase_timeout_swing` | **0** |
| `morphology_causal_contract_failure` | **0** |
| `hs_cancelled_count` | **0** |
| `resync_count` | **≤ 1** |

Qualità **cinematica**, vincolante **solo per J3**:

| criterio | soglia |
|---|---|
| ankle `q_min` | **≤ −0,03 rad** |
| ankle ROM | **≥ 0,30 rad** |
| knee ROM | **≥ 0,60 rad** |
| knee strictly flexed | **sì** |
| knee e ankle within bounds | **sì** |

Esempio verificato nel test: un minimo di caviglia **−0,0099 rad FALLISCE**, perché −0,0099 > −0,03.

`action_clipped_steps` è registrato come **diagnostica**, non come gate binding: promuoverlo richiederebbe una fonte già approvata, che non esiste. Il test verifica che non compaia in nessun gate vincolante.

**Luglio resta solo evidenza storica, esplicitamente non isometrica**: guardia 0,025 m, penetrazione teacher 22,94 mm, e i tre rollout deterministici del selezionato fra **0,0233 e 0,0246 m**. Quei valori stanno **sopra** la soglia soft corrente di 0,020 m: il successo closed-loop di luglio **non si trasferisce numericamente** al gate attuale, e nessun numero di luglio può essere riusato come criterio di pass. Il test verifica che nessuna soglia di luglio filtri in un criterio vincolante.

## 5. Piano di replay (specificato, non eseguito)

- **Attore**: uno solo, 35 → 256 → 256 → 4, colonne 0,1 e 25..34 hard-zero nello stadio base.
- **Parent**: esclusivamente `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best`. Nessun checkpoint o dataset di luglio come parent operativo.
- **J1 — collection**: nuovo rollout del teacher prescritto, 500 step, runtime v3 e corridoio correnti, vincolato dal **gate V26B corrente** (§4-bis). **Vietato sostituire** con le 1500 righe a 3 anchor, con LOTO/LOCO/11-fold o con le label u_IK.
- **J2 — fit base**: intera mean actor, colonne mascherate a zero esatto, split casuale 80/20, iperparametri recuperati verbatim, max 400 epoche.
- **J3 — closed-loop**: gate **primario**, per il precedente di luglio, con i criteri V26B correnti del §4-bis. Uno scarto solo offline non è di per sé uno stop.
- **J4 — Markov**: riattivazione di 25..34 nello stesso attore. **Fuori scope ora.** Sigma resta differita per governance e non è assunta.

## 6. Punti aperti — nessuno blocca il preflight

| id | tema | blocca ora |
|---|---|---|
| O1 | equivalenza semantica del teacher prescritto sotto v3 | no — verificabile alla collection |
| O2 | esito del gate del rollout teacher sotto v3 e corridoio corrente | no — noto alla collection |
| O3 | sigma di esplorazione (solo stadio Markov) | no — fuori scope, non assunta |

## 7. TODO futuri, non operativi

Conservati, **non cancellati**, nessun artefatto rimosso: LOTO 3-fold (B1), LOCO 6-fold (B1R1), 11-fold 8 cicli + 3 code (B1R2), braccio budget B1R2-A, braccio learning-rate B1R2-B. Tutti i receipt e i module state sotto `diagnostics/` e `candidates/` restano byte-identici.

## 8. File, test, digest

| file | SHA-256 |
|---|---|
| `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j0_audit.py` | 75c2a26df2b04f9877eb492f958679212fda45dffc03a923998f4b1f611ae694 |
| `.../test_v26c_j0_audit.py` | ae0d70f42381a4494abd3ef574e991096d75ec2da61813fef1f725e19154556a |
| `.../v26c_j0_audit_receipt.json` | 3512b19d6abcc570a37baa225efd2afacfa8f7f1c7990698d8040336fbecb4e5 |

**Test**: `python test_v26c_j0_audit.py` → **PASS, 156 check**.
**Eseguiti**: `python v26c_j0_audit.py` (dry) e `--run` (audit read-only, receipt no-clobber).

Produzione, FSM, GRF, morphology, reward, SEA e C++ **invariati**: `git status` mostra soltanto i tre file già modificati a inizio sessione.

## 9. Comando futuro esatto del primo stadio

Non esiste ancora: il collector J1 va scritto dopo il tuo GO, perché non può riusare il builder storico (§4). Il primo comando sarà nella forma

```
python v26c_j1_collect.py --preflight
python v26c_j1_collect.py --authorized-stage V26C-J1-TEACHER-COLLECT
```

con il preflight che verifica config pinnato, detector, FSM v3, corridoio, start, soglie **0,020 / 0,028 m**, gate V26B e timeout **prima** di qualunque step d'ambiente.

## 10. Verdetto

**GO-FOR-ARCHITECT-REVIEW.** Entrambi gli stadi di luglio sono ricostruiti da codice e artefatti, i due livelli di gate sono riconciliati con `full_r32_alt8` come precedente metodologico, la catena parent è verificata, il vincolo sul collector è codificato con controllo negativo, il gate vincolante è quello V26B corrente, e il piano non contiene default inventati. Nessun fit, rollout o training avviato. Attendo il tuo GO.
