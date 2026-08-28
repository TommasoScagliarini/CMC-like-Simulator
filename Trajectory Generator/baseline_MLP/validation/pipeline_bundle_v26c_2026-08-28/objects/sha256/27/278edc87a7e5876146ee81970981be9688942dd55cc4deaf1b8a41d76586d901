# V26C — J20: esecuzione singola del warm-up critic-only

**Data:** 2026-08-27
**Stadio:** `V26C_J20_CRITIC_WARMUP`
**Foglia:** `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_critic_warmup_v26c_2026-08-27_r1/`
**Verdetto emesso dal runner:** `AWAITING_RESTORE_AUDIT` — **12/12 gate**, nessun gate fallito
**Marker finale:** `TECHNICAL_INVALID` rimosso, `RESTORE_AUDIT_PENDING` presente
**Promozione:** `NONE` — **niente è training-ready, niente è promosso**

---

## 1. Problema

J19A ha prodotto un attore 35D valido ma esportato in forma `inference_only`: dieci
chiavi di solo attore, **senza critic**. Caricare quell'export in un modulo
addestrabile lascia un critic silenziosamente fresco — nessuna eccezione, nessun
warning. Prima di qualunque PPO occorreva quindi una fase che addestrasse **solo il
critic**, lasciando l'attore bit-per-bit intatto, e che lo dimostrasse invece di
asserirlo.

Due ostacoli erano stati individuati in preparazione:

1. **Il blocco della KL-guard.** `_enforce_kl_update_guard`
   (`train_ppo_mlp.py:716-773`) gira a ogni iterazione e richiede quattro metriche
   `kl_update/*` che esistono solo quando è installato il learner custom, il quale
   viene installato solo sotto `exact_start_sampling` (`:1380-1394`). Con guard a
   0.01 ed `exact_start_sampling` false, 7 controlli su 7 fallivano e il processo
   sarebbe abortito all'iterazione 1.
2. **Il manifest J19A** doveva restare byte-identico, quindi la correzione delle
   feature non poteva essere scritta dentro di esso.

## 2. Soluzione

- **Config additivo immutabile** `v26c_j20_warmup_critic_only_cfg.yaml`
  (`e98bde4e…`, 276 righe). Una sola differenza semantica rispetto al config J20:
  `supervision.max_minibatch_mean_kl_loss: 0.01 → null`. Le sezioni
  `model / ppo / parallelism / simulation / grf / logging / reward` sono identiche.
  Né `v26c_j20_warmup_cfg.yaml` né `train_ppo_mlp.py` sono stati toccati per questo.
  Il summary conferma l'effetto voluto: `kl_update_guard.enabled = false`.
- **Overlay content-addressed** `v26c_j20_actor_feature_manifest_overlay.json`
  (`b0f90354…`) per le feature, consumato via `explicit_manifest`. Il manifest J19A
  resta byte-identico.
- **Runner fail-closed** `v26c_j20_critic_warmup_execution.py` (`ce70797a…`,
  1333 righe), test ermetici 208/208, prereg sigillata (`3ecd96a1…`), GO operativo
  `v26c_j20_critic_warmup_architect_go.json` (`651927d8…`) con 41 pin chiusi.

## 3. Strategia di esecuzione

Esecuzione **unica**, senza retry di alcun tipo, dal percorso worker
(`--execute --go-file …`). Marker `TECHNICAL_INVALID` scritto **per primo** e rimosso
**per ultimo**, solo dopo che tutti i gate immediati sono passati; sostituito da
`RESTORE_AUDIT_PENDING`, non da un `PASS`.

- avvio child `2026-08-27T18:39:23Z`, fine `2026-08-27T18:47:26Z`, PID 43411
- `returncode = 0`, wall time 480,4 s, un solo processo figlio lanciato
- nessun `supervisor_state.json` → prova che nessun supervisore è intervenuto
- `stop_reason = completed`, `interrupted = false`, `timed_out = false`

## 4. Esito: i dodici gate

| Gate | Contenuto | Esito |
|---|---|---|
| **G1** | esattamente una iterazione logica | **PASS** |
| **G2** | lifetime **e** delta per-iterazione entrambi esattamente 4096 | **PASS** |
| **G3** | metriche del critic finite | **PASS** |
| **G4** | KL media esattamente zero | **PASS** |
| **G5** | attore bit-esatto a **ogni** audit | **PASS** |
| **G6** | log-std bit-esatto e σ invariata | **PASS** |
| **G7** | digest del critic **cambiato** | **PASS** |
| **G8** | hash del modulo completo diverso da J19A | **PASS** |
| **G9** | struttura di checkpoint e ottimizzatore | **PASS** (parte immediata) |
| **G10** | nessun retry, nessun crash, nessun timeout | **PASS** |
| **G11** | telemetria `training_health` completa | **PASS** |
| **G12** | foglia J19A byte-invariata | **PASS** |

> **G9 non è chiuso.** La parte immediata (struttura del checkpoint e indici Adam)
> è verificata; la prova di *restorabilità reale* richiede uno stadio separato a
> zero iterazioni, che non è stato eseguito. È esattamente ciò che dichiara il
> marker `RESTORE_AUDIT_PENDING`.

## 5. Metriche

### Critic — ha imparato

| Grandezza | Valore |
|---|---|
| `vf_loss` | **0,002849091310054064** (sorgente: chiave di riga) |
| `vf_explained_var` | **0,8018040657043457** |
| `vf_loss_unclipped` | 0,002849091310054064 (= clipped, nessun clipping attivo) |
| digest critic **prima** | `5ce4ef41d5677623de42081adc23b6f4168d3a0141d222f0ec64c683203e1096` |
| digest critic **dopo** | `2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410` |

Varianza spiegata di 0,80 dopo una sola iterazione su 4096 passi: il critic fresco
ha assorbito una quota sostanziale del segnale di ritorno. È una misura su un
singolo batch, non una stima di generalizzazione.

### Attore — non si è mosso

| Grandezza | Valore |
|---|---|
| digest attore `before_training` | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| digest attore `after_iteration` 1 | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| `max_abs_diff` a entrambi gli stadi | **0.0** |
| voci di audit del freeze | 2 (prima, dopo) |
| bias log-std (righe 2:4) | −5,2983174324035645 su entrambe le dimensioni |
| peso log-std (righe 2:4) | esattamente 0 → σ indipendente dallo stato |
| σ | 0,004999999670722372 su entrambe le dimensioni |
| entropia osservata | −7,758757591247559 |

L'entropia è una verifica indipendente della σ: per due dimensioni gaussiane,
`H = 2·(½·ln 2πe + ln σ)` con σ = e^−5,2983174 dà **−7,758758**, che coincide con il
valore misurato. La σ non si è mossa.

### KL

| Grandezza | Valore |
|---|---|
| `mean_kl_loss` | **0,0** (esattamente) |
| `curr_kl_coeff` | 0,10000000149011612 |
| `kl_update_guard` nella riga | `null` — guard disattivata per progetto |
| `kl_minibatch_count` / `kl_nonfinite_count` | `null` (learner custom non installato) |

KL identicamente nulla: l'attore è congelato, quindi la policy nuova coincide con
quella vecchia in ogni minibatch. È la conferma numerica attesa del freeze.

### Passi e iterazioni

| Grandezza | Valore |
|---|---|
| `num_env_steps_sampled_lifetime` | **4096,0** |
| delta per-iterazione (`learner_connector_sum_episodes_length_in`) | **4096,0** |
| `iterations_completed_this_process` | 1 |
| righe in `train_iterations.jsonl` | 1 |
| `iteration_start` / `next_iteration` | 1 / 2 |
| `num_module_steps_trained` | 41472 (81 × 512) |
| `step` Adam registrato | 81 |
| `module_train_batch_size_mean` | 512,0 |
| copertura di partenza | `offset_1p956871s: 4096,0` — start nominale unico |
| terminazioni / troncamenti | 0 / 0 |

### Altre grandezze di loss

`policy_loss = −0,10088169574737549`, `total_loss = −0,09803260117769241`. Sono
calcolate e riportate ma **prive di effetto**: il gradiente dell'attore è staccato
via `_detach_actor_gradient` / `_detach_logstd_gradient`, quindi nessun peso
dell'attore le riceve. Lo conferma il digest invariato.

## 6. Contatori di telemetria `training_health`

| Campo | Valore |
|---|---|
| `observed_rows` | **4096,0** |
| `missing_telemetry_rows` | **0,0** |
| `phase_timeout_stance_rows` | 0,0 |
| `morphology_causal_contract_failure_rows` | 0,0 |
| `resync_event_rows` | 0,0 |
| `resync_count_max` | 0,0 |
| `hs_cancelled_count_max` | 0,0 |
| `timeout_side_disagreement_rows` | 0,0 |

La partizione richiesta è esatta: `observed_rows + missing_telemetry_rows = 4096`,
cioè l'intero batch. Ogni passo campionato ha prodotto telemetria leggibile.
Nessun incidente su nessuno dei sei contatori: nessun timeout di fase in stance,
nessuna violazione del contratto causale della morfologia, nessun resync, nessun
heel-strike cancellato. **Sono osservazioni, non gate**: non hanno vincolato
l'esito, e sarebbero state registrate identicamente se fossero state non nulle.

## 7. Trapianto warm-start

`warm_start_mode = actor_only_drop`, `critic_init_mode = fresh_target_untouched`,
`optimizer_source_loaded = false` (Adam parte pulito, come voluto).

Le quattro validazioni di integrazione sono tutte esatte:

- attore del learner: `exact = true`, `max_abs_diff = 0.0`, digest `d4a13ff7…`
- critic del target: `exact = true`, digest `5ce4ef41…` — **il trapianto non lo ha toccato**
- **14** EnvRunner controllati (13 remoti + 1 locale), tutti con digest `d4a13ff7…`
- attore ri-esportato: `exact = true`, digest `d4a13ff7…`
- `weights_synced_before_first_sample = true`

Una sottigliezza che vale la pena registrare: il trapianto dichiara
`shared_features_zeroed = ['gait_phase_sin', 'gait_phase_cos']`, perché il clock di
andatura è disabilitato. Ho verificato che nell'attore J19A quelle due colonne di
`pi_encoder.0.weight` sono **già** esattamente nulle (`max|·| = 0.0`, contro 0,2430
sulle restanti 33 colonne). L'azzeramento è quindi un no-op, ed è la ragione per cui
il digest dell'attore sopravvive intatto al trapianto.

## 8. Struttura di checkpoint e ottimizzatore

`checkpoint_last_meta.json`: `logical_iteration = 1`, `rllib_training_iteration = 1`.

`checkpoint_last/learner_group/learner/state.pkl` — chiavi
`['metrics_logger', 'optimizer', 'should_module_be_updated', 'weights_seq_no']`.

Dentro `optimizer['default_policy_default_optimizer']['state']`:

- **12** parametri nei `param_groups`
- **6** voci di stato Adam, agli indici **[6, 7, 8, 9, 10, 11]** — esattamente i sei
  tensori del critic, nessuno dei sei dell'attore
- ogni voce ha `step`, `exp_avg`, `exp_avg_sq`; `step = 81` per tutte
- learning rate 1e-4, ottimizzatore `Adam`

Questo è il fatto più diretto: **l'ottimizzatore ha momento solo sul critic.**
L'attore non ha nemmeno una voce di stato Adam.

`checkpoint_last/…/module_state.pkl` ha **16 chiavi** (10 attore + 6 critic), hash
`57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168`, diverso da
quello J19A `8153dc97…` a 10 chiavi — come deve essere, dato che ora il critic esiste.

## 9. Verifica incrociata indipendente

Oltre alla verifica post-commit del runner (`commit_verification.json`, `ok = true`,
`problems = []`), ho ricalcolato ogni affermazione **dai byte grezzi**, con uno script
separato che non importa e non si fida del runner. Esito: **24/24 PASS**.

In particolare la mia implementazione indipendente del digest canonico
(`sha256` di chiave utf-8 + hexdigest ASCII di dtype/shape/byte C-order, sulle dieci
chiavi ordinate) riproduce esattamente `d4a13ff7…` per l'attore e `2fa9c124…` per il
critic. Byte-identità verificata come dtype + shape + byte C-order, **non** con
`numpy.array_equal`.

Ho inoltre ri-hashato **tutti i 41 pin del GO**: coincidono tutti. Le foglie J19A,
J19B, J19C, K1 e K1R1 sono byte-invariate; il receipt dichiara
`j19a_leaf_unchanged`, `k1_leaf_unchanged`, `k1r1_leaf_unchanged` tutti `true`.

## 10. File

**Creati** (foglia, 18 elementi, nessuno sovrascritto):
`v26c_j20_critic_warmup_receipt.json`, `v26c_j20_critic_warmup_result.json`
(`6fb4200c…`), `commit_verification.json`, `summary.json`,
`train_iterations.jsonl`, `checkpoint_last/`, `checkpoint_last_meta.json`,
`milestone_iteration_000001/`, `rl_module_last/`, `rl_module_initial_warm_start/`,
`actor_transplant_report.json`, `training_cfg.resolved.yaml`, `tensorboard/`,
`rllib/`, `watchdog_state.json`, `faulthandler.log`, `child_stdout_stderr.txt`,
`RESTORE_AUDIT_PENDING`.

**Non modificati**: plugin C++ / SEA, FSM v3, detector, morfologia, reward, soglie,
architettura, σ, `training_exnovo_cfg.yaml`, `v26c_j20_warmup_cfg.yaml`, e tutti gli
artefatti J0–J19C. Le uniche modifiche di produzione in essere restano quelle
additive già approvate e pinnate in `tb_logging.py` (`b5543e28…`) e
`train_ppo_mlp.py` (`cbd278c3…`).

## 11. Che cosa questo risultato **non** dimostra

- **Non** dimostra che il checkpoint sia ripristinabile. La struttura è corretta;
  il ripristino effettivo non è stato tentato. G9 resta aperto.
- **Non** dimostra che il critic sia buono. `vf_explained_var = 0,80` è misurata sul
  batch su cui il critic è stato appena addestrato, in-sample per costruzione.
- **Non** dice nulla sulla qualità della policy: nessun episodio si è concluso
  (`episode_return_mean` e `episode_len_mean` sono `null`, 0 terminazioni e 0
  troncamenti su 4096 passi), quindi non esiste alcun ritorno da confrontare.
- **Non** autorizza PPO, ex-novo, promozione o restore: `next_stage_authorized =
  false`, `training_ready = false`, `promotion = NONE`.

## 12. TODO propagati

- [ ] **Restore audit a zero iterazioni** che chiuda G9: stadio distinto
      `V26C_J20_RESTORE_AUDIT`, con prereg sigillata (`1326944e…`) più gli emendamenti
      immutabili rev1 (`3fa1ae23…`, R9 live + R13) e rev2 (`59e2e5e0…`,
      normalizzazione), child wrapper validation-only (`351598a5…`), runner
      (`7f179b97…`), suite ermetica 209/209 (`b908fe48…`) e GO **DRAFT inerte**
      (`1845b201…`, 62/62 pin). Preflight READY, foglia assente.
      **L'esecuzione richiede una review e un GO operativo espliciti.**
- [ ] TODO ereditati ancora aperti dai report precedenti: generalizzazione
      multimodello (epic del 22/08); chiusura del gate finale di recupero AB06.

---

**Stato conclusivo:** warm-up critic-only eseguito **una sola volta**, terminato
regolarmente, 12/12 gate immediati passati, verifica incrociata indipendente 24/24.
Il critic è cambiato, l'attore e la σ non si sono mossi di un bit. In attesa della
review dell'architetto sul restore audit.
