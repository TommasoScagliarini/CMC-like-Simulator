# V26C J15R1 — Esecuzione: refit fresco 35D post-mismatch, PASS supervisionato

**Data**: 2026-08-27
**Stadio**: `V26C_J15_FRESH_35D_POST_MISMATCH_REFIT` — nome operativo **J15R1**
**Esito**: **PASS**, `exit 0`, 83.4 s. **17/17 gate binding superati**, commit verification PASS.
**Esecuzioni**: una, e una sola. Runner corretto `r1`; il runner fallito non è stato invocato.
**Report obbligatorio di fine fase.**

> **Un PASS supervisionato NON è una pretesa closed-loop.** Questo stadio non ha costruito alcun
> environment, non ha fatto rollout e non ha toccato il critic. Dice che il fit è internamente
> sano, e nulla di più. Se il problema diagnosticato in J13 — lo swing che non si chiude — sia
> stato toccato resta **ignoto** e sarà deciso dal gate A–F di regressione, che è una fase
> separata e non è iniziata.

---

## 1. Comando, ambiente, esecuzione unica

```
cwd         : /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter : /opt/anaconda3/envs/envCMC-rllib/bin/python
argv        : Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j15_fresh_refit_r1.py
              --fit
              --authorized-stage V26C_J15_FRESH_35D_POST_MISMATCH_REFIT
              --out  <repo>/…/j15_runs/j15_fresh_refit_v26c_2026-08-27_r1
environment : PYTHONDONTWRITEBYTECODE  ASSENTE   (rimossa; assenza asserita prima del lancio)
              PYTHONPYCACHEPREFIX      ASSENTE   (rimossa; assenza asserita prima del lancio)
exit        : 0
durata      : 83.446 s
```

L'argv **non è stato alterato**: è stato letto da `3_frozen_argv` del record GO e passato invariato.
`argv_altered: false`, `failed_runner_invoked: false`.

Prima del lancio il driver ha riverificato: hash del record GO
(`f0bab097a68ab25ccea1297ef03c8d64928ae660ba7df3c7d9cfdb653722eea8`), `execution_permitted_now`,
`times_permitted == 1`, i **5 pin J15R1**, i **9 artefatti della catena fallita**, che l'argv punti
al runner **corretto**, e l'assenza di leaf, lock e staging.

`executions_performed: 1`. **Dopo il successo l'argv congelato non è stato più invocato**, in
nessuna forma: tutto l'audit che segue è read-only sui file committati.

---

## 2. Inventario della leaf — esattamente 8 file

```
j15_runs/j15_fresh_refit_v26c_2026-08-27_r1/
```

| file | SHA-256 |
|---|---|
| `v26c_j15_aggregate_dataset.npz` | `f4d0c09c78e812a78910626789043e42a6ee851e99b56e59595f4887d7faebe8` |
| `v26c_j15_fresh_refit_receipt.json` | `b7b3e9c7db116af4e19d2807359914a03d7d989824c97393dc997994213de349` |
| `commit_verification.json` | `551a9cfd8d727ba181bb8ffcb7611d002b49dc77e4a0c78599c436e2378f8821` |
| `history.json` | `71c375ebad117d24df8ff5b2dc573f957cbf43d2b610f0ea43023a7cd9523c81` |
| `rl_module/module_state.pkl` | `4d084a2a7f0012bd711f39a987dbb7af30b04e265484128c45b0a92b612ab928` |
| `rl_module/actor_feature_manifest.json` | `bb24bedca3f8572e370d92ff02640a3890171888215017cd2821d62245670653` |
| `rl_module/class_and_ctor_args.pkl` | `897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02` |
| `rl_module/metadata.json` | `3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12` |

**Otto file esatti, nessun extra.** Nessun marker `TECHNICAL_INVALID` — rimosso come ultima
scrittura dopo che la verifica post-commit è passata. Nessun lock, nessuno staging residuo. I due
sidecar sono **byte-identici** a quelli del parent J2.

**Commit verification**: `pass: true`, 6/6 file registrati e ri-hashati, 0 path mancanti, 0
mismatch, `receipt_matches_staging_bytes: true`, e lo SHA del receipt registrato coincide con il
file su disco.

**Identità dell'attore**: `actor_digest` `c3551341b9b017b40585ee1f50f8ee4ecaa7ec50722f37142badc2a44b6c8590` ·
`source_actor_digest` (J2) `59d54240ac628dcbd1d0dbf34328145afa08a7028047a5923298782b79bf5188`.
Questi due **non sono hash di file** ma digest `warm_start.actor_state_digest` calcolati sui
tensori; coincidono fra receipt e manifest, e il source coincide con il digest di J2 che J11 aveva
registrato — stesso parent, verificato per via indipendente.

---

## 3. I 17 gate BINDING — tutti superati

`pass: true`, `failed: []`, `identical_to_j11: false`, `16 preservati da J11 + 1 = 17`,
`j11_set_preserved_exactly: true`.

### 11 controlli di integrità

| gate | esito |
|---|---|
| `integrity_keys_and_shapes_match_parent` | PASS |
| `integrity_all_parameters_finite` | PASS |
| `integrity_clock_bit_zero` | PASS |
| `integrity_aliases_bit_identical` | PASS |
| `integrity_logstd_bit_identical_to_parent` | PASS |
| `integrity_no_critic_key` | PASS |
| `integrity_inputs_unchanged` | PASS |
| `integrity_aggregate_reproduces_content_hashes` | PASS |
| `integrity_split_counts_as_declared` | PASS |
| `integrity_best_state_reconstructible_from_history` | PASS |
| `integrity_all_metrics_finite` | PASS |

### 5 RMSE che dovevano decrescere — disuguaglianza stretta, nessuna soglia

| gate | righe | slice | prima | dopo | var. |
|---|---|---|---|---|---|
| `aggregate_rmse_decreases` | 25567 | `[0, 25567)` | 0.07786176 | **0.03930892** | −49.5% |
| `recovery_rmse_decreases` | 713 | `[16000, 16713)` | 0.08836083 | **0.08112133** | −8.2% |
| `cell_B_unique_rmse_decreases` | 500 | `[16713, 17213)` | 0.06390679 | **0.01737228** | −72.8% |
| `cell_C_unique_rmse_decreases` | 500 | `[20713, 21213)` | 0.02024800 | **0.01996982** | −1.4% |
| **`j14_increment_rmse_decreases`** | **854** | **`[24713, 25567)`** | **0.39233834** | **0.18155497** | **−53.7%** |

### 1 controllo strutturale

`controller_columns_nonzero` — le dieci colonne del blocco controller hanno norma L2 > 0. PASS.

### Il 17° gate, nel merito

Vincolava l'**intero blocco** di 854 righe, cellule E ed F insieme, ed è **superato**: la RMSE cala
del 53.7%. È il gate che il report J13 approvato aveva preregistrato e che tu hai promosso a
binding. Il refit ha imparato la regione correttiva per cui è stato costruito — **in senso
supervisionato**.

---

## 4. Diagnostici — nessuno vincola nulla

Riportati per visibilità. **Nessuno di questi è un gate**, e nessuna direzione era preregistrata
per loro.

| diagnostico | righe | prima | dopo |
|---|---|---|---|
| `j14_cell_E_rmse` (seed 124) | 500, `[24713, 25213)` | 0.35459079 | 0.19081006 |
| `j14_cell_F_rmse` (seed 125) | 354, `[25213, 25567)` | 0.44017322 | 0.16761442 |
| `j14_post_mismatch_rmse` | 671 | 0.43633450 | 0.19901529 |
| `j14_pre_mismatch_rmse` | 183 | 0.14229725 | 0.09272353 |
| `j11_prefix_rmse` | 24713 | 0.03086519 | 0.02143647 |
| `nominal_rmse` | 16000 | 0.00000007 | **0.01553584** |
| `j14_phase_invalid_fraction` | — | — | 0.785714 (671/854) |
| `clipping_out_of_bounds_rows` | — | — | 0 |
| `best_validation_mse` | — | — | 0.00150507 |
| `epochs_run` / `stopped_early` | — | — | 400 / false |

Osservazioni, senza trarne conclusioni non supportate:

- **E ed F migliorano entrambe**, F più di E (−61.9% contro −46.2%). La cella F era quella che in
  J12 falliva prima (step 354).
- La **regione post-mismatch**, le 671 righe che J13 ha indicato come causali, cala del 54.4%; le
  183 pre-mismatch — le uniche che un troncamento in stile J7 avrebbe tenuto — del 34.8%.
- Il **prefisso J11** migliora anche lui (−30.6%): aggiungere le 854 righe non ha peggiorato le
  righe che J11 già adattava.

---

## 5. Due cose che devo segnalare

### 5.1 Un diagnostico dichiarato ma mai calcolato

`gate_matrix()` elenca **13** diagnostici, il receipt ne contiene **12**: `nominal_mean_shift` è
dichiarato e **non viene mai misurato** da `audit()`.

Non vincola nulla, non tocca il verdetto né alcun gate, e nessuna delle 17 decisioni binding
dipende da esso. Ma è una dichiarazione non sostenuta da una misura, ed è esattamente il tipo di
disallineamento che non va lasciato passare. **Non l'ho corretto**: la leaf è committata e
immutabile, e nessuna correzione è autorizzata. Lo segnalo perché tu decida.

### 5.2 Il blocco nominale si è spostato più che in J11

`nominal_rmse` passa da ~0 (rumore forward torch/numpy) a **0.01554**, contro **0.00538** di J11:
**2.89×**. Il blocco nominale è il self-anchor e ci si aspetta che si muova — per questo non è un
gate, e nessuna direzione era preregistrata. Ma la deriva è più marcata che in J11, ed è coerente
con l'aver aggiunto 854 righe la cui RMSE iniziale era 0.392.

July usava storicamente una soglia 0.005 su `nominal_mean_shift`, una grandezza diversa e qui non
misurata (§5.1). Non la importo: sarebbe inventare un gate a posteriori. La registro come rischio.

---

## 6. Contratto scientifico, verificato sul committato

| controllo | esito |
|---|---|
| stato del modulo | 10 chiavi, le stesse del parent J2, tutte float32 e finite |
| **critic** | **ASSENTE** — nessuna chiave `critic` o `vf*` |
| **clock** | `max\|W[:, 0:2]\| = 0.0` **esatto**, sia su `pi.0.0.weight` sia sull'alias |
| **logstd** | **bit-identica al parent**, bias `[−5.2983174, −5.2983174]` (σ = 0.005) |
| alias `pi_encoder.*` | bit-identici ai corrispondenti `pi.*` |
| mean network | **cambiata** rispetto al parent su tutti e tre i tensori — il fit ha fatto qualcosa |
| aggregato | (25567, 35) / (25567, 2) float32, clock zero esatto |
| prefisso `[0, 24713)` | **bit-identico** all'aggregato J11 |
| coda `[24713, 25567)` | **bit-identica** all'incremento J14 |
| blocchi | j7_nominal 16000×32 · j7_recovery 713×1 · cell_B 4000×8 · cell_C 4000×8 · **j14_increment 854×1** |
| split | **5113 / 20454** |
| parent | **J2** `0f182ea9…` · **J11 mai caricato** |
| semi sigillati 126/127/128 | **letti: 0** |
| rollout / environment / critic / PPO | **nessuno**, `ppo_updates: 0` |
| sorgenti pinnate dopo il fit | **invariate** (`integrity_inputs_unchanged` PASS) |

**Training**: 400 epoche su 400, best epoch **397**, `stopped_early: false`,
`best_validation_mse` 0.00150507.

**Registrato per la prima volta**, colmando le lacune di J11: **torch 2.10.0** e gli argomenti Adam
risolti — `lr 5e-05`, `betas [0.9, 0.999]`, `eps 1e-08`, `weight_decay 0`, `amsgrad false`.

---

## 7. La catena fallita, preservata

I 9 artefatti della catena J15 fallita sono stati riverificati **prima** dell'esecuzione e sono
byte-identici: runner e test J15, authorization rev3, GO rev1 e rev2, i tre log del fallimento e il
report di esecuzione fallita. Il runner fallito `7b0073c8…` **non è stato invocato**.

I log di questa esecuzione sono additivi e distinti, accanto ai precedenti in `j15_runs/`:
`j15r1_execution_stdout_2026-08-27.txt`, `…stderr…` (vuoto), `…exit….json`.

---

## 8. Limitazioni

- **Un PASS supervisionato non è una pretesa closed-loop.** Il receipt lo dice da sé:
  `closed_loop_claim: "NONE - this stage is supervised only"`.
- `deployable: false`, `promotion: "NONE"`, `next_stage_authorized: false`,
  `closed_loop_authorized: false`.
- **`validation_mse` non è una stima di generalizzazione**: ogni blocco ripetuto mette la stessa
  riga unica in entrambe le partizioni. Ereditato da July, J7 e J11; registrato, non corretto.
- **Il critic è assente, non preservato**: un futuro stadio PPO parte da una value function
  inizializzata a caso.
- **L'etichetta resta un'assunzione**: il teacher è funzione pura del tempo, quindi definito
  ovunque; che sia il bersaglio giusto *dopo* la divergenza della FSM non è testato da questo
  stadio, e J13 lo aveva già dichiarato tale.
- **Le 854 righe sono il 3.3% dell'aggregato.** Che questa massa basti a spostare il comportamento
  closed-loop resta **ignoto**.

---

## 9. TODO propagati

- **`nominal_mean_shift` dichiarato e non misurato** (§5.1) — decisione tua.
- **La deriva del blocco nominale, 2.89× J11** (§5.2) — registrata come rischio.
- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Semi 126, 127, 128** — riserva held-out finale, mai letti.
- **Il gate A–F di regressione closed-loop** — è lì che si decide se il problema J12 è stato
  toccato. **Non iniziato.**
- **Regola di governance**: ogni fase si chiude con uno user report dedicato, auditato prima della
  fase successiva. Questo report chiude la fase J15R1-fit.

---

## 10. Stato della fase successiva

Il GO J15R1 autorizzava **una** esecuzione ed è **consumato**. L'argv congelato non va più invocato.

**Non ho iniziato nulla**: né la fase closed-loop A–F, né il critic warm-up, né PPO, né alcuna
readiness di fase successiva.

**Fermo in attesa del tuo audit.**
