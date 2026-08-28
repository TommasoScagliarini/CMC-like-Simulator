# V26C J10R1 — Raccolta multistart prescribed-teacher: esecuzione e audit

**Data**: 2026-08-27
**Stadio**: `V26C_J10R1_MULTISTART_TEACHER`
**Esecuzioni**: 1 (una sola, nessun retry)
**Exit code**: 0
**Verdetto**: **PASS** — 2/2 celle, coverage gate superato
**Esito conferito**: **NESSUNO**. Questa fase produce un dataset e non autorizza fit, critic, PPO,
promozione o deployment.

---

## 1. Problema

La qualifica closed-loop **J9R1 era FALLITA sulla cella B** (start `-0.20 s`): 0 cicli validi, 0
eventi. Il contratto canonico `training_exnovo_cfg` usa `exact_start_sampling: true` con tutti e
tre gli offset, quindi il checkpoint **non è training-ready finché B fallisce**.

Decisione architetturale: applicare la **metodologia multistart di luglio esclusivamente alla
lineage August V26**, raccogliendo label del teacher prescritto ai due start non nominali, in modo
che un fit successivo e separato possa colmare il gap di supporto misurato in J7 — dove
`phase_fsm_wait_hs` è costante a zero su tutte le 16713 righe.

La readiness **J10 era stata rifiutata prima dell'esecuzione**; J10R1 è il successore additivo che
ne applica le cinque correzioni obbligatorie. Dettagli in
`reports/user/2026-08-27_v26c_j10r1_multistart_teacher_readiness.md`.

---

## 2. Strategia

Due rollout del **teacher prescritto** (`target_domain_imitation.prescribed_teacher_action`,
riuso read-only), 500 step ciascuno, ordine congelato **B poi C**, seed 123, σ = (0, 0),
lookahead 0. Non è una policy: nessun RLModule caricato, nessun cluster Ray, nessun env runner.

Il parent August V26 (`module_state.pkl` = `0ba56eb7…`) fissa la lineage e **non è mai la sorgente
d'azione**: non viene caricato in alcun punto. Lo studente J8 non è mai parent, policy o sorgente
di label.

### Controlli pre-esecuzione (tutti superati, fail-closed)

- **34 hash frozen verificati**: 11 input dell'authorization, 6 repo, il parent, 7 pin locali del
  prereg, 6 pin repo del prereg, e i 5 del bundle J10 rifiutato.
- **Selftest**: 357 check, PASS.
- **Preflight inerte**: GO, nessun blocker; nessun ambiente costruito/resettato/steppato, nessuna
  policy caricata, nessun output scritto, nessun modulo pesante introdotto.
- **Assenze confermate**: nessun leaf, lock, staging o sentinella.

### Comando eseguito

Esattamente interpreter, cwd e argv congelati in
`v26c_j10r1_multistart_teacher_authorization.json`, senza flag aggiunti.
`OUTPUT_ROOT_OVERRIDE` non impostata (ed è comunque rifiutata dal runner per una raccolta reale).
stdout, stderr ed exit code catturati per sola redirezione, senza alterare la semantica del comando.

---

## 3. Risultati

### Verdetto

| | |
|---|---|
| verdict | **PASS** |
| aggregate_pass | `true` |
| celle behavioural PASS | 2 / 2 |
| coverage gate | **PASS** — 20 righe `phase_fsm_wait_hs == 1` |
| commit_verification | **pass: true**, 46 file e 2 directory ri-verificati |
| `TECHNICAL_INVALID` | **assente** (corretto) |
| lock / staging | rilasciati |

### Per cella

| | cella B (`-0.20 s`) | cella C (`+0.20 s`) |
|---|---|---|
| offset | `1.756870983805102` | `2.156870983805102` |
| verdict | PASS, `failed: []` | PASS, `failed: []` |
| gate checks | 15/15 OK | 15/15 OK |
| step | 500 | 500 |
| `end_reason` | `episode_time_limit` | `episode_time_limit` |
| reset time | `13.746870983805103` | `14.146870983805101` |
| errore di reset | **0.0 s** | **0.0 s** |
| righe WAIT_HS | **20** | 0 |
| cicli validi | 3 (HS 4, TO 3) | 2 (HS 3, TO 3) |
| `action_clipped_steps` | 0 | 0 |
| `episode_return` (diagnostico) | 62.125 | 46.887 |
| sim_outputs | **19** regolari, esatto | **19** regolari, esatto |
| hash registrati | 4 + 19 = **23** | 4 + 19 = **23** |

### Penetrazione (contratto 20/25/28 mm)

| | cella B | cella C |
|---|---|---|
| max | `0.0229442670858472` m | `0.022943897873928497` m |
| banda | `above_soft_below_july_legacy` | `above_soft_below_july_legacy` |
| `> 0.020` (soft, diagnostico) | 98/500 (19.6 %) | 112/500 (22.4 %) |
| `>= 0.025` (July legacy, diagnostico) | **0** | **0** |
| `> 0.028` (**hard binding**) | **0** | **0** |
| `binding_verdict` | **PASS** | **PASS** |
| media | 0.00903 m | 0.00992 m |

Valutata da `v26c_penetration_contract.evaluate_series`, contratto
`95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461`.

### Coverage: il gap J7 è colmato

Il gate era vincolante sull'**intera collezione**: almeno una riga con `phase_fsm_wait_hs == 1`.
Risultato: **20 righe**, tutte nella cella B. J7 ne aveva **zero su 16713**.

La feature è risolta **per nome dall'ambiente runtime vivo** (`base.actor_feature_names`,
larghezza 35), mai dal manifest del parent a 39 colonne. Il receipt lo dichiara esplicitamente.

### Telemetria e FSM

`telemetry_integrity` PASS su entrambe le celle: `hs_at_least_cycles`, `to_at_least_cycles` e
`hs_to_difference_within_one` tutti veri, `qualification_technically_valid: true`.

---

## 4. Verifica indipendente del leaf committato

Oltre alla verifica interna del runner, ho ricalcolato tutto dall'esterno:

- **46/46 hash riprodotti** indipendentemente (23 per cella: 4 artefatti + 19 sim_outputs).
- `commit_verification.json`: `pass: true`, `verified_against: "the committed receipt"`,
  `cells_verified: 2` su `cells_expected: 2`, `receipt_matches_staging_bytes: true`,
  `paths_missing: []`, `hash_mismatches: []`.
- **Dataset**: entrambi `observations (500, 35)`, `actions (500, 2)`, `times (500,)`.
  `executed_actions == actions` **bit a bit**, rumore **esattamente zero**, tempi strettamente
  monotoni, 35 nomi di feature runtime.
- **B e C sono distinti**: observations e times diversi, `t0(C) − t0(B) = 0.4 s` esatti.
- I 19 sim_outputs hanno **gli stessi 19 nomi** nelle due celle.
- **Parent invariato**: `0ba56eb7…` prima e dopo.
- `inert`: `fit_executed: false`, `ppo_updates: 0`, `student_used: false`,
  `policy_queried: false`, `rllib_checkpoint_loader: false`, `ray_cluster_started: false`,
  `env_runners: false`.
- `stack`: `production`, `operational: true`, `injected: false`.

Una conferma incidentale: i tempi di reset registrati sono **esattamente** i valori
`13.746870983805103` e `14.146870983805101` che avevo corretto nella preregistrazione dopo il
rilievo del reviewer. Le costanti J10 (`…102` per entrambe) sarebbero state sbagliate nell'ultima
cifra.

---

## 5. Anomalie e osservazioni

1. **Nessuna anomalia bloccante.** Nessun difetto tecnico, nessun mismatch, nessun leaf invalido.
2. **I due massimi di penetrazione coincidono a 3.7e-7 m** (`0.0229442670…` vs `0.0229438978…`).
   Ho verificato che **non** si tratta di dati duplicati: le due serie condividono solo 38 campioni
   su 500, con argmax a indici diversi (396 vs 356) e medie diverse. La profondità massima appare
   essere una proprietà della geometria terreno/piede, sostanzialmente invariante allo start.
   Osservazione, non difetto.
3. **WAIT_HS compare solo nella cella B** (20 righe, 0 in C). Il gate è collection-wide e non
   richiede nulla per cella, quindi è conforme; ma va registrato che il contributo alla copertura
   del gap J7 viene **interamente da B**.
4. **Contrasto con J9R1**: in closed-loop la cella B aveva **0 cicli validi**; qui il teacher
   prescritto ne produce **3**. Questo delimita il problema: il gap è nella policy, non
   nell'ambiente né nello start. È esattamente ciò che rende utili queste label — e **non**
   costituisce di per sé una prova che un fit le userà con successo.
5. **stderr** contiene solo tre warning attesi del simulatore: due `RuntimeWarning` di SLSQP su
   clipping ai bound, e un fallback a least-squares vincolati dello `StaticOptimizer`. Nessun
   errore.
6. Il flag `--no-progress` non è stato passato (come da authorization), quindi le righe di
   progresso sono mescolate ai log di recruitment in stdout.

---

## 6. File creati

### Leaf committato

`Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/`

| File | SHA-256 |
|---|---|
| `v26c_j10r1_multistart_teacher_receipt.json` | `9c1c26f6e1aaaa96f2c92cb45f6494e889b3124dd6ca99427a11026a8d099f20` |
| `commit_verification.json` | `59cd1563ad23713ef00ac5b2e95a334a088257af95c6727520fb895c61a8c61c` |
| `j10r1_cell_B_teacher_dataset.npz` | `2f37fc7cb101550d2fc0f8709cfdfc44ae5e9ae53003bb7903fcb590406acc62` |
| `j10r1_cell_C_teacher_dataset.npz` | `bd78e6ac13ab96d128f57ea5b36d058f5d80c18cfb056bcc76d2179bf1d756f0` |

Più, per cella: `trace.json`, `kinematics.npz`, `penetration.npz` e `<cella>_sim_outputs/` con 19
file. Tutti gli hash sono nel receipt, per path leaf-relative.

**I due SHA dei `teacher_dataset.npz` sono VINCOLANTI**: un futuro J11 dovrà verificarli prima di
consumare i dataset. Registrarli **non autorizza alcun fit**.

### Cattura dell'esecuzione

`…/v26c_july_replica_2026-08-26/j10r1_execution_2026-08-27/`

| File | SHA-256 |
|---|---|
| `j10r1_run.out` (47 951 B) | `1f3a78247a677806553cf75ebebf6e346165b2b47e81e19a52fd4979237a470d` |
| `j10r1_run.err` (618 B) | `0a33c525cdb02eb54efa6610fb669bdbed01c6f9237f334781d4d688d033f34d` |
| `j10r1_run.exit` (`exit=0`) | `19eaf43821a7660ec323a87c8457bf74823beb296c39f5e01aa8a683aa50f061` |

### Bundle di stadio (invariato dall'esecuzione)

| File | SHA-256 |
|---|---|
| `v26c_j10r1_prereg_multistart_teacher.json` | `72fcd6b9971ac2d3a2d8dda68af8d1aec9aba366c0a2a9ca679317d26fa12874` |
| `v26c_j10r1_multistart_teacher.py` | `b0e13e599e91aeeafb5ae178d8b164aa670645a7c97c4584af5344cc97c700e6` |
| `test_v26c_j10r1_multistart_teacher.py` | `6debac1897cc61ceaf3f9f7b411136ebfb92ccb07e4276961f1cceac94b73ee2` |
| `v26c_j10r1_multistart_teacher_authorization.json` | `36bb09a14d76dee90e93b5efb1f9e18db5b8da08b5a3a7cee831c06f02d23006` |

**Nessun file J0–J10R1, nessun artefatto July, nessun report precedente e nessuna configurazione di
produzione è stato modificato.** FSM, corridoio morfologico, reward, SEA e plugin C++ non sono
toccati. Il bundle J10 rifiutato resta byte-identico ai suoi cinque pin.

---

## 7. Test e verifiche eseguite

- 34 hash frozen verificati prima dell'esecuzione: tutti OK.
- Selftest sintetico: 357 check, PASS.
- Preflight inerte: GO, nessun blocker.
- Assenza di leaf/lock/staging/sentinella confermata prima della run.
- Esecuzione singola, exit 0, lock e staging rilasciati.
- Verifica post-commit interna: 46 file, 2 directory, `pass: true`.
- Ricalcolo esterno indipendente: 46/46 hash riprodotti.
- Audit di dataset, gate, telemetria, FSM, penetrazione e provenance.
- `git status`: nessun file tracciato modificato oltre ai tre già modificati a inizio sessione.

---

## 8. TODO propagati

- **LOTO** — non integrato. TODO futuro, non J10R1.
- **LOCO** — non integrato. TODO futuro, non J10R1.
- **B1R1** — non integrato. TODO futuro, non J10R1.
- **B1R2** — non integrato. TODO futuro, non J10R1.
- **Epic generalizzazione multi-modello** — APERTO. Questa fase copre ancora AB06 e tre start dello
  stesso trial; non dimostra alcuna generalizzazione ad altri soggetti, trial o velocità.
- **J11, il fit che userà questi dati** — NON fa parte di questa fase e **non è autorizzato**.
  Richiede propria preregistrazione e propria autorizzazione. Vincolo registrato: dovrà verificare
  gli SHA-256 di `j10r1_cell_B_teacher_dataset.npz` e `j10r1_cell_C_teacher_dataset.npz` prima di
  consumarli.
- **J9R1 FAIL sulla cella B** — resta aperto. Questa raccolta fornisce le label, **non** dimostra
  che un fit chiuderà il gap. Il checkpoint resta non training-ready.
- **Copertura WAIT_HS asimmetrica** — le 20 righe vengono tutte dalla cella B. Da valutare se
  20 righe siano sufficienti per il fit; questa fase non lo asserisce e non ha inventato soglie.
- **Bundle J10 rifiutato** — resta byte-identico come evidenza. TODO solo per l'architetto:
  decidere se archiviarlo.

---

## 9. Cosa questo PASS **non** conferisce

- Non autorizza il fit, il warm-up del critic o PPO.
- Non promuove alcun attore.
- Non dichiara nulla di deployabile.
- Non autorizza lo stadio successivo.
- Non modifica FSM, corridoio morfologico, reward, SEA o plugin C++.

**Fermo in attesa dell'architetto.**
