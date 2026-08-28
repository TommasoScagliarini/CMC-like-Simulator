# V26C J19B — Esecuzione della qualificazione closed-loop A–F

**Data**: 2026-08-27
**Stadio**: `V26C_J19B_J19A_CLOSED_LOOP_QUALIFICATION` — **esecuzione**
**Esecutore**: Opus 5, effort xhigh
**Esecuzione**: **una sola volta**, sotto GO architetto `0c790dca…`, exit code **0**, nessun retry.

## VERDETTO: `PASS` — 6/6 celle, 0 gate falliti

L'attore J19A supera la qualificazione closed-loop vincolante. È il **verdetto certificante**: ogni
gate offline finora era un filtro, questo è la prova.

**Nessuna autorizzazione a heldout o training è implicata o concessa da questo esito.**

---

## 1. Problema

J8, l'attore operativo, passa 5 celle su 6: fallisce la cella **B**, la partenza a −0.20 s. La catena
J18 → J19A ha prodotto un candidato che corregge quel blocco preservando il comportamento altrove,
riprodotto bit-identicamente e con eligibilità offline 11/11. Restava la domanda che nessuna misura
offline poteva sciogliere: **regge in closed loop?**

J17 aveva già misurato che l'offline non certifica il closed loop — J15R1 migliorava offline e faceva
0/6. La diagnosi J18 aveva misurato il verso opposto: J4, con il drift più basso di tutti, incorse
comunque in una violazione diagnostica a 25 mm.

## 2. Strategia

**Stessa qualifica, nuovo attore.** J19B è una **copia meccanica auditabile** del protocollo J16,
derivata programmaticamente con una mappa chiusa e pinnata; J16 resta byte-invariato. Otto funzioni
scientifiche sono **bytecode-identiche**, `run_cell` ha l'**instruction stream identico** con soli
quattro rinomini e due stringhe di output. Matrice, gate, sigma, tempi, seed e contratto di
penetrazione sono identici per valore.

Fit e rollout non sono mai stati concatenati: J19A ha chiuso con report e audit prima che J19B
fosse preregistrata.

## 3. Esecuzione unica

GO verificato **prima** del lancio: SHA-256
`0c790dca61ac73a2ff5b10fb288244c6c81049d03f5bf81ca47a5387b1f114a4`, stage corretto, **sette pin tutti
allineati** — actor J19A, runner e prereg J19B, strumento di derivazione, suite di test, J16 e
contratto di penetrazione. `j19b_runs` **assente** prima del lancio.

Ambiente reale: plugin SEA e `OnlineGRFContact` caricati, modello `AB06_SEASEA` con 21 coordinate
(19 bio, 2 protesiche), 76 muscoli, 19 riserve SO, GRF online ibrida — 8 contatti applicati a
sinistra, 12 sensor-only — detector binario a 2 punti, IK filtrata a 6 Hz.
Stack **production**, non iniettato.

Sei celle in sequenza A–F, **senza fail-fast comportamentale**. Nessuna interruzione, nessun retry.

---

## 4. Risultati

### Aggregato

| | |
|---|---|
| verdetto | **PASS** |
| celle comportamentalmente PASS | **6 / 6** |
| celle telemetry-valid | **6 / 6** |
| regola di aggregazione | PASS se e solo se 6/6 comportamentali **E** 6/6 telemetry-valid |
| gate falliti | **0** |
| `actor_unchanged` | **True** |

### Per cella: esito, FSM e morphology

| cella | verdetto | step | end_reason | cicli | HS | TO | timeout stance | timeout swing | morphology fail | resync | HS cancel | clip |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **A** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | 0 |
| **B** | PASS | 500 | `episode_time_limit` | **3** | 4 | 3 | 0 | 0 | 0 | 0 | 0 | 0 |
| **C** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | 0 |
| **D** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | 0 |
| **E** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | 0 |
| **F** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | 0 |

Tutte e sei completano i **500 step** con `episode_time_limit`: nessuna termina in anticipo. **Zero**
timeout di fase, **zero** fallimenti del contratto causale della morphology, **zero** resync, **zero**
heel-strike cancellati, **zero** step con azione clippata.

**La cella B — quella che J8 fallisce — passa, e con tre cicli validi invece di due.**

### Cinematica (rad)

| cella | knee min | knee max | ankle min | ankle max |
|---|---|---|---|---|
| A | −0.99628 | −0.16899 | −0.11189 | 0.40382 |
| B | −0.96850 | −0.17089 | −0.11843 | 0.39916 |
| C | −0.98490 | −0.12963 | −0.11089 | 0.41984 |
| D | −0.99602 | −0.16811 | −0.11405 | 0.40428 |
| E | −0.99334 | −0.16251 | −0.11114 | 0.40762 |
| F | −0.99736 | −0.16174 | −0.11101 | 0.40585 |

Escursioni coerenti fra le sei celle: ginocchio in una banda di ~0.83 rad, caviglia di ~0.52 rad, con
dispersione fra celle di poche centesimi di radiante. Nessun collasso del ginocchio come quello
misurato in J16 (0.148 rad di escursione anomala nella cella D).

### Penetrazione — bande 20 / 25 / 28 mm

| cella | max (mm) | > 20 soft | ≥ 25 July | **> 28 HARD** | banda | margine a 28 (mm) |
|---|---|---|---|---|---|---|
| A | 24.4214 | sì | no | **no** | `above_soft_below_july_legacy` | 3.5786 |
| B | 23.4726 | sì | no | **no** | `above_soft_below_july_legacy` | 4.5274 |
| C | 22.5548 | sì | no | **no** | `above_soft_below_july_legacy` | 5.4452 |
| D | 24.6818 | sì | no | **no** | `above_soft_below_july_legacy` | 3.3182 |
| E | 24.4568 | sì | no | **no** | `above_soft_below_july_legacy` | 3.5432 |
| **F** | **25.6151** | sì | **sì** | **no** | `july_legacy_breach_within_hard` | **2.3849** |

**Nessuna cella supera la barra binding a 28 mm**, che è l'unico criterio bloccante.

Due osservazioni da leggere per quello che sono, cioè **diagnostiche**:

- tutte e sei superano la soglia soft a 20 mm. Non è binding e non lo è mai stato in questa catena;
- la **cella F** supera anche la diagnostica July a 25 mm, a **25.6151 mm**. Resta 2.38 mm sotto la
  barra binding. Per contesto, non per confronto formale: la run July promossa registrava 23.27–24.60
  mm e J4/J5 registrò 26.91 mm.

Il margine minimo sulle sei celle è **2.38 mm**. Non è un margine ampio, e lo segnalo.

### D–E–F: sigma e rumore realizzato

| cella | modo | seed | rumore realizzato RMS (knee, ankle) | return |
|---|---|---|---|---|
| A | deterministic | 123 | 0.0, 0.0 | 42.2560 |
| B | deterministic | 123 | 0.0, 0.0 | 64.5394 |
| C | deterministic | 123 | 0.0, 0.0 | 47.0271 |
| **D** | stochastic_held | 123 | **0.0051578**, **0.0048870** | 42.1926 |
| **E** | stochastic_held | 124 | **0.0046604**, **0.0049809** | 42.1582 |
| **F** | stochastic_held | 125 | **0.0049045**, **0.0048957** | 42.1343 |

Il rumore realizzato nelle tre celle stocastiche sta fra **0.00466 e 0.00516**, coerente con la sigma
congelata **0.005** dell'attore. Le tre deterministiche hanno rumore **esattamente zero**, come
devono.

`policy_std` è `null` in tutte e sei: è il **difetto cosmetico noto**, ereditato da J12 e già a
verbale nei TODO propagati. Non incide su alcun gate.

---

## 5. Artefatti

Leaf: `j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1/`

**134 file su disco, 132 artefatti registrati e verificati.** I due non registrati sono il receipt
stesso — che non può contenere il proprio hash — e `commit_verification.json`, scritto dopo il
commit. Stesso schema di J18 e J19A.

Composizione: 20 file al livello superiore (receipt, commit verification, e per ciascuna cella
trace, kinematics e penetration) più **19 `sim_outputs` per cella × 6 = 114**, con conteggio
verificato esatto cella per cella.

| artefatto | SHA-256 |
|---|---|
| `v26c_j19b_closed_loop_receipt.json` | `789c57ca293ae7d5…` |
| `commit_verification.json` | `0ded736553a55eba…` |
| `j19b_cell_A_trace.json` | `42c8c147442b1fea…` |
| `j19b_cell_B_trace.json` | `51992719be57713d…` |
| `j19b_cell_C_trace.json` | `60cbb338d24c8bd8…` |
| `j19b_cell_D_trace.json` | `c00bb5e1194b7459…` |
| `j19b_cell_E_trace.json` | `75801f7c27bf2963…` |
| `j19b_cell_F_trace.json` | `c8de604d739960b4…` |

---

## 6. Verifiche

### Verifica post-commit

`pass = True`, **132 file controllati su 132 registrati**, `hash_mismatches = []`,
`paths_missing = []`, `receipt_matches_staging_bytes = True`.
Marcatore `TECHNICAL_INVALID` **assente**: rimosso solo dopo la verifica. **Nessuna staging né lock
residua.**

### Immutabilità byte dell'actor J19A

- `actor_before == actor_after` su **tutti e sette** gli artefatti;
- `actor_unchanged = True`;
- ricalcolati **adesso**, i sette hash coincidono ancora con quelli registrati;
- `module_state.pkl` = `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530`.

L'attore è stato **letto e mai toccato**.

### Provenienza e inerzia

- prereg registrata nel receipt: `31c2705a8c9501969ddc39a37db43c3a419187febcd5cc830ff3cbfce57347b8`
  — **coincide col sigillo**;
- contratto di penetrazione: `9257e9b8cdf54d9a59bfe2ee25526b283408d325a900156a69d64dbf196298dc`,
  unica autorità sulle bande;
- equivalenza scientifica a J9R1 (`f58b0d14…`): **11 campi confrontati**, `actor_now` correttamente
  `J19A (j19a_runs/…)`;
- stack **production**, `injected = false`;
- `inert`: `fit_executed false`, `critic_touched false`, `ppo_updates 0`, `actor_edited false`,
  `actor_copied false`, `logstd_head_edited false`.

---

## 7. Decisione

**L'attore J19A ha superato la qualificazione closed-loop A–F vincolante: 6/6, zero gate falliti,
nessuna cella oltre la barra binding a 28 mm.**

Cosa questo **non** autorizza, e che non concedo né implico:

- **nessuna fase heldout**: i semi 126, 127, 128 e la fase G–I restano **sigillati e fuori
  perimetro**;
- **nessun training**, nessun fit, nessun PPO, nessun critic, nessuna collection;
- **nessuna promozione a deployable**: nessuno stadio l'ha dimostrata e il manifest J19A non la
  rivendica;
- **nessuna generalizzazione** oltre le sei celle congelate. LOTO, LOCO, B1R1, B1R2 e l'Epic
  multi-modello restano fuori perimetro.

Il risultato è circoscritto a ciò che è stato misurato: sei celle, 500 step ciascuna, con la matrice
e i criteri congelati da J9R1.

**Elementi che segnalo alla tua attenzione, non come obiezioni ma perché la decisione è tua:**

1. il **margine minimo alla barra binding è 2.38 mm** (cella F, 25.6151 mm);
2. la **cella F supera la diagnostica July a 25 mm**, unica delle sei;
3. tutte e sei superano la soglia soft a 20 mm, come già accadeva alle run precedenti;
4. il diagnostico **POST-485** del candidato, misurato in J19A, era **0.957933** — fra J11 (4/6) e
   J15R1 (0/6). Il closed loop l'ha smentito come predittore, il che è coerente con la sua
   confusione di disegno già dichiarata.

---

## 8. Invarianti verificate

- Eseguito **una sola volta**, exit code 0, **nessun retry**, nessuna interruzione per fallimenti
  comportamentali.
- **Nessun fit, nessun optimizer step, nessun aggiornamento di pesi, nessun critic, nessun PPO.**
- **Non modificati**: l'attore J19A e i suoi sette artefatti, `v26c_j16_closed_loop.py`
  (`6ac45854…`), `v26c_penetration_contract.py` (`9257e9b8…`), la prereg J19B sigillata
  (`31c2705a…`), il leaf J18, J8, J2, J4, J7, J9R1, J10R1, produzione, FSM v3, detector/morphology,
  reward, sigma, SEA/C++, architettura.
- Semi 126, 127, 128: **non letti, non generati, non usati**.
- **Nessuna fase successiva avviata.**
- Worktree sporco preservato. Nessun subagente usato in questa fase.

---

## 9. TODO propagati

- **Nessuna fase successiva autorizzata**: heldout G–I sui semi 126–128 resta sigillata e richiede
  una tua decisione separata.
- Il margine di 2.38 mm della cella F merita la tua valutazione prima di qualunque passo ulteriore.
- `policy_std` sempre `null`, difetto cosmetico ereditato da J12, ora osservato anche qui.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- Le suite readiness J18, J19A e J19B resteranno con un check storicamente falso ciascuna dopo
  l'emissione dei rispettivi GO: **non modificate**, per decisione architetturale.
- **LOTO / LOCO / B1R1 / B1R2** e l'Epic di generalizzazione restano fuori perimetro.

---

## 10. STOP

Esecuzione singola completata, verdetto **PASS 6/6**, leaf verificato, attore immutato.

**Nessuna fase successiva avviata. Fermo in attesa del tuo audit.**
