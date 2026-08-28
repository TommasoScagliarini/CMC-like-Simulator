# V26C J16 — Esecuzione: la riqualifica A–F è FALLITA 0/6

**Data**: 2026-08-27
**Stadio**: `V26C_J16_J15R1_CLOSED_LOOP_REQUALIFICATION`
**Esito**: **FAIL**, `exit 1`. **0/6 comportamentali**, 6/6 telemetrie valide.
**Esecuzione**: una, e una sola. 25.1 minuti. Leaf **committata**, commit verification **PASS**.
**Report obbligatorio di fine fase.**

> **La risposta alla domanda dello stadio è: no.** Il refit correttivo **non** ha recuperato E ed F,
> e ha **fatto regredire A–D**. Il rischio di deriva nominale che la readiness aveva dichiarato si è
> materializzato, e in una forma peggiore di quella attesa.

---

## 1. Il risultato, contro J12

Stessa matrice, stesse soglie, unica variabile l'attore.

| cella | J12 (attore J11) | **J16 (attore J15R1)** | passi J12 → J16 | penetrazione J12 → J16 | `end_reason` J16 |
|---|---|---|---|---|---|
| A | **PASS** | **FAIL** | 500 → **206** | 0.022943 → **0.028013** | `grf_penetration` |
| B | **PASS** | **FAIL** | 500 → **392** | 0.022948 → **0.028366** | `grf_penetration` |
| C | **PASS** | **FAIL** | 500 → **61** | 0.022884 → **0.028932** | `grf_penetration` |
| D | **PASS** | **FAIL** | 500 → 500 | 0.023127 → 0.026871 | `episode_time_limit` |
| E | FAIL | **FAIL** | 500 → **209** | 0.020468 → **0.028142** | `grf_penetration` |
| F | FAIL | **FAIL** | 354 → **209** | 0.020968 → **0.028106** | `grf_penetration` |

**J12: 4/6 comportamentali. J16: 0/6.** Le quattro celle che passavano non passano più.

---

## 2. Il modo di fallire è cambiato

In J12 il fallimento era uno **swing che non si chiudeva**: E ed F morivano su `valid_cycles` e
`phase_timeout:swing`, con il piede in aria — e infatti la loro penetrazione era la **più bassa**
delle sei.

In J16 il fallimento è **penetrazione nel suolo**. Cinque celle su sei terminano su
`grf_penetration`, cioè l'episodio viene troncato perché il piede sfonda oltre la soglia di
terminazione.

| cella | penetrazione max | banda | eccesso sulla barra 0.028 |
|---|---|---|---|
| A | 0.028012526 | `above_hard` | **+0.0000125** |
| B | 0.028365651 | `above_hard` | **+0.0003657** |
| C | 0.028931729 | `above_hard` | **+0.0009317** |
| D | 0.026870843 | `july_legacy_breach_within_hard` | −0.0011292 (passa) |
| E | 0.028142241 | `above_hard` | **+0.0001422** |
| F | 0.028105596 | `above_hard` | **+0.0001056** |

Gli eccessi sono piccoli — su A appena **12.5 μm** — ma la regola è stretta e senza margine, ed è
la stessa che J12 superava con 5 mm di margine. Non ho applicato alcuna tolleranza: la barra è
`<= 0.028`, esattamente com'era.

**Nessun timeout di swing o stance in nessuna cella. Nessun `hs_cancelled`. Nessun resync.** Il modo
di fallire di J13 è sparito; ne è comparso un altro.

---

## 3. Dettaglio per cella

| cella | passi | cicli validi | HS | TO | clipping | cinematica fallita |
|---|---|---|---|---|---|---|
| A | 206 | 0 | 0 | 1 | 0 | nessuna |
| B | 392 | 1 | 2 | 2 | 0 | `ankle_min`, `knee_amplitude` |
| C | 61 | 0 | 0 | 0 | 12 | `ankle_min`, `ankle_amplitude`, `knee_amplitude` |
| D | **500** | 1 | 2 | 3 | 0 | nessuna |
| E | 209 | 0 | 0 | 1 | 0 | nessuna |
| F | 209 | 0 | 0 | 1 | 0 | nessuna |

**D è l'unica cella che ha completato i 500 passi** e l'unica che resta sotto la barra hard. Supera
13 dei 14 controlli binding: fallisce solo `valid_cycles` — un ciclo valido contro i due richiesti.

**C è il caso peggiore**: muore a 61 passi su 500 e porta 12 passi di clipping dell'azione, l'unica
cella in cui il clipping è diverso da zero. Il clipping resta diagnostico, non vincolante.

Il rumore realizzato sulle celle stocastiche è coerente con σ = 0.005 (D: 0.00516/0.00489,
E: 0.00484/0.00504, F: 0.00486/0.00511); A, B, C hanno rumore esattamente zero, come deve essere
per le celle deterministiche.

---

## 4. Integrità dell'esecuzione

| controllo | esito |
|---|---|
| esecuzioni | **una**, `argv_altered: false`, nessun retry, nessuna correzione, nessun probe |
| ambiente | `PYTHONDONTWRITEBYTECODE=1` come richiesto |
| celle eseguite | **tutte e sei**, anche dopo i FAIL — nessun fail-fast |
| commit verification | **pass true**, 132/132 file ri-hashati, 0 mancanti, 0 mismatch |
| receipt ↔ byte di staging | identici |
| marker born-invalid | rimosso come ultima scrittura |
| lock / staging | **rilasciati e rimossi** |
| file nella leaf | **134** = 6 × (19 + 3) + receipt + commit_verification |
| attore | **invariato** durante la matrice (`actor_unchanged: true`), verificato prima e dopo |
| soglie inventate | **0** |
| fit / critic / PPO | **nessuno**, `ppo_updates: 0` |
| telemetria | **6/6 valide** — il FAIL è comportamentale, non tecnico |

**Il FAIL è stato committato come evidenza**, non scartato: è ciò che il protocollo prescrive, e la
leaf è valida evidenza a tutti gli effetti.

---

## 5. Una nota sul turno precedente

A metà esecuzione era arrivata una lettura secondo cui la run fosse stata uccisa da un timeout di
10 minuti, con la fase da chiudere come `TECHNICAL_INVALID / ABORTED`. Ho verificato e **non era
così**: il processo era vivo (PID 48016, 100% CPU, il PID nominato dal lock), A–B–C già complete con
19 `sim_outputs` ciascuna e D in corso. Non ho scritto quel report, perché avrebbe inciso un fatto
falso nella catena di evidenza, e non ho toccato nulla.

La run è poi arrivata a termine da sola in **25.1 minuti** — oltre la finestra di 10 minuti, il che
spiega la lettura. L'assenza del log del wrapper non era prova di un kill: il wrapper scrive
stdout, stderr ed exit **solo dopo** la terminazione del figlio.

---

## 6. Cosa questo risultato dice, e cosa non dice

**Dice**, come evidenza misurata:
- il refit correttivo **non recupera** E ed F;
- **fa regredire** A, B, C e D, che con l'attore J11 passavano;
- il regime di fallimento è ora la **penetrazione nel suolo**, non lo swing bloccato.

**Non dice** — e non lo inferisco qui — *perché*. La readiness aveva dichiarato la deriva nominale
di J15R1 come rischio principale (`nominal_mean_shift` 4.44× J11, misurata sulle righe nominali che
A–D esercitano), e il risultato è **compatibile** con quel rischio: le celle che regrediscono sono
proprio quelle della regione spostata. Ma compatibilità non è causalità, e una diagnosi causale è
una fase separata, con la sua preregistrazione, che **non è stata autorizzata** e che non ho
iniziato.

Registro anche il difetto cosmetico già dichiarato in readiness: `policy_std` è `null` in tutte e
sei le celle, come previsto.

---

## 7. Artefatti

**Leaf**: `j16_runs/j16_closed_loop_v26c_2026-08-27_r1/` — 134 file.

| artefatto | SHA-256 |
|---|---|
| `v26c_j16_closed_loop_receipt.json` | `7cceb98f33f548e5930fb8e37846a08738c360aa801216533e59003f2b6ebbc9` |
| record GO consumato | `006a935b55009d17f63973e01c5bde059256d706065773a42096607ea3ae54d5` |
| runner | `6ac4585424cbce34957722e8fc64dc0669de14c57a0418e6aa940b1303cc2e34` |
| test | `67f00fc6bc9899c94f6780a72371c869e7250b77680598c01ede4b2977783b0b` |
| preregistrazione | `150f49b1bd2865d9224d43336d50098c2fd610b4b5d121dd1ce737213a0864aa` |
| record di readiness | `228f7eb3264908a4579ee541d73362a6b674b03b5ef497b8eb85a0c29903adde` |
| attore sotto test (J15R1) | `4d084a2a7f0012bd711f39a987dbb7af30b04e265484128c45b0a92b612ab928` |

Log additivi, fuori dalla leaf: `j16_runs/j16_execution_{stdout,stderr,exit}_2026-08-27.*`.

---

## 8. Limitazioni

- Nessuna promozione, nessuna deployability, `next_stage_authorized: false`.
- Il FAIL qualifica l'esito **su AB06 e queste tre partenze di gait**, nient'altro.
- I semi **126, 127, 128 restano sigillati**: la fase held-out G–I era condizionata a un PASS di
  A–F e **non si apre**.
- Non esiste alcuna diagnosi causale in questo report, per costruzione.

---

## 9. TODO propagati

- **Diagnosi causale della regressione J16** — nuova, aperta da questo esito. Non autorizzata.
- **Deriva nominale di J15R1 (4.44× J11)** — ora un candidato di primo piano, non una conclusione.
- **`nominal_mean_shift` dichiarato e non misurato nel runner J15R1** — aperto.
- **`policy_std` sempre `null`** — difetto cosmetico ereditato da J12, dichiarato e non corretto.
- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Semi 126, 127, 128** — riserva sigillata, non aperta.
- **Regola di governance**: ogni fase si chiude con uno user report, auditato prima della
  successiva.

---

## 10. STOP

Il record GO autorizzava **una** esecuzione ed è **consumato**. L'argv congelato non va più
invocato.

**Non ho iniziato nulla**: né la fase held-out G–I, né il critic, né PPO, né alcuna readiness
successiva, né alcuna diagnosi.

**Fermo in attesa del tuo audit.**
