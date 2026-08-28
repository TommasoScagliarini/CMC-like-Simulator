# V26C J12 — Qualifica closed-loop dell'attore J11: esecuzione e audit

**Data**: 2026-08-27
**Stadio**: `V26C_J12_CLOSED_LOOP_QUALIFICATION`
**Esecuzioni**: 1 (una sola, nessun retry, nessuna variante)
**Exit code**: 1
**Verdetto**: **FAIL** — 4/6 celle comportamentali PASS, 6/6 telemetria valida
**Esito conferito**: **NESSUNO**. Nessuna promozione, nessun critic, nessun PPO.

---

## 1. Sintesi in una riga

Il fit multistart **ha risolto il problema per cui era stato fatto** — la cella B, che con l'attore
J8 falliva con **0 cicli**, ora passa con **3 cicli**, il conteggio più alto di tutta la matrice —
**ma ha rotto due delle tre celle stocastiche** che con J8 passavano. Il netto è 5/6 → 4/6.

---

## 2. Provenance e comando

### Record di GO additivo

`v26c_j12_architect_go_2026-08-27.json` — SHA-256
`dfefccdd5a4d004d97d1b5722148e7f5e7cae794fc829599a48df89ad4a6f2e7`.
Additivo: **non** modifica l'authorization né il report di readiness, che restano evidenza
immutabile di ciò che è stato proposto e revisionato. Pinna i quattro hash che hai indicato,
la matrice A–F, single-attempt/no-retry e l'approvazione.

| file pinnato dal GO | SHA-256 | verificato |
|---|---|---|
| `v26c_j12_closed_loop_authorization.json` | `0d83943e0e057cc925cfc395945d86ff53c756909010d4781423039b595ca3bf` | OK |
| `v26c_j12_closed_loop.py` | `65d7b8619074c909753750f03aec751d38179c2226e50c87da62113ea5769fca` | OK |
| `test_v26c_j12_closed_loop.py` | `ba0a899b0b6c9835cefa27ea4fb0282f6237c0e30bc251825771eb02ca9209ce` | OK |
| `v26c_j12_prereg_closed_loop_qualification.json` | `5f0ed000b62c7c64ceaca8eca0d83d36e863619bf528a285aa7d596d63120bbe` | OK |

### Controlli pre-esecuzione (tutti superati)

- **4/4 hash pinnati dal GO** verificati.
- **22/22 pin dell'authorization** verificati (16 locali + 6 repo).
- `py_compile` su runner e test: OK.
- **Selftest: 410 check, PASS.**
- **Preflight inerte: GO**, nessun blocker, torch e ray assenti da `sys.modules` prima e dopo,
  `thresholds_invented_here: 0`, attore a 8 file con `commit_verification pass=true` e nessun
  marker.
- Nessun `j12_runs`, lock, staging o sentinella.

### Smoke test di caricamento del modulo J11 (nessun ambiente costruito)

Richiesto perché il caricatore di ray **cattura ogni eccezione** e restituisce in silenzio un
`RLModule` base senza pesi: un rollout contro una rete vuota produrrebbe evidenza su niente.

| controllo | esito |
|---|---|
| `module_state.pkl` = `19bf8a43…` | OK |
| classe = `AsymmetricActorCriticTorchRLModule` | OK |
| `get_state()` non vuoto | OK, 10 chiavi |
| dieci chiavi esatte | OK |
| pesi **bit-identici** al pickle pinnato | OK, 10/10 |
| colonne zero del primo layer | `[0, 1]` — solo il clock |
| norma minima controller 25..34 | 0.008437 (viva) |
| σ dall'attore | `0.004999999670722372`, dev **3.29e-10** |

### Comando eseguito

```
cwd:         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter: /opt/anaconda3/envs/envCMC-rllib/bin/python
argv:        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j12_closed_loop.py
             --run --authorized-stage V26C_J12_CLOSED_LOOP_QUALIFICATION
             --out <repo>/…/j12_runs/j12_closed_loop_v26c_2026-08-27_r1
```

Esattamente cwd/interpreter/argv congelati. `OUTPUT_ROOT_OVERRIDE` non impostata. stdout, stderr ed
exit code catturati per sola redirezione in `j12_execution_2026-08-27/`, **fuori dal leaf
transazionale**.

---

## 3. Esito tecnico

| | |
|---|---|
| exit code | **1** (FAIL comportamentale, come da outcome policy) |
| leaf committato | sì, con l'evidenza completa di tutte e sei le celle |
| `commit_verification.pass` | **true**, 132 file su 132 ri-risolti e ri-hashati |
| `receipt_matches_staging_bytes` | true |
| `paths_missing` / `hash_mismatches` | `[]` / `[]` |
| `TECHNICAL_INVALID` | **assente** |
| lock / staging | rilasciati, nessun residuo |
| `sim_outputs` per cella | **19 esatti** su tutte e sei |
| ricalcolo esterno indipendente | 132/132 hash riprodotti |

Il leaf contiene 134 file: i 132 hashati più il receipt (che non può hashare se stesso) e
`commit_verification.json` (scritto dopo il rename).

**stderr**: 911 byte, solo warning attesi — la deprecation di RLlib sul costruttore, due
`RuntimeWarning` di SLSQP su clipping ai bound e un fallback least-squares dello `StaticOptimizer`.
Nessun errore.

---

## 4. Esito scientifico — tabella A–F

### Telemetria FSM

| cella | modo | seed | offset | step | end_reason | cicli | HS | TO | tmo stance | tmo swing | morph | resync | hs_canc | verdetto |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **A** | deterministic | 123 | 1.9569 | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | **PASS** |
| **B** | deterministic | 123 | 1.7569 | 500 | `episode_time_limit` | **3** | 4 | 3 | 0 | 0 | 0 | 0 | 0 | **PASS** |
| **C** | deterministic | 123 | 2.1569 | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | **PASS** |
| **D** | stochastic_held | 123 | 1.9569 | 500 | `episode_time_limit` | 2 | 3 | 3 | 0 | 0 | 0 | 0 | 0 | **PASS** |
| **E** | stochastic_held | 124 | 1.9569 | 500 | `episode_time_limit` | **1** | 2 | 2 | 0 | 0 | 0 | 0 | 0 | **FAIL** |
| **F** | stochastic_held | 125 | 1.9569 | 500 | **354** · `phase_timeout:swing` | **0** | 1 | 1 | 0 | **1** | 0 | 0 | **1** | **FAIL** |

### WAIT_HS (diagnostico, non vincolante)

| cella | righe | frazione |
|---|---|---|
| A | 0 | 0.0000 |
| **B** | **20** | **0.0400** |
| C | 0 | 0.0000 |
| D | 0 | 0.0000 |
| E | 0 | 0.0000 |
| F | 0 | 0.0000 |

### Cinematica (rad)

| cella | ankle min | ankle max | ankle ROM | knee min | knee max | knee ROM |
|---|---|---|---|---|---|---|
| A | −0.11410 | 0.37862 | 0.49272 | −0.99177 | −0.16809 | 0.82368 |
| B | −0.11278 | 0.39500 | 0.50778 | −1.03342 | −0.16975 | 0.86367 |
| C | −0.11336 | 0.37925 | 0.49261 | −1.00089 | −0.17091 | 0.82998 |
| D | −0.10637 | 0.38733 | 0.49370 | −0.99567 | −0.16993 | 0.82575 |
| **E** | −0.10454 | 0.33406 | 0.43860 | **−1.21371** | −0.17420 | **1.03952** |
| **F** | −0.10114 | 0.42140 | 0.52255 | **−1.38976** | −0.17420 | **1.21557** |

Tutte e sei superano **ogni** criterio cinematico: `ankle_min ≤ −0.03`, `ankle ROM ≥ 0.30`,
`knee ROM ≥ 0.60`, ginocchio strettamente flesso (max −0.168), bounds rispettati. F arriva a
−1.390, a 0.11 rad dal bound −1.5, ma non lo viola.

### Penetrazione (contratto 20/25/28 mm)

| cella | max (m) | n > 20 mm (soft) | n ≥ 25 mm (July) | n > 28 mm (**binding**) | banda | binding |
|---|---|---|---|---|---|---|
| A | 0.022942603 | 49 | 0 | **0** | `above_soft_below_july_legacy` | PASS |
| B | 0.022947891 | 96 | 0 | **0** | `above_soft_below_july_legacy` | PASS |
| C | 0.022883755 | 86 | 0 | **0** | `above_soft_below_july_legacy` | PASS |
| D | 0.023127368 | 81 | 0 | **0** | `above_soft_below_july_legacy` | PASS |
| E | 0.020467608 | 17 | 0 | **0** | `above_soft_below_july_legacy` | PASS |
| F | 0.020968374 | 6 | 0 | **0** | `above_soft_below_july_legacy` | PASS |

`> 20 mm` è soft diagnostica; `≥ 25 mm` è la diagnostica July; **solo `> 28 mm` è hard binding**, e
28 esatti passerebbero. **Zero campioni sopra 25 mm in tutte e sei le celle**, e il massimo assoluto
della matrice è 23.1 mm — a 4.9 mm dalla barra vincolante. La penetrazione non è mai stata in
questione.

---

## 5. Gate

Verdetto per cella = gate comportamentale, con l'integrità telemetrica valutata **separatamente**.

- **A, B, C, D**: 14/14 check superati, `failed: []`.
- **E**: fallisce **solo** `valid_cycles` (1 < 2). Tutti gli altri 13 check passano, telemetria
  valida, penetrazione a posto, cinematica a posto, 500 step completati sul time limit.
- **F**: fallisce 5 check — `steps` (354 ≠ 500), `end_reason` (`phase_timeout:swing`),
  `valid_cycles` (0 < 2), `phase_timeout_swing` (1 > 0), `hs_cancelled_count` (1 > 0). Cinematica e
  penetrazione passano comunque.

**Integrità telemetrica: 6/6 valida.** Nessuna cella è INVALID: l'evidenza non si contraddice mai.
Il verdetto FAIL è un'affermazione sul **cammino**, non sull'evidenza.

**Aggregato**: PASS se e solo se 6/6 comportamentali PASS **e** 6/6 telemetria valida → **FAIL**.

Nessuna soglia inventata: `verify_scientific_equivalence` ha confrontato campo per campo con il
modulo J9R1 congelato e riportato `thresholds_invented_here: 0` prima che esistesse un ambiente.

---

## 6. Anomalie e lettura

### 6.1 La cella B è recuperata — l'obiettivo della catena è raggiunto

| cella | J9R1 (attore J8) | J12 (attore J11) |
|---|---|---|
| A | PASS, 2 cicli, 500 step | PASS, 2 cicli, 500 step |
| **B** | **FAIL, 0 cicli, 500 step** | **PASS, 3 cicli, 500 step** |
| C | PASS, 2 cicli, 500 step | PASS, 2 cicli, 500 step |
| D | PASS, 2 cicli, 500 step | PASS, 2 cicli, 500 step |
| **E** | **PASS, 2 cicli, 500 step** | **FAIL, 1 ciclo, 500 step** |
| **F** | **PASS, 2 cicli, 500 step** | **FAIL, 0 cicli, 354 step** |

In J9R1 la cella B aveva l'FSM bloccato in WAIT_HS e zero cicli. Ora B ha **3 cicli — il massimo
della matrice** — e solo il 4 % di righe WAIT_HS. Le 20 righe WAIT_HS compaiono **soltanto** in B,
esattamente le stesse 20 righe uniche che J10R1 aveva raccolto e che J11 ha usato in training.
Il meccanismo ha funzionato come progettato.

### 6.2 Ma la robustezza stocastica è regredita, e questo è il risultato che conta

Il confronto cinematico isola la causa:

| cella | knee_min J9R1 → J12 | knee ROM J9R1 → J12 |
|---|---|---|
| D (seed 123) | −0.99538 → −0.99567 | 0.83089 → 0.82575 |
| **E (seed 124)** | −0.99222 → **−1.21371** | 0.82463 → **1.03952** |
| **F (seed 125)** | −0.99609 → **−1.38976** | 0.83446 → **1.21557** |

Con l'attore J8 i tre semi stocastici producevano escursioni di ginocchio **quasi identiche**
(≈ −0.99, ROM ≈ 0.83). Con J11 il seed 123 resta invariato, ma i seed 124 e 125 divergono in
flessioni molto più profonde. È **amplificazione di varianza**: a parità di rumore (σ = 0.005,
hold 1, stessa sequenza `HeldStandardNormal` per gli stessi semi), l'attore J11 risponde molto più
fortemente su due semi su tre. In F la deriva rompe la fase di swing e l'episodio termina a 354
step.

Il clipping è **zero** in tutte le celle: le azioni non saturano. La divergenza è di dinamica, non
di saturazione.

### 6.3 Il collegamento con i diagnostici di J11, che avevo segnalato

Questo esito è coerente con ciò che il report di esecuzione J11 aveva registrato come diagnostici
**non vincolanti**:

- J11 era **0.59 % peggiore di J8** sul blocco recovery;
- l'ancora nominale aveva **perso l'esattezza** (da bit-esatta a 5.38e-3 RMSE);
- `nominal_mean_shift` era **0.0264**, contro 0.0221 di J8 e una soglia offline July di 0.005.

Un attore che si è allontanato di più dall'ancora è anche quello che risponde più fortemente al
rumore. I tre diagnostici puntavano nella stessa direzione e ora hanno un riscontro closed-loop.
Nessuno di essi era vincolante, per tua decisione esplicita; il punto non è che il gate offline
avrebbe dovuto fermarli, ma che **erano segnali leggibili in anticipo**.

### 6.4 Nessuna anomalia tecnica

Exit 1 è l'esito previsto dalla outcome policy per un FAIL comportamentale. Nessun errore in
stderr. Commit verificato. Lock e staging rilasciati. Sei celle su sei hanno prodotto esattamente
19 `sim_outputs`. La matrice non si è fermata al primo FAIL: E ed F sono state eseguite comunque,
come da contratto.

---

## 7. Inventario e hash

### Leaf committato

`Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j12_runs/j12_closed_loop_v26c_2026-08-27_r1/`

| file | SHA-256 |
|---|---|
| `v26c_j12_closed_loop_receipt.json` | `2b3e20362bb54ea545db3495d2a468d33138555cccf8bd20d3e5ecc3d75caf11` |
| `commit_verification.json` | `a5ef34e758c50437d28dfbf4413e5a18a2e3ad7ba9d6ec4fb164431b17fcb175` |

Più, per ognuna delle sei celle: `j12_cell_<ID>_trace.json`, `_kinematics.npz`, `_penetration.npz`
e `_sim_outputs/` con 19 file. 132 hash registrati nel receipt per path leaf-relative, tutti
ri-verificati.

### Cattura dell'esecuzione (additiva, fuori dal leaf)

`…/v26c_july_replica_2026-08-26/j12_execution_2026-08-27/`

| file | byte | SHA-256 |
|---|---|---|
| `j12_run.out` | 136 832 | `e53c3ff53edb733957af5c4a5f1616fe915477532119a53c1ba759b2a12f2486` |
| `j12_run.err` | 911 | `836df73429f4015635f2043e103cbfe9adb0881ae5bb72acd62e13d0714e3ca1` |
| `j12_run.exit` (`exit=1`) | 7 | `cf205dbb8cea84897b488abcc281bf96698d5e94b1096b16657b4caba9082a22` |

### Record di GO

`v26c_j12_architect_go_2026-08-27.json` — `dfefccdd5a4d004d97d1b5722148e7f5e7cae794fc829599a48df89ad4a6f2e7`

**Nessun file J0–J11, nessun artefatto July, nessun report esistente e nessuna configurazione di
produzione è stato modificato.** L'authorization e la readiness J12 sono intatte. FSM, corridoio
morfologico, reward, SEA e plugin C++ non toccati. `git status` mostra solo i tre file già dirty a
inizio sessione.

---

## 8. Conclusione

**FAIL.** L'attore J11 **non è qualificato in closed loop**.

Il risultato è però informativo, non nullo, e va letto in due parti:

1. **Il fit multistart ha funzionato per ciò per cui era stato fatto.** La cella B — lo start
   −0.20 s che aveva bloccato l'intera catena — passa ora con il conteggio di cicli più alto della
   matrice. Le tre celle deterministiche sono tutte PASS. Questo non era vero prima e ora lo è.
2. **Il costo è stato pagato sulla robustezza stocastica.** Due semi su tre che passavano con J8
   ora falliscono, e la causa misurata è un'amplificazione della risposta al rumore, visibile come
   flessioni di ginocchio molto più profonde a parità di sequenza di rumore.

Il netto sulla matrice è **5/6 → 4/6**: J12 ha *meno* celle passanti di J9R1. Entrambi gli
aggregati sono FAIL, ma per ragioni opposte.

---

## 9. Conseguenza per la fase successiva

**Il critic warm-up NON è avviato e non è autorizzato.** Una qualifica closed-loop FAIL è
precisamente la condizione in cui non lo si avvia.

Le opzioni restano decisioni tue, non mie. Le espongo senza sceglierne una:

- **Nessuna azione**: J11 resta un artefatto committato e non promosso, e la catena si ferma qui.
- **Rivedere il compromesso del fit**: il peso 8 sulle celle multistart è replicato da July e
  **non ha razionale documentato** (segnalato già nella readiness J11). Un peso diverso, o un
  `anchor_weight` maggiore, sposterebbe il compromesso — ma sarebbe uno stadio di fit nuovo, con
  la sua preregistrazione.
- **Considerare J8 e J11 come fratelli e scegliere**: J8 passa E ed F ma fallisce B; J11 passa B ma
  fallisce E ed F. Nessuno dei due passa tutta la matrice.
- **Rimettere in discussione σ = 0.005 come punto operativo**: è ereditato dalla testa logstd
  congelata, mai selezionato. Ma cambiarlo cambierebbe il contratto, non l'attore.

---

## 10. TODO propagati

- **LOTO** — non integrato. TODO futuro.
- **LOCO** — non integrato. TODO futuro.
- **B1R1** — non integrato. TODO futuro.
- **B1R2** — non integrato. TODO futuro.
- **Epic generalizzazione multi-modello** — APERTO. Questa fase copre ancora AB06 e tre start dello
  stesso trial; non dimostra alcuna generalizzazione.
- **Critic warm-up** — NON avviato, NON autorizzato. Quando e se lo sarà: dovrà costruire un modulo
  fresco `inference_only=False` e trapiantare l'attore, perché `class_and_ctor_args.pkl` fissa
  `inference_only=True`; servono tutte e dieci le chiavi attore, alias inclusi.
- **PPO** — non autorizzato.
- **Robustezza stocastica di J11 su seed 124 e 125** — **NUOVO, aperto da questa fase.** È il
  blocco all'avanzamento. Diagnosi misurata: amplificazione di varianza, non saturazione.
- **Fattore repeat 8 senza razionale documentato** — ora ha rilevanza operativa: è uno dei pochi
  parametri del compromesso che ha prodotto questo esito.
- **`nominal_mean_shift` 0.0264 di J11** — i tre diagnostici non vincolanti di J11 hanno trovato
  riscontro closed-loop. Se in futuro si vorrà una barriera anticipata, questa è la metrica
  candidata; oggi non è un gate e non l'ho reso tale.
- **J8 e J11 falliscono celle complementari** — nessuno dei due qualifica l'intera matrice.
- **WAIT_HS senza soglia** — resta diagnostico. In questa esecuzione è comparso solo nella cella B
  (20 righe, 4 %) e non ha segnalato alcuna patologia.

---

## 11. Cosa questo FAIL **non** significa

- Non significa che l'evidenza sia difettosa: la telemetria è valida 6/6 e il commit è verificato
  132/132.
- Non significa che la penetrazione o la cinematica siano un problema: entrambe passano ovunque,
  con margine.
- Non è un'affermazione su J8: quel confronto è diagnostico, non un gate, e J8 fallisce comunque la
  cella B.

**Fermo in attesa della tua revisione indipendente.**
