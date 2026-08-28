# V26C J11 — Multistart offline fit: esecuzione e audit

**Data**: 2026-08-27
**Stadio**: `V26C_J11_MULTISTART_FIT`
**Esecuzioni**: 1 (una sola, nessun retry, nessuna variante)
**Exit code**: 0 · **stderr: 0 byte**
**Verdetto**: **PASS** — 16/16 gate vincolanti superati
**Esito conferito**: **NESSUNO**. Questa fase produce un attore fittato e non autorizza qualifica
closed-loop, critic, PPO, promozione o deployment.

---

## 1. Problema

J9R1 aveva FALLITO in closed-loop sulla cella B (start `-0.20 s`): 0 cicli validi. La causa
misurata era un gap di supporto: in J7 la feature `phase_fsm_wait_hs` è **costante a zero su tutte
le 16713 righe**, quindi la policy non aveva mai visto lo stato che quello start attraversa.

J10R1 ha raccolto le label del teacher prescritto ai due start non nominali. J11 è il fit che le
consuma: **fresh dal parent J2**, su un aggregato di 24713 righe, con la metodologia July applicata
ai dati August.

---

## 2. Strategia e lineage August

**Lineage operativa**: August V26 imitation → **J2 35D** → J11. July è metodologia ed evidenza
soltanto: nessun checkpoint, dataset o label July è parent operativo.

**Parent**: `j2_runs/j2_base_v26c_2026-08-26_r1/rl_module`, `module_state`
`0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130`.
**Non** J8, **non** J4, **non** un checkpoint July. J8 ha fittato lo stesso parent J2 sulle sole
righe J7: i due fit sono **fratelli**, non una catena.

Distinti e mai confusi nel receipt:
- **parent dei pesi**: `0f182ea9…` (J2)
- **antenato del contratto di osservazione**: `0ba56eb7…`, l'attore imitativo V26 a 39 colonne sotto
  cui furono raccolte le celle J10R1 — mai caricato da questo stadio.

Un solo attore 35D. Nessun 25D, nessun widening, nessuna feature controlaterale.

### Aggregato: 24713 righe, semantica `np.tile`

| blocco | slice | righe | uniche misurate | repeat | fonte |
|---|---|---|---|---|---|
| `j7_nominal` | `[0:16000]` | 16000 | **500** | 32 | J7, stati dal rollout J3, label self-distilled dalle medie J2 |
| `j7_recovery` | `[16000:16713]` | 713 | **711** | 1 | J7, label teacher J1, seed 123/124/125 |
| `cell_B` | `[16713:20713]` | 4000 | **500** | 8 | J10R1 cella B, `-0.20 s`, teacher prescritto |
| `cell_C` | `[20713:24713]` | 4000 | **500** | 8 | J10R1 cella C, `+0.20 s`, teacher prescritto |

Ordine: J7 intero, poi B×8, poi C×8 — confermato contro l'evidenza primaria July, che costruiva
`[nominal×32] ++ [recovery×2] ++ [B×8] ++ [C×8]` con `np.tile`.

**Il gap J7 è chiuso**: l'aggregato contiene **160 righe** con `phase_fsm_wait_hs == 1` (20 uniche
dalla cella B, moltiplicate ×8 dal tiling), contro **zero su 16713** in J7.

---

## 3. Comando eseguito

```
cwd:         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter: /opt/anaconda3/envs/envCMC-rllib/bin/python
argv:        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j11_multistart_fit.py
             --fit --authorized-stage V26C_J11_MULTISTART_FIT
             --out <repo>/.../j11_runs/j11_multistart_fit_v26c_2026-08-27_r1
             --progress
```

Esattamente cwd/interpreter/argv congelati nell'authorization, `--progress` incluso come variazione
ammessa. `OUTPUT_ROOT_OVERRIDE` non impostata. stdout/stderr/exit catturati per sola redirezione in
`j11_execution_2026-08-27/`, senza alterare argv.

### Recheck pre-esecuzione (tutti superati)

- **18 hash frozen** verificati: 15 input della validation dir + 3 repo.
- Parent `0f182ea9…` invariato.
- J10R1 ancora valido: `commit_verification.pass = true`, nessun `TECHNICAL_INVALID`,
  `verdict = PASS`.
- Selftest **195/195 PASS**; preflight **GO**, torch assente da `sys.modules` prima e dopo.
- Nessun leaf, lock, staging o sentinella.

---

## 4. Esito

### Metriche prima/dopo sui quattro sottoinsiemi vincolanti

| sottoinsieme | righe | RMSE prima | RMSE dopo | dopo/prima | esito |
|---|---|---|---|---|---|
| `aggregate` | 24713 | 0.030865186 | **0.015608757** | 0.506 | PASS |
| `j7_recovery_original` | 713 | 0.088360829 | **0.084147858** | 0.952 | PASS |
| `cell_B_unique` | 500 | 0.063906785 | **0.008264150** | **0.129** | PASS |
| `cell_C_unique` | 500 | 0.020248002 | **0.007684291** | 0.379 | PASS |

Ho **ricalcolato indipendentemente** tutte e otto le cifre dal forward numpy sui pesi committati:
riproducono il receipt entro **3.4e-9** (rumore di ordine di accumulazione torch-vs-numpy). L'audit C
ha ripetuto l'intero calcolo anche in **float64 pieno**: tutte e quattro le diminuzioni strette
sopravvivono invariate, quindi la direzione del gate non è un artefatto della precisione float32.
Il divario più stretto è quello del recovery, 4.2e-3 — sei ordini di grandezza sopra il rumore.

### Norme di colonna controller (25..34) — tutte > 0

| feature | norma L2 | parent J2 |
|---|---|---|
| `pros_knee_angle_previous_endpoint` | 0.450841 | 0.0 |
| `pros_knee_angle_served_ref` | 0.172854 | 0.0 |
| `pros_knee_angle_served_ref_vel` | 0.044509 | 0.0 |
| `pros_knee_angle_served_ref_accel` | 0.008437 | 0.0 |
| `pros_knee_angle_sea_u` | 0.543140 | 0.0 |
| `pros_ankle_angle_previous_endpoint` | **0.687091** | 0.0 |
| `pros_ankle_angle_served_ref` | 0.341533 | 0.0 |
| `pros_ankle_angle_served_ref_vel` | 0.071310 | 0.0 |
| `pros_ankle_angle_served_ref_accel` | 0.010061 | 0.0 |
| `pros_ankle_angle_sea_u` | 0.278214 | 0.0 |

Il parent J2 è la base **doppiamente mascherata**: clock *e* controller entrambi esattamente zero.
Il fit lascia il clock morto e porta il blocco controller in vita.

### Invarianti clock / logstd / critic

| invariante | esito | misura |
|---|---|---|
| clock colonne 0,1 bit-zero in `pi.0.0.weight` | **OK** | max abs = 0.0 esatto, 0 entry non nulle |
| clock bit-zero anche in `pi_encoder.0.weight` | **OK** | idem |
| alias bit-identici ai tensori diretti (4 coppie) | **OK** | `array_equal` True, max diff 0.0 |
| `pi.1.weight[2:]` bit-identica a J2 | **OK** | max abs diff **0.0** |
| `pi.1.bias[2:]` bit-identica a J2 | **OK** | max abs diff **0.0**, valore `-5.2983174324035645` → σ = 0.005 |
| `pi.1.weight[2:]` tutta zero (σ indipendente dallo stato) | **OK** | 0 entry non nulle |
| nessuna chiave critic / `vf*` | **OK** | 10 chiavi, tutte `pi*` |
| chiavi/shape/dtype uguali a J2 | **OK** | 10 chiavi, tutte float32, 151044 parametri |

### Training

400 epoche eseguite, best all'epoca **397**, `best_validation_mse` = 2.797689e-4, nessun early stop.
La curva di validation è in **plateau da ~epoca 200** (2.975e-4 → 2.798e-4 in 200 epoche): il best
tardivo è rumore dentro il plateau, non un fit interrotto in salita. La patience 60 non è mai
scattata perché il plateau rumoroso produceva nuovi minimi marginali entro 60 epoche l'uno
dall'altro.

### Determinismo

Ordine esecutivo registrato: `import torch` → `torch.manual_seed(123)` → `np.random.seed(123)` →
init deterministico dal parent → **un solo** `np.default_rng(123)` → optimizer.
`use_deterministic_algorithms` e `set_num_threads` **non forzati** (osservati: `False`, 5), come in
July. Split **4943 / 19770**, verificato non assunto.

Precondizioni vincolanti superate prima che l'optimizer esistesse: colonne controller del parent
bit-zero; equivalenza raw-vs-scaled con `max_abs_diff` **esattamente 0.0** e bit-identità, misurata
in **due kernel** (numpy float32 e torch).

---

## 5. Verifiche indipendenti

Tre subagenti read-only in parallelo, lanciati **dopo** il fit, senza alcun potere di eseguire fit,
rollout, training o di modificare artefatti.

**Audit A — integrità del leaf: 6/6 PASS.** Inventario esatto (8 file, 0 extra, 0 mancanti);
`TECHNICAL_INVALID` assente confermato con sweep ricorsivo case-insensitive sull'intero albero;
nessun lock/staging residuo; i 6 path del receipt ri-risolti e ri-hashati indipendentemente, 6/6
riprodotti; NPZ aggregato 24713×35 / 24713×2 float32, tutto finito, clock **bit-zero** su tutte le
24713 righe; tiling verificato a livello di byte (tutte e 32 le tile nominali e tutte e 8 le tile
di B e C bit-identiche alla prima); J7, B e C bit-identiche dentro l'aggregato; `history.json` 400
epoche, best 397 riprodotto includendo la rappresentazione float completa.

**Audit B — invarianti del modulo: 9/9 PASS.** Tutte le voci della tabella invarianti sopra,
ricalcolate; `actor_digest` verificato chiamando la **funzione reale** `warm_start.actor_state_digest`
e coincidente con quello nel manifest; `source_actor_digest` coincidente con quello del parent; le
dieci norme dichiarate nel manifest coincidono con la misura float64 entro 2.1e-8.

**Audit C — ricalcolo del gate: PASS su tutti e sette i punti.** Gli otto valori RMSE ricalcolati
da zero riproducono il receipt entro **3.4e-9** (rumore di ordine di accumulazione float32
torch-vs-numpy, cinque o sei ordini di grandezza sotto il divario più piccolo che il gate deve
giudicare). **Controllo di robustezza aggiunto dall'audit: rifacendo ogni forward in float64 pieno,
tutte e quattro le diminuzioni strette sopravvivono invariate** — la direzione del gate non è un
artefatto di float32. Le dieci norme controller coincidono **bit-esattamente** (Δ = 0.0 su tutte).
Lo split riprodotto fino agli hash di membership: `digest` `f087065c…`, val `7043a49e…`, train
`3e1f5515…`, tutti e tre coincidenti. `clipping_out_of_bounds_rows = 0` confermato, con
max |mean| dopo = 0.8076 (prima 0.8187): nulla vicino al bound di clip.

### Il manifest non ripete il difetto di J8

Confermato dagli audit, misurato:

| | J8 (copia byte-per-byte) | J11 (rigenerato) |
|---|---|---|
| `module_state_sha256` | `0f182ea9…` = **il parent**, mentre il modulo J8 è `9c5b1571…` | `19bf8a43…` = **il modulo accanto**, verificato |
| controller | dichiara `controller_state_mask.active: true`, ma le colonne sono vive (max abs 0.0129) | `controller_contract.masked: false`, LIVE, con le norme |
| `actor_digest` | **assente** → nulla di applicabile | presente e verificato dalla funzione reale |
| `actor_label` / `status` | del parent, precedono J8 | propri di J11 |

`class_and_ctor_args.pkl` e `metadata.json` restano **byte-identici** a J2, come previsto.

---

## 6. Osservazioni oneste

**1. Il blocco recovery migliora poco, e J11 fa leggermente peggio di J8 su quel blocco.**

| | J8 (solo J7) | J11 (J7+B×8+C×8) |
|---|---|---|
| recovery 713 prima | 0.088360829 | 0.088360829 |
| recovery 713 dopo | **0.083651678** | **0.084147858** |
| nominal dopo | 0.005032696 | 0.005378645 |
| nominal shift max abs | 0.022095606 | 0.026410310 |
| norme controller (min–max) | 0.00085 – 0.13059 | 0.00844 – 0.68709 |

Aggiungere i dati multistart ha **peggiorato dello 0.59%** l'RMSE sul blocco recovery rispetto a J8,
in cambio di un guadagno grande su B (7.7×, −87%) e su C (2.6×, −62%). Entrambi i fit superano il
proprio gate vincolante (`dopo < prima`), ma il trade-off esiste e va detto.

**Il miglioramento sul recovery è marginale in senso assoluto: 4.8%** (0.088361 → 0.084148). Il gate
è una disuguaglianza stretta senza soglia, quindi passa onestamente — il divario è 4.2e-3, sei ordini
di grandezza sopra il rumore numerico. Ma chi leggesse questo receipt come "il recovery è migliorato"
sopravvaluterebbe un movimento del 4.8%. È il più debole dei quattro sottoinsiemi.

**2. Il blocco nominale è passato da bit-esatto a 5.38e-03.** L'audit ha precisato un punto che il
receipt sfuma: il "prima" del blocco nominale non è 6.85e-08, è **esattamente 0.0** — il forward
float32 del parent è `np.array_equal` alle label memorizzate, perché quelle label **sono** le medie
del parent. Il 6.85e-08 nel receipt è rumore torch-vs-numpy del forward, non un residuo reale.

Quindi la formulazione corretta non è "degradato di N volte" ma: **l'ancora nominale ha perso la
propria esattezza**. È un diagnostico esplicitamente non vincolante, come da tua decisione, e nessuna
direzione era preregistrata per esso — ma è il prezzo pagato per il guadagno sulle celle, e va detto
come perdita di esattezza, non come lieve peggioramento di una baseline già non nulla.

**3. `nominal_mean_shift` = 0.0264** — definito come `max|dopo − prima|` sulle 16000 righe
nominali, **non** come RMS. La soglia offline di July era **0.005**, e il run July
selezionato misurò 0.0316 — **fallendo il proprio gate offline**, per poi essere selezionato su base
closed-loop. J11 è nello stesso ordine di grandezza (0.0264), e J8 pure (0.0221). Il gate di questa
fase **non vincola** su questa metrica, per tua decisione esplicita. Lo segnalo perché, applicando la
soglia July, questo fit non l'avrebbe superata — esattamente come non la superò July.

**4. Il file del modulo è 604 834 B contro i 304 599 B del parent.** Spiegato: J2 memoizza gli alias
come lo stesso oggetto Python, J11 li scrive come buffer separati bit-identici. **Identico byte per
byte in dimensione a J8**, quindi è la convenzione già stabilita, non una novità.

**5. Il blocco recovery ha 711 righe distinte su 713.** Due righe di osservazione sono duplicate.
Ereditato da J7, già dichiarato dal receipt (`distinct_rows_measured: 711`), non un difetto
introdotto qui.

**6. Righe ripetute in entrambe le partizioni.** Ogni blocco è tiled, quindi la stessa riga unica
compare in training **e** in validation. Ereditato da July e da J7, registrato e **non corretto**:
la validation MSE è un diagnostico di training, **mai** una stima di generalizzazione.

**7. Nessuna anomalia tecnica.** stderr completamente vuoto, exit 0, lock e staging rilasciati.

---

## 7. Inventario e hash

### Leaf committato

`Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j11_runs/j11_multistart_fit_v26c_2026-08-27_r1/`

| file | byte | SHA-256 |
|---|---|---|
| `v26c_j11_multistart_fit_receipt.json` | 24 461 | `39228c5cf00a753f1d57f07d4794ac2996401e1b40587cf1ec1e5f5e2b0ae65f` |
| `commit_verification.json` | 1 325 | `1d24eed05dd04c187c014181c89a0611a648e5601cacff5e1ae0221adcfb8643` |
| `v26c_j11_aggregate_dataset.npz` | 2 072 185 | `a936c580c4db19383255d3d5f7346560e0fd99dc4b139ac202858e82cca13f42` |
| `history.json` | 45 806 | `bbfcc0c64e8d8c9019d8bb8a7bfd485228c707b7b3229649cce1d946dcc96d14` |
| `rl_module/module_state.pkl` | 604 834 | `19bf8a43804774c06c24db30626138856bb07acf7002419606d8c1bb887f6b73` |
| `rl_module/actor_feature_manifest.json` | 6 803 | `9cc0276c22c9cd41d75a12ad1e09e6ce1931cf8a27eafc3ab0741cd5937f6b28` |
| `rl_module/class_and_ctor_args.pkl` | 2 236 | `897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02` |
| `rl_module/metadata.json` | 197 | `3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12` |

Digest dell'attore: `actor_digest` = `f3c8fcb1b2ed28ed5fcb96440af8b5b9805e030125667fae3759a5c1447f1633`,
`source_actor_digest` (J2) = `59d54240ac628dcbd1d0dbf34328145afa08a7028047a5923298782b79bf5188`.

Il receipt hasha 6 file su 8. I due scoperti sono il receipt stesso (non può hashare se stesso) e
`commit_verification.json` (scritto dopo il rename). Il receipt è comunque pinnato
crittograficamente: `commit_verification.receipt_sha256` = `39228c5c…`, ricalcolato e coincidente.

### Cattura dell'esecuzione

`…/v26c_july_replica_2026-08-26/j11_execution_2026-08-27/`

| file | byte | SHA-256 |
|---|---|---|
| `j11_run.out` | 2 724 | `49b16e9b073dadb41940c0722261dbad9fde4773e666d9d319dd6c726ed1468f` |
| `j11_run.err` | **0** | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `j11_run.exit` (`exit=0`) | 7 | `19eaf43821a7660ec323a87c8457bf74823beb296c39f5e01aa8a683aa50f061` |

### Bundle di stadio, invariato dall'esecuzione

`v26c_j11_prereg_multistart_fit.json` `49e352466cc82ea9…` ·
`v26c_j11_multistart_fit.py` `2b4ac9f496f7a412…` ·
`test_v26c_j11_multistart_fit.py` `180607e79be29b42…` ·
`v26c_j11_multistart_fit_authorization.json` `faeb5eeb6db44080…`

Parent e input verificati invariati dopo il fit. **Nessun file J0–J10R1, nessun artefatto July,
nessun report precedente e nessuna configurazione di produzione è stato modificato.** FSM, corridoio
morfologico, reward, SEA e plugin C++ non toccati. `git status` mostra solo i tre file già dirty a
inizio sessione: il worktree dell'utente è preservato.

---

## 8. Test e verifiche eseguite

- 18 hash frozen verificati prima dell'esecuzione: tutti OK.
- Selftest sintetico: **195 check, PASS**.
- Preflight inerte: **GO**, nessun blocker, torch assente da `sys.modules`.
- Assenza di leaf/lock/staging/sentinella confermata prima della run.
- Esecuzione singola, exit 0, stderr vuoto, lock e staging rilasciati.
- Verifica post-commit interna: 6 file, `pass: true`, `receipt_matches_staging_bytes: true`.
- **Ricalcolo esterno indipendente**: tutti gli hash del leaf; gli 8 valori RMSE dal forward numpy
  sui pesi committati (entro 3.4e-9 dal receipt); il digest attore con la funzione reale di
  `warm_start`; il tiling a livello di byte; l'identità bit-a-bit delle fonti nell'aggregato.
- Tre audit read-only paralleli: A 6/6 PASS, B 9/9 PASS, C ricalcolo del gate.
- 16/16 gate vincolanti superati, `failed: []`.

---

## 9. TODO propagati

- **LOTO** — non integrato. TODO futuro, non J11.
- **LOCO** — non integrato. TODO futuro, non J11.
- **B1R1** — non integrato. TODO futuro, non J11.
- **B1R2** — non integrato. TODO futuro, non J11.
- **Epic generalizzazione multi-modello** — APERTO. Questa fase copre ancora AB06 e tre start dello
  stesso trial; non dimostra alcuna generalizzazione ad altri soggetti, trial o velocità.
- **Qualifica closed-loop** — NON parte di questa fase e **non autorizzata** da essa. Un PASS qui è
  un risultato **offline**. Il manifest lo dichiara: `closed_loop_qualification: PENDING`.
- **Critic warm-up** — NON parte di questa fase. Dovrà costruire un modulo fresco
  `inference_only=False` e trapiantare l'attore: `class_and_ctor_args.pkl` fissa
  `inference_only=True`, quindi `from_checkpoint` restituirà sempre un modulo actor-only.
- **J9R1 FAIL sulla cella B** — resta aperto. Questo fit fornisce un attore che ha visto lo stato
  mancante; **non dimostra** che il closed-loop passerà.
- **Trade-off recovery vs multistart** — J11 è dello 0.59% peggiore di J8 sul blocco recovery.
  Se l'architetto ritiene il recovery prioritario, la scelta fra i due fratelli è una sua decisione.
- **`nominal_mean_shift` 0.0264 contro la soglia July di 0.005** — segnalato sopra; il gate di
  questa fase non vincola su questa metrica per decisione esplicita.
- **Fattore repeat 8 senza razionale documentato** — replicato come valore osservato di July.
- **Copertura WAIT_HS asimmetrica** — le 160 righe dell'aggregato vengono tutte dalla cella B
  (20 uniche ×8). Nessuna soglia è stata inventata su questo numero.
- **Discrepanze documentali July** (label nominali `raw_policy_action` vs `policy_action_mean`, il
  gate July `ok:false`, il drift dei numeri di riga di `target_domain_imitation.py`) — segnalate
  nella readiness, non corrette: J0–J10R1 e i report esistenti sono immutabili.
- **Inventario del leaf definitivo** — lo stadio successivo pinnerà questi 8 file esattamente;
  l'hash del manifest non può essere corretto dopo senza invalidare i pin che lo referenziano.

---

## 10. Cosa questo PASS **non** conferisce

- Non autorizza la qualifica closed-loop di questo attore.
- Non autorizza un critic warm-up né PPO.
- Non promuove alcun attore. `deployable: false`, `promotion: NONE`.
- Non autorizza lo stadio successivo.
- Non modifica FSM, corridoio morfologico, reward, SEA o plugin C++.

**Fermo in attesa della revisione dell'architetto.**
