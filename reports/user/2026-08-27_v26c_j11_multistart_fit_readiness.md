# V26C J11 — Multistart offline fit: readiness

**Data**: 2026-08-27
**Stadio proposto**: `V26C_J11_MULTISTART_FIT`
**Stato**: bundle pronto per la revisione dell'architetto. **Nessun fit eseguito, nessun leaf creato.**
**Autorizzazione**: `execution_permitted_now: false`. Il comando è congelato ma **non** autorizzato.

---

## 1. Problema

J9R1 aveva FALLITO in closed-loop sulla cella B (start `-0.20 s`): 0 cicli validi. J10R1 ha
raccolto le label del teacher prescritto ai due start non nominali, colmando il gap di supporto
misurato in J7 (`phase_fsm_wait_hs` costante a zero su 16713 righe). Restano da usare.

J11 è il fit che le consuma: **fresh da J2**, su un aggregato di 24713 righe, con la metodologia
July applicata ai dati August. Non continua J8 — J8 ha fittato lo stesso parent J2 sulle sole
righe J7, quindi i due fit sono **fratelli, non una catena**.

---

## 2. Strategia e decisioni verificate

### Lineage
August V26 imitation → J2 35D → J11. July è **metodologia ed evidenza soltanto**: nessun
checkpoint, dataset, label o artefatto July è parent operativo.

Parent: `j2_runs/j2_base_v26c_2026-08-26_r1/rl_module`, `module_state` `0f182ea9…` — verificato.
Un solo attore 35D; nessun 25D, nessun widening, nessuna feature controlaterale.

### Aggregato: 24713 righe

| blocco | slice | righe | uniche | repeat | fonte |
|---|---|---|---|---|---|
| `j7_nominal` | `[0:16000]` | 16000 | 500 | 32 | J7, stati dal rollout J3, label self-distilled dalle medie J2 |
| `j7_recovery` | `[16000:16713]` | 713 | 713 | 1 | J7, label teacher J1, seed 123/124/125 |
| `cell_B` | `[16713:20713]` | 4000 | 500 | 8 | J10R1 cella B, `-0.20 s`, teacher prescritto |
| `cell_C` | `[20713:24713]` | 4000 | 500 | 8 | J10R1 cella C, `+0.20 s`, teacher prescritto |

Ordine: **J7 intero, poi B×8, poi C×8** — come da tua decisione.

---

## 3. Findings dei tre audit paralleli

### A — Evidenza primaria July

**L'ordine che hai indicato è CONFERMATO dall'evidenza primaria.** Nessuna contraddizione, quindi
non mi sono fermato.

- Layout July, da `markov_dataset_report.json` e `target_domain_markov_adaptation.py:174-187,362-390`:
  `[nominal 500×32 = 16000] ++ [recovery 356×2 = 712] ++ [B 500×8 = 4000] ++ [C 500×8 = 4000]`,
  aggregato **24712**. Nessun interleave, nessun sort.
- **`np.tile`, non `np.repeat`** — l'intero blocco ripetuto, non ogni riga ripetuta di seguito.
  Confermato sia nel codice July sia, indipendentemente, misurato da me sul dataset J7 committato:
  `nom[0:500] == nom[500:1000] == … == nom[15500:16000]`, tutte e 32 le copie identiche, 500 righe
  uniche. Il test dimostra che `np.repeat` darebbe **la stessa shape e contenuto diverso**, quindi
  la distinzione non è cosmetica.
- **Repeat 8 per entrambi gli start** — confermato.
- **Split July**: un solo `np.default_rng(123)`; `validation_count = max(1, int(round(N×0.2)))`;
  validation **ordinata**, training con **ordine della permutazione preservato**; lo **stesso**
  generator produce poi ogni shuffle d'epoca. Per July: 4942/19770 su 24712.
- **Scaling**: fit su input scalati, poi la scala è **riassorbita nel primo layer**
  (`first_layer_weight[:, idx].div_(scale)`), così il runtime consuma osservazioni **raw**.
  Verificato indipendentemente dall'audit: il forward del checkpoint July salvato su osservazioni
  raw riproduce l'RMSE del `run_summary.json` a 10 cifre.
- **Clock**: con clock disabilitato i rollout registrano **raw (0,1)** — confermato su tutte le
  24712 righe July. I **dati** non furono azzerati; furono azzerati i **pesi**, prima del fit,
  dopo ogni optimizer step e dopo il caricamento del best state.
- **Early stopping**: `val < best - 1e-9`, patience 60, epochs 400, batch 128, lr 5e-05,
  `clip_weight` 1.0, `logstd_weight` 0.0, `anchor_weight` 0.01.

**Una differenza, già preregistrata a monte**: il blocco recovery August è **713 righe uniche prese
una volta** (da J7), non 356 uniche prese due volte. Da qui 24713 invece di 24712. È ereditata dallo
stadio J7 già committato, non è una scelta fatta qui.

**Un fatto che va detto**: il fattore 8 **non ha alcuna motivazione documentata**. Sopravvive solo
come dato (`recovery_dataset_repeat: 8` nel report e il token `alt8` nel nome della directory); il
default CLI era 1, quindi 8 fu passato esplicitamente. Replico un valore **osservato**, e la
preregistrazione dice esattamente questo — non rivendica una giustificazione che non esiste.

**Discrepanze documentali segnalate dall'audit** (nessuna cambia J11):
1. Il report J0 afferma che le label nominali July sono `policy_action_mean`; sono in realtà
   `raw_policy_action` (il campo `policy_action_mean` è `null` su ogni riga della trace).
   Semanticamente innocuo — rollout deterministico — ma è un disaccordo di fatto.
2. Il run July selezionato registra `"ok": false` sul gate offline (`nominal_mean_shift` 0.0316 >
   0.005), mentre i report del 13/07 lo presentano come checkpoint selezionato senza menzionarlo.
   Riconciliato solo dopo, nell'audit J0, come gate a due livelli.
3. `target_domain_imitation.py` su disco **non è** il codice che ha prodotto il run July. Chi cita i
   numeri di riga attuali per la semantica July cita l'artefatto sbagliato.

### B — Riuso di J8

- Helper puri e stage-neutri riusabili così com'erano: `_sha_file`, `_sha_array`, `_sha_obj`,
  `_numpy_forward`, `_rmse`, `build_split`, `controller_indices`, `scale_vector`,
  `parent_preconditions`, `july_scales_from_source`. J11 **importa il modulo J8 frozen come
  libreria** e lo pinna per hash; importarlo non fa I/O e non tira torch.
- **Split derivato, non assunto**: per N=24713 la formula July/J8 dà **4943 / 19770**. Un `int()`
  troncato darebbe 4942. L'ho verificato indipendentemente prima che l'audit rientrasse, e il
  runner **rifiuta di continuare** se lo split non produce i conteggi dichiarati.
- Determinismo: `torch.manual_seed` → `np.random.seed` → init deterministico → **un solo**
  `default_rng` → optimizer. `use_deterministic_algorithms` e `set_num_threads` **non sono
  chiamati**, come in July.
- **J8 non ha alcuna verifica post-commit**: registra l'hash del receipt dopo il rename e non lo
  confronta con nulla. J11 adotta il protocollo J10R1.
- **Il test J8 è nato spento**: asserisce incondizionatamente che il proprio preflight è GO e che
  la propria run directory non esiste. Entrambe sono morte nell'istante del commit. **Nel suite J11
  quelle asserzioni sono condizionali** sull'esistenza del leaf, e dopo il commit asserisce lo
  stato post-commit. Non nascerà spento.
- Trappola: `class_and_ctor_args.pkl` richiede `asymmetric_rl_module` e torch per essere
  spacchettato. Va **solo hashato e copiato**, mai caricato. Un check AST lo impone nel preflight.

### C — Lineage, sidecar, manifest

- Parent J2 verificato: 10 chiavi, 35 in / 4 out, tutte float32. **Clock 0,1 esattamente zero**
  *e* **controller 25..34 esattamente zero**: J2 è la base **doppiamente mascherata**. Il fit lascia
  il clock morto e porta il blocco controller in vita.
- `pi.1.weight[2:]` esattamente zero, `pi.1.bias[2:] = -5.2983174324035645` → σ = 0.005.
  Ho verificato che le righe logstd `[2:]` di J8 sono **bit-identiche** a J2 (l'audit riportava un
  drift di 1.2e-4: quello è sulle righe 0:2, la testa delle medie).
- **Il manifest di J2 è veritiero su J2** — il suo `module_state_sha256` è quello di J2 stesso.
  Diventa falso solo quando viene copiato altrove.
- **Il manifest committato da J8 è falso su cinque campi**: dichiara `module_state_sha256` del
  *parent* mentre il modulo di J8 è `9c5b1571…`; dichiara `controller_state_mask.active: true`
  mentre le colonne 25..34 di J8 sono **vive** (norme 0.0009–0.1306); `actor_label`, `status` e
  `derived_from` precedono J8. Non è stato mai intercettato perché **nessun loader legge quei
  campi**: RLlib ignora il manifest, e J3/J9R1 leggono solo `actor_feature_names`.
- **Esiste un precedente corretto: J4.** Riusa `class_and_ctor_args.pkl` e `metadata.json` verbatim
  e **rigenera** il manifest col proprio `module_state_sha256` e una `controller_state_mask`
  veritiera. J11 segue J4, non J8.
- Vincoli per il critic warm-up: `class_and_ctor_args.pkl` fissa `inference_only=True`, quindi
  `from_checkpoint` restituirà sempre un modulo actor-only. Il warm-up dovrà costruire un modulo
  fresco `inference_only=False` e trapiantare l'attore. **Tutte e dieci** le chiavi devono essere
  emesse: togliere gli alias romperebbe `warm_start.transplant_actor_state`.

---

## 4. Cosa ho aggiunto oltre alla lettera dell'ordine, e perché

**`actor_digest` nel manifest.** `warm_start.resolve_source_actor_features` confronta attivamente
questo campo con `actor_state_digest(source_state)` e **rifiuta** il checkpoint se non combacia.
J2, J4 e J8 lo omettono tutti — ed è esattamente il buco da cui è passato il manifest stale di J8.
Aggiungerlo trasforma il manifest da documentazione a vincolo applicato. Non è circolare: è un
digest sullo stato, non sul manifest.

L'algoritmo è **trascritto** da `warm_start.py` (che è pinnato), non importato, perché importarlo
tira torch e il preflight deve restarne libero. Il test dimostra che la trascrizione riproduce
esattamente `warm_start.actor_state_digest`.

**`commit()` iniettabile.** Nella prima stesura `commit` eseguiva il fit internamente, il che
avrebbe reso ogni test del write path dipendente da un training reale da 400 epoche. L'ho
rifattorizzato sul modello di J8, che accetta il fit come argomento. Un fit iniettato è un test
double e **non può** scrivere nella root autoritativa; un fit reale **non può** scrivere in una root
redirezionata. Entrambe le direzioni sono testate.

**Distinzione fra i due antenati.** Il parent dei *pesi* è `0f182ea9…` (J2); l'antenato del
*contratto di osservazione* sotto cui sono state raccolte le celle J10R1 è `0ba56eb7…` (l'attore
imitativo V26 a 39 colonne, mai caricato qui). L'audit ha segnalato che sono facili da confondere
in un receipt, quindi sono tenuti distinti ovunque.

---

## 5. Gate offline vincolante

**Integrità** — chiavi/shape/dtype uguali a J2; ogni parametro finito; clock bit-zero nel tensore
diretto **e** nell'alias; alias bit-identici; righe logstd `[2:]` bit-identiche a J2; nessuna chiave
critic; ogni input pinnato invariato dopo il fit; l'aggregato riproduce i propri content hash; lo
split produce i 4943/19770 dichiarati; il best state è ricostruibile dalla history; ogni metrica
finita.

**Esito** — RMSE dopo < prima su **quattro** sottoinsiemi: aggregato 24713, recovery originale 713,
B **unique** 500, C **unique** 500; e tutte e dieci le norme di colonna controller > 0.

**Nessuna soglia inventata.** Ogni regola numerica vincolante è una **disuguaglianza stretta contro
una baseline misurata**. Un check AST verifica che nessun confronto vincolante usi un letterale
float diverso da zero — l'unico `1.0` nella funzione è il bound dell'azione, dentro il diagnostico
di clipping esplicitamente non vincolante.

**Diagnostici, non vincolanti**: RMSE nominale, shift nominale, righe clippate, validation MSE,
epoche, early stop, impostazioni torch osservate.

**Registrato e non corretto**: ogni blocco è tiled, quindi la stessa riga unica compare in
**entrambe** le partizioni. Ereditato da July e da J7. La validation MSE è un diagnostico di
training, **mai** una stima di generalizzazione.

---

## 6. Manifest

`class_and_ctor_args.pkl` e `metadata.json` restano **byte-identici** a J2. Il manifest è
**rigenerato**: 35 feature correnti, `module_state_sha256` del modulo effettivamente scritto,
`actor_digest` e `source_actor_digest`, lineage J2 con i due antenati distinti, contratto clock
(pesi esattamente zero) e controller (`masked: false`, LIVE, con le norme di colonna),
`deployable: false`, `status` che riporta l'esito offline PASS/FAIL, `closed_loop_qualification:
PENDING`, `critic: ABSENT`, e una lista esplicita di claim **non** fatti.

Il runner **rifiuta di partire** se la preregistrazione dichiara `manifest_policy.byte_identical_copy`
diverso da `false`.

---

## 7. Commit atomico

Protocollo J8 più l'hardening J10R1: lock esclusivo `O_CREAT|O_EXCL`, staging fratello, round-trip
del module state, receipt con l'hash di **ogni** file committato, marker `TECHNICAL_INVALID` scritto
nello staging **prima** del rename — quindi il leaf **nasce invalido** — verifica post-commit che
ri-risolve ogni path e ricalcola ogni hash contro il **receipt committato**, confronto dei byte del
receipt con quelli staged, e rimozione del marker come **ultima** scrittura solo se la verifica
passa.

**Regola di validità**: un leaf J11 è evidenza valida **se e solo se** `commit_verification.json`
esiste e dichiara `pass: true`.

---

## 8. File creati

| File | SHA-256 |
|---|---|
| `v26c_j11_prereg_multistart_fit.json` | `49e352466cc82ea9c0a1d7bf29608d41ba4457dfb8a83f3a050a6f1cd752e472` |
| `v26c_j11_multistart_fit.py` | `2b4ac9f496f7a412d40fec0eb5a0d9b69ee22ac634b8e7ed020ca5480cf26242` |
| `test_v26c_j11_multistart_fit.py` | `180607e79be29b426456a333005b39e1501adf1372dd26b656efbb19706ae6f7` |
| `v26c_j11_multistart_fit_authorization.json` | `faeb5eeb6db44080be8f41b638a76040091bd3e6c0fe77a5c374d2ea4e07bb4c` |

Tutti additivi. **Nessun file J0–J10R1, nessun artefatto July, nessun report esistente e nessuna
configurazione di produzione è stato modificato.** FSM, corridoio morfologico, reward, SEA e plugin
C++ non sono toccati. Il worktree dirty dell'utente è preservato: `git status` mostra solo i tre
file già modificati a inizio sessione.

---

## 9. Test e preflight

- **Selftest: 195 check, PASS.** Include: layout e boundary dei blocchi; prova che `np.repeat`
  darebbe la stessa shape e contenuto diverso; identità bit-a-bit delle fonti dentro l'aggregato;
  il fatto clock `(0,1)` misurato su entrambe le celle e la proiezione che tocca **solo** le colonne
  0:2; provenance J10R1 (`commit_verification pass`, nessun marker, hash vincolanti onorati); split
  ricostruito **indipendentemente** con un `default_rng(123)` fresco e prova che ordinare la metà
  di training cambierebbe ogni epoca; precondizioni binding sul parent; equivalenza della
  trascrizione `actor_state_digest` con la funzione reale di `warm_start`; gate PASS e gate FAIL
  sintetici, con ogni violazione d'integrità esercitata singolarmente; verità del manifest a
  confronto **misurato** con quello stale di J8; preflight inerte sotto primitive di scrittura
  monkeypatchate; contratto torch-free statico e dinamico; controllo AST che il preflight non
  chiami mai `pickle.load`; l'intero write path in root temporanea con lock conteso, cleanup con
  sentinella concorrente che sopravvive, doppio commit rifiutato, manomissioni intercettate, e il
  percorso "verifica fallita → marker + leaf preservato".
- **Preflight inerte: GO**, nessun blocker, **torch assente da `sys.modules` prima e dopo**.
- Rifiuti di argomento verificati: `--fit` senza stage token, con il token J8, e `--out` senza
  `--fit` falliscono tutti fail-closed.
- Nessun `j11_runs`, nessun lock, nessuno staging, nessuna sentinella.

---

## 10. Comando congelato proposto (NON eseguito, NON autorizzato)

```
cwd:         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter: /opt/anaconda3/envs/envCMC-rllib/bin/python
argv:        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j11_multistart_fit.py
             --fit
             --authorized-stage V26C_J11_MULTISTART_FIT
             --out <repo>/Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j11_runs/j11_multistart_fit_v26c_2026-08-27_r1
```

`--progress` è l'unica variazione ammessa e cambia solo lo stdout. `OUTPUT_ROOT_OVERRIDE` deve
restare non impostata; il runner rifiuta comunque un fit reale in una root redirezionata.

---

## 11. Blockers

**Nessuno.** Preflight GO, selftest PASS, tutti i pin verificati, nessun leaf preesistente.

Da tenere presente prima di un eventuale GO: l'inventario del leaf è **definitivo al momento della
scrittura**. Lo stadio successivo pinnerà questi file esattamente, e l'hash del manifest non potrà
essere corretto dopo, senza invalidare i pin che lo referenziano.

---

## 12. TODO propagati

- **LOTO** — non integrato. TODO futuro, non J11.
- **LOCO** — non integrato. TODO futuro, non J11.
- **B1R1** — non integrato. TODO futuro, non J11.
- **B1R2** — non integrato. TODO futuro, non J11.
- **Epic generalizzazione multi-modello** — APERTO. Questa fase copre ancora AB06 e tre start dello
  stesso trial; non dimostra alcuna generalizzazione.
- **Qualifica closed-loop** — NON parte di questa fase e non autorizzata da essa. Un PASS qui è un
  risultato **offline**.
- **Critic warm-up** — NON parte di questa fase. Dovrà costruire un modulo fresco
  `inference_only=False` e trapiantare l'attore: non può passare da `from_checkpoint`.
- **J9R1 FAIL sulla cella B** — resta la ragione per cui questa catena esiste. Il checkpoint non è
  training-ready finché non è dimostrato in closed-loop, e questo fit non lo dimostra.
- **Copertura WAIT_HS asimmetrica** — le 20 righe con `phase_fsm_wait_hs == 1` vengono tutte dalla
  cella B. Nell'aggregato diventano 160 dopo il tiling ×8. Nessuna soglia è stata inventata su
  questo numero.
- **Fattore repeat 8 senza razionale documentato** — replicato come valore osservato di July.
  Se l'architetto vuole un valore giustificato invece che ereditato, è una decisione sua.
- **Discrepanze documentali July** (label nominali, gate `ok:false`, drift dei numeri di riga) —
  segnalate sopra, non corrette da questa fase: J0–J10R1 e i report esistenti sono immutabili.

---

## 13. Cosa questa fase **non** ha fatto

- **Nessun fit eseguito.** Nessun leaf creato. Nessun optimizer step.
- Nessun critic, PPO, cluster Ray, env runner o rollout.
- L'autorizzazione è scritta ma dichiara `execution_permitted_now: false`.

**Fermo in attesa della revisione dell'architetto.**
