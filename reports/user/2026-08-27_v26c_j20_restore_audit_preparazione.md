# V26C — J20: preparazione del restore audit a zero iterazioni (chiude G9)

**Data:** 2026-08-27
**Stadio:** `V26C_J20_RESTORE_AUDIT`
**Stato:** **artefatti consegnati, NON eseguiti.** GO in stato **DRAFT inerte**.
**Foglia di destinazione:** `j20_runs/j20_restore_audit_v26c_2026-08-27_r1/` — **assente**
**Preflight:** READY, 63/63 pin — **Suite ermetica:** 266/266

---

## 1. Problema

Il warm-up critic-only ha chiuso con `AWAITING_RESTORE_AUDIT` e 12/12 gate. Uno di
quei gate, **G9**, era passato solo per metà: la parte strutturale (dodici parametri
nel gruppo, sei voci Adam agli indici `[6..11]`, ognuna con `step/exp_avg/exp_avg_sq`)
era verificata, ma la **ripristinabilità effettiva** no. Il warm-up non poteva
verificarla: ricostruire l'algoritmo e chiamare `restore_from_path` è un'azione
diversa, con un raggio d'azione diverso, e per questo era stata preregistrata come
dovuta e la foglia marcata `RESTORE_AUDIT_PENDING` invece che `PASS`.

Il rischio da escludere è preciso e ha già un precedente in questa pipeline: un
export `inference_only` caricato in un modulo addestrabile lascia un critic
**silenziosamente fresco**, senza eccezione né warning. L'analogo lato ottimizzatore
sarebbe un Adam ricostruito con momenti di forma corretta ma numericamente sbagliati.

Due ostacoli di metodo emersi in preparazione:

1. **Un controllo ermetico non basta.** Caricare il pickle in un `Adam` separato
   costruito dal runner dimostra che il *file* è caricabile. Non dimostra che i
   momenti siano presenti nell'**ottimizzatore vivo** creato dal vero
   `algo.restore_from_path`.
2. **Un `difference_count == 0` non è una prova.** Può essere riportato anche da un
   payload che non ha confrontato nulla.

## 2. Soluzione

Uno stadio distinto che attraversa il **percorso di produzione reale**
`train_ppo_mlp.run` → `algo.restore_from_path`, esegue **zero** nuove iterazioni, e
legge l'ottimizzatore **dal learner vivo** nell'istante in cui il restore ritorna.

**Il meccanismo dello zero-iterazioni** non è un flag ma una proprietà strutturale:
`train_ppo_mlp.py:1755-1756` deriva `iteration_start = restored_logical_iteration + 1
= 2`; il target resta `--iterations 1`; il ciclo a `:1964` è
`range(iteration_start, args.iterations + 1)` = `range(2, 2)`, **vuoto**. Nessuna
`algo.train()` è raggiungibile. `--iteration-start` non viene passato di proposito:
il trainer deve *derivare* il numero, non riceverlo da chi lo deve misurare.

**Il wrapper validation-only** (`v26c_j20_restore_audit_child.py`) importa
`train_ppo_mlp` e ribatte **un solo attributo in memoria**,
`_reapply_optimizer_learning_rate`, che la produzione chiama alla riga immediatamente
successiva al restore. Il sostituto:

1. chiama **prima l'originale** e ne restituisce il valore invariato;
2. cattura subito dopo l'ottimizzatore vivo con la traversata della produzione
   (`_learner_call_results` → `foreach_learner` → `get_optimizers_for_module`);
3. normalizza e confronta ricorsivamente ed esattamente contro
   `checkpoint_last/learner_group/learner/state.pkl`;
4. scrive l'evidenza canonica;
5. **solleva** in caso di mismatch → `run()` registra `stop_reason: "error"` e
   `ok: false` → `main()` fa `SystemExit(1)` → returncode 1 → **il run di produzione
   fallisce**.

Poi imposta `sys.argv` e chiama `train_ppo_mlp.main()`: il restore è quello vero,
non un'emulazione.

## 3. Strategia

**Comando**: quattro operazioni verificate sul `sealed_command()` del readiness
runner — unica sorgente dell'invocazione J20:

| # | operazione | dettaglio |
|---|---|---|
| 1 | sostituzione config | `v26c_j20_warmup_cfg.yaml` → `v26c_j20_warmup_critic_only_cfg.yaml` (fatta da `execution_command()` del warm-up) |
| 2 | rimozione | i 5 token warm-start, così `initialization_mode` legge `resume_from` e non `warm_start_raw` per un run che non trapianta |
| 3 | append | `--resume-from <checkpoint_last>` |
| 4 | sostituzione script | `train_ppo_mlp.py` → il wrapper, perché la cattura live deve esistere dentro il processo |

**Conteggi**: 20 token derivati, **18** delegati al trainer, **25** nell'argv completo
del child. Tutti e tre sono ora **rifiuti**, non annotazioni.

**Tredici gate R1–R13**, distinti dai G1–G12 del warm-up:

| Gate | Contenuto |
|---|---|
| R1 | zero nuove iterazioni (0 righe, `iterations_run` 0, `..._this_process` 0) |
| R2 | `history == []` |
| R3 | `restored_training_iteration` e `restored_logical_iteration` entrambi 1 |
| R4 | `iteration_start` 2 e `next_iteration` 2 |
| R5 | il percorso di restore è stato preso, nessun trapianto |
| R6 | digest attore del modulo **vivo** ripristinato == `d4a13ff7…`, una sola voce di audit |
| R7 | digest critic del modulo **vivo** ripristinato == `2fa9c124…`, sulle sei chiavi |
| R8 | restore ermetico del modulo completo: 16 chiavi `strict=True`, byte-identità, slice log-std, σ |
| R9 | **ottimizzatore vivo byte-esatto** dopo il restore (vedi §4) |
| R10 | niente è stato addestrato: nessun checkpoint, milestone, rl_module o report di trapianto |
| R11 | sorgenti invariate: intero albero della foglia warm-up prima/dopo, più tutti i pin |
| R12 | un solo processo, returncode 0, `completed`, nessun supervisore, nessun retry |
| R13 | seconda opinione ermetica, estesa alla byte-identità dei momenti |

R5–R9 chiudono G9; R1–R4 e R10 stabiliscono che nessun addestramento ha contaminato
l'evidenza; R11–R12 l'integrità.

## 4. R9 e i tre emendamenti immutabili

La prereg base è stata **sigillata e mai riscritta**. Le correzioni sono additive.

- **rev1** (`3fa1ae23…`) — sostituisce R9 con il confronto **live in-process** e
  aggiunge R13. Motivo: il caricamento ermetico prova che il file è caricabile, non
  che i momenti siano nell'ottimizzatore vivo; e R9 chiedeva solo forma e `step`, non
  la byte-identità di `exp_avg`/`exp_avg_sq` né i `param_groups`.
- **rev2** (`59e2e5e0…`) — risolve una contraddizione interna a rev1 sulle regole di
  normalizzazione. `step` è `float` Python nel pickle e tensore 0-D nel live: due
  regole di rev1 li mandavano su rami diversi. Governa la regola scalare; bool resta
  prim tipato (valutato per primo, essendo sottoclasse di `int`), str/None restano
  prim tipati.
- **rev3** (`bfb6ef9b…`) — quattro correzioni e un irrigidimento, senza aggiungere
  gate:
  1. **conteggi**: 20 / 18 / 25, con la nota errata di rev1 corretta additivamente;
  2. **wording del wrapper**: la frase «non scrive alcun file sotto `baseline_MLP`»
     era **falsa** — la foglia di validation di questo stadio sta fisicamente lì. La
     dichiarazione corretta è: **non modifica alcun file di produzione**; l'unico file
     scritto è l'evidenza, dentro la propria foglia;
  3. **stato del GO**: solo la stringa **esattamente `APPROVED`** autorizza. Prima si
     rifiutavano solo i letterali `DRAFT` e `PROPOSED`, quindi uno stato mancante o
     arbitrario sarebbe passato;
  4. **side effect dichiarati** (§6).

  E irrigidisce R9, che ora pretende: `kind` atteso, `stage_marker` `after_restore`,
  `source_state_sha256` == `51d97ef0…`, `exact: true`, `differences` **lista vuota**
  (non solo conteggio 0), `problems` vuoto, i due **digest globali** dell'albero
  normalizzato presenti e **uguali**, `param_groups` con `exact: true`,
  `differences` vuoto e digest coincidenti, chiavi top-level e indici di stato
  `[6..11]` su **entrambi** i lati, `learner_count: 1`, un solo ottimizzatore di nome
  `default_optimizer`, e una voce `after_restore` in `optimizer_lr_audit`.

  Il nome `default_optimizer` **non è un'assunzione nuova**: è già registrato nel
  `train_iterations.jsonl` committato del warm-up, sotto
  `optimizer_learning_rates`.

**Due difetti trovati e corretti in preparazione**, entrambi misurati:

- i numerici Python finivano nel ramo `prim` prima di quello tensoriale;
- `np.ascontiguousarray` **promuove gli array 0-dim a shape `(1,)`**, quindi leggendo
  `ndim` dopo la conversione il ramo scalare era **irraggiungibile**:
  `torch.as_tensor(81.0)` diventava `('tensor','float32',(1,),…)`. Ora `ndim` è letto
  prima.

Misure dopo la correzione: round trip pulito **0 differenze**; un singolo elemento di
`exp_avg` perturbato **esattamente 1 differenza** con sha256 distinti; ottimizzatore
azzerato — il caso "fresh" che tutto questo esiste per escludere — **12 differenze**.

## 5. File

**Creati** (tutti additivi, nessun clobber):

| file | sha256 | righe |
|---|---|---|
| `v26c_j20_prereg_restore_audit.json` | `1326944edcbe368319409705e6ff2aeafb62f3def4503f467736ba6780c7d9e6` | 158 |
| `v26c_j20_prereg_restore_audit_rev1.json` | `3fa1ae238cfe0c20865850951ee787cf528b1c17cc0ba6fe97a714d9e37c3ef1` | 152 |
| `v26c_j20_prereg_restore_audit_rev2.json` | `59e2e5e04d167eaafd8283800d277ca8415fac8d57a880818a082dbdbb838cf7` | 71 |
| `v26c_j20_prereg_restore_audit_rev3.json` | `bfb6ef9b4b9a176138adae837e622442e4727f1088c81ea124f3a91b6a7ed41e` | 116 |
| `v26c_j20_restore_audit_child.py` | `38a57b25da27666a99891511666d63cac07534aa7602f20469048012f536caf4` | 447 |
| `v26c_j20_restore_audit.py` | `322b9abc0a2feae7b2e77f7a51946ae19de34e7013901b6e9425e6ee7c5210ae` | 1336 |
| `test_v26c_j20_restore_audit.py` | `07213d4d13c2ffb0965673ec7960ce8db3153fb9c3393006a77d0aaba0530860` | 1049 |
| `v26c_j20_restore_audit_go_DRAFT.json` | `794ba369b7451a85b8c0c063e15661b123521dd59517129a0cad23ccea580ea8` | 108 |

**Non modificati**: nessun file di produzione — `train_ppo_mlp.py`, `tb_logging.py`,
`warm_start.py`, `asymmetric_rl_module.py`, plugin C++/SEA, FSM v3, detector,
morfologia, reward, soglie, architettura, σ. Nessun config: né
`v26c_j20_warmup_cfg.yaml`, né `v26c_j20_warmup_critic_only_cfg.yaml`, né
`training_exnovo_cfg.yaml`. La foglia del warm-up e tutti gli artefatti J0–J19C sono
byte-invariati.

## 6. Side effect dichiarato

Il config non viene toccato: chiede `num_env_runners: 13` e `ray_num_cpus: 14`.
Costruire l'Algorithm costruisce quei 13 EnvRunner, e costruire un EnvRunner
costruisce il suo ambiente OpenSim; RLlib può eseguire un reset iniziale in setup per
leggerne gli spazi.

**È costruzione di ambiente, non sampling né rollout.** Zero `algo.train`, zero
campionamento, zero rollout — garantito strutturalmente dal range vuoto e **misurato**
da R1, R2 e R10.

Non abbasso `num_env_runners` a 0: il checkpoint è stato scritto da una
configurazione a 13 runner e `restore_from_path` ripristina anche lo stato degli
EnvRunner. Auditarlo con un parallelismo diverso significherebbe auditare un altro
oggetto. Il costo è dichiarato, non evitato.

## 7. Verifiche eseguite

- `py_compile` sui tre moduli Python: **OK**
- suite ermetica `test_v26c_j20_restore_audit.py`: **266/266**, senza child, Ray,
  ambiente o restore RLlib
- `--preflight-only`: **READY, 63/63 pin**; sottoprocesso di controllo che né `torch`
  né `ray` finiscono in `sys.modules`
- `--dry-run`: stampa il piano e il comando, **non scrive nulla**
- GO DRAFT: `validate_go` → `valid: False`, unico problema
  `"status is 'DRAFT', and only the exact string 'APPROVED' authorises execution"`,
  **63/63 pin corretti**. Controprova: lo stesso payload con `status: "APPROVED"`
  valida senza problemi, quindi l'inerzia dipende **solo** dallo stato
- sigilli precedenti: base, rev1 e rev2 **intatti** dopo l'aggiunta di rev3
- foglia warm-up: **invariata**, marker `RESTORE_AUDIT_PENDING` al suo posto
- foglia di destinazione: **assente**; nessun processo child o Ray attivo

**Non vacuità dei gate**: ognuno dei 13 è dimostrato flippare a `False` sotto una
perturbazione mirata del proprio input — 55 perturbazioni in totale, di cui 24 sul
solo R9 (conteggio falsificato con lista differenze non vuota, digest mancanti o
diversi, `digests_match` falsificato, `param_groups.exact` falsificato,
`param_groups.differences` non vuoto, SHA sorgente errato, `kind` errato, marker di
stadio errato, chiavi top-level e indici errati su entrambi i lati, nome
ottimizzatore inatteso, due ottimizzatori, `problems` non vuoto).

## 8. Che cosa questa preparazione **non** è

- **Non** è un'esecuzione. Nessun restore è stato effettuato.
- **Non** dimostra che il restore funzionerà: dimostra che, se non funzionasse, lo
  stadio lo direbbe e fallirebbe chiuso.
- Un eventuale `RESTORE_AUDIT_PASS` chiuderà G9 e **niente altro**:
  `promotion: NONE`, `training_ready: false`, `next_stage_authorized: false`.

## 9. TODO

- [ ] **Review dell'architetto e GO operativo** (file distinto, `status: "APPROVED"`)
      per l'esecuzione singola del restore audit. Nessun retry, qualunque sia l'esito.
- [ ] Eventuale pronuncia su un rev4 se la review rileva altro; rev1 e rev2 restano
      sigillati e non riscritti.
- [ ] TODO ereditati ancora aperti: generalizzazione multimodello (epic del 22/08);
      chiusura del gate finale di recupero AB06.

---

**Stato conclusivo:** stadio preparato e fermo. Preflight READY, 63/63 pin, suite
266/266, GO in DRAFT inerte, foglia assente, sorgenti invariate. **In attesa della
review e del GO operativo.**
