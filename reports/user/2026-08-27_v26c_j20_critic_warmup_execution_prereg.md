# V26C J20 — Preregistrazione dell'esecuzione del warm-up critic-only

**Data**: 2026-08-27
**Stadio**: `V26C_J20_CRITIC_WARMUP` — **readiness**
**Esecutore**: Opus 5, effort xhigh — braccio operativo. Codex architetto e gate owner.

## ESITO: **PREPARATO / NON ESEGUITO — GO in stato DRAFT**

Nessun warm-up. Nessun child avviato. Nessun Ray, ambiente, rollout, optimizer, restore o
PPO. Nessun leaf, staging, lock o marcatore creato.

**Il GO è un DRAFT e il runner lo rifiuta**: `validate_go` respinge uno `status` `DRAFT` o
`PROPOSED`, quindi il file non può avviare nulla nemmeno se passato per errore a `--execute`.

---

## 1. Il bloccante trovato in preparazione, e come è stato risolto

La preparazione ha trovato che il warm-up, con il config J20, **sarebbe abortito
all'iterazione 1**. Non un'ipotesi: ho eseguito il guard come funzione pura.

`_enforce_kl_update_guard` (`train_ppo_mlp.py:716-773`) gira a **ogni** iterazione
(`:2028-2034`) e richiede le quattro metriche `kl_update/*`. Quelle esistono **solo** se è
installato il learner custom, e ciò accade **solo** sotto il contratto exact-start
(`:1380-1394`). Il warm-up deve tenere `exact_start_sampling: false`, perché quel contratto
pretende `num_epochs 1` e tre offset mentre le meccaniche di luglio sono 10 epoche e una sola
partenza. Guard attivo + learner custom assente → tutte e quattro le metriche `None` →
**7 check su 7 falliti** → `RuntimeError`, `stop_reason: "error"`, `ok: false`, exit 1.

| run | `exact_start_sampling` | `max_minibatch_mean_kl_loss` | esito |
|---|---|---|---|
| luglio, entrambi i warm-up | assente | **assente** | funzionò |
| agosto B0820 frozen-actor | true | 0.01 | funzionò |
| config J20 | **false** | **0.01** | **avrebbe abortito** |

**Emendamento additivo adottato**, per tua decisione: un nuovo config immutabile
`v26c_j20_warmup_critic_only_cfg.yaml`, con **una sola differenza semantica**.

```
supervision.max_minibatch_mean_kl_loss:  0.01  ->  null
```

**Correzione recepita**: la chiave vive sotto `supervision:`, non `training:`.
`training_config.SECTION_MAP` la colloca lì e il config J20 la porta a riga 109. La tua
verifica diretta coincide con l'implementazione.

`v26c_j20_warmup_cfg.yaml` **non è modificato né sovrascritto**; `train_ppo_mlp.py` **non è
toccato**. `exact_start_sampling` resta `false`.

**Perché è scientificamente inerte qui.** Il guard esiste per intercettare un update
dell'**attore** fuori controllo. In un warm-up critic-only attore e log-std sono congelati, il
rapporto di policy è identicamente 1 e il KL è identicamente 0: non può vincolare nulla. Al
suo posto restano prove dirette, tutte gated: **G5** attore byte-identico a ogni audit, **G6**
log-std byte-identico e σ 0.005, **G4** KL medio standard entro 1e-09.

Verificato in tre modi: diff semantico dei due YAML parsati (**1 differenza**, sette sezioni
identiche), esecuzione del guard sul set di metriche del learner stock (spara con 0.01, tace
con `null`), e precedente di luglio (chiave assente in entrambi i config).

## 2. Il comando: una sola sorgente, una sola sostituzione

Il comando **non è riscritto**. Il runner importa `sealed_command()` dal runner di readiness —
dopo averne verificato i byte contro l'hash pinnato — e sostituisce **un solo token**, il path
del `--config`. Verifica prima che il token che sta per sostituire sia davvero il config J20, e
poi che nessun altro token si sia mosso.

23 token. `--worker-process` (nessun supervisor, nessun restart, nessun crash-retry),
`--iterations 1`, nessun `--resume-from`, output-dir assoluto e fuori da `runs/` — condizione
necessaria perché `_resolve_category_output_dir` lo onori verbatim.

**Un solo `Popen`** in tutto il file, seguito da `communicate()`. `Popen` e non `run()` perché
il PID deve restare agli atti. Catturati: argv completo, working directory, PID, timestamp UTC
di inizio e fine, exit code, stdout+stderr uniti, scritti nel leaf e hashati.

## 3. Il leaf, nato invalido

Il child scrive **dentro** il leaf, quindi non esiste il pattern staging-poi-rename delle fasi
precedenti. Il runner crea la directory, ci scrive `TECHNICAL_INVALID` **per primo**, e
**asserisce che sia l'unico residente** prima di lanciare.

Quest'ultima non è formalità. `run()` **non rifiuta** una output-dir non vuota,
`_load_iteration_history` **eredita in silenzio** le righe preesistenti, e un
`milestone_iteration_000001` ereditato farebbe fallire duro il child. Con la directory sporca,
un run da una sola iterazione potrebbe legittimamente produrre un `train_iterations.jsonl` a
più righe.

A fine audit: se ogni gate immediato passa, `TECHNICAL_INVALID` è sostituito da
**`RESTORE_AUDIT_PENDING`**. Altrimenti resta dov'è.

## 4. Il verdetto non può essere PASS

Il gate `G9` ha due metà. La **strutturale** — checkpoint completo con optimizer e module state
a 16 chiavi — è valutata qui. La **esatta** — ricaricare il checkpoint e riprodurre entrambi i
digest — richiede un restore, che questo stadio è strutturalmente incapace di fare.

Il miglior esito disponibile è quindi **`AWAITING_RESTORE_AUDIT`**, non `PASS`. Dichiarare
`PASS` rivendicherebbe evidenza che non esiste ancora. Su qualunque fallimento:
**`FAIL_CLOSED`**, evidenza preservata, marcatore invalido al suo posto, **zero retry**.

Il test asserisce che la stringa `PASS` non sia mai assegnata come verdetto da questo stadio —
compare solo dove si **leggono** i verdetti di J19B e J19C.

## 5. Il layout del checkpoint è letto, non supposto

Ricavato dagli artefatti di luglio committati. Due percorsi sono **trappole**:

| percorso | contenuto reale |
|---|---|
| `learner_group/state.pkl` | **dict vuoto, 5 byte** |
| `learner_group/learner/rl_module/module_state.pkl` | **dict vuoto, 5 byte** |
| `learner_group/learner/state.pkl` | **l'optimizer**, 4 chiavi top-level |
| `learner_group/learner/rl_module/default_policy/module_state.pkl` | **le 16 chiavi** |

Un auditor che avesse aperto le prime due avrebbe concluso che optimizer e modulo mancavano.
Il runner apre quelle giuste e **registra cosa ha trovato anche nelle due esche**, così un
lettore futuro non può ripetere l'errore.

**Una lettura indipendente del freeze, dall'optimizer.** Dei 12 parametri del gruppo, solo
**6** portano momenti Adam, agli indici 6–11: i sei tensori critic. Un attore congelato non
riceve gradiente, quindi non può acquisirne. L'artefatto di luglio mostra esattamente la stessa
firma 6-su-12. È diventata parte di `G9`.

**`checkpoint_best` può non esistere** dopo una sola iterazione, se `episode_return_mean` è
`null` come accadde a luglio: non è asserito. **`supervisor_state.json` non è scritto dal
worker**: la sua assenza è prova diretta per `G10`.

## 6. I dodici gate immediati

| id | criterio |
|---|---|
| **G1** | una sola riga, `iteration == 1`, `iterations_completed_this_process == 1` |
| **G2** | `num_env_steps_sampled_lifetime` esattamente **4096.0** |
| **G3** | `vf_loss` e `vf_explained_var` **finiti** — nessuna soglia |
| **G4** | KL medio finito e `|KL| <= 1e-09` |
| **G5** | ogni voce di `actor_freeze_audit` con `exact` true, `max_abs_diff` 0.0 e digest `d4a13ff7…`; e i dieci tensori attore byte-identici a J19A |
| **G6** | righe 2:4 di `pi.1.weight` e `pi.1.bias` byte-identiche; σ 0.005 entro 1e-09 |
| **G7** | il digest del critic **cambia** |
| **G8** | l'hash del file modulo **differisce** da quello di J19A |
| **G9** | **solo metà strutturale**: optimizer al percorso giusto, 4 chiavi, 12 parametri, **6** entry Adam; module state a 16 chiavi |
| **G10** | exit 0, `summary.ok` true, `stop_reason "completed"`, nessun timeout, nessuna interruzione, **nessun `supervisor_state.json`** |
| **G11** | blocco `training_health` presente, otto campi non nulli, `missing_telemetry_rows` 0, somma = 4096 |
| **G12** | i sette file del leaf J19A invariati, prima e dopo |

**Su G2**: 4096 sono i **timestep campionati** nell'unica iterazione — totale e delta
coincidono perché il run parte da un algoritmo fresco trapiantato. **Non** è un numero di
environment: quelli sono 13 EnvRunner.

**Su G3**: nessuna soglia su EV. Luglio passò EV 0.598 ed EV 0.238 allo stesso modo.

**Telemetria registrata, non vincolata** — tua decisione, punto 6. `phase_timeout_stance_rows`,
`morphology_causal_contract_failure_rows`, `resync_event_rows`, `resync_count_max`,
`hs_cancelled_count_max` sono **misure**; il gate è la loro **presenza e completezza** (`G11`).
Nessun run esistente porta un blocco `training_health`: è più recente di ogni artefatto, e
l'auditor tratta un campo assente come `null`, mai come zero.

## 7. Artefatti

| SHA-256 | righe | file |
|---|---|---|
| `3ecd96a18a9a6e1d475c823b4fbf37d176fd549cb3c1fe4f57e36518548213f2` | 278 | **NUOVO** `v26c_j20_prereg_critic_warmup_execution.json` — sigillata |
| `ce70797ad6362298c63fccceeab127c760d5dd68bfe3e96ef8f92b76f8395114` | 1333 | **NUOVO** `v26c_j20_critic_warmup_execution.py` |
| `5e90c3df90e2bdeccde4e1f92abcddcf1cf2595e116f00f1a40b7ea59a3e9990` | 762 | **NUOVO** `test_v26c_j20_critic_warmup_execution.py` |
| `e98bde4eb3b1d339d4f94dd5b0883d874db0c8971c19adb55404026584bf0c20` | 276 | **NUOVO** `v26c_j20_warmup_critic_only_cfg.yaml` |
| `6a4b22c29fed2221b1870f788d9b7ede78b6a656eff418d25baa2d2eec78fa53` | 96 | **NUOVO** `v26c_j20_critic_warmup_go_DRAFT.json` — **rifiutato dal runner** |

## 8. Il GO: 41 pin, mappa chiusa

Il payload porta **label e hash**, mai path: ogni label è risolta da `go_pin_targets()` del
runner. Le label fuori da questa directory — i quattro moduli di produzione, la suite di
telemetria, il config runtime pinnato — sono risolte da costanti interne, ed è la ragione per
cui la mappa esiste.

Copertura: i tre artefatti di questo stadio; **entrambi** i config; l'overlay; il deriver; il
runner, la prereg e il GO della readiness; prereg, runner e GO di K1R1; i quattro moduli di
produzione che il **child** esegue; la suite di telemetria; il config runtime pinnato; i sette
file del leaf J19A; i receipt e le commit verification di J19B e J19C; i sei file del leaf K1;
i cinque del leaf K1R1. **Nessun report fra i pin.**

Rifiuti verificati: stage sbagliato, `authorises_execution` non esattamente `true`, uno
qualsiasi dei sei flag proibiti, pin mancante, pin stantio, pin fuori mappa, **e lo status
DRAFT**.

## 9. Verifiche eseguite

| verifica | esito |
|---|---|
| `py_compile` sui file toccati | **OK** |
| **suite esecuzione warm-up** (nuova) | **208/208** |
| suite K1R1 | **206/213** — vedi nota |
| suite telemetria | **161/161** |
| suite readiness J20 | **187/191**, fail esattamente `B01 B03 B05 B07` |
| `derive --check` | matches |
| preflight esecuzione | **READY**, **41/41 pin** |
| dry-run esecuzione | inerte, zero scritture |
| `git diff --check` | pulito |

### Sulle due suite che non sono a punteggio pieno

Entrambe per la stessa ragione, e nessuna è una regressione: sono le asserzioni
**pre-esecuzione** di assenza della propria destinazione, che ora esiste perché quello stadio
è stato eseguito. È la garanzia no-clobber a esecuzione singola che funziona.

- **Readiness J20, 187/191**: fail esattamente `B01 B03 B05 B07`.
- **K1R1, 206/213**: fail esattamente `B01 B03 B04 B07 B09 I01 I03`, tutti e soli i check
  sull'assenza del leaf K1R1. `B02`, «il preflight non scrive nulla», resta verde.

Non ho toccato nessuna delle due suite: i loro byte sono pinnati nei rispettivi GO e nella
mappa dei 41 pin di questo stadio. Nessun check scientifico è regredito in nessuna delle due.

Fra i 208 check nuovi, quelli che mordono: un solo `Popen` in tutto il file, mai
`subprocess.run`, `launch_once` senza loop e con **un solo** call site, la chiamata **non**
dentro un `try/except` che potrebbe rientrarci; `restore_from_path` mai chiamato; il diff
semantico dei due config a **una** differenza con le sette sezioni identiche; il guard che
**spara davvero** con 0.01 e tace con `null`; la sostituzione del comando a **un** token; il
marcatore invalido scritto per primo e rimosso per ultimo, dopo la guardia di uscita anticipata;
e il layout del checkpoint confrontato con l'artefatto **reale** di luglio, esche comprese.

### Nota sui miei test

Quattro check erano miei e imprecisi — tre cercavano sottostringhe («la parola retry», «la
stringa PASS», «restore») invece dell'invariante, e uno aveva il path degli artefatti di luglio
sbagliato di un livello. Riscritti come check AST e di ordine delle istruzioni. Gli artefatti
non sono stati cambiati per far passare i test.

## 9-bis. Le tre correzioni della revisione architetto

| # | prima | ora |
|---|---|---|
| **1** | `G2` leggeva solo `num_env_steps_sampled_lifetime` | legge **entrambe**: la lifetime **e** il delta per iterazione, dal campo che il trainer emette davvero, `learners/__all_modules__/learner_connector_sum_episodes_length_in` |
| **2** | `G9` chiedeva **sei** entry Adam | chiede gli indici **esattamente** `[6, 7, 8, 9, 10, 11]` |
| **3** | `vf_loss` letto solo dal top-level della riga | top-level se presente, **fallback** al literal `learners/default_policy/vf_loss`; la sorgente usata è registrata |

**Su 1.** Su una singola iterazione fresca lifetime e delta coincidono, ed è esattamente il
motivo per cui vanno letti entrambi: un gate che guardasse solo la lifetime non si accorgerebbe
se smettessero di coincidere. Il campo per-iterazione non è supposto: nel run agosto a cinque
iterazioni vale **4608 a ogni iterazione** mentre la lifetime sale 4608 → 9216 → 13824 → 18432
→ 23040. Il test lo verifica su entrambi gli artefatti reali.

**Su 2.** `named_parameters()` ordina i sei tensori attore unici agli indici 0–5 e la torre di
valore a 6–11. **Sei entry agli indici sbagliati** significherebbero che l'attore ha preso
gradiente e che qualche tensore critic non l'ha preso: il conteggio da solo non può dirlo. Il
set di indici è confrontato con quello dell'artefatto reale di luglio.

**Su 3.** `vf_explained_var` non è **mai** promosso a chiave di riga e resta letto solo da
`learner_metrics`; `vf_loss` invece è promosso solo quando il match per suffisso del trainer
l'ha trovato, e senza fallback un `None` si sarebbe letto come «non finito».

## 10. Punti che richiedono la tua decisione

1. **Rivedere e approvare il GO DRAFT.** Per autorizzare, rimuovi o cambi il campo `status`
   dopo aver controllato i 41 pin.
2. **Il config additivo cambia lo stato dei pin a valle.** `v26c_j20_warmup_cfg.yaml` resta
   invariato e continua a essere pinnato, ma il child ora consuma un file diverso. La prereg
   J20 continua a descrivere il comando sigillato originale: **non l'ho toccata**, perché è
   sigillata e il suo GO è già speso. Se preferisci un addendum che ne dichiari la
   supersessione, è una tua chiamata.
3. **Il restore audit resta dovuto** e non è raggiungibile da qui. Serve un suo stadio e un suo
   GO.

## 11. STOP

Stadio preparato, testato staticamente, **non eseguito**. GO in stato **DRAFT**, rifiutato dal
runner per costruzione.

**Nessun warm-up avviato. Fermo in attesa della tua revisione del GO.**
