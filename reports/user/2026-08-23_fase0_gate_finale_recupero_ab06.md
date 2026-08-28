# Fase 0 — Gate finale del recupero AB06

**Data:** 23 agosto 2026  
**Piano:** `reports/plans/2026-08-22_piano_operativo_recupero_pipeline_exnovo_v26_fsmv3_morphology.md`  
**Perimetro concluso:** F0 — freeze, provenienza, matrice comparativa, overlay e deriva actor  
**Esecutore operativo/revisore:** Claude Fable, effort `max`, da terminale  
**Architetto e gate owner:** Codex

## 1. Decisione finale

La Fase 0 è **completata con PASS tecnico**: il perimetro congelato è completo, riproducibile, content-addressed e verificato; tutti i 58 rollout previsti sono stati eseguiti o rigiocati, analizzati e legati alla rispettiva provenienza.

Il risultato comportamentale della lineage B0820 è invece **FAIL come baseline di recupero AB06**. I checkpoint `B0820_H0`, `B0820_V2_BEST`, `B0820_V3_BEST` e `B0820_V3_LAST` conservano ROM compresse, caviglia quasi sempre o interamente positiva nei casi rilevanti e un numero insufficiente di cicli validi. I 50 update PPO non hanno ricostruito la forma biologica persa prima del training.

La lineage del 15 luglio resta quindi la **baseline di rollback**. Il controllo 39D V26 e i controlli di luglio dimostrano che la qualità di forma non è incompatibile in assoluto con il runtime corrente, ma non sono confronti isometrici e non vengono promossi direttamente a policy deployable.

Per istruzione dell'utente, il processo si arresta qui. **F1, Teacher–Student/DAgger, critic warmup e nuovi training non sono stati avviati.**

## 2. Problema indagato

La nuova pipeline

```text
warm start imitativo V26 -> transplant 39D→35D -> warmup critic -> PPO B0820 -> FSM v3 + corridor
```

produce traiettorie molto peggiori della pipeline di luglio

```text
baseline imitativa 24/06 -> adattamento target-domain 35D -> DAgger/recovery -> H0 13/07 -> PPO 15/07
```

pur condividendo gran parte del software e della struttura di training. Occorreva distinguere una regressione dell'actor da problemi introdotti da FSM v3, Morphology Corridor, riproducibilità storica, rumore stocastico o topologia RLlib.

## 3. Strategia eseguita

La Fase 0 ha applicato il seguente protocollo fail-closed:

1. freeze dei checkpoint, manifest di feature, configurazioni, profili, plugin e input scientifici;
2. inventario separato delle lineage luglio, V26 e B0820, inclusi actor, critic, optimizer e testa `log_std`;
3. matrice canonica di 58 job, senza uso dei seed sigillati 126–128:
   - 9 replay di configurazioni storiche;
   - 19 rollout deterministici;
   - 27 rollout stocastici sui seed di sviluppo 123–125;
   - 3 controlli nativi V26 39D;
4. verifica delle receipt, degli input scientifici, dell'interprete, del comando effettivo e della source closure pre/post;
5. analisi quantitativa di cinematica effettiva e servita, coppie SEA, GRF, reserve, FSM v3, ledger causale e Morphology Corridor;
6. misura della deriva dell'actor su un corpus congelato di 10.034 osservazioni;
7. overlay time-aligned per tutti i 58 job e phase-aligned solo quando il contratto HS→TO→HS è dimostrabile;
8. re-audit indipendenti iterativi e correzione degli ultimi rilievi prima del PASS.

## 4. Evidenze principali

### 4.1 Completezza e ripetibilità

- matrice: **58/58 `PASS_ANALYSED`**;
- analisi globale: `analysis_complete = true`, `executed_subset_complete = true`;
- distribuzione: 9 replay, 19 deterministici, 27 stocastici, 3 controlli 39D;
- ripetizione deterministica `B0820_V3_BEST` nominale: trace e serie 1 kHz bit-identiche;
- replay B0820: bit-identici ai riferimenti disponibili;
- replay luglio divergenti sul codice corrente per una causa localizzata e documentata: il routing delle feature online di carico/contatto usa ora la GRF primaria, mentre il runtime storico le derivava dal detector legacy. I replay di luglio sono pertanto controlli storici, non confronti isometrici.

### 4.2 Il collasso precede PPO

Sotto runtime v3, i quattro actor B0820 deterministici producono **zero cicli validi a tutti e tre gli start**. La forma di H0 e dei checkpoint best/last è quasi sovrapposta, coerentemente con il KL estremamente piccolo osservato durante i 50 update.

Esempio nominale:

| actor | cicli | knee ROM [rad] | ankle min/max [rad] | reserve max [Nm] |
|---|---:|---:|---:|---:|
| B0820_H0 | 0 | 0,336 | +0,031 / +0,283 | 894,8 |
| B0820_V3_BEST | 0 | 0,339 | +0,031 / +0,273 | 876,8 |
| JUL_H0 sotto v3, controllo storico | 2 | 0,921 | −0,076 / +0,518 | 752,7 |
| V26 nativo 39D, controllo di compatibilità | 2 | 0,997 | +0,030 / +0,436 | 939,9 |

Il knee B0820 non è fisicamente positivo: è compresso verso l'estensione. Per l'ankle, invece, la positività è reale e coincide con l'assenza della fase negativa biologicamente richiesta. L'interpretazione usa la cinematica fisica servita/effettiva, non il solo segno dell'azione raw.

### 4.3 Esiti stocastici e contatori v3

Sui 27 rollout stocastici:

- 18 terminano regolarmente per `episode_time_limit`;
- 4 terminano per `grf_penetration`, tutti `JUL_H0`;
- 3 terminano per `phase_timeout:swing`;
- 2 terminano per `morphology_causal_contract_failure`;
- **0 terminano per `phase_timeout:stance`**.

Aggregati per candidato:

| candidato | orizzonte | cicli riportati | `resync_count` | `hs_cancelled_count` | failure distintive |
|---|---:|---:|---:|---:|---|
| B0820_H0 | 8/9 | 4 su 3 job | 8 | 14 | 1 swing timeout |
| B0820_V3_BEST | 5/9 | 2 su 1 job | 7 | 13 | 2 morphology, 2 swing timeout |
| JUL_H0, controllo storico | 5/9 | 13 su 5 job | 0 | 1 | 4 penetrazioni |

FSM v3 riduce quindi lo stance lock e permette spesso di raggiungere l'orizzonte tramite resync, ma non ricostruisce la forma dell'actor. Il semplice raggiungimento dei 500 step non è un gate di qualità sufficiente.

### 4.4 Causa primaria

La regressione dominante è il passaggio **39D→35D senza refit target-domain**:

- vengono eliminate quattro feature cinematiche privilegiate del teacher;
- i due ingressi `gait_phase_sin/cos` condivisi vengono azzerati nel target causale;
- il transplant conserva gli altri pesi, ma non conserva la funzione non lineare della policy;
- il clock online resta non informativo finché la policy non genera il primo ciclo valido, creando un deadlock di bootstrap;
- B0820 eredita una testa `log_std` state-dependent con sigma dell'ordine di 0,5, mentre luglio usava sigma costante `0.005`;
- il Morphology Corridor, peso `0.0025`, è un guardrail sparso e causale: non è un teacher capace di ricostruire una gait collassata;
- il critic warmup B0820 è stato eseguito prima del runtime v3/corridoio definitivo.

La deriva misurata conferma che PPO non ha risolto il problema. I pair B0820 H0→best hanno RMSE della media fra circa `0.0052` e `0.0063`, ma KL fra `1.26e-4` e `1.90e-4` sul corpus congelato perché la sigma ereditata è molto larga e state-dependent. Nella lineage luglio le variazioni della media sono più piccole ma operano contro sigma costante `0.005`, quindi rappresentano update controllati e funzionalmente visibili.

### 4.5 Incoerenza semantica scoperta nella FSM

L'overlay end-to-end ha trovato due trace nelle quali il runtime riporta `cycle_valid = 1` dopo `heel_strike_cancelled` senza una nuova catena accettata HS→TO→HS:

- `B0820_H0__v3_canonical__plus020__stoch_seed123`, step 453;
- `B0820_V3_BEST__v3_canonical__plus020__stoch_seed124`, step 449.

Il parser F0 resta fail-closed: i due job sono marcati `invalid_fsm_cycle_contract`, non producono phase CSV/PNG, conservano gli output time-domain verificati e riportano integralmente `phase_contract_error` nel record CSV, nel manifest e nel Markdown. L'artefatto finale contiene quindi 23 figure time-aligned e una sola figura phase-aligned valida.

Questa anomalia non invalida i dati time-domain né il freeze F0, ma costituisce un prerequisito tecnico da correggere o rendere esplicitamente identificabile prima di usare i contatori runtime come verità per i gate phase-aligned futuri.

## 5. Chiarimento definitivo sul valore 4096

Nel training B0820 **non esistevano 4096 environment e il loro numero non aumentava**:

- 12 EnvRunner remoti × 1 environment = 12 environment di sampling;
- 1 runner locale era usato per inference/sincronizzazione e non aggiungeva un frammento remoto;
- batch effettivo per update: `4608 = 12 × 384 = 3 × 1536` timestep unici;
- `num_env_steps_sampled_lifetime` cresceva di 4608 a ogni iterazione perché è un contatore cumulativo;
- `4096` apparteneva al vecchio batch target V26 e compare anche come cap delle diagnostiche morphology, non come numero di environment;
- le righe learner pre-compaction, ripetute nei nove minibatch, non rappresentano nuovi environment o nuove transizioni uniche;
- supervisor: `restart_count = 0`, `crash_restart_count = 0`; non esiste evidenza di replacement, ma la run non aveva ancora un audit dedicato delle generation degli EnvRunner.

## 6. Soluzione raccomandata, non avviata

La prosecuzione prevista dal piano resta:

1. F1: ablation isolata di hard-drop e sigma;
2. bridge supervisionato target-domain 35D sotto runtime finale;
3. teacher AB06 protesico/corridor-compliant, anchor anti-forgetting e DAgger phase-aligned multi-start;
4. testa `log_std` strutturalmente costante a sigma `0.005`;
5. gate closed-loop prima del critic: cicli, ROM, fase negativa ankle, forma, coppie, reserve e copertura morphology;
6. critic warmup e solo dopo un pilot PPO breve.

Questa soluzione non è stata eseguita. Se il bridge non recupererà i gate preregistrati entro il budget concordato, il piano prevede il rollback operativo alla baseline del 15 luglio.

## 7. Artefatti finali

Radice congelata:

`Trajectory Generator/runs/rollout/validation/f0_freeze_runs/2026-08-22_F0_freeze_inventory_baseline_r3/`

Artefatti decisionali:

- analisi matrice: `metrics/f0_matrix_analysis_20260823_011338.json`, SHA-256 `0ac942432d0505731b93e6322ac337edf91ab7dca8c9c8e9c1cb122089148987`;
- report matrice: `metrics/f0_matrix_analysis_20260823_011338.md`, SHA-256 `58212261fd18820ff3829b7939f7633033d8d62e3054d29478a6ccb2761a23c0`;
- deriva actor: `drift/20260823_033635/`, verifica strict PASS, 5 file, 5.233.728 byte, sidecar SHA-256 `9370b7001b34d67b7489eaf9a3b263c00e12552c708163c551a41611192312a4`;
- overlay finale: `overlays/20260823_034411/`, verifica strict PASS, 216 file, 128.179.435 byte, sidecar SHA-256 `ce74a9d5501d8613a761a66b8cdac417582b57dc0d27cd5b58ede163e4d0cba2`;
- manifest overlay: SHA-256 `30b5b01323dbd3a0dd46aa3bb865907b9de3a1b61473152f2c3a8363b2af4c99`;
- riepilogo overlay: SHA-256 `152b228028310fec8903a152be98b0ad294b71425e85c3bcdc3d7c63298c9a1c`.

L'overlay precedente `20260823_033856` resta immutabile come revisione superseded e non è l'artefatto decisionale finale.

## 8. Codice introdotto o modificato

Tutto il codice F0 vive in:

`Trajectory Generator/baseline_MLP/validation/f0_freeze_2026-08-22/`

Componenti principali:

- `f0_common.py`, `f0_closure.py`, `f0_artifacts.py`: registro, source closure, ledger byte-bound e pubblicazione no-replace;
- `f0_freeze_inventory.py`, `f0_runtime_matrix.py`, `f0_census_b0820.py`: freeze, provenienza, runtime e censimento;
- `f0_rollout_matrix.py`, `f0_replay_analysis.py`, `f0_matrix_analysis.py`: esecuzione/verifica e analisi della matrice;
- `f0_source_closure_assessment.py`: qualifica retrospettiva delle receipt legacy;
- `f0_actor_drift.py`: corpus congelato e deriva/KL dell'actor;
- `f0_overlays.py`: esportazione CSV/PNG time/phase, corridoio, FSM, SEA, GRF e reserve;
- sei self-test corrispondenti.

Non sono stati modificati pesi, checkpoint, plugin C++, modello OpenSim, reward, configurazione di training o semantica SEA. I tre file user-owned già sporchi sono rimasti invariati, con gli stessi SHA-256 registrati prima della Fase 0.

## 9. Test e revisioni

Ultima suite eseguita con `/opt/anaconda3/envs/envCMC-rllib/bin/python`:

- receipt/matrice: 138 PASS;
- source closure: 23 PASS;
- replay: 23 PASS;
- analisi matrice: 33 PASS;
- actor drift `--real`: 159 PASS;
- overlay `--real`: 23 PASS;
- totale: **399 controlli PASS**;
- `py_compile`: PASS su tutti i 19 file Python del perimetro F0.

Verifica indipendente finale del delta overlay: **PASS**. Sono stati confermati l'esatta propagazione dello stato dei due job degradati, la corrispondenza del manifest, la tabella Markdown, 58 job, una phase figure, nessun file mancante o non elencato e l'uguaglianza fra SHA dello script corrente e quello registrato nel sidecar.

## 10. TODO aperti e stato di arresto

- **Recupero AB06:** F1–F7 non iniziati; attendono un nuovo via esplicito.
- **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed.
- **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti.
- **Generalizzazione multi-modello EPIC:** resta differita e tracciata in `reports/user/2026-08-22_todo_generalizzazione_multimodello_epic.md`, come deciso nella conversazione del 22 agosto 2026.

**Stato finale del processo corrente:** F0 chiusa; nessun processo F1/training avviato; esecuzione arrestata.
