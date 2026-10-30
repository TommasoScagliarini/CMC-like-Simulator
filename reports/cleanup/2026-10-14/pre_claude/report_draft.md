# Audit della codebase e proposta di pulizia

Data: 14 ottobre 2026, Europe/Rome. Instruction check token: CMC_AGENT_OK_2026.

## Problema e risultato

La repository accumula mesi di simulatori, training, validazioni, snapshot,
risultati e copie degli input. L'utente richiede prima la verifica del proprio
snapshot e poi una valutazione con Claude Opus 5.5 a effort massimo dei file
fondamentali e dei candidati alla rimozione.

La copia di lavoro può essere alleggerita soprattutto archiviando run e campagne
chiuse e deduplicando alcuni input. I sorgenti operativi top-level esaminati hanno
dipendenze reali. La pulizia dei prodotti tecnici vale circa 136 MB; una riduzione
di molti GB richiede selezione degli esperimenti e un archivio restaurabile.

**Il commit corrente è uno snapshot locale dei file tracciati; il backup completo
della cartella non è verificato e il commit non risulta pubblicato su GitHub.**

Questa fase produce analisi e inventario. Nessun file operativo è stato eliminato
o modificato, nessun modello/plugin/peso è stato cambiato e nessun training,
rollout o nuova simulazione è stato eseguito. Le 31 cancellazioni SNN preesistenti
sono documentate senza ripristinarle.

## Verifica dello snapshot

| Elemento | Esito verificato |
| --- | --- |
| Commit locale | `dba42388c32a441a5543f20881612a52781d7a99` |
| Tree del commit | `85c00158318071f244c4ad4dc2bd9197186cb26f` |
| Percorsi nel commit | 65.125 |
| Remoto GitHub | HEAD e unico branch `main` a `5782540579b138c59fa4e64fd221cf59a5d44c5e` |
| Differenza | Il commit locale è avanti di uno e non è presente fra i riferimenti remoti verificati |
| Cancellazioni già presenti | 31 file sotto `Trajectory Generator/Prosthesis_SNN/`, tutti presenti nel commit locale |
| Prova di recupero | README SNN estratto con `git show` in `/tmp`, 5.807 B, SHA-256 `392c6dbe41c210e769f6d74e14e92c645fef0b40dd1ebdeb5466d8f6e79ae118` |

Un file presente in quel commit può essere recuperato selettivamente, anche
dopo una futura cancellazione nel ramo di lavoro, finché il commit è conservato:

```text
git restore --source=dba42388c32a441a5543f20881612a52781d7a99 --worktree -- "percorso/del/file"
```

Il comando è un esempio, non è stato eseguito sulla repository. Ripristina la
copia di lavoro del solo percorso indicato. Un archivio Git/bundle del commit
consente inoltre di recuperarlo indipendentemente da futuri cambiamenti locali.

I file esclusi da `.gitignore` non sono automaticamente nel commit: checkpoint,
gran parte dei dati dei modelli, plugin compilati, risultati e plot. La run
best47 `MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01` ha **zero file
tracciati**. Il commento `.gitignore` che definisce i training output
«reproducible» non garantisce di ricostruire gli stessi pesi di un training
stocastico. Alcuni input ignorati hanno copie in archivi tracciati, ma manca una
copertura verificata globale.

Il nuovo commit introduce anche tre percorsi oltre 100 MiB senza Git LFS:

- `Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/boundary_replay_v2/control/ppo_audit/episodes_pre_gae.pkl`: 271.503.010 B;
- stessa directory di audit, ramo `corrected_boundary/ppo_audit/episodes_pre_gae.pkl`: 271.503.010 B, stesso blob;
- `Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/exploration/exploration_records.jsonl`: 180.033.948 B.

GitHub blocca file oltre 100 MiB: la pubblicazione ordinaria di questa storia
incontrerebbe il limite. Non abbiamo tentato un push e non attribuiamo a questo
limite un eventuale tentativo precedente senza il relativo log. La gestione dei
file grandi e dell'eventuale migrazione LFS deve conservare prima il commit
originale; una cancellazione in un nuovo commit lascia i blob nella storia da
pubblicare. [Documentazione ufficiale GitHub](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).

La protezione concreta prima delle rimozioni comprende quindi due componenti:
archivio Git del commit attuale e backup separato dei file ignorati necessari,
con manifest SHA-256 e una prova di estrazione. Conservare checkpoint completi,
config risolte e sorgenti importabili; registrare ambiente OpenSim/Python e ABI
dei plugin. Una copia degli stessi file su questo disco non costituisce una
protezione da guasto del disco.

## Inventario e dimensioni

Inventario completo dei file presenti, incluse directory ignorate e nascoste,
esclusa `.git`: **153.445 file, 44.770.244.523 B**. Sono byte logici, espressi
qui in GB decimali; lo spazio allocato sul filesystem può differire.

- 65.094 file presenti tracciati: **13,649 GB**;
- 88.351 file ignorati: **31,121 GB**;
- zero file non tracciati/non ignorati all'inizio dell'audit;
- ulteriori 31 file tracciati già cancellati, presenti nel commit.

| Macroarea | GB | Classificazione generale |
| --- | ---: | --- |
| `Trajectory Generator/` | 30,664 | Runtime, rami di ricerca, run e prove |
| ↳ `unified_MLP_39/` | 11,797 | Ramo sospeso, run e audit |
| ↳ `runs/` | 9,881 | Run V26 e storici, quasi tutti ignorati |
| ↳ `baseline_MLP/` | 4,814 | Runtime V26 e workspace scientifiche |
| ↳ `unified_MLP/` | 2,533 | Ramo storico 35D, run e bundle |
| ↳ `validation/` | 1,637 | Campagne recenti AB06/AB07, reset e TNT |
| `validation/` root | 6,627 | Test, profili, dataset e prove |
| `models/` | 5,830 | Modelli, dati grezzi e convertiti; quasi tutto ignorato |
| `results/` | 0,892 | Output ignorati, selezionabili dopo backup |
| `plot/` | 0,649 | Figure ignorate; `plotter.py` è codice operativo |
| `reports/` | 0,0076 | Storia, piani e backlog |

Le sottorighe di Trajectory Generator sono incluse nel suo totale. Il censimento
trova 1.732 file Python, circa 40,19 MB inclusi copie di snapshot, campagne e run;
i 73 Python top-level nei quattro perimetri Trajectory Generator occupano
2,17 MB. Eliminare copie esatte di sorgenti ha un beneficio modesto.

## File fondamentali

**Core del simulatore: mantenere tutti i 16 Python root.**

- Avvio e setup: `main.py`, `config.py`, `model_loader.py`, `path_resolver.py`,
  `setup_io.py`, `create_setup.py`.
- Simulazione: `kinematics_interpolator.py`, `outer_loop.py`,
  `inverse_dynamics.py`, `static_optimization.py`, `prosthesis_controller.py`,
  `simulation_runner.py`, `output.py`.
- Contatto/detector: `online_grf.py`, `binary_phase_detector.py`.
- Riproduzione: `visualize.py`.

Mantenere anche `plot/plotter.py` e `validation/validate_sim_results.py`, chiamati
da `main.py` via subprocess: un grafo dei soli import non li individua.

**Ambiente e V26:** mantenere i sette moduli condivisi sotto Trajectory Generator:
`osim_trj_cmc_like.py`, `prosthetic_phase_fsm.py`, `binary_phase_fsm.py`,
`binary_phase_fsm_v26.py`, `binary_phase_adapter.py`,
`binary_phase_adapter_v26.py`, `prescribed_action.py`. Le varianti V26 ereditano
le precedenti. Il nome o la data di un file non stabiliscono se sia inutilizzato.

La baseline comprende 30 Python: 19 di runtime/training/supporto, due utility
diagnostiche, tre compatibility primary-split, quattro adattamento/DAgger e due
utility actor-only. Mantenere certamente env/config/reward/module/trainer/eval,
governor/morphology, watchdog, logging, sampling, warm-start e supporto Windows.
I file opzionali sono candidati di migrazione, descritti più avanti.

Il constructor pickle best47 registra
`asymmetric_rl_module.AsymmetricActorCriticTorchRLModule`: conservare nome e
percorso importabile del modulo o qualificare esplicitamente una migrazione.
`rl_module_best/` serve all'inferenza; per riprendere PPO serve il checkpoint
Algorithm/Learner con stato optimizer/critic, non il solo export dei pesi actor.

**Plugin e preparazione:** mantenere i sorgenti/CMake dei due tools C++ SEA e
OnlineGRF; librerie runtime e corrispondenza sorgente-build-ABI; builder ABx,
fitting, registrazione, calibrazione, converter MATLAB/Python e
`scripts/run_opensim_sea_pipeline.py`. I converter recovery verificano il
sorgente originale: non sono copie intercambiabili. Il plugin PI corrente
comprende `torque_error_integral` e `Ki`, mentre AGENTS conserva una descrizione
PD storica; questo audit non cambia plugin o comando SEA.

**Asset:** mantenere modello/setup AB06 PI, bundle AB07/AB08 e variante pylon400
usata dalle verifiche, insieme a IK, ExternalLoads, GRF, actuator/task XML,
healthy, associazioni dei profili e manifest di preparazione. Conservare donor,
input grezzi originali, statici convertiti e template necessari alla ricostruzione.

`Geometry/AM_foot_l.STL`, `osseo_femur.STL`, `pors_tibia.STL`,
`transfemur_l.stl` sono referenziati. La suola sinistra serve anche alla
registrazione fisica.

**Dipendenze nascoste da preservare:**

- `models/SEASEA/data/CMC_Actuators.xml` e `CMC_Tasks - modified Kp_Kv.xml`
  restano default del builder, benché il nome della cartella sembri storico.
- Il profilo selezionato dentro
  `validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/`
  è un input di produzione.
- Il warm-start predefinito punta a un checkpoint dentro
  `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/`;
  il raw transplant conserva anche un default verso una run di giugno.
- Tutti i 30 Python baseline sono inclusi in manifest recenti dei reset e di AB07,
  anche quando non chiamati da uno specifico rollout.

## Prodotti tecnici candidati alla rimozione

| Candidato | Byte | Condizione |
| --- | ---: | --- |
| `__pycache__`, `.pytest_cache`, `.ruff_cache`, `.DS_Store` | 48.279.701 | Cache incidentali, 1.993 file ignorati |
| `opensim.log` root | 78.159.231 | Log incidentale; distinguere i log scientifici delle campagne |
| `/build/`, `/build_online_grf/`, build dei due tools C++ | 9.185.879 | Verificare che i binari runtime necessari siano conservati fuori dalla build |
| Cinque prodotti LaTeX `paper/main.*` generati | 98.046 | Conservare sorgenti, figure e PDF voluti |
| **Totale preliminare** | **135.722.857** | Circa 136 MB |

I prodotti LaTeX sono `main.aux`, `main.fdb_latexmk`, `main.fls`, `main.log`,
`main.synctex.gz`. Le build censite hanno 81 file tracciati: occorre controllare
eventuali manifest prima di ritirarli e aggiungere ignore ancorati alle precise
directory root. Una regola generica per qualsiasi `build/` colpirebbe workspace
AB07 che contengono modelli e dati di evidenza.

## Codice e asset opzionali da ritirare con migrazione

| Perimetro | Valutazione |
| --- | --- |
| `Trajectory Generator/commands.txt` | Interamente SNN, oggi obsoleto: sostituibile/ritirabile |
| SNN già cancellata | Nessun import Python runtime vivo rilevato; conservare la storia Git e verificare i riferimenti documentali |
| `unified_MLP/` 35D | Ramo coerente da archiviare; un suo `feasibility39_bundle` è usato dai test attuali 39D |
| `unified_MLP_39/` | Sospeso; archiviabile come ramo se si decide di dismetterne la superficie operativa, mantenendo checkpoint e sorgenti per reload |
| `primary_split_input_adapter.py`, `primary_split_v25_residual.py`, `primary_grf_split_adaptation.py` | Compatibility non necessaria al best47 standard; ancora CLI/classi/campagne e pin storici |
| `freeze_actor_checkpoint.py`, `project_actor_update.py` | Utility opzionali; il secondo ha test importatori e entrambi sono pin recenti |
| `target_domain_*` | Da mantenere disponibili per la direzione student/DAgger; un eventuale ritiro richiede chiudere i consumer V26C |
| Tool di tuning motor/driver/cascade | Archiviabili se non si mantiene il workflow di tuning; sono entrypoint manuali |
| `scripts/evaluate_cmc_like_candidates.py`, `run_measured_grf_window_tests.py` | Ramo diagnostico opzionale; il secondo importa helper dal primo e dalla pipeline generale |
| Quattro mesh senza riferimento attivo rilevato | `AM_foot_r.stl`, `VR_Foot_left.STL`, `VR_foot_right.STL`, `transfemur_r.stl`, circa 1,61 MiB; controllare asset storici/uso manuale |
| Vecchi modelli SEASEA/whealthy/AB06 adjusted | Archiviazione selettiva insieme ai setup; preservare support XML e default ancora attivi |

I tool opzionali sono `motor_driver_pole_map.py`, `motor_driver_pole_locus.py`,
`driver_isolation_suite.py`, `driver_isolation_suite_pi.py`,
`cascade_outer_metrics.py` sotto `tools/`.

Non eliminare `unified_MLP_39/env_v1.py`: è la superclass importata dal runtime
corrente. Non eliminare optimizer/learner resume o KL audit per il solo nome:
servono alla fedeltà dei checkpoint. L'assenza di caller statici non prova
l'assenza di CLI, uso manuale, subprocess o dipendenze cloudpickle.

## Duplicati e archivi

Il confronto SHA-256 completo fra `models/AB06-raw/` e
`models/AB06_SEASEA-raw/` trova **1.719 file identici, circa 1,325 GB**. Mantenere
una copia originale e i trasformati/template esclusivi è un candidato concreto.
La seconda directory contiene anche dati necessari: non eliminarla interamente.
Questi file sono ignorati da Git e necessitano backup separato.

Alias identici da canonizzare dopo verifica dei riferimenti:

- modello `_pi-tuned.osim` e `_pi.osim` AB06, con setup distinti;
- `data/IK_rra_window.mot` e `IK_results_AB06_SEASEA.mot`, 10.701.447 B;
- `data/treadmill_01_01_grf.mot` e `AB06_SEASEA_GRF_FullSpan.mot`, 37.040.307 B;
- profili `grf_correct_magnitude` e `online_full_wrench_residual_tangent_v2`,
  entrambi ancora referenziati.

L'indice Git identifica 50 gruppi di blob oltre 1 MB con **2,695 GB** di copie
teoricamente ridondanti nel working tree, soprattutto input AB07 replicati in
studi distinti. Git conserva già un solo blob identico: lo spazio del remoto non
diminuirebbe nella stessa misura. La riduzione richiede un nuovo contratto di
archivio con mappa dei percorsi e ripristino, senza alterare le prove originali.
I duplicati Python esatti sono soltanto circa 102 KB e perlopiù snapshot deliberati.

L'archiviazione dei run può avere il maggiore beneficio, ma deve selezionare:
checkpoint correnti e parent, checkpoint completi per resume, milestone citate,
config risolte, log/summary/journal e dati necessari a rigenerare le figure.
`historical_runs.manual.yaml` contiene annotazioni manuali: conservarlo con gli
indici. Selezionare i plot dopo preservare dati e plotter; la rigenerazione può
richiedere il runtime e l'ambiente originale.

V26C J0–J21 e `pipeline_bundle_v26c_2026-08-28` sono archivi sigillati per percorso
e SHA-256. Lo store `objects/` è intenzionale; gli originali rimangono ancore
per import, pickle e pin. Un move, forwarder o symlink può rompere la verifica,
specialmente su Windows. L'archivio storico deve poter ricostruire i percorsi
originali in un checkout dedicato. Una futura versione operativa può avere una
closure più stretta e nuovi manifest, conservando separatamente l'esperimento
originale.

Le campagne ottobre AB07/AB08, reset e TNT sono evidenze correnti con dipendenze
fra runner datati: nessuna rimozione generale di `validation/`. I report pesano
7,60 MB; i 276 TODO del daily più recente sono letteralmente distinti e spesso
condividono il tema. Un indice con ID stabili alleggerirebbe il lavoro quotidiano
preservando report, propagazione e storico. Manoscritto e sorgenti `paper/`
rimangono materiale attivo; gli appunti di ricerca non sono prodotti generati.

## Strategia operativa proposta

1. Completare la protezione del commit e dei file ignorati, verificando un restore
   in un percorso separato e mantenendo il commit originale anche se si modifica
   la storia da pubblicare.
2. Rimuovere prodotti tecnici incidentali e correggere ignore precisi; per sole
   cache bastano controlli del perimetro, senza simulazioni.
3. Consolidare i comandi correnti e l'indice degli entrypoint; aggiornare
   documentazione SNN/PD storica nel contesto operativo.
4. Separare il checkout operativo dagli archivi di esperimenti con manifest e
   restore verificato, mantenendo i sorgenti/import path dei checkpoint attivi.
5. Canonizzare duplicati e ritirare i rami opzionali scelti, aggiornando consumer,
   test e configurazioni future senza riscrivere la storia scientifica.
6. Per le dipendenze cambiate verificare CLI/import, reload del checkpoint in un
   processo fresco, reset/step e rollout V26 breve equivalente, pipeline ABx,
   plotter/visualizer e percorso Windows. Uno smoke non certifica robustezza o
   generalizzazione scientifica.

Questa sequenza è una proposta da rendere esecutiva nella fase di pulizia. Una
riduzione dei sorgenti duplicati tramite libreria comune sarebbe un refactor
funzionale separato: richiede equivalenza di training, restore e contratti.

## Metodo, evidenze e limiti

Censimento filesystem/Git completo, confronto reale dei riferimenti remoti,
lettura di codice/config/README/XML, parsing AST, analisi sicura dei constructor
pickle con `pickletools`, ricerca di path dinamici e manifest, SHA-256 mirati e
prova di estrazione Git non distruttiva. Analisi parallele su core, reti e
artefatti; Claude indipendente prima del confronto con Codex.

Copertura di tutte le macroaree, con verifiche semantiche mirate dei workflow
operativi e classificazione degli artefatti per famiglia. L'inventario completo
non equivale a una lettura semantica di ogni file o a una prova di equivalenza
di tutti i percorsi. I candidati opzionali restano condizionati alla scelta delle
capacità da mantenere e alla chiusura dei consumer/manuali/manifest.

I deliverable aggiunti da questa fase sono esclusivamente report e dati
dell'audit. Tutti i sorgenti operativi rimangono quelli del commit iniziale;
la verifica Git finale distinguerà le 31 cancellazioni preesistenti dagli
artefatti documentali aggiunti.

<!-- CLAUDE_RECONCILIATION -->
