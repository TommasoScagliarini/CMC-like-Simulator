# Valutazione indipendente Codex — 14 ottobre 2026

Analisi precedente alla lettura della risposta Claude. Nessuna cancellazione,
modifica ai sorgenti operativi, esecuzione OpenSim o training. Inventario completo
del filesystem esclusa .git; lettura dei sorgenti e dei contratti mirata per tutte
le macroaree, non una lettura semantica individuale di 153.445 file.

## Protezione attuale

HEAD locale dba42388c32a441a5543f20881612a52781d7a99, tree
85c00158318071f244c4ad4dc2bd9197186cb26f. Commit e tree sono uno snapshot Git
dei 65.125 percorsi tracciati. Il working tree ha già 31 cancellazioni SNN;
ogni blob esiste nel commit. README SNN estratto con git show in /tmp e hashato:
392c6dbe41c210e769f6d74e14e92c645fef0b40dd1ebdeb5466d8f6e79ae118, 5.807 B.

git ls-remote origin verificato in rete: esistono solo HEAD/main e puntano a
5782540579b138c59fa4e64fd221cf59a5d44c5e. Il nuovo commit è solo locale.
Tre nuovi percorsi del commit superano 100 MiB senza attributo LFS: due copie
episodes_pre_gae.pkl da 271.503.010 B e exploration_records.jsonl da180.033.948 B.
GitHub blocca file oltre100MiB: il push normale di questa storia incontrerebbe
il limite. Non è stato tentato un push e non è provato cosa sia accaduto in un
eventuale tentativo precedente dell'utente. Fonte primaria:
https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github

44.770.244.523 B logici presenti esclusa .git: 65.094 tracciati =13.649.208.129 B;
88.351 ignorati =31.121.036.394 B; zero untracked non ignorati prima dell'audit.
Non sono garantiti dal commit: dati models/**/data e *-raw, plugin, checkpoint
best47 e altri run, risultati e plot. Esistono copie parziali di alcuni input
in archivi tracciati, senza copertura globale dei file ignorati. Protezione prima
della pulizia: conservare il commit in bundle/tag di backup e copiare con manifest
hash i dati ignorati necessari in archivio esterno; verificare estrazione e
caricamento del checkpoint. Tag da solo non include ignored e non è remoto.

## File fondamentali

Tutti i16 .py root risultano attivi o entrypoint: main, config, model_loader,
kinematics_interpolator, outer_loop, inverse_dynamics, static_optimization,
prosthesis_controller, output, simulation_runner, online_grf,
binary_phase_detector, path_resolver, setup_io, create_setup, visualize.
plot/plotter.py e validation/validate_sim_results.py sono chiamati via subprocess.

Mantenere i7 .py condivisi Trajectory Generator e il codice baseline_MLP V26,
le configurazioni operative, teacher/DAgger, critic/warm-start/restore e strumenti
di raccolta/diagnostica necessari. Non cancellare helper dal nome legacy:
class_and_ctor_args.pkl best47 conserva il modulo flat asymmetric_rl_module e
la classe AsymmetricActorCriticTorchRLModule; file non importati staticamente
possono essere indispensabili a reload o workflow manuali.

Mantenere sorgenti C++ e CMake dei tools SEA e OnlineGRF, binari runtime correnti,
builder ABx, fitting/registrazione/calibrazione, conversione MATLAB/Python,
scripts/run_opensim_sea_pipeline.py. AGENTS descrive il PD storico: il plugin
attuale include PI/torque_error_integral/Ki. Nessuna modifica semantica richiesta.

Mantenere bundle AB06 PI, AB07/AB08 e variante AB07 pylon400 con XML, IK, GRF,
healthy, modelli donor, template e associazioni dei profili; sorgenti comuni
models/SEASEA/data/CMC_Actuators.xml e CMC_Tasks restano default del builder.
Mantenere Geometry AM_foot_l.STL, osseo_femur.STL, pors_tibia.STL, transfemur_l.stl.

Dipendenza produzione dentro validation: profilo V25 selezionato in
validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json.
Default warm-start baseline dipende da validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last;
warm_start.py ha inoltre un parent June23 nel proprio default.

## Candidati a rimozione tecnica

Cache Python/pytest/ruff/.DS_Store:1.993 file48.279.701 B; opensim.log root
78.159.231 B; build root+build_online_grf+build dei2tools9.185.879 B;
paper/{main.aux,main.fdb_latexmk,main.fls,main.log,main.synctex.gz}98.046 B.
Totale preliminare135.722.857 B. Verificare binari runtime fuori build e manifest
dei prodotti tracciati prima di una rimozione. Non estendere pattern build a
directory dentro campagne, che contengono modelli e prove.

## Candidati condizionati per semplificazione sorgenti

SNN già cancellata localmente: nessun import runtime vivo rilevato, rimangono
comandi/documenti e possibili dipendenze scientifiche storiche. Ritirare o
riscrivere Trajectory Generator/commands.txt, oggi interamente SNN.
Archiviabili diagnostici tuning tools/{motor_driver_pole_map.py,
motor_driver_pole_locus.py,driver_isolation_suite.py,driver_isolation_suite_pi.py,
cascade_outer_metrics.py} e scripts/{evaluate_cmc_like_candidates.py,
run_measured_grf_window_tests.py}: entrypoint manuali, utilità future da decidere.
unified_MLP e unified_MLP_39 sono rami opzionali/storici, non prova di codice morto:
conservarli se interessa riusare checkpoint/benchmark. env_v1.py è ancora importato
da unified_MLP_39/env.py. Archiviazione del ramo può alleggerire la cartella
operativa dopo closure verificata, senza fingere che i risultati non servano.

4 mesh con zero riferimenti nei modelli e config attivi ispezionati:
Geometry/{AM_foot_r.stl,VR_Foot_left.STL,VR_foot_right.STL,transfemur_r.stl},
circa1,61MiB. Candidate dopo verifica di modelli storici e uso manuale.
Vecchi modelli SEASEA/whealthy e AB06_adjuted archiviabili selettivamente:
non cancellare support/data usato dalla pipeline.

## Duplicati

1.719 file AB06-raw vs AB06_SEASEA-raw identici SHA256, circa1,325GB;
conservare originali e derivati/template esclusivi. Tutti ignorati Git.
Alias modello AB06_pi-tuned identico a pi; IK_rra_window identico
IK_results_AB06_SEASEA (10.701.447B); treadmill_grf identico GRF_FullSpan
(37.040.307B);2 profili GRF identici ma entrambi referenziati. Nessun rm isolato.
50 gruppi blob Git>1MB con2.694.924.815B ridondanti teorici in working tree,
soprattutto input AB07 e prove. Git già deduplica i blob: beneficio non uguale
allo spazio remoto. Duplicati Python esatti solo102KB e quasi tutti snapshot.

## Archivi e documenti

TG/runs9,881GB, unified35 2,533GB, unified39 11,797GB: gran parte run ignorati.
Rootvalidation6,627GB, baseline4,814GB, TGvalidation1,637GB: prove e codice.
Archiviare per campagne/lineage dopo estrazione ripetibile e definizione del
perimetro attivo. V26C J0-J21 e pipeline_bundle sono sigillati per path+hash;
non riorganizzare runner/object store automaticamente. Originali servono da
ancore import/pickle/pin; object store247MiB è intenzionale.

reports7,60MB da conservare;276 TODO nel latestdaily sono distinti letteralmente
ma sovrapposti per tema. Nuovo indice operativo con ID evita riletture massive
senza riscrivere i report. Paper sorgenti/manoscritto attivi da conservare.

## Ordine proposto

1. Chiudere copertura snapshot e backup ignored, provando restore non distruttivo.
2. Pulizia cache/build/log, ignore precisi; nessun test simulator per sola cache.
3. Consolidare documentazione e ritirare entrypoint obsolete/SNN.
4. Separare working tree operativo dagli archivi congelati mediante manifest
   e restore alle posizioni originali; conservare closure dei checkpoint.
5. Deduplicare dati/modelli/profili dopo migrazione esplicita dei riferimenti.
6. Verifiche proporzionate: import statici, reload checkpoint, reset/step,
   piccolo rolloutV26, pipelineABx, plotter/visualizer e Windows live per i
   percorsi cambiati. Non promuovere robustezza scientifica da uno smoke.

Conclusione: cancellare genericamente validation, runs, build o file senza
import è inappropriato; produzione e prove sono mescolate. Per leggibilità
servono confini chiari e archivi restaurabili. Per disco il beneficio viene
dai grandi run e dati, non dai sorgenti, che pesano40,19MB nel censimento globale.
