# Rimozione dei rollout sperimentali — 2026-10-19

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../2026-10-19_baseline_validation_removal/README.md).

Su richiesta dell’utente è stata eliminata l’intera cartella
`Trajectory Generator/runs/rollout/validation/`: **4.145 file,
5.295.188.127 byte**, comprendenti tutte le campagne presenti e i
35 moduli di policy sperimentali. Non sono stati spostati altrove pesi,
dataset, file STO o tracce complete della policy.

## Documentazione conservata

Il solo [archivio documentale](historical_notes_and_summaries.zip) contiene 335
file originali: 41 note Markdown, 146 riepiloghi dei rollout e
148 riepiloghi del supervisore con i comandi e gli esiti registrati.
Occupa 758,339 byte; i contenuti sono identici agli originali.
La [mappa dei documenti archiviati](archived_documents.json) riporta percorsi
originali, dimensioni e hash. Le vecchie destinazioni riportate nei
documenti sono riferimenti storici e possono essere state rimosse.

Il [manifest completo della rimozione](removed_files.json) registra anche
i file non conservati. La [verifica finale](verification.json) riporta
il confronto di integrità e gli altri controlli.

## Effetto sugli strumenti conservati

La simulazione CMC-like ordinaria e i normali comandi baseline di training
e rollout non richiedono questi risultati. La policy V26C best47, il suo
training, i checkpoint H0 e i modelli/plugin sono fuori dalla rimozione.

Le campagne storiche F0, F1, F2R e V26B si leggevano reciprocamente trace,
dataset, riepiloghi e moduli. Quegli input ora mancano. Anche l’audit J0
e il materializzatore del bundle facevano riferimento ai pesi sperimentali
`V1_35D_transplant` e `B0_35D_MASKED`, anch’essi rimossi. Non sono stati
inseriti sostituti, bypass dei controlli o percorsi verso policy diverse.

Codice, protocolli, manifest e hash dei vecchi esperimenti restano nello
stato originale: documentano la procedura storica e non implicano che
sia ancora eseguibile con tutti gli input presenti. Per una replica dei
vecchi risultati occorre ripristinare i dati dal backup o ricostruirli.

## Procedimento per un nuovo training

La [guida V26→V26C](../../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md)
e gli strumenti in `Trajectory Generator/baseline_MLP/validation/` sono
conservati. Con una nuova rete imitativa occorrono nuovi rollout, dataset
e validazioni; i runner storici con percorsi e hash fissati devono essere
adattati in una nuova workspace, mantenendo gli originali come evidenza.

## Campagne rimosse

- `2026-07-23_h0_postrouting_frozen_ablation/`: 75 file, 10,035,050 byte.
- `actor_fragility_runs/`: 4 file, 4,214,877 byte.
- `controller_memory_ablation/`: 12 file, 30,375,712 byte.
- `event_anchored_morphology_runs/`: 8 file, 20,482,939 byte.
- `experimental_morphology_runs/`: 4 file, 4,426 byte.
- `exploration_noise_runs/`: 0 file, 0 byte.
- `f0_freeze_runs/`: 2015 file, 2,582,369,950 byte.
- `f1_ablation_runs/`: 205 file, 262,010,506 byte.
- `f2r_bridge_runs/`: 231 file, 219,220,745 byte.
- `heel_detector_validation_runs/`: 0 file, 0 byte.
- `ppo_pilot_screens/`: 35 file, 4,126,598 byte.
- `recovery_gate_runs/`: 13 file, 53,399 byte.
- `robust_reference_runs/`: 12 file, 47,640 byte.
- `v26b_bridge_runs/`: 1527 file, 2,152,113,275 byte.
- `warm_start_h2_runs/`: 4 file, 10,133,010 byte.
- `warm_start_rollout_runs/`: 0 file, 0 byte.
