# Avvio prosecuzione LEFT best80, peso penetrazione2, fino100

## Richiesta e strategia
Su richiesta esplicita dell’utente avviata la prosecuzione dal best del ramo con peso penetrazione2 fino all’iterazione100. La sorgente è checkpoint_best all’iterazione80, non il last82. Ripristino completo actor, critic, Adam e stato PPO; venti nuovi aggiornamenti81–100, batch4096,81.920 nuove transizioni previste. Nessuna modifica dell’obiettivo o dei controllori.

## Avvio
24 settembre2026,17:23:31 CEST. Supervisor PID17160, processo training17162; caffeinate17161. Controllata assenza di altri training/rollout prima dell’avvio. All’audit iniziale risulta in corso algo.train iteration81. Termine previsto orientativamente dopo circa3 ore, da aggiornare con i tempi effettivi.

## Configurazione
FSMv3, riferimento prescribed LEFT guidato dalla fase LEFT senza shift, prescribed e clock visibili100%, scala velocità1, slew disattivato, governor4Hz, start unico offset1s, episodio5s. Stessa reward del checkpoint: penetrazione2 e momento ankle0.1. PPO batch4096, minibatch512,10epoche, LR1e-4,13worker. Nessun override reward, nessun warmup aggiuntivo. Checkpoint ogni iterazione, milestone conservate; target logico100, max_consecutive_skips1. Telemetria degli episodi attiva. Nessun rollout automatico al termine.

## Verifiche
Config confrontata semanticamente con lo snapshot interno a checkpoint_best: invariata salvo limite iterazioni e metadati/output/resume. Parser e validazioni superati. Actor, medie actor e critic coincidono esattamente con la sorgente. Adam ripristinato esattamente con12 stati,step4455. KL adattivo ripristinato a0.30000001192092896. Reward_changed=false. Verificati invariati gli hash dei28 file del checkpoint sorgente dopo l’avvio. Nessuna modifica al codice di produzione.

## File e artefatti
- Config: `Trajectory Generator/unified_MLP_39/configs/left80_penetration_weight2_to100_2026-09-24.yaml`.
- Piano, launcher,preflight,hash sorgente,baseline80,launch.json,startup_audit.json e training.log: `Trajectory Generator/unified_MLP_39/runs/plans/left80_penetration_weight2_to100_2026-09-24/`.
- Nuovo run: `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT80_penetration_weight2_to100_2026-09-24/`.
- Source: `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT72_penetration_weight2_10iter_2026-09-24/checkpoint_best`.

## TODO
Verificare conclusione a100 con20 aggiornamenti e81.920 nuove transizioni, analizzare robustezza/coppie/cinematica e selezionare il checkpoint. Resta aperto lo screening rumoroso di best80. La prosecuzione non dimostra che i difetti dinamici si correggeranno automaticamente. Il ramo di controllo peso1 con codice terminale corretto non è stato avviato.
