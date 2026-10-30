# Rimozione delle analisi storiche — 2026-10-19

Su richiesta dell'utente è stata eliminata
`Trajectory Generator/runs/analysis/`: **35 file, 6.184.358 byte**, costituiti
da 20 tabelle CSV e 15 immagini PNG.

Questa pagina registra la rimozione; non contiene le tabelle o le immagini
originali. I riferimenti nei report sono ora indicati come percorsi storici
di materiali rimossi. I risultati descritti nei report restano riferiti agli
esperimenti di giugno 2026.

La verifica delle dipendenze non ha trovato utilizzi nel codice operativo o
nelle configurazioni. L'inventario con dimensioni e hash originali è nel
[manifest dei file rimossi](removed_files.json); i controlli conclusivi sono
nella [verifica di integrità](verification.json).

## Analisi cinematica dei pesi reward

Percorso originale:
`Trajectory Generator/runs/analysis/kinematic_weight_diagnostics_2026-06-21/`.

Rimossi 14 file: confronti tra cinematica desiderata, riferimento fornito al
controllore e movimento ottenuto, con diagnostica dei pesi della reward.

## Oscillazioni delle velocità e clipping

Percorso originale:
`Trajectory Generator/runs/analysis/qdot_chattering_vs_clipping_2026-06-21/`.

Rimossi 7 file: metriche delle oscillazioni rapide e della loro relazione
con la saturazione delle azioni della policy.

## GRF e tracking del ginocchio in appoggio

Percorso originale:
`Trajectory Generator/runs/analysis/grf_knee_stance_diagnostics_2026-06-25/`.

Rimossi 13 file: confronti tra tracking del ginocchio, penalità GRF, coppie
alla caviglia e contributo delle reserve.

## Prova con pesi delle penalità GRF dimezzati

Percorso originale:
`Trajectory Generator/runs/analysis/grfhalf_smoke_2026-06-25/`.

Rimossa la tabella `smoke_comparison_metrics.csv`, che confrontava lo smoke
training con i rollout degli esperimenti precedenti.
