# Analisi del training 39D RIGHT a 40 iterazioni

Richiesta: analizzare il training appena concluso, confrontandolo con 39D LEFT
e V26 del 20 agosto. Nessun aggiornamento dei pesi o nuovo training.

## Perimetro

- Run: `MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23`.
- Due nuove valutazioni deterministiche: export best35 e last40.
- Start fisico 12,99 s; orizzonte 5 s; configurazione recuperata dagli export.
- Riferimento RIGHT con shift ginocchio 0,465 e caviglia 0,452; clock RIGHT
  non sfasato; sei ingressi visibili; FSM v3.
- Confronti cinematici archiviati: LEFTlast40, LEFTbest72 e V26best87.
- Confronto training a transizioni equivalenti, con contatori di fine
  episodio differenziati e gestione delle discontinuità nelle riprese.

## Riproduzione

`analyze_rollouts.py --render` legge rollout e checkpoint, ricostruisce la
reward, verifica input RIGHT e riproduzione delle azioni, calcola le metriche
e genera grafici. Non avvia simulazioni. Le ricevute canoniche esistenti sono
riutilizzate dopo verifica dell'hash della traccia.

I comandi esatti delle due simulazioni sono in `rollout_launch.json`; i codici
di uscita sono in `rollout_execution.json`. Gli artefatti originali restano
immutati. Il codice di produzione non è modificato da questa analisi.

`training_integrity.json` contiene l'audit delle 40 iterazioni e degli export;
`source_verification.json` verifica i 48 input congelati prima del training.
`training_history/` contiene il confronto delle curve e i controlli delle
configurazioni. `rollout_metrics.json` conserva tutte le metriche deterministiche.

## Convenzioni

- RMSE in gradi rispetto al proprio target, mantenendo separati errore della
  reference servita ed errore di inseguimento del giunto.
- Coppia della molla SEA con segno canonico: ginocchio invertito, caviglia
  invariata; nessuna confusione fra coppia SEA e momento GRF.
- Chattering: RMS dopo filtro Butterworth ordine 4 passa-alto a 20 Hz,
  applicato in entrambe le direzioni; confronto anche dopo i primi 200 ms.
  È una misura del contenuto ad alta frequenza, non una diagnosi clinica.
- Penetrazione distinta fra campioni della reward (10 ms) e STO (1 ms).
- Lag stimato sulla stessa finestra 13,30–17,75 s e griglia ±200 ms: semplice
  descrizione del migliore allineamento, non stima causale del ritardo del filtro.
- Std e probabilità di clipping valutate sugli stati deterministici visitati
  da ciascuna policy. Non sostituiscono rollout con rumore controllato.

## Limiti

Un solo seed di training e un solo start di evaluation. LEFTbest72 e V26best87
dispongono di più esperienza dei checkpoint RIGHT valutati. Il ritorno che
seleziona il best deriva dalla raccolta esplorativa, mentre l'export è salvato
dopo l'aggiornamento PPO; non è una valutazione deterministica dell'export.

Il cambio RIGHT/LEFT comprende traiettoria, clock, velocità e normalizzazione
derivata dal target. Non separa l'effetto di ciascuno. LEFT ha inoltre riprese
con riavvio degli episodi fisici; il confronto prima della 25 aiuta a delimitare
questo fattore. I vecchi guasti V26 possono mancare dai contatori storici.

I sette plot canonici di ciascun nuovo rollout sono salvati in root `plot/`,
nelle cartelle `09_23_2026_39D_right_best35` e `09_23_2026_39D_right_last40`.
Le figure di confronto sono in `plot/09_23_2026_right_reference_comparison/`.
