# Analisi RIGHT39 dopo l'iterazione 60

Analisi richiesta dall'utente dopo la continuazione autorizzata 40–60. Nessun
nuovo training, nessuna modifica a produzione, FSM o checkpoint.

## Parti dell'analisi

- `training_history/analyze.py`: storico, contatori di terminazione, medie
  pesate con somme dei callback, confronti con LEFT e V26 a pari esperienza,
  audit della ripresa e degli input congelati. Risultati in `metrics.json` e
  `findings.md` nella stessa sottocartella.
- `run_evaluations.py`: due rollout deterministici dai milestone immutabili
  58 e 60, start e configurazione incorporati negli export. Comandi e stato
  nei file `rollout_launch.json`, `rollout_execution.json`, `rollout_status.json`.
- `analyze_rollouts.py --render`: confronto cinematico, reward, coppie SEA,
  contatto, FSM, lag descrittivo e distribuzione delle azioni. Riusa i helper
  verificati per RIGHT40; i vecchi export 35/40 sono risolti verso i milestone
  immutabili confrontando gli hash originali, senza riscrivere gli archivi.
- `noise_screening/`: cinque seed gaussiani 123–127 sul best58, rumore originale
  e seed ambiente 123. Confronto con RIGHT35, RIGHT40 e V26 archiviati. Ogni
  policy mantiene la propria deviazione standard appresa. Al massimo quattro
  simulazioni totali, incluse quelle deterministiche; un thread numerico per
  processo. Stato, comandi, analisi e provenienza sono nella sottocartella.

I manifest della simulazione (`rollout_input_hashes.json`) e dell'analisi
(`analysis_input_hashes.json`) sono distinti. Il completamento di ogni fase
va verificato dai rispettivi file di stato, non dalla sola presenza dei log.

## Figure

I sette grafici canonici per ogni nuovo rollout sono nella root del repository:

- `plot/09_23_2026_39D_right_best58/`
- `plot/09_23_2026_39D_right_last60/`

Le figure di confronto di training, cinematica e rumore sono in
`plot/09_23_2026_right60_comparison/`.

## Limiti

I contatori di training si riferiscono a episodi che possono attraversare più
aggiornamenti della policy. Le iterazioni senza episodi conclusi non ricevono
un ritorno artificiale: le proiezioni della console non sono usate come esiti.

Il confronto stocastico ha cinque seed e uno start fisso; non dimostra
equivalenza statistica o robustezza generale. Il V26 stocastico è il best87
storico valutato nel contenitore attuale con FSM v3. Il rollout V26
deterministico archiviato è un riferimento di qualità più maturo, mentre il
confronto degli storici usa transizioni comparabili. Eventuali guasti fatali
dei worker storici possono mancare dai contatori V26.

Completamento, cicli FSM, precisione cinematica e qualità delle coppie restano
misure separate. La penetrazione viene distinta fra campioni reward a 100 Hz
e registrazione fisica STO a 1 kHz; le coppie SEA non sono il momento GRF.
