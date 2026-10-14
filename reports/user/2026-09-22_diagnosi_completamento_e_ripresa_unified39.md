# Diagnosi del completamento e ripresa della 39D — 22 settembre 2026

> **Aggiornamento ore 08:14 CEST:** la prima sequenza si è fermata alle 02:17 dopo l'iterazione 42 per un errore del logger: un `numpy.bool_` nel campo `target_visible` non era serializzabile in JSON. I checkpoint e le statistiche aggregate 41–42 sono integri; manca invece la telemetria dettagliata di quegli episodi. Corretto il campo con la conversione già usata dagli altri dati diagnostici; superati 15 test, 11 controlli dell'orchestratore, una prova con un worker Ray reale e il ripristino offline del checkpoint 42. La sequenza è ripartita alle **08:13**, dal checkpoint 42, con Adam a 1377 e KL conservato. È in corso l'iterazione 43. Il nuovo stato operativo è in `Trajectory Generator/unified_MLP_39/runs/plans/completion_recovery42_2026-09-22/status.json` (PID 39640). Le indicazioni di avvio più sotto descrivono il primo tentativo.

Alla 42: return medio **227,57**, episodi completi **8/12 (66,7%)**. Nelle ultime cinque iterazioni 38–42: **17/45 (37,8%)**; il singolo risultato della 42 non dimostra un miglioramento stabile. La configurazione fisica e PPO resta invariata; rimangono previste le tappe 60/80/100 e analisi separate dal training.

Obiettivo: indagare il 38,8% di episodi completi (19/49 nelle iterazioni 36–40) prima della prosecuzione fino a 100.
La diagnosi usa il checkpoint 40 fisso, start invariato, prescribed/clock visibili, FSM v3 e identica dinamica.

## Diagnosi del rumore

| Scala della deviazione standard | Episodi completi | Passi medi | Return medio |
|---|---:|---:|---:|
|0|1/1|500.0|222.15|
|0.5|5/5|500.0|284.20|
|1|5/5|500.0|231.84|

Cinque semi per livello stocastico; gli stessi campioni gaussiani sono usati a scala 1 e 0,5, con identiche osservazioni iniziali. La scala modifica solo la valutazione. Questo è un controllo esplorativo, non una prova di equivalenza statistica né una stima precisa della probabilità di successo.
Il rollout deterministico riproduce esattamente tutti i 500 record del precedente rollout 40. Il dato di training include policy che cambiano e alcuni episodi che attraversano aggiornamenti: non è direttamente equivalente ai cinque episodi di una policy fissa.

La diagnosi non riproduce il basso completamento osservato nelle iterazioni 36–40: anche con il rumore originale, tutti e cinque gli episodi del checkpoint 40 arrivano alla fine. Questo rende insufficiente una spiegazione basata soltanto sull'ampiezza del rumore. Restano da distinguere la qualità delle policy precedenti alla 40, i cambi di policy durante gli episodi e la variabilità dei campioni. La nuova telemetria registra questi cambi, senza attribuire loro automaticamente un ruolo causale.

Il return medio è 231,84 con rumore pieno e 284,20 con rumore dimezzato; il deterministico vale 222,15. Quindi la qualità varia anche quando tutti gli episodi sono completi. Con cinque semi non si dimostra equivalenza né si giustifica da solo un cambiamento dell'esplorazione durante il training. Inoltre, ridurre sigma modifica anche l'effetto medio del clipping delle azioni. La ricetta di training rimane quella concordata.

## Correzioni della ripresa

È stato dimostrato un difetto nel ripristino Adam: step 2025 al checkpoint 25, poi 81 a 26 e 1215 a 40. RLlib aveva ripristinato i pesi, lasciando i momenti dell’ottimizzatore scollegati dai parametri. Il difetto può influire sulle iterazioni 26–40; non spiega da solo il divario già presente prima della 25. La verifica precedente dei pesi non aveva rilevato questo problema.
La nuova ripresa conserva i momenti effettivamente salvati a 40, verifica associazione e digest, e non modifica i pesi. Non ricostruisce una storia ipotetica: a 41 il contatore Adam atteso è 1296.
Anche il coefficiente KL adattivo viene ora salvato esplicitamente in un sidecar legato all’hash del checkpoint. Per il vecchio checkpoint 40 è verificato il valore storico 0,20000000298023224. Le riprese 60/80 useranno il sidecar. Learning rate ed entropy rimangono costanti e verificati.
Gli RNG globali e gli episodi OpenSim in corso non sono recuperabili dai checkpoint storici; la prosecuzione non coincide bit per bit con un processo mai interrotto.

## Telemetria e risorse

La registrazione osserva soltanto dati già prodotti dal training: esito per episodio, riepiloghi e ultimi 50 passi. La memoria è limitata e i file sono separati per worker/processo. Nessuna inferenza aggiuntiva o estrazione casuale.
Il confronto fisico A/B con la stessa sequenza stocastica ha superato il controllo di identità delle transizioni. Il sink ha richiesto 46,08 ms su 542,76 s di simulazione (0,00849%); il benchmark con MetricsLogger RLlib reale misura circa 57,78 microsecondi CPU aggiuntivi per passo. I costi misurati e i limiti del benchmark sono riportati nelle ricevute physical_ab_verification.json e real_metrics_benchmark.json. Questo misura un costo piccolo, senza garantire assenza assoluta di rallentamenti su un sistema condiviso.

## Strategia di esecuzione

Prosecuzione 40→60→80→100 con la ricetta concordata invariata. Il training viene chiuso prima di ogni valutazione: un rollout deterministico e dieci stocastici, più i sette grafici canonici salvati in plot/. Durante il training restano soltanto telemetria e controlli leggeri. Il processo si ferma se falliscono i controlli di ripristino, integrità dello storico, batch 4096, prescribed visibile o copertura della telemetria.
Stato dell’esecuzione: `Trajectory Generator/unified_MLP_39/runs/plans/completion_continuation_2026-09-22/status.json`. Questa verifica preliminare non avvia direttamente il training e non ne dichiara il completamento.

## File e verifiche

- `unified_MLP_39/train_ppo_mlp.py`, `optimizer_resume.py`, `learner_state_resume.py`: ripristino verificato Adam/KL.
- `unified_MLP_39/tb_logging.py`, `episode_telemetry.py`: telemetria opzionale.
- `unified_MLP_39/validation/completion_noise_rollout.py`, `run_completion_diagnosis.py`, `analyze_completion_diagnosis.py`: valutazioni appaiate e analisi.
- `unified_MLP_39/validation/run_completion_continuation.py`: sequenza durevole, isolamento delle fasi e audit.
- Test mirati di optimizer/KL e campionamento: 9 passati; test della telemetria e orchestratore nelle rispettive ricevute.
- Dodici rollout fisici preliminari: uno deterministico, dieci stocastici e una ripetizione per il confronto A/B.

TODO: completare le tappe 60/80/100 e interpretare esiti, penetrazione e cambi di policy usando i nuovi dati per episodio.

## Avvio effettivo

La sequenza è stata avviata il 22 settembre alle **02:01 CEST**, dal checkpoint completo 40. L'audit del processo reale conferma actor, media e critic identici al checkpoint, tutti i 12 stati Adam collegati ai parametri con contatore 1215, KL ripristinato esattamente e prefisso dello storico conservato. Sono attivi 13 worker; non sono attivi rollout diagnostici concorrenti.

Alla verifica iniziale è in corso l'iterazione 41; il primo aggiornamento PPO non è ancora terminato. Il watchdog controllerà i dati della prima iterazione, la telemetria di tutti i worker entro la seconda e la continuità di Adam nei checkpoint. La sequenza prevista è 60 → analisi → 80 → analisi → 100 → analisi, con i sette grafici canonici di ogni tappa in `plot/`.

Evidenze e stato: `Trajectory Generator/unified_MLP_39/runs/plans/completion_continuation_2026-09-22/{live_startup_verification.json,status.json,pipeline.log}`. Il processo principale ha PID 38222; il completamento fino a 100 è ancora in corso.
