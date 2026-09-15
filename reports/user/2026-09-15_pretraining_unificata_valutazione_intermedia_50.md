# Pretraining unificata: valutazione intermedia a 50 iterazioni

Redatto: 2026-09-15T08:12:03.072324+02:00. Instruction check token: `CMC_AGENT_OK_2026`.

## Problema

Verificare se la V1 a 35 feature, con reward imitativa e clock prescribed disabilitato, migliora dopo il resume dal checkpoint 5 e sviluppa cicli validi ai tre start. Il periodo online resta da distinguere dalla stabilità del cammino.

## Soluzione e risultato intermedio

Training completato a **50/50 iterazioni**; **45 aggiornamenti dopo CP5**. La valutazione è completa per 9 casi: CP5, ultimo CP50 e best di training CP49 (deduplicati se coincidono). Il best è selezionato dal return, senza implicare qualificazione.

- CP5: screening **non superato**; center:duration_5s, center:two_valid_cycles, center:online_period_observed, center:normal_episode_completion, early:two_valid_cycles, early:online_period_observed, late:duration_5s, late:two_valid_cycles, late:online_period_observed, late:normal_episode_completion.
- CP49: screening **non superato**; center:duration_5s, center:no_invalid_events, center:normal_episode_completion, center:penetration_below_28mm, early:two_valid_cycles, early:online_period_observed, late:duration_5s, late:two_valid_cycles, late:online_period_observed, late:normal_episode_completion.
- CP50: screening **non superato**; center:duration_5s, center:two_valid_cycles, center:online_period_observed, center:no_invalid_events, center:normal_episode_completion, center:penetration_below_28mm, early:two_valid_cycles, early:online_period_observed, early:no_invalid_events, late:duration_5s, late:no_invalid_events, late:normal_episode_completion, late:penetration_below_28mm.

Lo screening è una condizione necessaria di fattibilità deterministica. Non dimostra parità con la baseline, robustezza statistica o qualità del tracking. La sincronizzazione e l'osservabilità rimangono oggetto della fase 5.

## Strategia

Confronto per start e giunto con seed 123, azioni deterministiche, massimo 500 passi a 10 ms (5 s). Actual e served sono confrontati con il prescribed al timestamp allineato. Le finestre complete riportano l'intero episodio osservato; CP5↔CP50 usa il prefisso temporale comune, senza estrapolazione o aggregazione degli start sopravvissuti.

### Episodi completi

| Start | CP / ruolo | Durata [s] | HS netti / annullati / TO / cicli | Primo periodo disponibile [s] | Pen max [mm] | Fine |
|---|---|---:|---:|---:|---:|---|
| early | 5 / historical_CP5 | 5.000 | 0 / 0 / 0 / 0 | non disponibile | 23.984 | episode_time_limit |
| early | 49 / best_training_return | 5.000 | 0 / 0 / 0 / 0 | non disponibile | 21.263 | episode_time_limit |
| early | 50 / last | 5.000 | 0 / 2 / 0 / 0 | non disponibile | 18.443 | episode_time_limit |
| center | 5 / historical_CP5 | 2.800 | 0 / 0 / 1 / 0 | non disponibile | 24.862 | phase_timeout:swing |
| center | 49 / best_training_return | 3.490 | 3 / 1 / 4 / 3 | 1.5999999999991132 | 28.950 | grf_penetration |
| center | 50 / last | 1.670 | 1 / 0 / 2 / 0 | non disponibile | 28.123 | grf_penetration |
| late | 5 / historical_CP5 | 2.830 | 0 / 0 / 1 / 0 | non disponibile | 18.772 | phase_timeout:swing |
| late | 49 / best_training_return | 1.410 | 0 / 0 / 1 / 0 | non disponibile | 16.361 | joint_divergence:pros_knee_angle |
| late | 50 / last | 3.370 | 3 / 0 / 4 / 2 | 2.3099999999990466 | 28.657 | grf_penetration |

HS è il conteggio netto della FSM. La versione v3 può annullare un appoggio riconosciuto come rimbalzo; il valutatore ammette tali decrementi solo se giustificati da cancellazioni registrate. Nelle sottofinestre il saldo HS può essere negativo se viene annullato un appoggio precedente. Le cancellazioni sono esposte separatamente, senza contarle come appoggi validi.

Il primo periodo è il tempo trascorso dal reset alla prima osservazione pre-step con periodo > 0; non è la durata del ciclo. Startup e regime restano separati nel JSON; un regime nullo indica che il periodo non è diventato disponibile.

### RMSE sugli episodi completi

Ogni cella riporta **actual / served** rispetto al prescribed.

| Start | CP | Ginocchio q [°] | Caviglia q [°] | Ginocchio qdot [°/s] | Caviglia qdot [°/s] |
|---|---:|---:|---:|---:|---:|
| early | 5 | 24.965 / 24.914 | 12.269 / 12.209 | 117.721 / 115.474 | 129.517 / 127.967 |
| early | 49 | 19.994 / 19.919 | 9.980 / 9.902 | 122.864 / 119.995 | 78.194 / 77.184 |
| early | 50 | 20.227 / 20.165 | 9.476 / 9.428 | 120.862 / 118.340 | 73.055 / 72.476 |
| center | 5 | 25.274 / 25.234 | 11.297 / 11.249 | 130.477 / 128.634 | 104.841 / 103.618 |
| center | 49 | 17.353 / 18.439 | 11.189 / 11.347 | 126.222 / 125.933 | 75.663 / 70.861 |
| center | 50 | 9.720 / 9.488 | 16.274 / 16.427 | 73.425 / 71.717 | 93.564 / 80.870 |
| late | 5 | 23.634 / 23.591 | 10.750 / 10.697 | 118.232 / 116.485 | 112.071 / 110.189 |
| late | 49 | 11.033 / 10.953 | 14.237 / 14.234 | 60.263 / 56.834 | 65.676 / 61.793 |
| late | 50 | 18.497 / 18.824 | 12.009 / 12.169 | 125.092 / 123.364 | 84.764 / 79.334 |

### CP5 ↔ CP50: prefisso comune per start

| Start | CP | Finestra [s] | Ginocchio q actual/served [°] | Caviglia q actual/served [°] | Ginocchio qdot actual/served [°/s] | Caviglia qdot actual/served [°/s] |
|---|---:|---:|---:|---:|---:|---:|
| early | 5 | 5.000 | 24.965 / 24.914 | 12.269 / 12.209 | 117.721 / 115.474 | 129.517 / 127.967 |
| early | 50 | 5.000 | 20.227 / 20.165 | 9.476 / 9.428 | 120.862 / 118.340 | 73.055 / 72.476 |
| center | 5 | 1.670 | 25.365 / 25.332 | 10.565 / 10.524 | 138.901 / 136.544 | 99.456 / 98.666 |
| center | 50 | 1.670 | 9.720 / 9.488 | 16.274 / 16.427 | 73.425 / 71.717 | 93.564 / 80.870 |
| late | 5 | 2.830 | 23.634 / 23.591 | 10.750 / 10.697 | 118.232 / 116.485 | 112.071 / 110.189 |
| late | 50 | 2.830 | 18.887 / 19.275 | 12.386 / 12.405 | 126.572 / 124.077 | 87.592 / 81.959 |

Ulteriori confronti con il best, SEA, saturazione e penetrazione sopra 20/25/28 mm: [analisi completa](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/milestone_evaluation.md>) e [metrics.csv](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/metrics.csv>).

### Fonti e figure

- [milestone_evaluation.json](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/milestone_evaluation.json>)
- [training_summary_50.json](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/training_summary_50.json>)
- [stato del piano successivo](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/STATUS.md>)
- [tracking_early.png](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_early.png>)
- [tracking_early.pdf](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_early.pdf>)
- [tracking_center.png](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_center.png>)
- [tracking_center.pdf](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_center.pdf>)
- [tracking_late.png](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_late.png>)
- [tracking_late.pdf](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_late.pdf>)
- [daily precedente e TODO storici](<../daily/2026-09-14_daily-report.md>)
- [2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md](<2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md>)
- [2026-09-14_pilot_unificata_strategia_50_100_e_test_39_feature.md](<2026-09-14_pilot_unificata_strategia_50_100_e_test_39_feature.md>)

## File modificati

Questa redazione genera il presente report, aggiunge la sintesi e i TODO al daily della stessa data e salva `create_report_50_receipt.json` nella directory del piano. L'automazione è in `Trajectory Generator/unified_MLP/validation/create_milestone_report.py`; `validation/compare_pretraining_checkpoints.py` e `validation/evaluate_training_milestone.py` gestiscono i contatori netti e le cancellazioni HS, con verifiche nei rispettivi test. valutatore e coordinatore sono documentati nel report di avvio. Le sorgenti del training e il plugin SEA non vengono modificati dalla redazione.

## Test e verifiche eseguite

- Validati status completo, milestone 50, identità del run, history 1–50 senza salti, reward imitativa e bilanciamento dei tre start.
- Verificati tutti i casi last/best/CP5, metriche finite, tre prefissi comuni CP5/CP50 e presenza di JSON, CSV, Markdown e figure; hash delle fonti nella ricevuta.
- Summary archiviato: `ok=True`, `stop_reason=completed`, `iterations_completed=50`; restart totali: 0; crash restart: 0; iterazioni saltate: [].
- I conteggi di restart appartengono al summary archiviato del resume completato. Il fallimento iniziale sandbox `psutil/sysctl` resta un tentativo precedente, documentato nel report di avvio; non viene inferito dai file di stato mutabili.
- Tempo wall dichiarato dal summary: 24865.6 s, relativo all'ambito di quel processo/resume; non viene presentato come costo totale CP1–50.
- Questa redazione non esegue nuovi test, training o rollout: riporta i risultati runtime già conclusi. La verifica live Windows rimane aperta.

## TODO aperti

Le fasi fino a 50 e la valutazione intermedia sono concluse come esecuzione. Gli obiettivi di qualità non sono risolti automaticamente da questo completamento. I TODO storici seguenti sono propagati senza riavviare campagne precedenti.

- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.
- [ ] Creare la variante parallela a 39 feature soltanto nella successiva fase dei test di fattibilità, conservando V1 a 35 e baseline storica; nessuna implementazione ora.
- [ ] Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.
- [ ] Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.
- [ ] Risolvere e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start.
- [ ] Costruire dati diagnostici temporalmente allineati e valutare osservabilità delle 35 feature su episodi indipendenti.
- [ ] Scegliere fra curriculum con target mascherati, avvio supervisionato e possibili soluzioni causali di fase/memoria sulla base di evidenze.
- [ ] Se si procede con la maschera, definire il nuovo schema, la normalizzazione, l'eventuale indicatore di disponibilità e lo stato persistente del curriculum, conservando la V1.
- [ ] Implementare e verificare isolamento dei target, coerenza raccolta/PPO e indipendenza degli aggiornamenti ex-novo dalle copie dei target.
- [ ] Definire budget, seed, progressione e criteri di accettazione prima del confronto A/B/C; eseguire poi il pilot e le repliche necessarie.
- [ ] Validare i candidati completamente mascherati ai tre start, con metriche separate di avvio, tracking, cicli validi e protezioni.
- [ ] Verificare trasferimento, warm-up e resume della variante scelta e confrontarne il beneficio nel successivo ex-novo.
- [ ] Misurare in un nuovo run imitativo l'effetto del peso di penetrazione 1.0, senza attribuirgli gli esiti dello smoke storico a 0.05.
- [ ] Completare il benchmark e il verdetto di parità qualitativa secondo il report del 6 settembre, includendo costo di apprendimento e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows della pipeline scelta; riprendere predittività e compensazione del ritardo soltanto in una versione successiva.
- [ ] Fissare checkpoint baseline, manifest degli input e protocollo comune
  di confronto, mantenendo intatti gli artefatti storici.
- [ ] Implementare il valutatore comune sotto `Trajectory Generator/unified_MLP/`,
  con metriche per start e per giunto, copertura dei cicli e gestione esplicita
  dei fallimenti e dei dati mancanti.
- [ ] Completare e riprodurre il benchmark baseline deterministico sui tre start.
- [ ] Definire prima della selezione finale margini di regressione, condizioni
  riservate alla valutazione, numero di repliche e budget di addestramento.
- [ ] Eseguire un vero pretraining e training ex-novo della rete unificata,
  con valutazioni periodiche; pilot imitativo a cinque iterazioni concluso,
  prosecuzione e training ex-novo completo ancora da eseguire.
- [ ] Eseguire il confronto deterministico e stocastico finale, includendo
  variabilità tra seed di training e incertezza delle differenze.
- [ ] Produrre il verdetto quantitativo di parità, regressione o risultato
  inconclusivo, insieme al costo necessario per raggiungere la qualità ottenuta.
- [ ] Eseguire le verifiche live su Windows per attestare anche quella piattaforma.
- [ ] Riprendere actor predittivo e compensazione del ritardo in una versione
  successiva; restano esclusi dal recupero della parità della V1 attuale.
- [ ] Implementare il teacher inverse-governor e il generatore delle label raw.
- [ ] Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`
  ankle su più cadenze e stati del governor.
- [ ] Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e
  recovery.
- [ ] Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.
- [ ] Ripetere il critic-only rewarm quando la reward diventerà realmente
  asimmetrica.
- [ ] Valutare una fascia soft interna e una corretta assegnazione causale del
  credito, senza spostare il corridor finale.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con
  `sigma=0.010` isotropico e `freeze_logstd=true`, senza modificare J20 o il
  canonico lungo.
- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i
  rollout deterministici e i gate del pilot.
- [ ] Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM
  v3 in tutti i nuovi candidati.
- [ ] Rivalutare sigma dopo l'actor predittivo o una modifica della logica
  causale del morphology corridor.
- [ ] Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,
  dopo il recupero della qualità AB06.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione
  esplicita.
- [ ] Conservare sidecar J8 e artefatti storici senza modifiche retroattive.
- [ ] Riallineare in manutenzione separata le assertion legacy soggette a drift
  e gli altri debiti tecnici/amministrativi già propagati: osservabilità RLlib,
  `cycle_valid`, swing timeout, probe schema, pin V20, `best == last`,
  `fit_p2` V12R3, marker/audit storici e deflessione SEA iniziale.
- [ ] Completare V1 fino a100 e confrontare5/50/100.
- [ ] Creare la variante39 dopo valutazione100; superare isolamento e smoke.
- [ ] Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.
- [ ] Rendicontare tutti gli esiti e costi, distinguendo fattibilità e parità.
- [ ] Eseguire la verifica live Windows separata.
- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale 5/50/100.
- [ ] Svolgere la fase 5: variante 39, isolamento, smoke, confronto A/B/C e trasferimento ex-novo B/C secondo il protocollo.
- [ ] Completare la diagnostica dell'osservabilità e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime.
- [ ] Completare il benchmark di parità con la baseline, includendo costi e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows; mantenere aperti i debiti storici non risolti.
