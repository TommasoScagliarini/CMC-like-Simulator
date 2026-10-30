# Robustezza della unified 39D: diagnosi al checkpoint 60

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 22 settembre 2026. Stato verificato alle 12:31 CEST: completata l’iterazione 70, in corso la 71. Il training fino a 100 non è ancora concluso.

## Problema e conclusione provvisoria

L’obiettivo è spiegare il basso completamento degli episodi della unified 39D rispetto al training imitativo V26 del 20 agosto. Il riferimento iniziale della diagnosi era il **38,8% di episodi completi nelle iterazioni 36–40**, ossia 19/49.

I dati raccolti fino alla tappa 60 localizzano la fragilità soprattutto nel contatto piede-terreno durante l’appoggio. La policy riesce a completare una traiettoria nominale, ma durante l’esplorazione molti episodi si avvicinano alla soglia di penetrazione e alcuni la superano rapidamente. **Non è ancora isolata una singola causa radice**: errore della traiettoria, risposta dinamica e distribuzione delle azioni restano da distinguere. Non è dimostrata l’equivalenza statistica con V26.

## Configurazione e strategia

La continuazione mantiene la ricetta concordata: actor 39D, FSM v3, prescribed sinistro guidato dalla fase sinistra senza sfasamento, clock e prescribed sempre visibili, velocità prescribed non divise per dieci, slew limiter disattivato, start unico e batch PPO di 4096 campioni con 13 worker. Restano invariati filtro e dinamica SEA.

Training e simulazioni diagnostiche sono sequenziali. Alle tappe 60, 80 e 100 il training viene chiuso; seguono un rollout deterministico e dieci stocastici, su cinque seed appaiati con deviazione standard originale o dimezzata. La riduzione del rumore riguarda soltanto la valutazione. Durante il training rimangono registrazione per episodio e controlli leggeri.

## Risultati alla tappa 60

| Misura | Risultato |
|---|---:|
| Episodi completi nelle iterazioni 56–60 | 25/44, **56,8%** |
| Episodi conclusi registrati nel tratto 42→60 | 160 |
| Completi fra i 160 registrati | 92/160, **57,5%** |
| Rollout deterministico del checkpoint 60 | 500/500 passi, return **254,16** |
| Rollout con rumore originale | 4/5 completi, return medio **264,50** |
| Rollout con rumore dimezzato | 4/5 completi, return medio **271,04** |

Il return deterministico era 222,15 al checkpoint 40. Alla tappa 60 il RMSE deterministico è 6,42° al ginocchio e 5,13° alla caviglia; la penetrazione massima è 16,63 mm. Migliorare return o completamento non dimostra, da solo, una camminata equivalente a V26.

I denominatori delle finestre di training e dell’intera telemetria sono diversi. I rollout usano una policy fissa; il training comprende policy successive ed episodi che attraversano aggiornamenti.

## Localizzazione dei fallimenti

Dei 68 episodi falliti registrati:

- 64 terminano per `grf_penetration` e 2 per `grf_penetration_hard`;
- 1 termina per evento binario non valido e 1 per divergenza del ginocchio;
- 61 terminano in `STANCE_AFTER_HS`; tutti questi 61 appartengono ai 66 fallimenti per penetrazione.

Anche gli episodi completi hanno spesso poco margine. Il loro picco medio di penetrazione è **24,01 mm**, la mediana 24,63 mm e il massimo 27,82 mm, rispetto alla soglia ordinaria di arresto di circa 28 mm. **29/92** raggiungono almeno 26 mm e **16/92** almeno 27 mm. Si tratta di episodi stocastici di training, non del singolo rollout deterministico 60.

Nei 66 fallimenti per penetrazione, fra il primo superamento di 20 mm osservato nella coda e la terminazione trascorrono in mediana **70 ms**, in media 143 ms, con intervallo 20–480 ms. Nei due rollout falliti a checkpoint fisso 60 questo intervallo è di circa 40 ms. La misura usa gli ultimi 50 passi: non ricostruisce necessariamente il primo evento dell’intero episodio.

Questi dati suggeriscono poco margine e una finestra breve di recupero. **Non provano che il ritardo del filtro causi il fallimento.**

## Caviglia, esplorazione e clipping

L’errore terminale assoluto medio alla caviglia è **7,82° nei fallimenti contro 4,89° nei completi**. Fase e tempo terminali differiscono: è un indizio da verificare con confronti alla stessa fase, non una prova causale. Nei due rollout falliti del checkpoint 60 l’errore ha segno opposto, circa +7,95° e −8,37°: non emerge una correzione direzionale universale della caviglia.

Il rumore dimezzato non elimina i fallimenti: il seed 125 fallisce a 302 passi con scala 0,5 ma completa con scala 1; il seed 127 fallisce a 464 passi con scala 1 ma completa con scala 0,5. Cinque seed per condizione sono pochi: il risultato 4/5 contro 4/5 non dimostra uguaglianza delle probabilità di successo, né esclude un ruolo dell’esplorazione.

**Precisazione sulla telemetria:** lo 0% di clipping riportato dal riepilogo automatico misura il clipping osservato nell’ambiente, dopo il passaggio nel connector RLlib. Non significa che le azioni originali siano tutte entro i limiti.

Il ricalcolo dalle azioni originali nei soli ultimi 50 passi di ciascun episodio dà:

| Misura nella coda | Completi | Falliti |
|---|---:|---:|
| Componenti delle azioni fuori da [-1, 1] | 47,70% | 48,40% |
| Passi con almeno una componente fuori intervallo | 71,39% | 73,62% |

Sono 4600 righe nei completi e 3400 nei falliti. Questi numeri riguardano le azioni della policy, non la saturazione delle coppie dei SEA. Il clipping è diffuso anche negli episodi riusciti e quindi non discrimina da solo il fallimento. Ridurre la deviazione standard modifica inoltre la media effettiva delle azioni dopo clipping, non soltanto la loro variabilità.

## FSM e cambi di policy

Un resync compare negli ultimi 0,5 s in 9/68 fallimenti e 22/92 completi; negli ultimi 0,1 s in soli 3/68 fallimenti. Non emerge come evento immediato comune alla maggioranza dei fallimenti. Questo non esclude effetti più lontani nel tempo o altri aspetti della FSM.

154/160 episodi attraversano almeno un cambio di versione dei pesi. Solo 5/68 fallimenti hanno un cambio rilevato negli ultimi 0,25 s. I sei episodi senza cambi falliscono presto, con durata media 1,81 s: confrontarli direttamente con quelli lunghi introdurrebbe una selezione legata alla sopravvivenza. Non è dimostrato che i cambi di policy causino la scarsa robustezza.

## Correzioni software e verifiche già eseguite

Il precedente report documenta la correzione del ripristino Adam e il salvataggio esplicito del coefficiente KL adattivo. Il difetto Adam nella ripresa 25→40 può aver influito su quel tratto, ma non spiega il divario già presente prima della 25. Ripristinare lo stato salvato non ricostruisce gli RNG e gli episodi OpenSim interrotti.

La prima continuazione si era fermata dopo la 42, alle 02:17: un valore `numpy.bool_` della disponibilità dei target impediva la scrittura JSON della nuova telemetria. Il campo è stato convertito in un tipo JSON nativo. Sono passati **15 test della telemetria**, un controllo su worker Ray reale senza simulazione OpenSim e **11 test dell’orchestrazione**. La ripresa effettiva dalla 42 è avvenuta alle 08:13, con audit di Adam, KL e pesi. La telemetria dettagliata delle iterazioni 41–42 non è recuperabile; storico e checkpoint sono conservati.

La tappa 60 e le relative valutazioni sono concluse. La ripresa 60→80 è iniziata alle 11:08. Alle 12:31 risultano completate 70 iterazioni; nelle ultime cinque, 66–70, sono completi **33/46 episodi, 71,7%**, con 13 terminazioni per penetrazione. È un aggiornamento del trend, non parte del campione diagnostico di 160 episodi analizzato sopra.

## File, artefatti e modifiche

Questo aggiornamento aggiunge soltanto il presente report; non modifica codice, configurazione o processi del training. Sono stati letti gli artefatti già salvati e ricalcolate aggregazioni leggere dalle code degli episodi, senza nuove simulazioni.

File modificati nelle precedenti attività documentate:

- `Trajectory Generator/unified_MLP_39/train_ppo_mlp.py`, `optimizer_resume.py`, `learner_state_resume.py`: ripresa e verifiche dello stato;
- `Trajectory Generator/unified_MLP_39/episode_telemetry.py`, `tb_logging.py`: registrazione diagnostica e correzione della serializzazione;
- script in `Trajectory Generator/unified_MLP_39/validation/`: orchestrazione, rollout appaiati e analisi offline, con relativi test.

Fonti e risultati:

- [Report della diagnosi iniziale e delle correzioni della ripresa](2026-09-22_diagnosi_completamento_e_ripresa_unified39.md).
- [Riepilogo degli episodi fino alla tappa 60 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-f2f57dca18f87050>), con `summary.json` e `per_episode.jsonl` nella stessa cartella.
- [Valutazione del rumore al checkpoint 60 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-497ad289bd46c16a>), con dettagli per rollout in `noise_analysis.json`.
- [Confronto delle tappe — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-46df9dc9bb280b63>). Piano attivo: `completion_recovery42_2026-09-22`; stato corrente in `status.json` e cronologia in `pipeline.log`.
- [Set dei sette grafici canonici al checkpoint 60](../../plot/09_22_2026_1/), salvato nella cartella `plot/` del root. La ricevuta del rollout identifica checkpoint e target sinistro; il nome storico del file 07 contiene ancora `sound_leg`, ma le etichette del grafico indicano il target sinistro.

## TODO da propagare al daily report

- [x] Completare la tappa 60, i rollout appaiati, l’analisi della telemetria e i sette grafici canonici.
- [ ] Completare le tappe 80 e 100 e le relative verifiche separate dal training.
- [ ] Confrontare episodi riusciti e falliti alla stessa fase e prima del superamento della soglia: target, riferimento servito, stato articolare, contatto e azioni originali.
- [ ] Distinguere contributi di errore della traiettoria, risposta dinamica e distribuzione delle azioni con prove controllate dopo questa raccolta; non attribuire causalità alle sole associazioni osservate.
- [ ] Esplicitare nei futuri riepiloghi la distinzione fra clipping delle azioni originali e clipping interno all’ambiente. Evitare modifiche ai sorgenti congelati mentre il piano attivo li usa.
- [ ] Consolidare il confronto finale con V26 a esperienza e protocollo di valutazione comparabili; più seed di training sono necessari per una conclusione di equivalenza statistica.

Le tappe 80/100 e l’interpretazione delle cause restano aperte anche rispetto al TODO del precedente report.
