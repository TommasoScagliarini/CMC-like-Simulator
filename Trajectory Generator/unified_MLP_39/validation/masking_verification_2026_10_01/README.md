# Verifica completa del degrado con oscuramento — 2026-10-01

**Completata.** Due training da dieci aggiornamenti, 81.920 nuovi passi di training complessivi, sei nuove ablation a pesi fermi e otto valutazioni dei checkpoint ottenuti. Tutte le otto valutazioni dei nuovi training completano 500 passi. Salvati e verificati 98 PNG canonici. [Verifica automatica](verification.json).

## Risultato principale

La rimozione degli input del teacher causa già in CP90, senza cambiare i pesi, una forte perdita di imitazione. I primi dieci aggiornamenti PPO non recuperano questa capacità neppure mantenendo immutabile la funzione sigma di CP90. La crescita della sigma non è quindi necessaria per il degrado iniziale osservato. Il test non esclude un contributo della sigma nella parte successiva del curriculum originale.

Il quadro più coerente con i dati è: forte dipendenza dai target/clock durante il pretraining; adattamento insufficiente della policy agli input rimasti; sincronizzazione online che può cambiare con il movimento prodotto; reward aggregata che non ordina sempre le policy come la qualità delle curve cinematiche. Il peso causale specifico di reward, quantità di aggiornamento PPO e inizializzazione della fase richiede confronti separati: non è stato isolato dai due training di questa verifica.

## Protocollo e significato delle maschere

Il protocollo è in [PROTOCOL.md](PROTOCOL.md). “Progressivo” indica un aumento della probabilità di episodi completamente oscurati, non un’attenuazione continua dei segnali. Nei due training nuovi il 25% degli episodi è oscurato per tutta la durata. La maschera completa azzera clock prescribed (due canali) e target q/qdot (quattro canali), mantenendo propriocezione, fase online, FSM, timer e stato del controller.

CP90 era stato addestrato senza oscuramento. “CP90 oscurato” qui significa soltanto una nuova valutazione dei medesimi pesi con quei sei input azzerati. Non è un checkpoint riaddestrato.

Entrambi i rami ripartono dal checkpoint completo all’iterazione 90: stesso actor, critic, Adam, coefficiente KL iniziale e RNG delle maschere. Dieci aggiornamenti 91–100, batch4096, minibatch512, 10 epoche, lr1e-4, 13 worker, seed123, start offset1s, episodi5s. Media e critic apprendono in entrambi. Il ramo frozen conserva una copia immutabile della funzione sigma, inclusi gli strati condivisi originari: bloccare soltanto la testa logstd non sarebbe equivalente.

La quota effettiva di passi visibili è 75,86% nel controllo e 76,55% nel ramo frozen; la probabilità configurata è identica ma durate/eventi diversi possono cambiare il numero di passi per episodio. Il coefficiente KL è adattivo: parte da 0,45 in entrambi e termina a 0,45 / 0,1125. Il confronto misura l’effetto del trattamento sigma inclusa la risposta dell’algoritmo adattivo.

## Confronto dei training

RMSE actual–target knee/ankle in gradi, stessa finestra 13,19–17,94s. Ogni riga ha un rollout visibile e uno completamente oscurato, entrambi da 500 passi.

| Policy | RMSE visibile | RMSE oscurato | Escursione knee oscurato | Return visibile/oscurato |
|---|---:|---:|---:|---:|
| CP90, zero aggiornamenti | 6.64/3.94 | 13.28/9.55 | 11.55° | 301.14/146.06 |
| CP95, sigma appresa | 6.90/4.55 | 14.23/10.07 | 8.87° | 279.68/142.31 |
| CP95, sigma CP90 congelata | 7.29/4.37 | 14.00/9.45 | 17.22° | 282.82/148.59 |
| CP100, sigma appresa | 6.69/5.83 | 14.20/10.31 | 9.96° | 247.16/167.31 |
| CP100, sigma CP90 congelata | 7.74/3.45 | 13.70/10.20 | 9.93° | 295.23/158.85 |

A CP95 il ramo frozen aumenta l’escursione knee oscurata da 8,87° a 17,22° rispetto al controllo, ma i picchi restano fuori fase. A CP100 l’effetto non si mantiene: escursione 9,93° frozen e 9,96° controllo. CP90 visibile raggiungeva 41,71°. Congelare sigma, in questo budget, non recupera la forma imitativa.

Con input visibili il trattamento produce un compromesso: a CP100 migliora ankle (3,45° contro 5,83°) ma peggiora knee (7,74° contro 6,69°). Il return più alto del ramo frozen non equivale a miglioramento uniforme dei due giunti.

[Confronto CP100 oscurato](../../../../plot/10_01_2026_masking_verification_comparison/cp100_hidden_sigma_kinematics.png) · [CP95 oscurato](../../../../plot/10_01_2026_masking_verification_comparison/cp95_hidden_sigma_kinematics.png) · [CP100 visibile](../../../../plot/10_01_2026_masking_verification_comparison/cp100_visible_sigma_kinematics.png) · [Andamento PPO](../../../../plot/10_01_2026_masking_verification_comparison/training_comparison.png).

## Ablation clock e target a pesi fermi

| Checkpoint | Tutto visibile | Solo clock oscurato | Solo target oscurati | Tutto oscurato |
|---|---:|---:|---:|---:|
| CP90 | 6.64/3.94 | 10.06/5.50 | 11.41/6.98 | 13.28/9.55 |
| CP112 | 7.03/4.27 | 9.19/5.55 | 11.65/8.27 | 13.47/10.63 |
| CP184 | 10.90/5.66 (solo 0.98s) | 8.21/5.83 | 13.88/10.30 (solo 2.84s) | 12.43/8.89 |

Le due righe corte di CP184 sono terminazioni per limite articolare del ginocchio; le relative RMSE non vanno confrontate direttamente con quelle su cinque secondi. I confronti sul prefisso comune sono conservati in analysis.json. La matrice dimostra dipendenza da entrambi i gruppi di input. Le maschere parziali non erano presenti nel training: sono ablation fuori distribuzione, non prove di impossibilità di ricostruire il riferimento né proposte già validate di curriculum.

CP112 visibile mantiene buona parte dell’imitazione iniziale, mentre CP112 oscurato è già scarso. Perciò il peggioramento dei primi rollout oscurati non equivale a una cancellazione generale della capacità imitativa. CP184 visibile termina a 0,98s: nella fase avanzata del curriculum esiste anche un deterioramento del comportamento quando si ripristinano gli input completi.

## Fase online, avvio e confronto con V26 ex novo

La variabile che scandisce il passo resta presente anche senza clock prescribed: fase online da eventi, FSM e timer. La fase online usa il tempo dall’ultimo heel strike diviso per il periodo misurato, limitato a [0,1]. Prima di un periodo disponibile è zero. In V26 ex novo questi segnali potevano scandire un ciclo libero, con reward basata su eventi/regolarità; qui la reward di imitazione richiede inoltre una traiettoria sincronizzata a un riferimento esterno. I due problemi non sono equivalenti.

L’analisi confronta fasi nello stesso istante **prima dell’azione**, usando policy_segment_times[0], non la fase reward del passo successivo. L’errore è circolare, in frazioni di ciclo, calcolato soltanto dove il periodo online è disponibile.

| Rollout oscurato | Passi iniziali senza periodo | RMS fase online–prescribed dopo avvio |
|---|---:|---:|
| cp90_hidden | 258 | 0.054 cicli |
| learned_cp95_hidden | 209 | 0.384 cicli |
| frozen_cp90_cp95_hidden | 258 | 0.038 cicli |
| learned_cp100_hidden | 201 | 0.405 cicli |
| frozen_cp90_cp100_hidden | 206 | 0.382 cicli |
| cp112_hidden | 257 | 0.052 cicli |
| cp184_hidden | 261 | 0.038 cicli |

La fase è quindi un problema reale e dipendente dalla policy: in entrambi i nuovi CP100 oscurati si sposta di circa 0,4 cicli RMS, anche senza apprendimento della sigma. Tuttavia i best originali CP112/184 oscurati hanno fasi molto più vicine al riferimento e conservano comunque cinematica appiattita. La fase da sola non spiega tutti gli esiti. Anche traslando retrospettivamente le curve, CP184 mantiene circa 10,45° knee e 7,19° ankle di errore sul supporto fisso dell’analisi dei ritardi.

Questi confronti non isolano causalmente il clock: il movimento influenza gli eventi, gli eventi influenzano la fase osservata e la fase influenza le azioni. I timer restano disponibili anche prima del primo periodo, ma non sono automaticamente un sostituto del clock esterno già appreso. Le finestre startup/regime hanno porzioni diverse della reference e non sono confronti causali di RMSE. Eventi, resync e grafici per ogni rollout sono in analysis.json e nella cartella dei confronti.

## Reward, riferimento servito e coppie

Il tracking low-level resta preciso: nei due nuovi CP100 oscurati la RMSE actual–served è circa 0,3–0,4°. L’errore rispetto alla reference è già nel riferimento generato e servito. Questo localizza il problema principale a monte del tracking SEA.

La reward può salire mentre la cinematica peggiora. CP90 oscurato ha return146,06 e RMSE13,28°/9,55°; CP100 controllo ha return167,31 e RMSE14,20°/10,31°. Nel budget esatto, il punteggio misto di imitazione passa 241,23→253,81, i costi base 99,56→92,86, la correzione da clipping 5,32→6,74 e i costi extra 0,93→0,37. La reward comprende posizione, velocità, normalizzazioni e una trasformazione non lineare delle loss; non è una riscrittura della RMSE globale delle posizioni.

L’analisi precedente trova anche 67 passi con reward base a zero in CP184, tutti durante la porzione ad alta flessione knee. È evidenza compatibile con scarso incentivo differenziale nelle porzioni già molto sbagliate, non una dimostrazione causale della reward come unica causa: non è stata eseguita un’ablation della reward.

Le osservazioni sulle coppie vanno separate: nel confronto originale CP90 visibile / CP112 oscurato / CP184 oscurato, la frazione di tempo con coppia ankle positiva è 57,3% / 50,0% / 43,1%, ma l’area positiva è 2,88 / 8,41 / 3,42 Nm·s. Il knee spring RMS sopra20Hz è 3,62 / 3,37 / 3,50 Nm, mentre il corrispondente RMS della coppia motore è 13,24 / 14,25 / 15,32 Nm. Una curva visivamente più regolare non dimostra un miglioramento di tutte le misure di chattering o di coppia. Dettagli nel [report precedente](../masking_diagnosis_2026-10-01/README.md).

## Sigma e differenza fra training stocastico e rollout deterministico

La funzione sigma frozen è identica a CP90 dopo tutti e dieci gli aggiornamenti e su ogni osservazione dei quattro rollout frozen. Sulle 40 osservazioni fisse il controllo CP100 varia logstd fino a 0,2413 in valore assoluto; la sigma media passa knee0,8015→0,8021 e ankle0,7964→0,8567. Non si osserva su questi probe una crescita generalizzata paragonabile alle code estreme del training avanzato.

Il congelamento non equivale a sigma piccola: resta la distribuzione relativamente larga di CP90. In più, i rollout deterministici usano la media; durante training si campiona e poi si limita l’azione a [-1,1]. In generale E[clip(a)] differisce da clip(E[a]). Sugli stati registrati di CP184 oscurato, la probabilità condizionale media di clipping è 47,4% knee / 41,0% ankle; lo scarto RMS fra endpoint atteso dopo clipping e endpoint deterministico è circa9,06°/9,16°.

Questo è un calcolo analitico a osservazione fissata, con trasformazione azione→angolo verificata su ogni riga. Non è un rollout stocastico e non predice la traiettoria con feedback, governor e contatto. Mostra un meccanismo da considerare anche a sigma congelata, non attribuisce a esso l’intero degrado. Dati e limiti in rollout_distributions.json.

## Cosa farei prima di ripetere un curriculum lungo

1. Rendere esplicito il trasferimento dagli input teacher a quelli disponibili: per esempio un confronto controllato con distillazione della media del teacher su stati visitati dalla policy oscurata, mantenendo un vincolo di imitazione durante il passaggio. È una proposta da verificare, non una soluzione già dimostrata.
2. Isolare inizializzazione e allineamento della fase, distinguendo imitazione sincronizzata al tempo esterno da imitazione riferita agli eventi del passo. Il successo di V26 ex novo non prova che l’attuale actor sappia effettuare questo trasferimento.
3. Verificare che selezione dei checkpoint e reward distinguano perdita di ampiezza/sincronizzazione da riduzione dei costi. Le curve, la RMSE e il completamento devono restare criteri separati dal return.
4. Trattare sigma limitata/piccola e divario stocastico–deterministico come ulteriori confronti dedicati. Il solo congelamento della funzione iniziale, qui, non risolve.

## Integrità, file e riproducibilità

- Nessuna modifica al simulatore root, ai plugin C++ o alla semantica SEA. Nuovo codice sperimentale soltanto in questa directory; checkpoint originali conservati.
- preflight_sigma.json: ripristino esatto di actor/critic, 12 stati Adam e KL; 1.000 osservazioni di prova; gradienti della media, invarianza sigma, export/reload verificati.
- control_reproduction.json: tutti i dieci checkpoint del controllo riproducono esattamente export actor e stato completo del learner originale, inclusi sei tensori critic, metriche PPO e batch delle maschere.
- training_summary.json: 40.960 passi per ramo, media e critic aggiornati; bootstrap invariato come concordato. Il log opzionale dei singoli episodi di training non era attivo: non sono disponibili nuove RMSE di training separate per visibilità o quantili delle code stocastiche. Questo limite non riguarda i rollout registrati né i controlli sigma sugli stati fissi.
- matrix_execution.json e comparison_execution.json: ricevute di tutte le esecuzioni valide. verification.json: 257 voci di manifest verificate, maschere controllate su ogni passo, sigma frozen controllata su ogni stato di valutazione, 98 PNG canonici verificati.
- Primo avvio Ray fermato dalla sandbox prima di qualsiasi aggiornamento: conservato in attempt01_sandbox_startup_failure; rilancio con permessi locali riuscito. Il primo tentativo di valutare i moduli custom è stato rifiutato dal controllo di classe prima di simulare: conservato in attempt02_standard_evaluator_class_rejection. Nessun training ripetuto per questo problema.
- Per valutare questi export diagnostici usare sigma_rollout.py: ammette soltanto SigmaComparisonModule e mantiene tutti i controlli originali su feature, architettura, azioni e tensori, aggiungendo controlli sul riferimento CP90. evaluation_contract_preflight.json verifica i quattro export, il rifiuto di contratti errati e l’invarianza dei pesi. Il valutatore di produzione resta invariato.
- Script: mask_rollout.py/run_mask_matrix.py; sigma_module.py/train_sigma.py/preflight_sigma.py; run_sigma_comparison.py e resume_evaluations.py; analyze_results.py, summarize_training.py, audit_rollout_distributions.py, render_results.py, verify_complete.py. Configurazioni learned.yaml e frozen_cp90.yaml.
- Dati: runs/training/MLP39_masking_sigma_verification_2026-10-01 e runs/rollout/masking_verification_2026-10-01 sotto unified_MLP_39. Sette PNG per nuovo rollout nel root plot/10_01_2026_masking_verification_<nome>. Confronti nel root plot/10_01_2026_masking_verification_comparison.

## Tempi e limiti

Matrice delle sei nuove ablation: 17.23 minuti di tempo trascorso, con massimo tre rollout simultanei. Training controllo: 76.96 minuti; frozen: 76.19 minuti. Otto valutazioni: 25.08 minuti. Dall’avvio della matrice all’ultima valutazione: 3.334 ore, includendo gli intervalli operativi; preparazione e analisi finale sono ulteriori.

Un seed di training e un’unica inizializzazione deterministica: evidenza diagnostica, non stima di robustezza. Dieci aggiornamenti al25% di oscuramento non equivalgono ai cento aggiornamenti del curriculum completo. Il test esclude la necessità di far crescere sigma per il degrado iniziale, non la sua rilevanza tardiva. Nessuna correzione del bootstrap o nuova reward è stata introdotta. Verifica nativa eseguita su macOS; Windows non è stato eseguito.

## Tabella completa dei rollout

RMSE su13,19–17,94s quando disponibile; le righe corte hanno una finestra diversa. I return coprono l’intero episodio registrato.

| Rollout | Steps | RMSE knee/ankle [deg] | Window end [s] | Return |
|---|---:|---:|---:|---:|
| cp90_visible | 500 | 6.64/3.94 | 17.94 | 301.14 |
| cp112_hidden | 500 | 13.47/10.63 | 17.94 | 138.19 |
| cp184_hidden | 500 | 12.43/8.89 | 17.94 | 210.42 |
| cp90_hidden | 500 | 13.28/9.55 | 17.94 | 146.06 |
| cp112_visible | 500 | 7.03/4.27 | 17.94 | 273.12 |
| cp184_visible | 98 | 10.90/5.66 | 13.96 | 45.88 |
| cp112_clock_hidden | 500 | 9.19/5.55 | 17.94 | 214.03 |
| cp112_target_hidden | 500 | 11.65/8.27 | 17.94 | 255.66 |
| cp184_clock_hidden | 500 | 8.21/5.83 | 17.94 | 220.07 |
| cp184_target_hidden | 284 | 13.88/10.30 | 15.82 | 115.34 |
| cp90_clock_hidden | 500 | 10.06/5.50 | 17.94 | 201.91 |
| cp90_target_hidden | 500 | 11.41/6.98 | 17.94 | 248.81 |
| frozen_cp90_cp100_hidden | 500 | 13.70/10.20 | 17.94 | 158.85 |
| frozen_cp90_cp100_visible | 500 | 7.74/3.45 | 17.94 | 295.23 |
| frozen_cp90_cp95_hidden | 500 | 14.00/9.45 | 17.94 | 148.59 |
| frozen_cp90_cp95_visible | 500 | 7.29/4.37 | 17.94 | 282.82 |
| learned_cp100_hidden | 500 | 14.20/10.31 | 17.94 | 167.31 |
| learned_cp100_visible | 500 | 6.69/5.83 | 17.94 | 247.16 |
| learned_cp95_hidden | 500 | 14.23/10.07 | 17.94 | 142.31 |
| learned_cp95_visible | 500 | 6.90/4.55 | 17.94 | 279.68 |

