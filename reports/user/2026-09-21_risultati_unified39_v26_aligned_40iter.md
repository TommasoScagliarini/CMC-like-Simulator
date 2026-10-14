# Risultati della continuazione 39D riallineata al V26 fino a 40 iterazioni

Data: 21 settembre 2026.

## Valutazione

**La continuazione migliora il ginocchio e la regolarità dei contatti nel rollout deterministico della 40, ma non recupera la robustezza del V26.** A esperienza quasi identica, gli episodi completi nelle ultime cinque iterazioni sono il 38,8% contro l'88,4% storico. La caviglia rimane poco accurata; emerge inoltre una penalità importante sul momento GRF alla caviglia, quasi assente alla 25.

Il completamento deterministico dei 5 s è confermato per best29 e last40, ma il risultato complessivo non qualifica ancora la configurazione come equivalente al V26. Il minor budget di dati non spiega più il divario di stabilità. La prossima diagnosi deve distinguere la sensibilità all'esplorazione dalle difficoltà della traiettoria e del contatto, prima di assumere che bastino altre iterazioni.

## Obiettivo e metodo

Valutare se la continuazione dalla 25 alla 40 consolida l'imitazione e recupera la robustezza del training imitativo V26 del 20 agosto. Il confronto principale usa la quantità di transizioni effettive, perché il V26 storico raccoglieva batch talvolta superiori ai 4096 nominali durante il recupero dai guasti dei worker.

La configurazione è quella concordata: prescribed sinistro indicizzato dalla fase sinistra senza shift, clock e quattro target sempre visibili, velocità non divise per 10, FSM v3, start unico con offset 1 s, reward e parametri PPO riallineati al V26, slew limiter disattivato. La continuazione ripristina il checkpoint completo della 25. Non sono stati cambiati reward, configurazione di esplorazione, FSM o controllori SEA.

Sono analizzati lo storico di training, i contatori di terminazione, le loss TensorBoard e i rollout deterministici allo stesso start fisico, 12,99 s. Il best è selezionato dal ritorno medio durante training: non è necessariamente il checkpoint con il miglior rollout deterministico. Il riferimento cinematico storico disponibile è il V26 best87; i checkpoint intermedi del V26 non sono conservati.

## Integrità della continuazione

| Verifica | Esito |
|---|---:|
| Iterazioni totali | 40/40 |
| Iterazioni nuove | 26–40, tutte completate |
| Termine del training | 19:23:20 CEST |
| Durata della continuazione | 1 h 56 min 39 s |
| Transizioni aggiuntive | 61.440 |
| Transizioni cumulative | 163.840 |
| Miglior iterazione per ritorno di training | **29** |
| Miglior ritorno medio | **207,99** |
| Ritorno medio all'iterazione 40 | **175,72** |
| Crash, restart, iterazioni saltate | 0 |
| Prescribed visibili | 163.840/163.840 transizioni |

Gli hash dei sorgenti congelati al lancio sono invariati. Le prime 25 righe dello storico sono identiche alla copia precedente alla ripresa. I digest di actor, componente media e critic ripristinati coincidono con quelli salvati alla 25 e con l'inizio della 26. Actor e critic vengono aggiornati in tutte le iterazioni; loss e KL sono finite. Nessuna telemetria mancante.

La ripresa resta una continuazione dello stato di apprendimento, non una replica bit per bit di un processo mai interrotto: gli episodi fisici dei worker ripartono. Il calo all'iterazione 26 può risentire di questo transitorio; non spiega da solo l'andamento delle iterazioni 36–40.

## Apprendimento e completamento degli episodi

| Intervallo | Ritorno medio delle iterazioni | Durata media delle iterazioni | Episodi completi | Penetrazione | Altre interruzioni |
|---|---:|---:|---:|---:|---:|
| 21–25 | 143,12 | 4,19 s | 19/51 = **37,3%** | 32 | 0 |
| 26–30 | 168,19 | 4,02 s | 19/40 = **47,5%** | 20, di cui 1 hard | 1 evento binario invalido |
| 31–35 | 191,40 | 4,33 s | 30/50 = **60,0%** | 18, di cui 1 hard | 2 eventi binari invalidi |
| 36–40 | 184,65 | 4,08 s | 19/49 = **38,8%** | 30, di cui 1 hard | 0 |

Le percentuali usano le differenze dei contatori cumulativi di fine episodio. Ritorno e durata sono medie dei valori per iterazione: non sono medie ponderate sugli stessi denominatori dei contatori. Le terminazioni per penetrazione includono `grf_penetration` e `grf_penetration_hard`.

Il training migliora fino alla zona 29–32, ma la stabilità non si consolida. Nella finestra mobile 28–32 il completamento raggiunge il 66,0%; alla 32 la durata media arriva a 4,81 s. Successivamente il completamento cala; l'iterazione 39 registra da sola 13 terminazioni per penetrazione. Non c'è una progressione regolare verso episodi tutti completi.

Nel blocco aggiuntivo 26–40 si contano 68 episodi completi su 139 registrati, 68 interruzioni per penetrazione e 3 eventi binari invalidi. Non compaiono timeout stance o swing. Gli eventi invalidi sono terminazioni dell'ambiente, non crash di processo.

## Confronto con il V26 a esperienza simile

Il confronto principale è **39D iterazioni 36–40 contro V26 iterazioni 29–33**. Le due finestre contengono esattamente 20.480 transizioni e sono traslate di soli 628 campioni lungo il budget cumulativo.

| Metrica | 39D, iterazioni 36–40 | V26, iterazioni 29–33 |
|---|---:|---:|
| Budget cumulativo | 143.360 → 163.840 | 142.732 → 163.212 |
| Transizioni nella finestra | 20.480 | 20.480 |
| Episodi completi | **19/49 = 38,8%** | **38/43 = 88,4%** |
| Terminazioni per penetrazione | **30** | **4** |
| Altre terminazioni | 0 | 1 timeout stance |
| Durata media delle iterazioni | **4,08 s** | **4,83 s** |
| Ritorno medio delle iterazioni | 184,65 | 195,66 |

Il divario di completamento è circa **49,6 punti percentuali**. Le terminazioni per penetrazione nella finestra sono 7,5 volte quelle storiche. **Questa differenza non si può spiegare con la minore quantità di esperienza**, a differenza del confronto iniziale basato soltanto sul numero di iterazioni.

Alla 32 della 39D, vicino al budget del V26 alla 25, il completamento delle ultime cinque iterazioni era 66,0% contro 79,4%. Quelle due finestre avevano ampiezze diverse, 20.480 contro 27.412 campioni, quindi il confronto era meno preciso. Alla 40 la corrispondenza dei budget è invece particolarmente buona e il divario rimane.

Le percentuali storiche restano quelle degli episodi registrati: alcuni guasti fatali possono non comparire nei contatori. Non è una stima di equivalenza statistica, essendo disponibile un solo seed. Ritorni simili non implicano stessa accuratezza o stesso rischio di terminazione, anche perché i target imitativi differiscono.

![Confronto a quantità di esperienza simile](<../../Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/training_by_samples.png>)

## Imitazione e stabilità stanno evolvendo diversamente

Le medie delle loss di posizione normalizzate nelle ultime cinque iterazioni scendono da 0,06287 a 0,03727 al ginocchio e da 0,04717 a 0,02423 alla caviglia, confrontando 21–25 con 36–40. Sono miglioramenti rispettivamente del 40,7% e del 48,6% delle loss, non del RMSE in gradi. Anche lo score imitativo medio TensorBoard cresce, da 0,6771 a 0,7824.

Contemporaneamente, fra i blocchi 31–35 e 36–40, le terminazioni per penetrazione passano da 18 a 30. La rete apprende una traiettoria mediamente più vicina al riferimento senza ottenere una stabilità altrettanto affidabile durante l'esplorazione. Questa coesistenza è osservata nei dati; la sua causa non è ancora isolata.

La telemetria delle nuove 15 iterazioni contiene 85 righe con risincronizzazione FSM, contro 48 nelle prime 25; nelle ultime cinque sono 25. Sono righe con evento, non una misura del numero di episodi distinti. Non risultano discrepanze sul lato del timeout o violazioni del contratto causale della morfologia. L'aumento segnala che le riparazioni della FSM continuano a essere utilizzate, senza dimostrare che siano la causa delle penetrazioni.

## Rollout del best29 e dell'ultimo checkpoint 40

Sono stati eseguiti due nuovi rollout OpenSim deterministici di 500 step, con output completi e tracce della policy. Entrambi terminano per limite temporale, senza interruzioni anticipate. I confronti 25/29/40 usano lo stesso start, lo stesso intervallo fisico e lo stesso target sinistro, verificato numericamente sulla traccia.

| Metrica | Iterazione 25 | Best29 | Last40 | V26 best87 |
|---|---:|---:|---:|---:|
| Durata | 5 s | 5 s | 5 s | 5 s |
| Ritorno deterministico | 260,52 | 177,57 | 222,15 | 363,79 |
| RMSE ginocchio–target | 7,58° | 6,41° | **6,16°** | 6,91° |
| RMSE caviglia–target | **5,45°** | 6,17° | 5,64° | 1,46° |
| Picco di flessione ginocchio | 42,66° | 43,96° | **44,58°** | 63,78° |
| Picco richiesto dal proprio target | 56,23° | 56,23° | 56,23° | 64,32° |
| Penetrazione massima | 17,88 mm | **20,81 mm** | 18,21 mm | 14,80 mm |
| RMSE giunto–reference, ginocchio | 0,27° | 0,43° | 0,27° | 0,18° |
| RMSE giunto–reference, caviglia | 0,32° | 0,50° | 0,41° | 0,28° |

Il V26 best87 è un riferimento finale allenato più a lungo e con un altro target, non un confronto cinematico a parità di esperienza. In particolare il RMSE del ginocchio inferiore al V26 non basta a dichiarare il suo superamento: la 39D manca ancora circa 11,65° al picco richiesto, mentre il V26 riproduceva quasi interamente il proprio picco.

Il ginocchio della 40 migliora del 18,8% in RMSE rispetto alla 25 e recupera parte dell'escursione, da 29,41° a 34,34° su 45,35° richiesti. Il miglioramento interessa soprattutto i tratti meno flessi: la parte 60–100% della fase prescritta conserva un errore di 9,21°.

La caviglia non migliora nel RMSE globale rispetto alla 25. Alla 40 resta fra **+1,73° e +27,57°**, contro un target fra **−6,61° e +22,05°**. Persiste la mancata riproduzione della parte negativa e compare un maggiore superamento del picco positivo. Gli estremi sono misurati sull'intero episodio, non necessariamente nello stesso istante.

Il best29 supera lievemente la soglia di penalizzazione della penetrazione, 20 mm, ma resta sotto quella di terminazione, 28 mm. La 40 resta sotto entrambe. Non si tratta di penetrazione geometrica nulla.

![Confronto cinematico 25/29/40](<../../Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/rollout_comparison.png>)

### Contatti e FSM nel rollout

Tutti e tre i checkpoint 39D completano due cicli riconosciuti, senza invalid event, cancellazioni HS o risincronizzazioni FSM nel rollout deterministico.

| Checkpoint | Durata dei due cicli | Quota di stance nei due cicli |
|---|---|---|
| 25 | 1,680 / 1,544 s | 38,1% / 81,0% |
| 29 | 1,676 / 1,518 s | 57,2% / 79,1% |
| 40 | 1,679 / 1,521 s | **54,6% / 56,8%** |
| V26 best87 | 1,687 / 1,486 s | 33,8% / 36,7% |

Alla 40 la ripartizione stance/swing è più omogenea tra i due cicli. È un miglioramento concreto rispetto alla 25. I valori provengono dal rilevatore/FSM del simulatore: la maggiore omogeneità non costituisce una validazione clinica del passo, né implica l'identità dei contatti con il V26.

## Una causa misurabile della perdita di ritorno: momento di contatto alla caviglia

La ricostruzione della reward dai singoli step spiega perché il ritorno deterministico della 40 sia inferiore a quello della 25 nonostante il miglioramento del ginocchio:

| Componente cumulativa sui 500 step | Iterazione 25 | Best29 | Last40 |
|---|---:|---:|---:|
| Reward base, già inclusiva delle penalità condivise | 260,54 | 247,54 | 262,64 |
| Penalità sul segno del momento GRF alla caviglia | ≈0,00 | **69,85** | **40,43** |
| Penalità diretta di penetrazione | 0,00 | 0,07 | 0,00 |
| Penalità fuori intervallo | 0,02 | 0,05 | 0,07 |
| Reward finale | **260,52** | **177,57** | **222,15** |

La penalità in questione era già presente nella reward V26 ed è rimasta invariata: peso 0,1, tolleranza 8 Nm, attivazione sotto carico normale di almeno 50 N. Penalizza il **momento della forza di contatto rispetto all'asse della caviglia** quando è negativo oltre la tolleranza. Non è direttamente una penalità sulla coppia comandata al SEA.

Nel best29 è positiva in 172 step su 500; nella 40 in 126 su 500. Al picco, nella 40 a circa 17,53 s assoluti, il momento GRF è −36,36 Nm con forza normale circa 538 N: la penalità non è soltanto un piccolo attraversamento numerico della soglia. La variazione di questo termine spiega aritmeticamente la perdita di ritorno della 40 rispetto alla 25. Non dimostra da sola quale cambiamento di policy abbia prodotto quel contatto, né che il termine sia sbagliato.

![Momento GRF e penalità](<../../Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/contact_moment_comparison.png>)

## Diagnosi e prossimo passo

**Apprendimento presente, qualità ancora incompleta.** Il ginocchio migliora e la 40 regolarizza stance/swing, ma la caviglia rimane lontana dal riferimento. L'errore è prevalentemente già nella reference servita: alla 40 il RMSE reference–target è 6,14°/5,62°, mentre quello giunto–reference è 0,27°/0,41°. Non emerge un difetto dominante di inseguimento SEA. Il dato non separa da solo il contributo della policy da quello del governor.

**Clock e prescribed funzionano e sono utilizzati.** Tutti i sei canali sono verificati nei due rollout. Il ricalcolo offline riproduce le azioni registrate entro 9,54×10⁻⁷. Azzerare clock o velocità prescribed modifica le azioni; non emerge un input ignorato o accidentalmente oscurato. Queste sonde misurano sensibilità offline, non capacità di camminare con ingressi oscurati.

**Esplorazione ancora ampia.** Sugli stati visitati nel rollout della 40 le deviazioni standard delle azioni sono circa 0,86 e 0,93, contro 0,75 e 0,89 alla 25. La probabilità gaussiana stimata di uscire dai limiti dell'azione è circa 36% e 29%. Il completamento deterministico insieme alle molte terminazioni esplorative rende questa una priorità diagnostica, ma non dimostra che spieghi tutte le penetrazioni: sono state eseguite soltanto valutazioni fisiche deterministiche in questa analisi.

**Il best di training non basta per scegliere il checkpoint da usare.** Il best29 è peggiore della 40 in entrambi i RMSE, penetrazione massima e regolarità stance/swing del rollout. La 25 conserva però il ritorno deterministico più alto e una caviglia leggermente più accurata. Non c'è un checkpoint che prevalga su ogni criterio. Tutti rimangono conservati.

Il prossimo test più informativo è una valutazione controllata dei checkpoint 25/29/40 allo stesso start con azioni deterministiche e stocastiche, usando seed di azione comuni e registrando fase, penetrazione e momento GRF nei tratti problematici. Questo distinguerebbe una policy media relativamente stabile da una distribuzione di esplorazione fragile. In parallelo andrebbe confrontata la geometria del contatto con il target sinistro nei tratti che attivano la penalità sul momento, prima di modificarla. L'analisi attuale non giustifica attribuire il problema alla sola FSM o eliminare automaticamente una penalità storica.

Il test a 40 iterazioni è quindi informativo: ha evidenziato progressi cinematici e un divario di robustezza che persiste a parità di esperienza. Un'estensione lunga del training, da sola, non isolerebbe la causa. Nessun ulteriore training è stato avviato.

## Artefatti e verifiche eseguite

- [Metriche complete e audit riproducibile](<../../Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/metrics.json>).
- [Script di analisi](<../../Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/analyze.py>).
- [Rollout best29](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29/rollout_summary.json>).
- [Rollout last40](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40/rollout_summary.json>).
- [Elenco verificato dei sette grafici best29](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29/plots/plot_receipt.json>).
- [Elenco verificato dei sette grafici last40](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40/plots/plot_receipt.json>).

I sette grafici canonici della 40:

1. [Comandi SEA e reserve](<../../plot/09_21_2026_1/01_time_sea_control_reserve.png>).
2. [Stati dei giunti e dei motori](<../../plot/09_21_2026_1/02_time_joint_motor_states.png>).
3. [Coppia, angolo e potenza nel ciclo](<../../plot/09_21_2026_1/03_gaitcycle_torque_angle_power.png>).
4. [Velocità e potenza nel ciclo](<../../plot/09_21_2026_1/04_gaitcycle_joint_velocity_power.png>).
5. [Coppia in ingresso ed errore di tracking](<../../plot/09_21_2026_1/05_time_tau_input_tracking_error.png>).
6. [Giunti, reference e tracking SEA](<../../plot/09_21_2026_1/06_time_joint_ref_sea_error.png>).
7. [Imitazione del prescribed sinistro](<../../plot/09_21_2026_1/07_mlp_policy_vs_sound_leg_error.png>).

Sono stati aggiunti soltanto script e artefatti dell'analisi nella cartella dedicata, i due rollout con 14 grafici canonici complessivi e questo report. Il codice di training, la FSM, il plugin e la semantica SEA non sono stati modificati.

Verifiche: audit di tutte le 40 iterazioni, integrità della ripresa e dei sorgenti, 163.840 transizioni visibili, due rollout completi di 500 step, uguaglianza del target sinistro tra 25/29/40, ricostruzione numerica della reward, riproduzione delle azioni, generazione e controllo degli artefatti grafici. Un solo seed di training e un solo start di valutazione non consentono una conclusione di equivalenza statistica o una diagnosi causale definitiva.
