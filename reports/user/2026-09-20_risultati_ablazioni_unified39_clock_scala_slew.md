# Risultati delle ablazioni Unified MLP 39D: clock, scala delle velocità e slew limiter

Data: 20 settembre 2026. Instruction check token: CMC_AGENT_OK_2026.

## 1. Esito e decisione

**I tre training sono terminati correttamente, ma nessuna delle tre varianti recupera la qualità imitativa del V26 del 20 agosto.** Sono state completate 300 iterazioni PPO, 1.382.400 transizioni di training, nove rollout deterministici dei checkpoint best e 21 grafici canonici. La sequenza è finita il 20 settembre alle 09:29, dopo circa 45 ore e 13 minuti.

La variante **B, con velocità prescribed in rad/s senza divisione per 10 e slew mantenuto**, è la candidata più utile per proseguire l'indagine: completa due episodi su tre e migliora entrambi gli errori cinematici su tutte le finestre confrontabili. Non è però una soluzione soddisfacente: il ginocchio continua a flettersi molto meno del target, la caviglia mostra sovraelongazione e il terzo start termina anticipatamente.

La reintroduzione del clock, da sola, **non è sufficiente**. La rimozione dello slew, nella configurazione provata, **peggiora il risultato deterministico**. Questi risultati non dimostrano un limite della capacità rappresentativa dell'architettura 39D: dimostrano che le tre ricette di apprendimento e selezione del best provate non hanno raggiunto l'obiettivo.

**Non avvierei ancora l'oscuramento progressivo.** Prima serve una policy con tutti e sei gli ingressi prescribed visibili che riproduca bene il target e completi tutti gli start richiesti. Questi esperimenti sono diagnostici a seed singolo; non costituiscono una dimostrazione di equivalenza statistica.

## 2. Domanda sperimentale e protocollo

L'obiettivo era capire se il cattivo risultato della prima 39D dipendesse dall'assenza del metronomo esplicito, dalla scala delle velocità osservate o dalla limitazione degli endpoint dei comandi. Le differenze già accettate dall'utente — target sinistro invece del destro sfasato e penalità aggiuntive — sono state mantenute.

In tutte le nuove varianti, i due slot iniziali contengono sin/cos della fase prescribed **sinistra**, senza shift. La stessa maschera episodica controlla questi due ingressi e i quattro q/qdot prescribed: indici 0, 1, 35, 36, 37, 38. La disponibilità è stata sempre 100%, senza curriculum. L'actor resta 39D, l'osservazione completa del critic 88D. Il clock destro condiviso resta disabilitato: il flag `gait_clock_enable: false` non significa che il nuovo clock sinistro dell'actor sia spento.

| Variante | Divisore degli ingressi qdot | Slew ginocchio/caviglia | Clock sinistro |
|---|---|---|---|
| A — baseline_phase | 10 | 2,5 / 2,0 rad/s | visibile |
| B — velocity_scale1 | 1 | 2,5 / 2,0 rad/s | visibile |
| C — no_slew | 10 | disattivato | visibile |

Le tre reti partono da pesi nuovi identici, seed 123, MLP 256×256 tanh. Ogni run esegue 100 iterazioni, 12 worker, 4.608 transizioni per iterazione, minibatch 512 e 10 epoche PPO. Ogni batch comprende esattamente 1.536 transizioni per ciascuno dei tre start. Il governor Butterworth di terzo ordine a 4 Hz e i suoi limiti successivi rimangono attivi anche nella variante senza slew.

Gli start, espressi come offset, sono early = 1,756870983805102 s, center = 1,956870983805102 s e late = 2,156870983805102 s. Sono gli stessi start utilizzati nel training: questo è un controllo di riproduzione e robustezza su condizioni note, non una misura di generalizzazione a nuovi start.

Il best è selezionato dal massimo return medio registrato nel training. La valutazione successiva è deterministica, per un massimo di 500 step da 10 ms, ossia 5 s. Per ciascuna variante sono disponibili tre rollout; i sette grafici canonici si riferiscono allo start centrale.

## 3. Integrità tecnica e andamento del training

La rilettura indipendente degli artefatti conferma, per ciascun run: 100 iterazioni consecutive senza skip; nessun riavvio del supervisore; batch e aggiornamenti learner bilanciati; loss PPO finite; pesi iniziali actor/critic identici fra le varianti; checkpoint best coerente con il milestone indicato. I 100 file del manifest di sorgenti/configurazioni/dati risultano ancora corrispondenti agli hash congelati. Le configurazioni B e C differiscono da A solo per il fattore dichiarato, esclusa la provenienza descrittiva.

Le terminazioni fisiche degli episodi sono distinte dagli errori infrastrutturali: un run può finire tutte le sue iterazioni pur raccogliendo numerosi episodi terminati anticipatamente.


| Variante | Best iter | Return best training | Durata media episodi al best [step] | Durata training [h] |
|---|---|---|---|---|
| A — Clock + scala 10 + slew | 99 | 295,47 | 491,1 | 14,97 |
| B — Clock + scala 1 + slew | 78 | 281,15 | 468,7 | 14,65 |
| C — Clock + scala 10, senza slew | 84 | 241,68 | 500,0 | 14,96 |


| Variante | Return medio ultime 20 iter | Durata media ultime 20 iter [step] | Return iter 100 |
|---|---|---|---|
| A — Clock + scala 10 + slew | 257,09 | 430,5 | 225,15 |
| B — Clock + scala 1 + slew | 225,48 | 373,0 | 229,90 |
| C — Clock + scala 10, senza slew | 212,67 | 475,6 | 214,91 |


Le medie di questa seconda tabella sono medie aritmetiche delle metriche per iterazione; nel JSON sono disponibili anche medie pesate per il numero di episodi completati. Le due statistiche non vanno confuse.

La B raggiunge il best alla 78 e nelle ultime 20 iterazioni peggiora rispetto al blocco precedente: prolungare semplicemente il training non garantisce recupero. La C arriva a episodi di training mediamente lunghi, ma questo non si traduce in una buona policy deterministica. Non emerge un collasso numerico del PPO: il problema riguarda il comportamento appreso e il modo di valutarlo.

Un confronto particolarmente istruttivo: il return medio delle ultime 20 iterazioni di A è **257,09**, quasi uguale ai **257,43 del V26 storico**, ma la durata media è 430,46 step contro 493,60 e la qualità cinematica resta nettamente inferiore. Sono valori verificati nei rispettivi log; la loro somiglianza non dimostra parità degli obiettivi raggiunti.


![Curve dei tre training, metriche grezze e media mobile di 10 iterazioni](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/training_curves.png>)


## 4. Risultati di tutti i rollout

RMSE è la radice della media degli errori quadratici fra angolo simulato e **target proprio** del rollout, in gradi. Per gli episodi interrotti viene calcolato solo il tratto realmente osservato: non è un errore su cinque secondi e non consente di giudicare il tratto mancante.


| Variante | Start | Durata [s] | RMSE ginocchio [°] | RMSE caviglia [°] | Return | Esito |
|---|---|---|---|---|---|---|
| baseline_phase | early | 5,00 | 13,54 | 8,51 | 267,50 | 5 s completati |
| baseline_phase | center | 1,70 | 14,48 | 6,27 | 110,43 | limite superiore ginocchio |
| baseline_phase | late | 1,48 | 14,54 | 6,86 | 91,16 | limite superiore ginocchio |
| velocity_scale1 | early | 5,00 | 12,95 | 8,93 | 163,60 | 5 s completati |
| velocity_scale1 | center | 5,00 | 12,39 | 7,46 | 217,66 | 5 s completati |
| velocity_scale1 | late | 1,51 | 14,09 | 5,52 | 85,80 | limite superiore ginocchio |
| no_slew | early | 1,89 | 16,79 | 9,28 | 91,84 | limite superiore ginocchio |
| no_slew | center | 1,71 | 16,47 | 8,41 | 85,10 | limite superiore ginocchio |
| no_slew | late | 1,49 | 18,85 | 9,75 | 71,01 | limite superiore ginocchio |


La A completa **1/3** episodi, la B **2/3**, la C **0/3**. Sono conteggi descrittivi di tre reset deterministici, non stime affidabili di una probabilità di successo. La durata media sui tre start è rispettivamente 2,73 s, 3,84 s e 1,70 s.

I grafici seguenti mostrano target, reference servita e angolo simulato. L'area grigia indica il tratto non osservato dopo la terminazione: nessuna curva viene estrapolata oltre lo stop.


![Ginocchio: tutte le varianti e tutti gli start](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/knee_all_starts.png>)

![Caviglia: tutte le varianti e tutti gli start](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/ankle_all_starts.png>)


## 5. Confronto a parità di finestra temporale

Per evitare il vantaggio apparente degli episodi brevi, per ogni start tutte le varianti sono confrontate sul prefisso comune più lungo: 1,89 s early, 1,70 s center, 1,48 s late. Il target è stato verificato numericamente identico fra le tre reti negli stessi istanti.


| Start | Finestra [s] | A: ginocchio / caviglia [°] | B: ginocchio / caviglia [°] | C: ginocchio / caviglia [°] |
|---|---|---|---|---|
| early | 1,89 | 13,66 / 7,79 | 10,80 / 5,79 | 16,79 / 9,28 |
| center | 1,70 | 14,48 / 6,27 | 11,58 / 4,82 | 16,50 / 8,43 |
| late | 1,48 | 14,54 / 6,86 | 14,14 / 5,58 | 18,89 / 9,78 |


Rispetto ad A, B riduce l'RMSE del ginocchio di circa **21%, 20% e 3%** e quello della caviglia di **26%, 23% e 19%** sui tre prefissi. Il beneficio della scala 1 è quindi presente anche con durata controllata. C peggiora entrambi i giunti in tutti e tre i confronti.

Questo risultato resta locale ai tratti condivisi. Sull'intero episodio early, che A e B completano entrambe, B migliora il ginocchio solo del 4,4% e peggiora la caviglia del 4,9%: il vantaggio iniziale non si conserva uniformemente per tutti i cinque secondi.

## 6. Cosa significa `joint_divergence` in questi rollout

Il controllo in `osim_trj_cmc_like.py` termina l'episodio se il ginocchio simulato esce da **[-2,0; 0,0] rad**. Nei sei rollout interrotti il ginocchio attraversa il limite superiore, con valori finali fra **+0,0246° e +0,1519°**. Non si osserva una divergenza di ampiezza enorme o uno stato non finito: l'etichetta corrisponde qui a un piccolo superamento dell'estensione zero.

Anche la reference servita attraversa lo zero nei casi interrotti. Il controller SEA segue dunque una reference già troppo estesa; il target prescribed a quel punto richiede ancora circa **11–12° di flessione**. Il piccolo sforamento della guardia e il grande errore rispetto al target sono due fatti diversi.

Questa scoperta chiarisce le durate corte ma **non assolve l'imitazione**: prima dello stop il ginocchio ha già mancato la flessione richiesta. Allargare la soglia può allungare il rollout senza rendere migliore la traiettoria. Un'eventuale revisione della guardia richiede una decisione esplicita sul significato fisico del limite e una nuova misura degli errori; non è stata applicata in questa analisi.

## 7. Confronto con il V26 imitativo del 20 agosto

Il riferimento corretto rimane `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`, best 87, confermato dai report del 20 agosto. È il training imitativo, non il successivo ex-novo. Per un confronto senza episodi troncati si usano il rollout storico V26, il precedente 39D best54 e B sullo start centrale: tutti durano 5 s.


| Policy | RMSE ginocchio [°] | RMSE caviglia [°] | Correlazione ginocchio | Correlazione caviglia | Max flessione ginocchio: actual / target [°] |
|---|---|---|---|---|---|
| V26 agosto, best87 | 6,91 | 1,46 | 0,936 | 0,973 | 63,78 / 64,32 |
| 39D precedente, clock nullo, best54 | 14,21 | 8,49 | 0,389 | 0,427 | 25,67 / 56,24 |
| 39D B, clock e scala1, best78 | 12,39 | 7,46 | 0,698 | 0,862 | 28,92 / 56,24 |


B migliora rispetto al precedente 39D centrale, ma resta a circa **1,79 volte l'errore del ginocchio e 5,11 volte quello della caviglia del V26**. Normalizzando descrittivamente l'RMSE per l'escursione del proprio target, il divario rimane: circa 27,3% contro 12,9% per il ginocchio e 26,0% contro 7,2% per la caviglia. Questa normalizzazione descrittiva non cambia le loss di training.

Il ginocchio B raggiunge circa 28,92° contro 56,24° richiesti; nella porzione 60–100% della fase prescribed l'RMSE è circa 18,01°, contro 4,89° nello storico V26. La caviglia B arriva a +34,85° contro un massimo target di +22,05° e manca il minimo target negativo: non basta una correlazione relativamente alta per garantire ampiezza e offset corretti.

Le differenze dei dati prescribed sono accettate e non vengono considerate un bug: ogni rete è confrontata con il proprio obiettivo. Il confronto storico resta però **descrittivo**: V26 ha target destro sfasato, start fisso a offset 1,0 s, batch 4.096 e 13 worker; le nuove reti hanno target sinistro, tre start, batch 4.608 e 12 worker. Il precedente 39D ha inoltre una storia di ripresa dal best diversa dai tre run nuovi. Non è possibile attribuire ogni differenza storica al solo clock.


![Confronto dei tre rollout completi: V26, precedente 39D, nuova variante B](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/historical_comparison.png>)


## 8. Localizzazione del problema: policy/reference oppure SEA?

Sul rollout centrale completo di B:

| Metrica | Ginocchio | Caviglia |
|---|---|---|
| RMSE giunto–target | 12,39° | 7,46° |
| RMSE reference servita–target | 12,49° | 7,43° |
| RMSE giunto–reference servita | 0,55° | 0,59° |

La maggior parte del difetto è già nella **reference prodotta dalla policy attraverso il percorso di comando**. Il tracking SEA contribuisce molto meno. Questo rende poco giustificato partire modificando il plugin o i suoi guadagni per risolvere questi errori imitativi. Non esclude problemi fisici secondari, ma localizza il difetto dominante nelle tracce osservate.

### Il clock e i target vengono letti?

Sì: il controllo su tutti i nove trace verifica la presenza dei sei ingressi, la norma unitaria del clock e la corrispondenza fra target/phase della reward allo step precedente e osservazione prima dello step successivo. Non emerge uno scambio di canali né uno sfasamento software di uno step. Le azioni deterministiche sono state ricostruite caricando i checkpoint reali, con errore massimo inferiore a 8×10⁻⁷ in azione normalizzata.

Per verificare la dipendenza dagli ingressi, sono state eseguite sonde offline: si congelano gli altri ingressi osservati e si azzera un gruppo. La tabella riporta la variazione RMS della media dell'azione, in unità normalizzate, ginocchio/caviglia.


| Policy | Quattro target azzerati | Clock azzerato | Sole velocità target azzerate |
|---|---|---|---|
| V26 | 0,765 / 0,525 | 0,192 / 0,227 | 0,762 / 0,523 |
| 39D precedente | 0,015 / 0,046 | 0,000 / 0,000 | 0,012 / 0,015 |
| baseline_phase | 0,087 / 0,080 | 0,232 / 0,425 | 0,075 / 0,071 |
| velocity_scale1 | 0,275 / 0,376 | 0,158 / 0,252 | 0,276 / 0,373 |
| no_slew | 0,058 / 0,025 | 0,151 / 0,252 | 0,021 / 0,011 |


Il clock reintrodotto è effettivamente usato. In B cresce anche la dipendenza dalle velocità prescribed, coerentemente con il beneficio cinematico parziale osservato. La baseline A dipende maggiormente dal clock che dai quattro target; la C rimane poco sensibile alle velocità. Un metronomo utilizzato dalla rete non garantisce quindi che venga riprodotta la forma corretta.

Le sonde usano osservazioni visitate da policy e durate diverse e possono generare ingressi fuori distribuzione. Non equivalgono a rollout con feature oscurate, non dimostrano autonomia della rete e non provano da sole una causa del mancato apprendimento.

## 9. Perché return di training e qualità deterministica divergono

**Selezione del checkpoint.** Il trainer misura il return durante `algo.train()`, quindi salva i pesi dopo l'aggiornamento PPO. Il valore usato per eleggere il best proviene dal campionamento esplorativo; non è un rollout deterministico indipendente del checkpoint appena salvato. Le due misure non sono intercambiabili. Il best C aveva episodi di training mediamente da 500 step, ma i suoi tre rollout deterministici durano solo 149–189 step.

**Esplorazione e limitazioni dei comandi.** Le deviazioni standard gaussiane medie, valutate sulle osservazioni dei rollout centrali, sono A = 1,11/1,04, B = 1,77/2,18, C = 0,56/0,39 in azione normalizzata. In A e B lo slew modifica circa 85–92% dei comandi deterministici dei due giunti. Condizionatamente agli stessi stati, il modello gaussiano indica che circa 97% dei campioni sarebbe modificato dal clipping assoluto e/o dallo slew.

Il comando deterministico medio seguito dalle limitazioni non coincide in generale con la media dei comandi limitati durante l'esplorazione. È una motivazione concreta per verificare il divario fra raccolta PPO e deployment. Non è però una spiegazione unica già dimostrata: anche il V26 storico aveva esplorazione ampia e clipping, pur ottenendo una buona imitazione. Le percentuali gaussiane sono calcoli sulle osservazioni deterministiche, non frequenze misurate nei batch di training.

**Reward e qualità cinematica.** Il ricalcolo della reward ricostruisce i return dei nove nuovi rollout e dei due storici entro 10⁻⁶. Non emerge una reward applicata diversa da quella configurata. La parte imitativa usa ancora la combinazione 80% served e 20% actual; la scala degli ingressi qdot cambia le osservazioni, non la normalizzazione della loss.

Il return può cambiare anche per costi che non coincidono con l'RMSE. Per B centrale, circa 43,63 punti sono sottratti dalla penalità sul momento GRF alla caviglia, 3,00 dalla penetrazione e 9,81 dal costo di variazione dei comandi, quest'ultimo incluso nella penalità interna alla parte positiva. La penetrazione massima è 24,12 mm; sullo start early arriva a 25,33 mm, sotto la terminazione a 28 mm ma sopra l'inizio della penalizzazione a 20 mm. B completa più episodi senza essere la migliore in tutti gli aspetti fisici.

Non va quindi scelto il prossimo modello soltanto dal return: servono insieme durata, errore cinematico per giunto/fase e qualità del contatto. Le metriche sugli episodi terminati devono restare esplicitamente marcate come parziali.

## 10. Risposta alle tre ipotesi e limiti delle conclusioni

| Ipotesi | Evidenza ottenuta | Conclusione sostenibile |
|---|---|---|
| Mancava il metronomo esplicito | Il clock sinistro è visibile, corretto e influenza l'azione; A continua a imitare male | Reintrodurlo da solo non basta; non è stato isolato il suo effetto con una replica fresh senza clock nello stesso nuovo protocollo |
| La divisione delle velocità per 10 ostacolava l'apprendimento | B usa di più le velocità e migliora tutti i prefissi comuni; completa 2/3 rollout | Scala 1 promettente per il seguito, ma recupero solo parziale e a un seed |
| Lo slew era la causa sufficiente del difetto | C elimina lo slew ma peggiora RMSE e fallisce 3/3 rollout | Spegnerlo da solo non risolve, nella ricetta provata |
| La rete 39D è incapace di imitare V26 | Sono stati provati tre training di un seed con diversi fattori ancora non isolati | Non dimostrato: il risultato riguarda le ricette provate, non una impossibilità dell'architettura |

La combinazione **scala 1 + slew disattivato non è stata provata**: non si può dedurne il risultato dai due esperimenti separati. Neppure l'oscuramento progressivo è stato testato: la maschera è pronta e verificata, ma durante questi training è sempre rimasta aperta.

Non sono stati eseguiti test di equivalenza statistica: mancano repliche di training con seed indipendenti, margini di equivalenza concordati e una valutazione comune sufficientemente ampia. I 500 campioni temporali di un rollout sono autocorrelati e non sostituiscono 500 repliche indipendenti; i tre start condividono policy e tratto di cammino.

## 11. Prossimi controlli consigliati, in ordine

1. **Verificare la fattibilità del target sinistro con un oracle fisico**, sui tre start e con la stessa dinamica: comando prescribed per fase, con anticipazione controllata ove necessaria per compensare il filtro. Il precedente replay offline del governor non bastava a dimostrare fattibilità nel loop OpenSim. Questo separa un problema di obiettivo/percorso di comando da un problema di apprendimento.
2. **Valutare i checkpoint con rollout deterministici dedicati**, includendo durata, RMSE, errore nella flessione e contatto. Confrontare una piccola selezione di milestone e, separatamente, rollout con azioni campionate a seed fissati. Questo chiarisce se il best per return PPO sia rappresentativo e quanto conti il passaggio esplorazione→azione deterministica. Tenere poi un insieme di valutazione separato per non confondere selezione e verifica finale.
3. **Usare B come riferimento per le prossime prove**, mantenendo clock e sei ingressi sempre visibili. Dopo la verifica dell'oracle, isolare una sola ulteriore differenza: per esempio apprendimento su un solo start rispetto ai tre, oppure gestione dell'esplorazione vicino ai limiti. Non cambiare contemporaneamente target, reward, start e governor.
4. **Chiarire il significato della guardia a 0°**, senza spacciare un eventuale allargamento per recupero imitativo. Mantenere la distinzione fra violazione articolare, durata del rollout ed errore rispetto al target.
5. Solo dopo una baseline affidabile, introdurre il curriculum della maschera comune a clock e target. Per una dichiarazione di parità con V26 serviranno seed indipendenti e criteri fissati prima del confronto.

Questi passaggi sono raccomandazioni; **non sono stati avviati nuovi training, nuovi rollout fisici o modifiche della reward durante questa analisi**.

## 12. Artefatti, file e verifiche riproducibili

La modifica sperimentale già realizzata comprende il contratto v3, il clock sinistro con maschera comune, la scala qdot configurabile e i controlli di checkpoint. Interessa il perimetro `unified_MLP_39/`: ambiente, disponibilità target, factory, modulo actor/critic, contratti, configurazione/runtime di training, rollout e test. Le tre YAML e l'orchestratore sono nella cartella del piano. Non sono state cambiate la semantica SEA o il plugin C++.

In questa fase di resoconto sono stati aggiunti soltanto `analyze_results.py`, `write_report.py`, `metrics.json`, `input_hashes.json`, la ricevuta `report_verification.json`, quattro figure riassuntive e questo report. I run e i loro risultati originali sono stati letti e preservati.

Verifiche svolte per il resoconto: audit delle 300 iterazioni e degli hash congelati; confronto delle tre configurazioni; controllo dei nove trace e dei sei ingressi; ricalcolo indipendente degli RMSE; identità dei target nei prefissi comuni; ricostruzione di 11 return; caricamento di cinque checkpoint e riproduzione delle azioni; verifica dei 21 PNG canonici. Le quattro nuove figure sono state ispezionate visivamente. I precedenti 55 test unitari e smoke di salvataggio/ripresa restano documentati nella ricevuta di preflight, senza sostituire queste verifiche sui run completi.


- [Metriche complete e risultati degli audit](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/metrics.json>)
- [Hash degli artefatti analizzati](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/input_hashes.json>)
- [Script di analisi e figure](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/analyze_results.py>)
- [Generatore del report](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/write_report.py>)
- [Protocollo e configurazioni](<../../Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/README.md>)
- [Ricevuta preflight](<../../Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/preflight.json>)
- [Diagnosi precedente al ciclo di ablazioni](<../../Trajectory Generator/unified_MLP_39/validation/diagnosis_v26_2026-09-18/diagnosi.md>)
- [Identificazione del riferimento V26 del 20 agosto](<2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md>)


### Sette grafici canonici per variante


- A — Clock + scala 10 + slew: [cartella dei sette grafici](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_baseline_phase_2026-09-18/center/plots/09_19_2026_1>); [grafico 7, policy e target sinistro](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_baseline_phase_2026-09-18/center/plots/09_19_2026_1/07_mlp_policy_vs_sound_leg_error.png>).

- B — Clock + scala 1 + slew: [cartella dei sette grafici](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_velocity_scale1_2026-09-18/center/plots/09_19_2026_1>); [grafico 7, policy e target sinistro](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_velocity_scale1_2026-09-18/center/plots/09_19_2026_1/07_mlp_policy_vs_sound_leg_error.png>).

- C — Clock + scala 10, senza slew: [cartella dei sette grafici](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_no_slew_2026-09-18/center/plots/09_20_2026_1>); [grafico 7, policy e target sinistro](<../../Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_no_slew_2026-09-18/center/plots/09_20_2026_1/07_mlp_policy_vs_sound_leg_error.png>).


Il nome storico del file 07 contiene ancora `sound_leg`; il renderer di queste prove usa il target sinistro effettivamente registrato e lo etichetta come tale. I pannelli di ciclo eventualmente privi di un ciclo completo non costituiscono evidenza di assenza di errore.

Per rigenerare le misure e poi questo report, dalla radice del repository con l'ambiente Python del progetto:

```text
python "Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/analyze_results.py"
python "Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/write_report.py"
```

Il controllo degli hash deve passare: se i sorgenti vengono modificati dopo l'esperimento, non va ignorata la discrepanza; occorre usare lo snapshot registrato o documentare la nuova provenienza. L'analisi è stata eseguita su macOS arm64; gli script usano percorsi Python portabili, ma non è stata eseguita una validazione live su Windows.

