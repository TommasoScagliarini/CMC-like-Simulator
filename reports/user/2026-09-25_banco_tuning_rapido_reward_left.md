# Banco di tuning rapido della reward LEFT: implementazione, analisi offline e pilot predisposto

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 25 settembre 2026. Instruction check token: `CMC_AGENT_OK_2026`.

## Stato e punto di ripartenza

Implementato il banco riutilizzabile sotto `Trajectory Generator/unified_MLP_39/`.
Sono disponibili analisi di **17 rollout**, **9 candidate calibrate**, figure,
configurazioni dei due rami e lanciatore del pilot. La preparazione non ha
eseguito nuovi aggiornamenti PPO o rollout completi.

Al controllo per questo report il pilot è ancora **`prepared`**, con
`elapsed_seconds=0`, `stages={}` ed `evaluation={}`. Il confronto è `incomplete`:
mancano tutte le 12 valutazioni previste. Le directory dei due nuovi training
sono assenti. Il successivo messaggio utente «avvialo» è stato seguito
dall'interruzione del turno e dalla richiesta `create_report`; non risulta
un avvio eseguito prima dell'interruzione. Questo documento distingue tale
richiesta dallo stato effettivo: **nessun risultato di apprendimento del pilot
è disponibile**. La redazione del report non avvia il training.

Il riferimento resta **LEFT best80 con penetrazione peso 2**, dal run
`MLP39_LEFT72_penetration_weight2_10iter_2026-09-24/checkpoint_best`.
La prosecuzione storica da best80 fino100 non aveva migliorato il nominale;
robustezza e qualità delle coppie restano da consolidare.

## Problema e strategia concordata

Modificare pochi pesi e ripetere subito100 iterazioni richiede ore senza
isolare quale intervento sia utile. Il banco separa tre domande:

1. Come cambia la valutazione di comportamenti già osservati? Ricalcolo offline.
2. La modifica produce aggiornamenti utili della policy? Pilot breve controllato.
3. Il vantaggio è stabile e si riproduce imparando da zero? Verifica successiva
   dei candidati promettenti, fuori dalla prima consegna.

L'ordine fissato dall'utente, rispettando tracking e contatto, è:

1. Ridurre tutte le porzioni positive della coppia SEA ankle nell'intero ciclo.
2. Migliorare la regolarità della coppia SEA knee.
3. Migliorare la regolarità cinematica ankle/knee, osservando movimento fisico
   e reference servita; la nuova penalità agisce inizialmente sul movimento.

La coppia positiva è un obiettivo esplicito. Non viene interpretata come
equivalente a forza verticale negativa: il nominale best80 aveva circa+29Nm
di coppia SEA con carico positivo di460N e momento GRF sfavorevole.

## Soluzione implementata

Il comando `tune_reward.py` espone `analyze`, `prepare`, `run`, `compare`.
I primi due producono analisi e configurazioni; soltanto `run` esegue il pilot.
`compare` applica vincoli fisici, senza confrontare ritorni di reward diverse
come criterio di successo. Manifest e hash conservano input e provenienza.

| Famiglia opt-in | Loss | Scala iniziale |
|---|---|---|
| Ankle positiva | media di `max(tau_spring,0)` sui substep / scala | 30Nm |
| Regolarità knee | quadrato di RMS rateo coppia / scala | 1000Nm/s |
| Cinematica knee e ankle | quadrato di jerk effettivo / scala, pesi separati | 1000rad/s³ |

I nuovi termini hanno pesi predefiniti zero, sono sottratti dopo il clipping,
non hanno cap nascosti e sono attivi soltanto in imitazione. Le scale sono
normalizzazioni esplicite, non limiti fisiologici. La reward storica con pesi
zero è preservata. Safety, contatto, controller e semantica SEA restano invariati.

Il runner pubblica soltanto nuove statistiche diagnostiche della coppia
positiva dai campioni già disponibili. Nessuna logica di rete è aggiunta al
root e il plugin C++ non è modificato. Gli STO registrano prima dell'integrazione:
la ricostruzione usa gli intervalli `[inizio,fine)` e verifica numero e RMS
dei campioni. Il rateo knee nella reward esclude i confini fra segmenti,
come la diagnostica originale; la metrica dell'intero STO li include.

Il jerk usa velocità a100Hz e differenze causali fra accelerazioni medie di
intervallo. Reset, warm-up, input mancanti e segmenti hard incompleti sono
espliciti; non si riutilizzano statistiche vecchie. I filtri diagnostici sono
Butterworth ordine4 a fase zero, con tagli10/20Hz per le coppie e10Hz per
la cinematica. Nel confronto dei prefissi comuni il futuro viene escluso
prima del filtro, poi si ritaglia la finestra di analisi.

## Analisi offline e risultati

Il corpus contiene cinque nominali LEFT (best72, best80, precedente last82,
nuovo best82 e last100), dieci prove rumorose best72 e due confronti descrittivi
V26/RIGHT. Questi ultimi hanno target differenti e non sono controlli LEFT.

| Nominale LEFT, finestra13,15–17,95s | RMSE knee/ankle [°] | Area positiva ankle [Nm·s] | RMS knee HP10/20 [Nm] |
|---|---:|---:|---:|
| Best72 | 6,606 / 6,074 | 25,022 | 6,633 / 4,114 |
| Best80 | 6,376 / 4,040 | 8,608 | 8,197 / 3,784 |
| Precedente last82 | 5,693 / 5,326 | 21,976 | 7,562 / 3,645 |
| Nuovo best82 | 6,629 / 5,508 | 16,737 | 7,810 / 3,454 |
| Last100 | 6,737 / 5,729 | 16,147 | 9,203 / 3,400 |

Le differenze minime rispetto a precedenti tabelle dipendono anche dalla
gestione esplicita della tolleranza ai bordi temporali. Non sono nuovi rollout.

Le candidate variano una famiglia alla volta. Ogni intensità5/10/20% assegna
quel costo rispetto allo score imitativo positivo del best80, **385,769446**,
sull'intero episodio. Per la cinematica il budget è diviso equamente fra giunti.

| Famiglia | Peso5% | Peso10% | Peso20% |
|---|---:|---:|---:|
| Ankle positiva | 0,6689593 | **1,3379186** | 2,6758372 |
| Rateo coppia knee | 0,0280720 | 0,0561440 | 0,1122880 |
| Jerk knee | 0,00121666 | 0,00243331 | 0,00486663 |
| Jerk ankle | 0,00208413 | 0,00416827 | 0,00833653 |

Le due righe jerk appartengono alle stesse tre candidate, quindi le candidate
sono nove, non dodici. `ankle_10` aggiunge **38,576945 punti di costo** sul trace
best80 congelato: non significa10% di riduzione attesa della coppia e non
identifica un peso ottimale. Le curve fisiche non cambiano nel ricalcolo.

I dieci rollout rumorosi storici non hanno gli STO: area positiva ankle,
spettri della coppia e metriche di contatto che richiedono quei file restano
indisponibili. Replay dei ritorni e sensibilità knee/jerk utilizzabili dai trace
restano disponibili. Nessun dato mancante viene trattato come costo zero.

## Pilot predisposto

Due rami dallo stesso checkpoint completo80: `control` con reward invariata e
`ankle_10` con il solo nuovo peso ankle. Penetrazione resta2, prescribed LEFT
e clock restano visibili, FSMv3 e configurazione del simulatore sono conservati.

Per ramo: **1 aggiornamento solo critic +3 PPO**, batch4096 ed episodi5s.
Sono previsti32.768 nuovi campioni di training complessivi; gli aggiornamenti
logici sono81 di rewarm e82–84 PPO. La nuova fase `imitation_critic_rewarm`
mantiene visibili gli ingressi imitativi e verifica actor/logstd e relativi
momenti Adam immutati. Il countdown è persistente; il resume non ripete un
rewarm già completato. Un aggiornamento non certifica convergenza del critic.

La valutazione prevista confronta best80, controllo e candidata con un nominale
e seed123/125/127: **12 episodi**, massimo6.000 transizioni. La sigma dipende
dallo stato: il nominale best80 ne congela una sequenza di500 decisioni, riusata
con gli stessi campioni gaussiani per indice. È uno screening a perturbazioni
controllate, distinto dalla valutazione delle sigma native delle policy.

Budget cumulativo120min, compresi training, valutazioni e plot, senza estensione
automatica. Dopo il nominale best80 sono consentiti fino a quattro processi
di valutazione con un thread numerico ciascuno. Il lanciatore verifica input,
checkpoint completi e aggiornamenti effettivi, gestisce timeout/interruzione
e conserva i tentativi incompleti. I futuri sette plot canonici per rollout
completo saranno in root `plot/`.

I criteri registrati richiedono, contro best80 e controllo:

- RMSE globale e per fase: aumento massimo `min(10%,0,5°)` per giunto.
- Escursione e impulso verticale almeno90% del riferimento; completamento
  degli scenari completati dal riferimento e cicli non ridotti.
- Penetrazione nominale sotto20mm, incremento massimo2mm nei confronti
  pertinenti e nessuna nuova terminazione hard.
- Per un esito promettente: area positiva nominale ridotta almeno10%, nessun
  aumento mediano nelle prove rumorose e regolarità non peggiorate oltre10%.

I fallimenti non possono vincere riducendo l'area tramite un episodio più
corto. Dati mancanti/esiti discordanti restano incompleti o inconclusivi.
Lo screening non attesta robustezza statistica, parità V26 o apprendimento
migliore da zero. Oscuramento ed ex-novo restano fuori dal pilot.

## File modificati e artefatti

Nella tabella `39/` abbrevia `Trajectory Generator/unified_MLP_39/`.

| Perimetro | File e contenuto |
|---|---|
| Diagnostica root | `simulation_runner.py`: statistiche positive SEA |
| Reward39 | `39/reward_function.py`, nuovo `39/tuning_terms.py` |
| Fork e critic | `39/reward_resume.py`, `training_runtime.py`, `asymmetric_rl_module.py`, `training_config.py`, `train_ppo_mlp.py` |
| Banco | nuovo `39/tune_reward.py`; package `39/reward_tuning/` con I/O, analisi, metriche, confronto, pilot, valutazione e README |
| Configurazione | `39/configs/reward_tuning_left.yaml` |
| Test | `39/tests/test_tuning_terms.py`, `test_tuning_reward_resume.py`, `test_tuning_offline.py`, `test_tuning_comparison.py`, `test_tuning_pilot.py` |
| Risultati | `39/runs/reward_tuning/left_best80_2026-09-25/`: analisi, configurazioni, manifest, stato, confronto incompleto e ricevuta di verifica |

Collegamenti principali:

- [Guida e comandi — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-0fe045e94030c646>).
- [Configurazione — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-d8df9f1480de598e>).
- [Report offline — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-c465d3ef20ee64b5>).
- [Metriche, pesi e sensitività — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-a4272d7dceee8afc>).
- [Manifest pilot — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-d5dc8ff8273f953b>) e [stato — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-f4254909c9468124>).
- [Verifiche della consegna — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-3235408ad8f801b2>).
- [Smoke OpenSim — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-dd880b41e4e75ad8>).
- [Confronto fisico](../../plot/09_25_2026_LEFT_reward_tuning/offline_physical_comparison.png) e [distribuzione dei costi per fase](../../plot/09_25_2026_LEFT_reward_tuning/offline_reward_sensitivity.png).

## Test e verifiche

Durante l'implementazione:

- **192 test passati**, suite `unified_MLP_39/tests`,2,79s.
- Replay dei17 ritorni: errore massimo **4,55×10⁻¹³**, contro tolleranza1e-5.
- Concordanza RMS SEA offline/online: errore massimo **2,96×10⁻⁹Nm**.
- Smoke OpenSim **2×5 segmenti**,18,22s: stati fisici identici, reward legacy
  conservata, arricchimento offline/online identico e differenza di reward
  uguale alla nuova penalità. Nessun aggiornamento PPO o runtime Ray avviato.
- Test con Adam effettivo: actor/logstd e relativi momenti immutati nel rewarm,
  critic modificato, ritorno all'imitazione e ripresa corretti.
- Lanciatore verificato con processi fittizi: budget, timeout, ripresa,
  processi figli, integrità e perturbazioni comuni; non è un training RLlib live.
- Parser CLI e contratto runtime delle due configurazioni superati.
- **199 input congelati** verificati; figure ispezionate visivamente.

Per `create_report` sono riletti artefatti e stato e controllati i collegamenti;
non si ripetono training o test già passati. Le prove live disponibili attestano
macOS arm64; Windows resta da verificare.

## TODO da propagare

- [x] Completare il passaggio all'esecuzione del pilot: avviato su nuova richiesta
  esplicita alle12:46:53 CEST; aggiornamento di stato riportato sotto.
- [ ] Verificare il primo training RLlib live: restore, reward nei worker,
  rewarm imitativo, conservazione actor/Adam e tre aggiornamenti PPO per ramo.
- [ ] Completare nominale e prove rumorose del pilot, plot canonici e confronto
  fisico, senza estendere automaticamente il budget120min.
- [ ] Decidere dall'esito se approfondire ankle_10, provare ankle_5 o rivedere
  il termine; non assumere che tre aggiornamenti siano sempre risolutivi.
- [ ] Dopo l'ankle, sperimentare separatamente regolarità knee e cinematica
  usando lo stesso banco e controlli; non cambiare insieme le tre famiglie.
- [ ] Eseguire verifica live Windows e, se una candidata è promettente,
  repliche/condizioni più ampie prima di conclusioni sulla robustezza.

Il backlog precedente resta nel daily; la preparazione del pilot non chiude
screening storico best80/best72, controllo penetrazione peso1, diagnostici reset
FSM, generalizzazione o trasferimento senza feature prescribed.

## Aggiornamento successivo: avvio del pilot alle12:46:53 CEST

Su nuova richiesta esplicita «avvia il pilot», avviato il lanciatore persistente
PID32242, fuori dalla sandbox per le risorse locali Ray/OpenSim. Il budget
cumulativo resta7200s, con limite previsto intorno alle14:46:53 CEST.
Lo stato `prepared` descritto sopra è quello della redazione precedente.

Al controllo delle12:47:55 il pilot è `running`, ramo `train_control`, iterazione
logica81/84 nella fase `imitation_critic_rewarm`. Ripristino di best80 verificato:
actor, medie e critic esatti, Adam step4455 e coefficiente KL0,3000000119
conservati. Tutti13 i worker espongono reward corretta, LEFT visibile al100%,
penetrazione peso2 e FSMv3. Nessun errore fatale di avvio rilevato.

La prima iterazione non è ancora conclusa; candidata e valutazioni devono
ancora partire. L'avvio non costituisce un risultato del confronto.
Ricevute: [launch_receipt.json — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-4d8fa938c0d7c490>)
e [startup_check.json — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-9398c2c6609a9f32>).
