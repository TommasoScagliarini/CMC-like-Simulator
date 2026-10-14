# Banco di tuning rapido LEFT

Il banco separa il ricalcolo della reward sui trace congelati dalla verifica
dell'effetto appreso. La prima consegna conteneva 17 rollout analizzati e un
pilot preparato. Il pilot `ankle_10` è stato poi eseguito su richiesta: riduce
l'area positiva nominale ma non supera i vincoli fisici. Il test successivo
`ankle_5` usa un esperimento distinto e lo stesso protocollo.

## Uso

Dalla root del repository, nell'ambiente `envCMC-rllib`:

```sh
python "Trajectory Generator/unified_MLP_39/tune_reward.py" analyze --config "Trajectory Generator/unified_MLP_39/configs/reward_tuning_left.yaml"
python "Trajectory Generator/unified_MLP_39/tune_reward.py" prepare --config "Trajectory Generator/unified_MLP_39/configs/reward_tuning_left.yaml"
python "Trajectory Generator/unified_MLP_39/tune_reward.py" compare --config "Trajectory Generator/unified_MLP_39/configs/reward_tuning_left.yaml"
```

`analyze` scrive report, CSV, JSON e figure; `prepare` genera due YAML e un
manifest immutabile. `compare` dichiara `incomplete` finché mancano valutazioni.
La configurazione fornita è già preparata: una seconda `prepare` sullo stesso
percorso è rifiutata. Per un nuovo esperimento copiare il file YAML e scegliere
un nuovo `output_dir` e `plot_dir` prima di analizzare/preparare.

Solo il comando seguente avvia training e valutazioni:

```sh
python "Trajectory Generator/unified_MLP_39/tune_reward.py" run --config "Trajectory Generator/unified_MLP_39/configs/reward_tuning_left.yaml"
```

Per le successive intensità ankle, `pilot.candidate` seleziona `ankle_5`,
`ankle_10` (default) o `ankle_20` dalla calibrazione. `analysis_file` può puntare
all'analisi congelata precedente. `pilot.reuse_reference_experiment` consente
il riuso esplicito di best80 e controllo di un pilot completato: si verificano
checkpoint, fasi, protocollo, sigma, ricevute e hash degli artefatti. Le sole
modifiche ammesse ai vecchi sorgenti sono nei tre file di orchestrazione
`pilot.py`, `compare.py`, `__main__.py`, annotate con hash vecchio e nuovo.
Ogni differenza nel simulatore, reward, learner o evaluator impedisce il riuso.
Il nuovo manifest congela nuovamente tutti i sorgenti effettivi e gli input.
Le valutazioni e il training riutilizzati sono indicati come tali; il costo
nuovo è un ramo (quattro aggiornamenti) e quattro valutazioni. I vecchi
esperimenti conservano i loro manifest e non vanno risigillati dopo modifiche.

La configurazione `configs/reward_tuning_left_ankle5.yaml` usa questo riuso:

```sh
python "Trajectory Generator/unified_MLP_39/tune_reward.py" prepare --config "Trajectory Generator/unified_MLP_39/configs/reward_tuning_left_ankle5.yaml"
python "Trajectory Generator/unified_MLP_39/tune_reward.py" run --config "Trajectory Generator/unified_MLP_39/configs/reward_tuning_left_ankle5.yaml"
```

I comandi sono identici su macOS e Windows; usare il Python dell'ambiente
installato sulla piattaforma. I path nel YAML sono relativi alla root. Il
manifest preparato congela i path risolti e l'interprete: su un'altra macchina
preparare un nuovo esperimento locale, senza riutilizzare manifest con path
assoluti della prima macchina. Windows live non è attestato da test macOS.

## Cosa misura

La selezione controlla prima tracking e contatto, poi area positiva ankle,
regolarità coppia knee e cinematica. Target LEFT e clock rimangono visibili.
Il segno della coppia non viene confuso con quello della forza verticale.

Le tre famiglie sono isolate e opt-in:

| Famiglia | Loss | Scale iniziali |
|---|---|---|
| Ankle | media di `max(tau_spring,0)` sui substep / scala | 30 Nm |
| Knee | `(RMS rateo coppia / scala)^2` nei segmenti | 1000 Nm/s |
| Cinematica | `(jerk effettivo / scala)^2`, knee/ankle separati | 1000 rad/s³ |

I costi sono dopo il clipping e attivi solo in imitazione. Pesi zero conservano
la reward storica; le scale non sono limiti fisiologici. Non ci sono cap nascosti
sui nuovi quadrati. La reference servita è misurata ma non riceve una nuova
penalità in questa versione.

Il jerk usa velocità effettive ai confini della policy, accelerazioni medie di
intervallo e differenza fra i loro punti medi. Il reset inizializza la velocità;
il primo segmento riscalda la storia, il secondo rende disponibile il jerk.
Segmenti hard incompleti non riusano statistiche vecchie; safety/contatto restano
attivi. Input necessari mancanti non sono costi nulli.

Gli STO SEA registrano campioni **prima** dell'integrazione: l'analisi usa
`[inizio segmento, fine segmento)`, verificando count e RMS contro il trace.
Il rateo RMS della reward esclude i confini fra segmenti come la diagnostica
esistente; il rateo diagnostico sull'intero STO include quei confini.

Tracking globale e per dieci bin di fase prescribed, escursioni, impulso,
penetrazione e cicli sono separati dai ritorni di training. I filtri diagnostici
Butterworth ordine4 a fase zero precedono il ritaglio della finestra storica
13,15–17,95s. Nei confronti a prefisso comune si elimina prima il futuro oltre
il prefisso per evitare che `filtfilt` ne utilizzi i campioni. Il jerk e HP10
cinematico sono metriche a100Hz; i passa-alto di coppia usano gli STO a1kHz.

I dieci rollout rumorosi storici hanno trace ma non STO: replay e sensibilità
knee/jerk restano disponibili; area positiva ankle e metriche spettrali della
coppia sono N/D. Nessun ritorno stimato sostituisce questa informazione.

## Calibrazione e pilot

Le intensità 5/10/20% assegnano alla famiglia quel costo rispetto allo score
imitativo positivo di best80. Nella famiglia cinematica metà budget va a
ciascun giunto. Non è un confronto tra politiche apprese e non identifica
automaticamente un peso ottimale.

Il pilot usa best80 completo e due rami: controllo invariato e `ankle_10`.
Ogni ramo comprende un aggiornamento solo critic e tre PPO, batch4096,
episodi5s. La fase `imitation_critic_rewarm` conserva i prescribed e verifica
actor/logstd e momenti Adam immutati; al resume non viene ripetuta. Il critic
non è dichiarato convergente dopo un solo aggiornamento.

Nominale e seed123/125/127 valutano best80/controllo/candidata. Poiché la sigma
è dipendente dallo stato, il nominale best80 ne congela una sequenza di500
decisioni: tutti i test rumorosi usano la stessa sequenza e gli stessi campioni
gaussiani per indice. Questo è uno screening a perturbazioni controllate,
distinto dalla valutazione della sigma nativa appresa da ogni policy.

Budget120min cumulativo, checkpoint dopo ogni aggiornamento, ripresa dal più
recente completo. Le valutazioni successive al nominale best80 usano al massimo
quattro processi, ciascuno con un thread numerico. Timeout/interruzioni preservano
i checkpoint pubblicati e gli output incompleti. Lo stesso `run` riprende un
pilot interrotto con il solo budget residuo; non estende il budget esaurito.

Il protocollo è congelato: modifiche a codice, input, configurazioni o checkpoint
richiedono una nuova preparazione in un'altra directory. I sette plot canonici
dei futuri rollout completi sono in root `plot/`.

## Limiti del verdetto

`promising` richiede rispetto dei vincoli, riduzione nominale dell'area positiva
di almeno10%, nessun aumento mediano nei tre test rumorosi e nessun peggioramento
oltre10% delle regolarità. Mancanza di dati/metriche o esiti discordanti producono
`inconclusive`; valutazioni mancanti producono `incomplete`. I fallimenti non
vincono tramite una minore area su episodi più corti.

La durata breve serve a selezionare ipotesi. Non dimostra robustezza statistica,
migliore apprendimento da zero o successo dopo oscuramento.
