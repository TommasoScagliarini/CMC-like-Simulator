# Report - Perche e servita la pipeline e requisiti per dataset OpenSim puliti

Data: 2026-04-30

## Scopo del Report

Questo report riassume perche, nel progetto corrente, e stato necessario usare
una pipeline OpenSim completa sul modello invece di lanciare direttamente il
simulatore CMC-like sui file iniziali.

Il secondo obiettivo e fornire un testo utile da passare a un LLM o a un agente
di ricerca per cercare su internet modelli e dataset open-source di camminata
dritta, puliti e adatti al simulatore.

## Problema Principale

Il modello e i dati sperimentali disponibili non erano direttamente pronti per
una simulazione CMC-like stabile.

Il simulatore richiede coerenza stretta tra:

- modello `.osim`;
- marker set e cinematica IK;
- GRF misurate;
- mapping delle forze esterne ai piedi corretti;
- reserve actuators;
- attuatori SEA `SEA_Knee` e `SEA_Ankle`;
- finestra temporale in cui cinematica e GRF sono entrambe valide.

Nel dataset corrente questa coerenza non era garantita in modo nativo. Per
questo e stata necessaria una pipeline che facesse scaling, IK, ID, RRA,
promozione del modello e test CMC-like.

## Problemi Specifici Osservati

### 1. Le GRF non coprono l'intervallo desiderato

L'intervallo inizialmente desiderato era circa:

`17 - 23 s`

Tuttavia le GRF misurate affidabili non coprono tutto questo tratto.

La finestra misurata migliore trovata e:

`FP1/FP2 = 15.889 - 19.839 s`

La sottofinestra piu stabile per il simulatore e:

`17.500 - 19.000 s`

Dopo circa `19.839 s` non ci sono GRF FP coerenti per proseguire fino a `23 s`.
Quindi una simulazione lunga su `17 - 23 s` richiederebbe ricostruzione GRF o un
altro dataset.

### 2. Le finestre treadmill non sono utilizzabili

E stata testata anche la finestra:

`Treadmill_L/R = 30.065 - 34.275 s`

Dopo analisi del mapping CoP-marker e stata corretta l'associazione:

- `Treadmill_L -> calcn_r`
- `Treadmill_R -> foot_l`

Nonostante la correzione, la finestra treadmill e risultata numericamente
patologica:

- run full in timeout;
- sottofinestra iniziale in timeout;
- sottofinestra con entrambe le sorgenti attive fallita con accelerazioni RK4
  non finite;
- reserve molto elevate;
- saturazioni SEA.

Questo indica che la sola presenza di forze misurate non basta: le forze devono
essere coerenti con cinematica, modello, piedi, orientamenti, unità e timing.

### 3. Il modello iniziale non era direttamente pronto per CMC-like

Alcuni modelli manuali o intermedi non erano direttamente utilizzabili dal
simulatore perche il loader non trovava correttamente gli attuatori SEA nel
modello finale oppure perche non erano coerenti con i dati IK/GRF.

Il modello da usare doveva preservare:

- `SEA_Knee`;
- `SEA_Ankle`;
- coordinate protesiche;
- actuator set compatibile;
- reserve actuators;
- geometria e path necessari;
- massa/COM coerenti con RRA.

Per questo non era sufficiente prendere un `.osim` e lanciarlo: serviva una
promozione controllata del modello dopo RRA, preservando la struttura SEA.

### 4. RRA e CMC-like avevano criteri diversi

Un modello puo sembrare accettabile in RRA ma fallire nel simulatore CMC-like.

Per questo la scelta finale non e stata basata solo sui residuali RRA, ma anche
su:

- completamento della simulazione;
- assenza di non-finite;
- warning di static optimization;
- `tau_reserve_norm`;
- reserve pelvis;
- reserve articolari;
- saturazioni SEA;
- ispezione dei plot.

La pipeline e servita proprio a collegare questi passaggi e scegliere il modello
che funzionasse nel simulatore, non solo in OpenSim.

### 5. Le reserve pelvis restano alte

Anche nella finestra migliore, la simulazione completa e stabile ma non e ancora
una soluzione biomeccanicamente perfetta.

Nella run locale `17.500 - 19.000 s`:

- `status = complete`
- `nonfinite_total = 0`
- `SEA saturation frames = 0`
- `tau_reserve_norm_mean = 233.684`
- `tau_reserve_norm_max = 385.845`
- `pelvis_reserve_torque_max_abs = 374.208`
- `joint_reserve_torque_max_abs = 29.634`

Questi valori indicano che il modello e simulabile, ma il sistema sta ancora
usando reserve di bacino importanti. Per analisi biomeccanica robusta serve un
dataset piu pulito e coerente.

## Soluzione Implementata Nel Progetto Corrente

E stata costruita una pipeline dedicata per testare finestre misurate prima di
ricostruire GRF.

Script principale:

`scripts/run_measured_grf_window_tests.py`

La pipeline esegue:

1. IK sulla finestra scelta.
2. Generazione `ExternalForces.xml` coerente con il profilo GRF.
3. ID diagnostico.
4. RRA su candidati multipli.
5. Promozione del miglior modello SEA-preserving.
6. CMC-like con:

```powershell
--solver osqp --sea-feasibility-scaling
```

7. Report per finestra e confronto delle metriche.

Modello operativo ottenuto:

`models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_ready.osim`

Setup locale:

`models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_setup.xml`

Run stabile:

`results/fp12_17500_19000_local_model`

Plot:

`plot/30_04_2026 - 1`

## Perche Serve Cercare Un Dataset/Modello Migliore

Il dataset corrente e utile per debug e per validare il simulatore su una
finestra corta, ma non e ideale come base pulita per sviluppo e validazione.

Limiti principali:

- finestra GRF valida troppo corta;
- impossibilita di coprire `17 - 23 s` con GRF misurate;
- treadmill non stabile;
- necessita di mapping manuale delle force plate;
- reserve pelvis alte;
- possibile incoerenza tra modello, marker set, forze e timing;
- assenza di piu cicli completi puliti nella finestra finale;
- plot gait-cycle parziali se la finestra non contiene eventi completi.

Serve quindi un dataset open-source piu pulito di camminata dritta, idealmente
gia organizzato per OpenSim.

## Requisiti Per Il Dataset Da Cercare

### Requisiti Minimi

Il dataset dovrebbe includere:

- modello OpenSim `.osim` del soggetto o modello generico scalabile;
- marker trajectories in `.trc`;
- GRF misurate in `.mot`;
- file ExternalLoads `.xml` o informazioni chiare per generarlo;
- almeno 1-3 cicli di cammino dritto overground;
- force plate pulite con contatti chiari;
- massa corporea e altezza del soggetto;
- frequenze di acquisizione marker/force;
- sistema di coordinate documentato;
- licenza open-source o almeno uso accademico chiaro.

### Requisiti Preferibili

Meglio se il dataset include:

- modello gia scalato del soggetto;
- IK gia calcolata;
- ID gia calcolata;
- RRA o CMC setup OpenSim;
- piu trial di walking straight;
- marker set compatibile con modelli OpenSim standard;
- camminata su piano, non treadmill;
- dati senza grossi gap marker;
- GRF senza clipping o foot strike fuori pedana;
- eventi gait-cycle gia annotati;
- dataset healthy control e, se possibile, amputee/prosthetic.

### Requisiti Specifici Per Questo Simulatore

Per usare bene il simulatore CMC-like di questo repository, il dataset dovrebbe
permettere di ottenere:

- coordinate coerenti con un modello tipo gait2392/gait2354 o derivato;
- pelvis e arti inferiori ben tracciati;
- forze esterne applicabili a `calcn_r` e piede sinistro/protesico;
- finestre di almeno `1.5 - 3.0 s` con GRF pulite;
- nessun bisogno di ricostruire GRF per la prima validazione;
- reserve pelvis ragionevoli dopo RRA/CMC-like;
- possibilita di sostituire o aggiungere due attuatori SEA su ginocchio e
  caviglia protesici.

## Dataset Da Evitare

Sono meno adatti dataset che hanno:

- solo cinematica senza GRF;
- solo dati treadmill senza mapping chiaro delle forze;
- marker set proprietario non documentato;
- file C3D senza conversione chiara a `.trc`/`.mot`;
- camminata con curve, scale, ostacoli o perturbazioni;
- force plate non colpite correttamente;
- finestre di contatto troppo brevi;
- licenza non chiara;
- assenza di massa corporea o sistema di coordinate.

## Prompt Suggerito Per Un LLM Di Ricerca

Usare o adattare questo testo:

```text
Sto cercando dataset e modelli OpenSim open-source per una simulazione CMC-like
di camminata dritta su piano. Mi serve materiale pulito e riproducibile, non
solo video o dati incompleti.

Requisiti minimi:
- modello OpenSim .osim oppure modello scalabile con dati del soggetto;
- marker trajectories .trc o C3D convertibile in modo documentato;
- ground reaction forces misurate .mot o C3D con force plates;
- ExternalLoads XML o informazioni complete per generarlo;
- almeno 1-3 cicli di walking straight overground;
- force plate pulite, con foot strikes chiari e senza contatti misti;
- massa corporea, altezza, frequenze di acquisizione e sistema coordinate;
- licenza open-source o uso accademico permesso;
- possibilita di usare i dati in OpenSim per IK, ID, RRA/CMC.

Preferenze:
- dataset healthy walking overground, ma vanno bene anche amputee/prosthetic
  se ben documentati;
- modelli gia scalati o setup IK/ID/RRA/CMC inclusi;
- dati gia in formato OpenSim (.osim, .trc, .mot, .xml);
- piu trial di camminata rettilinea;
- eventi gait-cycle annotati.

Contesto tecnico:
ho un simulatore Python/OpenSim CMC-like con due attuatori SEA protesici su
ginocchio e caviglia. Il dataset attuale richiede una pipeline complessa perche
le GRF misurate sono disponibili solo su una finestra corta, le treadmill GRF
falliscono numericamente, e le reserve pelvis restano alte. Voglio un dataset
piu pulito per validare il simulatore su camminata dritta.

Per ogni dataset trovato, riportami:
- nome dataset;
- link ufficiale;
- licenza;
- formati disponibili;
- presenza di modello .osim;
- presenza di .trc/.mot oppure C3D con force plates;
- numero di soggetti e trial;
- tipo di camminata;
- se include GRF overground;
- quanto e adatto a OpenSim IK/ID/RRA/CMC;
- eventuali problemi noti.

Ordina i risultati dal piu adatto al meno adatto.
```

## Query Web Suggerite

Esempi di query utili:

- `OpenSim walking dataset overground GRF TRC MOT`
- `OpenSim gait dataset straight walking ground reaction forces`
- `open source OpenSim gait2392 walking data force plates`
- `OpenSim lower limb dataset C3D GRF overground walking`
- `prosthetic gait OpenSim dataset ground reaction forces`
- `amputee walking OpenSim dataset force plates`
- `healthy overground walking OpenSim example data`
- `SimTK OpenSim walking dataset GRF`
- `SimTK gait dataset OpenSim CMC RRA`

## File Modificati o Creati In Questa Fase

Report creato:

- `reports/user/2026-04-30_problema_pipeline_e_requisiti_dataset_camminata.md`

File tecnici rilevanti della fase precedente:

- `scripts/run_measured_grf_window_tests.py`
- `models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_ready.osim`
- `models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_setup.xml`
- `models/SEASEA - whealthy data/data/ExternalForces_fp12_15889_19839.xml`
- `results/fp12_17500_19000_local_model`
- `plot/30_04_2026 - 1`

## Verifiche Eseguite

Le conclusioni di questo report si basano su:

- test finestra FP1/FP2;
- test finestra treadmill;
- RRA candidati;
- CMC-like con `osqp`;
- timeout controllato sulle finestre treadmill;
- simulazione locale completa su `17.500 - 19.000 s`;
- plot dei risultati.

## Conclusione

La pipeline e stata necessaria perche il materiale iniziale non era un dataset
OpenSim pulito e direttamente simulabile: mancavano finestre GRF lunghe e
coerenti, il modello doveva preservare gli attuatori SEA, e la stabilita andava
validata nel simulatore CMC-like.

Per proseguire in modo piu robusto, conviene cercare un dataset OpenSim di
camminata dritta overground con GRF complete, modello e setup ben documentati.
Il prompt incluso sopra puo essere usato direttamente con un LLM con accesso al
web per ottenere una shortlist di dataset candidati.
