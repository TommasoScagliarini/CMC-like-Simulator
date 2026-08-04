# Daily Report - 2026-07-21

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-21_detector_virtuale_heel_toe_fsm_implementazione_validazione.md`
- `reports/user/2026-07-21_detector_virtuale_v3_confirmed_time_holdout.md`
- `reports/user/2026-07-21_audit_geometrico_detector_due_sensori_v4.md`
- `reports/user/2026-07-21_profilo_detector_v4_mesh_sperimentale.md`

## Sintesi

La giornata ha implementato e analizzato un detector protesico semplice a due
sensori virtuali, uno heel e uno toe/forefoot. I sensori sono detector-only:
misurano il carico locale, non generano GRF e non applicano forze al modello.
La loro logica e' stata integrata nella `ProstheticPhaseFSM` esistente, senza
creare una seconda macchina a stati e senza modificare schema actor, reward,
plugin SEA o checkpoint.

La validazione ha separato correttamente tre domande:

1. la logica HS/TO della FSM e' semanticamente corretta;
2. il detector deve superare timing, conteggi e fase su dati prescribed senza
   ricorrere a training;
3. soltanto dopo il freeze del detector si puo' verificare se una policy
   addestrata con gli eventi legacy resta robusta alla nuova temporizzazione.

Il replay breve a cadenza runtime ha dato un PASS in-scope, ma il protocollo
full-span ha mostrato limiti reali. La V3 con `confirmed_time_s` come timestamp
causale ha superato development e sensitivity, ma ha fallito il holdout
cronologico `50-100 s`. L'audit successivo ha inoltre dimostrato che il sensore
toe corrente era molto piu' basso e staccato dalla mesh. Il primo profilo V4
mesh-based ha corretto la geometria, ma ha perso la continuita' heel-forefoot e
ha fallito nettamente il replay funzionale.

Il nuovo detector non e' stato promosso. `legacy_events` e' rimasto il default,
il sealed set non e' stato aperto e non e' stato avviato alcun training sul
detector sperimentale.

## 1. Problema affrontato

Il detector storico aggregava i contatti del piede. Nei trace del checkpoint
best alcuni eventi chiamati Heel Strike cadevano quando il tallone era
scarico, la punta sosteneva il contatto e il ginocchio served era ancora molto
flesso. Il contatto forefoot-first poteva quindi essere interpretato come HS.

La modifica richiesta doveva preservare le invarianti gia' validate:

- due soli sensori semplici, heel e toe;
- sensori usati soltanto per il detecting;
- una sola FSM del gait cycle;
- nessuna variazione della generazione GRF;
- nessuna variazione del contratto actor a 35 feature e dell'osservazione
  completa a 84 feature;
- retrocompatibilita' del checkpoint e del percorso legacy;
- nessuna scorciatoia tramite eventi sintetici o rilassamento post-hoc dei
  gate.

## 2. Soluzione implementata nella FSM esistente

Il percorso online GRF espone ora quattro canali regionali additivi:

```text
left_heel
left_toe
right_heel
right_toe
```

Ogni canale registra carico normale, penetrazione e stato di contatto. I
componenti hanno `appliesForce = false`, quindi restano strumenti di misura.

La FSM dispone di tre modalita' esplicite:

| Modalita' | Autorita' sugli eventi | Stato |
| --- | --- | --- |
| `legacy_events` | detector aggregato storico | default validato |
| `shadow` | detector legacy, heel/toe solo diagnostici | non interferente |
| `two_sensor` | guardie heel/toe | sperimentale |

La prima implementazione usa soglie ON/OFF `5/2 N` e dwell `30 ms`:

- il detector si arma dopo heel e toe stabilmente scarichi;
- toe-only produce `forefoot_first`, ma non HS;
- HS richiede un heel stabile;
- TO richiede heel e toe stabilmente OFF;
- un reset gia' in stance crea un segmento parziale senza HS sintetico,
  credito o incremento dei contatori;
- handler HS/TO, timeout, journal e gate anti-fake-cycle preesistenti restano
  invariati.

Gli eventi legacy sinistri vengono esclusi quando `two_sensor` e' autoritativo,
evitando di fondere due semantiche. I carichi regionali non sono stati aggiunti
all'actor e rimangono disponibili soltanto nel trace e nelle diagnostiche.

## 3. Replay prescribed e forward-state iniziale

La validazione del detector e' stata eseguita senza actor, checkpoint, reward o
PPO. Il replay ricostruisce i carichi heel/toe lungo l'IK prescribed e usa la
stessa FSM di produzione.

Alla cadenza runtime di `10 ms`:

| Evento | Reference | Predetti | Precision/recall | Errore massimo |
| --- | ---: | ---: | ---: | ---: |
| HS | 5 | 5 | `1.00/1.00` | `47.93 ms` |
| TO | 4 | 4 | `1.00/1.00` | `76.85 ms` |

Sono stati ricostruiti quattro cicli completi, senza timeout, invalidi o
candidati sensoriali rifiutati. Dal primo HS completo in poi l'accordo grezzo
stance/swing e' stato `98.02%`, con `100%` di accordo lontano dalle finestre di
transizione.

La sensitivity a `1 ms` ha mantenuto conteggi, ordine e semantica, ma ha
superato marginalmente le tolleranze congelate:

- HS massimo `53.93 ms`, cioe' `3.93 ms` oltre il gate da `50 ms`;
- TO massimo `80.85 ms`, cioe' `0.85 ms` oltre il gate da `80 ms`.

Il risultato e' quindi `timing FAIL`, `semantic PASS` e `phase-state PASS`.
Gli stessi risultati sono stati riprodotti sugli stati forward storici. La
procedura dimostra consistenza del data path, non generalizzazione a un gait
indipendente.

## 4. Protocollo full-span V1-V3

Sono stati preregistrati tre blocchi cronologici:

| Blocco | Finestra | Cicli completi | Uso |
| --- | ---: | ---: | --- |
| development | `11.99-50 s` | 23 | selezione |
| validation | `50-100 s` | 51 | candidato congelato |
| sealed | `100-155.045 s` | 48 | apertura solo dopo PASS |

### V1 e V2

La V1 con soglie `5-20 N` e la V2 low-force hanno fallito il development. Il
miglior candidato diagnostico V2, `ON=0.5 N / OFF=0.01 N`, ha conservato
`24/23/23` HS, TO e cicli, ma ha raggiunto `72.96 ms` su HS e `92.52 ms` su TO.
Validation e sealed non sono stati aperti in queste revisioni.

### V3 confirmed-time

La V3 ha reso esplicito il tempo realmente disponibile alla policy:

```text
primary_event_time_field    = confirmed_time_s
diagnostic_event_time_field = event_time_s
sensor_dwell_s              = 0.03
```

Ha inoltre costruito una reference di fase coerente con gli intervalli
prescribed HS-TO, eliminando micro-riattraversamenti GRF gia' esclusi
dall'estrattore degli eventi.

Il development ha selezionato `ON=0.50 N / OFF=0.25 N`:

| Metrica | 10 ms | 1 ms |
| --- | ---: | ---: |
| HS / TO / cicli | `24/23/23` | `24/23/23` |
| Precision / recall | `1.000/1.000` | `1.000/1.000` |
| HS massimo confirmed | `42.96 ms` | `46.96 ms` |
| TO massimo confirmed | `62.52 ms` | `65.52 ms` |
| Invalidi / timeout | `0/0` | `0/0` |

Development e sensitivity sono PASS. Il holdout `50-100 s` e' invece FAIL:

```text
HS reference / detector       52 / 49
TO reference / detector       51 / 49
cicli completi                48 / 51
F1 / IoU fase                 0.93244 / 0.87343
mismatch vietati              212 campioni
timeout                       1
```

Quattro HS sono stati confermati con ritardo `157-162 ms`; un quinto HS non e'
stato confermato e ha innescato una cascata fino al timeout. Gli impulsi heel
critici erano toe-dominant e, in alcuni casi, piu' brevi del dwell o non
campionabili correttamente a `10 ms`. Il sealed e' rimasto chiuso.

## 5. Audit geometrico V4

L'audit ha chiarito che la reference `Fy > 20 N` identifica l'initial contact
del piede, non un heel strike anatomico localizzato. Ha inoltre trovato un
difetto sostanziale nel profilo corrente:

| Metrica | Heel | Toe corrente |
| --- | ---: | ---: |
| distanza centro-mesh | `4.79 mm` | `41.17 mm` |
| gap sfera-mesh | `0 mm` | `18.26 mm` |
| fondo sotto il minimo mesh | `10.50 mm` | `51.70 mm` |

Il toe coincideva con il marker laterale `L_Toe_Lat`, non con un punto
plantare, e poteva quindi generare un contatto molto anticipato. I cinque HS
critici non potevano essere usati per concludere che AB06 stesse realmente
eseguendo un forefoot strike.

La lunga validazione storica non era falsa: validava correttamente un detector
aggregato. Non validava pero' un heel sensor anatomico separato dal toe.

## 6. Profilo V4 mesh-based sperimentale

E' stato creato un profilo separato, mantenendo heel, FSM, soglie e dwell
invariati e spostando soltanto il toe su un punto riproducibile della mesh.

Il profilo ha superato il gate geometrico:

```text
distanza centro-mesh toe       4.512 mm
gap sfera-mesh                 0 mm
centro nei bounds mesh         si
fondo toe vs fondo heel        8.770 mm piu' alto
```

Il fixed replay ha pero' fallito nettamente:

| Metrica a 10 ms | Reference | Corrente | V4 mesh |
| --- | ---: | ---: | ---: |
| HS | 51 | 49 | 1 |
| TO | 50 | 49 | 0 |
| cicli validi | 50 | 48 | 0 |
| F1 stance | - | `0.941` | `0.083` |
| IoU stance | - | `0.889` | `0.043` |

Il toe V4 era aderente alla mesh ma troppo anteriore e alto rispetto alla
regione realmente portante: heel e toe rimanevano entrambi OFF fino a
`390 ms`, i TO precoci venivano correttamente respinti e la stance finiva in
timeout. La sensitivity a `1 ms` ha replicato lo stesso fallimento.

V4 e' quindi `PASS_GEOMETRY`, `FAIL_DETECTOR`, `NOT_PROMOTABLE`.

## 7. Compatibilita' con il checkpoint best

In modalita' shadow il checkpoint best ha riprodotto esattamente il rollout
storico:

```text
step                       500 / 500
return                     52.4269395298
penetrazione massima       24.2174 mm
HS / TO / cicli            4 / 3 / 3
azioni e trace             identici
```

Il rollout attivo con il detector two-sensor disponibile il 21 luglio ha
invece terminato a `198/500` step:

```text
return                     -5.8691
penetrazione massima       25.9772 mm
gate hard                  25 mm
HS / TO / cicli            1 / 1 / 0
terminazione               grf_penetration
```

Questo risultato non invalida automaticamente il detector: dimostra che il
checkpoint addestrato con la temporizzazione legacy non e' compatibile, allo
stato attuale, con lo switch autoritativo. La matrice preregistrata da 72 casi
e' stata lasciata pronta ma non eseguita, poiche' il primo smoke aveva gia'
reso impossibile la promozione.

## Decisione e stato finale

```text
acquisizione heel/toe detector-only          VALIDATA
regole HS/TO nella FSM esistente              VALIDATE
shadow del checkpoint corrente                IDENTICO
V3 development e sensitivity                  PASS
V3 holdout 50-100 s                           FAIL
V4 mesh geometria                             PASS
V4 mesh detector                              FAIL
sealed 100-155.045 s                          NON APERTO
two_sensor default/training                    NON PROMOSSO
phase_fsm_input_mode                           legacy_events
training sul nuovo detector                    NON ESEGUITO
```

Prima di qualunque adattamento della policy occorre ottenere e congelare un
detector che superi il protocollo prescribed. Il training non e' necessario
per diagnosticare il detector.

## File modificati o aggiunti

### Runtime e configurazione

- `online_grf.py`
- `simulation_runner.py`
- `Trajectory Generator/prosthetic_phase_fsm.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`

### Validazione

- `validation/test_prosthetic_phase_fsm_two_sensor.py`
- `validation/test_detector_sensor_data_path.py`
- `validation/validate_two_sensor_prescribed_replay.py`
- `validation/validate_two_sensor_forward_states.py`
- `validation/sweep_two_sensor_prescribed_thresholds.py`
- `validation/two_sensor_prescribed_threshold_sweep_protocol_v3.json`
- `validation/audit_two_sensor_prescribed_geometry.py`
- `validation/test_two_sensor_prescribed_geometry_audit.py`
- `validation/build_two_sensor_mesh_profile_v4.py`
- `validation/compare_two_sensor_mesh_profiles_prescribed.py`
- relativi test, protocolli e plotter in `validation/`.

### Profili e artefatti

- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO_v4_mesh_experimental.json`
- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/`
- `validation/two_sensor_geometry_audit_runs/2026-07-21_v4_design_audit/`
- `validation/two_sensor_geometry_audit_runs/2026-07-21_v4_mesh_profile_design_check/`
- `validation/two_sensor_mesh_profile_v4_fixed_replay_runs/2026-07-21_ab06_50_100_fixed_v4/`
- `validation/heel_detector_validation_runs/2026-07-21_best_nominal_shadow/`
- `validation/heel_detector_validation_runs/2026-07-21_best_nominal_two_sensor/`

I manifest e gli output numerici principali risultano presenti. Alcune
directory di plot citate nei report del giorno non sono invece presenti nel
workspace corrente; questo e' un limite archivistico da non confondere con i
risultati numerici conservati nei manifest.

## Test e verifiche

- logica FSM two-sensor iniziale: `11/11` PASS;
- suite mirata iniziale FSM/data path/replay: `30/30` PASS;
- protocollo V3: `21/21` test sweep e `7/7` test phase gate PASS;
- threshold + FSM V3: `32/32` PASS;
- audit geometrico: 8 test dedicati e regressioni PASS;
- V4 mesh, core e regressioni: `83` controlli PASS;
- replay prescribed e forward-state a 10 ms e 1 ms completati;
- pinning SHA-256, ricostruzione development e protezioni anti-leakage PASS;
- `py_compile`, Ruff, validazione JSON e `git diff --check`: PASS;
- sealed verificato non aperto;
- nessun processo training avviato per validare il detector.

## TODO aperti e propagati

### Detector e FSM

- [ ] Creare una revisione sperimentale separata con il forefoot sotto una
      regione realmente load-bearing, senza sovrascrivere il profilo corrente.
- [ ] Mantenere esattamente due sensori detector-only, soglie/dwell espliciti e
      la FSM esistente; non usare i sensori come generatori GRF.
- [ ] Ripetere gate geometrico e fixed replay direttamente a `10 ms` e `1 ms`
      prima di qualunque promozione.
- [ ] Congelare geometria, semantica, soglie, dwell, routing e hash prima di
      aprire un nuovo holdout; mantenere chiuso il sealed fino a un PASS
      preregistrato.
- [ ] Chiarire il limite della reference: la GRF totale valida direttamente
      l'initial contact, non un heel strike anatomico localizzato.
- [ ] Soltanto dopo un PASS prescribed, eseguire shadow e active A/B sul
      checkpoint congelato; fare fine-tuning solo se il detector valido riduce
      realmente la robustezza della policy.
- [ ] Conservare `legacy_events` come default fino al completamento dei gate.

### Morphology Corridor, propagati dal 2026-07-20

Nessuno dei TODO del corridor del 20 luglio e' stato chiuso dal lavoro sul
detector del 21 luglio.

- [ ] Completare un rollout live shadow da 500 step della modalita'
      complete-segment in una sessione OpenSim pulita e ripetere il confronto
      A/B; lo shadow del detector svolto oggi non sostituisce questo test.
- [ ] Scegliere il design definitivo del bordo esterno phase-dependent e
      chiarire se la sicurezza debba agire immediatamente o retrospettivamente.
- [ ] Implementare e testare la riscrittura complete-segment/complete-episode
      delle reward prima del calcolo GAE.
- [ ] Gestire esplicitamente segmenti incompleti, timeout e bootstrap nel
      ledger completed-segment.
- [ ] Definire il trattamento di `WAIT_HS`, dello stato temporale non
      completamente osservato e del regime ankle lento.
- [ ] Eseguire soltanto dopo questi gate un A/B breve a morphology weight
      `0 / 0.0025 / 0.005`, senza cambiare contemporaneamente learning rate.
- [ ] Conservare ogni update e schermare loss morphology, uscite dal corridor,
      cicli, penetrazione, return, SEA, reserve e drift dell'actor.
- [ ] Aggiungere logging per-evento e partire soltanto con un peso piccolo,
      senza promuovere il termine sulla base del solo replay offline.
- [ ] Validare il pilot su start e seed held-out e su almeno un
      profilo/modello esterno ad AB06 prima di training lungo o promozione.
- [ ] Mantenere le reserve come gate fisico, non come reward, finche' la GRF
      non sia validata o l'origine del residuo non sia attribuibile.

### Training, reward e deployability

- [ ] Conservare il checkpoint best/H0 e tutti i rollback; non usare il
      fallimento del detector per modificare reward, soglia da `25 mm` o
      informazioni actor.
- [ ] Selezionare i checkpoint futuri con robustezza multistart e held-out,
      non soltanto con return PPO nominale.
- [ ] Integrare nei gate penetrazione, cicli, reserve, SEA, clipping, distanza
      dall'actor iniziale e recovery del worst case.
- [ ] Risolvere o caratterizzare la dipendenza da GRF online e reserve prima di
      attribuire alla sola policy un failure fisico.
- [ ] Eseguire prove di durata maggiore, trial, velocita' e soggetti differenti
      prima di dichiarare generalizzazione o deployment validato.
- [ ] Ripartire da H0 o da una sorgente preregistrata, non da logical 51 o dal
      best selezionato per return.
- [ ] Mantenere exact-start, compaction, interleaving, singola epoca e gate
      reserve condition-matched.
- [ ] Concentrare recovery e raccolta dati sugli stati/eventi intorno agli
      step 210-230.
- [ ] Progettare e preregistrare la non-regressione delle reserve tramite
      reward, vincolo o criterio di update, senza confonderla con errori GRF.
- [ ] Mantenere i seed 126-128 sigillati finche' un candidato non supera tutta
      la matrice development; poi usare un solo candidato sugli held-out.
- [ ] Eseguire un nuovo pilot controllato prima di un altro run lungo.
- [ ] Analizzare return e advantage separatamente per start.
- [ ] Non eseguire altri dimezzamenti della learning rate o proiezioni sulla
      stessa trace/seed; raccogliere recovery data event-aligned indipendenti.
- [ ] Riprovare `sigma=0.0075` su almeno tre seed per start soltanto dopo un
      PASS completo a `sigma=0.005`.
- [ ] Mantenere differita una memoria ricorrente finche' non emerga un limite
      sequenziale non coperto dallo stato Markov.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo.

### Audit ex-novo e contratto di deployment

- [ ] Confrontare H0 e logical 24 con protocollo identico e overlay di azioni,
      served, cinematica, SEA, GRF, eventi e reward.
- [ ] Eseguire ablation progressive del plant prescribed: reset sensor-based,
      perturbazioni encoder/carico, detector, GRF online bilaterale e riduzione
      della dipendenza dalla IK biologica.
- [ ] Eseguire training A/B da H0 modificando un solo gruppo reward per volta:
      detector/FSM, ramo GRF diretto, morphology e command-rate.
- [ ] Congelare ordine e scaling delle feature, frequenza, unita',
      normalizzazione BW, filtri, FSM, reset, governor, limiti, fallback e
      watchdog del contratto hardware.
- [ ] Validare export actor-only, equivalenza host-target, latenza worst-case,
      HIL e poi human-in-the-loop.

### Hardware e TODO storici

- [ ] Validare rumore sensori, delay, domain randomization, HIL e una geometria
      hardware reale prima del deployment; il profilo AB06 non e' una specifica
      fisica del futuro dispositivo.
- [ ] Portare il modello GRF online a uno stato production-ready e validare
      separatamente il routing della GRF primaria rispetto ai sensori detector.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta,
      TODO storico ancora aperto.
