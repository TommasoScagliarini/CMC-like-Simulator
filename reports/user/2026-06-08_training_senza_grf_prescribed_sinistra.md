# Training diagnostico senza GRF prescribed sinistra

## Obiettivo

Eseguire un training breve della baseline MLP/RLlib con:

- modalita GRF `online_sensor`;
- sensore onlineGRF attivo e visibile alla rete;
- GRF prescribed della gamba protesica sinistra rimossa dalla dinamica;
- GRF prescribed destra applicata normalmente.

Lo scopo era osservare il comportamento congiunto di rete e simulatore in
assenza del supporto esterno sinistro e verificare se la pipeline rilevasse o
compensasse questa condizione.

## Problema

La validazione precedente della modalita `online_sensor` aveva mostrato che il
sensore onlineGRF era capace di rilevare gli eventi gait nel perimetro testato.
Restava pero da capire se rete e simulatore fossero sensibili alla presenza
fisica della GRF applicata, oppure se tracking cinematico e reserve potessero
mascherarne l'assenza.

## Semantica del test

La GRF prescribed sinistra non e stata azzerata nello Storage, perche lo
Storage prescribed viene usato anche come oracle diagnostico. E stata invece
omessa dal modello la relativa `ExternalForce`.

Di conseguenza:

- la forza prescribed sinistra non contribuisce alla dinamica;
- i dati prescribed sinistri restano disponibili come oracle;
- il sensore onlineGRF sinistro continua a stimare il contatto virtuale dallo
  stato corrente;
- in modalita `online_sensor`, la forza stimata online non viene applicata al
  modello.

Il test rappresenta quindi una condizione in cui nessuna GRF e applicata alla
gamba protesica sinistra, pur mantenendo osservabili i segnali onlineGRF.

## Strategia

E stata aggiunta un'opzione diagnostica opt-in:

```text
--disable-prescribed-grf-side left
```

Il flag e disponibile sia nel training sia nel rollout. Il loader valida i lati
richiesti e non aggiunge al modello le `ExternalForce` prescribed selezionate,
senza modificare la modalita GRF predefinita o il plugin C++ onlineGRF.

Prima del training e stato eseguito uno smoke test da `0.08 s`, che ha
confermato nel log:

```text
left_ground_force1  -> skipped
right_ground_force2 -> addForce OK
online GRF contacts -> sensor-only
```

Configurazione del training breve:

```text
output_dir:                         runs/_baseline_mlp_no_left_prescribed_short_20260608
iterations:                         5
num_env_runners:                    1
ray_num_cpus:                       2
train_batch_size:                   50
minibatch_size:                     10
num_epochs:                         2
episode_duration:                   1.0 s
segment_duration:                   0.02 s
network:                            64 64
grf_mode:                           online_sensor
disable_prescribed_grf_side:        left
online_grf_observation:             true
```

Dopo il training sono stati eseguiti due rollout deterministici dello stesso
checkpoint migliore:

1. rollout con GRF prescribed sinistra disabilitata;
2. controllo con GRF prescribed bilaterali.

Questo confronto separa l'effetto della GRF mancante dal comportamento appreso
dalla rete durante il training.

## Esito del training

Il training ha completato tutte le `5/5` iterazioni:

```text
elapsed wall time: 898.13 s
timeout:           nessuno
best return mean:  34.331
```

| Iterazione | Return medio | Lunghezza media episodio | Esito cumulativo |
|---:|---:|---:|---|
| 1 | `6.373` | `14` step | `1` divergenza knee |
| 2 | `32.470` | `50` step | `1` episodio completo |
| 3 | `34.331` | `50` step | `2` episodi completi |
| 4 | `33.765` | `50` step | `3` episodi completi |
| 5 | `33.069` | `50` step | `4` episodi completi |

Dopo una divergenza iniziale del ginocchio protesico, quattro episodi hanno
raggiunto correttamente il limite temporale da `1.0 s`.

Il critic non mostra apprendimento affidabile nel run breve:

```text
vf_explained_var:
  +0.0264
  -0.0225
  +0.0039
  -0.0421
  +0.0011
```

## Confronto dei rollout

Entrambi i rollout completano `1.0 s` senza terminazioni unsafe.

| Metrica | Senza GRF prescribed sx | Controllo bilaterale |
|---|---:|---:|
| Reward medio | `0.9814` | `0.9800` |
| Episode return | `49.070` | `48.999` |
| Altezza minima bacino | `0.9552 m` | `0.9549 m` |
| Massimo valore assoluto action | `0.1892` | `0.1859` |
| Tau reserve norm media | `544.4` | `135.8` |
| Tau reserve norm massima | `915.4` | `583.7` |
| Muscle share media | `0.126` | `0.392` |
| Reserve control norm media | `5.444` | `1.358` |

Reward e altezza del bacino restano quasi identiche. L'assenza della GRF
sinistra emerge invece chiaramente nella recruitment diagnostics.

### Compensazione tramite reserve

La compensazione dominante avviene sulla coordinata verticale del bacino:

| `pelvis_ty_reserve_torque` | Senza GRF sx | Controllo bilaterale |
|---|---:|---:|
| Media | `513.6 N` | `6.6 N` |
| Media assoluta | `513.6 N` | `19.2 N` |
| Massimo | `908.5 N` | `78.9 N` |

La differenza RMSE tra i due rollout sulla sola reserve `pelvis_ty` e
`539.4 N`.

Il simulatore mantiene quindi la cinematica e l'altezza del bacino applicando
una grande forza verticale residuale, che sostituisce artificialmente il
supporto esterno sinistro mancante.

### Cinematica

Le traiettorie restano molto simili nel breve orizzonte. Le maggiori differenze
tra i due rollout sono:

| Coordinata | RMSE | Massima differenza assoluta |
|---|---:|---:|
| `pros_ankle_angle` | `0.0207 rad` | `0.0467 rad` |
| `hip_flexion_l` | `0.0077 rad` | `0.0187 rad` |
| `pros_knee_angle` | `0.0056 rad` | `0.0185 rad` |

La differenza principale sulla caviglia protesica e circa `1.19 deg` RMS.

### Attuazione SEA

| Metrica | Senza GRF sx | Controllo bilaterale |
|---|---:|---:|
| Knee `abs(tau_input)` medio | `67.9 Nm` | `68.1 Nm` |
| Knee `abs(tau_input)` massimo | `466.1 Nm` | `458.6 Nm` |
| Knee campioni saturati | `0 / 1000` | `0 / 1000` |
| Ankle `abs(tau_input)` medio | `3.0 Nm` | `30.8 Nm` |
| Ankle `abs(tau_input)` massimo | `9.8 Nm` | `109.4 Nm` |
| Ankle campioni saturati | `0 / 1000` | `0 / 1000` |

La SEA della caviglia risulta quasi scarica quando manca la GRF applicata
sinistra. Il ginocchio mantiene invece un carico simile nei due casi.

### Segnale onlineGRF

Il sensore online continua a rilevare contatto sinistro per tutto il rollout,
anche se la sua forza non viene applicata:

```text
left contact fraction:       1.0
left normal force mean:      637.1 N
left normal force max:       1128.4 N
left penetration mean:       0.0956 m
left penetration max:        0.1067 m
```

Questo non e una contraddizione: in modalita `online_sensor` il contatto online
e solo osservato e registrato. Il test rimuove la forza prescribed sinistra, ma
non trasforma il sensore online in una forza applicata.

## Interpretazione

Il test mostra che la pipeline corrente puo mantenere reward elevato e
cinematica stabile anche in assenza completa della GRF applicata sulla gamba
protesica sinistra.

La causa e la combinazione di:

- tracking cinematico molto forte;
- reserve del bacino sufficientemente autorevoli da sostituire il supporto
  verticale mancante;
- reward biologica basata sulla deviazione cinematica delle coordinate, non sul
  costo delle reserve;
- nessuna penalty diretta sulla coerenza tra onlineGRF osservata e forza
  effettivamente applicata.

Il reward medio leggermente migliore senza GRF sinistra non indica quindi una
dinamica migliore. Indica che la funzione obiettivo attuale non distingue
adeguatamente questa compensazione non fisica.

## Soluzione implementata

E stata introdotta una modalita diagnostica riutilizzabile senza cambiare i
default:

- `SimulatorConfig.prescribed_grf_disabled_sides`;
- validazione dei lati nel loader;
- omissione selettiva delle `ExternalForce` prescribed;
- propagazione del flag nell'ambiente RL, training e rollout;
- registrazione del flag nei summary e nel run status;
- documentazione nel README della baseline MLP.

Il plugin C++ SEA e il plugin C++ onlineGRF non sono stati modificati.

## File modificati

```text
config.py
model_loader.py
simulation_runner.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/README.md
```

## Test e verifiche

- `py_compile` dei file Python modificati: superato;
- verifica CLI training e rollout: flag presente;
- smoke training da `0.08 s`: superato;
- training diagnostico da `5` iterazioni: completato;
- rollout senza GRF prescribed sinistra: completato;
- rollout di controllo bilaterale con lo stesso checkpoint: completato;
- controllo output `.sto`: valori finiti;
- controllo saturazione SEA: nessuna saturazione nei rollout da `1 s`;
- `git diff --check`: superato, salvo warning attesi LF/CRLF;
- cleanup finale: nessun processo Ray/Conda/Python del test rimasto attivo.

## Artefatti principali

```text
runs/_baseline_mlp_no_left_prescribed_smoke_20260608/
runs/_baseline_mlp_no_left_prescribed_short_20260608/
runs/_baseline_mlp_no_left_prescribed_short_20260608_rollout_no_left/
runs/_baseline_mlp_no_left_prescribed_short_20260608_rollout_bilateral_control/
```

## Limiti

- Il training e molto breve e non dimostra convergenza della policy.
- I rollout durano `1.0 s` e non coprono un gait cycle completo.
- Il checkpoint e stato addestrato senza GRF prescribed sinistra; il confronto
  bilaterale isola l'effetto dinamico nel rollout, ma non sostituisce un training
  comparativo bilaterale completo.
- La grande penetrazione rilevata dal sensore online conferma che lo stato
  mantenuto dalle reserve non rappresenta un contatto fisicamente valido.

## TODO

- Aggiungere alla reward una penalty esplicita sulle reserve, almeno sulle
  reserve del bacino e in particolare su `pelvis_ty`.
- Esporre e penalizzare la discrepanza tra GRF online osservata e forza
  effettivamente applicata al modello.
- Ripetere il confronto su episodi full-gait solo dopo avere introdotto una
  misura che impedisca alle reserve di mascherare la GRF mancante.
