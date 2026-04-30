# Daily Report - 2026-04-28

## Sintesi

La giornata del 28 aprile 2026 ha avuto due filoni principali:

1. rendere il simulatore piu robusto nella gestione dei file di input, passando
   da path flat/hardcoded a un layout a bundle e a setup XML espliciti;
2. diagnosticare il nuovo setup `newmarkers`, costruito sul modello
   `Adjusted_newmarkers.osim` e su un riferimento cinematico sano.

Il primo filone ha introdotto una base di I/O piu usabile:

- `path_resolver.py` centralizza la risoluzione dei path;
- `SimulatorConfig` usa `model_bundle_dir`;
- `main.py` supporta setup XML, riuso dell'ultimo setup valido e override CLI;
- `setup_io.py` gestisce lettura/scrittura setup XML e stato persistito;
- `create_setup.py` fornisce un wizard `tkinter` per creare setup XML.

Il secondo filone ha chiarito che il setup newmarkers non fallisce piu per
caricamento, dopo le correzioni a GRF e modello, ma resta fisicamente critico
per via di spike nel riferimento IK e di copertura GRF limitata.

## Report Utente Considerati

- `reports/user/2026-04-28_setup_xml_gui_e_bundle_model_paths.md`
- `reports/user/2026-04-28_diagnosi_newmarkers_grf_modello_e_spike_ik.md`

## 1. Refactor bundle-aware e setup XML

Il progetto e stato adattato a una struttura in cui ogni modello vive dentro un
bundle:

```text
models/<bundle>/
  *.osim
  data/
    *.sto
    *.mot
    *.xml
```

La configurazione ora tratta `model_bundle_dir` come unita primaria. I file
`model_file`, `kinematics_file`, `external_loads_xml` e
`reserve_actuators_xml` vengono risolti rispetto al bundle attivo, salvo path
assoluti o path repo-relative espliciti.

### File introdotti

- `path_resolver.py`
- `setup_io.py`
- `create_setup.py`

### File aggiornati

- `config.py`
- `main.py`
- `model_loader.py`
- `kinematics_interpolator.py`
- `visualize.py`
- `plot/plotter.py`
- `validation/validate_sim_results.py`
- `validation/hpf_noise_metric.py`
- `validation/cmc_strict_metrics.py`
- `validation/outer_gain_sweep.py`
- `validation/sea_driver_sweep.py`
- `validation/sea_inner_10020_sweep.py`
- `validation/_tmp_sea_parameter_sweep.py`
- `.gitignore`

### Funzioni nuove

`main.py` supporta ora:

```text
python main.py --setup
python main.py --setup path/to/setup.xml
python main.py
```

Nel terzo caso il simulatore prova a riusare l'ultimo setup valido salvato in:

```text
.simulator_last_setup.json
```

Sono stati aggiunti anche gli override CLI:

```text
--external-loads
--reserve-actuators
```

Gli override espliciti hanno precedenza sul setup XML e impediscono l'auto-load
dell'ultimo setup, cosi i flussi batch e gli sweep restano controllabili.

## 2. Verifiche sul refactor I/O

Sono state eseguite verifiche di compilazione e smoke test:

```text
conda run -n envCMC-like python -m py_compile config.py path_resolver.py model_loader.py main.py kinematics_interpolator.py visualize.py plot/plotter.py validation/validate_sim_results.py validation/hpf_noise_metric.py validation/cmc_strict_metrics.py validation/outer_gain_sweep.py validation/sea_driver_sweep.py validation/sea_inner_10020_sweep.py validation/_tmp_sea_parameter_sweep.py
```

Smoke run con configurazione corrente:

```text
conda run -n envCMC-like python main.py --t-start 4.26 --t-end 4.261 --output-dir results/_bundle_smoke_current_cfg
```

Smoke run con bundle e modello espliciti:

```text
conda run -n envCMC-like python main.py --model-bundle models/SEASEA --model "Adjusted_SEASEA - Copia_tuned.osim" --t-start 4.26 --t-end 4.261 --output-dir results/_bundle_smoke_explicit
```

Plotter e validator sono stati verificati sul nuovo layout. Il validator arriva
ai check numerici; eventuali `FAIL` in smoke run a 1 step sono fisici/numerici,
non errori di path.

La feature setup XML e stata verificata con:

- `python -m py_compile setup_io.py create_setup.py main.py`;
- `conda run -n envCMC-like python -m py_compile setup_io.py create_setup.py main.py`;
- round-trip reale di creazione, lettura e persistenza setup;
- run esplicita con `--setup`;
- run con riuso dell'ultimo setup;
- run con override diretti che bypassano l'auto-load;
- import del wizard GUI `create_setup.py`.

La GUI `tkinter` e stata importata correttamente ma non cliccata end-to-end in
modo automatico.

## 3. Diagnosi newmarkers

Il setup:

```text
models/SEASEA - whealthy data/Adjusted_newmarkers_setup.xml
```

punta a:

- `Adjusted_newmarkers.osim`;
- `data/3DGaitModel2392-scaled_Kinematics_q.sto`;
- `data/ExternalForces.xml`;
- `data/CMC_Actuators.xml`.

L'obiettivo e usare un modello scalato su marker sani e far inseguire alla
protesi un riferimento sano.

### Problema GRF iniziale

Una prima versione del file `ExternalForces.xml` aveva gli identificatori
scambiati:

```text
force_identifier = FP1_p
point_identifier = FP1_v
```

Nel `.mot`, pero:

- `FP*_v` sono le forze;
- `FP*_p` sono i punti;
- `FP*_moment_` sono i momenti.

Questo produceva carichi irrealistici. Dopo la correzione, il file usa:

```text
right: FP1_v, FP1_p, FP1_moment_ -> calcn_r
left : FP2_v, FP2_p, FP2_moment_ -> foot_l
```

Il loader conferma:

```text
'right' -> addForce OK (applied_to: calcn_r, force_id: FP1_v)
'left'  -> addForce OK (applied_to: foot_l, force_id: FP2_v)
```

### Problema modello iniziale

Durante una verifica intermedia, `Adjusted_newmarkers.osim` non conteneva piu
gli attuatori SEA nel `ForceSet`, e il loader falliva con:

```text
[ModelLoader] Attuatore 'SEA_Knee' non trovato nell'ActuatorSet.
```

Il modello e stato corretto reinserendo:

- `SeriesElasticActuator name="SEA_Knee"` su `pros_knee_angle`;
- `SeriesElasticActuator name="SEA_Ankle"` su `pros_ankle_angle`.

I parametri SEA risultano coerenti con il modello tuned:

```text
SEA_Knee : F_opt=100, K=1000, Kp=3.9, Kd=9.7
SEA_Ankle: F_opt=250, K=700,  Kp=8.8, Kd=9.7
```

### Stato dopo correzioni

Dopo le correzioni a GRF e modello:

- il plugin viene caricato;
- il modello viene inizializzato;
- le GRF vengono lette;
- gli attuatori SEA vengono trovati;
- una smoke run `16.900 -> 16.901 s` completa;
- il tratto `16.900 -> 17.100 s` completa;
- il tratto critico `19.600 -> 19.750 s` completa.

Quindi il fallimento di setup/caricamento e risolto.

## 4. Problema residuo: riferimento fuori scala

Il tratto critico resta dinamicamente problematico.

Test `19.600 -> 19.750 s`:

```text
status=complete
step=150
SEA max u:
  knee  = 1.0
  ankle = 1.0
SEA_Ankle_tau_input_raw max ~= 819.9 Nm
SEA_Ankle_tau_input_saturated = 1.0
pelvis_ty_reserve_torque max ~= 1080 Nm
pelvis_tx_reserve_torque max ~= 582 Nm
```

Questo indica che il setup ora gira, ma il riferimento sta chiedendo una
dinamica fuori scala per il sistema corrente.

La causa piu forte individuata e uno spike nel file:

```text
models/SEASEA - whealthy data/data/3DGaitModel2392-scaled_Kinematics_q.sto
```

In particolare:

- `pros_ankle_angle` cambia molto rapidamente attorno a `19.655 s`;
- la velocita numerica stimata arriva a circa `9300 deg/s`;
- `ankle_angle_r` mostra picchi alti attorno a `17.56 s`.

Con cutoff IK a `2 Hz`, sul tratto critico erano stati osservati miglioramenti:

```text
max reserve_tau           ~= 1106 -> 261 Nm
SEA_Ankle max |u|         ~= 1.0  -> 0.79
SEA_Ankle_tau_input_raw   ~= 820  -> 362 Nm
```

Quindi il filtro IK a `6 Hz` e probabilmente troppo permissivo per questo
dataset.

## 5. Copertura GRF

Nel `.mot` usato dal setup, dentro la finestra `16.9 -> 22.999`, i contatti
utili sopra soglia sono:

```text
FP1_vy > 20 N: 16.9   -> 19.172 s
FP2_vy > 20 N: 18.965 -> 19.839 s
```

Dopo circa `19.839 s`, il riferimento cinematico continua ma le GRF utili sono
praticamente assenti. Questo puo contribuire ai reserve elevati su pelvis e
anca.

## Stato Finale

A fine giornata:

- la nuova infrastruttura bundle/setup XML e operativa;
- il setup newmarkers carica i file corretti;
- `ExternalForces.xml` e coerente con il `.mot`;
- `Adjusted_newmarkers.osim` contiene di nuovo i due SEA;
- i test brevi completano;
- resta un problema dinamico nel riferimento sano, soprattutto attorno a
  `19.6 -> 19.75 s`.

## Raccomandazioni Per La Prossima Sessione

1. Pulire o filtrare il riferimento IK newmarkers, partendo da
   `pros_ankle_angle` attorno a `19.655 s`.
2. Provare una baseline diagnostica con cutoff IK `2 Hz`.
3. Limitare temporaneamente `t_end` entro la copertura GRF utile, ad esempio
   prima di `19.839 s`.
4. Verificare che IK e GRF appartengano allo stesso trial e siano
   temporalmente allineati.
5. Solo dopo la pulizia del riferimento, rieseguire una run piu lunga e il
   validator per distinguere tracking fisico da problemi residui di setup.
