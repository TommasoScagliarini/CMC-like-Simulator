# Sweep cascade ankle e aggiornamento configurazione - 2026-05-18

## Problema

Lo sweep locale cascade `results/_cascade_local_gain_sweep_20260517_234151`
non aveva prodotto combinazioni full perche lo script si era fermato prima
delle full run:

```text
error = no acceptable stage1 candidates
```

La causa non era un crash dello sweep. I risultati stage 1 erano stati salvati,
ma il gate di accettazione era troppo rigido sul knee:

- `stage1_knee_screen.csv`: 125 candidati, 125 completi/finiti, 0 accettabili;
- `stage1_ankle_screen.csv`: 125 candidati, 125 completi/finiti, 5 accettabili;
- tutti i knee candidate fallivano per `tau_input_saturated` e
  `tau_input_raw_gt_500`.

Il dubbio operativo era se fosse possibile proseguire usando i risultati stage 1
senza rilanciare tutto da capo.

## Strategia

Sono stati riusati i file gia prodotti dallo sweep:

```text
results/_cascade_local_gain_sweep_20260517_234151/stage1_knee_screen.csv
results/_cascade_local_gain_sweep_20260517_234151/stage1_ankle_screen.csv
results/_cascade_local_gain_sweep_20260517_234151/failures.csv
results/_cascade_local_gain_sweep_20260517_234151/baseline_metrics.csv
results/_cascade_local_gain_sweep_20260517_234151/sweep_summary.json
```

Poiche solo lo stage ankle aveva candidati accettabili, sono state lanciate
cinque full run usando:

- knee alla configurazione cascade corrente;
- ankle variato sui 5 candidati accettabili dello stage 1;
- setup/model PI `321/500`;
- filtro GRF attivo;
- baseline omogenea per normalizzazione:
  `results/_cascade_aggressive_full_20260517`.

La nuova cartella di lavoro e':

```text
results/_cascade_full_ankle5_20260518_105345
```

## Soluzione e risultati

Le 5 full run sono terminate tutte con esito `OK` e sono tutte risultate
`acceptable`, senza saturazioni.

File prodotti:

```text
results/_cascade_full_ankle5_20260518_105345/full_results.csv
results/_cascade_full_ankle5_20260518_105345/ranking.csv
results/_cascade_full_ankle5_20260518_105345/best_candidate.json
results/_cascade_full_ankle5_20260518_105345/sweep_summary.json
results/_cascade_full_ankle5_20260518_105345/source_candidates.json
```

Best candidate full:

```text
run_id = full_ankle5_kpo18p85_kpi29p2_kii1377_kil50_apo47p125_api2p8275_aii213_ail200

knee:
  Kp_outer = 18.85
  Kp_inner = 29.2
  Ki_inner = 1377.0
  I_limit  = 50.0

ankle:
  Kp_outer = 47.125
  Kp_inner = 2.8275
  Ki_inner = 213.0
  I_limit  = 200.0
```

Metriche principali:

```text
score_kinematic_deg = 0.907516
knee RMS            = 0.177624 deg
ankle RMS           = 1.220328 deg
score_chattering    = 1.425529
score_power         = 0.758111
sat_count           = 0
max tau_input_raw   = 124.823 Nm
knee max |u|        = 0.331625
ankle max |u|       = 0.481839
```

Interpretazione:

- il candidato migliora fortemente il tracking ankle rispetto alla baseline
  cascade aggressive usata per normalizzare;
- non introduce saturazioni di `tau_input` o del comando `u`;
- paga un aumento del chattering normalizzato;
- riduce invece la metrica normalizzata di motor power.

## Plot

E' stato generato il plot del best candidate ankle con knee alla configurazione
corrente:

```text
plot/05_18_2026_1
```

Il plotter ha caricato correttamente:

- setup PI `AB06_SEASEA_stiff321_500_pi_setup.xml`;
- modello PI `AB06_SEASEA_stiff321_500_pi.osim`;
- healthy overlay AB06;
- 4 gait cycle left e 4 right;
- reference kinematics `IK_results_AB06_SEASEA.mot`.

Il file:

```text
plot/05_18_2026_1/missing_channels.txt
```

riporta:

```text
No missing channels.
```

## File modificati

E' stato aggiornato:

```text
config.py
```

Nuova configurazione cascade ankle:

```python
sea_cascade_kp_outer = {
    "pros_knee_angle":  18.85,
    "pros_ankle_angle": 47.125,
}

sea_cascade_kp_inner = {
    "pros_knee_angle":  29.2,
    "pros_ankle_angle": 2.8275,
}

sea_cascade_ki_inner = {
    "pros_knee_angle":  1377.0,
    "pros_ankle_angle": 213.0,
}

sea_cascade_inner_i_torque_limit = {
    "pros_knee_angle":  50.0,
    "pros_ankle_angle": 200.0,
}
```

Il knee e' rimasto invariato.

## Verifiche eseguite

Verifiche sui risultati stage 1:

- confermata presenza di `stage1_knee_screen.csv` e
  `stage1_ankle_screen.csv`;
- confermato che lo stage knee aveva 0 candidati accettabili;
- confermato che lo stage ankle aveva 5 candidati accettabili.

Full run:

- lanciate 5 full run su finestra `11.99 -> 21.00 s`;
- tutte complete;
- tutte finite;
- tutte acceptable;
- nessuna saturazione `tau_input`;
- nessuna saturazione comando `u`.

Plot:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python plot/plotter.py \
  --results-dir results/_cascade_full_ankle5_20260518_105345/full_runs/full_ankle5_kpo18p85_kpi29p2_kii1377_kil50_apo47p125_api2p8275_aii213_ail200 \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim
```

Esito:

```text
No missing channels.
Plots saved to: plot/05_18_2026_1
```

Compilazione:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py
```

Esito: OK.

## TODO

- Decidere se fare un secondo pass sul knee: lo stage 1 knee e' stato bloccato
  solo dal gate rigido (`tau_input_raw > 500` e poche saturazioni), ma contiene
  comunque candidati completi/finiti che potrebbero essere riesaminati con
  criteri rilassati o con una griglia meno aggressiva.
- Confrontare il nuovo default ankle contro PD/PI/cascade precedenti nei plot e
  nelle metriche HPF/power, per decidere se il peggioramento del chattering
  normalizzato e' accettabile rispetto al guadagno di tracking.
- Propagare su Windows la DLL/plugin PI aggiornata e verificare il modello PI
  con `Ki`, `integral_torque_limit` e `torque_error_integral`.
- **Valutare l'impatto della mancata cancellazione zero/polo nell'ankle**: con
  il redesign del motor driver (ω_n=280, ζ=0.7, p=0.2·ω_n) lo zero PI a -43.75
  e il polo reale a -56 hanno miss del 22% (intrinseco alla coppia p/ω_n=0.2,
  ζ=0.7). Verificare in simulazione se la coda lenta residua a 8.9 Hz produce
  artefatti visibili su `tau_spring`, motor power o chattering. Se rilevante,
  considerare riduzione del rapporto p/ω_n (es. a 0.1) per migliorare la
  cancellazione a costo di un integratore piu' lento.
