# SEA Motor-Driver Parameter Sweep

Data: 2026-04-17

## Problema

Lo sweep e stato lanciato per trovare una combinazione robusta di `K`, `Kp` e
`Kd` per i due attuatori SEA (`SEA_Knee`, `SEA_Ankle`) in modo non-impedance.
L'obiettivo pratico era rendere l'inner loop PD del plugin un motor driver piu
trasparente: dato `tau_ref = u * F_opt`, il sistema dovrebbe produrre una
`tau_spring` pulita, senza chattering o saturazioni, mantenendo il tracking
cinematico entro limiti accettabili.

Lo sweep usa la parametrizzazione:

```text
Kp = Jm * omega_n^2 / K - 1
Kd = 2 * zeta * Jm * omega_n - Bm
```

## Strategia

E' stato analizzato l'output dello sweep completo in:

```text
results/_sea_driver_sweep_20260417_181301/
```

La procedura ha eseguito:

- screening su 198 candidati;
- full validation sui migliori 5 candidati;
- generazione di `sweep_results.csv`;
- generazione di `best_candidate.json`;
- salvataggio degli output per-candidato in `runs/`.

## Risultati sintetici

Totale righe analizzate: 203.

| Fase | Run totali | Run accettabili per lo script |
|------|-----------:|------------------------------:|
| screening | 198 | 143 |
| full validation | 5 | 5 |

La simulazione risulta completa per 202 run su 203. L'unico run incompleto e:

```text
screen_kk750_ka750_wn300_z0p7
```

con `run_status=failed`, saturazione in 52 campioni e `max |u| = 1.000`.

## Best candidate dello scoring

Il miglior candidato secondo lo scoring dello script e:

```text
full_kk500_ka500_wn900_z0p7
```

Parametri:

| Attuatore | K | Kp | Kd |
|-----------|--:|---:|---:|
| Knee | 500.0 | 15.2 | 12.5 |
| Ankle | 500.0 | 15.2 | 12.5 |

Parametri di sweep:

| omega_n | zeta | max_motor_torque |
|--------:|-----:|-----------------:|
| 900.0 | 0.7 | 5000.0 |

Metriche full validation:

| Metrica | Valore |
|---------|-------:|
| Score | 4.1328 |
| Knee tracking RMS | 3.8505 deg |
| Ankle tracking RMS | 4.3746 deg |
| Knee noise fraction | 0.5379 |
| Ankle noise fraction | 0.0106 |
| Worst noise fraction | 0.5379 |
| Max \|tau_input\| | 444.4306 Nm |
| Max \|u\| | 0.3098 |
| Saturazioni `tau_input` | 0 |
| Output finiti | true |
| Run status | complete |

Il candidato e solido su tracking, assenza di saturazione, ampiezza del comando
e finitezza degli output. Il limite principale e la noise fraction del ginocchio.

## Classifica full validation

| # | Run | Score | K knee | K ankle | omega_n | zeta | Knee RMS | Ankle RMS | Worst noise | Max tau | Max u |
|--:|-----|------:|-------:|--------:|--------:|-----:|---------:|----------:|------------:|--------:|------:|
| 1 | `full_kk500_ka500_wn900_z0p7` | 4.1328 | 500 | 500 | 900 | 0.70 | 3.8505 | 4.3746 | 0.5379 | 444.4 | 0.3098 |
| 2 | `full_kk750_ka500_wn900_z0p7` | 4.1427 | 750 | 500 | 900 | 0.70 | 5.4243 | 4.4699 | 0.3379 | 283.9 | 0.3164 |
| 3 | `full_kk500_ka500_wn900_z0p85` | 4.2679 | 500 | 500 | 900 | 0.85 | 4.5038 | 4.2961 | 0.5432 | 459.1 | 0.3080 |
| 4 | `full_kk750_ka750_wn1100_z0p7` | 4.3610 | 750 | 750 | 1100 | 0.70 | 4.5480 | 4.3453 | 0.5581 | 488.6 | 0.3076 |
| 5 | `full_kk1000_ka500_wn900_z0p7` | 4.5586 | 1000 | 500 | 900 | 0.70 | 6.8010 | 4.6255 | 0.2380 | 203.7 | 0.3242 |

## Nota sulle soglie documentate

Applicando alla lettera i criteri di accettazione scritti in
`SWEEP_INSTRUCTIONS.md`, nessun candidato soddisfa tutte le soglie
contemporaneamente.

In particolare:

- candidati con tracking knee e ankle < 5 deg: 16;
- candidati con worst noise fraction < 0.20: 36;
- candidati con entrambe le condizioni, piu run completo, output finiti,
  `max |tau_input| < 4500 Nm` e `max |u| < 0.95`: 0.

Il punto critico e il trade-off tracking/noise:

- i candidati migliori per tracking hanno `worst_noise_frac` circa 0.54-0.79;
- i candidati sotto 0.20 di noise hanno tracking molto peggiore, ad esempio
  `screen_kk1000_ka500_wn700_z0p7` ha `worst_noise_frac=0.1755`, ma tracking
  RMS di 8.67 deg al knee e 8.90 deg all'ankle.

Questo significa che il flag `acceptable=True` dello script non coincide con i
criteri di accettazione documentati: lo script penalizza noise e tracking nello
score, ma non li usa come soglie dure.

## Raccomandazione

Per una validazione pratica immediata, il candidato consigliato e:

```text
full_kk500_ka500_wn900_z0p7
```

con:

```text
Knee:  K=500, Kp=15.2, Kd=12.5
Ankle: K=500, Kp=15.2, Kd=12.5
```

E' il miglior compromesso trovato dallo sweep completo: tracking sotto 5 deg su
entrambi i giunti, nessuna saturazione del `tau_input`, `max |u|` molto sotto
0.95 e output numericamente finiti.

Non lo considererei pero una soluzione finale se la soglia `noise fraction < 20%`
deve restare vincolante. In quel caso servono un secondo sweep o una revisione
della metrica di noise, perche il migliore candidato full rimane a 0.5379 sul
ginocchio.

## File modificati

Nessun file di codice e stato modificato durante questa analisi.

File creato/aggiornato:

```text
reports/user/2026-04-17_sea_driver_sweep.md
```

File dati analizzati:

```text
results/_sea_driver_sweep_20260417_181301/sweep_results.csv
results/_sea_driver_sweep_20260417_181301/best_candidate.json
results/_sea_driver_sweep_20260417_181301/runs/full_kk500_ka500_wn900_z0p7/sim_output_run_status.txt
```

## Verifiche eseguite

- Letto `best_candidate.json` e confermato `found=true`.
- Importato `sweep_results.csv` e contato run per fase.
- Verificato che i 5 candidati full validation siano completi e accettabili per
  lo script.
- Verificato `sim_output_run_status.txt` del best candidate:
  `status=complete`, `t=11.06`, `dt=0.001`.
- Applicati manualmente i criteri documentati di `SWEEP_INSTRUCTIONS.md`; esito:
  nessun candidato soddisfa tutte le soglie dure contemporaneamente.
