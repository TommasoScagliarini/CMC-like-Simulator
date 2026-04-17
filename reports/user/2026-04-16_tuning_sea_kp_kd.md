# Tuning SEA Kp/Kd

Data: 2026-04-16

## Problema

Dopo la validazione del path `sea_forward_mode = "plugin"`, la dinamica SEA risultava finalmente non tautologica, ma i risultati mostravano ancora comportamenti numericamente aggressivi:

- `tau_input` raggiungeva il clamp `+/-500 Nm`;
- `motor_speed` mostrava picchi/chattering;
- la `motor_power` risultava localmente molto elevata per effetto combinato di `tau_input` e `motor_speed`;
- il problema era particolarmente evidente nel quinto grafico e nei diagnostici SEA.

L'obiettivo di questo intervento era fare tuning esclusivamente su `Kp` e `Kd` del plugin SEA, lasciando invariati:

- `Jm`;
- `Bm`;
- legge dinamica del plugin;
- codice C++;
- logica del simulatore Python.

## Diagnosi Di Partenza

La run precedente validata, contenuta in `results/`, mostrava:

| Metrica | SEA_Knee | SEA_Ankle |
|---|---:|---:|
| Saturazioni `tau_input` | 19 campioni | 5 campioni |
| `tau_input_raw max_abs` | 1898.927 Nm | 946.986 Nm |
| `motor_speed max_abs` | 58.484 rad/s | 32.106 rad/s |
| `motor_speed_dot max_abs` | 52829.529 rad/s^2 | 51565.088 rad/s^2 |
| `tau_error rms` | 0.927 Nm | 0.453 Nm |

La diagnostica aveva gia chiarito che le saturazioni non dipendevano da un errore di interfaccia sim/plugin, ma dal loop interno SEA:

```text
tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m
```

Con i vecchi parametri:

| SEA | Kp | Kd | Jm | Bm |
|---|---:|---:|---:|---:|
| SEA_Knee | 64 | 1.28 | 0.01 | 0.1 |
| SEA_Ankle | 100 | 1.6 | 0.01 | 0.1 |

il derivative gain era molto basso rispetto al proportional gain. In pratica il loop interno era molto rigido e poco smorzato: piccoli errori di coppia potevano produrre comandi motore enormi.

## Strategia

La strategia adottata e stata conservativa:

1. non modificare il modello originale `models/Adjusted_SEASEA - Copia.osim`;
2. creare modelli candidati separati per il tuning;
3. testare prima intervalli brevi/medi;
4. includere esplicitamente i tratti temporali dove la run originale saturava:
   - knee intorno a `5.467 s`;
   - knee intorno a `6.026 s`;
   - ankle all'avvio e intorno a `6.460 s`;
5. scegliere un candidato solo se:
   - nessuna saturazione di `tau_input`;
   - validator senza FAIL;
   - `tau_input_raw` restava sotto il clamp con margine;
   - la dinamica SEA restava plugin-driven e non tautologica.

## Candidati Provati

### Candidato 1

File:

```text
models/Adjusted_SEASEA - Copia_tuned.osim
```

Parametri:

| SEA | Kp | Kd |
|---|---:|---:|
| SEA_Knee | 20 | 10 |
| SEA_Ankle | 25 | 16 |

Esito:

- eliminava bene la saturazione ankle;
- riduceva molto i picchi;
- nel tratto critico lungo causava tracking ankle troppo degradato;
- validator: FAIL su `pros_ankle_angle output vs IK`.

Conclusione: troppo cedevole per ankle.

### Candidato 2

File:

```text
models/Adjusted_SEASEA - Copia_tuned_candidate2.osim
```

Parametri:

| SEA | Kp | Kd |
|---|---:|---:|
| SEA_Knee | 32 | 14 |
| SEA_Ankle | 50 | 24 |

Esito:

- tracking migliore rispetto al candidato 1;
- ankle senza saturazione nel tratto testato;
- knee ancora in saturazione nel tratto critico;
- `speed_dot` knee tornava oltre `5e4 rad/s^2`.

Conclusione: troppo aggressivo sul knee.

### Candidato 3

File:

```text
models/Adjusted_SEASEA - Copia_tuned_candidate3.osim
```

Parametri:

| SEA | Kp | Kd |
|---|---:|---:|
| SEA_Knee | 19.5 | 10 |
| SEA_Ankle | 45 | 22 |

Esito:

- validator senza FAIL nel tratto critico;
- nessuna saturazione;
- knee molto vicino al clamp: `tau_input_raw max_abs = 493.344 Nm`;
- ankle ancora piuttosto alto: `tau_input_raw max_abs = 426.144 Nm`.

Conclusione: valido ma con margine insufficiente.

### Candidato 4 Scelto

File:

```text
models/Adjusted_SEASEA - Copia_tuned_candidate4.osim
```

Parametri:

| SEA | Kp | Kd |
|---|---:|---:|
| SEA_Knee | 18.5 | 10 |
| SEA_Ankle | 35 | 20 |

Esito sul tratto critico `4.26 -> 6.55 s`:

- validator senza FAIL;
- nessuna saturazione `tau_input`;
- margine migliore dal clamp;
- `tau_input_raw max_abs`:
  - knee: `465.412 Nm`;
  - ankle: `331.445 Nm`;
- tracking protesico meno perfetto ma accettato dal validator.

Conclusione: candidato selezionato.

## Soluzione Applicata

Il candidato 4 e stato promosso a modello tuned stabile:

```text
models/Adjusted_SEASEA - Copia_tuned.osim
```

Il default in `config.py` e stato aggiornato per usare il modello tuned:

```python
model_file: str = "models/Adjusted_SEASEA - Copia_tuned.osim"
```

Gain finali:

| SEA | Kp precedente | Kd precedente | Kp tuned | Kd tuned | Jm | Bm |
|---|---:|---:|---:|---:|---:|---:|
| SEA_Knee | 64 | 1.28 | 18.5 | 10 | 0.01 | 0.1 |
| SEA_Ankle | 100 | 1.6 | 35 | 20 | 0.01 | 0.1 |

`Jm` e `Bm` sono rimasti invariati:

```text
SEA_Knee:
  motor_inertia = 0.01
  motor_damping = 0.1

SEA_Ankle:
  motor_inertia = 0.01
  motor_damping = 0.1
```

## File Modificati

### Repository simulatore

- `config.py`
  - aggiornato il modello default a `models/Adjusted_SEASEA - Copia_tuned.osim`.

- `models/Adjusted_SEASEA - Copia_tuned.osim`
  - aggiornati solo `Kp` e `Kd` di `SEA_Knee` e `SEA_Ankle`;
  - lasciati invariati `motor_inertia`, `motor_damping`, `stiffness`, `optimal_force`, `Impedence`.

### File Candidati Creati

Sono stati mantenuti per tracciabilita:

- `models/Adjusted_SEASEA - Copia_tuned_candidate2.osim`
- `models/Adjusted_SEASEA - Copia_tuned_candidate3.osim`
- `models/Adjusted_SEASEA - Copia_tuned_candidate4.osim`

### File Non Modificati

Non sono stati modificati:

- plugin C++;
- DLL del plugin;
- `simulation_runner.py`;
- `output.py`;
- legge SEA;
- `Jm`;
- `Bm`;
- `stiffness`;
- `optimal_force`;
- high-level SEA controller Python.

## Verifiche Eseguite

### Compilazione Config

```powershell
python -m py_compile config.py
```

Esito: OK.

### Smoke Default

Comando:

```powershell
conda run -n envCMC-like python main.py --t-start 4.26 --t-end 4.27 --output-dir results\_tune_default_smoke --validate
```

Esito:

- il default carica correttamente `models/Adjusted_SEASEA - Copia_tuned.osim`;
- validator senza FAIL;
- `SEA XML props` conferma:

```text
SEA_Knee:  Kp=18.5, Kd=10.0, Jm=0.01, Bm=0.1
SEA_Ankle: Kp=35.0, Kd=20.0, Jm=0.01, Bm=0.1
```

### Run Completa Tuned

Comando:

```powershell
conda run -n envCMC-like python main.py --model "models\Adjusted_SEASEA - Copia_tuned_candidate4.osim" --output-dir results\_tune_k18p5d10_a35d20_full --validate --plot --log
```

Esito:

```text
Summary: PASS=44, WARN=2, FAIL=0
```

Output principali:

- `results/_tune_k18p5d10_a35d20_full`
- `results/_tune_k18p5d10_a35d20_full/sim_output_phase3_log_20260416_192238.txt`
- `plot/16_04_2026 - 5`
- `reports/user/2026-04-16_tuning_sea_kp_kd_validation.md`

## Confronto Prima/Dopo

| Metrica | Prima | Tuned |
|---|---:|---:|
| Knee saturazioni `tau_input` | 19 | 0 |
| Ankle saturazioni `tau_input` | 5 | 0 |
| Knee `tau_input_raw max_abs` | 1898.927 Nm | 465.412 Nm |
| Ankle `tau_input_raw max_abs` | 946.986 Nm | 331.445 Nm |
| Knee `motor_speed max_abs` | 58.484 rad/s | 34.336 rad/s |
| Ankle `motor_speed max_abs` | 32.106 rad/s | 9.941 rad/s |
| Knee `motor_speed_dot max_abs` | 52829.529 rad/s^2 | 47835.296 rad/s^2 |
| Ankle `motor_speed_dot max_abs` | 51565.088 rad/s^2 | 33144.518 rad/s^2 |
| Knee `tau_error rms` | 0.927 Nm | 1.144 Nm |
| Ankle `tau_error rms` | 0.453 Nm | 0.774 Nm |

Il tuning ha quindi eliminato le saturazioni e ridotto il chattering soprattutto sull'ankle. Il knee resta dinamicamente aggressivo, ma non arriva piu al clamp.

## Tradeoff Osservato

Il tracking protesico peggiora rispetto al modello precedente:

| Coordinata | RMS tuned |
|---|---:|
| `pros_knee_angle` | circa `7.96 deg` |
| `pros_ankle_angle` | circa `8.09 deg` |

Questo peggioramento e atteso: il vecchio sistema era troppo rigido e vicino al comportamento ideale; il tuning riduce la capacita del loop interno di inseguire istantaneamente la coppia desiderata, rendendo il SEA piu realistico e meno numericamente impulsivo.

Il punto importante e che il peggioramento non deriva da una scorciatoia o da un errore d'interfaccia. Il validator conferma:

- `tau_ref - tau_spring` non e nullo;
- `motor_angle` non e vincolato algebricamente a `q + tau_ref/K`;
- `motor_speed` non coincide con `qdot`;
- `tau_input` del plugin coincide con la ricostruzione indipendente Python della legge SEA;
- non ci sono FAIL.

## Stato Finale

Il simulatore ora usa di default:

```text
models/Adjusted_SEASEA - Copia_tuned.osim
```

con:

```text
SEA_Knee:  Kp=18.5, Kd=10
SEA_Ankle: Kp=35,   Kd=20
```

Le saturazioni di `tau_input` sono eliminate nella run completa validata. La dinamica SEA resta invariata e plugin-driven.

## Prossimi Passi Consigliati

1. Analizzare i nuovi plot in `plot/16_04_2026 - 5`, soprattutto:
   - motor speed;
   - motor power;
   - tau input;
   - errori `tau_spring - tau_ref`.

2. Decidere se il tracking protesico tuned e accettabile per questa fase.

3. Se serve recuperare tracking senza tornare alla saturazione:
   - intervenire successivamente sui gain high-level `sea_kp/sea_kd`;
   - valutare un filtro/rate limit su `tau_ref`;
   - analizzare feed-forward e transitori di `tau_pros_ff`;
   - solo dopo, eventualmente, ragionare su `Jm/Bm`, che in questa fase sono rimasti volutamente invariati.
