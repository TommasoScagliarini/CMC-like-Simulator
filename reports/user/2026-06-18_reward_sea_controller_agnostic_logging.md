# Reward SEA controller-agnostic e logging fisico MLP

Data: 2026-06-18

## Problema

Nel confronto tra rollout MLP a 60 e 100 iterazioni e emerso che molti indici
cinematici e imitativi migliorano, mentre la curva di coppia SEA, soprattutto
all'ankle, puo degradare nella forma. Il punto critico e che la reward non deve
diventare specifica del controller protesico interno: non deve usare stati come
integratori, comandi interni della cascata o dettagli PI/PID, e non deve
premiare direttamente l'imitazione della coppia prescritta protesica.

Serviva quindi introdurre segnali fisici osservabili all'interfaccia SEA, utili
per logging e futuri pesi reward, mantenendo la modularita tra i tre stadi di
controllo.

## Strategia

La modifica e stata implementata come prima versione "logging only":

- i nuovi termini vengono calcolati e propagati in `reward_terms`;
- i componenti sono sempre visibili in `reward_components`;
- TensorBoard puo loggare i nuovi loss e le nuove diagnostics;
- i pesi nei file YAML sono inizializzati a `0.0`, quindi i training esistenti
  non cambiano comportamento finche i termini non vengono abilitati.

La normalizzazione segue la scelta discussa:

- `tau_spring` normalizzata con `abs(F_opt)`;
- `d(tau_spring)/dt` normalizzata con `abs(F_opt) / segment_duration`;
- forma bounded continua `x^2 / (1 + x^2)`, quindi i nuovi loss restano in
  `[0, 1)` senza hard clipping discontinuo.

La potenza SEA esistente non e stata modificata: `sea_motor_power_loss` resta
basata su `tau_input * omega_m`, cioe potenza lato motore, non su
`tau_spring * qdot_joint`.

## Soluzione implementata

In `simulation_runner.py` sono state estese le diagnostics SEA substep e
segment-level:

- `time_s`;
- `tau_spring_nm`;
- `tau_spring_rms_nm`;
- `tau_spring_abs_max_nm`;
- `tau_spring_rate_rms_nm_s`;
- `tau_spring_rate_abs_max_nm_s`.

Il rate e calcolato usando i tempi reali dei substep:

```text
d(tau_spring)/dt = diff(tau_spring_nm) / diff(time_s)
```

Se non ci sono almeno due campioni validi o i `dt` non sono positivi, le metriche
di rate valgono `0.0`.

In `Trajectory Generator/osim_trj_cmc_like.py` sono stati aggiunti due nuovi
loss fisici controller-agnostic:

```text
sea_tau_spring_effort_loss = mean(x^2 / (1 + x^2))
x = tau_spring_rms_nm / abs(F_opt)

sea_tau_spring_rate_loss = mean(x^2 / (1 + x^2))
x = tau_spring_rate_rms_nm_s / (abs(F_opt) / segment_duration)
```

I valori per giunto vengono anche esposti con prefisso, ad esempio:

- `pros_knee_angle_sea_tau_spring_rms_nm`;
- `pros_ankle_angle_sea_tau_spring_rate_rms_nm_s`.

In `Trajectory Generator/baseline_MLP/reward_function.py` sono stati aggiunti:

- `sea_tau_spring_effort_weight`;
- `sea_tau_spring_rate_weight`;
- `policy_action_clip_weight`.

I tre pesi hanno default `0.0`. La penalty viene sottratta solo in proporzione
al peso configurato, ma i componenti restano sempre presenti in
`reward_components`.

Il `RewardShapingWrapper` ora misura anche l'eventuale escursione della raw
action rispetto ai bound dell'action space:

```text
policy_action_clip_loss = mean(x^2 / (1 + x^2))
x = abs(raw_action - clipped_action)
```

Sono inoltre loggati:

- `policy_action_clip_fraction`;
- `policy_action_clip_abs_max`.

## File modificati

- `simulation_runner.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`

## Configurazione reward

Nei due YAML sono stati aggiunti i nuovi pesi:

```yaml
sea_tau_spring_effort_weight: 0.0
sea_tau_spring_rate_weight: 0.0
policy_action_clip_weight: 0.0
```

Questo mantiene invariato il comportamento della reward attiva rispetto ai
training precedenti.

## Verifiche eseguite

Compilazione Python nell'ambiente `envCMC-rllib`:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-rllib python -m py_compile simulation_runner.py "Trajectory Generator/osim_trj_cmc_like.py" "Trajectory Generator/baseline_MLP/reward_function.py" "Trajectory Generator/baseline_MLP/tb_logging.py"
```

Test sintetico `compute_reward`:

- con nuovi pesi a `0.0`, reward invariata;
- con pesi positivi, reward ridotta dalla nuova penalty;
- componenti nuovi presenti in `reward_components`.

Test sintetico action clipping:

- `policy_action_clip_loss` positivo quando una raw action supera i bound;
- `policy_action_clip_fraction` e `policy_action_clip_abs_max` coerenti;
- componente mirrorato in `reward_components`.

Test sintetico diagnostics SEA:

- `tau_spring` costante produce `tau_spring_rate_rms_nm_s = 0.0`;
- rampa lineare di `tau_spring` produce rate RMS positivo;
- i nuovi loss bounded restano nel dominio `[0, 1)`.

Smoke test ambiente:

- `reset + step` completati con OpenSim/plugin disponibili;
- nuove chiavi presenti in `info["reward_terms"]`;
- nuove chiavi presenti in `info["reward_components"]`;
- presenti anche le diagnostics per giunto su `tau_spring_rate`.

Controllo diff:

```powershell
git diff --check -- simulation_runner.py "Trajectory Generator/osim_trj_cmc_like.py" "Trajectory Generator/baseline_MLP/reward_function.py" "Trajectory Generator/baseline_MLP/tb_logging.py" "Trajectory Generator/baseline_MLP/training_cfg.yaml" "Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml"
```

Esito: nessun errore di whitespace; solo warning LF/CRLF attesi su Windows.

## Nota architetturale

La modifica non introduce termini controller-specifici: non legge stati PI/PID,
integratori, comandi della cascata o segnali interni del prosthesis controller.
I nuovi termini guardano solo grandezze fisiche del SEA e il contratto generale
dell'action space della policy. In questo modo resta preservata la separazione
tra policy high-level, interfaccia fisica SEA e implementazione low-level del
controller.
