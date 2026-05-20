# AB06_SEASEA - Outer PID selezionabile con anti-windup

Data: 2026-05-13

## Sintesi

E' stato implementato un nuovo controllore high-level protesico `PID` per gli
attuatori SEA di ginocchio e caviglia, mantenendo il controllore storico `PD`
come default.

Il nuovo PID e' selezionabile da configurazione e da CLI. Integra solo l'errore
di posizione, con:

- limite esplicito dell'integrale;
- leakage dell'integrale;
- anti-windup condizionale;
- diagnostica dedicata per contributi `P`, `D`, `I`, stato integrale e
  anti-windup.

Non sono stati modificati il plugin C++ SEA ne' la semantica del comando SEA:

```text
u = tau_ref / F_opt
```

La validazione AB06 full-span sul setup operativo `AB06_SEASEA_Threadmill` ha
completato la simulazione e mostra un miglioramento moderato del tracking
protesico rispetto al PD corrente:

```text
knee q RMS:  0.054718 -> 0.052535 rad  (-3.99%)
ankle q RMS: 0.094858 -> 0.084598 rad (-10.82%)
```

Il validator globale resta formalmente `FAIL` per `mtp_angle_r`, ma questo FAIL
era gia presente nel baseline PD corrente e non riguarda il nuovo outer PID
protesico.

## Problema

Nei plot AB06 si osservava errore cinematico residuo su:

```text
pros_knee_angle
pros_ankle_angle
```

Il controllore high-level precedente era un PD puro:

```text
tau_cmd = Kp * (q_ref - q) + Kd * (qdot_ref - qdot)
u       = clip(tau_cmd / F_opt, -1, 1)
```

Un PD puro non puo' cancellare completamente offset dovuti a disturbi lenti o
quasi statici, come GRF, gravita, mismatch dinamico fra cinematica sana e
modello protesico e banda finita della SEA.

Serviva quindi provare un termine integrale, ma senza perdere il comportamento
validato del PD e senza rischiare windup o saturazione del comando SEA.

## Soluzione implementata

### Configurazione

In `config.py` sono stati aggiunti:

```python
sea_outer_controller_mode = "pd"  # "pd" | "pid"

sea_ki = {
    "pros_knee_angle": 20.0,
    "pros_ankle_angle": 60.0,
}

sea_integral_limit = {
    "pros_knee_angle": 0.25,
    "pros_ankle_angle": 0.25,
}

sea_integral_leak_s_inv = 0.1
```

Il default resta `pd`, quindi le run esistenti non cambiano se non viene
esplicitamente selezionato `pid`.

### CLI

In `main.py` sono stati aggiunti:

```text
--sea-outer-controller {pd,pid}
--sea-ki-knee
--sea-ki-ankle
--sea-integral-limit-knee
--sea-integral-limit-ankle
--sea-integral-leak
```

Il plotter riceve gli stessi parametri quando viene chiamato da `main.py
--plot`, in modo che gli header dei plot riportino correttamente se la run e'
PD o PID.

### Controller

In `prosthesis_controller.py` e' stato aggiunto stato interno:

```text
integral_error[coord]
last_time
```

In modalita `pid` la legge diventa:

```text
tau_cmd = Kp * e_q + Kd * e_qdot + Ki * integral(e_q)
u       = clip(tau_cmd / F_opt, -1, 1)
```

L'integrale:

- usa `dt_control` ricavato da `state.getTime()`;
- usa fallback a `T_control`;
- si resetta se il tempo torna indietro;
- viene attenuato con leakage;
- viene clampato a `sea_integral_limit`;
- non viene aggiornato quando il comando saturerebbe e l'errore spingerebbe
  ancora nella stessa direzione.

### Diagnostica

In `output.py` le colonne SEA diagnostiche esistenti sono state preservate e
sono state aggiunte:

```text
outer_p_cmd
outer_d_cmd
outer_i_cmd
outer_integral_error
outer_integral_clamped
outer_anti_windup_active
outer_controller_mode_id
```

Durante il lavoro e' stata corretta anche la ricostruzione Python di
`tau_input` nella diagnostica: il plugin attivo `ff` include il termine
feedforward `tau_ref`, quindi la diagnostica Python ora rispecchia:

```text
tau_input = tau_ref + Kp_inner * (tau_ref - tau_spring) - Kd_inner * omega_m
```

Questa correzione riguarda il mirror diagnostico e la previsione di feasibility,
non la semantica del comando SEA.

## Strategia

La modifica e' stata mantenuta conservativa:

1. mantenere `pd` come comportamento default;
2. introdurre `pid` come modalita selezionabile;
3. usare valori `Ki` piccoli;
4. limitare esplicitamente l'integrale;
5. aggiungere diagnostica per capire quanto il termine integrale contribuisce;
6. testare AB06 prima con smoke brevi, poi sul full setup operativo.

## File modificati

```text
config.py
main.py
prosthesis_controller.py
output.py
plot/plotter.py
```

File generati di verifica:

```text
results/_outer_pid_smoke_pd/
results/_outer_pid_smoke_pid/
results/_outer_pid_smoke_pid_ffdiag/
results/_outer_pid_ab06_full/
plot/14_05_2026 - 2/
reports/user/2026-05-14_validazione_simulatore.md
```

## Test e verifiche

### Compilazione

Eseguito con l'ambiente OpenSim:

```text
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  config.py prosthesis_controller.py output.py main.py \
  plot/plotter.py validation/validate_sim_results.py
```

Risultato: nessun errore.

### Smoke test

Eseguiti smoke AB06 brevi:

```text
PD:  results/_outer_pid_smoke_pd
PID: results/_outer_pid_smoke_pid
PID diagnostica ff corretta: results/_outer_pid_smoke_pid_ffdiag
```

Gli smoke hanno completato con:

```text
status=complete
output .sto finiti
zero saturazioni tau_input
```

Lo smoke `results/_outer_pid_smoke_pid_ffdiag` ha validazione:

```text
FAIL=0
```

### Full run AB06

Comando usato:

```text
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --sea-outer-controller pid \
  --output-dir results/_outer_pid_ab06_full \
  --validate \
  --plot
```

Range usato dal setup:

```text
t_start = 11.99 s
t_end   = 21.00 s
```

Run status:

```text
status=complete
t=21
step=9010
wall_time_s=744.7091625
```

Plot generati:

```text
plot/14_05_2026 - 2
```

Missing channels:

```text
No missing channels.
```

### Metriche PD vs PID

Baseline PD corrente:

```text
results/
```

PID full:

```text
results/_outer_pid_ab06_full/
```

Risultati principali:

```text
pros_knee_angle:
  q RMS PD        = 0.054718 rad  (3.135085 deg)
  q RMS PID       = 0.052535 rad  (3.010021 deg)
  miglioramento   = -3.99%
  max |u| PID     = 0.307064
  tau_error RMS   = 3.205758 Nm
  saturazioni     = 0
  max |I term|    = 2.128318 Nm
  anti-windup     = 0 campioni

pros_ankle_angle:
  q RMS PD        = 0.094858 rad  (5.434958 deg)
  q RMS PID       = 0.084598 rad  (4.847111 deg)
  miglioramento   = -10.82%
  max |u| PID     = 0.399096
  tau_error RMS   = 1.051801 Nm
  saturazioni     = 0
  max |I term|    = 14.902613 Nm
  anti-windup     = 0 campioni
```

Differenza diretta fra segnali PID e PD:

```text
pros_knee_angle:
  RMS(PID - PD) angolo = 0.465 deg
  max(PID - PD) angolo = 0.734 deg

pros_ankle_angle:
  RMS(PID - PD) angolo = 1.285 deg
  max(PID - PD) angolo = 2.018 deg
```

Quindi il cambiamento visivo nei plot e' moderato: il PID corregge soprattutto
l'offset cinematico, ma non ridisegna drasticamente torque-angle e power.

### Validator

Il validator sul full run PID riporta:

```text
PASS=40
WARN=5
FAIL=1
```

Il FAIL residuo e':

```text
mtp_angle_r output vs IK
RMS = 14.11482 deg
max = 29.79542 deg
```

Questo stesso problema e' presente anche nel baseline PD corrente:

```text
mtp_angle_r RMS PD  = 14.11410 deg
mtp_angle_r max PD  = 29.79734 deg
```

Quindi non e' introdotto dal nuovo PID protesico.

I check SEA rilevanti passano:

```text
SEA_Knee plugin/Python tau_input agreement: PASS
SEA_Ankle plugin/Python tau_input agreement: PASS
SEA_Knee tau_input saturation: PASS
SEA_Ankle tau_input saturation: PASS
SEA_Knee control saturation: PASS
SEA_Ankle control saturation: PASS
```

## Stato finale

Il nuovo outer PID e' implementato, selezionabile e testato sul setup AB06
operativo.

Il comportamento default resta il PD storico. Per usare il PID:

```text
python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --sea-outer-controller pid
```

Il tuning attuale e' stabile e prudente: migliora il tracking cinematico senza
saturare la SEA, ma produce differenze visive piccole nei plot. Per ottenere
effetti piu marcati serve uno sweep dedicato dei `Ki`, mantenendo sotto
controllo `max |u|`, saturazioni, `tau_ref - tau_spring` e rumore su power.

