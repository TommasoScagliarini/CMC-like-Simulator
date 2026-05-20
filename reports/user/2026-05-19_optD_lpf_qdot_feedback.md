# Opzione D — LPF sul feedback qdot del velocity loop - 2026-05-19

## Problema

I test del 19/05 sul tuning del motor driver (Opzioni B e C) hanno mostrato
effetto marginale sul chattering ankle (riduzione 2-9% del tau HPF50). La
diagnosi via decomposizione HPF50 (vedi
`reports/user/2026-05-19_diagnosi_chattering_motor_vs_cascade.md`) e
l'investigazione delle quattro ipotesi candidate hanno identificato:

```text
Loop di chattering identificato:
  joint_qdot rumoroso -> velocity_err rumoroso -> inner_p_cmd rumoroso
                                                          |
                                                          v
                                                    tau_ref con HF
                                                          |
                                                          v
                                              tau_input motor con HF
                                                          |
                                                          v
                                            sostiene il disturbo del joint
```

Sorgenti reali del rumore HF di `joint_qdot`:

- **Knee**: risonanza meccanica spring-rotor a 28 Hz eccitata durante stance
  (picco netto PSD a 28 Hz, coincide con `omega_mech_knee=sqrt(Ks/Jm)`).
- **Ankle**: shock heel-strike + rumore di base costante (anche in swing).

In entrambi i casi il rumore arriva al cascade come feedback `qdot_cur`
proveniente dallo stato OpenSim, e il velocity loop lo moltiplica
direttamente per `Kp_inner` e lo inietta in `tau_ref`. Il tuning del motor
driver non aiuta perche' agisce a valle del punto di amplificazione.

## Strategia (Opzione D)

Spezzare il loop a monte applicando un **LPF del primo ordine sul feedback
`qdot_cur`** prima del calcolo della velocity_error, **senza** filtrare il
riferimento `qdot_ref` (che e' gia' pulito dal LPF cinematica a 6 Hz).

Cutoff scelto: **25 Hz** per entrambi i SEA.

Motivazione del cutoff:

- Attivita' fisiologica utile (qdot del passo): < 8 Hz. Cutoff 25 Hz lascia
  passare la banda utile con attenuazione < 0.5 dB.
- Risonanza meccanica knee: 28 Hz. Cutoff 25 Hz introduce circa -3 dB sulla
  risonanza (gain 0.667), spezza il loop di sostegno.
- Banda chattering target: > 50 Hz. A 50 Hz l'attenuazione e' -6 dB
  (gain 0.45), a 80 Hz -10 dB (gain 0.30).
- Coerente con la banda del cascade (omega_v ~146 rad/s ~23 Hz ankle):
  il filtro non interferisce con la dinamica utile.

## Soluzione (implementazione)

### Nuova proprieta' in `config.py`

```python
sea_qdot_feedback_lpf_cutoff_hz: Dict[str, float] = field(
    default_factory=lambda: {
        "pros_knee_angle":  0.0,   # disabled by default
        "pros_ankle_angle": 0.0,
    }
)
```

### Filtro LPF in `prosthesis_controller.py`

Nuovo metodo `_apply_qdot_lpf(coord_name, qdot_raw, dt_control)` analogo
al gia' esistente `_apply_u_lpf`. Stato persistente in
`self._qdot_filtered` e `self._qdot_filtered_initialised`, resettato da
`reset()`.

Uso in `compute()`:

```python
qdot_cur_raw = coord.getSpeedValue(state)
qdot_cur, qdot_lpf_alpha = self._apply_qdot_lpf(
    coord_name, qdot_cur_raw, dt_control,
)
# qdot_cur (filtrato) usato in tutte le successive formule:
#   e_qdot = qdot_ref - qdot_cur
#   cascade_velocity_error = cascade_qdot_ref - qdot_cur
```

Implementazione algoritmica:

```python
tau_filter = 1.0 / (2.0 * np.pi * fc)
alpha = dt_control / (dt_control + tau_filter)
qdot_filt = prev + alpha * (qdot_raw - prev)
```

### CLI in `main.py`

Aggiunti `--sea-qdot-lpf-knee` e `--sea-qdot-lpf-ankle` (entrambi
`<=0` disabilita).

### Diagnostica in `output.py`

Aggiunte 3 colonne per ciascun SEA nel file `sim_output_sea_diagnostics.sto`:

```text
{sea_name}_qdot_feedback_raw
{sea_name}_qdot_feedback_filtered
{sea_name}_qdot_feedback_lpf_alpha
```

`SEA_DIAGNOSTIC_WIDTH` aggiornata da 42 a 45.

## Smoke test

### Test algoritmico isolato

Input: sinusoide composta 5 Hz + 0.5x sinusoide 80 Hz, fs=1000 Hz.

```text
RAW  HPF50 RMS = 0.3536  (atteso ~0.35)
LPF  HPF50 RMS = 0.1001  (atteso < 0.35)
Attenuation factor = 0.283   (atteso teorico 0.298 a 80 Hz)

RAW 4-6 Hz RMS = 0.7071
LPF 4-6 Hz RMS = 0.6913  (98% preservato)
```

Coerente con risposta del 1ordine fc=25 Hz.

### Test end-to-end (simulazione 0.236 s push-off)

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup .../AB06_SEASEA_stiff321_500_pi_setup.xml \
  --model .../AB06_SEASEA_stiff321_500_pi.osim \
  --t-start 13.1638 --t-end 13.4 \
  --output-dir results/_optD_smoke_20260519 \
  --filter-grf --sea-outer-controller cascade \
  --sea-cascade-kp-outer-knee 18.85 ... \
  --sea-qdot-lpf-knee 25.0 --sea-qdot-lpf-ankle 25.0
```

Verifica:

```text
qdot_feedback_raw       column present: True
qdot_feedback_filtered  column present: True
qdot_feedback_lpf_alpha column present: True

alpha typical = 0.1358   (atteso 0.1357 per fc=25Hz, dt=1ms)
ankle qdot raw  HPF50 = 2.5336
ankle qdot filt HPF50 = 0.9713   (-62%)
ankle velocity_err HPF50 = 1.3083  (filtrato propagato)
```

LPF attivo end-to-end, colonne diagnostiche corrette.

## Full simulation

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup .../AB06_SEASEA_stiff321_500_pi_setup.xml \
  --model .../AB06_SEASEA_stiff321_500_pi.osim \
  --t-start 11.99 --t-end 21.0 \
  --output-dir results/_optD_full_20260519 \
  --filter-grf --sea-outer-controller cascade \
  --sea-cascade-kp-outer-knee 18.85 --sea-cascade-kp-inner-knee 29.2 --sea-cascade-ki-inner-knee 1377 --sea-cascade-inner-i-torque-limit-knee 50 \
  --sea-cascade-kp-outer-ankle 47.125 --sea-cascade-kp-inner-ankle 2.8275 --sea-cascade-ki-inner-ankle 213 --sea-cascade-inner-i-torque-limit-ankle 200 \
  --sea-qdot-lpf-knee 25.0 --sea-qdot-lpf-ankle 25.0
```

Stato:

```text
status=complete
t_end=21.0
step=9010
wall_time=772.1 s
sat_count knee/ankle = 0/0
```

## Risultati — full window 11.99-21.0 s

Confronto con morning best 18/05 (cascade + motor driver invariati,
unica differenza = LPF qdot abilitato).

| metric | morning | optD | delta% |
|---|---:|---:|---:|
| knee_rmse_deg | 0.1776 | 0.2956 | **+66.4%** |
| ankle_rmse_deg | 1.2203 | 1.2360 | **+1.3%** |
| max_u_knee | 0.3316 | 0.4252 | +28.2% |
| max_u_ankle | 0.4818 | 0.4888 | +1.4% |
| sat_knee / sat_ankle | 0 / 0 | 0 / 0 | invariato |
| knee qdot HPF50 | 0.0156 | 0.0112 | -27.9% |
| ankle qdot HPF50 | 0.1597 | 0.1165 | -27.0% |
| knee velocity_error HPF50 | 0.0158 | 0.0039 | **-75.4%** |
| ankle velocity_error HPF50 | 0.1604 | 0.0588 | **-63.3%** |
| knee inner_p_cmd HPF50 | 0.4619 | 0.1137 | -75.4% |
| ankle inner_p_cmd HPF50 | 0.4535 | 0.1663 | -63.3% |
| **knee tau_input HPF50** | 4.99 | 1.00 | **-79.9%** |
| **ankle tau_input HPF50** | 2.68 | 1.26 | **-53.1%** |
| **knee motor_speed_dot HPF50** | 534.7 | 105.5 | **-80.3%** |
| **ankle motor_speed_dot HPF50** | 317.8 | 46.2 | **-85.5%** |

LPF effectiveness (right cycle 1, push-off window):

```text
ankle qdot_raw HPF50  = 0.1850 rad/s
ankle qdot_filt HPF50 = 0.0640 rad/s    (-65%)
knee  qdot_raw HPF50  = 0.0127 rad/s
knee  qdot_filt HPF50 = 0.0050 rad/s    (-60%)
```

Coerente con risposta teorica del LPF.

## Risultati per ciclo di passo

### right_cycle_1 (13.16-14.78 s)

```text
metric                  morning   optD     delta%
knee_rmse_deg           0.1765    0.3245   +83.9%
ankle_rmse_deg          1.2066    1.2299   +1.9%
knee_tau_input_HPF50    5.55      1.09     -80.4%
ankle_tau_input_HPF50   3.62      0.83     -77.2%
knee_motor_sp_dot HPF50 601       117      -80.5%
ankle_motor_sp_dot HPF50 439      73       -83.3%
```

### right_cycle_2 (17.95-19.53 s)

```text
metric                  morning   optD     delta%
knee_rmse_deg           0.1435    0.2584   +80.1%
ankle_rmse_deg          1.3334    1.3375   +0.3%
knee_tau_input_HPF50    4.71      0.91     -80.7%
ankle_tau_input_HPF50   2.28      0.43     -81.0%
knee_motor_sp_dot HPF50 500       95       -81.1%
ankle_motor_sp_dot HPF50 260      32       -87.9%
```

Comportamento consistente su due cicli del passo destro: chattering
ridotto del 53-88% su tutte le metriche, ankle tracking invariato (+0.3
e +1.9%), knee tracking peggiora di circa 0.1-0.15 deg in assoluto
(+80% in relativo ma RMSE finale ancora basso < 0.33 deg).

## Plot generati

```text
plot/05_19_2026_2/
  01_time_sea_control_reserve.png
  02_time_joint_motor_states.png
  03_gaitcycle_torque_angle_power.png
  04_gaitcycle_joint_velocity_power.png
  05_time_tau_input_tracking_error.png
  06_time_joint_ref_sea_error.png
```

`No missing channels`. Plot standard del simulatore.

## Discussione

**Successo**: il LPF su `qdot_cur` rompe il loop di amplificazione del
rumore del joint che causava il chattering. Effetto sul motor side:

- knee tau HPF50: -80%
- ankle tau HPF50: -53%
- knee motor_speed_dot HPF50: -80%
- ankle motor_speed_dot HPF50: -86%

**Trade-off knee tracking**: knee RMSE passa da 0.18 a 0.30 deg (+66%
relativo, +0.12 deg assoluto). Cause probabili:

- knee morning best era a `zeta=0.59` se la dinamica reale e' vicina a
  J_pair=0.445 (sotto-smorzato). L'aggiunta del LPF introduce fase-lag e
  riduce ulteriormente lo smorzamento effettivo del velocity loop.
- Il knee non aveva chattering critico (HPF50 ankle dominava), quindi il
  beneficio sul knee e' minore in termini di "problema risolto"  rispetto
  al costo sul tracking.

**Ankle tracking invariato**: ankle RMSE +1.3%. Il LPF taglia il rumore
heel-strike che era amplificato dal cascade ma non era utile per il
tracking. Beneficio puro.

**`max u` saturazione**: nessuna saturazione, max u knee passa da 0.33 a
0.43 (margine ampio dal limite 1.0).

## Configurazione raccomandata

Promuovere il LPF qdot a default in `config.py` con cutoff = 25 Hz su
entrambi i SEA, oppure proporre come opzione attivabile via CLI quando
si vuole ridurre chattering al costo di leggero peggioramento knee
tracking.

In alternativa, applicare LPF asimmetrico:

```python
sea_qdot_feedback_lpf_cutoff_hz = {
    "pros_knee_angle":  0.0,    # disable on knee
    "pros_ankle_angle": 25.0,   # enable on ankle (chatter dominante)
}
```

Questa variante eviterebbe il +66% RMSE knee mantenendo il -53/-86% di
chattering ankle (che era il problema dichiarato dall'inizio).

## File modificati

```text
config.py
  + sea_qdot_feedback_lpf_cutoff_hz: Dict[str, float] (default 0.0)

prosthesis_controller.py
  + self._qdot_filtered, self._qdot_filtered_initialised (state)
  + _apply_qdot_lpf(coord_name, qdot_raw, dt_control) method
  + qdot_cur filtrato in compute() prima di e_qdot / cascade_velocity_error
  + esposizione qdot_feedback_raw/filtered/lpf_alpha in u_dict
  + reset() include reset filtro qdot

main.py
  + --sea-qdot-lpf-knee  CLI flag
  + --sea-qdot-lpf-ankle CLI flag
  + override cfg.sea_qdot_feedback_lpf_cutoff_hz se passati

output.py
  + SEA_DIAGNOSTIC_WIDTH: 42 -> 45
  + qdot_feedback_raw/filtered/lpf_alpha in _rec_sea_diagnostics row
  + tre colonne nominali in sea_diagnostic_cols
```

## Test e verifiche eseguiti

```bash
# Syntax
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
    config.py prosthesis_controller.py main.py output.py
# Tutti PASS.

# Config plumbing
/opt/anaconda3/envs/envCMC-like/bin/python -c "
from config import SimulatorConfig
cfg = SimulatorConfig()
print(cfg.sea_qdot_feedback_lpf_cutoff_hz)
cfg.sea_qdot_feedback_lpf_cutoff_hz['pros_ankle_angle'] = 25.0
print(cfg.sea_qdot_feedback_lpf_cutoff_hz)
"
# Output: {'pros_knee_angle': 0.0, 'pros_ankle_angle': 0.0}
#         {'pros_knee_angle': 0.0, 'pros_ankle_angle': 25.0}

# LPF logic
/opt/anaconda3/envs/envCMC-like/bin/python -c "..."  (see Smoke test sopra)

# Smoke run
main.py --t-start 13.1638 --t-end 13.4 --sea-qdot-lpf-knee 25 --sea-qdot-lpf-ankle 25
# status=complete, columns OK, alpha=0.1358 (atteso 0.1357)

# Full run
main.py --t-start 11.99 --t-end 21.0 --sea-qdot-lpf-knee 25 --sea-qdot-lpf-ankle 25
# status=complete, sat=0/0, 9010 step, 772 s wall

# Comparison
/opt/anaconda3/envs/envCMC-like/bin/python /tmp/optD_analysis.py \
  --optD results/_optD_full_20260519
# Saved to results/_optD_full_20260519/comparison_vs_morning_best.txt

# Plot
python plot/plotter.py --results-dir results/_optD_full_20260519 ...
# Output: plot/05_19_2026_2/ (No missing channels)
```

## Configurazione attiva

Al momento la configurazione attiva e' invariata (default `cfg`):

- `sea_qdot_feedback_lpf_cutoff_hz = {knee: 0.0, ankle: 0.0}` (LPF
  disabilitato di default).
- Il filtro e' implementato e validato. Per attivarlo serve override via
  CLI o modifica del default in `config.py`.

Il modello plugin e i gain cascade restano invariati. Tutte le run con
Opzione D hanno usato lo stesso modello `AB06_SEASEA_stiff321_500_pi.osim`
e gli stessi gain cascade del morning best.

## TODO

- **Decidere se promuovere il LPF a default** in `config.py`. Pro:
  chattering molto ridotto. Contro: knee tracking peggiora del 66%
  relativo (0.18 -> 0.30 deg).
- **Testare la variante asimmetrica** (LPF solo su ankle, knee
  invariato) per evitare il costo knee tracking. Atteso: ankle metric
  preservati, knee invariato come morning best.
- **Esplorare cutoff alternativi** (es. 30 Hz, 35 Hz) per trovare il
  miglior trade-off tracking/chattering.
- **Riprodurre il LPF sulla risonanza spring-rotor knee a 28 Hz**:
  notch filter sul feedback knee come alternativa al LPF (preserverebbe
  la banda < 25 Hz utile e taglierebbe solo i 28 Hz).
- **Validare su una run lunga** (es. 30+ s) per verificare la stabilita'
  del LPF in regime continuo e l'assenza di derive.
