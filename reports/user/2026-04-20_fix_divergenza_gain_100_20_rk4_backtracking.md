# Fix divergenza gain 100/20 con RK4 bypass e diagnostica SO

**Data**: 2026-04-20

## Problema

La simulazione completa `4.26 -> 11.06 s` con gain SEA/outer `100/20`,
`use_control_window=True` e `T_control=0.003 s` divergeva intorno a `t=5.10 s`.
Il file `results/sim_output_run_status.txt` indicava un `FloatingPointError`
con accelerazioni non finite sui DOF root/pelvis. Nei campioni precedenti al
crash erano visibili esplosioni di `tau_target_norm`, `residual_norm`,
accelerazioni root e comandi SEA saturati.

Il piano iniziale richiedeva due interventi:

- rendere misurabile per coordinata la catena `tau_bio richiesto -> tau muscolo
  / reserve prodotto -> residuo`;
- chiudere il loop con un controllo di fattibilita' della static optimization
  prima di integrare la finestra.

Il piano prevedeva anche un fallback esplicito: se SO backtracking non fosse
bastato, introdurre `integration_scheme = "rk4_bypass"` mantenendo il bypass
`compute_udot_bypass` e senza usare `realizeAcceleration()` o OpenSim Manager.

## Soluzione implementata

Sono stati implementati sia il controllo di fattibilita' SO sia il fallback RK4
bypass. Il fallback e' stato attivato perche' la prova breve con backtracking SO
ma integratore Euler semi-implicito continuava a fallire nella finestra critica.

La configurazione default ora usa:

- `use_control_window = True`
- `T_control = 0.003`
- `integration_dt = 0.001`
- `integration_scheme = "rk4_bypass"`
- `enable_so_feasibility_backtracking = True`
- `save_so_torque_diagnostics = True`

Il plugin C++ non e' stato modificato.

## Strategia tecnica

### Diagnostica SO per coordinata

`StaticOptimizer` ora salva, per ogni coordinata biologica:

- `tau_target_<coord>`
- `tau_muscle_<coord>`
- `tau_reserve_<coord>`
- `tau_delivered_<coord>`
- `tau_residual_<coord>`
- `muscle_row_capacity_<coord>`
- `feasibility_scale`

Questi dati vengono scritti in:

`results/_codex_full_rk4_backtracking_100_20/sim_output_so_torque_diagnostics.sto`

### Backtracking di fattibilita'

In `_compute_controls_for_window()` il runner ora:

1. calcola `qddot_des_bio_raw` con l'outer loop;
2. prova una lista di scale sulla sola correzione PD biologica:
   `qddot_ref + scale * (qddot_des_raw - qddot_ref)`;
3. lascia i DOF protesici su `qddot_ref`;
4. ricalcola inverse dynamics e static optimization per ogni scala;
5. accetta il primo tentativo sotto soglia assoluta o relativa;
6. se nessun tentativo passa, usa il candidato con residuo relativo migliore e
   registra un warning;
7. calcola il comando SEA solo dopo aver scelto il candidato finale.

### RK4 bypass

E' stato aggiunto `integration_scheme = "rk4_bypass"` come integrazione esplicita
RK4 sul vettore:

- coordinate `q`;
- velocita' `qdot`;
- stati motore SEA `motor_angle` e `motor_speed`.

Ogni stage RK4 valuta le accelerazioni con `compute_udot_bypass`, quindi la
pipeline resta fuori da `realizeAcceleration()` e non usa OpenSim Manager. Gli
stati motore SEA sono avanzati leggendo le derivate prodotte dal plugin.

## File modificati

- `config.py`
  - aggiunti `integration_scheme`, flag e soglie per SO backtracking;
  - aggiunto `save_so_torque_diagnostics`.
- `simulation_runner.py`
  - aggiunto backtracking ID+SO nella finestra di controllo;
  - SEA high-level calcolato dopo la scelta SO finale;
  - aggiunto integratore `rk4_bypass`;
  - aggiornato `sim_output_run_status.txt` con `integration_scheme`.
- `static_optimization.py`
  - aggiunta diagnostica per coordinata;
  - aggiunto `feasibility_scale`;
  - aggiunto `remember_solution()` per usare il candidato accettato come warm
    start successivo.
- `output.py`
  - aggiunto buffer per diagnostica SO per coordinata;
  - aggiunto salvataggio `sim_output_so_torque_diagnostics.sto`.

## Verifiche eseguite

### Compilazione

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py simulation_runner.py static_optimization.py output.py inverse_dynamics.py prosthesis_controller.py main.py
```

Esito: **PASS**.

### Diagnosi breve: backtracking SO con Euler

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py --t-start 4.90 --t-end 5.12 --output-dir results/_codex_diag_backtracking_100_20 --log
```

Esito: **FAIL**, ancora non-finite accelerations a `t=5.120`.

Il nuovo `.sto` SO ha mostrato una esplosione del residuo intorno a `t=5.119`,
con residui massimi su coordinate root/hip, ad esempio:

- `tau_residual_pelvis_tilt ~ 2.14e49`
- `tau_residual_hip_adduction_r ~ 2.03e49`
- `tau_residual_hip_flexion_r ~ -7.41e48`

Questo ha attivato il fallback previsto dal piano.

### Diagnosi breve: RK4 bypass + backtracking SO

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py --t-start 4.90 --t-end 5.12 --output-dir results/_codex_diag_rk4_backtracking_100_20 --log
```

Esito: **PASS**, run completata.

Statistiche SO:

- righe: `222`
- intervallo: `4.900 -> 5.121 s`
- `feasibility_scale`: min/mean/max `1.0 / 1.0 / 1.0`
- count `feasibility_scale < 1`: `0`
- max residuo SO: `0.0`

Interpretazione: una volta sostituito Euler con RK4 bypass, la static
optimization non risulta piu' infeasible nella finestra critica.

### Acceptance principale: run completa 4.26 -> 11.06

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py --output-dir results/_codex_full_rk4_backtracking_100_20 --log
```

Esito: **PASS**.

Run status:

- `status=complete`
- `t=11.061`
- `step=6801`
- `wall_time_s=534.0009828`
- `use_control_window=True`
- `T_control=0.003`
- `integration_dt=0.001`
- `integration_scheme=rk4_bypass`
- nessun errore `Non-finite accelerations`

Output:

`results/_codex_full_rk4_backtracking_100_20/`

Tutti gli `.sto` principali della run completa sono finiti.

### Statistiche SO run completa

Dal file:

`results/_codex_full_rk4_backtracking_100_20/sim_output_so_torque_diagnostics.sto`

Statistiche:

- righe: `6801`
- intervallo: `4.260 -> 11.060 s`
- `feasibility_scale`: min/mean/max `1.0 / 1.0 / 1.0`
- count `feasibility_scale < 1`: `0`
- max `abs(tau_residual)`: `1e-08`
- coordinata del max residuo: `ankle_angle_r`
- tempo del max residuo: `t=4.281`
- max `abs(tau_target)`: `3592.65`
- max `muscle_row_capacity`: `252.579`

Interpretazione: il residuo SO e' sotto controllo nella full run stabile. Il
backtracking resta attivo come guardia diagnostica, ma non deve tagliare la
richiesta in questa configurazione.

### Validator

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python validation/validate_sim_results.py --results-dir results/_codex_full_rk4_backtracking_100_20 --prefix sim_output --model "models/Adjusted_SEASEA - Copia_tuned.osim" --reference data/3DGaitModel2392_Kinematics_q.sto --out reports/user/2026-04-20_validazione_simulatore_rk4_backtracking.md
```

Esito complessivo: **FAIL**, ma non per crash o interfaccia SEA.

Riassunto:

- `PASS=19`
- `WARN=6`
- `FAIL=21`
- `run status`: PASS, simulazione completa a `t=11.061`
- derivate plugin SEA finite: PASS
- accordo plugin/Python su `tau_input`: PASS
- tracking vs IK: FAIL su molte coordinate

Tracking RMS principali:

- `pros_knee_angle`: `246.259 deg`
- `pros_ankle_angle`: `388.256 deg`
- `pelvis_tx`: `0.0677 m`
- `pelvis_ty`: `0.0843 m`
- `pelvis_tz`: `0.1098 m`
- `pelvis_tilt`: `19.220 deg`
- `pelvis_list`: `13.211 deg`
- `pelvis_rotation`: `27.988 deg`
- `knee_angle_r`: `7.093 deg`
- `ankle_angle_r`: `48.092 deg`

Report validator:

`reports/user/2026-04-20_validazione_simulatore_rk4_backtracking.md`

### HPF noise metric

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python validation/hpf_noise_metric.py --results-dir results/_codex_full_rk4_backtracking_100_20 --prefix sim_output
```

Esito: **PASS** come esecuzione metrica.

Risultati:

| Metrica | Valore |
|---|---:|
| `ankle_hpf_noise_tau_input` | `0.8983` (`89.8 %`) |
| `ankle_hpf_noise_tau_ref` | `0.6998` (`70.0 %`) |
| `ankle_hpf_noise_tau_spring` | `0.6589` (`65.9 %`) |
| `knee_hpf_noise_tau_input` | `0.8504` (`85.0 %`) |
| `knee_hpf_noise_tau_ref` | `0.4103` (`41.0 %`) |
| `knee_hpf_noise_tau_spring` | `0.2161` (`21.6 %`) |

### Saturazioni SEA

Dal validator e da `sim_output_sea_diagnostics.sto`:

- `SEA_Knee_tau_input_saturated`: `429` campioni, `6.308 %`
- `SEA_Ankle_tau_input_saturated`: `4314` campioni, `63.432 %`
- `SEA_Knee_tau_input_plugin`: max abs `500 Nm`, RMS `185.858 Nm`
- `SEA_Ankle_tau_input_plugin`: max abs `500 Nm`, RMS `435.547 Nm`

## Conclusioni

Il criterio minimo di accettazione del fix e' stato raggiunto: la run completa
`4.26 -> 11.06 s` con gain `100/20`, `use_control_window=True` e
`T_control=0.003 s` termina con `status=complete`, output finiti e nessun errore
di accelerazioni non finite.

La nuova diagnostica mostra che, con RK4 bypass, la static optimization non e'
la sorgente immediata della divergenza: `feasibility_scale` resta sempre `1.0`
e il max residuo SO resta circa nullo. La causa piu' probabile del crash
precedente era quindi l'amplificazione numerica introdotta dall'integrazione
Euler semi-implicita in presenza di dinamica SEA ad alto gain.

Il risultato non e' ancora quantitativamente CMC-like: il validator fallisce sul
tracking e le saturazioni SEA, soprattutto ankle, sono alte. La stabilita'
full-length e' stata ottenuta; la prossima fase deve ridurre tracking error,
rumore HPF e saturazione SEA senza introdurre filtri o rate limiter come
soluzione primaria.

## Prossimi passi consigliati

1. Usare `sim_output_so_torque_diagnostics.sto` come baseline per verificare che
   futuri interventi di tuning non riaprano infeasibilita' SO.
2. Concentrarsi sul loop protesico ankle, dove saturazione e HPF noise sono i
   peggiori:
   - `tau_input_saturated = 63.432 %`
   - `ankle_hpf_noise_tau_input = 89.8 %`
3. Confrontare `tau_ref`, `tau_spring`, `outer_pd_cmd`, `tau_ff_cmd` e
   `tau_input_raw` nella finestra iniziale `4.26 -> 4.35 s`, dove la
   saturazione ankle compare molto presto.
4. Valutare una correzione fisica del riferimento SEA o della ripartizione
   feed-forward/prosthetic torque, non un filtro su `u`.
5. Mantenere `rk4_bypass` come default finche' non esiste un integratore
   alternativo altrettanto stabile.
