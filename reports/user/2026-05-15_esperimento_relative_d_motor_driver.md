# Esperimento relative-D nel motor driver SEA - 2026-05-15

## Problema

Dopo lo sweep PID outer e l'analisi del best candidate
(`reports/user/2026-05-15_pid_sweep_best_candidate_plot_naming.md`) e' emerso
che il PID outer migliora molto il tracking cinematico
(`mean_pros_rmse_deg` da `4.27` a `2.14`) ma peggiora la dinamica interna del
SEA: `motor_speed_dot` RMS knee da `92.25` a `271.18 rad/s2`, ankle da `128.03`
a `345.31 rad/s2`, con picchi oltre `5700 rad/s2`. Il `tau_error` non si
riduce; anzi sale leggermente.

Analizzando la struttura del driver non-impedance attivo:

```text
tau_input = tau_ref + Kp_inner * (tau_ref - tau_spring) - Kd_inner * omega_m
```

il termine derivativo `-Kd * omega_m` agisce sulla velocita' assoluta del
motore. Riscrivendo:

```text
omega_m = (tau_spring_dot / K) + omega_j

-Kd * omega_m = -Kd * tau_spring_dot / K - Kd * omega_j
```

quindi il "D" fa due cose insieme: (1) smorza la variazione di coppia
elastica, (2) inietta `-Kd * omega_j`, un termine spurio proporzionale alla
velocita' del giunto, che non e' damping fisico della molla.

Su `AB06_SEASEA_stiff321_500.osim` il termine spurio vale circa:

```text
Knee:  ~2.0 Nm per rad/s
Ankle: ~1.0 Nm per rad/s
```

contro circa due ordini di grandezza meno se si usasse damping relativo
`-Kd * (omega_m - omega_j)`. L'ipotesi era che questo contributo assoluto
contribuisse al chattering visibile sul best PID.

## Soluzione e strategia

E' stato fatto un esperimento chirurgico: sostituire la legge del driver
non-impedance con damping relativo, mantenendo invariato tutto il resto
(stiffness `321/500`, outer PID con il candidato quasi-best meno nervoso,
filtro GRF, T_control = 1 ms, modello AB06).

Nuova legge non-impedance:

```text
tau_input = tau_ref + Kp_inner * (tau_ref - tau_spring) - Kd_inner * (omega_m - omega_j)
```

Strategia:

1. modificare la legge nel plugin C++ e ricompilare il `.dylib`;
2. aggiornare la stessa legge nella diagnostica Python (`output.py`) e nel
   feasibility predictor (`prosthesis_controller.py`) per mantenere l'identita'
   plugin/Python;
3. fare smoke breve per verificare il match `tau_input_plugin` vs
   `tau_input_python`;
4. lanciare una run full con il candidato outer quasi-best e confrontare con
   la run abs-D di riferimento.

Outer PID usato (quasi-best meno nervoso):

```text
knee  Kp=340, Kd=30, Ki=120
ankle Kp=850, Kd=2,  Ki=300
```

## File modificati

Codice C++ plugin (sorgente di lavoro, non installato nel repo principale):

- `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp`
  - riga 139: `-Kd * omega_m` -> `-Kd * (omega_m - omega_joint)` nel ramo
    non-impedance di `getMotorTorque`.

Plugin compilato e installato:

- `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib` (nuovo, relative-D)
- `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.abs_omega_backup_20260515`
  (backup della versione abs-D precedente)

Codice Python:

- `output.py` riga 473: `inner_damp_term = -Kd * (omega_m - omega_j)` nel ramo
  non-impedance della diagnostica SEA, per restare allineata al plugin.
- `prosthesis_controller.py` riga 354: `-Kd_inner * (omega_m - omega_j)` nel
  feasibility predictor non-impedance, stesso motivo.

Il ramo `Impedence = true` non e' stato toccato.

## Run e risultati

Smoke breve: `results/_relative_d_pid_tradeoff_smoke_20260515`.
Esito: OK, match plugin/Python entro circa `1e-6 Nm` su `tau_input`.

Full run:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --output-dir results/_relative_d_pid_tradeoff_full_20260515 \
  --sea-outer-controller pid \
  --sea-kp-knee 340 --sea-kd-knee 30 --sea-ki-knee 120 \
  --sea-kp-ankle 850 --sea-kd-ankle 2 --sea-ki-ankle 300 \
  --filter-grf \
  --log
```

Esito da `sim_output_run_status.txt`:

```text
status         = failed
t              = 12.666 s
step           = 676
wall_time_s    = 142.34
error_type     = FloatingPointError
error          = Non-finite RK4 accelerations at t=12.6670 on pelvis DoFs
sea_forward_mode = plugin
enable_grf_contact_filter = True
```

La simulazione e' divergita molto presto. Andamento della diagnostica
recruitment nel log (`sim_output_phase3_log_20260515_110042.txt`):

```text
t=12.140  |tau_res|=676.83 Nm   |u_res|=6.77
t=12.190  |tau_res|=1464.26 Nm  |u_res|=14.64
t=12.240  |tau_res|=2284.06 Nm  |u_res|=22.84
t=12.290  |tau_res|=2311.64 Nm  |u_res|=23.12
t=12.340  |tau_res|=1867.93 Nm  |u_res|=18.68
t=12.390  |tau_res|=2093.33 Nm  |u_res|=20.93
t=12.440  |tau_res|=1520.10 Nm  |u_res|=15.20
t=12.490  |tau_res|=1228.11 Nm  |u_res|=12.28
t=12.540  |tau_res|=10970.27 Nm |u_res|=109.70
```

Le riserve sono esplose progressivamente fino a `10^4` Nm, segno tipico di
perdita di stabilita' dell'inner loop SEA che richiede a tau_bio una coppia
non rappresentabile, scaricandola sulle reserve.

Confronto sintetico nella finestra comune `11.990 - 12.666 s` contro la run
abs-D di riferimento con lo stesso outer PID quasi-best:

```text
metric                          abs-D (riferimento)     relative-D
|u| max knee                    0.200                   1.000 (saturato)
|u| max ankle                   0.224                   1.000 (saturato)
reserve torque max [Nm]         134.9                   ~10^5
ankle tau_input saturazioni     0                       143 campioni
ankle motor accel max [rad/s2]  6229.8                  76852.4
```

Plot parziali generati:

- `plot/05_15_2026_3/01_time_sea_control_reserve.png`
- `plot/05_15_2026_3/02_time_joint_motor_states.png`
- `plot/05_15_2026_3/03_gaitcycle_torque_angle_power.png`
- `plot/05_15_2026_3/04_gaitcycle_joint_velocity_power.png`
- `plot/05_15_2026_3/05_time_tau_input_tracking_error.png`
- `plot/05_15_2026_3/06_time_joint_ref_sea_error.png`

## Interpretazione

Il drop-in replacement relative-D, mantenendo gli stessi gain inner
(`SEA_Knee Kp=3.9 Kd=9.7`, `SEA_Ankle Kp=8.8 Kd=9.7`) e lo stesso outer PID,
non e' stabile. Il sistema esplode entro circa `0.55 s` dopo l'inizio.

Lettura tecnica:

- la legge abs-D `-Kd * omega_m` contiene, oltre alla parte di damping della
  deflessione, anche un termine `-Kd * omega_j` che agisce come freno rispetto
  al moto della base articolare. Concettualmente e' damping "sbagliato" per un
  servo di coppia, ma di fatto stava fornendo margine di stabilita' al loop
  motore-molla quando il giunto si muove velocemente.
- togliere quel termine in blocco fa perdere quel margine. L'inner P con
  `Kp=8.8` e `Kp=3.9` non ha piu' sufficiente smorzamento "assoluto", la
  deflessione cresce, `tau_input` satura, le accelerazioni del motore salgono
  oltre `7e4 rad/s2`, l'errore protesico si propaga sulle reserve e la
  integrazione esplode.

Quindi l'esperimento non dice "relative-D non funziona": dice che relative-D
puro come drop-in con gli stessi gain non e' equivalente al driver attuale.
La componente assoluta del damping stava facendo da stabilizzatore implicito.

## Verifiche eseguite

- smoke breve relative-D con candidato outer quasi-best: completata,
  `tau_input_plugin` vs `tau_input_python` entro `1e-6 Nm`;
- full run relative-D: fallita a `t=12.666 s` con `FloatingPointError` su DoF
  pelvis (RK4 non-finite), come atteso da una divergenza dell'inner SEA che si
  propaga alle riserve;
- backup del plugin abs-D salvato come
  `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.abs_omega_backup_20260515`
  per ripristino immediato;
- plot parziali generati sulla finestra valida `11.990 - 12.666 s` per
  confronto qualitativo con la run abs-D di riferimento.

Nota: lo stato attuale del repo lascia il plugin attivo nella variante
relative-D. Qualsiasi run lanciata adesso usa la legge relative-D divergente
finche' non si ripristina il backup o non si rimodula il damping.

## Prossimi passi

Direzione consigliata: non lanciare altri sweep PID. Prima:

1. **Ripristinare la variante abs-D** come default sicura, copiando il backup
   sopra il `.dylib` attivo (oppure mantenere lo stato attuale solo per i test
   successivi consapevolmente).
2. **Damping parziale**, parametrizzato:

   ```text
   tau_input = tau_ref + Kp * (tau_ref - tau_spring) - Kd * (omega_m - alpha * omega_j)
   ```

   con `alpha` in `{0.25, 0.5, 0.75, 1.0}`, sullo stesso candidato outer
   quasi-best. Una sola variabile per run; quattro run brevi.
3. **Solo dopo**, se nessun `alpha` da margine sufficiente, considerare il
   retuning di `Kp_inner / Kd_inner` per la legge relative-D pura, oppure uno
   shaping low-pass / rate-limit su `tau_ref` per ridurre la banda del comando
   prima di entrare nell'inner loop.
4. Mantenere come diagnostica shadow nei log il valore che `tau_input` avrebbe
   con `alpha` diversi, senza modificare l'attuazione, per quantificare la
   divergenza tra le varianti prima di cambiare il plugin attivo.
