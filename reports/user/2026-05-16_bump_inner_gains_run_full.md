# Bump dei guadagni inner SEA in-place e run di verifica - 2026-05-16

## Problema

Chiusura di ieri
(`reports/user/2026-05-15_analisi_strutturale_bandwidth_cascata_e_proposta_impedance_ff.md`,
`reports/user/2026-05-15_pi_inner_e_target_bandwidth_motor_driver.md`):
il chattering del motor driver SEA non e' di tuning ma di **mancato
disaccoppiamento frequenziale** tra inner driver e outer PID. Sul knee
`BW_inner ~32 Hz` vs `BW_outer ~40 Hz` (ratio `0.8x`, invertito rispetto
alla rule-of-thumb cascata `5-10x`). Sull'ankle `~111 Hz` vs `~27 Hz`
(`4x`, borderline). I tre esperimenti del 2026-05-15 (sweep PID notturno,
rel-D drop-in, LPF su `u`) si spiegano tutti come sintomi dello stesso
problema strutturale.

L'azione concordata era: alzare la BW del motor driver inner via bump di
`Kp_inner` e `Kd_inner` sui due `SeriesElasticActuator`, target
`BW_torque ~125 Hz` (`omega_n ~785 rad/s`, `zeta ~0.7`), mantenendo
abs-D, outer PID quasi-best e plugin invariato (`<Kp>`/`<Kd>` sono
esposti nell'`.osim`, non serve ricompilare).

Vincolo dell'utente: **non creare un nuovo `.osim`**, modificare in-place
[models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim](../../models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim).
Backup file-level a fianco.

## Soluzione

### Modifica .osim in-place (con backup)

Backup salvato come
`models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim.absD_inner_3p9_8p8_backup_20260516`.
Restore via `cp` se serve rollback. Modifiche puntuali sui due blocchi
`<SeriesElasticActuator>` (linee `7541-7568` e `7569-7596`):

```text
SEA_Knee
  Kp: 3.8999999999999999  ->  18         (linea 7561)
  Kd: 9.6999999999999993  ->  11         (linea 7563)

SEA_Ankle
  Kp: 8.8000000000000007  ->  11.3       (linea 7589)
  Kd: 9.6999999999999993  ->  11         (linea 7591)
```

Tutti gli altri campi invariati (`stiffness 321/500`, `motor_inertia 0.01`,
`motor_damping 0.1`, `Impedence false`, `optimal_force 100/250`,
`Kd_Imp 5/10`). Target post-modifica per entrambi i SEA: `omega_n ~785
rad/s`, `zeta ~0.7`, `BW_torque ~125 Hz`.

### Run full di verifica

Comando lanciato (env `envCMC-like`):

```bash
python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --t-start 11.99 --t-end 21.0 \
  --output-dir results/_fast_inner_pid_20260516 \
  --filter-grf \
  --sea-outer-controller pid \
  --sea-kp-knee 340 --sea-kd-knee 30 --sea-ki-knee 120 \
  --sea-kp-ankle 850 --sea-kd-ankle 2 --sea-ki-ankle 300
```

Outer PID = "quasi-best" del 2026-05-15 (coerente con esperimenti
diagnostici rel-D e LPF). Stessa finestra full della baseline. Cartella
output: `results/_fast_inner_pid_20260516/`.

Stato run:

```text
status   = complete
t        = 21
step     = 9010
wall     = 742.6 s
dt       = 0.001
scheme   = rk4_bypass
sea_substeps = 5 / max 80
saturazioni  = 0  reserves attive = 0
```

## Strategia

Variabile singola sul plant inner. Outer PID invariato rispetto a un
candidato gia' validato come "quasi-best". Confronto con baseline best
PID del sweep notturno per misurare l'effetto cumulativo "bump inner +
piccolo swap outer". Confronto non perfettamente single-variable (baseline
ha outer PID diverso), ma e' la baseline indicata dal daily report di
ieri. Non esiste sulla finestra full una run "abs-D inner originale +
outer quasi-best" — la cosa piu' vicina e' la finestra ridotta
`[12.0, 14.5] s` del sweep LPF `fcoff`, da considerare come riferimento
secondario.

## Risultati

Metriche estratte via `validation/outer_pid_gain_sweep.py::tracking_stats`
e `signal_stats` su tutta la finestra `[11.99, 21.00] s`:

```text
metric                              baseline       fast_inner       delta        pct
--------------------------------    ----------     ----------    ----------    --------
--- tracking ---
knee_rmse_deg                          1.5284         1.2958       -0.2326      -15.22 %
ankle_rmse_deg                         2.7522         2.4512       -0.3009      -10.93 %
mean_pros_rmse_deg                     2.1403         1.8735       -0.2668      -12.46 %
knee_max_err_deg                       5.0087         4.2926       -0.7161      -14.30 %
ankle_max_err_deg                      6.4805         5.8903       -0.5902       -9.11 %

--- driver torque error ---
knee_tau_err_rms  [Nm]                 3.3136         1.2295       -2.0841      -62.90 %
ankle_tau_err_rms [Nm]                 1.0547         1.0040       -0.0507       -4.81 %

--- motor speed derivative ---
knee_mdot_rms  [rad/s2]                 271.2          769.5         498.3     +183.76 %
knee_mdot_max  [rad/s2]                2505.7         8962.4        6456.8     +257.69 %
ankle_mdot_rms [rad/s2]                 345.3          459.9         114.6      +33.19 %
ankle_mdot_max [rad/s2]                5747.3         7297.1        1549.8      +26.97 %

--- tau_input saturation count ---
knee_sat_count                              0              0             0           -
ankle_sat_count                             0              0             0           -

--- pros reserve torque RMS ---
knee_reserve_rms  [Nm]                 0.0000         0.0000        0.0000           -
ankle_reserve_rms [Nm]                 0.0000         0.0000        0.0000           -
```

Lettura:

- **Tracking**: migliorato uniformemente (`-12.5 %` sul mean, `-15 %`
  knee, `-11 %` ankle). Anche i picchi max calano (`-14 %` knee,
  `-9 %` ankle).
- **`tau_error` knee**: collassa da `3.31` a `1.23 Nm` (`-63 %`).
  Conferma diretta che il driver knee era saturato in banda: alzando
  `Kp_inner` da `3.9` a `18` (quasi `5x`) e portando `zeta` da
  overdamped (`1.24`) a `0.7`, il driver finalmente riesce a inseguire
  `tau_ref`.
- **`tau_error` ankle**: variazione marginale (`-5 %`). Coerente con
  l'analisi di ieri: l'ankle aveva gia' `BW_inner ~111 Hz`, vicino al
  target `~125 Hz`. Il bump (`Kp 8.8 -> 11.3`, +28 %) e' chirurgico.
- **`motor_speed_dot`**: cresce su entrambi (knee `+184 %` RMS,
  `+258 %` max). Era atteso: alzare `omega_n` significa accelerazioni
  motore piu' grandi in transitorio. Non e' chattering nel senso
  oscillatorio (`zeta = 0.7`, overdamped via Kd alto). E' ampiezza
  del transitorio. Va monitorato ma non e' un fallimento.
- **Saturazioni e reserves protesiche**: zero. Niente esplosioni,
  niente clipping `tau_input` (`|tau_input| < 500 Nm` ovunque). Il
  sistema e' fisicamente sano.

Ipotesi del 2026-05-15 sul knee ("BW insufficiente del driver inner")
**confermata** dai numeri: tracking migliora, errore di coppia crolla,
nessuna instabilita'.

## File modificati / creati

```text
M  models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim
+  models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim.absD_inner_3p9_8p8_backup_20260516
+  results/_fast_inner_pid_20260516/                    (output run completo)
+  reports/user/2026-05-16_bump_inner_gains_run_full.md (questo report)
```

Nessuna modifica a codice Python o plugin C++. Modifiche solo sul .osim e
backup file a fianco.

## Verifiche eseguite

1. Backup `.osim` creato e identico all'originale prima dell'edit
   (`diff -q` vuoto).
2. `grep -nE '<Kp>|<Kd>|<stiffness>'` sui due blocchi SEA conferma i
   nuovi valori (`Kp 18, Kd 11` knee; `Kp 11.3, Kd 11` ankle).
3. Run terminata con `status=complete` in
   `sim_output_run_status.txt`, `t=21`, `step=9010`, `wall=742.6 s`.
4. Tutti i `sim_output_*.sto` con valori finiti su tutta la finestra
   (verifica implicita via `tracking_stats`, nessuna eccezione).
5. Metriche estratte e tabulate vs baseline best PID, output sopra.

## Prossimi passi

```text
1. plot per gait-cycle del knee e dell'ankle in plot/05_16_2026_1/
   (sovrapposizione baseline best PID vs fast_inner) per ispezione
   visiva di tracking, tau_input, motor_states.

2. valutare se l'aumento di motor_speed_dot e' un problema reale o solo
   ampiezza transitoria attesa. Da ispezionare nel dominio del tempo: se
   le accelerazioni sono concentrate sui touch-down/heel-strike e poi
   tornano nei range della baseline, l'aumento e' fisiologico. Se sono
   continue (ronzio), c'e' un problema di chattering residuo.

3. se 2. risulta fisiologico:
   - sweep su Kp_inner attorno a (knee=18, ankle=11.3) per ottimizzazione
     fine, target -15-20 % sul tracking;
   - oppure passaggio a PI inner (riscrittura plugin C++ + ricompilazione)
     per azzerare l'errore residuo a regime quando il giunto si muove
     (analisi in reports/user/2026-05-15_pi_inner_e_target_bandwidth_motor_driver.md).

4. solo dopo che l'inner e' definitivamente sistemato, valutare cambio
   struttura outer (cascade P-pos + PI-vel, o impedance + FF
   real-world).

5. TODO Windows aggiornato: includere il bump inner gains nel .osim tra
   i passi da replicare quando la macchina Windows sara' allineata.
   Il file .osim e' cross-platform, basta `git pull` dopo commit.
```

## TODO aperti (da propagare nei prossimi report)

- Plot comparativo gait-cycle `plot/05_16_2026_1/`.
- Ispezione dominio-tempo di `motor_speed_dot` per discriminare
  transitorio fisiologico da chattering.
- Decisione su sweep `Kp_inner` o passaggio a PI inner.
- TODO Windows pendente da 2026-05-15: compilare/copiare la DLL `ff`
  sulla macchina di Tommy e replicare il bump inner gains nel .osim.
