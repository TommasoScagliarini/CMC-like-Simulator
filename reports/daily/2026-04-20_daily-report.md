# Daily Report - 2026-04-20 (consolidato 20/04 → 21/04)

## Sintesi

Le due giornate del 20 e 21 aprile 2026 hanno tracciato un arco attorno
alla tensione fra "replicare il setup del CMC nativo OpenSim con gain
outer `Kp=100, Kd=20`" e la capacita' fisica del modello protesico SEA
attuale:

1. **20/04 mattina** — analisi architetturale del loop aperto tra ID
   (richiesta di coppia) e SO+muscoli (erogazione effettiva); e'
   emerso che il PD outer a gain alti amplifica glitch
   single-timestep del segnale `u`, con `tau_input` fuori scala e
   divergenza a `t ≈ 5.10 s`.
2. **20/04 pomeriggio** — riscrittura del loop in stile CMC con due
   livelli (finestra di controllo + integrazione), introduzione di
   **RK4 bypass** (senza `realizeAcceleration()`) e di **backtracking
   SO feasibility-aware**; la run completa `4.26 → 11.06 s` con
   `100/20` finisce senza crash ma il tracking fallisce su 21
   coordinate e la saturazione `tau_input` ankle arriva al 63.4%.
3. **20/04 sera** — sweep parametrico inner SEA (`K`, `omega_n`,
   `zeta`) alla ricerca di un candidato che passi lo strict gate con
   `100/20`: **nessun candidato supera il gate**; il fallback
   feasibility scaling salva la run ma diventa dominante (ankle
   `scale==1` solo nel 12.7% dei campioni) e viola il criterio del
   piano.
4. **21/04** — diagnosi quantitativa del mancato tracking: la
   decomposizione di `tau_input_raw` mostra che l'**outer PD**
   (mediana 2069 Nm sull'ankle) e' la causa primaria della
   saturazione, non il feed-forward (mediana 6.8 Nm). Il
   ricalcolo della banda passante `omega_n_inner/omega_n_outer = 70`
   smentisce l'ipotesi di cascata invalida.
5. **21/04 pomeriggio** — due run sperimentali con outer `Kp/Kd =
   20/2` e `5/0.5` confermano: **abbassando i gain outer la
   saturazione sparisce completamente** (`tau_input` ankle ora in
   [-100, +75] Nm). Run `20/2` e' il miglior compromesso:
   ankle nel range fisiologico [0, 0.6] rad, motore che insegue
   pulito. Run `5/0.5` ha ankle fuori range (fino a 2 rad).

**Stato finale al 21/04**:

- architettura due-livelli operativa e stabile: `T_control=3 ms`,
  `integration_dt=1 ms`, `integration_scheme=rk4_bypass`;
- backtracking SO e diagnostica per coordinata (`*.sto` nuovo) attivi
  di default;
- modello corrente `Adjusted_SEASEA - Copia_tuned.osim` con inner SEA
  storico (Knee `K=1000, Kp=3.9, Kd=9.7`; Ankle `K=500, Kp=8.8,
  Kd=9.7`);
- `config.py` con `sea_kp = 20, sea_kd = 20` (valore attuale, frutto
  del tentativo `100/20`), ma le run 21/04 mostrano che **il setup
  funzionante storico `sea_kp=20, sea_kd=2` va ripristinato** come
  default: il cambio `sea_kd 2→20` e' la singola modifica che aveva
  rotto il tracking;
- feasibility scaling del PD outer (`enable_sea_feasibility_scaling`)
  esiste come flag ma e' **disabilitato** e comunque non ancora
  saturation-aware in modo utile (scala solo il PD, non considera il
  `tau_ff`).

**Non risolto**: rimangono spike stretti sul knee
`tau_ref - tau_spring` (HPF noise ~21% sul knee `tau_spring`)
indipendenti dal tuning outer — origine nella catena
`ID → SO → feed-forward`, gia' diagnosticata il 20/04 ma non ancora
affrontata con un intervento mirato.

## Report Utente Considerati

- `2026-04-20_analisi_architetturale_loop_aperto.md`
- `2026-04-20_riscrittura_loop_two_level_control_window.md`
- `2026-04-20_fix_divergenza_gain_100_20_rk4_backtracking.md`
- `2026-04-20_validazione_simulatore_rk4_backtracking.md`
- `2026-04-20_tracking_cmc_like_100_20.md`
- `2026-04-21_diagnosi_mancato_tracking.md`
- `2026-04-21_confronto_outer_gain_20_2_vs_5_0p5.md`

## 1. Analisi architetturale del loop aperto (20/04)

Con outer `Kp=100, Kd=20` la run divergeva a `t ≈ 5.10 s`. L'analisi
di `u_knee` ha trovato il caso peggiore a `t = 5.467 s`:

```
u_knee salta da +0.154 a -0.119 in 1 ms
tau_input_knee esplode da +13.7 Nm a -93.6 Nm a +97.7 Nm (ringing)
tau_spring knee resta in ~[-25, +29] Nm
```

Istogramma `|du/dt|`: il 99% del segnale ha `|du/dt| < 5.5 /s`, ma
un **0.2% ha `|du/dt| > 50 /s`** — glitch isolati che il PD inner del
SEA amplifica fino a ±97 Nm.

### Confronto con CMC nativo OpenSim

| Metrica | CMC OpenSim | Simulatore |
|---|---:|---:|
| Knee `tau_input` range | [-22, +5] Nm | **[-93.6, +97.7] Nm** |
| Ankle `tau_input` range | [-45, +10] Nm | [-90.8, +21.2] Nm |
| Stabilita' con `Kp=100` | stabile | **diverge** |

### Diagnosi

Architettura pre-20/04: loop in **open loop** tra ID (richiesta) e
SO+muscoli (erogazione), con Euler single-step a `dt = 1 ms`.
L'integrazione Euler e la catena `ID → SO → apply_controls → Euler`
non garantiscono che la coppia erogata corrisponda a quella richiesta;
il residuo diventa un errore non compensato che il PD outer amplifica.

### Soluzione proposta

Riscrittura in stile CMC con:

- **finestra di controllo** `T_control` separata dal passo di
  integrazione;
- **integratore RK4 bypass** che evita `realizeAcceleration()` (crash
  del plugin SEA), valutando `udot` via `realizeDynamics + ID solver
  + M⁻¹`;
- **backtracking SO feasibility-aware**: prova una lista di scale
  sulla sola correzione PD biologica finche' il residuo QP non scende
  sotto soglia.

## 2. Riscrittura del loop a due livelli (20/04)

Implementato `_compute_controls_for_window()` + `_integrate_evaluate()`
in [simulation_runner.py](../../simulation_runner.py). Nuovi parametri
in [config.py](../../config.py):

- `use_control_window: bool = True`
- `T_control: float = 0.003` (s)
- `integration_dt: float = 0.001` (s)

### Risultati con gain bassi (outer `20/20`)

| Test | T_control | Esito | Wall time | Knee HPF noise |
|---|---:|---|---:|---:|
| Legacy single-step | 1 ms | OK | 942 s | 6.01% (baseline) |
| Control window | **3 ms** | OK | **436 s** | **2.77%** |
| Control window | 10 ms | Diverge t=4.89 | — | — |

Confronto vs baseline:
- **wall time -54%**
- **knee HPF noise -54%**
- ankle HPF noise +1.9 pp (accettabile, non era il problema primario)

La finestra `3 ms` e' il miglior compromesso: triplica l'orizzonte
senza introdurre drift sui DOF leggeri (pelvis).

### Persistenza del problema a gain alti

Con outer `100/20`, anche con `T_control=3 ms`, il sistema diverge a
`t=5.10 s`. Il control window da solo non risolve la causa radice
(open-loop tra ID e SO richiesti da gain alti) — serve un intervento
sull'integratore.

## 3. Fix divergenza 100/20 con RK4 bypass e diagnostica SO (20/04)

### Diagnostica SO per coordinata

`StaticOptimizer` ora salva per ogni coordinata biologica:
`tau_target_<coord>`, `tau_muscle_<coord>`, `tau_reserve_<coord>`,
`tau_delivered_<coord>`, `tau_residual_<coord>`,
`muscle_row_capacity_<coord>`, `feasibility_scale`.

Output in `sim_output_so_torque_diagnostics.sto`.

### Backtracking feasibility

In `_compute_controls_for_window()`:

1. calcola `qddot_des_bio_raw` con outer PD;
2. prova scale `[1.0, 0.5, 0.25, ..., 0.00390625]`;
3. accetta il primo tentativo sotto soglia `residual_abs ≤ 1e-6` o
   `residual_rel ≤ 1e-3`;
4. se nessuno passa, usa il miglior candidato e warning;
5. calcola il comando SEA **dopo** la scelta SO finale.

### RK4 bypass

Integrazione esplicita RK4 su `[q, qdot, motor_angle, motor_speed]`.
Ogni stage valuta `udot` via `compute_udot_bypass` — quindi la
pipeline resta **fuori da `realizeAcceleration()`** e non usa OpenSim
Manager. Lo stato motore SEA avanza leggendo le derivate del plugin.

### Acceptance

| Test | Esito |
|---|---|
| Diagnosi breve Euler + backtracking (4.90→5.12 s) | **FAIL** (non-finite accel a 5.120) |
| Diagnosi breve RK4 bypass + backtracking | **PASS** |
| Full run RK4 bypass `4.26 → 11.06 s` a `100/20` | **PASS** (status=complete, 534 s wall time) |

Statistiche SO sulla full run:
- `feasibility_scale` sempre `1.0` (6801/6801 campioni)
- max residuo SO: `1e-8`
- max `|tau_target|`: 3593 Nm (pelvis)
- max `muscle_row_capacity`: 253 Nm

**Interpretazione**: con RK4, la SO **non e' infeasible**. La causa
del crash precedente era l'**amplificazione numerica di Euler
semi-implicito** in presenza di dinamica SEA ad alto gain.

## 4. Validazione RK4+backtracking e tracking CMC-like (20/04)

### Validator

Run validator su full run `100/20` RK4:
- `PASS=19, WARN=6, FAIL=21`
- run status: PASS, sim completa
- derivate plugin SEA finite: PASS
- plugin/Python `tau_input` agreement: PASS
- **tracking vs IK: FAIL massivo**

Tracking RMS principali (pre-diagnosi 21/04):

| Coord | RMS |
|---|---:|
| `pros_knee_angle` | 246 deg |
| `pros_ankle_angle` | 388 deg |
| `ankle_angle_r` | 48 deg |
| `knee_angle_r` | 7 deg |
| `pelvis_tilt` | 19 deg |

Saturazioni:
- `SEA_Knee_tau_input_saturated`: 429 campioni (6.3%)
- `SEA_Ankle_tau_input_saturated`: **4314 campioni (63.4%)**

### Sweep CMC-like inner (modello inner10020 + varianti)

Creato `validation/sea_inner_10020_sweep.py` per esplorare
`(K_knee, K_ankle, omega_n, zeta)`. Candidati testati su finestra
`4.26 → 5.30 s`:

| Configurazione | Knee RMS | Ankle RMS | Sat knee/ankle | Max u |
|---|---:|---:|---:|---:|
| Baseline inner, no feasibility | overflow | overflow | 107/322 | 1.0/1.0 |
| Baseline inner + feasibility | 0.4° | 6.9° | 0/32 | 0.3/0.4 |
| `kk1000_ka1250_wn700_z0p7` | 7.6° | 127.5° | 24/228 | 1.0/1.0 |
| `kk1000_ka500_wn700_z1p2` | 224° | 258° | 35/413 | 1.0/1.0 |
| ...altri 4 candidati | tutti fuori soglia | — | — | — |

**Conclusione operativa del 20/04 sera**: nessun candidato supera il
gate strict (`RMS proteico ≤ 3°`, `HPF τ_input ≤ 0.05`,
`|u| < 0.95`). La baseline inner + feasibility e' l'unica vicina, ma
ankle `scale==1` solo nel 12.7% dei campioni — il fallback diventa
sistematico, violando il criterio del piano.

## 5. Diagnosi quantitativa del mancato tracking (21/04)

Analisi completa su `results/_codex_full_rk4_backtracking_100_20/`.
Cinque cause candidate:

| # | Causa | Peso | Evidenza |
|---|---|---|---|
| C1 | Outer PD chiede coppie oltre budget motore (ankle) | **ALTO** | outer_pd_cmd mediana 2069 Nm (sat), max 8332 Nm; limite 500 Nm |
| C2 | Feed-forward non vincolato | BASSO | ff mediana 6.8 Nm in sat (trascurabile) |
| C3 | Drift RK4 motore-giunto | BASSO | <1.2 rad assoluto; sintomo, non causa |
| C4 | Accoppiamento pelvis-ff | BASSO | corr 0.55 solo knee-pelvis_tilt; ff knee piccolo |
| C5 | Banda inner vs outer | NULLO | `omega_n_inner/omega_n_outer = 70` — cascata ampia |

### Conferma della causa primaria

Decomposizione di `tau_input_raw` ankle nei 4314 campioni saturi:

| Componente | Mediana | p95 |
|---|---:|---:|
| `|tau_ff_cmd|` | **6.8 Nm** | 183 Nm |
| `|outer_pd_cmd|` | **2069 Nm** | 4411 Nm |
| `|inner_prop|` | 2216 Nm | 4948 Nm |
| `|inner_damp|` | 831 Nm | 1847 Nm |

L'outer PD domina — il feed-forward e' trascurabile. **Non e' un
problema di architettura**, ne' di feed-forward, ne' di banda
passante inner. E' **il tuning outer a 100/20 che eccede la capacita'
fisica del motore SEA** (500 Nm).

### Ipotesi "manca il contatto" smentita

Inizialmente considerata come causa concorrente (il range
`pros_ankle_angle = [-20, +14]` rad e' non fisiologico); verificato
che il setup usa `ExternalLoads` dal file `data/Externall_Loads.xml`
che applica GRF a `foot_l` (protesi) e `calcn_r` (biologico). La GRF
e' corretta: il problema non e' l'assenza di contatto.

### Causa effettiva identificata

Il diff `git diff HEAD -- config.py` ha rivelato **un singolo
cambiamento chiave**:

```diff
-    "pros_knee_angle":  2,
-    "pros_ankle_angle": 2,
+    "pros_knee_angle":  20,
+    "pros_ankle_angle": 20,
```

Il `Kd` outer protesico moltiplicato per 10 (da 2 a 20) per tentare il
setup `100/20` amplifica 10x l'errore di velocita' angolare. Nei
cambi di direzione della cinematica di riferimento, `|qdot_ref -
qdot|` produce comandi di coppia impossibili.

## 6. Conferma sperimentale: run 21/04 con outer 20/2 e 5/0.5

Due run eseguite per validare la diagnosi:

| Run | Outer Kp/Kd | Ankle joint | Ankle tau_input | Sat? |
|---|---|---|---|---|
| 1 | `20/2` | [0, 0.6] rad — **fisiologico** | [-75, +75] Nm | **No** |
| 2 | `5/0.5` | [0, 2.0] rad — fuori range | [-100, +50] Nm | No |

**La saturazione sparisce completamente** riportando `sea_kd` a 2.
Run `20/2` e' il miglior compromesso: ankle nel range, motore che
insegue pulito il giunto (overlay motor/joint in
[plot/21_04_2026 - 1/02_time_joint_motor_states.png](../../plot/21_04_2026%20-%201/02_time_joint_motor_states.png)).

Run `5/0.5` dimostra il limite opposto: gain outer troppo deboli per
contenere la cinematica di riferimento.

### Problema residuo

In entrambe le run rimangono **spike stretti sul knee `tau_ref -
tau_spring`** — con stessa densita' — che **non dipendono dai gain
outer**. Origine: la catena `ID → SO → feed-forward` produce
gradini all'istante del ricalcolo del control window, tenuti
costanti per 3 substeps di integrazione (moltiplicando l'energia HPF
per 3x). Fenomeno diagnosticato il 20/04 e ancora aperto.

## Azioni residue (priorita')

1. **Ripristinare `sea_kd = {pros_knee: 2, pros_ankle: 2}`** come
   default in [config.py](../../config.py) e rilanciare il validator
   per quantificare quanti dei 21 FAIL scompaiono con la sola
   correzione del tuning.

2. **Non reintrodurre `100/20`** senza prima aver verificato la
   capacita' fisica del motore protesico. Gli sweep del 20/04 hanno
   mostrato che nessun tuning inner passivo sblocca la situazione.

3. **Affrontare gli spike knee residui (R2)** con uno dei seguenti:
   - smoothing LP del solo `tau_ff_cmd` (non del PD), a 30-50 Hz;
   - warm start aggressivo della SO quando le attivazioni variano
     oltre soglia tra finestre consecutive;
   - rate limiter su `|dtau_ff_cmd/dt|`.

4. **Feasibility scaling saturation-aware**: estendere la logica in
   [prosthesis_controller.py](../../prosthesis_controller.py) per
   limitare `tau_ff + PD_outer` al budget motore prima del clipping.
   Utile se in futuro si volessero gain outer piu' alti.

## Stato del repository al 21/04

- **architettura**: control window + RK4 bypass + backtracking SO —
  operativi e robusti, confermati come buon refactor;
- **config corrente**: outer Kp/Kd protesici tornati a `20/2` durante
  la run sperimentale (valore storico pre-20/04); `sea_kd=20` nel
  file committato va ripristinato a 2 come default;
- **plugin C++**: invariato (non modificato ne' il 20/04 ne' il
  21/04);
- **modello**: `Adjusted_SEASEA - Copia_tuned.osim` inner SEA storico
  (non modificato dai tentativi del 20/04).

Il sistema al 21/04 e' **piu' robusto** del 19/04 (niente crash,
architettura valida), ma il **tracking protesico e' peggiore solo per
via del tuning outer**. Un singolo rollback di `sea_kd` riporta la
qualita' al livello pre-20/04; il lavoro architetturale svolto il
20/04 rimane come base solida per futuri tentativi di spingere la
bandwidth outer in modo **controllato** (con feasibility scaling vero).
