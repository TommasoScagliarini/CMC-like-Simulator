# 2026-05-17 - Cascade outer position-P velocity-PI

## Problema

Serviva aggiungere al prosthesis controller un terzo modo high-level, oltre a
PD/PID, con struttura a cascata:

```text
e_q      = q_ref - q
qdot_cas = qdot_ref + Kp_outer * e_q
e_v      = qdot_cas - qdot
xi_v_dot = e_v
tau_cmd  = Kp_inner * e_v + Ki_inner * xi_v
u        = clip(tau_cmd / F_opt, -1, 1)
```

Il plugin SEA resta invariato: il cascade modifica solo il comando Python
high-level inviato al motor driver PI validato nel modello
`models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`.

## Soluzione implementata

- `prosthesis_controller.py`
  - aggiunto mode `cascade`;
  - aggiunta memoria separata `xi_v` per il PI di velocita;
  - reset automatico della memoria su rewind temporale;
  - anti-windup con clamp su `Ki_inner * xi_v` e freeze se il comando satura e
    l'errore di velocita spinge ancora nella saturazione;
  - diagnostica esplicita:
    `cascade_qdot_ref`, `cascade_velocity_error`, `cascade_inner_p_cmd`,
    `cascade_inner_i_cmd`, `cascade_xi_v`, `cascade_i_clamped`,
    `cascade_anti_windup_active`.

- `config.py`
  - aggiunti default conservativi:
    - knee: `Kp_outer=9.4248`, `Kp_inner=15.587`, `Ki_inner=391.744`;
    - ankle: `Kp_outer=9.4248`, `Kp_inner=1.003`, `Ki_inner=25.209`;
    - clamp integrale: knee `50 Nm`, ankle `125 Nm`.

- `main.py`
  - esteso `--sea-outer-controller` a `pd | pid | cascade`;
  - aggiunti override CLI per i gain cascade knee/ankle;
  - i gain vengono passati anche al plotter.

- `output.py`
  - portata la diagnostica SEA da 35 a 42 colonne per SEA;
  - salvati i nuovi canali cascade in `sim_output_sea_diagnostics.sto`.

- `plot/plotter.py`
  - accetta il mode `cascade`;
  - mostra nei titoli i gain cascade usati nella run.

- `tools/cascade_outer_metrics.py`
  - script di confronto PD vs PI vs cascade;
  - produce `metrics.csv`, `summary.md` e plot comparativi.

## Gain testati

| set | f_velocity | f_position | Knee Kp_inner | Knee Ki_inner | Knee Kp_outer | Ankle Kp_inner | Ankle Ki_inner | Ankle Kp_outer |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| conservative | 8 Hz | 1.5 Hz | 15.587 | 391.744 | 9.4248 | 1.003 | 25.209 | 9.4248 |
| balanced | 10 Hz | 2 Hz | 19.484 | 612.100 | 12.5664 | 1.254 | 39.389 | 12.5664 |
| aggressive | 15 Hz | 3 Hz | 29.226 | 1377.22 | 18.8496 | 1.881 | 88.625 | 18.8496 |

La logica di banda e:

```text
Kp_inner = 2*zeta*I_eff*omega_v
Ki_inner = I_eff*omega_v^2
Kp_outer = omega_p
zeta     = 1
```

Il loop posizione resta molto piu lento del loop velocita, e il loop velocita
resta molto piu lento del motor driver PI SEA.

## Verifiche

Static checks:

```text
python -m py_compile prosthesis_controller.py main.py output.py plot/plotter.py config.py tools/cascade_outer_metrics.py
git diff --check -- prosthesis_controller.py main.py output.py plot/plotter.py config.py tools/cascade_outer_metrics.py
```

Entrambi passati.

Smoke run, finestra `13.1638 -> 13.1938 s`:

| run | status | finite u | xi_v finite | mode id |
| --- | --- | --- | --- | --- |
| `_cascade_conservative_smoke_20260517` | complete | yes | yes | 2 |
| `_cascade_balanced_smoke_20260517` | complete | yes | yes | 2 |
| `_cascade_aggressive_smoke_20260517` | complete | yes | yes | 2 |

Full run, finestra `11.99 -> 21.00 s`:

| run | status |
| --- | --- |
| `_cascade_conservative_full_20260517` | complete |
| `_cascade_balanced_full_20260517` | complete |
| `_cascade_aggressive_full_20260517` | complete |

## Metriche full run

Confronto contro:

- PD: `results/_fast_inner_pid_20260516_rerun_20260517`
- PI: `results/_pi_inner_bandmatched_full_20260517`

Sintesi da `plot/05_17_2026_cascade_outer_comparison/summary.md`:

| run | RMSE mean rad | tau_error RMS Nm | HPF >50 Hz RMS | |u|>0.95 | reserve RMS Nm | sat count |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PD | 0.03256 | 1.11672 | 468.57 | 0.00% | 114.08 | 0 |
| PI | 0.03286 | 0.97941 | 486.42 | 0.00% | 114.12 | 0 |
| cascade conservative | 0.34614 | 2.25125 | 147.64 | 0.00% | 114.56 | 0 |
| cascade balanced | 0.18258 | 1.56082 | 203.99 | 0.00% | 114.40 | 0 |
| cascade aggressive | 0.05886 | 0.95398 | 323.43 | 0.00% | 113.94 | 0 |

Dettaglio cascade:

| run | knee RMSE | ankle RMSE | knee max I torque | ankle max I torque | clamp | anti-windup |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| conservative | 0.01454 | 0.67773 | 30.22 Nm | 116.71 Nm | 0.00% | 0.00% |
| balanced | 0.00737 | 0.35778 | 30.47 Nm | 117.64 Nm | 0.00% | 0.00% |
| aggressive | 0.00242 | 0.11530 | 30.85 Nm | 119.17 Nm | 0.00% | 0.00% |

## Interpretazione

Il cascade e implementato e numericamente stabile: nessuna full run ha
saturazioni di `tau_input`, nessuna supera `|u| > 0.95`, nessun clamp integrale
entra in azione, e tutte le uscite sono finite.

La selezione automatica tra i tre cascade sceglie `cascade_aggressive`, perche
e l'unico che si avvicina al tracking PD/PI mantenendo chattering HPF sotto il
PI baseline. Rimane pero un compromesso chiaro: rispetto a PD/PI, il cascade
riduce il contenuto ad alta frequenza del motor driver, ma peggiora il tracking
di posizione medio, soprattutto alla caviglia. Questo e coerente con il
disaccoppiamento frequenziale richiesto: il loop posizione piu lento filtra il
comando, ma paga in ritardo/errore.

## Plot prodotti

- Standard cascade conservative: `plot/05_17_2026_cascade_conservative`
- Standard cascade balanced: `plot/05_17_2026_cascade_balanced`
- Standard cascade aggressive: `plot/05_17_2026_cascade_aggressive`
- Comparativo PD/PI/cascade:
  `plot/05_17_2026_cascade_outer_comparison`

## Raccomandazione

Tenere `conservative` come default config per coerenza con il requisito di
disaccoppiamento frequenziale. Per esperimenti comparativi in cui serve un
tracking piu vicino a PD/PI, usare `cascade_aggressive`: e la migliore tra le
tre tarature testate, resta senza saturazioni, riduce il chattering HPF rispetto
al PI, ma non batte PD/PI sul RMSE medio di posizione.
