# 2026-04-19 — Rollback del filtro derivativo SEA e scelta dei parametri dallo sweep parziale

## Problema

Lo sweep completo introdotto con il plugin "derivative-filter" (2026-04-18) si è bloccato in modo deterministico a **1049/1056** candidati per via di un subprocess senza timeout su `main.py`. Sui 1049 screening completati:

- nessun candidato soddisfa **contemporaneamente** `tracking < 5°` (entrambi i giunti), `noise_fraction < 0.20` (entrambi), `max|τ| < 4500 N·m` e `max|u| < 0.95`;
- accoppiando candidati con parametri identici e solo `derivative_filter_tau ∈ {0, 1, 2, 5}·10⁻³`, il **`noise_fraction = 1 − R²(τ_input, τ_ref)` non migliora** attivando il filtro — e a τ_d=5 ms peggiora sensibilmente (es. K=250/500, ω_n=300, ζ=1.5 → 0.156 → 0.162 → 0.168 → 0.531).

Conclusione: la LPF su `ω_m` nel termine D introduce phase-lag che R²(τ_input, τ_ref) penalizza più di quanto non guadagni dalla riduzione del chattering HF. La metrica `1−R²` confonde lag con rumore; l'hard-cut su `noise_fraction < 0.20` resta irraggiungibile con la griglia corrente. Nessun guadagno netto dal filtro → si fa rollback e si archivia la strada.

## Soluzione

Rollback completo del plugin alla versione pre-filtro e scelta, dalla frontiera di Pareto dei 1049 candidati, di due operating point:

1. **Best noise** — il candidato accettabile con rumore ginocchio più basso.
2. **Least bad** — il candidato con il miglior compromesso `worst_track + 10·worst_noise` (cioè quello che bilancia tracking e rumore pur senza rispettare entrambi i target).

Entrambe le configurazioni hanno `τ_d = 0` (il filtro non aiuta), dunque sono implementabili sul plugin pre-filtro senza perdita di ottimo rispetto allo sweep.

## Strategia

1. **Plugin C++** → rimuovere tutte le estensioni introdotte per il filtro:
   - property `derivative_filter_tau` e `max_motor_torque`
   - state variable continua `motor_speed_filt` (add / init / derivative)
   - output `motor_speed_filt` e `motor_speed_filt_dot` + accessor
   - nel ramo non-impedance di `getMotorTorque`, tornare a `tau_input = Kp·(τ_ref − τ_spring) − Kd·ω_m`
   - clamp di nuovo hardcoded `MAX_MOTOR_TORQUE = 500.0`
2. **Build Windows** (x64 Release, VS 2022, OpenSim in `C:/OpenSim-mCMC`) in una build dir fresca (`SEA_plugin_core-agent/build_fresh`) per evitare cache CMake puntata alla sibling dir `SEA_plugin_core`.
3. **Deploy** → backup del `.dll` corrente e sovrascrittura di `plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll`.
4. **Estrazione parametri vincenti** dai 1049 risultati screening con uno script d'analisi standalone in stdlib (`validation/_analyze_sweep.py`), ordinato per Pareto e per composite score.

## File modificati

| File | Modifica |
|------|----------|
| [../../SEA_plugin_core-agent/SeriesElasticActuator.h](../../../SEA_plugin_core-agent/SeriesElasticActuator.h) | rimosse 2 property, 2 output, 2 accessor (`getMotorSpeedFilt`, `getMotorSpeedFiltDot`) |
| [../../SEA_plugin_core-agent/SeriesElasticActuator.cpp](../../../SEA_plugin_core-agent/SeriesElasticActuator.cpp) | rimossi `constructProperty_*` del filtro, state variable `motor_speed_filt` (add/init/derivative), `getMotorSpeedFilt(Dot)`, ramo D filtrato; ripristinato clamp hardcoded 500 N·m |
| [../../plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll](../../plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll) | rebuild pre-filtro (196 096 byte, x64 Release); backup `.bak_20260419_095419` del precedente |
| [../../validation/_analyze_sweep.py](../../validation/_analyze_sweep.py) | script nuovo: estrae Pareto e top-15 composite dai 1049 screening |
| [../../validation/_analyze_filter_effect.py](../../validation/_analyze_filter_effect.py) | script nuovo: confronto appaiato `noise_frac` al variare di `τ_d` a parametri fissi |

## Parametri scelti

Entrambi da `results/_sea_driver_sweep_20260418_052206/sweep_results.csv`.

### Best noise — `screen_kk1000_ka500_wn700_z0p7_td0`
`ω_n = 700 rad/s`, `ζ = 0.7`, `τ_d = 0`

| Giunto | K [N·m/rad] | Kp | Kd |
|---|---|---|---|
| Knee  | 1000 | 3.90 |  9.70 |
| Ankle |  500 | 8.80 |  9.70 |

Screening: `trk_k=8.67°`, `trk_a=8.90°`, `nf_k=0.175`, `nf_a=0.004`, `max|τ|=98 N·m`, `max|u|=0.40`.

### Least bad — `screen_kk1000_ka500_wn900_z0p7_td0`
`ω_n = 900 rad/s`, `ζ = 0.7`, `τ_d = 0`

| Giunto | K [N·m/rad] | Kp | Kd |
|---|---|---|---|
| Knee  | 1000 |  7.10 | 12.50 |
| Ankle |  500 | 15.20 | 12.50 |

Screening: `trk_k=6.79°`, `trk_a=4.64°`, `nf_k=0.341`, `nf_a=0.011`, `max|τ|=204 N·m`, `max|u|=0.32`.

## Test / verifiche eseguite

- **Diff sorgenti**: `SEA_plugin_core-agent/*.cpp|*.h` ripuliti da ogni menzione di `derivative_filter_tau`, `motor_speed_filt`, `max_motor_torque` (verificato via Grep).
- **CMake configure** della build dir fresca: successo (`Configuring done (4.2s)`, `Generating done (0.1s)`).
- **MSBuild Release**: successo, output `.dll` a `SEA_plugin_core-agent/build_fresh/Release/SEA_Plugin_BlackBox_mCMC_impedence.dll`.
- **Deploy**: dimensione nuovo `.dll` 196 096 byte (coerente con versioni pre-filtro 17/04).
- **Analisi sweep**:
  - 1049 rows, 673 `complete+finite`, 588 safe (`max|τ|<4500` e `|u|<0.95`);
  - Pareto front: 33 punti;
  - 0 candidati passano tutte le soglie;
  - su 3 gruppi con tutti i 4 valori di `τ_d` rappresentati, il filtro non migliora `nf_k` e lo peggiora a 5 ms.

## Follow-up consigliati

1. **Full validation** dei due candidati (`Best noise` e `Least bad`) su finestra completa `[4.26, 11.06] s` con `main.py` dopo aver aggiornato `models/Adjusted_SEASEA - Copia_tuned.osim` (e/o `models/Adjusted_SEASEA.osim`) con i K/Kp/Kd vincenti e aver rimosso i tag `<derivative_filter_tau>` / `<max_motor_torque>` (il plugin pre-filtro rifiuterà property sconosciute).
2. **Metrica noise onesta**: affiancare a `1-R²` un HPF-energy-ratio (energia > 50 Hz / totale su `τ_input`) calcolato in post-processing dagli `.sto` esistenti — consentirebbe di ri-valutare l'utilità del filtro su dati già prodotti, senza rilanciare simulazioni.
3. **Grid expansion** (se serve abbattere davvero la soglia noise): provare `ω_n ∈ {1400, 1800}` con `ζ = 0.7` e `K_ankle = 250` — l'asse tracking-dominato è sull'aumento di `ω_n`, non sul filtro.
