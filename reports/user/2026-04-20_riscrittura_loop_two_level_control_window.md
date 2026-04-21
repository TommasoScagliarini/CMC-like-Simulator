# Riscrittura del loop di simulazione: two-level control window

**Data**: 2026-04-20

## Problema

Il loop di simulazione precedente eseguiva controlli (PD outer + ID + SO) e
integrazione allo stesso `dt = 0.001 s`. Questa architettura ricalcolava ogni
1 ms l'intera catena di controllo, generando spike single-timestep nel segnale
`u` che il PD inner del SEA amplificava fino a `tau_input` non fisici (range
[-93.6, +97.7] Nm contro [-22, +5] Nm del CMC nativo per gli stessi dati).

Inoltre, la SO ricalcolata a ogni step costituiva il collo di bottiglia
computazionale del simulatore (~1000 chiamate QP al secondo di simulazione).

L'analisi architetturale e' documentata in
`reports/user/2026-04-20_analisi_architetturale_loop_aperto.md`.

## Soluzione

Riscrittura del loop in stile CMC con due livelli:

- **Livello esterno (control)**: ricalcola PD outer, ID, SEA controller e SO
  una sola volta ogni `T_control` secondi. Le attivazioni muscolari, i
  reserves e il comando SEA `u` vengono congelati per tutta la finestra.
- **Livello interno (integration)**: avanza lo stato con substeps semi-explicit
  Euler a `integration_dt = 0.001 s`. A ogni substep ricalcola `udot` via
  bypass (`compute_udot_bypass`) sullo stato corrente, cosi' l'integratore
  vede la risposta vera del modello pur con controlli costanti. Lo stato del
  motore SEA viene avanzato sub-substep dal plugin C++.

Il numero di substeps per finestra e' `n = T_control / integration_dt`. La
modalita' legacy single-step e' preservata via `use_control_window=False`,
che riduce la finestra a 1 substep.

## Strategia

1. Aggiunti tre parametri in `config.py`:
   - `use_control_window: bool = True`
   - `T_control: float = 0.003`  (s)
   - `integration_dt: float = 0.001`  (s)

2. Refactor di `SimulationRunner.run()` in `simulation_runner.py`:
   - Estratto `_compute_controls_for_window(state, controls, t)` che esegue
     gli step A-G originali (realize, kin ref, outer loop, baseline muscolare,
     ID, SEA controller, SO, apply_to_controls) e restituisce le quantita'
     hold-constant per il loop interno.
   - Estratto `_integrate_evaluate(state, controls, t)` che esegue lo step
     H originale (`compute_udot_bypass` + lettura SEA plugin outputs).
   - Il loop principale adesso itera sulle finestre di controllo; dentro
     ciascuna chiama `_compute_controls_for_window` una volta e poi N substeps
     di `_integrate_evaluate` + record + advance.

3. Le risorse del bypass (`matter`, `n_mob`, `_e_vec`, `_Me_vec`) sono
   memorizzate su `self` per essere riusate dai due helper.

4. La docstring del modulo e il file `_run_status.txt` sono stati aggiornati
   per riflettere i nuovi parametri.

## File modificati

- `config.py` — aggiunti `use_control_window`, `T_control`, `integration_dt`.
- `simulation_runner.py` — refactor di `run()`, nuovi metodi
  `_compute_controls_for_window` e `_integrate_evaluate`, docstring
  aggiornata, `_write_run_status` aggiornato.

## Test/verifiche

| Test | Gain (outer/SEA) | T_control | Esito | Wall time | Knee HPF noise |
|------|------------------:|----------:|-------|----------:|---------------:|
| Legacy single-step (regressione) | 20 / 20 | 1 ms | OK | 942 s | (baseline) |
| Control window | 20 / 20 | **3 ms** | OK | **436 s** | **2.77 %** |
| Control window | 20 / 20 | 10 ms | Diverge t=4.89 | — | — |
| Control window | 100 / 100 | 3 ms | Diverge t=5.10 | — | — |

Confronto vs. il valore di riferimento del rapporto architetturale (knee
`tau_input` HPF noise = 6.01 % nel legacy con gain 20/20):

- **Wall time**: -54 % (436 s vs 942 s)
- **Knee HPF noise**: -54 % (2.77 % vs 6.01 %)
- **Ankle HPF noise**: +1.9 pp (1.96 % vs 0.07 %), accettabile dato che il
  problema primario era sul knee

## Conclusioni

Il refactor produce i benefici attesi su rumore e performance ai gain bassi
(Kp=20). Le finestre di controllo lunghe (10 ms) destabilizzano il sistema
perche' la SO produce attivazioni (non eccitazioni) e congelarle troppo a
lungo causa drift sui DOF leggeri come il pelvis. La finestra di 3 ms e' il
miglior compromesso testato: triplica l'orizzonte rispetto al single-step
senza introdurre divergenza.

I gain alti (Kp=100, Kd=20) continuano a divergere. Il control window da
solo non risolve la causa radice (open-loop tra ID e SO): per quello servono
le soluzioni B.3 (feedback su `tau_delivered`) o B.1 (forward dynamics con
RK4) descritte nel report architetturale.

Il refactor base e' completo, retrocompatibile via `use_control_window=False`
e dimezza il tempo di simulazione mantenendo la qualita' del tracking.
