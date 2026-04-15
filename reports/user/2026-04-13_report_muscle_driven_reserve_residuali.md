# Report - Recruitment muscle-driven e reserve residuali

## Data e ora

- Data: 2026-04-13
- Ora: 23:12:33 CEST
- Workspace: `/Users/tommy/Documents/CMC-like-Simulator - Claude`

## Problemi

1. Il tracking biologico era stabile ma non realmente muscle-driven.
   - La static optimization precedente poteva ricadere su una formulazione troppo povera del blocco muscolare.
   - Le reserve actuators tendevano a diventare il canale principale per produrre `tau_bio`.

2. La formulazione muscolare basata su `moment_arm * Fmax` non era coerente con il comportamento reale dei muscoli Thelen/OpenSim.
   - Nei muscoli Thelen la forza non dipende solo dall'attivazione istantanea.
   - La forza tendinea dipende anche dallo stato di fibra e dall'equilibrio muscolo-tendine.

3. L'applicazione diretta dell'equilibrio muscolare nello stato forward poteva destabilizzare la simulazione.
   - Reimpostare le fibre in equilibrio ad ogni frame spingeva alcuni muscoli verso limiti di lunghezza.
   - Questo produceva warning Thelen e poteva portare a input non finiti per la SO.

4. Il collasso osservato intorno a `t ~= 4.60 s` non era causato solo dalla SO muscolare.
   - Anche un test reserve-only con vecchi bound divergeva nello stesso intervallo.
   - La causa pratica era che il controller SEA riceveva `tau_ff` dal runner, ma la riga effettiva in `prosthesis_controller.py` lo ignorava.

5. La diagnostica globale poteva essere fuorviante.
   - Le coordinate `pelvis_*` sono DOF root/residuali nel modello e non sono coperte dai muscoli come i joint biologici.
   - Quindi una quota reserve globale alta non implica necessariamente reserve dominance sui DOF muscolari.

## Soluzione

1. Ho abilitato una static optimization muscle-first.
   - `use_muscles_in_so = True`
   - Il blocco muscolare ora usa una mappa `A_muscle` stimata frame-by-frame da forza Thelen-equilibrium.
   - La coppia muscolare viene calcolata come contributo rispetto a `muscle_min_activation`.

2. Ho reso le reserve residuali.
   - `reserve_weight = 1.0e6`
   - `reserve_u_max = 50.0` per i joint/reserve biologici.
   - `unactuated_reserve_u_max = 1000.0` solo per coordinate residuali `pelvis_*`.
   - Le coordinate `pelvis_*` sono escluse dal denominatore della diagnostica `muscle_capable_share`.

3. Ho applicato la forza muscolare ottimizzata sui muscoli OpenSim via `overrideActuation`.
   - La SO stima la forza baseline a `a_min` e il gain di forza tra `a_min` e `a_max`.
   - Nel forward loop la forza risultante viene applicata al muscolo OpenSim, evitando reset instabili delle fibre ad ogni frame.

4. Ho ripristinato l'uso di `tau_ff` nel controller SEA.
   - La firma e la docstring gia prevedevano `tau_ff`.
   - La semantica resta: `tau_cmd = tau_ff + Kp*e + Kd*edot`, poi `u = clip(tau_cmd / F_opt, -1, 1)`.
   - Non e stata cambiata l'interfaccia high-level -> plugin e il plugin continua a ricevere un comando normalizzato.
   - Il root cause del bug (PD scalato in spazio u) e' documentato in [[DIVERGENZA_BUGFIXES]].

5. Ho aggiunto diagnostica di recruitment.
   - Nuovo file output: `sim_output_recruitment.sto`.
   - Metriche registrate:
     - `tau_target_norm`
     - `tau_muscle_norm`
     - `tau_reserve_norm`
     - `muscle_share`
     - `muscle_capable_share`
     - `muscle_capable_reserve_norm`
     - `unactuated_reserve_norm`
     - `reserve_control_norm`
     - `activation_mean`
     - `activation_nonzero_fraction`
     - `residual_norm`
     - `equilibrium_failures`

## Strategia di risoluzione

1. Ho confrontato la nuova SO muscolare con il comportamento reserve-only.
   - Questo ha mostrato che la divergenza a `t ~= 4.60 s` non era imputabile solo ai muscoli.
   - Il sistema divergeva anche senza usare i muscoli in SO, indicando un problema nel lato SEA/forward loop.

2. Ho isolato il ruolo del feed-forward protesico.
   - Il runner calcolava `tau_pros_ff` con inverse dynamics e lo passava al controller SEA.
   - Il controller pero ignorava quel termine nella riga effettiva.
   - Reinserire `tau_ff` ha stabilizzato il tratto che prima divergeva.

3. Ho separato i problemi biologici dai problemi protesici.
   - Lato biologico: recruitment muscolare coerente con Thelen-equilibrium.
   - Lato protesico: preservata la struttura high-level Python / low-level plugin C++ (→ [[AGENT]]).
   - Lato numerico: preservato il bypass senza `realizeAcceleration()`.

4. Ho validato iterativamente.
   - Run breve/intermedio fino a `t=4.80 s`.
   - Run breve con diagnostica aggiornata fino a `t=4.85 s`.
   - Run completo da `t=4.26 s` a `t=11.06 s`.

## Modifiche fatte

### `config.py`

- Aggiunta configurazione muscle-first.
- Aggiunti parametri:
  - `use_muscles_in_so`
  - `muscle_mapping_strategy`
  - `muscle_force_application`
  - `muscle_min_activation`
  - `muscle_max_activation`
  - `muscle_activation_weight`
  - `muscle_active_threshold`
  - `muscle_row_capacity_threshold`
  - `reserve_weight`
  - `reserve_u_max`
  - `unactuated_reserve_u_max`
  - `unactuated_reserve_coord_prefixes`
  - `save_recruitment_diagnostics`
  - `recruitment_diagnostics_interval`

### `model_loader.py`

- Aggiunta cache degli indici degli state variables muscolari:
  - `muscle_activation_sv_idx`
  - `muscle_fiber_length_sv_idx`
- Aggiunti sanity check per activation/fiber_length mancanti.

### `inverse_dynamics.py`

- Nel baseline zero-actuator, le attivazioni muscolari vengono portate a `muscle_min_activation`.
- Le attivazioni originali vengono salvate e ripristinate dopo il calcolo della baseline.
- Sono preservati:
  - baseline inverse dynamics zero-actuator
  - azzeramento deflessione SEA con `motor_angle = theta_joint`
  - bypass senza `realizeAcceleration()`

### `static_optimization.py`

- Sostituito il vecchio mapping muscolare con `A_muscle` basato su equilibrio Thelen.
- Aggiunta preparazione baseline muscolare a `a_min`.
- Aggiunta applicazione della forza muscolare via `overrideActuation`.
- Aggiunti bounds differenziati per reserve joint e reserve residuali.
- Aggiunta diagnostica completa del recruitment.
- Aggiornati fallback numerici per mantenere robustezza.

### `simulation_runner.py`

- Preparazione del baseline muscolare prima dell'inverse dynamics.
- Applicazione delle attivazioni/forze muscolari allo stato prima di calcolare `udot`.
- Aggiunto buffer diagnostico.
- Aggiunto salvataggio di `sim_output_recruitment.sto`.
- Aggiunta stampa periodica di diagnostica recruitment.

### `prosthesis_controller.py`

- Ripristinato l'uso di `tau_ff` nella legge gia documentata:
  - prima: `tau_cmd = Kp*e + Kd*edot`
  - dopo: `tau_cmd = tau_ff + Kp*e + Kd*edot`
- Non e stata modificata l'interfaccia del controller.
- Non e stata modificata la semantica del comando al plugin:
  - il plugin riceve ancora `u = tau_cmd / F_opt`, saturato in `[-1, 1]`.

## Verifica

Run completo eseguito:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py --output-dir /tmp/cmc_muscle_full2
```

Risultato:

- 680 step completati.
- Intervallo simulato: `4.26 s -> 11.06 s`.
- Nessuna eccezione per accelerazioni non finite.
- File controllati tutti finiti:
  - `sim_output_recruitment.sto`
  - `sim_output_activations.sto`
  - `sim_output_kinematics.sto`
  - `sim_output_sea_controls.sto`
  - `sim_output_tau_bio.sto`

Statistiche principali:

- `muscle_capable_share`: media `0.958`, mediana `0.99999`.
- `activation_nonzero_fraction`: media `0.446`.
- `activation_mean`: media `0.07497`.
- `residual_norm`: max `4e-08`.
- `tau_muscle_norm`: media `72.33`.
- `muscle_capable_reserve_norm`: media `8.34`.
- `unactuated_reserve_norm`: media `83.10`, dovuta soprattutto ai DOF residuali/root.

## Vedi anche

- [[DIVERGENZA_BUGFIXES]] — root cause del bug SEA (mobility indices, PD scaling, baseline molla)
- [[2026-04-14_output_espansi_e_visualizzatore]] — output diagnostici aggiuntivi e fix state cache
- [[AGENT]] — struttura del plugin SEA, equazioni PD e dinamica motore

## Limiti residui

- In 42 frame su 680 le reserve aiutano anche su DOF classificati muscle-capable.
- Questo indica transitori in cui la mappa muscolare lineare/quasi-statica non chiude tutta la coppia richiesta.
- La pipeline e ora molto piu vicina a un comportamento CMC-like muscle-first, ma non e ancora un CMC completo con integrazione dinamica esplicita di tutte le fibre muscolari.
- I warning sulle geometrie del visualizer sono separati dalla dinamica del simulatore.
