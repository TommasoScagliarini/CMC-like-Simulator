# Design reward core ex-novo, timing gait cycle e profili GRF

## Problema

Si e consolidata la separazione tra due usi distinti della GRF online:

- supporto dinamico della simulazione CMC-like;
- rilevamento di contatto, heel strike, toe off e fase protesica per reward/actor.

Restava da aggiornare la nomenclatura dei profili e chiarire il design della reward task-based ex-novo, in particolare:

- come gestire periodicita e durata del gait cycle;
- come evitare un `T_nom` autoreferenziale;
- se includere subito un termine di forma della traiettoria;
- come mantenere la reward coerente con una claim prescribed-free.

## Soluzione

I profili GRF sono stati rinominati semanticamente:

- `AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json`: profilo da usare per il supporto dinamico della simulazione;
- `AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`: profilo/detector da usare per HS/TO e fase protesica.

La reward core ex-novo viene definita, per il primo pass, con sei blocchi:

1. `stance contact/load`: premiare contatto e carico non nullo durante stance protesica.
2. `swing unloading`: penalizzare contatto/carico quando la protesi dovrebbe essere in swing.
3. `contact validity`: penalizzare penetration, slip, COP non plausibile e moment flip.
4. `phase sequence + periodicity + timing range`: premiare sequenza HS -> stance -> TO -> swing -> HS, regolarita tra cicli consecutivi e durata del ciclo dentro un range biomeccanico.
5. `prosthetic joint range`: tenere knee/ankle protesici in range biomeccanici morbidi.
6. `SEA usability`: penalizzare saturazione, chattering, power assurda e uso meccanico degenerato.

Il punto 7, `prosthetic kinematic morphology`, viene accettato come necessario ma rimandato a un secondo step.

## Strategia per il timing del gait cycle

Per il primo pass si evita una stima dinamica di `T_nom` durante training.

Decisione:

- `T_nom` fisso;
- model-based;
- specifico per AB06 treadmill;
- stimato offline dalla condizione/reference sperimentale;
- salvato come parametro dichiarato in config.

Questo evita un loop autoreferenziale in cui la policy produce cicli troppo veloci o troppo lenti e lo stimatore adatta `T_nom` a quella stessa anomalia.

Schema previsto:

```text
T_cycle = HS_i+1 - HS_i

T_low  = T_nom - margin_low
T_high = T_nom + margin_high

se T_cycle in [T_low, T_high]:
    penalty_time = 0

se T_cycle < T_low:
    penalty_time = huber((T_low - T_cycle) / margin_T)

se T_cycle > T_high:
    penalty_time = huber((T_cycle - T_high) / margin_T)
```

In parallelo:

```text
penalty_periodicity = huber((T_cycle_i - T_cycle_i-1) / sigma_period)
```

Hard guard/termination solo per casi patologici:

- HS troppo ravvicinati;
- `T_cycle` oltre un massimo hard;
- stance troppo lunga senza TO;
- swing troppo lunga senza HS.

## Nota su Huber e scale temporali

`huber(x)` e una loss robusta:

- quadratica per errori piccoli;
- lineare per errori grandi.

Serve a penalizzare gli errori temporali senza far esplodere la reward quando il ciclo e molto sbagliato.

`margin_T` o `sigma_period` sono scale temporali di normalizzazione. Trasformano errori in secondi in grandezze adimensionali, ad esempio:

```text
x = errore_temporale / margin_T
```

## Punto 7 rimandato

Il termine `prosthetic kinematic morphology` e utile per incentivare forme di traiettoria biomeccanicamente plausibili senza tornare a una imitazione prescribed.

Esempi futuri:

- knee flexion minima in swing;
- knee extension prima di HS;
- ankle non collassata in stance;
- toe clearance;
- smoothness;
- assenza di oscillazioni ad alta frequenza.

Per ora non viene incluso nel primo pass per mantenere la reward core piu interpretabile e diagnosticabile.

## File modificati

Profili rinominati:

- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json`;
- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`.

Riferimenti operativi aggiornati:

- `Trajectory Generator/baseline_MLP/env_factory.py`;
- `Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py`;
- `Trajectory Generator/baseline_MLP/rollout_eval.py`;
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`;
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`;
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`;
- `validation/validate_online_grf_events.py`;
- `validation/_hybrid_env_smoke.py`;
- `validation/_env_timing.py`.

Report aggiornati:

- `reports/user/2026-06-26_validazione_eventi_grf_fase_protesica_reward.md`;
- `reports/user/2026-06-26_confronto_ibrido_profili_grf_online.md`;
- `reports/user/2026-06-26_decisione_profili_grf_supporto_fase_reward.md`;
- `results/hybrid_profile_ab_comparison/report.md`.

Creato questo report:

- `reports/user/2026-06-26_design_reward_core_exnovo_timing_grf.md`.

## Test e verifiche

Verifiche eseguite dopo il rename:

- ricerca globale: nessuna occorrenza residua di `coorect`;
- verifica presenza del nuovo file `AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json`;
- `py_compile` sui Python operativi toccati:
  - `Trajectory Generator/baseline_MLP/env_factory.py`;
  - `Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py`;
  - `Trajectory Generator/baseline_MLP/rollout_eval.py`;
  - `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`;
  - `validation/validate_online_grf_events.py`;
  - `validation/_hybrid_env_smoke.py`;
  - `validation/_env_timing.py`.

## TODO

- Stimare offline `T_nom`, `T_low`, `T_high` e i limiti hard per AB06 treadmill dalla reference GRF/eventi.
- Inserire in config i parametri temporali della fase protesica in modo esplicito e documentato.
- Implementare reward core con logging separato per i sei blocchi.
- Implementare la separazione tra profilo dinamico `grf_correct_magnitude` e detector `grf_detector_HS-TO` nella pipeline RL.
- Rinviare il punto 7, `prosthetic kinematic morphology`, a un secondo step dopo la diagnosi della reward core.
- Preparare in futuro una stima dinamica di `T_nom` basata su velocita stimata da sensori protesici, lenta, filtrata e clampata, senza dipendere direttamente dal periodo HS->HS prodotto dalla policy nel ciclo corrente.
