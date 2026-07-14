# 2026-06-22 - Test controfattuale fattore indiretto sym60/asym100

## Problema

Le curve di coppia SEA ankle del rollout `sym60` risultano un unicum rispetto ai rollout successivi, in particolare rispetto ad `asym100`. L'obiettivo era capire se la forma negativa-dominante della coppia fosse:

- un errore di plotting;
- un effetto reale del training di `sym60`;
- un minimo locale/bacino fortunato;
- un fattore indiretto identificabile nella catena policy -> reference -> cascade -> integratore SEA.

## Run confrontati

- `sym60`: `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`
- `asym100`: `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100`

Differenze principali di setup:

- `sym60`: actor-critic simmetrico, critic non privilegiato, best checkpoint iterazione logica 59.
- `asym100`: actor-critic asimmetrico, critic privilegiato, training da zero fino a 100 iterazioni, best checkpoint iterazione 100.
- Escludendo `asymmetric_actor_critic`, parallelismo e numero di iterazioni, il resto del config e' equivalente: reward imitation, online GRF, reference governor e pesi reward.

## Strategia

Prima e' stata esclusa l'ipotesi di errore di plotting:

- `SEA_Ankle_tau_spring_state` coincide con `SEA_Ankle_tau_spring`.
- La coppia si ricostruisce da `K * (SEA_Ankle_motor_angle - pros_ankle_angle)` con errore nell'ordine di `1e-6`.

Poi e' stata analizzata la decomposizione del controller cascade:

```text
qdot_cas = qdot_ref + Kp_outer * (q_ref - q)
e_v      = qdot_cas - qdot
I_cmd    = Ki_inner * integral(e_v) dt
tau      ~= I_cmd
```

Il fattore candidato e' risultato essere l'errore relativo ankle in early stance, subito dopo heel strike left:

```text
heel strike left: circa 15.616 s
finestra critica: 15.5-16.0 s
```

In quella finestra `asym100` manda la reference ankle leggermente davanti allo stato reale, caricando positivo l'integratore. `sym60` invece mantiene `q_ref - q` negativo o vicino a zero, e quindi mantiene negativa la coppia.

## Test controfattuale eseguito

E' stato eseguito un rollout dinamico completo usando `asym100` come policy base, ma applicando un override in memoria nel controller SEA ankle solo tra `15.5` e `16.0 s`.

Override applicato:

```text
q_ref_asym100_cf     = q_asym100     + (q_ref_sym60     - q_sym60)
qdot_ref_asym100_cf  = qdot_asym100  + (qdot_ref_sym60  - qdot_sym60)
```

Quindi non e' stata copiata la reference assoluta di `sym60`; e' stato copiato l'errore relativo visto dal controller. Questo e' importante per non mischiare due stati dinamici diversi.

Il test e' stato fatto con monkeypatch in memoria, senza modificare codice sorgente. Tutti gli output `.sto` sono stati scritti in una cartella temporanea sotto `/private/tmp` e rimossi a fine test.

Cartelle temporanee usate:

- Primo tentativo: `/private/tmp/cmc_counterfactual_8qmhajdj`, fallito prima del rollout per ambiente Python senza `ray`, poi eliminato.
- Test valido: `/private/tmp/cmc_counterfactual__hlx_9cz`, eliminato dopo estrazione metriche.

Verifica cleanup:

```text
tmp_removed: true
```

## Risultati

Baseline `asym100`:

```text
I_at_16                 = +21.21 Nm
tau_at_16               = +20.78 Nm
I_delta 15.5-16.0       = +21.28 Nm
tau_mean 15.5-16.0      = +3.59 Nm
tau_area_pos / neg      = +9.09 / -7.44
```

Baseline `sym60`:

```text
I_at_16                 = -7.60 Nm
tau_at_16               = -7.00 Nm
I_delta 15.5-16.0       = -8.13 Nm
tau_mean 15.5-16.0      = -6.36 Nm
tau_area_pos / neg      = +1.48 / -18.03
```

Controfattuale `asym100` con errore relativo `sym60`:

```text
I_at_16                 = -8.07 Nm
tau_at_16               = -7.94 Nm
I_delta 15.5-16.0       = -8.00 Nm
tau_mean 15.5-16.0      = -6.75 Nm
tau_area_pos / neg      = +6.05 / -12.22
episode_return          = 358.33
override_count          = 500
override_first_t        = 15.501 s
override_last_t         = 16.000 s
```

Il risultato conferma in modo forte che il fattore indiretto e' l'errore relativo ankle in early stance. Quando `asym100` vede lo stesso errore `q_ref - q` / `qdot_ref - qdot` di `sym60`, l'integratore e la coppia ankle diventano praticamente quelli di `sym60` nella finestra critica.

## Interpretazione

La forma positiva/negativa della coppia non nasce direttamente dalla policy action o dal plotting, ma dal modo in cui la reference servita si posiziona rispetto allo stato reale nel momento subito successivo all'heel strike.

Il termine dominante e':

```text
Kp_outer * (q_ref - q)
```

Una differenza media di pochi milliradi viene amplificata da:

```text
Kp_outer_ankle = 47.125
Ki_inner_ankle = 213
```

e quindi diventa una differenza di decine di Nm nell'integratore.

Conclusione operativa:

- `sym60` non e' un artefatto.
- `sym60` ha trovato un bacino funzionale in cui la reference ankle non anticipa lo stato reale in early stance.
- `asym100` ottimizza meglio la reward globale, ma permette alla reference ankle di andare davanti allo stato reale e quindi carica un burst positivo nell'integratore.

## File modificati

- Creato questo report: `reports/user/2026-06-22_test_controfattuale_fattore_indiretto_sym60_asym100.md`

Nessun file sorgente e' stato modificato. Nessun file temporaneo del test e' stato lasciato nel repository o in `/private/tmp`.

## Test e verifiche eseguite

- Verifica dati grezzi vs plot: `SEA_Ankle_tau_spring_state`, `SEA_Ankle_tau_spring` e ricostruzione da molla.
- Confronto config `sym60` vs `asym100`.
- Decomposizione offline di `q_ref - q`, `qdot_ref - qdot`, `cascade_velocity_error`, `outer_i_cmd`.
- Rollout dinamico controfattuale con override in memoria della reference ankle relativa.
- Verifica finale che la cartella temporanea sia stata eliminata (`tmp_removed: true`).

## TODO

- Implementare un test ripetibile, ma sempre temporaneo o diagnostico, per provare finestre piu' ampie: `14.75-16.25 s` e una finestra phase-based legata all'heel strike invece che a tempi assoluti.
- Valutare una reward/penalty mirata su early stance che scoraggi `q_ref - q > 0` per ankle o scoraggi direttamente il burst positivo di `outer_i_cmd` / `tau_spring`.
- Aggiungere ai plot diagnostici una vista dedicata a `q_ref - q`, `qdot_ref - qdot`, `cascade_velocity_error`, `outer_i_cmd` e gait event, per rendere visibile il meccanismo senza script ad hoc.
