# Analisi knee jerk 2000 e direzione sweetspot GRF - 2026-06-25

Instruction check token: CMC_AGENT_OK_2026

## Problema

Il 2026-06-24 era stato preparato un nuovo training da zero con:

```yaml
grf_penetration_weight: 1.0
grf_ankle_moment_flip_weight: 0.10
pros_knee_ref_acceleration_limit_rad_s2: 60.0
pros_knee_ref_jerk_limit_rad_s3: 2000.0
```

La domanda operativa era se il limite di jerk knee piu' basso potesse ridurre
il chattering/high-frequency del knee senza perdere il beneficio del GRF soft
sulla coppia `tau_spring` dell'ankle.

L'obiettivo piu' ampio resta trovare uno sweetspot della penalizzazione GRF:

```text
penalizzare abbastanza il contatto/GRF per ottenere una coppia ankle piu'
plausibile, ma non cosi' tanto da far sacrificare alla policy il tracking knee
in stance.
```

## Artefatti analizzati

Training nuovo:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-24-2026
```

Rollout nuovo:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-24-2026
```

Plot generati:

```text
plot/06_25_2026_1__asym100_GRFpenalty-lowered2
```

Confronti principali:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_grfsoft_knee1_ankle2_100iter
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
```

## Soluzione / esito

Il rollout del run `knee jerk 2000` e' stato completato, i plot MLP sono stati
generati e le metriche sono state confrontate con il GRF-soft precedente.

Esito:

```text
Il limite knee jerk 2000 riduce clipping e quota high-pass del knee, ma peggiora
troppo imitation, contatto online, reserve e forma della coppia ankle.
```

Quindi il run non va promosso a nuovo baseline.

## Strategia di analisi

L'analisi ha separato:

1. qualita' della reference servita dalla policy rispetto al target imitativo;
2. tracking SEA/plant della reference servita;
3. contenuto high-frequency delle coppie SEA;
4. effetto su GRF online, reserve root e coppia ankle.

Le metriche SEA sono state calcolate scartando i primi `0.2 s` di transiente,
come nei report precedenti. Per il chattering e' stata usata una high-pass
a `20 Hz` su `tau_spring` e `tau_input_plugin`.

Sono stati letti:

```text
rollout_summary.json
rollout_policy_trace.json
rollout_episode_kinematics.sto
rollout_episode_kinematics_reference.sto
rollout_episode_sea_diagnostics.sto
rollout_episode_online_grf.sto
rollout_episode_reserve_torques.sto
rollout_episode_power.sto
```

## Risultati quantitativi

Confronto compatto:

```text
metric                         hard GRF    soft GRF prev   knee jerk 2000   sym60
rollout return                 333.7       348.5           334.8            334.8
reward_mean                    0.666       0.696           0.668            0.668
clip fraction any              0.012       0.058           0.030            0.000
clip knee                      0.006       0.040           0.008            0.000
clip ankle                     0.006       0.026           0.022            0.000
served imitation loss          0.0591      0.0298          0.0491           0.0490
knee target->served q RMSE     0.182 rad   0.137 rad       0.181 rad        0.119 rad
knee served->actual q RMSE     0.0025 rad  0.0026 rad      0.0031 rad       0.0034 rad
ankle target->served q RMSE    0.0357 rad  0.0219 rad      0.0258 rad       0.0449 rad
ankle served->actual q RMSE    0.0041 rad  0.0047 rad      0.0046 rad       0.0054 rad
knee tau_spring RMS            7.03 Nm     6.73 Nm         7.66 Nm          8.06 Nm
knee tau_spring HP20 ratio     0.335       0.460           0.275            0.330
knee tau_input HP20 ratio      0.751       0.920           0.720            0.860
ankle tau_spring RMS           2.81 Nm     4.11 Nm         2.93 Nm          5.68 Nm
ankle tau area positive        +2.36 Nms   +1.70 Nms       +1.79 Nms        +1.46 Nms
ankle tau area negative        -5.95 Nms   -11.30 Nms      -7.20 Nms        -15.90 Nms
left Fy mean                   28.3 N      61.0 N          42.3 N           83.8 N
left contact fraction          0.278       0.411           0.346            0.500
root reserve RMS               502.5       462.6           486.3            446.3
pelvis_ty reserve RMS          490.2       449.1           473.4            432.4
grf penetration loss           0.000890    0.000907        0.000949         0
grf flip loss                  0           0               0                n/a
```

Training:

```text
soft GRF prev:
  best train return = 293.595
  best iter         = 94
  last iter return  = 272.482

knee jerk 2000:
  best train return = 266.641
  best iter         = 83
  last iter return  = 252.098
```

Il peggioramento non e' quindi solo un'anomalia del rollout: si vede gia' nel
training.

## Lettura critica

Il limite `pros_knee_ref_jerk_limit_rad_s3: 2000.0` fa quello che doveva fare
su una metrica locale:

```text
knee tau_spring HP20 ratio:
  soft GRF prev   0.460
  jerk 2000       0.275

knee tau_input HP20 ratio:
  soft GRF prev   0.920
  jerk 2000       0.720

clip knee:
  soft GRF prev   0.040
  jerk 2000       0.008
```

Pero' il prezzo e' alto:

```text
served imitation loss:
  soft GRF prev   0.0298
  jerk 2000       0.0491

knee target->served q RMSE:
  soft GRF prev   0.137 rad
  jerk 2000       0.181 rad

left Fy mean:
  soft GRF prev   61.0 N
  jerk 2000       42.3 N

root reserve RMS:
  soft GRF prev   462.6
  jerk 2000       486.3

ankle tau negative area:
  soft GRF prev   -11.30 Nms
  jerk 2000       -7.20 Nms
```

Il plant/SEA segue ancora bene la reference servita:

```text
knee served->actual q RMSE  = 0.00315 rad
ankle served->actual q RMSE = 0.00456 rad
```

Quindi il collo di bottiglia non e' il low-level tracking della reference. Il
problema e' che la reference servita dalla policy diventa meno adatta: il cap
di jerk riduce la componente rapida, ma anche la capacita' della policy di
gestire il compromesso stance/contact/tracking.

## Interpretazione sul rapporto GRF-knee

I dati disponibili indicano che il peggioramento del knee con penalizzazione GRF
non va spiegato come "il SEA non riesce a seguire". La catena piu' coerente e':

```text
penalty/contact-validity GRF
-> la policy modifica la reference protesica per evitare configurazioni di
   contatto sfavorevoli
-> il knee diventa una variabile posturale/contact-feasibility, soprattutto in
   stance
-> target imitativo knee e obiettivo contatto/ankle entrano in conflitto
-> il tracking target->served del knee peggiora, mentre served->actual resta buono
```

Evidenza a supporto:

- il GRF hard riduce molto il carico sinistro (`left Fy mean 28.3 N`) e ha
  tracking peggiore;
- il GRF soft recupera carico e reserve, ma aumenta clipping e chattering knee;
- il jerk 2000 riduce chattering/clipping, ma perde carico, reserve e tracking;
- la perdita piu' grande e' target->served, non served->actual.

Questa spiegazione e' ancora incompleta: serve una diagnosi per fase del passo,
con stance/swing, COP, knee error e termini reward GRF sullo stesso asse
temporale. Non basta guardare la tabella aggregata.

## File modificati

Creato:

```text
reports/user/2026-06-25_analisi_knee_jerk2000_e_sweetspot_grf.md
```

Generati prima del report:

```text
plot/06_25_2026_1__asym100_GRFpenalty-lowered2
```

Nessuna modifica a codice, plugin C++, semantica SEA o simulatore root.

## Verifiche eseguite

- Monitoraggio del rollout fino alla comparsa dei file finali.
- Generazione plot con:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python plot/plotter.py --mlp
```

- Verifica plot:

```text
plot/06_25_2026_1__asym100_GRFpenalty-lowered2/missing_channels.txt
No missing channels.
```

- Lettura `rollout_summary.json` per il run nuovo e il soft-GRF precedente.
- Lettura `summary.json` del training nuovo e del training soft-GRF precedente.
- Parsing `.sto` per kinematics, SEA diagnostics, online GRF, reserve e power.
- Calcolo di:
  - return e reward mean;
  - clipping totale e per giunto;
  - target->served->actual RMSE;
  - served imitation loss;
  - `tau_spring` RMS, area positiva/negativa e high-pass ratio;
  - `tau_input_plugin` high-pass ratio;
  - GRF left force/contact/penetration;
  - reserve root e `pelvis_ty`.
- Ispezione qualitativa dei plot:
  - `01_time_sea_control_reserve.png`;
  - `03_gaitcycle_torque_angle_power.png`;
  - `05_time_tau_input_tracking_error.png`;
  - `06_time_joint_ref_sea_error.png`;
  - `07_mlp_policy_vs_sound_leg_error.png`.

## TODO

- [ ] Ripristinare `pros_knee_ref_jerk_limit_rad_s3` a `3000.0` prima del
      prossimo training, salvo decisione esplicita contraria.
- [ ] Abbassare ulteriormente o sweepare i pesi GRF per cercare lo sweetspot:
      abbastanza penalty per mantenere ankle `tau_spring` negativa/plausibile,
      ma non abbastanza da sacrificare il knee in stance.
- [ ] Non usare subito acceleration knee `45` o `30`: il jerk 2000 ha gia'
      mostrato che irrigidire la reference migliora una metrica locale ma
      peggiora il risultato globale.
- [ ] Eseguire una diagnosi dati-driven del peggioramento knee in appoggio,
      separando stance e swing e usando almeno:
      `knee target->served error`, `knee served->actual error`,
      `tau_spring`, `tau_input_plugin`, raw/applied action, `left_in_contact`,
      `left_force_y`, `left_penetration`, COP, `grf_ankle_moment_flip_tau_nm`,
      `grf_penetration_loss`, `grf_ankle_moment_flip_loss`.
- [ ] Testare se il peggioramento knee correla temporalmente con:
      contatto sinistro attivo, picchi di penetrazione, forza verticale
      protesica, COP vicino a regioni sospette, oppure saturazione/clipping
      della raw action.
- [ ] Solo dopo questa diagnosi, scegliere tra:
      GRF weight piu' basso, penalty knee-specific, bounded action head,
      oppure nuova formulazione weak contact-confidence.
