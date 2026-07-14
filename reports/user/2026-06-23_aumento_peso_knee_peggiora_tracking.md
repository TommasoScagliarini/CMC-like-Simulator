# Aumento peso knee e peggioramento tracking - 2026-06-23

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo aver osservato che il rollout `asym100 + GRF contact-validity package`
peggiorava il tracking del knee soprattutto in appoggio, e' stata testata
l'ipotesi che il knee fosse semplicemente sottopesato nella reward imitation.

La prova piu' rilevante e' stata un training da zero a 40 iterazioni con peso
posizione knee aumentato, ma senza aumentare la velocity knee:

```yaml
imitation_knee_position_weight: 3.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_position_weight: 2.0
imitation_ankle_velocity_weight: 0.04
```

L'obiettivo era capire se, evitando il bias del checkpoint preaddestrato, la
policy avrebbe imparato a mantenere il knee piu' vicino al target imitativo senza
perdere i benefici del pacchetto GRF.

## Run analizzati

Training:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026
```

Caratteristiche:

```text
training da zero
target logico 40 iterazioni
best checkpoint iterazione 39
best_episode_return_mean 101.5160474100792
```

Rollout del best:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_knee3_pos_40iter
```

Plot generati:

```text
plot/06_23_2026_4
```

Il plotter ha riportato:

```text
No missing channels.
```

## Risultati rollout

Metriche globali:

```text
episode_return             202.91400486840143
reward_mean                0.40501797378922444
reward_min                -0.940269505019008
reward_max                 0.9592022961148391
action_abs_max             1.1816991567611694
action_clipped_steps       20
action_clipped_fraction    0.01996007984031936
terminated                 false
truncated                  true  # episode_time_limit
```

Metriche imitation medie dal `rollout_policy_trace.json`:

```text
pros_knee_angle_served_imitation_position_loss   0.07751590324371488
pros_knee_angle_served_imitation_velocity_loss   0.010257086146805754
pros_ankle_angle_served_imitation_position_loss  0.0255369458004525
pros_ankle_angle_served_imitation_velocity_loss  0.015377598228513494
served_imitation_loss                            0.2844418469841263
command_rate_loss                                2.562754337982955
grf_penetration_loss                             0.0009247479849812493
grf_ankle_moment_flip_loss                       0.0
```

## Confronto con GRFpenalty 100

Run di riferimento:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
plot/06_23_2026_1_asym100_GRFpenalty
```

Confronto sintetico:

```text
                                  GRFpenalty 100      knee_pos=3, 40iter
episode_return                    333.665             202.914
reward_mean                       0.6660              0.4050
action_clipped_fraction           0.0060              0.0200
knee served position loss         0.0383              0.0775
knee served velocity loss         0.0057              0.0103
ankle served position loss        0.0101              0.0255
ankle served velocity loss        0.0113              0.0154
served imitation loss             0.0591              0.2844
command_rate_loss                 1.77                2.56
```

L'aumento del peso posizione knee non ha migliorato il tracking knee: lo ha
peggiorato. Il peggioramento non e' limitato al knee, ma coinvolge anche ankle,
command rate, clipping e reward globale.

## Interpretazione

Il risultato falsifica, almeno per questa configurazione, l'ipotesi semplice:

```text
il knee peggiora perche' pesa troppo poco nella reward
```

La lettura piu' coerente con i dati e':

```text
il knee e' una leva che la policy usa per soddisfare contatto/GRF/ankle.
Quando il peso del knee aumenta, la policy non trova una soluzione dinamica
alternativa stabile: il compromesso peggiora, aumenta il command rate e cresce
il clipping.
```

In altri termini, il tracking knee non sembra essere un obiettivo ignorato per
mancanza di peso. Sembra piuttosto un vincolo in conflitto con il pacchetto GRF
contact-validity e con la dinamica del contatto online.

Questo e' coerente anche con l'osservazione visiva del plot:

```text
plot/06_23_2026_1_asym100_GRFpenalty/07_mlp_policy_vs_sound_leg_error.png
```

Nel run GRFpenalty, il knee segue i picchi di flessione ma si stacca
soprattutto nei tratti bassi del ciclo, compatibili con appoggio/carico. Il
fenomeno non appare come un errore globale distribuito uniformemente.

## Strategia aggiornata

Non conviene continuare ad aumentare il peso knee. In particolare, un ulteriore
training con `imitation_knee_position_weight` ancora piu' alto rischia di
rendere piu' rigido un vincolo gia' in conflitto, senza offrire alla policy una
soluzione fisicamente migliore.

La prossima ipotesi da testare e':

```text
il pacchetto GRF contact-validity e' troppo forte o troppo severo;
la policy protegge il contatto/ankle sacrificando il knee in stance.
```

Prova consigliata:

```yaml
reward:
  grf_penetration_weight: 1.0        # invece di 5.0
  grf_ankle_moment_flip_weight: 0.10 # invece di 0.25
```

mantenendo invariati i pesi imitation del run GRFpenalty:

```yaml
imitation_knee_position_weight: 1.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_position_weight: 2.0
imitation_ankle_velocity_weight: 0.04
```

In alternativa o in seconda battuta, rendere meno severe le soglie di
penetrazione:

```yaml
simulation:
  grf_penetration_penalty_threshold_m: 0.015
  grf_penetration_termination_m: 0.022
```

L'esperimento va letto come confronto di direzione a 40 iterazioni, non come
validazione finale. Se la GRF soft recupera knee senza far tornare il burst
ankle, allora il problema principale e' la pressione eccessiva della reward GRF.

## File modificati

Nessun file di codice o configurazione e' stato modificato per questa analisi.

File creato:

```text
reports/user/2026-06-23_aumento_peso_knee_peggiora_tracking.md
```

Output generati prima del report:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_knee3_pos_40iter
plot/06_23_2026_4
```

## Test e verifiche eseguite

- Confermato dal `summary.json` del training:
  - training completo;
  - 40 iterazioni;
  - best checkpoint a iterazione 39;
  - reward config con `imitation_knee_position_weight: 3.0`.
- Eseguito rollout del `rl_module_best`.
- Generati plot con:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python plot/plotter.py --mlp
```

- Verificato `No missing channels`.
- Estratte metriche medie da `rollout_policy_trace.json`.
- Confrontate le metriche principali con il run `GRFpenalty 100`.

## TODO

- [ ] Non proseguire con ulteriori aumenti del peso knee prima di testare una
      GRF penalty piu' morbida.
- [ ] Eseguire un training da zero a 40 iterazioni con GRF soft e pesi imitation
      originali del run `GRFpenalty`.
- [ ] Fare rollout del best del training GRF soft e generare plot.
- [ ] Confrontare `GRFpenalty 100`, `knee_pos=3.0 40iter` e `GRF soft 40iter`
      su:
      knee served-target RMS in stance,
      knee bias in stance,
      ankle `tau_spring`,
      online left mean Fy,
      durata/campioni di contatto,
      `grf_penetration_loss`,
      `grf_ankle_moment_flip_loss`,
      clipping,
      return.
