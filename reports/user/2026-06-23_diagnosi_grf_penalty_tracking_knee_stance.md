# Diagnosi GRF penalty e tracking knee in stance - 2026-06-23

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo il training `asym100 + GRF contact-validity package`, la coppia ankle e'
migliorata nettamente rispetto ad `asym100`, ma il tracking del knee rispetto al
target imitativo e' peggiorato. Il peggioramento non appare uniforme lungo tutto
il ciclo: dal plot

```text
plot/06_23_2026_1_asym100_GRFpenalty/07_mlp_policy_vs_sound_leg_error.png
```

la curva `served ref` del knee segue bene i picchi di flessione, ma si stacca
soprattutto nei tratti bassi del ciclo, compatibili con le fasi in cui il piede
protesico e' in appoggio/carico.

Questo rende debole l'ipotesi "solo problema di pesi imitation": se fosse solo
un sottopeso globale del knee, ci si aspetterebbe un errore piu' distribuito nel
ciclo. Il comportamento osservato suggerisce invece un conflitto tra vincolo di
contatto GRF e imitazione knee.

## Evidenze principali

Rollout di riferimento con pacchetto GRF:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
plot/06_23_2026_1_asym100_GRFpenalty
```

Metriche rollout:

```text
episode_return             333.66533305871303
reward_mean                0.6659986687798664
action_clipped_fraction    0.005988023952095809
action_abs_max             1.2520431280136108
terminated                 false
truncated                  true
```

Confronto sintetico `asym100` vs `GRFpenalty` sul tracking imitativo:

```text
                         asym100      GRFpenalty
knee served pos RMS       0.099 rad    0.182 rad
ankle served pos RMS      0.028 rad    0.036 rad
knee served bias         -0.041 rad   -0.124 rad
clip fraction             0.0429       0.0060
```

Separando i campioni in base al carico online sinistro:

```text
GRFpenalty:
all samples       knee bias -0.124 rad, RMS 0.182 rad
left_force_y>50N  knee bias -0.153 rad, RMS 0.170 rad
not loaded        knee bias -0.116 rad, RMS 0.186 rad
```

Il bias e' piu' negativo quando il piede protesico e' caricato, ma resta visibile
anche subito fuori dal carico perche' la deviazione e' lenta, non un picco
istantaneo.

Inoltre il pacchetto GRF cambia molto il modo in cui viene usato il contatto
online:

```text
online left mean Fy:
asym100      circa 116 N
GRFpenalty   circa  28 N

campioni left_force_y > 50 N:
asym100      217 / 501
GRFpenalty   108 / 501
```

Questo e' compatibile con l'ipotesi che la policy impari a evitare regioni di
contatto rischiose, pagando con una postura knee diversa.

## Normalizzazione reward e pesi

La reward imitation e' gia' normalizzata per range:

```text
position_loss = ((target_q - current_or_served_q) / target_q_range)^2
velocity_loss = ((target_qdot - current_or_served_qdot) / target_qdot_range)^2
```

Range imitativi usati:

```text
pros_knee_angle  position range = 0.93256 rad
pros_ankle_angle position range = 0.35478 rad
pros_knee_angle  velocity range = 8.11114 rad/s
pros_ankle_angle velocity range = 3.37594 rad/s
```

Quindi la normalizzazione evita che una coordinata domini solo per il suo range
fisiologico. Tuttavia, a parita' di errore assoluto in radianti, l'ankle resta
piu' costoso del knee, soprattutto perche' nel setup `GRFpenalty` i pesi sono:

```yaml
imitation_knee_position_weight: 1.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_position_weight: 2.0
imitation_ankle_velocity_weight: 0.04
```

Questo non basta pero' a spiegare tutto: due resume da checkpoint preaddestrato
con pesi knee piu' alti non hanno recuperato il tracking knee.

## Esperimenti resume gia' osservati

Resume con:

```json
{
  "imitation_knee_position_weight": 2.0,
  "imitation_knee_velocity_weight": 0.04,
  "imitation_ankle_position_weight": 1.0,
  "imitation_ankle_velocity_weight": 0.02
}
```

Rollout:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_resume_best_rebalance_knee2_ankle1_10iter_resume_best_rebalance_knee2_ankle1_10iter
plot/06_23_2026_3
```

Risultato:

```text
episode_return             299.07750825450046
action_clipped_fraction    0.018962075848303395
knee served pos loss       0.043957
knee served vel loss       0.005655
ankle served pos loss      0.007981
ankle served vel loss      0.016270
```

Rispetto a `GRFpenalty`, il tracking knee non migliora; peggiora leggermente in
posizione. Aumentano anche clipping e rumore. Questo falsifica l'idea che basti
un semplice reweight locale dopo pretraining.

Resta pero' possibile che i resume siano falsati dal bacino di attrazione del
checkpoint precedente: la policy aveva gia' imparato a usare il knee come leva
di compensazione del contatto. Dieci iterazioni di resume possono non bastare a
cambiare strategia.

## Interpretazione aggiornata

La spiegazione piu' plausibile non e':

```text
il knee e' solo sottopesato
```

ma:

```text
il pacchetto GRF contact-validity e' abbastanza forte da spingere la policy
a modificare il carico/contatto protesico; il knee diventa la leva posturale
con cui evitare regioni GRF sfavorevoli, soprattutto in stance.
```

Il termine GRF puo' risultare piccolo nel rollout finale proprio perche' la
policy lo evita gia' durante il comportamento appreso. Quindi la pressione del
termine va valutata sul training e sugli effetti emergenti, non solo sul valore
medio della loss nel rollout best.

## Strategia proposta

Evitare per ora reward/guardie troppo model-specific sul knee in stance. Una
penalty del tipo "se left_force_y > soglia, limita il knee" sarebbe efficace ma
rischia di codificare dettagli del modello e del lato applicato.

La teoria alternativa da testare e':

```text
la GRF penalty attuale e' troppo forte o troppo severa,
quindi il knee viene sacrificato per soddisfarla.
```

Test consigliato:

1. Eseguire un training da zero a 40 iterazioni con stessi pesi imitation del
   run GRFpenalty, ma pacchetto GRF piu' morbido.
2. Confrontarlo contro il comportamento del run `GRFpenalty` alle prime 40
   iterazioni, non solo contro il best a 100.
3. Fare rollout del best e generare plot.
4. Misurare separatamente errore knee loaded/unloaded, ankle torque, durata del
   contatto online e clipping.

Esempi di varianti GRF soft da valutare:

```yaml
reward:
  grf_penetration_weight: 1.0        # invece di 5.0
  grf_ankle_moment_flip_weight: 0.10 # invece di 0.25
```

oppure soglie meno severe:

```yaml
simulation:
  grf_penetration_penalty_threshold_m: 0.015
  grf_penetration_termination_m: 0.022
```

Un training da zero con `imitation_knee_position_weight: 3.0` resta una prova
possibile, ma ora e' meno prioritario rispetto alla verifica "GRF penalty troppo
forte", perche' i resume con pesi knee piu' alti non hanno mostrato recupero.

## File modificati

Nessun file di codice o configurazione e' stato modificato in questa fase di
analisi.

File creato:

```text
reports/user/2026-06-23_diagnosi_grf_penalty_tracking_knee_stance.md
```

## Test e verifiche eseguite

- Ispezionato il plot:

```text
plot/06_23_2026_1_asym100_GRFpenalty/07_mlp_policy_vs_sound_leg_error.png
```

- Verificato nel codice che la reward imitation usa range normalizzati e pesi
  per coordinata.
- Verificati i range di normalizzazione gia' documentati nel report del
  2026-06-16.
- Confrontati i rollout:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_resume_best_rebalance_knee2_ankle1_10iter_resume_best_rebalance_knee2_ankle1_10iter
```

- Calcolate metriche knee/ankle served-target su tutti i campioni e separando
  `left_force_y > 50 N`.
- Verificato che i due resume con pesi knee piu' alti non hanno migliorato il
  tracking knee e hanno peggiorato return/clipping.

## TODO

- [ ] Eseguire un training da zero a 40 iterazioni con GRF penalty piu' morbida
      mantenendo invariati i pesi imitation del run `GRFpenalty`.
- [ ] Fare rollout del `rl_module_best` del training GRF soft e generare i plot.
- [ ] Confrontare `GRFpenalty` vs `GRF soft` su:
      knee served-target RMS in stance,
      knee bias in stance,
      ankle `tau_spring`,
      online left mean Fy,
      durata/campioni di contatto,
      `grf_penetration_loss`,
      `grf_ankle_moment_flip_loss`,
      clipping e return.
- [ ] Solo se GRF soft non risolve, rivalutare un training da zero con
      `imitation_knee_position_weight: 3.0`,
      `imitation_knee_velocity_weight: 0.02`,
      ankle invariato.
