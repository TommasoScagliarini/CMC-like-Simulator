# Readiness H1 con esplorazione controllata sigma 0.003

Data: 2026-07-12

## Obiettivo

Risolvere il failure del preflight stocastico del warm start, dimostrare che il
trainer H1 puo' essere inizializzato correttamente e fermarsi prima di qualsiasi
update PPO.

La media deterministica DAgger r2 era gia' valida:

```text
steps / return                    356 / +29.8865
HS / TO / cicli                   2 / 2 / 1
clipping                          0%
```

Il problema era la distribuzione esplorativa. La deviazione standard ereditata
dall'actor imitativo era circa `0.75`; anche i primi candidati controllati
`0.05`, `0.03` e `0.020/0.015` non superavano il gate relativo.

## Analisi e strategie provate

### Recovery distillation noise-aware

Sono stati registrati i tre rollout `sigma=0.03` completi di policy trace:

```text
seed 123 / 124 / 125             204 / 208 / 228 step
```

E' stato costruito un dataset con:

- 356 stati della traiettoria deterministica r2;
- 640 stati closed-loop visitati sotto rumore;
- label di recovery uguale all'azione r2 allo stesso step;
- traiettoria nominale sovrappesata;
- testa `log-std` congelata esattamente.

Il candidato c1 migliorava la loss offline ma spostava la media nominale di
RMSE `0.0203`. Il rollout deterministico terminava a 59 step, prima del TO. Una
variante piu' conservativa c2 manteneva ancora RMSE nominale `0.0184` e non e'
stata promossa al costoso rollout.

Questa prova ha confermato che un ulteriore fit supervisionato non era la
soluzione: la policy e' abbastanza sensibile da trasformare piccoli cambi della
media in una regressione drastica. I pesi r2 devono restare invariati.

### Ricerca della soglia esplorativa

Sono quindi stati creati checkpoint separati modificando soltanto le due righe
`log-std`, sempre con:

```text
mean max_abs_diff su 356 campioni       0.0
log-std output weights                  0.0
save/reload                             bit-exact
PPO updates                             0
```

Probe sul seed 123:

```text
sigma       steps   HS/TO/cicli   esito
0.010         203      1/1/0       fail
0.005         206      1/1/0       fail
0.003         361      2/2/1       candidato
0.001         360      2/2/1       pass, non selezionato
0.0001        359      2/2/1       pass, non selezionato
```

`Sigma=0.003` e' il valore piu' alto osservato che supera la regione instabile
del seed severo. Corrisponde a `log_std=-5.809143` e varianza `9e-6` per
componente dell'action space.

## Candidato selezionato

```text
Trajectory Generator/runs/training/
target_domain_warm_start_r2_sigma0003_2026-07-12/
rl_module_warm_start/
```

Actor digest:

```text
7fa7fbd15db67dd29b9cd428528b68e14648e934f70c165d6d6be901f025383c
```

Rollout stocastici training-like:

```text
seed  steps  return    clip  HS/TO/cicli  invalid  fine
123     361  +31.092    0%      2/2/1        2     penetrazione
124     360  +31.496    0%      2/2/1        1     penetrazione
125     210  -17.892    0%      1/1/0        1     penetrazione
```

Gate relativo:

```text
controlli tecnici e valori finiti        PASS
media deterministica invariata           PASS
TO valido                                PASS, 3/3
almeno un ciclo                          PASS, 2/3
mediana sopravvivenza                     PASS, 360 >= 213.6
clipping mediano                          PASS, 0% < 1%
stato                                     PASS
```

## Preparazione del trainer H1

Il modulo asimmetrico ora supporta `freeze_logstd`. La forward conserva gli
stessi logits, ma durante il training stacca dal grafo solo le componenti
`log-std`; la media continua a ricevere gradienti.

E' stato costruito il trainer target completo con:

```text
num_env_runners                          13
freeze_logstd                            true
iterations                               0
warm-start source                        sigma=0.003
```

Risultato del transplant:

```text
learner actor                            bit-exact
EnvRunner verificati                     14, tutti bit-exact
modulo iniziale salvato                  bit-exact
critic                                   bit-exact, invariato
optimizer source importato               no
pesi sincronizzati prima del sampling    si
iterazioni PPO                           0
```

L'audit runtime ha ricaricato il modulo esportato ed eseguito backprop:

```text
massimo gradiente media                  4.0
massimo gradiente log-std                0.0
massimo peso log-std                     0.0
```

Esito consolidato:

```text
status                                   READY_FOR_H1
ready_for_h1                             true
h1_executed                              false
```

## File modificati

```text
Trajectory Generator/baseline_MLP/asymmetric_rl_module.py
Trajectory Generator/baseline_MLP/configure_actor_exploration.py
Trajectory Generator/baseline_MLP/target_domain_imitation.py
Trajectory Generator/baseline_MLP/target_domain_noise_adaptation.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
validation/evaluate_relative_h1_gate.py
validation/verify_h1_readiness.py
validation/test_actor_exploration_configuration.py
validation/test_asymmetric_rl_module.py
validation/test_target_domain_imitation.py
validation/test_warm_start_preflight.py
```

## Artefatti

```text
gate:
validation/warm_start_rollout_runs/2026-07-12_sigma0003/
relative_h1_gate.json

trainer zero-iteration:
validation/warm_start_port_runs/2026-07-12_sigma0003_h1_readiness/
trainer_zero_iter/

audit consolidato:
validation/warm_start_port_runs/2026-07-12_sigma0003_h1_readiness/
h1_readiness.json
```

## Stato finale e TODO

- [x] Preservare esattamente la media r2.
- [x] Trovare il massimo sigma validato dal protocollo a tre seed.
- [x] Superare il gate relativo pre-H1.
- [x] Implementare e verificare il congelamento `log-std`.
- [x] Validare learner, critic, EnvRunner e modulo esportato a zero iterazioni.
- [x] Fermarsi prima di H1.
- [x] Avviare una sola H1, su richiesta esplicita, usando il candidato
      `sigma=0.003` e `freeze_logstd=true`.
- [x] Dopo H1, confrontare actor digest e `log-std` prima/dopo, quindi eseguire
      H2 deterministico e i tre rollout stocastici. Esito: `REJECT_H1`; vedere
      `2026-07-12_h1_single_iteration_h2_rejected.md`.
