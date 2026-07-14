# Actor robusto deployable e warm-up del critic

Data: 2026-07-13

## Obiettivo

Individuare e correggere la causa per cui il warm-start actor richiedeva
`sigma=0.003`, quindi eseguire il warm-up del solo critic senza usare dati
prescribed nell'actor deployabile.

Vincoli mantenuti:

- dati prescribed ammessi come supervisione iniziale e osservazione privilegiata
  del critic;
- nessun target IK, sound-side state o riferimento prescribed nell'actor;
- reward, FSM, limiter, governor e soglie `15/25 mm` invariati;
- nessun aggiornamento PPO dell'actor durante il warm-up critic.

## Diagnosi causale

E' stato registrato un nuovo trace corretto a `sigma=0.005`. I precedenti trace
chiamati `sigma003` avevano rumore empirico circa `0.03` e sono stati esclusi.

Sul seed 123 corretto:

```text
rumore RMS knee / ankle              0.005103 / 0.004554
deriva RMS media actor               0.240576 / 0.095030
primo drift della media > 0.05       step 76
primo mismatch eventi/FSM            step 120
fine                                 step 206, penetrazione
```

La deriva iniziava quindi prima del disallineamento di fase. Il controfattuale
per gruppi di feature ha mostrato che sostituire lo stato interno nominale del
controller/reference eliminava il `47.52%` della deriva della media. Le feature
principali erano:

- accelerazione della reference servita del ginocchio;
- endpoint precedente del ginocchio;
- velocita' della reference servita;
- stato articolare e motor speed del ginocchio.

La causa era strutturale: durante l'imitazione, previous endpoint, reference
servita e ultimo comando erano fortemente correlati con l'azione prescribed.
L'actor li aveva quindi usati come proxy autoregressivo della prossima azione.
Una piccola perturbazione dell'azione modificava questi input al passo seguente;
la policy spostava la propria media e amplificava la deviazione closed-loop.

## Strategie respinte

Il vecchio prototipo noise-aware etichettava trace a `sigma~0.03`, inclusi stati
dopo mismatch di fase. Modificava inoltre la media nominale fino a `0.114`; non
era una correzione valida.

E' stato provato un adattamento limitato ai 119 step precedenti il primo mismatch
FSM, seguito da proiezione trust-region con massimo spostamento nominale `0.005`.

```text
deterministico                       361 step, 1 ciclo
stocastico sigma 0.005 seed 123      230 step, 0 cicli
clipping stocastico                  1.96%
```

Il candidato e' stato respinto: attenuava il problema, ma non eliminava il proxy
autoregressivo.

## Correzione strutturale

E' stato introdotto `include_controller_state_observation`. Quando e' `false`:

- previous endpoint, reference servita e ultimo comando SEA non entrano
  nell'actor;
- le stesse 14 feature vengono spostate nel suffisso privilegiato del critic;
- il vettore completo resta di 84 feature;
- l'actor passa da 39 a 25 feature e usa soltanto encoder articolari/SEA, GRF
  ipsilaterale ed FSM deployable;
- target IK, biological/sound-side state e reference prescribed restano
  critic-only.

La configurazione ex-novo raccomandata ora usa:

```yaml
include_reference_state_observation: true
include_controller_state_observation: false
deployable_minimal_observation: false
```

Il porting a zero iterazioni ha copiato 23 colonne sensoriali, azzerato i due
clock disabled e verificato actor identico su learner, EnvRunner ed export. Il
critic e l'optimizer sono rimasti freschi.

## Nuovo warm start imitativo

Il teacher ha completato il contratto a 25 feature:

```text
step                                 500/500
HS / TO / cicli                      3 / 3 / 2
return                               64.0324
max penetrazione                     22.944 mm
```

E' stato adattato soltanto l'actor portato sulle 500 coppie actor-observation ->
azione prescribed. La supervisione prescribed e' stata usata esclusivamente per
il warm start; non e' un input della rete.

```text
epoch eseguite / best                243 / 183
RMSE supervisionato                  0.011782
max errore                           0.074282
sigma                                0.005 costante
PPO update                           0
```

Checkpoint congelato:

```text
Trajectory Generator/runs/training/
target_domain_warm_start_deployable_sigma0005_2026-07-13/
rl_module_warm_start
```

Actor digest:

```text
5616be85c815eece79c08730d05529a7b41ea60f3e36033fb5276edc83dd2bf7
```

## Validazione robustezza

```text
modalita'       seed  step  return   HS/TO/cicli  max pen   clip
deterministica  123    500  64.040     3/3/2       22.75 mm  0%
stocastica      123    500  72.715     4/3/3       22.73 mm  0%
stocastica      124    500  66.558     3/3/2       22.29 mm  0%
stocastica      125    500  67.353     3/3/2       22.54 mm  0%
```

Tutti i rollout stocastici usano la distribuzione PPO standard con
`sigma=0.005`, rumore nuovo a ogni step e nessun filtro held. La fragilita' che
imponeva `sigma=0.003` e' quindi risolta nel contratto AB06/start fisso testato.

## Warm-up critic-only

E' stata aggiunta la modalita' `freeze_actor` al custom RLModule. I logits
mantengono gli stessi valori ma sono staccati dal grafo; il value tower continua
a ricevere e usare tutte le 84 feature privilegiate.

Warm-up eseguito:

```text
iterazioni critic-only               1
step campionati                      4096
EnvRunner                            13
epoch value/PPO                      10
learning rate                        1e-4
vf loss                              0.344006
vf explained variance               0.598370
mean KL                              0.0
actor max parameter change           0.0
```

Digest:

```text
actor prima/dopo                     5616be85...2bf7, identico
critic fresco                        89dbf05c...6052
critic caldo                         c0b159c9...7fdb
critic dopo restore                  c0b159c9...7fdb
```

Il `policy_loss=-0.0313` e' una metrica calcolata dalla loss PPO, ma non produce
gradienti sull'actor congelato. `KL=0` e il digest bit-identico confermano che non
e' avvenuto alcun update della policy.

Il critic caldo e l'optimizer vivono nel checkpoint RLlib completo:

```text
validation/critic_warmup/
2026-07-13_deployable_sigma0005_iter1/checkpoint_last
```

Gli export `rl_module_last/best` sono inference-only e rimuovono intenzionalmente
il value tower; non devono essere usati per continuare il critic.

## Deployability e limiti

Il contratto di input actor e' deployable: nessuna delle 25 feature richiede IK
prescribed, sound-side state o stato biologico privilegiato. Il prescribed resta
nel critic durante training e nella provenienza del warm start, come richiesto.

Questo non dimostra ancora generalizzazione: validazione e warm-up usano AB06,
trial e start fissi. L'actor e' ancora un prior interamente imitativo; diventera'
una policy ex-novo solo attraverso PPO con reward task-based. Il risultato non va
quindi interpretato come policy clinica finale.

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/asymmetric_rl_module.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
Trajectory Generator/baseline_MLP/target_domain_imitation.py
Trajectory Generator/baseline_MLP/target_domain_noise_adaptation.py
Trajectory Generator/baseline_MLP/project_actor_update.py
Trajectory Generator/baseline_MLP/README.md
validation/diagnose_actor_fragility.py
validation/test_diagnose_actor_fragility.py
validation/test_project_actor_update.py
validation/test_asymmetric_rl_module.py
validation/test_target_domain_imitation.py
validation/validate_training_config.py
```

## Verifiche

```text
porting zero-iteration               PASS
teacher 500/500                      PASS
actor deterministico 500/500         PASS
actor sigma 0.005                    PASS, 3/3 seed
actor clipping                       PASS, 0%
critic-only warm-up                  PASS
actor bit-exact dopo warm-up         PASS
critic full-checkpoint restore       PASS
```

## TODO

- [x] Individuare la causa della fragilita' a sigma maggiore di `0.003`.
- [x] Rimuovere il proxy autoregressivo senza aggiungere prescribed all'actor.
- [x] Validare deterministico e tre seed a `sigma=0.005`.
- [x] Eseguire warm-up critic-only con actor bit-identico.
- [ ] Riprendere da `checkpoint_last` con `freeze_actor=false`,
      `freeze_logstd=true`, learning rate/epoch actor conservativi e gate KL con
      rollback prima di una nuova H1.
- [ ] Validare start, segmenti/trial e soggetti differenti prima di dichiarare
      generalizzazione o deployability clinica.
- [ ] Integrare memoria soltanto se test di osservabilita' successivi ne mostrano
      la necessita'; non reintrodurre direttamente la memoria di comando rimossa.
