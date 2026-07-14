# Piano migrazione imitation-exnovo: config separati e actor transplant - 2026-06-25

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo il chiarimento sulla transizione da training imitativo a generazione
`ex_novo`, il primo nodo operativo da risolvere e' la migrazione della rete:

```text
policy imitation-trained -> policy ex-novo senza target sano nell'actor
```

Il training imitativo era stato pensato come pretraining della rete, ma i run
imitativi attuali espongono all'actor quattro feature target sane:

```text
healthy_knee_angle_imitation_target
healthy_knee_angle_imitation_target_vel
healthy_ankle_angle_imitation_target
healthy_ankle_angle_imitation_target_vel
```

In `ex_novo` queste feature devono sparire dall'actor. Di conseguenza, un
semplice `--resume-from checkpoint_best --reward-mode ex_novo` non e' una
migrazione pulita: cambia la dimensione/semantica del primo layer della policy e
il critic e' stato addestrato su una reward diversa.

Inoltre e' emersa la necessita' di separare in modo esplicito i config YAML
imitativo ed ex-novo.

## Soluzione proposta

Separare la pipeline in due obiettivi espliciti:

```text
Trajectory Generator/baseline_MLP/training_imitation_cfg.yaml
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

e aggiornare l'entrypoint di training per accettare due flag di alto livello:

```bash
--imitation   # default
--exnovo
```

Comportamento desiderato:

```text
--imitation -> usa training_imitation_cfg.yaml
--exnovo    -> usa training_exnovo_cfg.yaml
```

Per retrocompatibilita' iniziale:

```text
training_cfg.yaml
```

puo' restare come alias/copia del default imitativo finche' comandi e README non
sono stati aggiornati.

Decisione consigliata sugli override:

```text
--config esplicito non va combinato con --imitation/--exnovo
```

oppure, in alternativa, `--config` vince sempre. La scelta preferita e' la prima:
errore esplicito su combinazioni ambigue, per evitare run del tipo
`--exnovo --config training_imitation_cfg.yaml`.

## Strategia di migrazione della rete

La migrazione non deve essere trattata come resume RLlib completo, ma come:

```text
imitation-pretrained policy initialization with feature-aligned actor weight transfer
```

La policy ex-novo viene costruita da zero con la sua observation corretta, poi
inizializzata copiando i pesi dell'actor imitativo per nome feature condiviso.

### Responsabilita' del tool di transplant

Creare un tool dedicato, ad esempio:

```text
Trajectory Generator/baseline_MLP/transfer_imitation_to_exnovo.py
```

Responsabilita':

1. Caricare la rete sorgente imitativa, ad esempio:

   ```text
   Trajectory Generator/runs/training/<run_imitativo>/rl_module_best
   ```

2. Ricostruire l'observation schema sorgente dal relativo
   `training_cfg.resolved.yaml`, includendo le feature imitation.

3. Ricostruire l'observation schema target dal nuovo
   `training_exnovo_cfg.yaml`, senza le feature target sane.

4. Copiare i pesi dell'actor per nome feature, non per indice cieco:

   ```text
   pros_knee_angle -> pros_knee_angle
   SEA_Knee_motor_speed -> SEA_Knee_motor_speed
   online_left_normal_grf_bw -> online_left_normal_grf_bw
   ```

5. Gestire esplicitamente le quattro feature rimosse:

   ```text
   healthy_knee_angle_imitation_target
   healthy_knee_angle_imitation_target_vel
   healthy_ankle_angle_imitation_target
   healthy_ankle_angle_imitation_target_vel
   ```

6. Salvare una inizializzazione target compatibile con il training ex-novo.

7. Scrivere un audit JSON, ad esempio:

   ```text
   actor_transplant_report.json
   ```

   con:

   - run sorgente;
   - config sorgente;
   - config target;
   - feature copiate;
   - feature rimosse;
   - feature target nuove o non inizializzate da sorgente;
   - shape dei layer;
   - norme dei pesi prima/dopo;
   - modalita' di gestione bias.

### Actor si', critic no al primo esperimento

Per il primo esperimento si propone di copiare solo l'actor:

```text
pi_encoder
pi output head
```

e reinizializzare il critic:

```text
vf_encoder
vf
```

Razionale: il critic imitativo stima il value di una reward diversa e puo'
introdurre bias negativo nella prima fase del training ex-novo. Una copia del
critic puo' essere valutata in una ablation separata solo dopo aver validato il
transplant dell'actor.

## Perche' compensare il bias

Il primo layer della policy calcola:

```text
h = activation(Wx + b)
```

Nel training imitativo:

```text
x = [x_shared, x_removed]
```

dove `x_removed` sono le quattro feature target sane. Il contributo pre-attivazione
era quindi:

```text
W_shared * x_shared + W_removed * x_removed + b
```

Nel target ex-novo quelle feature non esistono piu'. Se vengono semplicemente
rimosse, il layer diventa:

```text
W_shared * x_shared + b
```

Questo equivale a dire implicitamente:

```text
x_removed = 0
```

ma zero non e' necessariamente il valore medio naturale delle feature target sane
usate durante il training imitativo. Il risultato puo' essere uno shift brusco
delle attivazioni interne gia' al primo step.

La compensazione del bias prova a preservare il comportamento medio:

```text
b_new = b_old + W_removed * mean(x_removed)
```

ottenendo:

```text
W_shared * x_shared + W_removed * mean_removed + b_old
```

In questo modo non si conserva la dipendenza istantanea dal target sano, che e'
proprio cio' che si vuole rimuovere, ma si conserva il livello medio di
attivazione del layer.

## Modalita' da supportare nel tool

Per non rendere il metodo dogmatico, il tool dovrebbe supportare due modalita':

```bash
--removed-feature-mode drop
--removed-feature-mode mean-bias
```

Interpretazione:

- `drop`: rimuove le colonne delle feature sane e lascia il bias invariato;
- `mean-bias`: rimuove le colonne e compensa il bias con la media delle feature
  sane rimosse.

Default consigliato:

```text
mean-bias
```

ma da validare con mini-ablation su rollout iteration 0.

## Validazione proposta

Prima di lanciare training lunghi:

1. Generare una policy ex-novo inizializzata via transplant.
2. Fare un rollout deterministico iteration 0.
3. Confrontare con:

   ```text
   source imitation rollout
   ex-novo random-init rollout
   ```

4. Monitorare:

   - range azioni raw/applied;
   - clipping;
   - terminazioni/truncation;
   - left Fy;
   - reserve root e pelvis_ty;
   - range knee/ankle;
   - saturazione SEA;
   - tracking served reference -> actual;
   - `tau_spring` knee/ankle.

Poi lanciare due training identici:

```text
A: ex_novo random init
B: ex_novo actor transplant
```

Stesso seed, stessa config, stesso numero di env runner, stesso OS.

Il transplant e' promosso solo se migliora sample efficiency/stabilita' iniziale
senza bloccare l'esplorazione o peggiorare la biomeccanica.

## File modificati

Creato:

```text
reports/user/2026-06-25_piano_migrazione_imitation_exnovo_cfg_transplant.md
```

Nessuna modifica a codice, config, plugin C++ o modelli OpenSim.

## Verifiche eseguite

- Verificato che il loader corrente usa ancora:

  ```text
  Trajectory Generator/baseline_MLP/training_cfg.yaml
  ```

  come `DEFAULT_CONFIG_PATH`.

- Verificato che `training_config.py` usa il YAML come default argparse e che i
  flag CLI espliciti vincono sui valori YAML.

- Verificato dai file letti nella discussione precedente che:
  - `reward_mode: imitation` abilita le quattro feature target sane nell'actor;
  - `reward_mode: ex_novo` le rimuove;
  - con `AC asym` la policy legge `obs[:n_actor]` e il critic legge il vettore
    completo.

## TODO

- [ ] Creare `training_imitation_cfg.yaml` a partire dal config imitativo attuale.
- [ ] Creare `training_exnovo_cfg.yaml` con `reward_mode: ex_novo` e schema actor
      senza target imitation.
- [ ] Decidere se mantenere `training_cfg.yaml` come alias legacy del config
      imitativo o deprecarlo subito.
- [ ] Aggiornare `training_config.py` per conoscere i default path imitation ed
      ex-novo.
- [ ] Aggiornare `train_ppo_mlp.py` con flag `--imitation` e `--exnovo`,
      default `--imitation`.
- [ ] Rendere errore esplicito la combinazione ambigua `--config` +
      `--imitation/--exnovo`, oppure documentare che `--config` vince.
- [ ] Implementare `transfer_imitation_to_exnovo.py`.
- [ ] Supportare nel tool le modalita' `drop` e `mean-bias` per le feature
      rimosse.
- [ ] Scrivere `actor_transplant_report.json` per ogni inizializzazione generata.
- [ ] Validare rollout iteration 0 per:
      source imitation, ex-novo random init, ex-novo transplant drop,
      ex-novo transplant mean-bias.
- [ ] Lanciare ablation training:
      `ex_novo random init` vs `ex_novo actor transplant`.
- [ ] Propagare nei report futuri la distinzione tra config imitativo, config
      ex-novo, checkpoint completo RLlib e export `rl_module_best`.
