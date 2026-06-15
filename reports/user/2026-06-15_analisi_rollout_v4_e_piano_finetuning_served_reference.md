# Analisi rollout V4 e piano di fine-tuning della served reference

**Data:** 2026-06-15

## Problema

Il rollout deterministico del training imitativo V4 ha mostrato un forte
miglioramento della dinamica e dell'inseguimento SEA rispetto al V3, ma la forma
della reference servita non riproduce ancora in modo soddisfacente il target
imitativo. Il comando raw della policy presenta inoltre un'alternanza prossima a
50 Hz, ma il reference governor la attenua prima che raggiunga i controllori.

La priorita decisa non e quindi eliminare subito il chattering raw, ma migliorare
la forma globale della served reference e verificarne la consistenza su piu cicli
del passo.

## Risultati principali del rollout V4

- Rollout completato: 201 step, pari a 2 secondi.
- Knee tracking RMSE: da circa `0.0436 rad` nel V3 a `0.00394 rad`.
- Knee torque error RMSE: da circa `11.83 Nm` a `0.96 Nm`.
- Knee `tau_input` RMS: da circa `158.7 Nm` a `18.4 Nm`.
- Potenza motore assoluta media: da circa `1561 W` a `15 W`.
- Ankle torque error RMSE: da circa `0.655 Nm` a `0.267 Nm`.
- Penetrazione oltre 20 mm: da circa `25.9%` a `10.8%`.
- `tracking_score` medio prossimo a `0.993`: il controllore segue molto bene la
  reference servita.
- Gli score di imitazione actual-target e served-target risultano quasi
  coincidenti, confermando la ridondanza parziale dei due obiettivi quando il
  tracking SEA e accurato.

Il problema residuo principale e quindi nella reference scelta dalla policy, non
nella capacita del controllore SEA di inseguirla.

## Analisi della reward

### Termini imitativi

- `served_imitation_loss`: confronta il target sound-side con la reference
  effettivamente servita ai controllori.
- `sound_imitation_loss`: confronta il target sound-side con il movimento fisico
  effettivo.
- `tracking_loss`: confronta la reference servita con il movimento fisico.

Dato l'eccellente tracking, e stato deciso di rendere dominante la served
imitation, mantenendo una componente ridotta sull'imitazione fisica come guardia
di fattibilita ed eliminando il premio diretto sul tracking.

Configurazione scelta:

```yaml
reward:
  imitation_weight: 4.0
  served_imitation_weight: 8.0
  blend_served_imitation: 0.80
  blend_imitation: 0.20
  blend_imitation_tracking: 0
```

### Chattering e smoothness

Con `policy_knots: 1`, l'attuale `smoothness_loss` vale sempre zero: il termine
`smoothness_weight: 0.02` non regolarizza quindi i comandi tra step consecutivi.

`segment_delta_loss` misura invece la distanza tra il nuovo comando e la
reference servita corrente. E utile per misurare quanto aggressivamente la
policy sollecita il reference governor, ma non misura direttamente
`q_cmd(t) - q_cmd(t-1)`.

Una penalita inter-step esplicita resta una modifica futura opzionale. Non viene
aggiunta ora perche il chattering raw e filtrato e non e stato identificato come
causa primaria della forma errata della served reference.

## Orizzonte temporale

La configurazione e stata aggiornata da:

```yaml
gamma: 0.95
episode_duration: 2.0
```

a:

```yaml
gamma: 0.99
episode_duration: 5.0
```

A 100 Hz, una reward distante un secondo passa da un peso di:

```text
0.95^100 ~= 0.006
```

a:

```text
0.99^100 ~= 0.366
```

Gli episodi da 5 secondi permettono inoltre di osservare piu cicli. `lam` resta
per ora a `0.9`: un eventuale passaggio a `0.95` aumenterebbe la profondita
temporale della GAE, ma verra valutato separatamente per non introdurre troppe
variabili contemporaneamente.

## Asymmetric actor-critic e struttura della rete

L'asymmetric actor-critic e gia implementato e validato, ma non viene attivato
nel prossimo fine-tuning per mantenere invariati rete e observation space.

Il critic simmetrico V4 ha raggiunto una explained variance circa
`0.72-0.81`, quindi non appare il collo di bottiglia principale. La rete corrente
`2 x 256 tanh` ha inoltre gia prodotto un comportamento fisico accurato. I paper
locali supportano soprattutto l'uso di informazioni privilegiate nel critic,
storia delle osservazioni e action-space design, piu che il semplice aumento
della dimensione della MLP.

Un training asymmetric dovra essere eseguito separatamente e partire da zero,
perche cambia RLModule e observation space.

## Strategia di fine-tuning

Il prossimo run sara un fine-tuning breve dal best checkpoint del V4 precedente:

- checkpoint di partenza: `baseline_mlp_imit_v4_c2_4hz/checkpoint_best`;
- best checkpoint corrispondente all'iterazione logica `37`;
- nuove iterazioni richieste: `20`;
- target logico finale: `57`;
- nuovo output directory, per non mescolare return calcolati con reward e durata
  episodio differenti.

Comando:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --config "Trajectory Generator\baseline_MLP\training_cfg.v4_imitation.yaml" --resume-from "runs\baseline_mlp_imit_v4_c2_4hz\checkpoint_best" --output-dir "runs\baseline_mlp_imit_v4_c2_4hz_finetune" --iterations 57
```

`checkpoint_best` viene usato al posto di `rl_module_best` per ripristinare lo
stato RLlib completo, inclusi policy, critic e optimizer.

## File modificati

- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`
  - `gamma` portato a `0.99`;
  - durata episodio portata a `5.0 s`;
  - reward imitativa resa dominante sulla served reference;
  - rimosso il blend positivo sul tracking;
  - `iterations` spostato nella sezione `simulation`.
- `Trajectory Generator/baseline_MLP/training_config.py`
  - `simulation.iterations` reso posizione canonica;
  - mantenuta compatibilita con i vecchi YAML contenenti
    `supervision.iterations`.

## Test e verifiche

- Verificata la semantica di `smoothness_loss`, `segment_delta_loss`,
  `command_rate_loss` e dei tre score imitativi.
- Verificato lo split actor realistico / critic privilegiato e lo stato
  dell'implementazione asymmetric actor-critic.
- Verificato `checkpoint_best_meta.json`: best checkpoint all'iterazione logica
  `37`.
- `python -m py_compile Trajectory Generator/baseline_MLP/training_config.py`:
  PASS.
- Caricamento configurazione V4 aggiornata: `iterations=40`.
- Caricamento retrocompatibile di `training_cfg.yaml`: `iterations=40`.
- Il nuovo fine-tuning non e ancora stato lanciato.

## TODO

- [ ] Lanciare il fine-tuning di 20 iterazioni dal best checkpoint V4.
- [ ] Analizzare la forma della served reference su piu cicli, confrontandola con
      target imitativo e cinematica fisica effettiva.
- [ ] Confrontare frequenze della served reference, jerk, carichi SEA, contatti e
      penetrazione GRF con il rollout V4 precedente.
- [ ] Valutare `lam: 0.95` in un'ablation separata se il credito temporale resta
      troppo locale.
- [ ] Aggiungere una penalita esplicita `q_cmd(t) - q_cmd(t-1)` solo se il
      chattering raw degrada served reference, SEA o stabilita numerica.
- [ ] Valutare la rimozione di `smoothness_weight` finche `policy_knots` resta
      uguale a 1.
- [ ] Eseguire separatamente un training reale asymmetric actor-critic da zero.
