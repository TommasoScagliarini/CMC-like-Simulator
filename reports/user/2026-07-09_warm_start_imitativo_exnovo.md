# Warm start imitativo per training ex-novo - 2026-07-09

Instruction check token: CMC_AGENT_OK_2026

## Problema

Serviva introdurre un warm start imitativo actor-only per il training ex-novo
MLP, usando come base la policy `asym100_GRFpenalty-lowered` senza fare resume
RLlib completo.

Il resume completo da `checkpoint_best` non era adatto perche' avrebbe portato
con se' critic, optimizer state e stato PPO del vecchio objective imitativo,
mentre il training ex-novo deve partire come esperimento nuovo.

## Soluzione

Implementato il flag pubblico:

```bash
--warm-start
```

Quando attivo, il trainer usa di default:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter/rl_module_best
```

come sorgente ufficiale `asym100_GRFpenalty-lowered`.

Il transplant e' actor-only:

- critic inizializzato fresco;
- optimizer/PPO state fresco;
- primo layer copiato per nome feature;
- feature target-only dell'ex-novo azzerate;
- modalita' iniziale `drop`, senza mean-bias.

## Strategia

Creato un helper dedicato che non importa Ray/OpenSim e lavora direttamente su
state dict RLModule. Il trainer costruisce prima il nuovo algoritmo ex-novo,
legge dal probe env lo schema actor target corrente, applica il transplant e
poi sincronizza i pesi tramite `algo.set_state()` sul `LearnerGroup`.

Il trainer esporta anche:

```text
actor_transplant_report.json
rl_module_initial_warm_start/
```

cosi' il comportamento pre-training puo' essere analizzato separatamente.

Sono stati aggiunti guardrail:

- `--warm-start` valido solo con `reward_mode: ex_novo`;
- richiesto `asymmetric_actor_critic: true`;
- vietato combinare `--warm-start` con `--resume-from` a livello supervisor;
- sui restart interni da checkpoint non viene riapplicato il transplant.

## File modificati

```text
Trajectory Generator/baseline_MLP/warm_start.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
```

## Test e verifiche

Eseguiti:

```text
py_compile warm_start.py train_ppo_mlp.py asymmetric_rl_module.py
```

Verificato con test sintetico:

- source actor `(256, 31)`;
- target actor `(256, 39)`;
- colonne condivise copiate;
- colonne target-only azzerate;
- critic target lasciato invariato.

Verificato sul checkpoint reale:

```text
pi_encoder.0.weight = (256, 31)
pi_encoder.2.weight = (256, 256)
pi.1.weight         = (4, 256)
```

Verificato target ex-novo corrente:

```text
n_actor = 39
n_full  = 84
```

Smoke tecnico completato con batch ridotto:

```text
Trajectory Generator/runs/training/_warm_start_smoke_20260709_smallbatch
```

Esito:

```text
ok: true
iterations_run: 1
warm_start_applied: true
```

Artefatti presenti:

```text
actor_transplant_report.json
rl_module_initial_warm_start/
checkpoint_last/
checkpoint_best/
rl_module_last/
rl_module_best/
```

Nota: uno smoke con batch reale `4096` e un solo env locale ha applicato
correttamente il warm start ed esportato `rl_module_initial_warm_start/`, ma e'
andato in iteration timeout durante il campionamento. Questo non e' stato
interpretato come errore del transplant.

## TODO

- [ ] Validare scientificamente la feature `--warm-start` con un training
      ex-novo reale, non solo smoke tecnico.
- [ ] Confrontare a parita' di config e seed:
      `ex-novo random init` vs `ex-novo --warm-start`.
- [ ] Eseguire rollout deterministico da `rl_module_initial_warm_start`,
      `rl_module_best` warm-start e baseline random, confrontando reward terms,
      FSM, contatto, penetrazione, clipping e saturazioni SEA.
- [ ] Verificare se il warm start migliora sample efficiency/stabilita' iniziale
      senza introdurre negative transfer dalla policy imitativa.
- [ ] Decidere se promuovere il warm start a percorso sperimentale principale
      solo dopo un confronto almeno 10/40/100 iterazioni contro random init.
