# Completamento del run imitativo (40/40) e validazione del seed

**Data**: 2026-06-11
**Contesto**: esecuzione e monitoraggio fino a completamento del pre-training
imitativo PPO/MLP in `Trajectory Generator/runs/baseline_mlp_imit_win`, lo stesso
run il cui crash nativo Ray all'iterazione 22 è analizzato nel report compagno
`2026-06-11_diagnosi_crash_nativo_ray_e_recovery_training.md`.

## Problema

Il pre-training imitativo (la protesi imita la gamba sana in anti-fase, seed per
l'ex-novo) era partito la sera del 2026-06-10, era già crashato una prima volta
all'iter 14 ed era stato ripreso da checkpoint. Durante questa giornata ha subito
un **secondo crash nativo Ray all'iter 22** (diagnosi nel report compagno). Restava
da stabilire operativamente:

1. se il run sarebbe arrivato alle 40 iterazioni target attraversando il crash;
2. la qualità del seed prodotto (return, salute del critico, terminazioni);
3. dove sono salvati i risultati e quale checkpoint usare per i passi successivi.

Questo chiude di fatto il TODO #1 del report compagno: *"eseguire un training
prolungato e verificare recovery e avanzamento dopo crash reali"*.

## Esito: training COMPLETATO

Il run è arrivato a **40/40 iterazioni** con esito pulito (da `summary.json`):

```text
ok                              = true
stop_reason                     = completed
stop_message                    = "Training COMPLETE — logical target 40,
                                   40 successful iteration(s), best return 47.61."
best_episode_return_mean        = 47.614
restored_logical_iteration      = 21        (resume dopo il crash dell'iter 22)
iteration_start                 = 22
iterations_completed_this_process = 19      (22 -> 40)
elapsed_wall_time_s (proc. finale) = 15629  (~4.34 h)
restart_count / crash_restart_count = 0 / 0
```

### Curva di apprendimento (resume da iter 13 -> 40)

Return quasi triplicato, con il **recupero dal crash dell'iter 22 ben visibile**
(dip 21.39 -> 19.39 = resume dal checkpoint dell'iter 21, poi risalita monotòna):

```text
iter 13: 15.51   iter 21: 21.39   [crash iter 22]   iter 22: 19.39
iter 25: 23.55   iter 30: 33.26   iter 35: 41.54
iter 38: 46.17   iter 40: 47.61  (finale)
```

- **Critico sano** per tutto il run: `vf_explained_var` ~0.75–0.87 (inversione
  stabile rispetto alla explained-variance storicamente negativa).
- **Terminazioni**: nettamente dominate da `episode_time_limit` (episodi che
  completano i 2 s), con `grf_penetration` minoritaria e ~costante.
- Nessun NaN, nessuna divergenza di policy/critico.

### Nota sul meccanismo di recovery

Il processo che ha completato è ripartito dal checkpoint dell'**iter 21** e ha
eseguito 22→40 **senza ulteriori crash** (`crash_restart_count = 0`,
`crash_restarts = []`). Di conseguenza il seed è stato consegnato, ma il percorso
di **auto-recovery interno da crash nativo reale** (introdotto oggi nel supervisor)
**non è stato esercitato dal processo vincente**, perché quel processo non ha
crashato. Inoltre il wrapper PowerShell esterno (`imit_autorestart.log`) è rimasto
fermo a "Attempt 1": non è stato lui a guidare la ripresa. Vedi TODO sotto.

## Dove sono salvati i risultati

Tutto in `Trajectory Generator/runs/baseline_mlp_imit_win/` (~8.5 MB):

| Artefatto | Contenuto | Uso |
|---|---|---|
| `checkpoint_best/`, `checkpoint_last/` | Checkpoint RLlib completo (policy+critic+optimizer+stato), iter 40 | **Warm-start / resume** (`--resume-from`) |
| `rl_module_best/`, `rl_module_last/` | Solo RLModule (pesi policy) | **Rollout / inference** (`--checkpoint`) |
| `train_iterations.jsonl` | 40 righe di metriche per-iterazione | Analisi curve |
| `tensorboard/` | 4 file di eventi (4 sessioni-driver successive) | TensorBoard |
| `summary.json`, `supervisor_state.json`, `watchdog_state.json` | Bookkeeping supervisor/restart | Diagnostica |
| `faulthandler.log` (~5.3 MB) | Stack dump periodici + tracce crash | Debug |

`best` e `last` coincidono (entrambi iter 40, return 47.61) perché il return è
cresciuto in modo quasi monotòno fino alla fine. I **4 file tensorboard** (PID
distinti nel nome) sono la prova fisica che il run completo ha attraversato più
sessioni-driver senza perdere progresso, grazie a `--checkpoint-every 1`.

## Strategia seguita

1. Monitoraggio in tempo reale via journal (`train_iterations.jsonl`) e log del
   wrapper, con un monitor che emette un evento per iterazione e si chiude a iter 40.
2. Diagnosi dello stato dei processi (trainer, wrapper, Ray) durante la finestra
   di crash dell'iter 22 e durante la ripresa.
3. Verifica del completamento e degli artefatti finali tramite `summary.json`,
   `supervisor_state.json` e i metadati dei checkpoint.

## File modificati

Nessuno. Giornata operativa di esecuzione/monitoraggio/validazione: nessuna
modifica a codice Python, plugin C++ SEA, semantica del comando SEA, reward o
simulatore biomeccanico. (Le modifiche al supervisor citate dal report compagno
sono state fatte e documentate separatamente.)

## Verifiche eseguite

- Completamento confermato: `summary.json` `ok:true`, `stop_reason:completed`,
  40/40, best return 47.61.
- Checkpoint finali validi: `checkpoint_best`/`checkpoint_last` e
  `rl_module_best`/`rl_module_last` a `logical_iteration 40`.
- Curva di return e `vf_explained_var` ispezionate iter-per-iter (13→40),
  recupero dell'iter 22 verificato sui dati.
- Stato processi a fine run: nessun trainer/wrapper/Ray residuo (uscita pulita).
- Posizione dei risultati confermata e dimensioni verificate.

## TODO

### Immediati (linea imitazione -> ex-novo)
- [ ] **Rollout deterministico di verifica** del seed
      (`rl_module_best`, `--record-outputs`) per controllare il comportamento, non
      solo il return: inseguimento anti-fase della gamba sana, assenza di cadute,
      terminazioni; generare i `.sto` ed eventualmente visualizzarli.
- [ ] **Warm-start ex-novo** dal seed
      (`--resume-from runs\baseline_mlp_imit_win\checkpoint_last --reward-mode ex_novo`),
      una volta pronta la reward task-based.

### Propagati dal report compagno `2026-06-11_diagnosi_crash_nativo_ray_e_recovery_training.md`
- [ ] **Validare l'auto-recovery interno su un crash nativo REALE**: il run
      completato non ha esercitato il nuovo percorso di crash-restart
      (`crash_restart_count = 0`), quindi resta da osservare una ripresa automatica
      end-to-end dopo un access violation reale, con `--checkpoint-every 1` e
      `--max-consecutive-crash-restarts 5`.
- [ ] Rendere il recovery **osservabile/loggato**: né il wrapper esterno (log fermo
      a "Attempt 1") né i contatori interni hanno tracciato chi ha rilanciato dopo
      l'iter 22; serve un log esplicito del restart per non lasciare ambiguità.
- [ ] Stabilizzare/validare l'ambiente Ray su Windows (matrice controllata
      Python/Ray/`grpcio`/`protobuf`); raccogliere un dump nativo più informativo
      del prossimo access violation per la causa radice del `dashboard_agent` crash.
- [ ] Verificare su macOS arm64 il cleanup del process group e il resume RLlib
      (propagato dal 2026-06-10); ricompilare il plugin onlineGRF `.dylib`.
- [ ] Monitorare `vf_explained_var` e il divario predetto-vs-return anche nel
      prossimo training (propagato da
      `2026-06-11_monitoraggio_critico_vf_metrics_prossimo_training.md`).

### Generatore ex-novo (linea principale, propagati)
- [ ] Reward ex-novo task-based (auto-periodicità via gait clock, coordinazione/
      anti-fase, fattibilità GRF, stabilità, effort/energia SEA) e decisione sul
      riferimento IK nel critico (cfr.
      `2026-06-11_progettazione_reward_ex_novo_task_based.md`).
- [ ] Throughput ~1.18 s/env-step (SO QP) come collo di bottiglia; timing
      heel-strike online lato protesico e flag `in_contact`.
