# Daily Report - 2026-07-12

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-12_warm_start_exploration_variance_preflight.md`
- `reports/user/2026-07-12_teacher_noise_correlated_actor_diagnostics.md`
- `reports/user/2026-07-12_readiness_h1_sigma0003.md`
- `reports/user/2026-07-12_h1_single_iteration_h2_rejected.md`

## Sintesi

La giornata ha caratterizzato la fragilita' esplorativa del warm start,
selezionato `sigma=0.003` come massimo valore capace di superare il gate
relativo pre-H1 e verificato il congelamento di `log-std`. E' stata poi
eseguita una sola iterazione PPO controllata.

H1 ha spostato correttamente la media mantenendo sigma invariato, ma il KL e'
salito a `0.392` e H2 ha perso TO e cicli. Il checkpoint post-H1 e' stato
respinto, non e' stata eseguita una seconda iterazione e il warm start pre-H1
e' rimasto la baseline valida.

## 1. Preflight della varianza

La varianza state-dependent ereditata, con sigma mediana circa `0.75`, causava
clipping e terminazioni pre-TO. Sono state create copie separate del checkpoint
modificando soltanto le due uscite `log-std`, con media bit-identica.

| Sigma | Risultato su seed 123/124/125 | Decisione |
| --- | --- | --- |
| `0.05` | 206-213 step, TO 3/3, cicli 0/3 | FAIL |
| `0.03` | 204/208/228 step, un ciclo, penetrazione 3/3 | FAIL assoluto |
| `0.020/0.015` | 205/205/216 step, cicli 0/3 | FAIL relativo |

Il replay deterministico a `sigma=0.03` ha riprodotto esattamente DAgger r2:
356 step, return `+29.886`, un ciclo e clipping nullo. Il failure stocastico
derivava da una sensibilita' closed-loop progressiva, non da un singolo spike.

Il gate e' stato reso relativo alla baseline deterministica, richiedendo TO su
tre seed, almeno un ciclo, mediana di sopravvivenza almeno pari al 60% di 356
step e clipping mediano inferiore all'1%.

## 2. Recovery noise-aware e soglia selezionata

Una recovery distillation su stati perturbati ha migliorato la loss offline ma
ha spostato la media nominale e fatto terminare il rollout deterministico a 59
step. Il candidato e' stato respinto: un altro fit supervisionato non era un
criterio affidabile per questa dinamica.

La ricerca della soglia sul seed severo 123 ha dato:

```text
sigma 0.010      203 step, nessun ciclo
sigma 0.005      206 step, nessun ciclo
sigma 0.003      361 step, un ciclo
sigma 0.001      360 step, un ciclo
sigma 0.0001     359 step, un ciclo
```

E' stato selezionato `sigma=0.003`, il massimo valore osservato sopra la
regione instabile. I tre rollout training-like hanno prodotto:

| Seed | Step | Return | HS/TO/cicli | Clip |
| ---: | ---: | ---: | ---: | ---: |
| 123 | 361 | +31.092 | 2/2/1 | 0% |
| 124 | 360 | +31.496 | 2/2/1 | 0% |
| 125 | 210 | -17.892 | 1/1/0 | 0% |

Il gate relativo e' passato: TO `3/3`, cicli `2/3`, mediana 360 step e media
deterministica invariata.

Checkpoint selezionato per H1:

```text
Trajectory Generator/runs/training/
target_domain_warm_start_r2_sigma0003_2026-07-12/rl_module_warm_start/
actor digest: 7fa7fbd15db67dd29b9cd428528b68e14648e934f70c165d6d6be901f025383c
```

## 3. Teacher e struttura temporale del rumore

I test discriminanti hanno separato fragilita' dell'actor e limite del sistema:

| Teacher, rumore bianco | Step | Cicli | Esito |
| --- | ---: | ---: | --- |
| `sigma=0.003` | 500 | 2 | PASS |
| `sigma=0.005` | 500 | 2 | PASS |
| `sigma=0.010` | 388 | 1 | FAIL swing timeout |

Il teacher ha quindi dimostrato che reward, FSM, dinamica e soglia da 25 mm
sono compatibili con `sigma=0.005`. L'actor, invece, falliva a quel livello con
rumore bianco, held 50 ms e held 100 ms.

Il rumore held per 50 ms ha recuperato il seed fragile a `sigma=0.003` da 210 a
359 step e un ciclo, ma non ha risolto `sigma=0.005`. La correlazione temporale
era quindi una componente del problema, non la causa completa. La modalita'
held e' rimasta esclusivamente diagnostica per non invalidare log-probability e
ratio di PPO.

## 4. Trainer H1 readiness

E' stato implementato `freeze_logstd`: durante il backward la media continua a
ricevere gradienti, mentre le componenti `log-std` sono staccate dal grafo.

Il trainer a zero iterazioni con 13 EnvRunner ha verificato:

```text
actor learner/14 EnvRunner/export     bit-exact
critic                                invariato
optimizer source                      non importato
gradiente massimo mean                4.0
gradiente massimo log-std             0.0
stato                                 READY_FOR_H1
```

## 5. H1 singola e H2

E' stata eseguita una sola iterazione PPO con batch 4096, 10 epoch, LR `1e-4`,
`sigma=0.003` e `freeze_logstd=true`.

```text
return medio batch                  -16.972
lunghezza media                     210.778
mean KL                             0.391552
KL target                           0.010000
spostamento massimo media           0.007263
RMSE media                          0.002119
log-std                             bit-identico
```

Lo spostamento della media era piccolo in valore assoluto ma pari a circa
`2.42 sigma`; dieci epoch hanno portato l'update molto oltre il trust region.

H2 deterministico valido, eseguito serialmente dopo aver scartato tentativi
paralleli saturati da OpenSim:

```text
                         pre-H1             post-H1
step                     356                221
return                   +29.886            -59.689
HS / TO / cicli          2 / 2 / 1          1 / 0 / 0
fine                     penetrazione       stance timeout
```

Anche i tre rollout stocastici sono regrediti: mediana `360 -> 214` step,
TO `3/3 -> 2/3` e cicli `2/3 -> 0/3`. Il probe post-H1 a `sigma=0.005` non ha
mostrato recupero.

Decisione finale:

```text
checkpoint H1                    REJECT_H1
iterazione 2                     non eseguita
training lungo                   bloccato
baseline conservata              warm start pre-H1 sigma=0.003
```

## File modificati

- `Trajectory Generator/baseline_MLP/asymmetric_rl_module.py`
- `Trajectory Generator/baseline_MLP/configure_actor_exploration.py`
- `Trajectory Generator/baseline_MLP/exploration_noise.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/target_domain_imitation.py`
- `Trajectory Generator/baseline_MLP/target_domain_noise_adaptation.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `validation/analyze_exploration_divergence.py`
- `validation/evaluate_relative_h1_gate.py`
- `validation/verify_h1_readiness.py`
- test focalizzati per exploration, rollout, RLModule e preflight.

## Test e verifiche

- suite diagnostica rumore: `50/50` PASS;
- `py_compile`: PASS;
- `git diff --check`: PASS;
- save/reload di tutti i candidati sigma: bit-exact;
- media DAgger r2 preservata prima di H1: exact;
- teacher noise sweep: completato;
- trainer zero-iteration e gradient audit: PASS;
- una sola H1: completata tecnicamente;
- H2 deterministico e stocastico: eseguiti, gate comportamentale FAIL;
- nessun checkpoint post-H1 promosso.

## TODO aperti e propagati

- [x] Eseguire H1/H2 warm-start prima di un confronto lungo contro la fresh
      policy. Il checkpoint e' stato respinto gia' dopo il primo update; il
      confronto a 10 iterazioni e pari budget e' quindi superato e non va
      eseguito sul candidato regressivo.
- [ ] Eseguire un warm-up critic-only mantenendo actor e `log-std`
      bit-identici.
- [ ] Ripetere il primo update actor con learning rate/epoch ridotti, budget KL
      verificabile e rollback automatico su H2.
- [ ] Rendere l'actor robusto almeno a `sigma=0.005`, livello sostenuto dal
      teacher, senza abbassare ancora sigma o modificare reward e soglia.
- [ ] Conservare il warm start pre-H1 come baseline immutabile; non ripetere la
      H1 a `1e-4` per 10 epoch.
- [ ] Richiedere al prossimo H2 full episode, cicli validi, hard guard superato
      e una verifica esplicita del picco reserve.
- [ ] Estendere successivamente i gate a start e trial differenti.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo, TODO ancora propagato.
- [ ] Valutare stato Markov/memoria solo con test di osservabilita' e
      deployability, senza dati prescribed nell'actor.
