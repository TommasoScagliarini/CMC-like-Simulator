# FSM ledger/clawback reward per HS-TO-HS

Data: 2026-07-01

## Problema

Il training ex-novo da 100 iterazioni con FSM protesica ha confermato che la
policy non chiude un gait cycle completo. Il rollout del best checkpoint segue
una sequenza stabile:

```text
HS protesico valido -> TO protesico valido -> swing prolungato -> phase_timeout:swing
```

La FSM si comportava correttamente: rilevava un solo `HS`, un solo `TO`, nessun
ciclo completo e terminava per timeout swing. Il problema era nella reward: la
sequenza parziale `HS -> TO` continuava a ricevere credito tramite
`phase_event_progress_score = 0.5` e `phase_regular_score` positivo anche senza
secondo HS. In questo modo il mezzo ciclo rimaneva una strategia redditizia.

## Soluzione

E' stato introdotto un ledger per ciclo nella FSM:

```text
HS valido iniziale    -> event credit + pending credit
TO valido             -> event credit + pending credit
HS successivo valido  -> ciclo completato, bonus ciclo, nuovo ciclo iniziato
timeout / invalid     -> clawback del pending credit + extra failure penalty
```

La ricompensa di progresso fase e' ora evento-only, non piu' un premio
persistente per ogni step nello stato `SWING_AFTER_TO`. Il credito persistente
`phase_cycle_progress_credit` resta disponibile come feature/diagnostica, ma il
reward usa `phase_event_progress_score` come evento discreto.

La configurazione ex-novo usa:

```yaml
phase_hs_event_credit: 0.10
phase_to_event_credit: 0.20
phase_cycle_complete_bonus: 0.70
phase_failure_extra_penalty: 0.05
blend_phase_event_progress: 1.00
phase_clawback_penalty_weight: 1.00
```

Inoltre `phase_regular_score` ora viene attivato solo dopo almeno un ciclo
completo, evitando che un ciclo parziale riceva reward di regolarita'.

## Strategia

La correzione mantiene reward shaping denso ma reversibile:

- `HS` e `TO` restano segnali positivi per guidare l'apprendimento;
- se il ciclo corrente fallisce, il credito accumulato viene sottratto;
- il mezzo ciclo fallito diventa leggermente negativo lato FSM;
- un ciclo completo `HS -> TO -> HS` resta chiaramente piu' vantaggioso;
- il primo HS del ciclo successivo e' anche l'HS finale del ciclo precedente.

Questo implementa la gerarchia desiderata:

```text
0 cicli completi < mezzo ciclo fallito < 1 ciclo completo < 2 cicli completi
```

con la precisazione che `mezzo ciclo + timeout` non deve piu' essere una
strategia redditizia.

## File modificati

- `Trajectory Generator/prosthetic_phase_fsm.py`
  - aggiunti `pending_cycle_credit`, `phase_event_progress_score`,
    `phase_cycle_complete_bonus`, `phase_clawback_penalty`,
    `phase_failure_extra_penalty`, `phase_cycle_failed_this_step`;
  - aggiunti parametri `hs_event_credit`, `toe_off_event_credit`,
    `cycle_complete_bonus`, `failure_extra_penalty`;
  - applicato clawback su timeout o invalid event del ciclo corrente.

- `Trajectory Generator/osim_trj_cmc_like.py`
  - propagati i nuovi parametri dalla config env alla FSM;
  - aggiunti i nuovi campi FSM nei `reward_terms`.

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - aggiunti campi `RewardConfig` per ledger/clawback;
  - sottratto `phase_clawback_penalty_term` post-clip;
  - `phase_regular_score` richiede ora `valid_cycle_count > 0`.

- `Trajectory Generator/baseline_MLP/env_factory.py`
  - propagati i nuovi campi reward verso `CMCEnvConfig`.

- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - aggiunti i nuovi termini FSM al logging TensorBoard/diagnostico.

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - configurati crediti, bonus, extra penalty e peso clawback.

- `validation/test_reward_function.py`
  - aggiunti test su event credit one-shot, clawback e gating di
    `phase_regular_score`.

- `validation/validate_training_config.py`
  - aggiunti check sui nuovi campi FSM ledger/clawback nella config ex-novo.

## Test e verifiche

Verifiche eseguite:

```text
python3 -m py_compile ...
python3 validation/validate_training_config.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
CMC_FULL_FSM_PRESCRIBED_TEST=1 /opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
git diff --check ...
```

E' stato inoltre rivalutato il vecchio checkpoint 100iter con la reward nuova:

```text
Trajectory Generator/runs/rollout/
MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout_ledger_eval_07-01-2026
```

Risultato del rollout rivalutato:

```text
steps: 140
terminated: true
truncated: false
end_reason: phase_timeout:swing
episode_return: 15.9950
reward_mean: 0.11425
```

Confronto col rollout precedente dello stesso checkpoint:

```text
prima ledger/clawback:  episode_return = 43.4747
dopo ledger/clawback:   episode_return = 15.9950
```

Il comportamento dinamico della vecchia policy non cambia, come atteso, ma il
failure mode `HS -> TO -> timeout` non riceve piu' il vecchio premio FSM
persistente.

Dettaglio verificato nel trace rivalutato:

```text
HS valido:       phase_event_progress_score = 0.10, pending = 0.10
TO valido:       phase_event_progress_score = 0.20, pending = 0.30
timeout swing:   phase_clawback_penalty = 0.30
                 phase_failure_extra_penalty = 0.05
                 phase_regular_score = 0.0
```

## Stato finale

La modifica risolve l'exploit reward del mezzo ciclo: la sequenza incompleta
`HS -> TO -> timeout` non viene piu' premiata come prima. La verifica sul vecchio
checkpoint dimostra che il ledger/clawback viene applicato correttamente.

Resta da verificare con un nuovo training se la policy impara effettivamente a
chiudere cicli `HS -> TO -> HS` usando la reward aggiornata.

## TODO

- Lanciare un nuovo training ex-novo con `training_exnovo_cfg.yaml` aggiornata.
- Eseguire rollout del best checkpoint e verificare:
  - `phase_valid_cycle_count > 0`;
  - assenza o forte riduzione di `phase_timeout:swing`;
  - comportamento dei nuovi termini `phase_clawback_penalty`,
    `phase_cycle_complete_bonus`, `phase_pending_cycle_credit`;
  - confronto return/episode length rispetto al rollout ledger-eval.
- Se la policy continua a non chiudere il ciclo, valutare una penalita' densa
  nella landing window per assenza di contatto, mantenendo il ledger per ciclo.
