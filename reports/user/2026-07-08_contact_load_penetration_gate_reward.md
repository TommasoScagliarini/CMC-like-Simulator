# 2026-07-08 - Gate premio contatto su penetrazione GRF

## Problema

Nel rollout H2 precedente la reward ex-novo permetteva ancora una scappatoia:
durante `STANCE_AFTER_HS` la protesi restava caricata, la penetrazione GRF
continuava a crescere e `contact_load_score` premiava comunque il carico
verticale finche' la terminazione hard per `grf_penetration` chiudeva
l'episodio.

La soglia scelta per distinguere contatto ancora credibile da contatto
innaturale e':

- premio pieno fino a `0.010 m` di penetrazione;
- taper lineare tra `0.010 m` e `0.012 m`;
- premio di carico nullo da `0.012 m` in su.

Questa soglia non cambia la fisica del contatto e non alza la terminazione: serve
solo a impedire che la reward paghi un carico ottenuto penetrando il terreno.

## Soluzione implementata

In `RewardConfig` sono stati aggiunti:

```yaml
contact_load_penetration_full_reward_m: 0.010
contact_load_penetration_zero_reward_m: 0.012
```

In `_task_reward_terms` ora viene calcolata una qualita' di contatto:

```text
penetration <= 10 mm  -> quality = 1
10-12 mm              -> quality lineare 1 -> 0
penetration >= 12 mm  -> quality = 0
```

Lo score usato dalla reward diventa:

```text
contact_load_score = contact_load_raw_score * contact_load_penetration_quality
```

Nel trace sono stati aggiunti anche i diagnostici:

- `contact_load_raw_score`
- `contact_load_penetration_quality`
- `contact_load_penetration_m`
- `contact_load_penetration_full_reward_m`
- `contact_load_penetration_zero_reward_m`

## File modificati

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `validation/test_reward_function.py`
- `reports/plans/2026-07-02_piano_validazione_reward_exnovo.md`

Non sono stati modificati il plugin C++ SEA, la semantica del comando SEA o il
modello OpenSim.

## Verifiche rapide

Comandi eseguiti:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python -m py_compile "Trajectory Generator/baseline_MLP/reward_function.py"
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/validate_training_config.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
git diff --check
```

Esito:

- `validate_training_config.py`: PASS
- `test_reward_function.py`: PASS, 27 test
- `py_compile`: PASS
- `git diff --check`: PASS

## H1 - Training 10 iterazioni

Run:

```text
Trajectory Generator/runs/training/2026-07-08_H1_contact_penetration_gate/
```

Comando:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" --output-dir "Trajectory Generator/runs/training/2026-07-08_H1_contact_penetration_gate" --iterations 10
```

Risultato:

```text
ok: true
iterations_completed: 10
best_episode_return_mean: 0.14104304784150673
best iteration: 10
episode_len_mean: 31.51 -> 33.40 step
```

Serie sintetica:

```text
iter 01: return=-0.063335, len=31.510
iter 02: return=-0.049566, len=31.375
iter 03: return=-0.030825, len=31.663
iter 04: return=-0.013370, len=31.683
iter 05: return=-0.019110, len=31.933
iter 06: return=-0.002753, len=32.173
iter 07: return= 0.031874, len=32.433
iter 08: return= 0.109190, len=32.615
iter 09: return= 0.137758, len=33.192
iter 10: return= 0.141043, len=33.404
```

Interpretazione: il training e' tecnicamente valido e il return diventa
positivo, ma la lunghezza episodio resta corta. Le terminazioni per
`grf_penetration` restano dominanti, quindi H1 non basta a sbloccare il training
lungo.

## H2 - Rollout del best checkpoint H1

Run:

```text
Trajectory Generator/runs/rollout/2026-07-08_H2_contact_penetration_gate_best_recorded/
```

Comando:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/rollout_eval.py" --checkpoint "Trajectory Generator/runs/training/2026-07-08_H1_contact_penetration_gate/rl_module_best" --output-dir "Trajectory Generator/runs/rollout/2026-07-08_H2_contact_penetration_gate_best_recorded" --record-outputs --no-progress --startup-timeout-s 600 --step-timeout-s 180 --stall-timeout-s 300 --run-timeout-s 3600
```

Risultato:

```text
ok: true
steps: 36
episode_return: 0.8193196165167906
reward_mean: 0.022758878236577518
reward_min: -2.6110670050242133
reward_max: 0.2184578247297007
terminated: true
truncated: false
action_clipped_fraction: 0.027777777777777776
pelvis_ty_min: 0.9602308046707383
```

Eventi online:

```text
13.946870983805102 -> left heel_strike
13.946870983805102 -> right heel_strike
14.200870983804961 -> right toe_off
```

Non compare nessun `toe_off` left/protesico.

Stato FSM finale:

```text
phase_fsm_state_id: 1  # STANCE_AFTER_HS
phase_valid_hs_count: 1
phase_valid_to_count: 0
phase_valid_cycle_count: 0
phase_stance_elapsed_s: 0.36
```

## Verifica della nuova logica reward in H2

La modifica funziona come previsto:

```text
step 28, pen=0.009196 m, raw=0.607597, quality=1.000000, score=0.607597, reward=0.210050
step 29, pen=0.009952 m, raw=0.631894, quality=1.000000, score=0.631894, reward=0.218458
step 30, pen=0.010757 m, raw=0.656155, quality=0.621696, score=0.407929, reward=0.139972
step 31, pen=0.011870 m, raw=0.679883, quality=0.064904, score=0.044127, reward=0.012530
step 32, pen=0.013007 m, raw=0.702635, quality=0.000000, score=0.000000, reward=-0.020291
```

All'ultimo step:

```text
penetration: 0.0175275 m
contact_load_raw_score: 0.7791
contact_load_penetration_quality: 0.0
contact_load_score: 0.0
reward_base: 0.0
safety_term: 2.0
grf_penetration_term: 0.6111
reward: -2.6111
```

Quindi il carico penetrato non genera piu' premio. La reward non sta piu'
pagando la scappatoia; il fallimento residuo e' nel comportamento della policy,
che resta in stance compressiva e non scarica il lato protesico prima della
terminazione per penetrazione.

## Conclusione

La logica richiesta e' implementata e validata sui trace H2: sopra `12 mm` il
premio di contatto viene spento anche se il carico verticale aumenta. Questo
risolve la scappatoia reward identificata.

La reward attuale pero' non e' ancora pronta per procedere a training 20-50 o
100 iterazioni: H2 resta FAIL perche' non compare la transizione
`left HS -> left TO` e la policy termina ancora per `grf_penetration`.

## TODO

- [ ] Prima di aumentare il budget di training, aggiungere un incentivo/penalita'
      che favorisca lo scarico protesico e il TO sinistro dopo una stance minima
      valida, senza reintrodurre target prescribed.
- [ ] Ripetere H1/H2 dopo questa modifica e richiedere come gate minimo almeno
      `phase_valid_hs_count >= 1`, `phase_valid_to_count >= 1`, nessuna
      terminazione precoce per `grf_penetration` prima del TO.
- [ ] Mantenere bloccati training 20-50/100 iterazioni finche' H2 non supera il
      gate comportamentale.
