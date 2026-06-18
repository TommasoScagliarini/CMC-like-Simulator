# Analisi rollout FullObs normReward 60iter - 2026-06-17

## Problema

E' stato analizzato il rollout del training imitativo baseline MLP `40+20`
iterazioni, cioe' la lineage:

- `baseline_mlp_imit_v4_c2_4hz_obs_target`;
- `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`.

Il rollout e i plot analizzati sono:

```text
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout
plot/06_17_2026_1 - imititation_FullObs_normReward_60iter
```

Le domande principali erano:

- capire se l'errore residuo fosse dovuto al tracking SEA o alla reference
  generata/servita dalla policy;
- quantificare la qualita' del critico PPO (`vf_loss`, `vf_loss_unclipped`,
  `vf_explained_var`);
- spiegare perche' i grafici torque/angle della figura 3 hanno una forma non
  biomeccanicamente corretta;
- definire un confronto sperimentale successivo fra baseline simmetrica e
  asymmetric actor-critic.

## Soluzione / risultato dell'analisi

Il rollout e' stabile e completa l'episodio da `5 s`:

```text
steps = 501
terminated = false
truncated = true
episode_return = 334.7528621460614
action_clipped_fraction = 0.0
pelvis_ty_min = 0.9518210480919542 m
```

Il tracking fisico del SEA rispetto alla reference servita e' molto buono:

```text
knee actual-served RMSE  = 0.00336 rad
ankle actual-served RMSE = 0.00545 rad
```

L'errore principale resta quindi a monte, nella forma della reference
generata/servita dalla policy rispetto al target sano anti-fase:

```text
knee served-target RMSE  = 0.119 rad
ankle served-target RMSE = 0.0449 rad
knee served-target corr  = 0.978
ankle served-target corr = 0.944
```

Rispetto alla diagnosi precedente, l'ankle non e' piu chiaramente compresso
verso la media: il range served e' anzi piu ampio del target.

```text
ankle target span = 0.355 rad
ankle served span = 0.442 rad
```

Il knee invece mantiene offset/compressione:

```text
knee target span = 0.933 rad
knee served span = 0.871 rad
knee mean served-target error ~= -0.073 rad
```

## Critico PPO

Dal training `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`, i
valori principali sono:

```text
best iteration = 59
best return    = 277.75698812995

iter 59:
  vf_loss clipped    = 2.135979175567627
  vf_loss_unclipped  = 29.469728469848633
  vf_explained_var   = 0.10315835475921631

iter 60:
  vf_loss clipped    = 2.7540037631988525
  vf_loss_unclipped  = 54.7823600769043
  vf_explained_var   = 0.113750159740448

last 5 mean:
  vf_loss clipped    = 2.5935944080352784
  vf_loss_unclipped  = 41.98524894714355
  vf_explained_var   = 0.08470591306686401
```

La configurazione usa:

```yaml
ppo:
  vf_clip_param: 10.0
  vf_loss_coeff: 1.0
```

Nella versione RLlib installata, `vf_loss` e' la squared error value clampata
tra `0` e `vf_clip_param`, mentre `vf_loss_unclipped` e' la MSE raw:

```python
vf_loss = (value_fn_out - value_targets) ** 2
vf_loss_clipped = clamp(vf_loss, 0, vf_clip_param)
```

Quindi, con `vf_clip_param = 10`, ogni errore value con modulo superiore a
`sqrt(10) ~= 3.16` viene cappato nella loss ottimizzata. Il gap fra
`vf_loss` e `vf_loss_unclipped` indica che il critic ha ancora errori raw
consistenti e che il clipping nasconde una parte importante della MSE.

Interpretazione:

- il critic non e' completamente inutile, ma resta debole;
- `vf_explained_var ~0.10` significa che spiega solo circa il 10% della varianza
  dei value target;
- alzare solo `vf_clip_param` non basta necessariamente, perche' puo aumentare
  il gradiente sugli errori grossi ma non risolve mancanza di informazione o
  target rumorosi;
- l'asymmetric actor-critic resta una prova piu pulita, per dare al critic uno
  stato privilegiato senza cambiare l'osservazione realistica dell'actor.

## Diagnosi dei grafici torque/angle

La figura:

```text
plot/06_17_2026_1 - imititation_FullObs_normReward_60iter/03_gaitcycle_torque_angle_power.png
```

non va letta come curva biomeccanica healthy corretta. Il plotter usa:

- `SEA_*_tau_spring` come coppia SEA al giunto;
- `pros_*_angle_q` come angolo protesico;
- flip di segno per il knee tramite `apply_joint_sign`;
- i cicli gait sinistri dal file `rollout_episode_gait_events.csv`.

Nel rollout entrano solo due cicli sinistri completi:

```text
13.94687098 -> 15.61596084 s
15.61596084 -> 17.16396799 s
```

La forma non corretta nasce soprattutto da tre fattori:

1. La policy e' ottimizzata per imitation cinematica e tracking della served
   reference, non per produrre un loop momento-angolo fisiologico.
2. Il grafico torque/angle e' parametrico nel tempo: allo stesso angolo possono
   corrispondere coppie diverse in stance e swing, quindi il loop si incrocia.
3. Il plotter media `angle(gait%)` e `torque(gait%)` e poi disegna
   `mean_torque` contro `mean_angle`; con x non monotona, anche la banda
   `fill_between` puo diventare visivamente fuorviante.

Numeri sui due cicli usati:

```text
ankle:
  mean angle range  ~= 0.057 -> 0.473 rad
  mean torque range ~= -13.04 -> 1.67 Nm

knee:
  mean angle range  ~= 0.331 -> 1.060 rad
  mean torque range ~= -18.55 -> 13.95 Nm
```

Il knee e' il piu sospetto: a pari angolo la coppia cambia spesso segno, e la
motor power e' ancora rumorosa nella prima parte del gait cycle. Questo non
sembra un errore del plugin SEA; e' piu coerente con una policy che segue bene
una reference cinematica ma non e' ancora vincolata dinamicamente.

## Strategia consigliata

Per i prossimi esperimenti sono stati proposti due rami separati:

1. training da zero con configurazione attuale piu asymmetric actor-critic,
   target `100` iterazioni;
2. resume della configurazione attuale dal best del run a `80` iterazioni fino
   a `100` iterazioni.

Il confronto deve distinguere:

- miglioramento dovuto a piu budget di training;
- miglioramento dovuto al critic privilegiato.

Metriche da confrontare oltre al return:

```text
vf_explained_var
vf_loss / vf_loss_unclipped
served-target RMSE knee/ankle
actual-served RMSE
knee mean error / offset
action turn fraction
SEA torque error / motor power
reserve/root load
gait-cycle torque-angle shape
```

E' stato suggerito di non cambiare `vf_clip_param` nello stesso primo test
asymmetric. Una eventuale ablation su `vf_clip_param = 50` o `100` va fatta
separatamente.

## File modificati

Nessun file di codice e' stato modificato durante questa analisi.

File creato:

```text
reports/user/2026-06-17_analisi_rollout_fullobs_normreward_60iter.md
```

## File e artefatti analizzati

```text
Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/summary.json
Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/train_iterations.jsonl
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout/rollout_summary.json
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout/rollout_policy_trace.json
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout/rollout_reset_diagnostics.json
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout/sim_outputs/*.sto
plot/06_17_2026_1 - imititation_FullObs_normReward_60iter/*.png
plot/plotter.py
```

## Verifiche eseguite

- Lettura di `rollout_summary.json` e `rollout_reset_diagnostics.json`.
- Analisi visuale delle figure `01`-`07`, con focus su figure `03`, `06` e `07`.
- Calcolo da `rollout_policy_trace.json` di:
  - RMSE served-target;
  - RMSE actual-served;
  - range target/served/actual;
  - correlazioni target-policy;
  - action turn fraction;
  - componenti reward e diagnostiche reference governor.
- Calcolo da `.sto` di:
  - range e RMS di `tau_ref`, `tau_spring`, `tau_error`, `tau_input`;
  - saturazione SEA;
  - motor speed, motor acceleration e motor power;
  - reserve/root load;
  - recruitment e GRF online.
- Verifica del codice RLlib locale per confermare la semantica di
  `vf_loss` e `vf_loss_unclipped`.
- Verifica del codice `plot/plotter.py` per confermare quali segnali entrano
  nella figura torque/angle.

## TODO

- [ ] Eseguire un training asymmetric actor-critic da zero a `100` iterazioni,
      mantenendo invariati reward e `vf_clip_param`.
- [ ] Eseguire il resume della baseline simmetrica dal best del run a `80`
      iterazioni fino a `100` iterazioni.
- [ ] Confrontare i due rami con rollout deterministici usando le metriche sopra,
      non solo il return.
- [ ] Creare o correggere una figura torque/angle piu robusta: cicli individuali,
      colore per gait percentage, niente `fill_between` su x non monotona.
- [ ] Valutare una ablation separata su `vf_clip_param` solo dopo il confronto
      asymmetric vs baseline estesa.
