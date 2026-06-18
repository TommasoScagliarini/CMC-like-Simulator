# Reward modulare e coppia SEA nel training MLP

Data: 2026-06-18

## Problema

Nel confronto tra:

- `plot/06_17_2026_1 - imititation_FullObs_normReward_60iter/01_time_sea_control_reserve.png`
- `plot/06_18_2026_1 - imititation_FullObs_normReward_100iter/01_time_sea_control_reserve.png`

e' emerso che il training a `100` iterazioni migliora diversi grafici
cinematici e il return, ma degrada la forma della coppia SEA, soprattutto
all'ankle.

Il dubbio principale era capire come fosse possibile ottenere una traiettoria
cinematica piu vicina al target sano anti-fase ma una curva di coppia meno
simile a quella prodotta dal CMC-like sim puro.

## Diagnosi

La differenza e' reale nei dati, non solo visiva.

Metriche principali dal rollout:

```text
60 iter:
  return = 334.7528621460614
  action_clipped_fraction = 0.0
  ankle served-target RMSE = 0.04486 rad
  ankle tau_spring min/max = -21.78 / +1.76 Nm
  ankle tau_spring area positiva/negativa ~= +1.48 / -18.03 Nms

100 iter:
  return = 369.8268926867439
  action_clipped_fraction = 0.07285
  ankle served-target RMSE = 0.02632 rad
  ankle tau_spring min/max = -19.91 / +43.13 Nm
  ankle tau_spring area positiva/negativa ~= +14.90 / -7.32 Nms
```

Il `100iter` migliora quindi la cinematica ankle rispetto al target, ma cambia
la strategia dinamica. Il picco ankle positivo attorno a `t ~= 15.974 s` non
deriva da un errore del tracking SEA o da una reserve, ma dalla traiettoria
servita al controller protesico.

Nel punto di picco:

```text
SEA_Ankle_tau_spring ~= 43.13 Nm
SEA_Ankle_tau_ref    ~= 42.97 Nm
outer_p_cmd          ~= 0.51 Nm
outer_i_cmd          ~= 42.46 Nm
```

Questo mostra che il training piu lungo ha trovato una soluzione cinematica che
fa accumulare una dinamica di controllo molto diversa, senza che la reward la
scoraggi abbastanza.

## Interpretazione

L'ipotesi iniziale era:

```text
se la rete genera una buona traiettoria cinematica,
allora il plant produrra anche una buona curva di coppia.
```

Questa ipotesi non regge in modo generale. Una traiettoria `q(t)` simile puo
essere realizzata con coppie molto diverse, per effetto di:

- fase e velocita articolari;
- contatto e GRF;
- dinamica multibody;
- reference governor;
- plant SEA;
- controller intermedio.

Quindi la cinematica non identifica in modo univoco la dinamica.

## Vincolo progettuale

Non si vuole inserire nella reward un termine controller-specifico, per
preservare la modularita fra i tre stadi:

```text
rete MLP -> reference cinematica
outer/prosthesis controller -> tracking della reference
SEA/plant -> risposta fisica
```

Sono quindi da evitare termini di reward basati su dettagli interni del
controller, come:

```text
outer_i_cmd
cascade_xi
stati integrali del PI
termini P/I/D specifici
```

Sono anche da evitare termini che facciano imitare direttamente la coppia
prescribed protesica:

```text
(tau_spring - tau_prescribed_prosthetic)^2
```

perche' trasformerebbero il problema in imitazione dinamica supervised e
potrebbero far "barare" la rete.

## Soluzione proposta

La direzione corretta e' usare termini fisici o cinematici
controller-agnostic, cioe' grandezze che non dipendono dalla struttura del
controller intermedio.

Termini accettabili sulla reference generata dalla rete:

- velocita, accelerazione e jerk della reference;
- smoothness inter-step;
- banda/frequenza massima della traiettoria;
- limiti articolari e out-of-band;
- periodicita e coerenza di fase.

Termini accettabili sulla risposta fisica del plant:

- costo di sforzo `tau_spring_rms`;
- costo di variazione `d(tau_spring)/dt`;
- potenza o lavoro SEA;
- saturazione fisica;
- motor speed/accel se interpretati come limiti fisici dell'attuatore, non come
  struttura di controllo.

La distinzione progettuale e':

```text
NO: tau_spring deve assomigliare alla curva prescribed.
SI: tau_spring non deve diventare impulsiva, enorme, rumorosa o energeticamente assurda.
```

## Strategia consigliata

Per il prossimo ciclo sperimentale, introdurre prima diagnostica e poi reward
opzionale con pesi bassi:

1. Aggiungere metriche controller-agnostic per confrontare i rollout:
   `tau_spring_rms`, `tau_spring_rate_rms`, lavoro/potenza SEA,
   impulso positivo/negativo ankle, action clipping.
2. Valutare simmetrico `100iter` e asymmetric actor-critic `100iter` con le
   stesse metriche fisiche, non solo con return e RMSE cinematico.
3. Solo dopo, se confermato, aggiungere regularizer fisici leggeri alla reward,
   evitando riferimenti a coppie prescribed o stati interni del controller.
4. Mantenere separati logging diagnostico e reward effettiva: i segnali
   controller-specifici possono restare utili per capire il fenomeno, ma non
   devono entrare nell'obiettivo della rete.

## Aggiornamento registry storico

E' stato aggiornato il registry storico dei training includendo:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_asym_actor_critic_100
```

La run risulta:

```text
status = completed
platform = mac
iterations = 1-100
best return = 307.1684731522426 @ iteration 100
asymmetric_actor_critic = true
```

La run e' stata annotata come training da zero e candidato diretto per il
confronto con la baseline simmetrica a `100` iterazioni.

## File modificati

Registry storico:

```text
Trajectory Generator/runs/historical_runs.manual.yaml
Trajectory Generator/runs/historical_runs.md
Trajectory Generator/runs/historical_runs.index.json
```

Report creato:

```text
reports/user/2026-06-18_reward_modulare_e_coppia_sea_mlp.md
```

Non sono stati modificati:

```text
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml
```

## Verifiche eseguite

- Ispezione visuale dei plot `01_time_sea_control_reserve.png` a `60iter` e
  `100iter`.
- Lettura di `rollout_summary.json` per i due rollout.
- Lettura dei file `.sto`:
  - `rollout_episode_sea_torques.sto`;
  - `rollout_episode_sea_controls.sto`;
  - `rollout_episode_sea_diagnostics.sto`;
  - `rollout_episode_kinematics.sto`.
- Lettura di `rollout_policy_trace.json` per confrontare target, served ref e
  stato protesico.
- Calcolo di RMSE served-target, action clipping, range/area di `tau_spring`,
  e contributi diagnostici nel punto di picco ankle.
- Esecuzione di `Trajectory Generator/baseline_MLP/update_historical_runs.py`.
- Verifica che la run asymmetric compaia in Markdown, JSON e manual YAML.
- Verifica che `asymmetric_actor_critic: true` sia registrato nell'indice.
- Verifica assenza di path assoluti macOS/Windows nei registry rigenerati.
- `git diff --check` sui file del registry, con solo warning CRLF/LF atteso su
  Windows.

## TODO

- [ ] Creare una diagnostica comparativa `60iter` vs `100iter` vs asymmetric
      `100iter` con metriche fisiche controller-agnostic.
- [ ] Aggiungere al logging metriche su `tau_spring_rms`, `tau_spring_rate_rms`,
      impulso positivo/negativo ankle, lavoro/potenza SEA e clipping.
- [ ] Valutare regularizer fisici leggeri e opzionali nella reward, evitando
      coppie prescribed e stati interni del controller.
- [ ] Confrontare asymmetric actor-critic `100iter` contro baseline simmetrica
      `100iter` con rollout deterministici e metriche dinamiche, non solo return.
