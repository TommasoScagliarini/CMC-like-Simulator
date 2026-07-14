# 2026-06-19 - Confronto sym60 vs asym100 sulla tau_spring ankle

## Problema

Dopo il rollout del training simmetrico a 60 iterazioni:

`Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`

e la generazione dei plot in:

`plot/06_19_2026_2 - imitTraining_sym_60`

e emersa una domanda specifica: perche la `tau_spring` dell'ankle nel training
simmetrico a 60 iterazioni risulta visivamente piu simile ai risultati del
05_23, mentre il training asimmetrico actor-critic a 100 iterazioni mostra una
forma ankle molto diversa, nonostante le cinematiche served sembrino simili?

La differenza non e altrettanto marcata sulla coppia del ginocchio.

## Soluzione / diagnosi

La spiegazione principale e che la posizione served da sola non determina la
`tau_spring`. La catena effettiva e:

```text
served q_ref, qdot_ref
  -> outer controller cascade
  -> comando normalizzato u
  -> dinamica motore SEA
  -> deflessione theta_m - theta_j
  -> tau_spring
```

Nel controller protesico la modalita attiva e `cascade`:

```text
qdot_cas = qdot_ref + Kp_outer * (q_ref - q)
tau_cmd  = Kp_inner * (qdot_cas - qdot)
         + Ki_inner * integral(qdot_cas - qdot)
u        = tau_cmd / F_opt
```

Per ankle i parametri sono:

```text
Kp_outer = 47.125
Kp_inner = 2.8275
Ki_inner = 213.0
F_opt    = 250 Nm
```

Quindi l'ankle e particolarmente sensibile a:

- piccole differenze di fase tra riferimento served e stato reale;
- differenze nella velocita served;
- differenze nell'accelerazione/curvatura del riferimento;
- memoria integrale del PI interno del cascade.

Anche se la posizione ankle served e molto simile tra `sym60` e `asym100`, il
comando SEA ankle non lo e.

## Verifiche numeriche

Confronto su finestra comune 13-18 s:

```text
ankle position corr      ~0.954
ankle velocity corr      ~0.795
ankle acceleration corr  ~0.355
ankle control u corr     ~0.118
ankle tau_spring corr    ~0.136
```

Quindi:

- la posizione e realmente simile;
- la velocita e solo moderatamente simile;
- accelerazione, comando `u` e `tau_spring` divergono molto.

Esempio a circa `t = 15.98 s`:

```text
sym60:
  q      = +0.260 rad
  qdot   = +0.506 rad/s
  u      = -0.033
  tau    = -8.3 Nm

asym100:
  q      = +0.211 rad
  qdot   = +0.735 rad/s
  u      = +0.086
  tau    = +21.5 Nm
```

Le posizioni sono vicine, ma il comando cambia segno. In quella regione
`asym100` accumula una memoria integrale positiva nel cascade, che produce i
picchi positivi di ankle torque. `sym60`, invece, mantiene un comando piu
negativo o vicino a zero, producendo una morfologia ankle piu simile al 05_23.

## Interpretazione

Il training `sym60` non e globalmente migliore di `asym100` come tracking o
reserve, ma sull'ankle genera una forma di `tau_spring` piu plausibile perche:

- evita i grandi picchi positivi visibili in `asym100`;
- mantiene una componente negativa piu ripetuta e phase-locked;
- produce una forma piu vicina al 05_23, almeno qualitativamente.

La somiglianza con il 05_23 resta pero soprattutto di forma, non di scala:

```text
sym60 ankle tau_spring:
  RMS circa 6.16 Nm
  minimo circa -21.8 Nm

05_23 ankle tau_spring:
  RMS circa 48.1 Nm
  minimo circa -120.5 Nm
```

Quindi `sym60` recupera meglio la direzione/timing della coppia ankle, ma non
recupera l'ampiezza biomeccanica del 05_23.

## Perche il ginocchio cambia meno

La differenza sulla knee torque e meno sorprendente perche:

- le cinematiche knee dei due rollout sono piu coerenti anche in velocita;
- il ginocchio non lavora cosi vicino al cambio di segno della coppia;
- il comando knee tra i due run resta piu correlato rispetto all'ankle;
- la dinamica del ginocchio sembra meno dominata da piccoli sfasamenti locali.

In breve: l'ankle e il grado di liberta fragile, dove piccole differenze
cinematiche e integrali cambiano qualitativamente il segno della coppia.

## File modificati e output

Codice sorgente modificato per questa analisi: nessuno.

Output generati/consultati:

- `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/`
- `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/rollout_summary.json`
- `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/rollout_policy_trace.json`
- `plot/06_19_2026_2 - imitTraining_sym_60/01_time_sea_control_reserve.png`
- `plot/06_18_2026_2_imitTraining_asymActCrit_100/01_time_sea_control_reserve.png`
- `plot/05_23_2026_2/01_time_sea_control_reserve.png`

File creato:

- `reports/user/2026-06-19_confronto_sym60_asym100_tau_spring_ankle.md`

Nota: nel worktree risultano gia presenti modifiche non legate direttamente a
questa analisi, tra cui `Trajectory Generator/baseline_MLP/rollout_eval.py` e
file di configurazione/registry. Non sono state revertite.

## Test / verifiche eseguite

- Rollout del checkpoint `rl_module_best` del training simmetrico 60 iterazioni.
- Plotter MLP eseguito e cartella rinominata con suffisso `- imitTraining_sym_60`.
- Verificato `missing_channels.txt`: nessun canale mancante.
- Confronto visivo dei plot:
  - `sym60`
  - `asym100`
  - `05_23`
- Confronto numerico tra `sym60` e `asym100` su:
  - posizione ankle/knee;
  - velocita ankle/knee;
  - accelerazione ankle/knee;
  - comando SEA `u`;
  - `tau_spring`;
  - errore cascade ricostruito dal trace.

## TODO

- Aggiungere ai log o ai plot diagnostici i termini interni del controller
  cascade: `e_q`, `e_qdot`, `qdot_cas`, `cascade_velocity_error`, contributo P
  e contributo I. Questo renderebbe immediata la diagnosi dei cambi di segno
  della `tau_spring`.
- Nel prossimo confronto training, non valutare solo la posizione served:
  includere sempre anche velocita, accelerazione, comando `u` e `tau_spring`.
- Prima di introdurre nuove penalty fisiche pesanti, chiarire se la priorita e
  migliorare la forma della coppia ankle, ridurre le reserve globali, o
  mantenere il tracking imitativo verso il target sound-leg corrente.
