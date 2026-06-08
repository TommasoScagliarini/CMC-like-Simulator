# Training full-gait diagnostico MLP/RLlib

## Problema

Il training MLP/RLlib validato il 2026-06-07 usava episodi molto brevi da
`0.08 s`. Quel run aveva verificato stabilita della pipeline, timeout, cleanup
Ray e aggiornamenti PPO, ma non permetteva alla policy di osservare heel strike
o gait cycle completi.

Il primo TODO aperto era quindi eseguire un training diagnostico full-gait con
episodi sufficientemente lunghi da includere cicli completi, mantenendo limiti
CPU e timeout rigidi.

## Strategia

Il setup AB06 parte a `11.99 s`. Per includere almeno un ciclo completo per
entrambi i lati e stata scelta una durata episodio di `4.0 s`:

- right prescribed: circa `13.164 -> 14.780 s`;
- left prescribed: circa `13.947 -> 15.616 s`.

Configurazione del training:

```text
output_dir:         runs/_baseline_mlp_fullgait_diag_20260608
iterations:         3
num_env_runners:    1
ray_num_cpus:       2
train_batch_size:   200
minibatch_size:     50
num_epochs:         2
episode_duration:   4.0 s
segment_duration:   0.02 s
network:            64 64
grf_mode:           online_sensor
checkpoint_every:   1
iteration_timeout:  1800 s
run_timeout:        5400 s
```

Sono stati mantenuti i default correnti di reward, action authority,
osservazione onlineGRF e profilo AB06, cosi da misurare la robustezza della
configurazione esistente senza introdurre tuning contestuale.

Dopo il training e stato eseguito un rollout deterministico da `4.0 s` del
checkpoint migliore, con registrazione completa degli output `.sto` e degli
eventi gait:

```text
runs/_baseline_mlp_fullgait_diag_20260608_rollout_best_conda
```

## Soluzione ed esito

### Training full-gait

Il training ha completato tutte le `3/3` iterazioni:

```text
elapsed wall time: 2115.09 s
timeout:           nessuno
processi Ray finali: 0
```

| Iterazione | Return medio | Lunghezza media episodio | Knee divergence cumulative | Full episode cumulative |
|---:|---:|---:|---:|---:|
| 1 | `1.741` | `6.5` step | `2` | `0` |
| 2 | `67.099` | `103.7` step | `4` | `1` |
| 3 | `81.098` | `124.0` step | `5` | `2` |

La configurazione non regge sempre il full-gait: su sette episodi conclusi,
cinque terminano per `joint_divergence_pros_knee_angle`. Tuttavia, due episodi
raggiungono correttamente il limite temporale da `4.0 s`.

Il critic resta debole:

```text
vf_explained_var:
  iter 1: -0.0056
  iter 2: -0.0180
  iter 3: -0.0083
```

La saturazione SEA media osservata durante il training resta alta:

```text
u_saturation_fraction:
  iter 1: 0.205
  iter 2: 0.195
  iter 3: 0.235
```

Le metriche TensorBoard confermano che il training ha osservato heel strike,
toe-off e durate di ciclo non nulle su entrambi i lati.

### Rollout deterministico del checkpoint migliore

Il checkpoint migliore ha completato l'intero episodio:

```text
steps:          201
episode return: 189.209
reward medio:   0.9413
reward minimo:  0.7063
max |action|:   0.3257
pelvis_ty min:  0.9281 m
terminated:     false
truncated:      true
```

La chiusura come `truncated=True` e corretta: l'episodio termina per limite
temporale, non per stato unsafe.

Tutti gli output principali controllati sono finiti, senza `NaN` o `Inf`.

### Gait cycle osservati

Eventi online rilevati nel rollout:

| Lato | Heel strike | Toe-off | Cicli online completi |
|---|---:|---:|---|
| left | `3` | `2` | `1.701 s`, `1.547 s` |
| right | `3` | `3` | `1.156 s`, `1.646 s` |

Il rollout ha quindi osservato almeno un ciclo online completo per lato.

Confronto heel strike online/prescribed, escludendo l'evento iniziale:

```text
right:
  -17.8 ms
  +12.1 ms

left:
  -255.9 ms
  -378.0 ms
```

Il timing destro resta accurato, mentre il lato sinistro mostra uno sfasamento
importante nel rollout prodotto dalla policy.

### Diagnostica SEA

```text
SEA Knee:
  max tau_input_raw:       1393.9 Nm
  campioni saturati:       164 / 4001
  frazione saturazione:    4.10%

SEA Ankle:
  max |tau_input_raw|:     136.5 Nm
  campioni saturati:       0 / 4001
```

Il knee resta il principale limite dinamico. L'ankle non satura nel rollout
deterministico.

### Diagnostica biologica

```text
tau_reserve_norm:
  mean: 210.56
  max:  1612.14

muscle_capable_share:
  mean: 0.813
  min:  0.100

muscle_capable_reserve_norm:
  mean: 60.90
  max:  920.91

equilibrium_failures:
  campioni non-zero: 60 / 4001
```

La simulazione resta finita e completa, ma la parte biologica e sottoposta a
stress elevato. Durante il training sono inoltre comparsi tre warning:

```text
[StaticOptimizer] QP did not converge. Using bounded least-squares fallback.
```

### Contatto online

Nel rollout registrato:

```text
left_in_contact  = 1 per tutti i campioni
right_in_contact = 1 per tutti i campioni
```

Questo avviene anche quando la forza normale diventa quasi nulla. La logica
streaming degli eventi riesce comunque a produrre toe-off e heel strike, ma il
flag raw `in_contact` non e affidabile come osservazione binaria in questo
rollout e richiede approfondimento.

## Problemi operativi incontrati

Il primo lancio del training con `Start-Process` ha spezzato il path dello
script in corrispondenza dello spazio in `Trajectory Generator`. Il launcher e
stato corretto e il training e stato poi eseguito completamente.

La chiamata PowerShell con redirezione dei log ha trattenuto il controllo fino
alla fine del training, impedendo gli aggiornamenti chat ogni cinque minuti
durante il run principale.

Per il rollout e stato inoltre verificato che:

- l'esecuzione diretta di `envCMC-rllib/python.exe` puo terminare nativamente su
  Windows;
- il percorso supportato e robusto resta:

```powershell
conda run -n envCMC-rllib python ...
```

Durante il rollout full-gait gli aggiornamenti ogni cinque minuti sono stati
eseguiti regolarmente.

## File modificati

Nessun file sorgente del simulatore, del plugin SEA o del Trajectory Generator
e stato modificato.

E stato aggiunto solo questo report:

```text
reports/user/2026-06-08_training_full_gait_diagnostico_mlp.md
```

## Artefatti principali

Training:

```text
runs/_baseline_mlp_fullgait_diag_20260608/
  summary.json
  checkpoint_best/
  checkpoint_last/
  rl_module_best/
  rl_module_last/
  tensorboard/
```

Rollout:

```text
runs/_baseline_mlp_fullgait_diag_20260608_rollout_best_conda/
  rollout_summary.json
  sim_outputs/
```

Output diagnostici principali:

```text
rollout_episode_gait_events_online.csv
rollout_episode_online_grf.sto
rollout_episode_kinematics.sto
rollout_episode_sea_controls.sto
rollout_episode_sea_diagnostics.sto
rollout_episode_recruitment.sto
rollout_episode_so_torque_diagnostics.sto
```

## Test e verifiche eseguite

- training full-gait RLlib da `3` iterazioni;
- verifica timeout e cleanup Ray;
- ispezione `summary.json`;
- lettura diretta degli eventi e degli scalari TensorBoard;
- rollout deterministico full-gait del checkpoint migliore;
- registrazione completa degli output simulatore;
- verifica assenza `NaN`/`Inf` negli output principali;
- conteggio cicli online per lato;
- confronto heel strike online/prescribed;
- analisi saturazioni SEA;
- analisi recruitment, reserve ed equilibrium failures;
- verifica assenza di processi Ray/Python/Conda residui.

## Conclusione

Il primo TODO del daily 2026-06-07 e chiuso a livello diagnostico:

- la pipeline e in grado di allenare e valutare episodi full-gait;
- il checkpoint migliore completa un episodio da `4.0 s`;
- entrambi i lati producono cicli online completi;
- timeout e cleanup restano robusti.

La configurazione non e ancora pronta per essere scalata a training lunghi.

## TODO

- Ridurre le terminazioni `joint_divergence_pros_knee_angle`.
- Ridurre la saturazione del knee SEA.
- Analizzare e ridurre reserve biologiche ed equilibrium failures.
- Diagnosticare i fallback bounded least-squares della Static Optimization.
- Correggere o ricalibrare il timing heel strike online del lato sinistro sui
  rollout prodotti dalla policy.
- Correggere il flag onlineGRF `in_contact`, che resta sempre attivo nel rollout
  registrato.
- Migliorare il critic, la cui explained variance resta negativa.
- Prima del prossimo training lungo, rendere il launcher di monitoraggio capace
  di restituire subito il controllo e pubblicare aggiornamenti chat ogni cinque
  minuti anche durante il training principale.
