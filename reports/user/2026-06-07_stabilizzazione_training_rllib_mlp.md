# Stabilizzazione training RLlib MLP

## Obiettivo

Risolvere i TODO relativi alla pipeline MLP/RLlib:

- capire perche il tiny training restava appeso e generava molti processi Ray;
- garantire l'arresto pulito in caso di errore, interruzione o timeout;
- rivalidare il training parallelo con `num_env_runners > 0`;
- eseguire almeno `50` iterazioni reali;
- analizzare critic, return, stabilita e cause di fine episodio tramite
  `end_reason`;
- impedire che un training bloccato possa inchiodare il PC.

## Problemi individuati

### Import prima del supervisore

`train_ppo_mlp.py` importava Torch, Ray e OpenSim prima di avviare il processo
supervisore. Un eventuale blocco durante gli import avveniva quindi prima che
il timeout potesse intervenire.

Quando l'interprete Conda veniva invocato direttamente tramite il percorso di
`python.exe`, il `PATH` poteva inoltre non includere
`envCMC-rllib/Library/bin`. NumPy/Torch restavano bloccati durante il
caricamento delle dipendenze BLAS/MKL.

### Directory RLlib non scrivibile

`PPOConfig.build_algo()` tentava di creare la directory standalone sotto
`~/ray_results`.

Quando il percorso non era scrivibile, `tempfile.mkdtemp()` su Windows
interpretava ogni `PermissionError` come una possibile collisione e riprovava
indefinitamente. Questo era il blocco principale osservato durante
`build_algo()`.

### Proliferazione dei processi Ray

Senza un limite CPU esplicito, `ray.init()` rilevava tutte le CPU host e
preparava numerosi worker Python inattivi anche con `num_env_runners=0`.

## Soluzione implementata

### Supervisore e timeout rigidi

Il processo principale e ora un launcher minimale che non importa Torch, Ray o
OpenSim. Il training effettivo viene eseguito in un processo figlio
supervisionato.

Sono disponibili timeout separati per:

- import e startup;
- ogni chiamata `algo.train()`;
- esportazione checkpoint;
- `algo.stop()` e `ray.shutdown()`;
- durata totale del run;
- sampling EnvRunner.

Su Windows, il processo figlio viene assegnato a un Job Object con
`KILL_ON_JOB_CLOSE`. In caso di timeout viene terminato l'intero albero dei
processi Ray. E presente anche un fallback tramite `taskkill /T /F`.

### Diagnostica automatica

Durante le fasi bloccanti viene scritto `faulthandler.log`, contenente gli stack
Python dei thread prima del timeout. Il processo worker pubblica inoltre la
fase corrente in `watchdog_state.json`.

Questa diagnostica ha permesso di distinguere:

- blocco NumPy/Torch causato dal `PATH` Conda incompleto;
- blocco RLlib dentro `tempfile.mkdtemp()` su `~/ray_results`.

### Configurazione Ray e directory dei log

- Ray vede per default solo `num_env_runners + 1` CPU;
- il numero di CPU resta configurabile con `--ray-num-cpus`;
- i log standalone RLlib vengono scritti in `<output_dir>/rllib`;
- il worker supervisionato aggiunge automaticamente al `PATH` le directory DLL
  dell'ambiente Conda associato al proprio interprete.

### Metriche e cause di fine episodio

`summary.json` registra ora per ogni iterazione:

- return ed episode length;
- policy loss, value loss ed entropy;
- metriche numeriche complete del learner, inclusa explained variance e KL;
- contatori cumulativi `episode_end/*`;
- medie per-step di `terminated` e `truncated`.

I contatori delle cause di fine episodio usano il reducer RLlib
`lifetime_sum`.

## Verifiche eseguite

### Timeout e cleanup

Un probe intenzionalmente bloccato e stato interrotto dal watchdog. Dopo il
timeout non erano presenti processi Python Ray, `raylet` o `gcs_server`
residui.

### Tiny training locale

Configurazione:

- `num_env_runners=0`;
- `ray_num_cpus=1`;
- `grf_mode="online_sensor"`;
- osservazione onlineGRF/gait attiva.

Risultato:

- una iterazione completata;
- checkpoint best/last e RLModule creati;
- arresto pulito.

### Tiny training parallelo

Configurazione:

- `num_env_runners=1`;
- `ray_num_cpus=2`;
- `grf_mode="online_sensor"`.

Risultato:

- EnvRunner remoto avviato correttamente;
- plugin SEA e onlineGRF caricati nel worker;
- una iterazione completata;
- cleanup senza processi residui.

### Probe metriche

Un training parallelo da `3` iterazioni ha verificato la registrazione di:

- `policy_loss`;
- `vf_loss`;
- `entropy`;
- `vf_explained_var`;
- `episode_end/episode_time_limit`;
- `terminated` e `truncated`.

### Training controllato da 50 iterazioni

Run:

`runs/_baseline_mlp_50iter_online_sensor`

Configurazione principale:

- `50` iterazioni;
- un EnvRunner e due CPU Ray;
- `grf_mode="online_sensor"` e osservazione gait attiva;
- batch `4`, minibatch `4`, un epoch;
- episodio `0.08 s`, segmento `0.02 s`;
- checkpoint ogni `10` iterazioni;
- timeout per iterazione `90 s`;
- timeout totale `900 s`.

Risultati:

| Metrica | Valore |
|---|---:|
| Iterazioni completate | `50/50` |
| Timeout | `0` |
| Tempo wall-clock | `716.42 s` |
| Miglior return | `3.3045` |
| Return medio prime 10 | `2.4356` |
| Return medio ultime 10 | `2.6363` |
| Value loss media prime 10 | `2.8313` |
| Value loss media ultime 10 | `2.5147` |
| Explained variance prime 10 | `-0.1345` |
| Explained variance ultime 10 | `-0.0042` |
| Entropy prime 10 | `8.3611` |
| Entropy ultime 10 | `8.2745` |
| `episode_time_limit` | `48` |
| `joint_divergence_pros_knee_angle` | `2` |
| Processi Ray residui | `0` |

Tutte le metriche core sono rimaste finite. Il return mostra un miglioramento
modesto e l'entropia non collassa. Il critic migliora ma resta debole, poiche
l'explained variance rimane leggermente negativa.

Il run ha validato la pipeline RLlib, gli update PPO, i timeout, il cleanup,
il parallelismo e la diagnostica `end_reason`. L'orizzonte breve non permette
pero di considerarlo una validazione dell'apprendimento su gait cycle completi.

Durante il training sono comparsi ripetuti fallback bounded least-squares della
Static Optimization. Non hanno interrotto il run, ma rappresentano un limite da
analizzare prima di training full-gait costosi.

## TODO originari risolti

- tiny training RLlib che restava appeso;
- proliferazione dei processi Ray;
- arresto pulito in caso di timeout/interruzione;
- timeout per evitare il blocco del PC;
- training parallelo con `num_env_runners > 0`;
- training reale di almeno `50` iterazioni;
- analisi di critic, return, stabilita ed `end_reason`.

## TODO residui emersi

- eseguire training full-gait con episodi sufficientemente lunghi da osservare
  heel strike e gait cycle completi;
- migliorare il critic, la cui explained variance resta negativa;
- ridurre le terminazioni unsafe `joint_divergence_pros_knee_angle`;
- analizzare i fallback bounded least-squares della Static Optimization;
- rivalidare training paralleli con piu di un EnvRunner solo dopo aver misurato
  consumo CPU/RAM e scalabilita.

## File modificati

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `reports/user/2026-06-05_baseline_mlp_rllib_ppo_single_agent.md`
- `reports/user/2026-06-05_reward_centralizzata_e_tensorboard.md`
- `reports/user/2026-06-06_correzione_semantica_terminated_truncated.md`
- `reports/user/2026-06-06_dalla_reward_imitativa_alle_grf_online_glide.md`
- `reports/user/2026-06-07_modalita_online_grf_simulatore.md`

## Artefatti principali

- `runs/_baseline_mlp_tiny_after_logroot_fix`
- `runs/_baseline_mlp_parallel_tiny`
- `runs/_baseline_mlp_metrics_probe`
- `runs/_baseline_mlp_50iter_online_sensor`

## Verifiche finali

- `py_compile` sui moduli Python modificati: completato;
- `git diff --check`: completato senza errori;
- checkpoint best/last e RLModule best/last: presenti;
- file TensorBoard: presente;
- metriche core finite su tutte le `50` iterazioni;
- processi Python Ray, `raylet` e `gcs_server` residui: `0`.
