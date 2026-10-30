# baseline_MLP — training e rollout delle traiettorie protesiche

Questa cartella contiene la pipeline PPO con Ray RLlib e una MLP per generare
riferimenti cinematici di ginocchio e caviglia protesici. L'ambiente condiviso
[`osim_trj_cmc_like.py`](../osim_trj_cmc_like.py) trasforma le azioni in segmenti
di traiettoria e fa avanzare il simulatore CMC-like/OpenSim.

La policy comanda i riferimenti protesici; controllore SEA, dinamica, muscoli,
reserve e static optimization restano nel core parallelo
[`CMC-like Simulator/`](<../../CMC-like Simulator/README.md>). La reward usata dal
training e dal rollout è definita in [`reward_function.py`](reward_function.py).
La gestione delle fasi del passo è condivisa in
[`../phase_tracking/`](../phase_tracking/), fuori dalla specifica baseline MLP.

## Struttura attuale

Percorsi relativi a questa cartella, salvo i collegamenti che iniziano con `../`:

```text
baseline_MLP/
  train_ppo_mlp.py
  rollout_eval.py
  env_factory.py
  asymmetric_rl_module.py
  reward_function.py
  training_config.py
  training_exnovo_cfg.yaml
  training_cfg.yaml
  _bootstrap.py
  __init__.py
  commands.txt
  utility/
    exploration.py
    prescribed_action.py
    process_watchdog.py
    progress_display.py
    start_sampling.py
    start_condition_metrics.py
    tb_logging.py
    validate_online_grf_train_inference.py
    win_runtime.py
    phase_fsm_graph.html
  morphology_corridor/
    experimental_morphology_corridor.py
    profiles/
      ab06_prosthetic_mean_std_corridor.json
      ab06_prosthetic_event_warped_mean_std_corridor.json
  imitation_to_exnovo/
    warm_start.py
    adaptation/
    utilities/
    procedures/
```

| Componente | Ruolo |
| --- | --- |
| [train_ppo_mlp.py](train_ppo_mlp.py) | Configura PPO, supervisiona il processo di training e salva checkpoint, metriche e configurazione risolta. |
| [rollout_eval.py](rollout_eval.py) | Carica un modulo della policy e ne valuta il comportamento in un processo supervisionato. |
| [env_factory.py](env_factory.py) | Costruisce l'ambiente, rimodella e limita le azioni con `FlattenClipAction`, quindi applica `RewardShapingWrapper`. |
| [asymmetric_rl_module.py](asymmetric_rl_module.py) | Fornisce all'actor soltanto le sue feature e al critic il vettore completo delle osservazioni. |
| [training_config.py](training_config.py) | Legge i preset YAML, li traduce in opzioni e gestisce gli snapshot delle configurazioni. |
| [utility/](utility/) | Supporto a esplorazione, diagnostica, sampling, logging e runtime. È distinta da `imitation_to_exnovo/utilities/`. |
| [morphology_corridor/](morphology_corridor/) | Logica e profili del corridor cinematico protesico. |
| [imitation_to_exnovo/](imitation_to_exnovo/) | Warm start, strumenti di adattamento e procedure di trasferimento conservate. |
| [../phase_tracking/](../phase_tracking/README.md) | FSM binarie e FSM del ciclo del passo, condivise dall'ambiente. |
| [../runs/](../runs/) | Training, rollout e registro storico; il generatore del registro è [update_historical_runs.py](../runs/update_historical_runs.py). |
| [commands.txt](commands.txt) | Raccolta di esempi aggiuntivi, anche storici. I comandi essenziali aggiornati sono riportati qui sotto. |

Runtime, profili e strumenti GRF sono nella cartella condivisa
[`online_GRF/`](../../online_GRF/README.md) della repository.
I plugin compilati sono in [`plugins/SEA/`](../../plugins/SEA/) e
[`plugins/online_grf_contact/`](../../plugins/online_grf_contact/).
I sorgenti SEA vivono nel checkout esterno descritto in
[`AGENTS.md`](../../AGENTS.md); i sorgenti del plugin GRF sono in
[`online_GRF/sources/`](../../online_GRF/sources/README.md).

## Ambiente e import

Servono OpenSim e i plugin compatibili con il sistema operativo e
l'architettura del Python usato, oltre a NumPy, SciPy, Gymnasium, PyYAML,
Torch e Ray RLlib con il nuovo API stack. TensorBoard e Matplotlib servono
per logging e grafici. L'ambiente conda usato dal progetto è `envCMC-rllib`;
per compilazione e installazione dei plugin vedere [`tools/README.md`](../../tools/README.md).

Gli entrypoint preparano i percorsi tramite
[`_bootstrap.py`](_bootstrap.py), usando solo la libreria standard. Per un
nuovo script o worker, inserire `baseline_MLP/` nel percorso degli import,
poi rispettare quest'ordine:

```python
import _bootstrap
_bootstrap.ensure_sim_paths()
import win_runtime
# Poi NumPy, Torch, Ray e OpenSim.
```

Il bootstrap usa [`workspace_bootstrap.py`](../../workspace_bootstrap.py)
per esporre `CMC-like Simulator/` e la radice delle risorse condivise.
I nomi di import del simulatore restano `config`, `model_loader`,
`simulation_runner` e gli altri nomi originali. I due core rimangono nella
stessa repository e l'ambiente avanza la simulazione nello stesso processo.

[`utility/win_runtime.py`](utility/win_runtime.py) gestisce l'ordine di
caricamento Torch/OpenSim su Windows. Il bootstrap espone anche `utility/`,
`phase_tracking/`, `morphology_corridor/`, `imitation_to_exnovo/`,
`imitation_to_exnovo/adaptation/` e `../runs/`.
Gli import canonici restano, per esempio, `binary_phase`,
`prosthetic_phase_fsm`, `exploration` e `warm_start`: conservarli anche nei
worker Ray e prima di caricare oggetti serializzati.

## Configurazione predefinita

Il training usa [`training_exnovo_cfg.yaml`](training_exnovo_cfg.yaml) salvo
`--config` esplicito. Il preset contiene i parametri V26C importati dal run
`MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01` (best47), aggiornati
ai percorsi correnti. **Caricare questo preset non carica i pesi di best47**:
senza warm start o resume viene costruito un nuovo Algorithm.

| Parametro del preset ex novo | Valore corrente |
| --- | --- |
| Modello | AB06 SEASEA treadmill, stiffness ginocchio/caviglia 321/500, driver PI |
| Rete | MLP standard, 2 livelli da 256, `tanh`, actor-critic asimmetrico |
| Esplorazione | Sigma fisso `0.010`, `freeze_logstd: true` |
| Azione | Un endpoint ginocchio/caviglia ogni `0.01 s`, modalità `absolute` |
| Episodio e training | `5 s`, target finale di `51` iterazioni logiche |
| Parallelismo | `12` EnvRunner, `13` CPU Ray, sampling esatto sulle tre partenze configurate |
| PPO | Batch `4608`, minibatch `512`, una epoch, learning rate `5e-7` |
| GRF | `online_sensor`, contatto online applicato a sinistra |
| Eventi del passo | Geometria V25, FSM binaria V26, FSM dell'actor v3 |
| Morphology | Profilo event-warped, valutazione causale ritardata, pesi ginocchio/caviglia `5.0/5.0` |
| Penetrazione | Soglia della loss `20 mm`, terminazione `28 mm` |
| Supervisione | `60 s` per step, `3600 s` per iterazione, checkpoint a ogni iterazione |

[`training_cfg.yaml`](training_cfg.yaml) è il preset imitativo, da selezionare
esplicitamente. Ha una propria configurazione di osservazioni, GRF e reward:
non equivale alla ricetta imitativa V26 delle procedure storiche.
Le sezioni dei preset sono `model`, `ppo`, `parallelism`, `simulation`, `grf`,
`reward`, `supervision`, `logging` ed eventualmente `run`.

### Precedenza delle configurazioni

- **Training:** il YAML selezionato fornisce i valori iniziali; i flag CLI
  espliciti prevalgono. `--resume-from` non legge automaticamente lo snapshot
  del checkpoint: selezionare anche la configurazione coerente con quel run.
- **Rollout:** se trova uno snapshot vicino al checkpoint,
  `training_cfg.resolved.yaml` sostituisce i default globali. Il file passato
  con `--config` aggiunge i propri override; i flag CLI prevalgono.
  `--no-auto-config` disabilita soltanto la lettura dello snapshot.
  Senza snapshot si parte da `training_exnovo_cfg.yaml`.
- **Reward:** segue la configurazione selezionata, con gli ulteriori override
  forniti da `--reward-json`, che accetta un file oppure JSON inline.

Il training scrive la configurazione effettiva in
`<output_dir>/training_cfg.resolved.yaml`. Lo snapshot di best47 è stato
rimosso dopo l'importazione nel preset predefinito: il suo rollout usa quindi
il preset corrente, salvo override. Modificare quel YAML cambia anche i
futuri rollout best47. Per altri checkpoint privi di snapshot occorre
fornire una configurazione coerente con il relativo training.

Il numero di layer, le feature e il loro ordine fanno parte del contratto
del checkpoint. Non basta scegliere lo stesso modello OpenSim per rendere
compatibili due policy.

## Comandi essenziali

Eseguire dalla **radice della repository**, con l'ambiente corretto già
attivo. I percorsi con spazi sono tra virgolette e usano `/`, anche su
Windows. In alternativa anteporre `conda run --no-capture-output -n envCMC-rllib`
al comando `python`, per conservare la visualizzazione dei progressi.

Opzioni disponibili, senza avviare training o simulazioni:

```sh
python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --help
python "Trajectory Generator/baseline_MLP/rollout_eval.py" --help
```

Training ex novo e imitativo:

```sh
python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py"
python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --config "Trajectory Generator/baseline_MLP/training_cfg.yaml"
```

Prova operativa breve senza EnvRunner remoti, diversa dal protocollo V26C:

```sh
python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --no-exact-start-sampling --num-env-runners 0 --ray-num-cpus 1 --iterations 1 --train-batch-size 4 --minibatch-size 2 --num-epochs 1 --episode-duration 0.1 --no-retain-iteration-checkpoints --name _smoke
```

Il sampling esatto del preset richiede worker e batch compatibili con le tre
partenze. L'esempio locale lo disabilita esplicitamente; per cambiare il
parallelismo mantenendo quel protocollo consultare
[`utility/start_sampling.py`](utility/start_sampling.py).

Rollout deterministico dell'ultimo training valido, oppure rollout leggero
con trace della policy:

```sh
python "Trajectory Generator/baseline_MLP/rollout_eval.py"
python "Trajectory Generator/baseline_MLP/rollout_eval.py" --no-record-outputs --record-policy-trace
```

Senza `--checkpoint`, il rollout sceglie fra i run direttamente sotto
`../runs/training/` che contengono `rl_module_best` e non dichiarano
`ok: false` nel summary. La selezione usa la data di modifica del summary,
oppure quella della cartella se il summary manca; non il nome del run.
Per scegliere esplicitamente best47:

```sh
python "Trajectory Generator/baseline_MLP/rollout_eval.py" --checkpoint "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best"
```

La valutazione stocastica usa `--action-selection stochastic` oppure
`stochastic_held`. Quest'ultima mantiene lo stesso draw gaussiano per
`--exploration-noise-hold-steps`; non mantiene necessariamente uguale
l'azione finale, che dipende anche dall'osservazione corrente.

Grafici e visualizzazione dell'ultimo rollout valido:

```sh
python "plot/plotter.py" --mlp
python "CMC-like Simulator/visualize.py" --mlp
```

## Warm start e ripresa del training

| Opzione | Stato caricato |
| --- | --- |
| Nessuna | Nuovo Algorithm inizializzato con i parametri del YAML selezionato. |
| `--warm-start` | Checkpoint completo della baseline canonica H0 di luglio: actor, critic, optimizer e stato PPO. Non seleziona best47. |
| `--warm-start-raw` | Trapianto storico dei soli pesi dell'actor in un Algorithm nuovo; la sorgente predefinita è la policy imitativa di giugno. |
| `--resume-from <checkpoint>` | Ripresa da un checkpoint completo RLlib scelto esplicitamente. |
| `--freeze-actor` | Congela la distribuzione dell'actor durante il training, per aggiornare il critic; non è una sorgente di pesi. |

I due warm start richiedono reward `ex_novo` e actor-critic asimmetrico.
Sono alternativi fra loro e non si combinano con un `--resume-from`
fornito esplicitamente dall'utente.

La baseline H0 è conservata sotto
[`validation/critic_warmup/`](../../validation/critic_warmup/).
Per continuare un run usare `checkpoint_last` o `checkpoint_best`, con una
configurazione coerente. Gli export `rl_module_*` sono inference-only e
rimuovono il value tower: servono al rollout e non sostituiscono un checkpoint
completo per la ripresa di PPO.

`--iterations` è l'ultima iterazione logica da raggiungere, non il numero di
nuovi aggiornamenti. Riprendendo un checkpoint arrivato a `51`, occorre un
valore maggiore di `51` per proseguire. Le opzioni della sorgente per il
trapianto actor-only iniziano con `--warm-start-raw-source`; per i dettagli
consultare `train_ppo_mlp.py --help` e
[`imitation_to_exnovo/warm_start.py`](imitation_to_exnovo/warm_start.py).

## GRF, fasi del passo e osservazioni

Il preset corrente sceglie questi asset condivisi:

- [profilo di contatto tangent_v2](../../online_GRF/profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json);
- [detector a sfere HS/TO](../../online_GRF/profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json);
- [geometria binaria V25](../../online_GRF/binary_detector/operational/heel_toe_v25.json).

In `online_sensor`, la lista dei lati applicati decide se il contatto
influenza la dinamica. Nel preset ex novo contiene `left`: il contatto online
sostiene il lato sinistro e le GRF prescritte di quel lato sono disabilitate;
il lato destro mantiene il sostegno prescritto. Una lista vuota conserva
invece il contatto online come solo sensore.

[`binary_phase.py`](../phase_tracking/binary_phase.py) interpreta i campioni
tallone/punta: la V26 richiede contatto stabile del tallone per iniziare
l'appoggio. Campionamento `1 ms`, debounce `5 ms`, limite di consegna `10 ms`.
L'adapter trasferisce gli eventi alla distinta
[`prosthetic_phase_fsm.py`](../phase_tracking/prosthetic_phase_fsm.py), che
mantiene il ciclo HS–TO–HS, ne controlla tempi e sequenza e fornisce stato,
contatori e diagnostica. La versione v3 aggiunge recupero della sincronizzazione
e cancellazione dei contatti rimbalzati: V26 e v3 sono selezioni differenti.
Il contratto corrente è `binary_point_v25+heel_qualified_fsm_v2`.

L'actor del preset ex novo usa 35 feature. Il critic riceve l'intero vettore,
che include anche informazioni privilegiate. Le osservazioni di stato del
controllore sono abilitate; le diagnostiche derivate restano nel critic.
Il clock cinematico del lato sano è disabilitato. Il contratto effettivo delle
feature va letto nel checkpoint e nell'ambiente, soprattutto cambiando preset.

Con `binary_phase_invalid_event_policy: reject_continue`, un evento candidato
rifiutato viene scartato e l'episodio può continuare. Le incoerenze dello stato
possono comunque causare una terminazione. I payload sono esposti in
`info["phase_fsm"]`, `info["binary_phase_fsm"]`, `info["online_grf"]`,
`info["online_events"]` e `info["online_gait"]`.

## Reward e morphology corridor

L'ambiente calcola loss fisiche e diagnostica in `info["reward_terms"]`.
[`RewardShapingWrapper`](reward_function.py) le combina nella reward effettiva,
sostituendo quella interna dell'ambiente. I pesi si configurano tramite
`RewardConfig`, sezione `reward` del YAML e override `--reward-json`.

Nel preset ex novo i blend di tracking, reference e biologico sono zero.
La reward combina contatto e sostegno protesico, regolarità ed eventi del
passo, morphology e penalità su comandi, attuatori e sicurezza. Il credito
provvisorio di sostegno viene confermato da un TO valido oppure recuperato
se la sequenza fallisce. Le reserve non hanno una penalità diretta nel preset
corrente (`reserve_residual_weight: 0`).

Il corridor usa il profilo
[event-warped AB06](morphology_corridor/profiles/ab06_prosthetic_event_warped_mean_std_corridor.json)
e il modulo
[`experimental_morphology_corridor.py`](morphology_corridor/experimental_morphology_corridor.py).
La modalità corrente è `event_anchored_causal_delayed_experimental`, con effetti
causali abilitati: allinea i segmenti agli eventi del passo, raccoglie i campioni
e valuta con `40 ms` di ritardo. Il margine è `7.5°` per entrambi i giunti;
i moltiplicatori della deviazione standard sono `1.6` per il ginocchio e `0.6`
per la caviglia. I pesi sono `5.0/5.0`; la combinazione della loss è
`0.5 * (5 * L_knee + 5 * L_ankle)`. La terminazione morphology per limiti
articolari hard è disabilitata nel preset. Il secondo profilo
[mean/std AB06](morphology_corridor/profiles/ab06_prosthetic_mean_std_corridor.json)
resta disponibile per le modalità che lo richiedono.

Distinguere i valori del preset dai fallback delle classi Python:

| Controllo | Preset ex novo corrente |
| --- | --- |
| Penetrazione: inizio loss / terminazione | `20 / 28 mm`; i fallback dell'ambiente sono `12 / 17 mm`. |
| Guardia fisica del runner | `30 mm`, distinta dalla terminazione dell'episodio. |
| Banda OOB del riferimento comandato | Ginocchio `[-1.25, 0]`, caviglia `[-0.6, 0.6] rad`, peso `2.0`. |
| Timeout wall-time di uno step | `60 s`; il fallback è `30 s`. |
| Timeout hard FSM appoggio / oscillazione | `2.20 / 2.60 s`. |

`info["end_reason"]` distingue conclusioni e fallimenti. Le terminazioni
comprendono caduta, divergenza articolare, penetrazione e guardie FSM/reward,
secondo le opzioni attive. Limite temporale/dataset, errore numerico catturato
e timeout wall-time producono invece un troncamento. Anche il wrapper può
terminare l'episodio, per esempio per timeout di fase o contratto morphology
causale non valido.

## Output, TensorBoard e supervisione

| Percorso | Contenuto |
| --- | --- |
| `Trajectory Generator/runs/training/<run>/` | Config risolta, `summary.json`, `train_iterations.jsonl`, checkpoint completi `checkpoint_*` ed export inference-only `rl_module_*`. |
| `Trajectory Generator/runs/rollout/<run>/` | `rollout_summary.json`, diagnostica e, quando abilitati, trace della policy e `sim_outputs/` con gli STO. |
| `<training_run>/tensorboard/` | Eventi TensorBoard; `--no-tensorboard` disattiva il writer, non le callback di metriche e sampling. |
| `<training_run>/rllib/` | Log RLlib del run. |
| `plot/MM_DD_YYYY_N/` nella radice | I sette grafici canonici del rollout selezionato, prodotti da `plot/plotter.py --mlp` in una sottocartella dedicata alla singola esecuzione. |

I nomi dei training sono generati da strategia e data; `--name` aggiunge un
suffisso. I rollout derivano il nome dal training selezionato. Le collisioni
producono suffissi numerici. I percorsi relativi di output vengono ricondotti
alla categoria training o rollout; preferire i prefissi completi negli esempi
personalizzati per evitare ambiguità.

Il rollout registra gli STO di default. `--no-record-outputs` li disabilita;
`--record-policy-trace` conserva la trace JSON anche senza STO. Con gli output
abilitati viene salvata anche la trace della policy.

```sh
python -m tensorboard.main --logdir "Trajectory Generator/runs/training" --port 6006
```

Il training e il rollout usano processi figli supervisionati. Le guardie
controllano startup, heartbeat, iterazioni, checkpoint e cleanup. Un timeout
può causare uno skip con ripartenza dal checkpoint; un crash nativo può
ritentare la stessa iterazione, entro i limiti configurati. Il self-timeout
del child e il cleanup dell'albero Ray evitano che un worker bloccato lasci
il run attivo indefinitamente. Progressi, `supervisor_state.json` e log del
watchdog permettono di ricostruire l'esito.

## Utility e trasferimento imitativo

| Modulo | Uso |
| --- | --- |
| [utility/exploration.py](utility/exploration.py) | Sigma costante dell'actor e rumore gaussiano mantenuto per più step. La CLI copia un modulo e conserva i pesi della media. |
| [utility/prescribed_action.py](utility/prescribed_action.py) | Playback IK in radianti, solo con azioni diagnostiche `raw`; usato dal ramo opzionale del test prescritto della FSM. |
| [utility/start_sampling.py](utility/start_sampling.py) | Contratti del sampling esatto e delle partenze bilanciate. |
| [utility/start_condition_metrics.py](utility/start_condition_metrics.py) | Metriche per partenza dopo GAE e diagnostica degli aggiornamenti. |
| [utility/process_watchdog.py](utility/process_watchdog.py) | Supervisione dei processi e self-test dei timeout. |
| [utility/progress_display.py](utility/progress_display.py) | Progressi, tempi trascorsi ed ETA. |
| [utility/tb_logging.py](utility/tb_logging.py) | Metriche reward/FSM/morphology, sampling e TensorBoard. |
| [utility/validate_online_grf_train_inference.py](utility/validate_online_grf_train_inference.py) | Gate e smoke del contratto GRF: richiede un report di accettazione valido con `--acceptance-report`. Il report predefinito in `results/` è stato eliminato. |

`utility/phase_fsm_graph.html` è una visualizzazione della FSM, senza ruolo
nel runtime. Le utility dotate di una CLI espongono gli argomenti con `--help`.
La CLI di esplorazione richiede una configurazione risolta e il manifest delle
feature accanto al modulo sorgente, oppure un manifest indicato esplicitamente;
non genera questi metadati dal solo checkpoint.

[`imitation_to_exnovo/adaptation/`](imitation_to_exnovo/adaptation/) contiene
`target_domain_imitation.py`, `target_domain_dagger.py`,
`target_domain_noise_adaptation.py` e `target_domain_markov_adaptation.py`:
rispettivamente fitting imitativo, DAgger, adattamento al rumore e alle feature
dello stato del controllore. Espongono helper usati dalle procedure e CLI
con propri input. Non costituiscono da soli la procedura V26.

[`imitation_to_exnovo/utilities/`](imitation_to_exnovo/utilities/) contiene:

- [freeze_actor_checkpoint.py](imitation_to_exnovo/utilities/freeze_actor_checkpoint.py):
  congela una copia dell'actor con manifest e digest; richiede modulo,
  configurazione risolta e feature metadata nel run sorgente. Best47, privo
  del suo snapshot, richiede il ripristino degli input per questo comando.
- [imitation_oracle_rollout.py](imitation_to_exnovo/utilities/imitation_oracle_rollout.py):
  esegue un target imitativo attraverso il simulatore. Usa propri default CLI
  per la simulazione; senza `--reward-json` legge da `training_cfg.yaml` solo
  nove campi di banda OOB, smoothness/command-rate e stress SEA. Gli altri
  campi usano i default di `RewardConfig`, con `reward_mode: imitation`.
  Un `--reward-json` esplicito sostituisce quel sottoinsieme, non lo integra.

[`imitation_to_exnovo/procedures/`](imitation_to_exnovo/procedures/) conserva
le cinque famiglie `f0_freeze_2026-08-22`, `f1_ablation_2026-08-23`,
`f2r_bridge_2026-08-23`, `v26b_bridge_2026-08-24` e
`v26c_july_replica_2026-08-26`. Contengono strumenti, protocolli, dataset,
pesi ed evidenze delle procedure storiche. Per riutilizzarle occorrono gli
input e i contratti di ciascuna procedura.

La proiezione degli aggiornamenti actor è incorporata nel comando `project`
di [`validation/test/test_project_actor_update.py`](../../validation/test/test_project_actor_update.py),
insieme al relativo test.

## Verifiche e compatibilità storica

I test correnti sono in [`validation/test/`](../../validation/test/) della
repository; prerequisiti e comandi sono nel
[`README di validazione`](../../validation/README.md). Alcuni controlli
richiedono fixture eliminate durante la pulizia: il profilo V19 e la sequenza
attesa della FSM v2, per esempio, non sono più presenti. Il test della reward
che attende ancora un timeout di swing di `1.10 s` non corrisponde al preset
corrente, che usa `2.60 s`.

Le precedenti cartelle della baseline `validation/`, `experimental_configs/`
e i moduli primary split adapter/residual sono stati ritirati. Le utility
prima nella radice sono ora nelle sottocartelle descritte qui. I nomi canonici
degli import sono conservati dal bootstrap; per vecchi pickle dei quattro
moduli binari precedenti importare anche `binary_phase`, che registra gli
alias necessari in memoria.

Manifest, snapshot e protocolli conservano i percorsi e gli hash originali.
Gli inventari congelati F0 e i gate J20/J21 non certificano il layout
riorganizzato: possono richiedere sorgenti ritirati, incluso il test di
telemetria eliminato. Per rieseguire quei controlli serve il checkout
originale completo con gli input previsti; spostare un file non giustifica
riscriverne il pin o aggirare il gate.

La documentazione delle precedenti riorganizzazioni è nei registri di
[pulizia della validazione](../../reports/cleanup/2026-10-19_baseline_validation_removal/README.md),
[trasferimento imitativo](../../reports/cleanup/2026-10-19_imitation_to_exnovo_rename/README.md)
e [rimozione dei preset sperimentali](../../reports/cleanup/2026-10-19_experimental_configs_removal/README.md).
I risultati dei vecchi tiny training e dei benchmark descrivono le relative
configurazioni storiche, non una qualificazione automatica del preset attuale.
