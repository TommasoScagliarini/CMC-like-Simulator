# Validazione del simulatore e test

Questa cartella contiene il validatore generale dei risultati CMC-like, un
controllo eseguibile della configurazione di training, i test di regressione e
un checkpoint storico ancora utilizzato dal warm-start.

Gli strumenti di generazione, calibrazione e verifica delle GRF sono descritti
nel [README di `online_GRF/`](../online_GRF/README.md).
La sottocartella `online_GRF/validation/` riguarda il contatto GRF;
questa `validation/` riguarda il simulatore e i test del progetto.
Il codice del simulatore è nel core parallelo
[`CMC-like Simulator/`](<../CMC-like Simulator/README.md>); questa cartella
rimane nella radice condivisa del workspace.

## Struttura presente

Ricognizione del 2026-10-19:

```text
validation/
  validate_sim_results.py
  validate_training_config.py
  critic_warmup/
    2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/
      checkpoint_last/
      rl_module_last/
      checkpoint_last_meta.json
      training_cfg.resolved.yaml
  test/                     # 49 moduli test_*.py e __init__.py
  __init__.py
  README.md
```

Le cartelle `experimental_detector_profiles/` e `fixtures/` non sono presenti
nel working tree attuale. Alcuni test le referenziano ancora:
leggere la sezione sulle dipendenze mancanti prima di interpretare l'esito
della suite completa.

## Validatori rimasti

| File | Ruolo e utilizzo |
| --- | --- |
| [validate_sim_results.py](validate_sim_results.py) | Legge gli STO salvati, controlla il comportamento SEA, le derivate e le diagnostiche dell'interfaccia e riporta gli errori di tracking. È richiamato da `python "CMC-like Simulator/main.py" --validate` con i percorsi della configurazione attiva. |
| [validate_training_config.py](validate_training_config.py) | Controllo eseguibile di caricamento YAML, precedenza degli argomenti CLI, impostazioni di training/rollout e salvataggio/rilettura della configurazione. Contiene anche aspettative su valori specifici dei preset. |

Dalla radice del repository:

```sh
python "CMC-like Simulator/main.py" --validate
```

Il comando usa la configurazione del simulatore e scrive il report in
`reports/user/`. Per indicare direttamente risultati e prefisso:

```sh
python validation/validate_sim_results.py --results-dir results --prefix sim_output
```

Il validatore diretto accetta anche `--model-bundle`, `--model`, `--reference`
e `--out`. Senza override usa i percorsi risolti dalla configurazione; i file
STO e il modello selezionato devono essere coerenti con il run analizzato.

Il controllo della configurazione di training si esegue separatamente:

```sh
python validation/validate_training_config.py
```

I suoi controlli possono richiedere revisione quando cambiano i preset.
Non viene raccolto automaticamente dal comando pytest sotto.

Dal 2026-10-19 training e rollout usano come configurazione predefinita
[`training_exnovo_cfg.yaml`](<../Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml>),
con i parametri V26C importati dal run best47. Il preset imitativo
`training_cfg.yaml` resta selezionabile con `--config`. Il rollout usa lo
snapshot del checkpoint al posto del default quando presente; applica poi la
configurazione esplicita e infine i flag CLI. Best47 usa il default perché il
suo snapshot è stato rimosso dopo l'importazione. Per dettagli vedere il
[README della baseline](<../Trajectory Generator/baseline_MLP/README.md>).

Il codice del corridor morphology è in
[`baseline_MLP/morphology_corridor/experimental_morphology_corridor.py`](<../Trajectory Generator/baseline_MLP/morphology_corridor/experimental_morphology_corridor.py>),
e i due profili JSON AB06 sono nella sua sottocartella
[`profiles/`](<../Trajectory Generator/baseline_MLP/morphology_corridor/profiles/>).
`validate_training_config.py` controlla il nuovo percorso del profilo
event-warped scelto dal preset corrente. Il bootstrap conserva il nome di
import del modulo; il resolver continua a risolvere i vecchi percorsi
`morphology_profiles/` dei checkpoint, senza riscrivere configurazioni e
manifest congelati. I test numerici restano sotto `validation/test/`.

La vecchia cartella `baseline_MLP/experimental_configs/` è stata rimossa il
2026-10-19. I due YAML morphology e il detector V17 erano stati trasferiti
sotto `baseline_MLP/validation/fixtures/morphology/`, poi eliminati dall'utente.
Il test corrente
[test_causal_delayed_morphology_corridor.py](test/test_causal_delayed_morphology_corridor.py)
conserva i controlli numerici della reward; il caso dedicato a quelle fixture
è stato ritirato insieme agli input.
Il test dedicato al candidato V26, che viveva direttamente in
`baseline_MLP/validation/`, è stato eliminato nella successiva pulizia; il suo
YAML non è più presente. La configurazione operativa predefinita
non cambia. Il
[registro della rimozione](../reports/cleanup/2026-10-19_experimental_configs_removal/README.md)
documenta anche le dipendenze delle procedure storiche ritirate.

La successiva [pulizia della validazione della baseline](../reports/cleanup/2026-10-19_baseline_validation_removal/README.md)
ha eliminato i file direttamente nella radice di `baseline_MLP/validation/`,
le directory `v12*` e `h0_v12r1_run_20260809/`, il bundle V26C e lo sweep sigma
del 31 agosto. Restano la workspace V26C J0–J21, i namespace F0/F1/F2R/V26B e
il loro contenuto originale. Dal 2026-10-19 sono riuniti in
[`baseline_MLP/imitation_to_exnovo/procedures/`](<../Trajectory Generator/baseline_MLP/imitation_to_exnovo/procedures/>).
I moduli condivisi `adaptation/` e le CLI `utilities/` restano direttamente
sotto `imitation_to_exnovo/`.
Le fixture morphology e la cache pytest sono state eliminate dall'utente prima
della rinomina. Pesi, dataset, manifest e protocolli conservati non sono stati
modificati. Le verifiche storiche J20/J21 richiedono però anche il test
`baseline_MLP/validation/test_training_health_telemetry.py`, già rimosso:
per rieseguirle serve ripristinare l'originale con l'hash atteso. Il
[registro della rinomina](../reports/cleanup/2026-10-19_imitation_to_exnovo_rename/README.md)
descrive compatibilità dei percorsi e limiti delle procedure storiche.

## Test di regressione — `test/`

I 49 moduli coprono preparazione e percorsi dei modelli, detector e FSM,
static optimization, reward, configurazione e ripresa del training, adattatori
delle osservazioni e warm-start. Diversi test costruiscono e avanzano un vero
ambiente OpenSim; altri usano input sintetici e directory temporanee.

Il detector geometrico e in
[`online_GRF/binary_detector/binary_phase_detector.py`](../online_GRF/binary_detector/binary_phase_detector.py),
accanto alla geometria operativa V25. Il test dedicato importa
`online_GRF.binary_detector.binary_phase_detector`, come il loader e
il runner del simulatore. L'import canonico registra l'alias precedente
`binary_phase_detector` e il precedente nome qualificato sotto
`online_grf_profiles` per i pickle; caricare il modulo prima di aprire
manualmente un pickle che cita quei nomi. Il package non lo importa
automaticamente. Gli inventari congelati F0 conservano il vecchio percorso
root e il relativo hash: la loro verifica richiede il checkout originale.

Dal 2026-10-20 i test delle FSM binarie e dei loro adapter importano
[`Trajectory Generator/phase_tracking/binary_phase.py`](<../Trajectory Generator/phase_tracking/binary_phase.py>),
che riunisce i quattro moduli precedenti. Le classi base e le sottoclassi V26
restano separate: i test conservano i rispettivi casi di contatto, debounce,
consegna degli eventi, rifiuto dei batch invalidi e aggiornamento atomico
delle due FSM. Campionamento a 1 ms, debounce a 5 ms, limite di consegna a
10 ms e identificatori dei contratti restano invariati. I nomi dei file test
continuano a distinguere base/V20 e V26. Gli inventari congelati, inclusa la
closure F0, mantengono i quattro vecchi percorsi dei sorgenti e richiedono il
checkout originale per la verifica degli hash. La FSM actor-facing e nel
modulo separato
[`phase_tracking/prosthetic_phase_fsm.py`](<../Trajectory Generator/phase_tracking/prosthetic_phase_fsm.py>).
Il bootstrap della baseline espone la cartella condivisa conservando gli
import canonici `binary_phase` e `prosthetic_phase_fsm`, anche per i pickle.
Per i pickle che citano i quattro moduli binari precedenti, importare prima
`binary_phase` dopo il bootstrap per registrarne gli alias.
Le fixture mancanti elencate
piu avanti restano dipendenze dei relativi test.

Dal 2026-10-19 sono ritirati i quattro test dedicati a primary GRF split,
input adapter e residual V25, insieme ai tre moduli della baseline e alla
relativa selezione nel trainer. La MLP standard resta il percorso corrente.
I checkpoint adapter/residual e la closure storica F0 richiedono il checkout
originale completo; i loro riferimenti e hash sono conservati.

I test [test_start_sampling.py](test/test_start_sampling.py) e
[test_start_condition_metrics.py](test/test_start_condition_metrics.py)
verificano i moduli correnti in
[`baseline_MLP/utility/`](<../Trajectory Generator/baseline_MLP/utility/>).
Il primo controlla il contratto aritmetico del sampling con la sola libreria
standard; il secondo richiede Ray/Torch per connector, Learner e metriche per
start. Il bootstrap della baseline conserva gli import canonici
`start_sampling` e `start_condition_metrics`, usati anche nei pickle dei
checkpoint e dai worker avviati con spawn su Windows e macOS. La closure F0
conserva i vecchi percorsi dei sorgenti e richiede il checkout storico.

Anche `process_watchdog.py`, `progress_display.py`, `tb_logging.py`,
`exploration.py`, `win_runtime.py` e `validate_online_grf_train_inference.py` vivono in
`baseline_MLP/utility/`, mantenendo i nomi di import canonici tramite bootstrap.
I test [test_progress_display.py](test/test_progress_display.py) e
[test_tb_logging_start_metrics.py](test/test_tb_logging_start_metrics.py)
verificano avanzamento e aggregazione delle metriche per partenza; il watchdog
offre `python "Trajectory Generator/baseline_MLP/utility/process_watchdog.py" --self-test`.
Il callback di `tb_logging` resta attivo anche con `--no-tensorboard` per
l'assegnazione multi-start e i controlli sul bilanciamento delle partenze.

Il validatore onlineGRF si avvia con
`python "Trajectory Generator/baseline_MLP/utility/validate_online_grf_train_inference.py" --help`.
Per le prove serve un report di accettazione valido tramite
`--acceptance-report`: quello predefinito in `results/` e stato eliminato.
Il modulo condiviso `warm_start` e in
[`baseline_MLP/imitation_to_exnovo/warm_start.py`](<../Trajectory Generator/baseline_MLP/imitation_to_exnovo/warm_start.py>),
mentre il generatore del registro e in
[`Trajectory Generator/runs/update_historical_runs.py`](<../Trajectory Generator/runs/update_historical_runs.py>).
Gli import canonici restano disponibili dopo `_bootstrap.ensure_sim_paths()`;
importare `win_runtime` subito dopo il bootstrap e prima delle librerie numeriche.

I test [test_target_domain_imitation.py](test/test_target_domain_imitation.py)
e [test_target_domain_imitation_split.py](test/test_target_domain_imitation_split.py)
verificano gli helper di fitting e adattamento ora in
[`baseline_MLP/imitation_to_exnovo/adaptation/`](<../Trajectory Generator/baseline_MLP/imitation_to_exnovo/adaptation/>).
Il bootstrap espone dalla nuova cartella i quattro nomi canonici
`target_domain_imitation`, `target_domain_dagger`,
`target_domain_noise_adaptation` e `target_domain_markov_adaptation`:
eseguire `_bootstrap.ensure_sim_paths()` prima di importarli.
Le CLI si avviano dai nuovi percorsi; manifest e protocolli storici conservano
i percorsi e gli hash originali.

Usare un ambiente con pytest, NumPy/SciPy, OpenSim, Ray/Torch e le altre
dipendenze del progetto. Su questo Mac l'ambiente di test è `envCMC-rllib`.
Dalla radice del repository:

```sh
python -B -m pytest -q -p no:cacheprovider validation/test/
```

Per un singolo modulo, per esempio i percorsi dei modelli:

```sh
python -B -m pytest -q -p no:cacheprovider validation/test/test_model_layout_paths.py
```

`-B` disabilita la scrittura del bytecode; `-p no:cacheprovider` evita la cache
pytest. `--basetemp` permette di scegliere una directory temporanea esterna al
repository. I comandi su singola riga sono validi sia in PowerShell sia nelle
shell macOS.

[test_phase_fsm_prescribed_env.py](test/test_phase_fsm_prescribed_env.py) è un
controllo manuale dell'ambiente reale, eseguito tramite la sua funzione `main`:
pytest importa il modulo ma non esegue quel controllo. Il ramo
`CMC_FULL_FSM_PRESCRIBED_TEST=1` usa
[`baseline_MLP/utility/prescribed_action.py`](<../Trajectory Generator/baseline_MLP/utility/prescribed_action.py>)
per campionare dalla IK le posizioni future in radianti, con
`action_mode="raw"`. Il nome di import canonico `prescribed_action` è
disponibile dopo `_bootstrap.ensure_sim_paths()`.
Gli scenari sintetici della FSM v3 sono incorporati in
[test_prosthetic_phase_fsm_v3_resync.py](test/test_prosthetic_phase_fsm_v3_resync.py).

[test_project_actor_update.py](test/test_project_actor_update.py) contiene sia
il test di interpolazione actor-only sia l'utility di proiezione, prima in
`baseline_MLP/project_actor_update.py`. Senza argomenti esegue il test ed è
raccolto normalmente da unittest e pytest. Il sottocomando `project` limita
la variazione della media nominale sulle osservazioni di una trace:

```sh
python validation/test/test_project_actor_update.py project --source-module source/rl_module --candidate-module candidate/rl_module --reference-trace trace.json --output-dir projected_actor
```

`--max-nominal-mean-shift` ha default `0.005`; `--action-dim` ha default `2`.
La trace JSON deve contenere `actor_observation_vector_before` per ogni riga.
La sorgente deve avere accanto `training_cfg.resolved.yaml` e
`actor_feature_manifest.json`. Il comando usa `baseline_MLP/imitation_to_exnovo/warm_start.py` e le
sue dipendenze. Salva un modulo `rl_module_projected`, configurazione, manifest aggiornato e
`projection_report.json` in una nuova cartella di output.

## Checkpoint — `critic_warmup/`

È rimasto un solo esperimento:
[`2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/`](critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/),
con 29 file per circa 2,51 MB. I metadata registrano l'iterazione 1.
Il critic è la parte della rete che stima il ritorno futuro; questa cartella
contiene il risultato salvato di quella specifica fase di inizializzazione.

| Contenuto | Ruolo attuale |
| --- | --- |
| `checkpoint_last/` | Checkpoint completo RLlib: modulo, stato dell'algoritmo, ottimizzatore e componenti dei worker/connettori. È il checkpoint H0 selezionato dall'opzione `--warm-start`. |
| `rl_module_last/` | Modulo di inferenza, circa 607 KB. È usato dai test di adattamento della policy e dei wrapper delle osservazioni. |
| `training_cfg.resolved.yaml` | Configurazione originale del modello, ambiente, reward e training. |
| `checkpoint_last_meta.json` | Metadata dell'iterazione e del checkpoint salvato. |

Il trainer `baseline_MLP` in `Trajectory Generator/` mantiene il riferimento
a questo checkpoint. Con `--warm-start` ripristina il checkpoint completo;
`--warm-start-raw` è una modalità distinta di trasferimento dell'actor.

La normale simulazione CMC-like non legge questa cartella. Eliminandola senza
aggiornare i consumer si interrompono il warm-start H0 e alcuni test.
Contiene pesi appresi: ricrearla richiede ripetere il training e non garantisce
byte identici. Preservare anche configurazione e metadata insieme ai pesi.

## Dipendenze mancanti dei test

Alla ricognizione del 2026-10-17 i tre file seguenti sono assenti. Sono elencati
come percorsi richiesti dal codice, non come contenuti presenti della cartella.

| Percorso richiesto sotto `validation/` | Utilizzo ancora presente |
| --- | --- |
| `experimental_detector_profiles/two_point_binary_v19_outsole_25mm.json` | Geometria binaria di riferimento letta da [test_binary_phase_detector_v19.py](test/test_binary_phase_detector_v19.py), [test_binary_phase_fsm_env_v20.py](test/test_binary_phase_fsm_env_v20.py) e [test_binary_phase_active_env.py](test/test_binary_phase_active_env.py). |
| `experimental_detector_profiles/two_sensor_v17_high_rate_v13_geometry.json` | Profilo a sfere usato da [test_detector_sensor_data_path.py](test/test_detector_sensor_data_path.py). |
| `fixtures/fsm_v2_golden_scripted_sequence.json` | Sequenza attesa di 615 passi della FSM v2, usata dai confronti di compatibilità della FSM v3 in [test_prosthetic_phase_fsm_v3_resync.py](test/test_prosthetic_phase_fsm_v3_resync.py). |

Queste fixture storiche sono distinte dalla
[geometria binaria operativa V25](../online_GRF/binary_detector/operational/heel_toe_v25.json).
L'assenza dei tre file a questi percorsi impedisce l'esecuzione corretta dei
controlli che li leggono direttamente. Dal 2026-10-19
[test_causal_delayed_morphology_corridor.py](test/test_causal_delayed_morphology_corridor.py)
mantiene sei controlli numerici; il caso che leggeva le fixture morphology è
stato ritirato dopo la loro eliminazione. Il test di readiness V26 della
baseline è stato rimosso insieme agli altri file nella radice della precedente
`baseline_MLP/validation/`. Il percorso V17 richiesto da
`test_detector_sensor_data_path.py` e le altre due fixture rimangono mancanti.
Per riutilizzarli occorre ripristinare gli input originali o aggiornare
esplicitamente i relativi consumer. Non sostituire automaticamente le fixture
con il profilo operativo e non aggiungere skip per nasconderne l'assenza.

## Verifiche registrate e storia

La [verifica della riorganizzazione GRF](../reports/cleanup/2026-10-17_grf_tools_reorganization/verification.json)
registrava 519 test raccolti: 517 superati e due fallimenti già presenti, oltre
ai controlli CLI e al confronto numerico Python/C++ del plugin GRF.
Quella verifica precede l'attuale assenza delle tre fixture e non certifica la
suite nel working tree corrente.

I due fallimenti registrati riguardavano aspettative sui preset:

- `test_candidate_yaml_is_separate_preflight_only_contract`: il test attendeva
  `morphology_weight = 0`, mentre il preset riportava `0.0025`.
- `test_exnovo_config_enables_task_based_reward_terms`: il test attendeva un
  timeout swing di `1.1 s`, mentre il preset riportava `2.6 s`.

Le campagne storiche, i tool policy/pilot, la diagnostica SEA opzionale e i
builder/audit ritirati hanno perso anche i rispettivi test dedicati. I test
rimasti fanno riferimento ai tool GRF nelle loro nuove sottocartelle sotto
`online_GRF/`.

I report, i manifest e le configurazioni congelate conservano i percorsi e gli
hash originali. Per riprodurre una campagna ritirata servono il checkout e gli
input corrispondenti, recuperabili dalla versione Git appropriata o dal backup.
Le procedure di trasferimento della policy e le relative verifiche sono in
[`Trajectory Generator/baseline_MLP/imitation_to_exnovo/procedures/`](<../Trajectory Generator/baseline_MLP/imitation_to_exnovo/procedures/>),
con un perimetro distinto da questa cartella.
