# H1-bis multi-start e correzione del resume optimizer: H2 approvata

## Obiettivo

Ripetere una singola H1 controllata dal checkpoint H0 con l'actor Markov a 35 feature, mantenendo sigma a 0,005 e `logstd` congelato, quindi verificare con H2 che l'update PPO non distruggesse la robustezza già ottenuta. Reward, FSM, soglie 15/25 mm, stato osservabile dell'actor e dati privilegiati del critic sono rimasti invariati.

## Problema iniziale

La H1 precedente, eseguita con 10 epoche e learning rate `1e-4`, aveva prodotto KL `0,15469`. H2 aveva poi fallito lo start `-0,20 s` a 231 step per penetrazione oltre 25 mm; seed 124 era arrivato a 24,945 mm, con soli 0,055 mm di margine. Il checkpoint era stato correttamente respinto.

La strategia H1-bis richiedeva due proprietà mancanti:

1. il batch PPO doveva contenere esplicitamente gli start nominale e +/-0,20 s;
2. l'update dell'actor doveva restare dentro un trust region misurabile, con KL <= 0,01.

## Strategia implementata

### 1. Sampling multi-start verificabile

È stata aggiunta la configurazione `episode_start_offset_choices_s`. I 13 EnvRunner assegnano gli offset in round-robin, dando cinque runner al caso critico `-0,20 s` e quattro agli altri due. L'ambiente riporta l'offset effettivamente applicato in ogni `info`; il callback conta gli step per offset e li salva nel risultato di training.

Durante il primo tentativo diagnostico tutti i 4096 step risultavano sul primo offset. L'analisi del sorgente RLlib 2.55 ha mostrato la causa: durante la creazione del vector env, `EnvContext` viene convertito in un normale `dict` e perde `worker_index`. La correzione applica quindi l'offset in `on_environment_created`, dove RLlib passa ancora l'`EnvContext` originale, prima del primo reset.

Copertura finale osservata:

| Start | Step nel batch |
|---|---:|
| -0,20 s (`1.756870983805102`) | 1576 |
| nominale (`1.956870983805102`) | 1260 |
| +0,20 s (`2.156870983805102`) | 1260 |

### 2. Controllo reale del learning rate al resume

Il primo tentativo H1-bis a `2e-5` ha prodotto KL `8,4039`; un secondo tentativo dichiarato a `5e-7` ha ancora prodotto KL `5,4728`. In entrambi i casi l'actor si spostava di circa 0,014 sulla media, incompatibile con la riduzione del learning rate.

La causa non era la reward né `logstd`: il restore completo di H0 ripristinava anche i `param_groups` dell'optimizer Adam, inclusa la LR salvata `1e-4`. La metrica RLlib mostrava la LR della nuova configurazione, ma l'optimizer reale continuava a usare `1e-4`.

È stato corretto il resume affinché, subito dopo `restore_from_path()`, riapplichi la LR risolta da YAML/CLI all'optimizer reale e registri un audit. Nel run valido l'audit è:

- LR optimizer prima dell'override: `9.9999997e-5`;
- LR richiesta: `5e-7`;
- LR optimizer dopo l'override e dopo l'update: `5e-7`.

Non sono stati azzerati i momenti Adam e non è stato ricostruito l'optimizer: è stata corretta soltanto la LR che il checkpoint sovrascriveva.

## H1-bis valida

Configurazione:

- sorgente: checkpoint H0 con critic warm-start;
- una sola iterazione PPO e una sola epoca;
- batch 4096, minibatch 512;
- LR reale `5e-7`;
- `clip_param=0.05`, `kl_coeff=1.0`, `kl_target=0.01`;
- sigma `0,005`, `freeze_logstd=true`;
- reward e guard di penetrazione invariati.

Risultati:

- KL RLlib: `0,0005523`;
- KL empirico su 1500 osservazioni dei tre start: `0,000605-0,000668`;
- RMSE dello spostamento della media: `0,000123-0,000129`;
- massimo delta assoluto dei parametri actor: `3,892e-6`;
- `logstd` esattamente invariato;
- digest actor finale: `654b2d180177ed9a00d77e94a45e8248989d8281de8e2641209c0d6130616c20`.

## H2

| Probe | Step | Return | Cicli validi | Penetrazione | Margine da 25 mm | Clip |
|---|---:|---:|---:|---:|---:|---:|
| deterministico -0,20 s | 500 | 40,955 | 2 | 23,825 mm | 1,175 mm | 0 |
| deterministico nominale | 500 | 54,606 | 3 | 24,097 mm | 0,903 mm | 0 |
| deterministico +0,20 s | 500 | 55,692 | 2 | 24,285 mm | 0,715 mm | 0 |
| stocastico seed 123 | 500 | 62,779 | 3 | 23,108 mm | 1,892 mm | 0 |
| stocastico seed 124 | 500 | 47,662 | 2 | 23,901 mm | 1,099 mm | 0 |
| stocastico seed 125 | 500 | 51,048 | 2 | 24,172 mm | 0,828 mm | 0 |

Tutti i rollout completano 500/500 step, producono almeno due cicli validi, restano sotto 25 mm e non richiedono clipping dell'azione. H1-bis e H2 sono quindi approvate per una continuazione controllata del training.

## Interpretazione

Il blocco non dimostrava una reward intrinsecamente errata né l'inadeguatezza dell'actor Markov a 35 feature. Due difetti di orchestrazione rendevano invalido il protocollo: mancata distribuzione reale degli start e override LR annullato dal checkpoint optimizer. Una volta corretti, PPO può aggiornare realmente l'actor senza distruggere il comportamento warm-start.

Questo risultato non valida ancora il deployment hardware. L'actor usa soltanto le 35 feature dichiarate deployable e il critic conserva le feature privilegiate, quindi il contratto architetturale è corretto; H2 valida però soltanto il simulatore, un singolo soggetto/start locale e una singola iterazione PPO.

## File modificati

- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/env_factory.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `validation/validate_training_config.py`
- `validation/test_env_start_assignment.py`
- `validation/test_training_resume.py`

## Verifiche

- 63 test unitari: tutti passati;
- smoke completo configurazione train/rollout: passato;
- `py_compile`: passato;
- `git diff --check`: passato;
- sei rollout H2 seriali: tutti passati;
- nessun processo Ray/OpenSim residuo.

## TODO

- Continuare con un blocco PPO breve e controllato, mantenendo multi-start, sigma `0,005`, `freeze_logstd=true`, audit della LR reale e gate KL <= 0,01.
- Aumentare la LR solo gradualmente; `1e-6` è il prossimo valore prudente da testare, non avviare ancora 100 iterazioni a `1e-4`.
- Ripetere i probe critici durante la rampa e l'H2 completa prima di promuovere un training lungo.
- Estendere in seguito la validazione a condizioni/soggetti non usati nell'adattamento prima di parlare di deployment validato.
