# Rete unificata V1: prescribed sinistro → ex-novo

Instruction check token: `CMC_AGENT_OK_2026`.

La pipeline usa lo stesso actor, le stesse osservazioni e lo stesso ambiente
per pretraining imitativo e PPO ex-novo. L'actor iniziale ha pesi nuovi. Il
passaggio carica direttamente il suo checkpoint: non servono bridge, transplant
di feature o DAgger. **La V1 non include actor predittivo né compensazione del
ritardo**; slew limiter e filtro C2 restano quelli della baseline.

La baseline del commit `8ef48f914898fff8431cf8d164554448dbe35947` è conservata.
Le modifiche della pipeline sono qui; le utility baseline e il simulatore
condiviso vengono riutilizzati senza modificarli. Training, TensorBoard,
checkpoint e rollout scrivono esclusivamente sotto `unified_MLP/runs/` e non
aggiornano il registro storico della baseline. I checkpoint baseline non sono
sorgenti ammesse per `--init-actor-from`.

## Contratto e dati

- Actor MLP: **35 → 256 tanh → 256 tanh → 2 target assoluti**, a 100 Hz.
  La testa gaussiana produce due medie e due logstd. Critic privilegiato separato.
- Nomi, ordine, unità, bounds e normalizzazione coincidono nelle due modalità;
  le due colonne storiche del clock prescribed restano zero. I target imitativi
  non entrano nell'actor.
- La loss usa `pros_knee_angle` e `pros_ankle_angle` da `base_kin` al tempo
  simulato corrente: prescribed **sinistro**, posizioni in radianti e velocità
  dello stesso interpolatore. Il setup AB06 legge un `.mot` con `inDegrees=yes`;
  la conversione è responsabilità dell'interpolatore condiviso. Coordinate
  mancanti e target non finiti sono errori. Non si applicano shift temporali,
  lookahead o mapping dal controlaterale.
- Actual e served reference sono confrontate separatamente col target;
  l'errore è normalizzato sulle escursioni prescribed sinistre. Formula e pesi
  imitativi riprendono la baseline imitativa: blend served/actual 0.8/0.2,
  pesi posizione knee/ankle 1/2 e velocità 0.02/0.04.
- GRF online fisiche sinistre e prescribed destre; profili fisico e analogico
  canonici, detector heel/toe V25 e FSM V26/v3 sono identici nelle due fasi.
  Il contratto eventi è `binary_point_v25+heel_qualified_fsm_v2`, con
  `binary_active` e `reject_continue`; detector ogni 1 ms, debounce 5 ms.
- Il selettore `reward_mode: imitation | ex_novo` attiva soltanto il rispettivo
  obiettivo. Penalità comuni, protezioni SEA, limiti articolari, penetrazione,
  validità degli eventi e timeout restano attivi. Premi task, clawback collegati
  e morphology non contribuiscono durante il pretraining. Metriche dell'obiettivo
  inattivo, anche non finite, non devono influenzare il ritorno.
  Anche le terminazioni specifiche morphology sono abilitate soltanto in
  ex-novo; il ledger e la relativa diagnostica restano disponibili in imitazione.

## Configurazioni riproducibili

I quattro YAML in `configs/` sono configurazioni complete. La provenienza e gli
SHA256 delle sorgenti sono in [configs/provenance.json](configs/provenance.json).
Ogni run salva `training_cfg.resolved.yaml` con override CLI e provenienza;
il rollout lo rilegge automaticamente dal checkpoint.

| Preset | Uso | Iterazioni | Batch / worker / minibatch / epoche |
|---|---|---:|---|
| `pretraining.yaml` (default) | PPO imitativo, actor nuovo | 40 | 4608 / 12 / 512 / 10 |
| `ex_novo.yaml` | PPO ex-novo | 50 totali | 4608 / 12 / 512 / 1 |
| `smoke_pretraining.yaml` | Verifica imitativa | 1 | 192 / 3 / 64 / 1 |
| `smoke_ex_novo.yaml` | Verifica transizione | 2 totali | 192 / 3 / 64 / 1 |

Ambiente e parametri ex-novo provengono dal run
`MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01`: morphology **5.0/5.0**,
sigma **0.010 congelata**, LR `5e-7`, clip PPO `0.05`, KL coefficiente `1.0`.
Il pretraining riprende LR `1e-4`, 10 epoche, clip `0.2`, KL coefficiente `0.2`
e logstd apprendibile; non applica il guard di KL per minibatch del pilot
ex-novo. Dal 14 settembre 2026, su indicazione dell'utente,
`grf_penetration_weight` vale **1.0 nel pretraining** (anche nel preset smoke)
e **0.05 nell'ex-novo**, riprendendo per il pretraining il peso del run V26
`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`. Gli altri coefficienti
delle penalità fisiche restano quelli ex-novo. Le soglie soft/hard restano
20/28 mm in entrambe le modalità; cambiare il peso non cambia l'actor.
L'eccezione e la provenienza sono salvate anche nei nuovi snapshot risolti.

Il pretraining usa PPO con reward imitativa: il target valuta la reference
servita e il movimento effettivo, senza essere un ingresso dell'actor. La
sincronizzazione con il prescribed in assenza di clock prescribed è una
questione aperta, registrata con priorità alta in [TODO.md](TODO.md).

Entrambe le fasi campionano in modo bilanciato i tre offset
`1.756870983805102`, `1.956870983805102`, `2.156870983805102` s: 1536 transizioni
per start nel batch completo, 64 nello smoke. Il bilanciamento riguarda la
raccolta del batch; con 10 epoche RLlib rimescola i minibatch delle epoche
successive. Lo smoke riduce soltanto dimensioni di raccolta e aggiornamento,
numero di processi e durata del training; fisica, reset e detector coincidono
con i preset completi.

## Comandi

Dalla root del repository, attivare l'ambiente della baseline con OpenSim,
plugin SEA, PyTorch e RLlib (`conda activate envCMC-rllib`). I comandi seguenti
sono identici in shell macOS e PowerShell Windows; i path relativi ai preset
sono risolti rispetto alla directory `unified_MLP`.

Pretraining da pesi nuovi:

```console
conda activate envCMC-rllib
python "Trajectory Generator/unified_MLP/train_ppo_mlp.py" --config configs/pretraining.yaml --output-dir pretraining_v1
```

L'interprete selezionato deve appartenere a `envCMC-rllib`: il supervisor
avvia il child con lo stesso Python. Un errore `ModuleNotFoundError: No module
named 'ray'` prima dell'inizializzazione richiede di controllare l'ambiente
usato dal comando, non di riprendere un checkpoint inesistente.

Passaggio diretto all'ex-novo:

```console
python "Trajectory Generator/unified_MLP/train_ppo_mlp.py" --config configs/ex_novo.yaml --init-actor-from "Trajectory Generator/unified_MLP/runs/training/pretraining_v1/rl_module_last" --output-dir ex_novo_v1
```

Il nuovo algoritmo crea critic e optimizer nuovi, verifica
`unified_contract.json`, carica i tensori dell'actor senza rimappature e applica
sigma `0.010` separatamente, controllando che la rete delle medie non cambi.
La prima iterazione è critic-only, con actor e logstd congelati; segue PPO.
Questa iterazione è inclusa in `--iterations`: il preset ex-novo da 50 produce
un warm-up e 49 aggiornamenti PPO. Nessun training lungo viene avviato
implicitamente dai test o dal passaggio di modalità.

Resume completo della stessa fase, aumentando il totale desiderato:

```console
python "Trajectory Generator/unified_MLP/train_ppo_mlp.py" --config configs/ex_novo.yaml --resume-from "Trajectory Generator/unified_MLP/runs/training/ex_novo_v1/checkpoint_last" --output-dir ex_novo_v1 --iterations 60
```

`--resume-from` ripristina algoritmo, optimizer e stato della fase senza
ripetere caricamento actor o warm-up. È distinto e incompatibile con
`--init-actor-from`, che serve a inizializzare un nuovo run. Reward e contratto
devono corrispondere alla fase salvata.

Smoke completo (una imitativa, una critic-only, una ex-novo):

```console
python "Trajectory Generator/unified_MLP/train_ppo_mlp.py" --config configs/smoke_pretraining.yaml --output-dir smoke_pretraining_v1
python "Trajectory Generator/unified_MLP/train_ppo_mlp.py" --config configs/smoke_ex_novo.yaml --init-actor-from "Trajectory Generator/unified_MLP/runs/training/smoke_pretraining_v1/rl_module_last" --output-dir smoke_ex_novo_v1
```

Rollout deterministico con traccia raw/served/actual e output del simulatore:

```console
python "Trajectory Generator/unified_MLP/rollout_eval.py" --checkpoint "Trajectory Generator/unified_MLP/runs/training/smoke_ex_novo_v1/rl_module_last" --episode-start-offset-s 1.756870983805102 --output-dir smoke_start_early --record-policy-trace
python "Trajectory Generator/unified_MLP/rollout_eval.py" --checkpoint "Trajectory Generator/unified_MLP/runs/training/smoke_ex_novo_v1/rl_module_last" --episode-start-offset-s 1.956870983805102 --output-dir smoke_start_center --record-policy-trace
python "Trajectory Generator/unified_MLP/rollout_eval.py" --checkpoint "Trajectory Generator/unified_MLP/runs/training/smoke_ex_novo_v1/rl_module_last" --episode-start-offset-s 2.156870983805102 --output-dir smoke_start_late --record-policy-trace
```

I risultati sono in `unified_MLP/runs/rollout/<nome>/`: summary, traccia policy,
configurazione risolta e `sim_outputs/`. `--no-record-outputs` disabilita i `.sto`;
`--max-steps` limita un rollout diagnostico. Il confronto del contratto verifica
nomi e ordine delle feature, classe del modulo, dimensioni e bounds delle azioni.
Il runtime effettivo deve coincidere con lo snapshot del checkpoint, anche usando
`--config` o `--no-auto-config`: cambiare filtro, detector o fisica genera un
errore. Restano modificabili start, durata e budget del rollout diagnostico.

Analisi offline del confronto pilot tra checkpoint 3 e 5 del 14 settembre,
dopo aver prodotto i sei rollout registrati nel relativo manifest:

```console
python "Trajectory Generator/unified_MLP/validation/compare_pretraining_checkpoints.py"
python "Trajectory Generator/unified_MLP/validation/plot_pretraining_checkpoints.py"
```

L'analisi usa solo la libreria standard; i grafici richiedono Matplotlib.
Gli script accettano `--rollout-root` e `--output-dir` e non avviano training
o simulazioni. I risultati del pilot con penetrazione 1.0, comprese le
limitazioni del cammino, sono nella [validazione aggiornata](VALIDATION.md#pilot-concluso-e-confronto-checkpoint-35--14-settembre-2026).

## Verifica e limiti

La [validazione eseguita il 6 settembre 2026](VALIDATION.md) documenta gli
82 test superati, lo smoke completo, il resume, la prova a 10 epoche e gli
esiti dei tre rollout, con collegamenti ai risultati originali.
Quelle prove live usavano ancora peso di penetrazione **0.05 nel pretraining**;
non costituiscono una validazione dinamica del preset aggiornato a **1.0**.
Gli snapshot e i checkpoint storici conservano la loro reward originale.
Per riprendere un vecchio run imitativo con `--resume-from` occorre usare
la sua configurazione risolta: il resume rifiuta cambiamenti della reward.

```console
python -m pytest "Trajectory Generator/unified_MLP/tests" -q
```

I test verificano preset/provenienza, isolamento degli output, contratto unico,
target sinistro, separazione delle reward e caricamento dell'actor. Lo smoke
verifica il percorso reale probe → worker → checkpoint → rollout, la modifica
del critic con actor congelato e il reload in un processo nuovo. I file sotto
`runs/` documentano le verifiche dinamiche effettivamente eseguite; un comando
riportato qui non costituisce da solo un risultato di test.

Una rete dopo una sola iterazione può terminare presto per le protezioni fisiche:
lo smoke non certifica la qualità del cammino. La verifica live della prima
consegna riguarda macOS arm64; la compatibilità Windows è predisposta con path
portabili e shim runtime della baseline e richiede esecuzione su Windows per
attestare il comportamento dinamico. Predittività e recupero del ritardo sono
rinviati alla versione successiva.
