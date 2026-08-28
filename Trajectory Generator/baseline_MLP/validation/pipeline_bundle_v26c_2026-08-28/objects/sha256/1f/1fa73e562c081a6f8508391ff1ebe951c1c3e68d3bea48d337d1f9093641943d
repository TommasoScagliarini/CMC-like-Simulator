# Ablation dello stato del controller nell'actor

> **Stato superato.** Questo report documenta il primo candidato 35-feature,
> correttamente rifiutato con il protocollo allora disponibile. La successiva
> diagnosi ha trovato label recovery fuori fase e input non condizionati. La
> decisione finale aggiornata e' in
> `2026-07-13_sblocco_actor_markov35_e_warmup_critic.md` e nel gate
> `validation/controller_memory_ablation/2026-07-13_markov35_final_gate.json`.

## Problema

I report storici del 14 giugno e del 1 luglio descrivevano lo stato del reference governor e del comando SEA come stato Markov deployable o memoria leggera. La rimozione precedente di tutte queste feature aveva quindi eliminato non solo un possibile shortcut imitativo, ma anche informazione interna del controllore disponibile in deployment.

L'obiettivo di questa ablation era verificare se reintrodurre una parte selezionata di tale stato rendesse l'actor piu robusto senza esporre dati prescribed e senza compromettere il warm start gia validato.

## Contratti confrontati

La baseline selezionata usa 25 feature actor: cinematica protesica, stato SEA, GRF online e FSM. Lo stato del controller e i dati prescribed restano nel suffisso privilegiato del critic.

Il candidato usa 35 feature actor. Alle 25 feature aggiunge, per ginocchio e caviglia:

- endpoint precedente della policy;
- riferimento servito `q`, `qdot` e `qddot`;
- ultimo comando SEA grezzo.

Le quattro diagnostiche derivate `abs(u)` e saturazione restano critic-only, perche non aggiungono stato rispetto al comando grezzo. Nessuna feature prescribed e stata aggiunta all'actor.

## Strategia

1. E stato introdotto un flag separato per distinguere lo stato Markov del controller dalle diagnostiche ridondanti.
2. L'actor da 25 feature e stato espanso a 35 azzerando le dieci nuove colonne del primo layer. Tutti i pesi condivisi sono rimasti invariati; il rollout deterministico iniziale ha mantenuto 500 step e 2 cicli.
3. Sono state raccolte tre trace a `sigma=0.005`, una trace nominale e due dataset teacher con start spostato di `-0.20 s` e `+0.20 s`. Entrambi i teacher hanno completato 500 step e 2 cicli, dimostrando che gli start erano fisicamente eseguibili.
4. Durante l'adattamento sono rimaste addestrabili solo le dieci nuove colonne del primo layer. Tutti gli altri pesi actor e `logstd` sono rimasti congelati.
5. Due candidati sono stati rifiutati dal gate nominale: scostamento massimo rispettivamente `0.03570` e `0.00611`, contro il limite `0.005`.
6. Il candidato conservativo finale, con 96 repliche dei 500 stati nominali, ha superato il gate con scostamento RMS `0.001336` e massimo `0.004909`.
7. Il candidato e stato confrontato in closed loop con la baseline a schema identico su nominale, tre seed stocastici, rumore correlato per 50 ms e due start alternativi.

## Risultati A/B

| Test | Baseline | Candidato 35 | Esito |
|---|---:|---:|---|
| Deterministico nominale | 500 step, 2 cicli, return 64.04 | 500 step, 2 cicli, return 63.19 | lieve regressione |
| `sigma=0.005`, 3 seed | 3/3 pass, return medio 64.77 | 3/3 pass, return medio 63.43 | regressione media 1.34 |
| Rumore held 50 ms, seed 123 | 500 step, 2 cicli, return 67.80 | 500 step, 3 cicli, return 73.37 | miglioramento |
| Start `-0.20 s` | stop passo 51, 0 cicli | stop passo 48, 0 cicli | candidato peggiore |
| Start `+0.20 s` | 500 step, 3 cicli | stop passo 52, 0 cicli | regressione decisiva |

Il candidato migliora la risposta a una perturbazione locale temporalmente correlata, ma non generalizza al cambio di fase iniziale. In particolare fallisce lo start `+0.20 s`, che la baseline completa con tre cicli. I dati teacher usati offline non sono stati sufficienti a insegnare il recupero closed-loop dagli stati visitati dall'actor a quello start.

## Decisione

Il candidato da 35 feature e rifiutato. `training_exnovo_cfg.yaml` resta sul contratto deployable da 25 feature:

- stato del controller actor-visible: no;
- diagnostiche del controller actor-visible: no;
- dati prescribed actor-visible: no;
- stato controller e dati prescribed disponibili al critic privilegiato: si.

Non e stato eseguito il warm-up del critic sul candidato rifiutato. Rimane selezionato il checkpoint actor `target_domain_warm_start_deployable_sigma0005_2026-07-13/rl_module_warm_start`, digest `5616be85...2bf7`, con critic gia riscaldato in `validation/critic_warmup/2026-07-13_deployable_sigma0005_iter1/checkpoint_last`. In quel warm-up l'actor e rimasto identico, l'explained variance del critic ha raggiunto `0.598` e il checkpoint e stato ripristinato con digest critic coerente.

Il risultato non dimostra che lo stato Markov sia intrinsecamente inutile. Dimostra che questa reintroduzione e questo adattamento offline non soddisfano il gate di robustezza. Un eventuale nuovo tentativo dovra raccogliere DAgger closed-loop dagli start randomizzati usando gli stati effettivamente visitati dall'actor, non soltanto traiettorie teacher coerenti.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/target_domain_imitation.py`
- `Trajectory Generator/baseline_MLP/target_domain_markov_adaptation.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/README.md`
- `validation/validate_training_config.py`
- `validation/test_target_domain_imitation.py`
- `validation/controller_memory_ablation/2026-07-13_markov35_ablation_gate.json`

## Verifiche

- smoke test della configurazione: tutti i controlli passati;
- suite unit test: 58 test passati;
- audit pesi candidato: soli ingressi Markov modificati, shared weights e `logstd` invariati;
- rollout A/B: 14 episodi riassunti nel gate JSON;
- `git diff --check`: nessun errore di whitespace.
