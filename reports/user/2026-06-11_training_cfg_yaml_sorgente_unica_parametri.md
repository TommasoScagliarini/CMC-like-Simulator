# training_cfg.yaml — sorgente unica dei parametri di training

**Data:** 2026-06-11
**Perimetro:** `Trajectory Generator/baseline_MLP/` (nessuna modifica a SEA, plugin C++, o semantica del comando SEA)

## Problema

Il training MLP/RLlib esponeva **~40 flag CLI** in `train_ppo_mlp.py` e **~30** in
`rollout_eval.py`. Conseguenze:

- Comandi di lancio lunghissimi e fragili (vedi `commands.txt` storico).
- Il **rollout doveva ri-specificare a mano ~14 parametri che DEVONO combaciare**
  con il training (`action_mode`, `grf_*`, `reward_*`, rete, critic asimmetrico).
  Il checkpoint salva solo `logical_iteration`, **non** la config dell'env/rete →
  un mismatch silenzioso tra training e rollout non veniva rilevato.

Obiettivo richiesto: un unico `training_cfg.yaml` dentro `baseline_MLP/` che
raduni tutti i parametri di rete e simulazione e diventi il riferimento per
lanciare un training; tutti gli script devono leggere i valori da quel file.

## Soluzione

Un file YAML canonico + un loader condiviso, con due decisioni confermate
dall'utente in fase di piano:

1. **Precedenza YAML → CLI**: il YAML fornisce i *default* di ogni parametro; un
   flag CLI esplicito **vince** sul YAML (retro-compatibile coi comandi esistenti).
2. **Rollout auto-match via snapshot**: il training scrive la config **risolta**
   (YAML + override CLI) in `<output_dir>/training_cfg.resolved.yaml`; il rollout,
   dato `--checkpoint`, la **carica automaticamente** dalla cartella del run, così
   env/rete/reward/`action_mode` combaciano sempre con ciò che è stato allenato.

Recepiti inoltre 4 affinamenti richiesti sul contenuto:

- **Rete come `num_hidden_layers` × `dim_hidden_layers`** (larghezza uniforme) →
  internamente `fcnet_hiddens = [dim] * num`; esposto anche `fcnet_activation`.
- **Rinomina** `critic_privileged_observation` → **`asymmetric_actor_critic`**
  (chiave YAML + flag `--asymmetric-actor-critic`); il campo interno
  `CMCEnvConfig.critic_privileged_observation` resta invariato (nessun tocco a env/RLModule).
- **`action_mode` fuori dal YAML**: verificato nel codice
  ([osim_trj_cmc_like.py:516]) che `delta`/`raw` sono ormai **solo diagnostici**;
  `absolute` è l'unica modalità di produzione → `action_mode`/`max_delta_rad`
  restano flag CLI per diagnostica, non nel file.
- **oob bounds fuori dal YAML** (si usano i default di `RewardConfig` o `--reward-json`).

Dipendenza: `PyYAML 6.0.3` è **già presente** in `envCMC-rllib` (verificato) — nessuna aggiunta.

## Strategia (implementazione)

- **Loader `training_config.py`**: `SECTION_MAP` (sezione → dest argparse, con
  coercizione tipi) come single source per flatten e dump; `to_argparse_defaults`
  (YAML → dict di default + reward a parte); `dump_resolved` (snapshot dai valori
  risolti su `args`, escludendo la sezione `run` di run-identity); `load_resolved_for_checkpoint`
  (risale da `rl_module_*`/`checkpoint_*` alla cartella del run).
- **Two-pass argparse**: pre-parser legge solo `--config`, carica lo YAML, poi
  `parser.set_defaults(**flat)` prima di `parse_args()` → i flag CLI espliciti
  vincono "gratis". La passthrough `sys.argv[1:]` del supervisor porta `--config`
  e ogni override al child, che ri-parsa lo stesso YAML → coerente sui restart.
- **Reward**: precedenza `reward:` del YAML < `--reward-json` < `--reward-mode`.
- **GRF-side**: `action="append"` → `nargs="*"` per una precedenza YAML/CLI pulita
  (il flag sostituisce la lista del YAML invece di appenderla).
- **Rollout**: i default per i parametri condivisi vengono da snapshot < `--config`
  < built-in; CLI esplicito vince; `--no-auto-config` disattiva l'auto-load.

## File modificati / aggiunti

```text
NUOVI
  Trajectory Generator/baseline_MLP/training_cfg.yaml            (config canonica)
  Trajectory Generator/baseline_MLP/training_config.py          (loader condiviso)
  validation/validate_training_config.py                        (test regressione loader)

MODIFICATI
  Trajectory Generator/baseline_MLP/train_ppo_mlp.py            (two-pass, num/dim, rename, reward merge, snapshot)
  Trajectory Generator/baseline_MLP/rollout_eval.py            (--config/--no-auto-config, auto-match, rename, nargs)
  Trajectory Generator/baseline_MLP/commands.txt              (comandi YAML-based, regole d'oro)
  Trajectory Generator/baseline_MLP/README.md                (sezione Configurazione)
```

Nessuna modifica al plugin C++ SEA, alla semantica del comando SEA, o all'env.

## Test / verifiche eseguite

- **`py_compile`** di `training_config.py`, `train_ppo_mlp.py`, `rollout_eval.py`: OK.
- **Regressione loader** (`validation/validate_training_config.py`, no OpenSim/Ray):
  **28/28 PASS** — precedenza YAML/CLI, alias, espansione `num/dim → [256,256]`,
  `nargs` GRF-side, round-trip snapshot (`dump_resolved` → `load_resolved_for_checkpoint`),
  alias deprecato `--fcnet-hiddens`.
- **Funzionale A** (solo YAML, single-process, 1 iter): `ok:true`; snapshot
  `training_cfg.resolved.yaml` scritto correttamente (rete num/dim, `action_mode`/`max_delta_rad`
  per il round-trip, reward risolta, sezione `run` omessa).
- **Funzionale B** (override CLI): snapshot registra `lr 0.0005`, `grf prescribed`,
  `dim_hidden_layers 128`, `online_grf_applied_side []` → gli override vincono.
- **Funzionale C** (rollout auto-match, **nessun flag di env**): env `prescribed`/no-online-obs
  ricostruito dallo snapshot, `reward_mean 0.69`, nessun mismatch.
- Verifiche fatte con `--num-env-runners 0` per **non interferire** col training
  imitativo a 12 worker in esecuzione; run temporanei `_cfg_verify_*` rimossi.

### Bug trovato e corretto durante la verifica
La verifica end-to-end (Funzionale C) ha scoperto un bug reale: il lookup dello
snapshot usava il path grezzo `runs\…` invece di risolverlo a
`Trajectory Generator\runs\…` (convenzione del 2026-06-09) → snapshot non trovato,
fallback ai default built-in, mismatch obs `1x28` vs rete `22x128`. **Corretto**
riusando `_resolve_input_path` nel pre-parser del rollout; ri-verificato OK.

### Protezione del training in corso
La rimozione di `--fcnet-hiddens` avrebbe rotto il run imitativo attivo al
prossimo auto-restart (argparse "unrecognized arguments"). Aggiunto
`--fcnet-hiddens` come **alias deprecato nascosto** (convertito internamente in
`hiddens`), che preserva il run in corso e i comandi storici.

## Uso

```powershell
# training di riferimento: tutti i parametri da training_cfg.yaml
... train_ppo_mlp.py --output-dir runs\baseline_mlp_hybrid_win
# override puntuale (vince sul YAML)
... train_ppo_mlp.py --output-dir runs\... --lr 5e-4 --reward-mode imitation
# rollout: env/rete/reward auto dallo snapshot del run
... rollout_eval.py --checkpoint runs\<run>\rl_module_best --output-dir runs\..._rollout --record-outputs
```

## TODO aperti / follow-up (propagare nel daily)

- **macOS**: valutare un `training_cfg.mac.yaml` (5 worker / 6 cpu / batch 2048)
  da passare con `--config`, al posto degli override CLI mostrati in `commands.txt`.
- **Pulizia futura alias deprecati**: `--fcnet-hiddens` e
  `--critic-privileged-observation` sono mantenuti per retro-compatibilità; rimuoverli
  quando i comandi/launcher storici (`run_imit_*.ps1`) e i checkpoint in volo saranno esauriti.
- (Pre-esistente) Pulire launcher/log temporanei `run_imit_*.ps1`, `imit_*.log` a
  run imitativo concluso.

[osim_trj_cmc_like.py:516]: ../../Trajectory%20Generator/osim_trj_cmc_like.py
