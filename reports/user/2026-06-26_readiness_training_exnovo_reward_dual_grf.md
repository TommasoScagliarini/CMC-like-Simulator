# 2026-06-26 - Readiness training ex-novo con reward task-based e dual GRF

## Problema

L'obiettivo era controllare in modo accurato se il `Trajectory Generator`, e in
particolare la baseline MLP, fosse pronto per avviare un training in modalita'
ex-novo dopo l'introduzione della reward task-based sui punti 1-6.

Durante l'audit sono emersi due blocchi concettuali/tecnici:

1. la reward di fase avrebbe usato lo stesso profilo GRF scelto per sostenere la
   dinamica della simulazione;
2. l'actor ex-novo aveva ancora nel prefisso osservativo il gait clock sano
   prescritto, nato come pacemaker interim ma non coerente con una claim
   prescribed-free.

## Soluzione

E' stata introdotta una separazione esplicita tra:

- `grf_correct_magnitude`: profilo online-GRF usato per il supporto fisico della
  simulazione, applicato sul lato protesico sinistro;
- `grf_detector_HS-TO`: profilo online-GRF caricato come stream sensor-only e
  usato solo per eventi HS/TO e fase protesica.

In `training_exnovo_cfg.yaml` la sezione GRF ora dichiara entrambi:

```yaml
online_grf_profile: online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
online_grf_detector_profile: online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
```

Inoltre, nel cfg ex-novo e' stato disattivato il gait clock sano:

```yaml
gait_clock_enable: false
```

L'actor mantiene uno schema stabile, ma le feature del clock sano diventano
costanti (`sin=0`, `cos=1`). La fase utile per reward/actor viene quindi dal
segnale protesico online: `online_left_gait_phase_*`, `online_left_heel_strike`,
`online_left_toe_off`, `online_left_in_contact`.

## Strategia

Il controllo e' stato eseguito dal basso verso l'alto:

1. verifica parsing YAML e propagazione config;
2. verifica reward ex-novo e pesi task-based;
3. verifica schema osservativo actor/critic;
4. verifica caricamento doppio profilo online-GRF;
5. env probe con reset + step;
6. smoke PPO minimale per controllare RLlib, modulo asimmetrico, sampling,
   learner e checkpoint.

La modifica del dual-profile e' stata implementata senza cambiare la semantica
del plugin SEA e senza applicare il profilo detector alla dinamica.

## File modificati

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - esposti i pesi reward ex-novo punti 1-6;
  - aggiunto `online_grf_detector_profile`;
  - aggiunto `gait_clock_enable: false`.
- `Trajectory Generator/baseline_MLP/reward_function.py`
  - reward task-based ex-novo: contact/load, swing unloading, contact validity,
    phase regularity/timing, joint range, SEA usability.
- `Trajectory Generator/baseline_MLP/training_config.py`
  - parsing YAML di `online_grf_detector_profile` e `gait_clock_enable`.
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
  - propagazione dei nuovi campi all'env;
  - summary run con profilo detector e stato gait clock.
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - ricostruzione rollout coerente con training per detector profile e gait
    clock.
- `Trajectory Generator/osim_trj_cmc_like.py`
  - aggiunto campo env `online_grf_detector_profile_file`;
  - applicazione override su `SimulatorConfig`.
- `config.py`
  - nuovo campo `online_grf_detector_profile_file`.
- `path_resolver.py`
  - risoluzione path del profilo detector.
- `online_grf.py`
  - supporto a prefisso nome custom per contatti online-GRF.
- `model_loader.py`
  - caricamento del profilo fisico e del profilo detector come due set di forze
    distinti;
  - detector sempre sensor-only.
- `simulation_runner.py`
  - eventi HS/TO calcolati sul detector stream se presente;
  - payload `online_grf` fisico lasciato invariato.
- `validation/test_reward_function.py`
  - test aggiornati al cfg imitativo corrente e al nuovo cfg ex-novo.
- `validation/validate_training_config.py`
  - validazione del layout attuale: default imitativo, ex-novo esplicito,
    detector profile e gait clock disabilitato.

## Verifiche eseguite

Parsing e test leggeri:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python -m py_compile ...
validation/test_reward_function.py: PASS
validation/validate_training_config.py: PASS
git diff --check: PASS
```

Probe env ex-novo:

```text
reset_ok
n_actor = 31
n_obs = 76
gait_clock available = False
sound_clock_features = 0.0, 1.0
actor_has_imitation_target = False
profile = AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
detector_profile = AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
force_counts = 20 physical/sensor contacts, 4 detector contacts
```

Smoke PPO minimale:

```text
output_dir = Trajectory Generator/runs/training/_exnovo_smoke_audit
ok = true
stop_reason = completed
iterations_completed = 1
num_env_runners = 0
sampled env steps = 2
checkpoint_last scritto
```

Lo smoke PPO non serve come risultato scientifico: verifica solo che la catena
config -> env -> RLModule asimmetrico -> PPO learner -> checkpoint funzioni.

## Verdetto

Il `Trajectory Generator/baseline_MLP` e' pronto per avviare un training
ex-novo reale con:

- actor senza target imitativo sano;
- actor senza pacemaker di gait sano prescritto;
- critic privilegiato ancora disponibile tramite observation suffix;
- GRF fisica separata dal detector HS/TO;
- reward task-based punti 1-6 configurata nel cfg ex-novo.

Comando base:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"
```

## TODO

- Monitorare nel primo training reale i componenti `contact_load_score`,
  `phase_regular_score`, `phase_period_loss`, `phase_timeout_loss`,
  `swing_unloading_loss`, `grf_penetration_loss`, `grf_ankle_moment_flip_loss` e
  i termini SEA per capire se i pesi sono bilanciati.
- Disegnare in un secondo pass il punto 7 della reward, cioe' la morfologia
  cinematica protesica, senza introdurre una traiettoria prescribed.
- Tenere `T_nom` fisso/model-based AB06 treadmill nel primo training; rimandare
  la stima dinamica da velocita' di cammino protesica a un pass separato,
  filtrato e clampato in range clinico/biomeccanico.
