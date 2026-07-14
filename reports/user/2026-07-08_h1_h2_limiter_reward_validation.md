# Validazione H1/H2 con target slew limiter

Data report: 2026-07-08

## Problema

Dopo l'introduzione del target slew limiter, il vecchio rollout H2 non falliva
piu' immediatamente per salti di target, ma restava bloccato prima del
`toe_off` protesico. Serviva quindi ripetere la validazione dinamica corretta:

1. rifare H1, cioe' training diagnostico breve da 10 iterazioni, con limiter
   attivo;
2. valutare il best checkpoint ottenuto con H2 registrato;
3. decidere se procedere a 20-50/100 iterazioni oppure fermarsi per revisione
   della reward.

Il criterio non era solo avere reward positiva, ma ottenere almeno progresso
FSM reale sul lato protesico/left:

```text
HS -> TO
```

senza terminare per `grf_penetration`.

## Strategia

E' stato mantenuto il piano di validazione progressivo:

- H1: training ex-novo breve, 10 iterazioni, stesso start allineato su HS
  sinistro e stesso YAML aggiornato con target slew limiter;
- H2: rollout registrato del best checkpoint H1, con output `.sto`, eventi
  online, trace policy/reward e summary JSON;
- decisione: bloccare training piu' lunghi se H2 non produce TO left/protesico.

Questa scelta evita di interpretare una metrica PPO positiva come prova di
camminata valida. Il rollout registrato resta il gate comportamentale.

## H1 - Training 10 iterazioni

Run:

```text
Trajectory Generator/runs/training/2026-07-07_H1_diag10_reward_validation_limiter/
```

Comando eseguito:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --output-dir "Trajectory Generator/runs/training/2026-07-07_H1_diag10_reward_validation_limiter" \
  --iterations 10
```

Risultato:

```text
ok = true
iterations_completed = 10/10
timed_out = false
best_iteration = 10
best_episode_return_mean = 1.8264632251286659
elapsed_wall_time_s = 4685.679895500012
```

Trend principale:

```text
iter 1  episode_return_mean = 1.380061  episode_len_mean = 31.510
iter 10 episode_return_mean = 1.826463  episode_len_mean = 34.308
```

Interpretazione:

- H1 e' tecnicamente PASS: training completo, checkpoint best/last prodotti,
  nessun timeout e nessun NaN nelle metriche PPO principali.
- Il limiter migliora nettamente il training rispetto al vecchio H1 negativo.
- Il problema dinamico non e' risolto: la lunghezza media episodio resta molto
  corta, circa 0.34 s a 100 Hz, quindi serve H2.

Checkpoint best:

```text
Trajectory Generator/runs/training/2026-07-07_H1_diag10_reward_validation_limiter/rl_module_best
```

## H2 - Rollout best H1

Run:

```text
Trajectory Generator/runs/rollout/2026-07-08_H2_H1_limiter_best_recorded/
```

Comando eseguito:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/rollout_eval.py" \
  --checkpoint "Trajectory Generator/runs/training/2026-07-07_H1_diag10_reward_validation_limiter/rl_module_best" \
  --output-dir "Trajectory Generator/runs/rollout/2026-07-08_H2_H1_limiter_best_recorded" \
  --record-outputs \
  --no-progress \
  --startup-timeout-s 600 \
  --step-timeout-s 180 \
  --stall-timeout-s 300 \
  --run-timeout-s 3600
```

Risultato summary:

```text
ok = true
steps = 45
episode_return = 3.7272192931149526
reward_mean = 0.08282709540255451
reward_min = -2.2327904749774614
reward_max = 0.24131053738963598
terminated = true
truncated = false
action_clipped_fraction = 0.011111111111111112
pelvis_ty_min = 0.9602307489760281
```

Eventi online:

```text
13.946870983805102 -> left  heel_strike, confirmed 13.997870983805074
13.946870983805102 -> right heel_strike, confirmed 13.997870983805074
14.200870983804961 -> right toe_off
```

Non compare nessun `toe_off` left/protesico.

Trace finale:

```text
phase_valid_hs_count = 1.0
phase_valid_to_count = 0.0
phase_valid_cycle_count = 0.0
phase_stance_elapsed_s = 0.4499999999997506
grf_penetration_m = 0.017142811510407953
grf_penetration_loss = 1.0579404092633808
safety_loss = 1.0
prosthetic_normal_force_bw = 0.5521064605776393
target_slew_limited_fraction = 0.5
```

Interpretazione:

- H2 e' tecnicamente valido: checkpoint caricabile, output registrati,
  watchdog pulito.
- H2 e' comportamentalmente FAIL: non produce `HS -> TO` sul lato
  protesico/left.
- Il nuovo H1 migliora la durata del rollout rispetto al vecchio H2 con limiter
  (`45` step contro `27`), ma la policy resta in stance e termina ancora per
  `grf_penetration`.

## Decisione

Non procedere a training 20-50 o 100 iterazioni nello stato attuale.

Il problema non e' piu' il fake gait cycle originale e non e' solo il salto di
target al reset. Il limiter ha mitigato i salti e ha reso H1 apprendibile, ma
la reward continua a permettere una scorciatoia dinamica:

```text
restare in stance -> accumulare carico/contact-load -> aumentare penetrazione
-> terminare per safety prima del TO protesico
```

Dato che la stance prescritta media nella finestra di validazione e' circa
1.09 s, non bisogna forzare un TO a 0.45 s. Bisogna invece impedire che nei
primi 0.45 s la policy venga premiata mentre comprime progressivamente il
contatto fino alla soglia di penetrazione.

## File modificati

Nessun file di codice e' stato modificato in questa fase.

File di piano aggiornato:

```text
reports/plans/2026-07-02_piano_validazione_reward_exnovo.md
```

Artefatti principali generati:

```text
Trajectory Generator/runs/training/2026-07-07_H1_diag10_reward_validation_limiter/
Trajectory Generator/runs/rollout/2026-07-08_H2_H1_limiter_best_recorded/
```

## Verifiche eseguite

- Training H1 10 iterazioni: PASS tecnico.
- Checkpoint best/last H1: prodotti.
- Rollout H2 registrato: PASS tecnico.
- Output `.sto` H2: prodotti in `sim_outputs/`.
- Eventi online H2: letti e verificati.
- Trace reward H2: letto e verificato.
- Gate comportamentale H2: FAIL per assenza di TO left/protesico.
- `git diff --check` sul piano: PASS.

## TODO

1. Modificare la reward early-stance per penalizzare/limitare il caricamento
   progressivo che porta a `grf_penetration`, senza forzare un TO precoce.
2. Rendere il `contact_load_score` meno sfruttabile: premiare supporto stabile
   solo se penetrazione e carico restano entro un profilo fisico ragionevole.
3. Ripetere H1 solo dopo la revisione reward.
4. Ripetere H2 e richiedere almeno `HS -> TO` left/protesico prima di sbloccare
   training 20-50 iterazioni.
5. Tenere bloccato il training da 100 iterazioni finche' H2 e il gate
   intermedio non passano.
