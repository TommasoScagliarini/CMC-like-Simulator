# Reward ex-novo prescribed-free, fase protesica e critic privilegiato - 2026-06-25

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo la creazione di `training_exnovo_cfg.yaml`, il primo lavoro sulla reward
`ex_novo` e' stato chiarire quali termini fossero ancora legati ai dati
prescribed/IK e quali informazioni restassero necessarie per rendere il task
apprendibile.

Sono emersi due problemi principali:

1. La reward `ex_novo`, dopo aver rimosso `reference_score` e `bio_score`, e'
   molto povera e deve essere sostituita/rafforzata con termini task-based che
   non usino dati prescribed.
2. Se non si vogliono usare i dati della sound leg, anche la definizione della
   fase del gait cycle deve migrare verso una fase protesica online, basata su
   heel strike/toe off della protesi rilevati dalla GRF online.

In piu' e' stato discusso l'effetto del critic privilegiato/onnisciente: anche
se non passa direttamente informazioni all'actor, puo' influenzare il training
tramite gli advantage.

## Stato attuale del config ex-novo

File:

```text
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Modifiche applicate:

```yaml
reward:
  reward_mode: ex_novo
  blend_reference: 0.0
  blend_bio: 0.0
```

Quindi la parte positiva della reward e' ora:

```text
reward_base = clip(
    0.25 * tracking_score
  - penalty,
  0.0,
  1.0
)
```

`reference_score` e' disattivato: non viene piu' premiata la vicinanza dei DOF
protesici alla IK/prescribed base.

`bio_score` e' disattivato: non viene piu' premiato il tracking del contesto
biologico rispetto alla IK/prescribed base.

Restano attive le penalita':

```text
OOB commanded-reference band
SEA effort/saturation/torque-error/motor stress
command-rate/smoothness
safety
GRF penetration
GRF ankle moment flip guard
```

## Nuova penalty GRF

La penalty GRF introdotta nei lavori precedenti e' gia' inclusa nel config
ex-novo:

```yaml
grf_penetration_weight: 0.5
grf_ankle_moment_flip_weight: 0.05
grf_ankle_moment_flip_tau_tol_nm: 8.0
grf_ankle_moment_flip_force_threshold_n: 50.0
```

Interpretazione:

- `grf_penetration_weight` penalizza la penetrazione del piede protesico nel
  contatto online;
- `grf_ankle_moment_flip_weight` penalizza momenti GRF alla caviglia di segno
  sospetto, associati al ribaltamento COP/contact patch.

## Reward attuale: limite principale

Con `reference_score` e `bio_score` spenti, la reward e' prescribed-free ma
insufficiente:

```text
Premia:
- reference protesica generata che il plant/SEA riesce a seguire.

Penalizza:
- comandi fuori banda;
- stress/saturazione SEA;
- contatto GRF invalido;
- stati unsafe.
```

Ma non premia ancora direttamente:

```text
cammino
carico protesico utile
stance/swing organizzati
propulsione
regolarita' del ciclo
uso meccanico funzionale del SEA
stabilita' dinamica oltre i guardrail
```

Quindi il prossimo passaggio deve introdurre termini task-based non prescribed.

## Termini task-based candidati

Candidati iniziali discussi:

- **contact/load reward**: premiare carico verticale protesico in stance, evitando
  la soluzione degenere a carico quasi nullo;
- **swing unloading**: penalizzare GRF protesica elevata durante swing;
- **contact validity**: continuare a usare penetration e ankle-moment flip guard;
- **phase/event regularity**: premiare sequenze HS -> TO -> HS ragionevoli, senza
  imporre timing sano prescribed;
- **prosthetic joint range**: mantenere OOB e valutare bande biomeccaniche piu'
  informative;
- **SEA usability**: penalizzare saturazione, chattering e power/stress assurdi,
  senza target torque prescribed;
- **stability/survival**: pelvis height e stati non divergenti;
- **reserve/residual cost**: possibile costo globale, da usare con cautela perche'
  le reserve possono riflettere anche limiti del lato sano/prescribed.

## Problema della fase del gait cycle

Nel framework attuale l'actor puo' ancora ricevere:

```text
gait_phase_sin
gait_phase_cos
```

Questa fase deriva dal clock sound-side/prescribed. Se l'obiettivo e' eliminare
la dipendenza dai dati della sound leg, questa fase non e' piu' coerente con la
claim `ex_novo`.

La soluzione naturale e' costruire una fase protesica online:

```text
phi_pros = fase stimata da eventi protesici online
```

basata su:

```text
heel strike protesico
toe off protesico
GRF online protesica
```

Schema iniziale:

```text
HS_pros       -> phi = 0
TO_pros       -> fine stance / inizio swing
next HS_pros  -> chiusura ciclo
```

con stima del periodo:

```text
T_hat = media mobile degli intervalli HS_pros -> HS_pros
phi = (t - last_HS_pros) / T_hat
```

e duty factor:

```text
stance_fraction_hat = (TO - HS) / T_hat
```

Problemi aperti:

- fase non definita prima del primo HS protesico;
- eventi rumorosi o contatto spurio;
- necessita' di debounce/durata minima contatto;
- fallback se un ciclo viene saltato;
- rischio che la policy manipoli GRF/eventi per manipolare la fase.

Decisione concettuale: la fase protesica va introdotta come segnale actor-side
realistico e/o come gating reward, ma inizialmente con pesi deboli e diagnostica
forte.

## Critic privilegiato / onnisciente

Con asymmetric actor-critic:

```text
actor  -> legge obs[:n_actor]
critic -> legge obs[:n_full]
```

Il critic non passa direttamente al policy network le feature privilegiate in
inference. Tuttavia influenza il training tramite:

```text
advantage = return - V_critic(full_obs)
```

Quindi un critic che vede il gait cycle sano/prescribed puo' ridurre la varianza
del value estimate, ma puo' anche creare una forma di credit assignment non
coerente con cio' che l'actor puo' osservare.

Impatto sul problema fase:

```text
critic vede fase sana:
  puo' stimare meglio il valore dello stato;

actor non vede fase sana:
  non puo' usare quella fase per scegliere l'azione;

risultato:
  non insegna magicamente una fase protesica all'actor.
```

Il rischio aumenta se due stati indistinguibili per l'actor richiedono azioni
diverse distinguibili solo dalla feature privilegiata del critic.

Per una claim ex-novo pulita si propone:

- actor prescribed-free;
- reward prescribed-free;
- critic privilegiato eventualmente mantenuto, ma senza usare target sano o fase
  sana prescribed se la claim e' "senza dati prescribed/sound-leg".

## File modificati

Modificato:

```text
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Creato:

```text
reports/user/2026-06-25_reward_exnovo_prescribed_free_fase_protesica_critic.md
```

Nessuna modifica al plugin C++, alla semantica SEA o al simulatore root.

## Verifiche eseguite

- Verificato che `training_exnovo_cfg.yaml` contiene:

  ```text
  reward_mode: ex_novo
  blend_tracking: 0.25
  blend_reference: 0.0
  blend_bio: 0.0
  ```

- Verificato via loader:

  ```text
  ex_novo 0.25 0.0 0.0
  ```

- Verificato `git diff --check` dopo le modifiche al config.

## TODO

- [ ] Definire i primi termini task-based prescribed-free da aggiungere alla
      reward ex-novo.
- [ ] Decidere se rimuovere dall'actor `gait_phase_sin/cos` sound-side e
      sostituirli con una fase protesica online.
- [ ] Implementare o esporre una fase protesica online basata su HS/TO protesici
      da GRF online.
- [ ] Gestire il periodo iniziale prima del primo HS protesico.
- [ ] Aggiungere diagnostica su affidabilita' HS/TO protesici online,
      contatti spuri e cicli saltati.
- [ ] Valutare se il critic privilegiato deve mantenere o perdere la fase sana
      prescribed per coerenza con la claim ex-novo.
- [ ] Prima di training lunghi, eseguire rollout diagnostici con reward
      prescribed-free attuale per misurare degenerazioni attese: zero load,
      no-contact, clipping, reserve e fallimenti.
- [ ] Aggiornare i report futuri distinguendo:
      reward prescribed-free, actor prescribed-free e critic prescribed-free,
      perche' sono tre livelli diversi.
