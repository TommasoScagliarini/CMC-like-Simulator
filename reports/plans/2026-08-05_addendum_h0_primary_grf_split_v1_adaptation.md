# Addendum — adattamento H0 a `primary_grf_split_v1`

Data: 2026-08-05

## Provenienza e scopo

Questo addendum apre un ramo nuovo e circoscritto dopo l'esito terminale
`ERROR_H0_REFERENCE` del preflight H0/V25 A-B-C. Non riscrive e non riapre la
matrice consumata.

Base di autorizzazione esplicita:

> Per proseguire servirà una nuova autorizzazione per adattare H0 alla
> semantica `primary_grf_split_v1`, senza ripristinare il detector come
> sorgente del carico continuo.

Istruzione utente successiva: `fallo`.

Il candidato si chiama `H0_primary_split_v1`. Non è `H0_sep`: quest'ultimo
resta legato al target combinato detector/primary e al vecchio ramo C.

Output massimo di questo addendum:

```text
H0_PRIMARY_GRF_SPLIT_V1_BASELINE_READY
```

Non autorizza `H0_BINARY_V25_CANDIDATE_READY`, trial protetti, corridor o PPO.

## Invarianti

- H0 sorgente resta immutabile.
- Layout actor/full `35/84` e ordine delle feature restano invariati.
- `online_left_normal_grf_bw` e `online_left_in_contact` runtime provengono
  esclusivamente dalla GRF primaria.
- Il detector analogico può essere letto soltanto dal teacher offline; non può
  sostituire la GRF primaria nel runtime del candidato.
- Eventi HS/TO durante questo ramo: `legacy_events`.
- V25 non è actor input e V20 resta disabilitata.
- `morphology_weight=0`.
- Plugin C++, SEA, primary GRF, detector e relative geometrie restano
  intoccati.
- Nessun actor prescritto, PPO, morphology reward positivo o dato protetto.
- Trial 05/06 e reserve 03/07 restano chiusi.
- Il ramo A/B/C terminato resta no-retry e no-clobber.

## Sorgente e target semantico

Sorgente H0:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last
```

Configurazione sorgente:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/
training_cfg.resolved.yaml
```

Target:

```text
primary_grf_split_v1+legacy_events_v1
```

La testa logstd deve restare bit-identica a H0. L'intera rete che produce le
medie è adattabile; critic e altri tensori del modulo inference restano
immutati durante il fit supervisionato.

## Teacher controfattuale e viste paired

Ogni campione deriva da un singolo stato fisico corrente:

1. `student_view`: prefisso actor corrente, con carico/contatto della GRF
   primaria;
2. `teacher_view`: copia della stessa osservazione che ricostruisce la semantica
   H0 pre-split, sostituendo gli indici 10 e 11 con:
   - `online_grf_detector.left.normal_force / body_weight_n`;
   - `online_grf_detector.left.in_contact`;
   e gli otto indici 17:25 con l'osservazione di una seconda
   `ProstheticPhaseFSM` indipendente, alimentata dagli stessi eventi legacy ma
   con load/contact continui del detector analogico;
3. la FSM runtime/student resta quella corrente, alimentata dalla GRF primaria;
4. `teacher_mean`: media deterministica di H0 sulla `teacher_view`;
5. l'ambiente viene avanzato con `teacher_mean + sigma*z`, mai con l'azione
   dello student.

Le altre 25 feature devono restare bit-identiche tra le due viste. Il collector
deve verificare a ogni step che:

- la vista student coincida con la GRF primaria pubblicata;
- la vista teacher coincida con il detector analogico pubblicato;
- la shadow FSM parta dallo stesso reset, riceva causalmente solo dati `<=t` e
  riproduca esattamente le otto feature H0 pre-split;
- il detector analogico sia force-free e con profilo/hash congelato;
- entrambe le viste siano `float32[35]`, finite e nello stesso ordine;
- l'azione e il rumore siano `float32[2]`, finiti e non clippati.

L'evidenza storica conferma che H0 pre-split usava il detector analogico non
solo nei due campi load/contact, ma anche come evidenza continua della FSM. Una
teacher view che cambiasse soltanto 10/11 sarebbe quindi semanticamente
incompleta.

Il teacher analogico è vietato nel rollout del candidato: in qualificazione può
essere calcolato soltanto come diagnostica controfattuale, dopo che l'azione
student è stata scelta in autonomia.

## Dati development e split preregistrato

Sono usati esclusivamente i setup development già aperti 02/04/08, plateau 04.
Nessun dato modifica o riseleziona V25.

I setup sono sorgenti `grf_mode=prescribed`; per i rollout full-environment si
creano overlay no-clobber dei profili primary e detector. L'unica differenza
ammessa rispetto ai profili congelati è
`ground.surface_velocity=[0,0,plateau_speed_mps]`, rispettivamente 0,95, 1,05 e
1,25 m/s come già attestato dai receipt development. Geometria, materiale,
contatto, routing e `appliesForce` restano identici; gli originali non vengono
sovrascritti. Entrambi gli overlay di uno stesso trial devono avere la stessa
velocità.

### Raccolta paired

| Ruolo | Trial | Velocità | Tempo assoluto | Offset setup | Seed | Sigma |
|---|---:|---:|---:|---:|---:|---:|
| train | 02 | 0.95 m/s | 119.578 s | 107.880 s | 123 | 0.0025 |
| train | 04 | 1.05 m/s | 122.189 s | 107.550 s | 124 | 0.0025 |
| offline validation | 08 | 1.25 m/s | 120.390 s | 106.878 s | 125 | 0.0025 |

Ogni rollout deve produrre 500 stati paired. Per ogni stato sono creati due
record con la stessa label:

- `student_view -> teacher_mean`;
- `teacher_view -> teacher_mean`, come ancora funzionale H0.

Split per gruppi, senza leakage:

- training: 02 e 04, `2 × 500 × 2 = 2000` record;
- validation: 08, `1 × 500 × 2 = 1000` record.

La validation 08 non entra negli update.

### Qualificazione closed-loop su finestre nuove

Finestre a +12 s dall'inizio del plateau 04, mai usate dal fit:

| Trial | Velocità | Tempo assoluto | Offset setup | Seed |
|---|---:|---:|---:|---:|
| 02 | 0.95 m/s | 129.578 s | 117.880 s | 126 |
| 04 | 1.05 m/s | 132.189 s | 117.550 s | 127 |
| 08 | 1.25 m/s | 130.390 s | 116.878 s | 128 |

Per ciascuna finestra vengono congelati due casi:

- deterministico;
- stocastico con sigma `0.005` e innovazioni standard-normal preregistrate.

Prima si eseguono le sei reference teacher; soltanto se tutte passano si
eseguono i sei rollout autonomi del candidato. Ogni coppia è
condition-matched e usa lo stesso noise tape; le azioni non vengono replayate.

## Gate della raccolta teacher

Ogni rollout paired e ogni reference di qualificazione deve rispettare:

- 500/500 step e `episode_time_limit`;
- almeno due cicli completi;
- penetrazione GRF primaria finita e `<0.025 m`;
- zero safety stop, timeout, fallback, hard-invalid, non-finiti e clipping;
- layout `35/84`, osservazione `float32`, azione `float32[2]`;
- `morphology_weight=0`;
- tutte le metriche SEA/reserve/residual finite;
- nessun campo V25/V20 nell'osservazione;
- zero PPO update e zero accessi protetti.

Un fallimento teacher chiude il ramo come `ERROR_PRIMARY_SPLIT_TEACHER` senza
fit, rescue o nuove finestre.

## Fit supervisionato one-shot

È autorizzato un solo candidato, senza sweep:

```text
optimizer                 Adam
learning_rate             1e-4
epochs_max                300
batch_size                128
patience                  60
anchor_weight             1e-3
clip_weight               1.0
logstd_weight             0.0
seed                       123
training_records          2000
validation_records        1000
trainable_scope           full mean actor
logstd                     frozen bit-exact
critic/non-actor tensors   unchanged
```

Gate offline preregistrati:

- validation `student_view` RMSE `<=0.01` e max abs error `<=0.10`;
- validation `teacher_view` RMSE `<=0.005` e max abs error `<=0.05`;
- RMSE student validation ridotto almeno del 50% rispetto a H0 sorgente;
- zero output non finiti o fuori `[-1,1]` sul corpus;
- logstd parameter/output bit-identici;
- non-actor state bit-identico;
- save/reload actor bit-identico;
- actor digest diverso da H0.

Un FAIL è terminale: niente secondo learning rate, epoca, seed, corpus o
candidato.

## Gate closed-loop del candidato

Ogni rollout autonomo del candidato deve rispettare gli stessi gate fisici
della reference. Inoltre, sullo stesso stato visitato dal candidato viene
calcolata solo per diagnostica la mean H0 sulla teacher view:

- RMSE candidato-vs-teacher controfattuale `<=0.015`;
- max abs candidato-vs-teacher controfattuale `<=0.10`;
- zero dipendenza dell'azione servita dalla teacher view.

Non-regressione condition-matched contro la reference teacher:

- per reserve/residual RMS e abs max:
  `candidate <= reference + max(5%, 5 N m)` per reserve e
  `candidate <= reference + max(5%, 1e-6 N m)` per residual;
- per ciascuna metrica SEA continua RMS e abs max:
  `candidate <= reference + max(5%, 1e-6 nell'unità della metrica)`;
- saturazioni, fallback, timeout, invalidità e safety stop:
  `candidate <= reference`, senza tolleranza.

Tutte le sei condizioni devono passare. Il primo FAIL arresta la
qualificazione senza retry.

## Critic, optimizer e restore

Solo dopo il PASS closed-loop:

1. trapiantare l'actor adattato con `--warm-start-raw` in un trainer target
   nuovo a zero iterazioni;
2. verificare actor learner/EnvRunner/export bit-identico al candidato;
3. verificare critic e optimizer freschi, senza restore da H0;
4. eseguire save/reload zero-update e confrontare actor/non-actor.

Il warm-up critic resta chiuso in questo addendum. Non viene eseguito alcun
update PPO o critic.

## Autorità

Unico nuovo permesso:

```text
h0_primary_split_supervised_adaptation_authorized=true
```

Restano false:

```text
ppo_updates_authorized
general_training_authorized
h0_sep_authorized
v25_ab_c_execution_authorized
protected_trial_access_authorized
reserve_trial_access_authorized
detector_retuning_authorized
primary_grf_modification_authorized
sea_semantic_modification_authorized
corridor_authorized
runtime_promotion_authorized
```

## Output e arresto

Protocol lock, execution unlock, dataset, tape, trace, summary, manifest,
receipt e ledger devono essere no-clobber, strict JSON dove applicabile,
atomici e privi di NaN/Inf. NPZ e checkpoint devono avere SHA e size in un
manifest atomico.

Esiti terminali:

- `ERROR_PRIMARY_SPLIT_TEACHER`;
- `FAIL_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION`;
- `FAIL_H0_PRIMARY_SPLIT_CLOSED_LOOP`;
- `ERROR_H0_PRIMARY_SPLIT_ZERO_ITER_PORT`;
- `H0_PRIMARY_GRF_SPLIT_V1_BASELINE_READY`.

Un PASS non apre automaticamente V25 A/B/C, H0_sep, trial protetti, PPO o
promozione runtime. Serve una nuova autorità esplicita per il passo successivo.
