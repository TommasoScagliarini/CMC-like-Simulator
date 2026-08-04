# Audit dettagliato del checkpoint best: ex-novo, dipendenza prescribed, reward e deployabilità

Data del report: 2026-07-20

## Sintesi esecutiva

Questo report analizza il rollout registrato del checkpoint `rl_module_best`
prodotto dal pilot PPO da 50 nuovi update:

```text
Trajectory Generator/runs/training/validation/warm_start_h1_runs/
2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/rl_module_best
```

Rollout analizzato:

```text
Trajectory Generator/runs/rollout/
2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded/
```

Conclusione principale:

> Il lato protesico genera realmente una traiettoria closed-loop e la reward PPO
> è realmente configurata in modalità ex-novo. Tuttavia il checkpoint osservato
> non è stato appreso da pesi casuali: è una fine-tuning estremamente conservativa
> di H0, actor derivato da teacher prescribed. Il plant di validazione rimane
> inoltre ibrido: la protesi sinistra usa contatto online, mentre il lato biologico
> segue la IK prescribed e la GRF destra rimane prescribed.

La qualifica corretta è quindi:

| Livello | Esito |
| --- | --- |
| reward PPO ex-novo | sì |
| generazione closed-loop delle due reference protesiche | sì |
| training ex-novo da pesi casuali | no |
| actor senza lookup prescribed durante inferenza | sostanzialmente sì |
| simulatore completamente indipendente dai prescribed | no |
| actor eseguibile nello stack simulativo corrente | sì |
| controller hardware validato | no |

Un secondo risultato decisivo riguarda la reward:

- il Morphology Corridor ha impatto effettivo **esattamente nullo**, perché
  `morphology_weight = 0.0`;
- anche tracking prescribed, reference prescribed, bio tracking e imitation
  hanno contributo positivo nullo nella modalità ex-novo corrente;
- tutta la sorgente positiva della reward deriva dal ramo GRF/contatto/FSM;
- il termine dominante è `phase_regular`, che rappresenta circa l'86,5% del
  credito positivo lordo del rollout;
- il ramo diretto magnitudo/contatto GRF ha un impatto netto relativamente
  piccolo, mentre il detector di eventi e la FSM hanno un impatto dominante.

Il return del rollout è stato ricostruito offline sui 500 step:

```text
return nel rollout_summary.json     52.42693952981385
return ricostruito dalla trace      52.42693952981385
differenza                          0.0
```

La ricostruzione è quindi contabilità esatta della reward osservata, non una
stima qualitativa.

## Problema affrontato

L'ispezione dei sette plot standard e della visualizzazione OpenSim ha mostrato
un comportamento nominale molto convincente:

- curve cinematiche protesiche plausibili;
- ripetibilità dei cicli;
- coppie SEA regolari e prive di chattering evidente;
- nessuna saturazione/clipping dell'azione della policy;
- episodio completo di 5 secondi.

Questi risultati aprono però quattro domande scientifiche differenti:

1. La traiettoria è realmente prodotta dalla rete oppure è un replay nascosto
   della IK prescribed?
2. Il pilot può essere chiamato davvero “generazione ex-novo 50 iterazioni”?
3. Quanto la reward osservata dipende dal modello/stimatore GRF e dal
   Morphology Corridor?
4. Se il comportamento nominale è quello desiderato, la rete è già deployable?

Rispondere soltanto dai plot sarebbe insufficiente. Una curva plausibile può
derivare da più cause contemporaneamente:

- actor realmente competente;
- inizializzazione imitativa;
- corpo biologico guidato dalla IK;
- GRF prescribed sul lato sano;
- limiti strutturali del reference governor;
- filtraggio che attenua chattering e dinamiche non realistiche;
- reward proxy che premia eventi del detector senza garantire sim-to-real.

È stato quindi eseguito un audit di provenance, data flow, reward accounting e
contratto di inferenza.

## Strategia di audit

L'analisi è stata suddivisa in cinque livelli.

### 1. Provenance del checkpoint

Sono stati verificati:

- il checkpoint effettivamente usato dal rollout;
- l'iterazione logica del `best`;
- il checkpoint H0 da cui il pilot è ripartito;
- la lineage imitativa e target-domain precedente a H0;
- la deriva cumulativa dell'actor `H0 -> best`.

### 2. Data flow prescribed

È stata distinta la dipendenza prescribed in:

- input diretti dell'actor;
- input privilegiati del critic;
- inizializzazione/reset;
- dinamica del lato biologico;
- GRF applicate alla dinamica;
- calibrazione offline del profilo GRF;
- sola diagnostica e plotting.

### 3. Ricostruzione della reward

Per ogni elemento di `rollout_policy_trace.json` sono stati riutilizzati:

- `reward_terms` registrati dall'ambiente;
- `policy_segment_values` usati dall'eventuale penalità OOB;
- `RewardConfig` esatta da `training_cfg.resolved.yaml`;
- la funzione canonica `compute_reward(...)`.

La somma dei 500 reward ricostruiti è stata confrontata con
`episode_return` del summary.

### 4. Controfattuali offline

Sulla traiettoria invariata sono stati azzerati gruppi di pesi per quantificare:

- ramo GRF diretto;
- ramo detector/FSM;
- intero gruppo GRF/contatto/FSM;
- Morphology Corridor con pesi ipotetici.

Questi controfattuali misurano l'impatto sul return della traiettoria osservata.
Non equivalgono a un nuovo training e non misurano direttamente l'influenza sui
gradienti PPO.

### 5. Audit di deployabilità

Sono stati separati tre livelli:

1. esportabilità computazionale dell'actor;
2. causalità e realizzabilità dei suoi input;
3. validazione fisica/hardware end-to-end.

## Artefatti principali esaminati

### Training e checkpoint

```text
Trajectory Generator/runs/training/validation/warm_start_h1_runs/
2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/
```

File principali:

```text
summary.json
training_cfg.resolved.yaml
checkpoint_best_meta.json
policy_drift_from_h0_milestones.json
train_iterations.jsonl
rl_module_best/
rl_module_last/
```

### Rollout

```text
Trajectory Generator/runs/rollout/
2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded/
```

File principali:

```text
rollout_summary.json
rollout_policy_trace.json
rollout_reset_diagnostics.json
watchdog_summary.json
sim_outputs/rollout_episode_*.sto
sim_outputs/rollout_episode_gait_events.csv
sim_outputs/rollout_episode_gait_events_online.csv
```

### Plot standard

```text
plot/07_15_2026_1/
```

Contenuto:

```text
01_time_sea_control_reserve.png
02_time_joint_motor_states.png
03_gaitcycle_torque_angle_power.png
04_gaitcycle_joint_velocity_power.png
05_time_tau_input_tracking_error.png
06_time_joint_ref_sea_error.png
07_mlp_policy_vs_sound_leg_error.png
missing_channels.txt
```

Il file `missing_channels.txt` contiene:

```text
No missing channels.
```

## Risultato effettivo del rollout

Protocollo:

| Parametro | Valore |
| --- | --- |
| checkpoint | `rl_module_best` |
| selezione azione | deterministica |
| seed | 123 |
| modalità azione | `absolute` |
| start offset | `1.956870983805102 s` |
| durata richiesta | 5 s |
| massimo step | 500 |
| output `.sto` | attivi |
| trace policy | attiva |

Metriche:

| Metrica | Valore |
| --- | ---: |
| step completati | 500/500 |
| fine episodio | `episode_time_limit` |
| return | 52.4269395298 |
| reward media | 0.1048538791 |
| azione assoluta massima | 0.828099 |
| step con clipping | 0 |
| frazione clipping | 0 |
| pelvis `ty` minima | 0.952515 m |
| penetrazione GRF massima | 0.0242174 m |
| norma reserve massima | 493.449803 Nm |
| residuo dinamico massimo | 3.8868e-9 Nm |
| heel strike validi | 4 |
| toe off validi | 3 |
| cicli validi | 3 |
| eventi invalidi | 1 |

La terminazione per penetrazione è configurata a 25 mm. Il margine nominale è:

```text
25.000 mm - 24.217 mm = 0.783 mm
```

L'episodio è quindi completo, ma vicino al limite fisico del contact model.

La `reserve_norm_max_nm` è la norma aggregata delle reserve del recruitment/
Static Optimization. Non va confusa con il singolo reserve actuator protesico
mostrato nei plot, che rimane nullo sulle coordinate protesiche.

## Il checkpoint osservato non è il risultato dell'ultimo dei 50 update

Il pilot è ripartito dal full checkpoint H0 all'iterazione logica 1 e ha
eseguito 50 nuovi update:

```text
nuovi update           50
iterazioni logiche     2 ... 51
checkpoint finale      logical 51
```

Il `checkpoint_best`, tuttavia, è:

```json
{
  "logical_iteration": 24,
  "rllib_training_iteration": 24
}
```

Contiene quindi:

```text
logical 2 ... logical 24 = 23 nuovi update dopo H0
```

Il rollout visualizzato non è l'endpoint delle 50 iterazioni. È il massimo
return PPO incontrato durante il run.

Il fatto che `summary.json` riporti tecnicamente `warm_start_applied=false` non
significa che il run sia partito da pesi casuali. Il pilot usa `resume_from` sul
full checkpoint H0, ripristinando actor, critic, optimizer e iterazione logica.
Il flag `warm_start_applied` descrive invece il percorso CLI di transplant
`--warm-start`, non la semantica scientifica del resume da H0.

## Deriva del checkpoint best rispetto a H0

Audit cumulativo H0 -> logical 24:

| Metrica | Valore |
| --- | ---: |
| action-mean RMSE | 0.000367987 |
| action-mean delta massimo | 0.001278656 |
| KL empirica media | 0.00541658 |
| KL empirica massima | 0.03510496 |
| RMS differenza parametri actor | 3.49094e-6 |
| differenza parametro massima | 1.90997e-5 |
| logstd | bit-exact a H0 |

La rete è stata realmente aggiornata: digest e parametri non sono identici a
H0. La dimensione dell'aggiornamento è però piccola. Il comportamento nominale
osservato deve essere attribuito prevalentemente al prior H0, con correzioni PPO
successive, non a una scoperta ex-novo da zero.

## Lineage dell'actor H0

La catena semplificata è:

```text
actor imitativo
  -> target-domain adaptation supervisionata da teacher
  -> porting al contratto actor 35 feature
  -> adaptation phase-aligned / recovery multi-start
  -> critic warm-up con actor congelato
  -> H0
  -> pilot PPO ex-novo
  -> best logical 24
```

Il teacher della target-domain adaptation legge esplicitamente la IK di base e
la codifica nell'azione assoluta. L'adaptation che precede H0 usa 24.712 esempi:

| Sorgente | Campioni di training |
| --- | ---: |
| ancore nominali del source actor | 16.000 |
| recovery phase-aligned | 712 |
| teacher start -0,20 s / +0,20 s | 8.000 |
| totale | 24.712 |

H0 contiene quindi una forte informazione statistica prescribed nei pesi.
Durante il pilot la reward imitativa non è attiva, ma il prior appreso non viene
azzerato.

## In che senso la generazione è realmente closed-loop

L'azione della policy non è un offset aggiunto alla IK protesica. Con
`action_mode=absolute`, le due uscite normalizzate vengono mappate in target
angolari assoluti:

```text
azione knee  [-1, 1] -> riferimento circa [-1.5, 0.0] rad
azione ankle [-1, 1] -> riferimento circa [-0.7, 0.7] rad
```

La catena runtime è:

```text
actor MLP
  -> target assoluto knee/ankle
  -> target slew limiter
  -> reference model Butterworth3 jerk-limited
  -> limiti velocità/accelerazione/jerk
  -> controllore protesico esterno
  -> comando normalizzato SEA
  -> dinamica motore/molla del plugin SEA
  -> coppia articolare
  -> forward dynamics OpenSim
  -> nuovo stato osservato dall'actor
```

Il `ProstheticSegmentKinematics` sostituisce soltanto i riferimenti di:

```text
pros_knee_angle
pros_ankle_angle
```

Tutte le altre coordinate restano delegate al `KinematicsInterpolator` di base.
Di conseguenza:

- la protesi non sta riproducendo direttamente la propria IK prescribed;
- il resto del corpo continua a essere cinematicamente guidato;
- la generazione è closed-loop per il sottosistema protesico, non per l'intero
  sistema uomo-protesi.

## Dipendenza prescribed: classificazione completa

| Ambito | Uso prescribed | Impatto |
| --- | --- | --- |
| pesi iniziali dell'actor | sì, tramite teacher/imitazione | alto |
| input actor durante inferenza | nessuna IK protesica o GRF destra diretta | basso |
| input critic durante PPO | sì, stato biologico e reference privilegiate | training-only |
| reset episodio | sì, q/qdot iniziali | diretto |
| DOF biologici | seguono IK prescribed | alto sul plant |
| GRF protesica sinistra | online applicata | non prescribed runtime |
| GRF destra/sound side | ExternalForce prescribed | diretto sul plant |
| profilo GRF online | calibrato offline su dati/stati prescribed | indiretto |
| sound-leg imitation loss | calcolata | diagnostica, peso effettivo 0 |
| Morphology Corridor | calcolato da profilo AB06 | diagnostica, peso 0 |
| plot healthy/sound overlay | prescribed/healthy | sola visualizzazione |

### Reset

Nel reset nominale, `served`, `actual` e `prescribed` coincidono:

```text
pros_knee_angle actual_q     -0.174195694 rad
pros_knee_angle prescribed_q -0.174195694 rad

pros_ankle_angle actual_q      0.051824071 rad
pros_ankle_angle prescribed_q  0.051824071 rad
```

Dopo il primo comando, la reference è guidata dalla policy e dalla memoria del
reference governor. Resta da validare un handover hardware che inizializzi
questi stati da sensori reali senza oracle prescribed.

### Lato biologico

Tutti i DOF non protesici continuano a ricevere q/qdot/qddot dalla IK. Il CMC-like
usa quindi:

```text
IK biologica
  -> outer loop
  -> inverse dynamics
  -> static optimization
  -> muscoli + reserve
```

Questo produce un contesto corporeo coerente e ripetibile, ma riduce la libertà
del plant rispetto a un soggetto reale human-in-the-loop.

### GRF ibrida

La configurazione usa:

```yaml
grf_mode: online_sensor
online_grf_applied_side:
  - left
disable_prescribed_grf_side: []
```

Il loader unisce automaticamente i lati online-applied ai lati prescribed da
disabilitare. Nel run effettivo:

```text
left_ground_force1  -> skipped
left online contact -> applied
right_ground_force2 -> prescribed ExternalForce applied
```

Non c'è doppio carico sulla protesi sinistra. La dinamica complessiva resta
però ibrida perché la destra è ancora prescritta.

### Profilo GRF online

Il plugin online è state-based:

- usa geometria, penetrazione, velocità e parametri di contatto;
- non usa tempo o lookup della GRF prescribed durante il rollout;
- il residual runtime è scalato dallo stato/forza istantanea.

Il profilo dichiara però:

```text
source: calibrated_state_only_full_wrench_residual
status: calibrated
note: Short calibration smoke; rerun full calibration before production.
online_mode_status: requires_forward_validation
```

Il detector HS/TO dichiara inoltre:

```text
source: calibrated_against_prescribed_grf
status: preliminary
```

Non esiste leakage temporale runtime, ma esiste una dipendenza statistica e di
calibrazione dal dataset AB06 prescribed.

## Contratto actor/critic

Il rollout dichiara:

```text
n_actor       35
n_observation 84
critic_privileged_observation true
```

Il modulo asimmetrico applica:

```text
actor -> obs[:35]
critic -> obs[:84]
```

Il value network non è necessario all'inferenza e può essere escluso
dall'export actor-only.

### Feature actor

Le 35 feature si dividono in:

| Gruppo | Quantità | Esempi |
| --- | ---: | --- |
| clock legacy disabilitato | 2 | `gait_phase_sin/cos` |
| encoder articolari | 4 | q/qdot knee e ankle |
| stati motore SEA | 4 | motor angle/speed |
| carico/eventi/fase sinistra | 7 | Fy/BW, contact, HS, TO, phase sin/cos, durata |
| stato FSM | 8 | stato, attese, elapsed, cycle credit |
| memoria controller | 10 | endpoint, served ref/qdot/qddot, SEA u |

Nel checkpoint corrente `gait_clock_enable=false`. Le due feature legacy
`gait_phase_sin/cos` sono costanti 0/1 lungo tutti i 500 step: non portano nel
rollout una fase prescribed della gamba controlaterale.

Le feature `served_ref` non sono una IK oracle durante l'episodio. Sono lo stato
interno del governor/reference model alimentato dalle azioni precedenti.

### Feature critic-only

Il suffisso privilegiato contiene, tra gli altri:

- stato pelvis;
- articolazioni biologiche/controlaterali;
- reference cinematiche biologiche;
- target e tracking error protesici;
- carico ed eventi del lato destro.

Questi segnali possono influenzare il critic e quindi la stima degli advantage
durante PPO. Non sono però richiesti dall'actor esportato a runtime.

## La reward è realmente ex-novo?

Sì per l'obiettivo PPO corrente, no per l'origine dei pesi.

Configurazione decisiva:

```yaml
reward_mode: ex_novo

blend_tracking: 0.0
blend_reference: 0.0
blend_bio: 0.0

morphology_weight: 0.0
```

Il ramo ex-novo usa come sorgenti positive:

```text
contact_load_score
contact_support_to_score
phase_regular_score
phase_event_progress_score
landing_window_contact_score
```

Il codice calcola ancora:

- tracking score;
- reference score;
- bio score;
- sound imitation score;
- served imitation score;
- morphology loss.

Queste diagnostiche non entrano nel return quando il loro blend/peso effettivo
è zero.

Il parametro configurato `blend_imitation=0.8` può essere fuorviante se letto
isolatamente: viene usato soltanto nel branch `reward_mode == "imitation"`.
Nel branch ex-novo il contributo imitativo effettivo è zero.

## Formula della reward corrente

La base ex-novo per step è, schematicamente:

```text
raw_base =
    0.10 * contact_load_score
  + 0.30 * contact_support_to_score
  + 0.25 * phase_regular_score
  + 1.00 * phase_event_progress_score
  + 0.25 * landing_window_contact_score
  - penalty_preclip
```

Poi:

```text
base = clip(raw_base, 0, 1)
```

e infine:

```text
reward = base
  - safety
  - penetration GRF
  - ankle moment flip
  - phase timeout
  - phase clawback
  - contact-support clawback
  - swing unloading / contact overload / slip
  - reserve/pelvis se abilitati
  - joint range
  - morphology corridor
  - commanded-reference OOB
```

Le penalità post-clip rimangono attive anche quando la base positiva è già
stata portata a zero.

## Ricostruzione esatta della reward del rollout

### Credito positivo lordo

| Termine | Peso | Somma su 500 step | Step non nulli | Quota credito lordo |
| --- | ---: | ---: | ---: | ---: |
| `phase_regular` | 0.25 | 63.642690 | 341 | 86.54% |
| `contact_load` | 0.10 | 6.550903 | 86 | 8.91% |
| `phase_event_progress` | 1.00 | 3.100000 | 7 | 4.22% |
| `contact_support_to` | 0.30 | 0.247497 | 3 | 0.34% |
| `landing_window_contact` | 0.25 | 0.000000 | 0 | 0% |
| tracking prescribed | 0.00 | 0.000000 | 0 | 0% |
| reference prescribed | 0.00 | 0.000000 | 0 | 0% |
| bio | 0.00 | 0.000000 | 0 | 0% |
| **Totale** | | **73.541089** | | 100% |

La reward positiva è quindi completamente costruita da contatto/eventi/FSM.

Il credito discreto di avanzamento fase, pari a `3.10`, è inoltre
ricostruibile direttamente dai conteggi del rollout:

```text
4 heel strike x 0.10 + 3 toe-off x 0.20 + 3 cicli x 0.70 = 3.10
```

Non è quindi un residuo opaco del logger: coincide esattamente con gli eventi
riconosciuti dalla FSM.

### Penalità pre-clip

| Termine | Somma pesata |
| --- | ---: |
| command-rate | 20.229607 |
| effort | 1.142662 |
| evento invalido | 0.100000 |
| SEA torque error + speed + accel + power | circa 0.001056 |
| clipping azione | 0 |
| saturazione SEA | 0 |
| altri termini a peso zero/non attivi | 0 |
| **Totale** | **21.473326** |

La penalità che più contribuisce alla regolarità dei comandi è il
`command_rate`, non la penalità specifica di chattering o torque error SEA.

### Effetto del clip della base

Prima del clip:

```text
credito positivo lordo - penalty preclip = 52.067764
```

Il raw base è sotto zero in 146 step e sopra uno in un solo step. Poiché il
floor è zero, le porzioni negative vengono eliminate. La somma della base dopo
clip è:

```text
reward_base = 57.804477
```

I contributi lordi non possono quindi essere semplicemente sommati e divisi
per il return: il clip introduce interazione non lineare tra premi e penalità.

### Penalità post-clip

| Termine | Somma | Step non nulli |
| --- | ---: | ---: |
| GRF penetration | 2.328497 | 190 |
| contact-support clawback | 2.472297 | 2 |
| swing unloading | 0.424905 | 27 |
| phase clawback | 0.150000 | 1 |
| ankle moment flip | 0.001839 | 5 |
| safety | 0 | 0 |
| phase timeout | 0 | 0 |
| joint range | 0 | 0 |
| morphology | 0 | 0 |
| OOB reference | 0 | 0 |
| **Totale** | **5.377538** | |

Bilancio finale:

```text
base dopo clip        57.804477
penalità post-clip    -5.377538
return                52.426940
```

## Impatto dello “stimatore GRF”

Nel codice non esiste un unico termine denominato `GRF estimator reward`.
Esistono almeno due rami concettualmente diversi.

### Ramo A: magnitudo e qualità del contatto

Include:

- credito di carico verticale;
- timing TO/supporto;
- landing contact;
- penetration penalty;
- ankle moment flip;
- contact-support clawback;
- swing unloading;
- contact overload;
- slip, attualmente a peso zero.

Durante lo stance, il candidato principale del credito `contact_load` è
costruito come:

```text
clip(Fnormal_BW / 0.20, 0, 1) * penetration_quality
```

La `penetration_quality` vale 1 fino a 10 mm, decresce linearmente e diventa
zero da 12 mm. Inoltre il credito denso viene accumulato in un ledger: se il
supporto non viene poi confermato, il `contact-support clawback` può ritirare
credito già assegnato. Per questo il contributo netto del ramo non coincide
con la sola somma dei campioni di forza verticale.

Contabilità lorda:

```text
positivi diretti GRF/contact     +6.798399
negativi diretti GRF/contact     -5.227538
netto lordo                      +1.570861
```

Per effetto del clip, l'ablation esatta dei relativi pesi sulla stessa
traiettoria dà:

```text
return originale                 52.426940
return senza ramo diretto GRF    50.984088
impatto controfattuale           +1.442852
```

La sola magnitudo GRF non è quindi la principale sorgente del return.

### Ramo B: detector eventi e FSM

Include:

- regolarità della fase;
- progressione HS/TO/ciclo;
- timeout;
- clawback di cicli falliti;
- validità degli eventi;
- requisiti di contatto/carico/escursione del ciclo.

Ablation sulla stessa traiettoria:

```text
return originale                 52.426940
return senza detector/FSM        -1.557530
impatto controfattuale           +53.984470
```

Il ramo detector/FSM domina quindi la reward osservata.

### Ablation completa GRF/contact/FSM

Azzerando tutte le sorgenti positive e le relative componenti GRF/FSM:

```text
return controfattuale = 0.0
```

Questo accade perché tracking/reference/bio/imitation hanno blend effettivo
zero. La policy corrente è addestrata quasi interamente a produrre contatto e
sequenze di fase regolari secondo il proxy online.

### Limite interpretativo

Queste ablation rispondono alla domanda:

> Quanto vale ciascun gruppo nella reward ricevuta dalla traiettoria attuale?

Non rispondono completamente alla domanda:

> Quanto quel gruppo ha cambiato i pesi dell'actor durante 23 update?

Per la seconda domanda servono training A/B con seed, start, H0 e budget
identici, modificando un solo gruppo di reward alla volta.

## Impatto del Morphology Corridor

Configurazione effettiva:

```yaml
morphology_profile: morphology_profiles/ab06_prosthetic_mean_std_corridor.json
morphology_weight: 0.0
morphology_std_multiplier_knee: 1.6
morphology_std_multiplier_ankle: 0.6
morphology_margin_knee_deg: 7.5
morphology_margin_ankle_deg: 7.5
```

Conclusione:

```text
impatto corrente sul reward = 0.0 esatto
```

La buona cinematica non è stata imposta dal corridor durante il pilot.

### Diagnostica corridor

Il corridor è comunque stato calcolato:

| Diagnostica | Valore |
| --- | ---: |
| step con corridor disponibile | 495/500 |
| step con almeno una loss morphology non nulla | 239/500 |
| morphology loss media all-step | 0.1569998 |
| morphology loss massima | 1.4848892 |
| step knee fuori corridor | 173 |
| knee excursion media all-step | 0.096858 rad |
| knee excursion massima | 0.567125 rad |
| step ankle fuori corridor | 178 |
| ankle excursion media all-step | 0.057214 rad |
| ankle excursion massima | 0.393050 rad |

Il fatto che la loss diagnostica sia non nulla non modifica la policy quando il
peso è zero.

È importante anche chiarire quale grandezza viene confrontata con il
corridor: l'implementazione valuta la **served reference** di ginocchio e
caviglia, cioè il riferimento protesico filtrato e realmente consegnato alla
catena di controllo. Non confronta direttamente la coordinata articolare
misurata `q`. Di conseguenza, anche qualora il peso venisse attivato, il suo
effetto diretto sarebbe sul riferimento comandato; l'effetto sulla cinematica
reale passerebbe attraverso controllore, SEA e dinamica del modello.

### Reweighting controfattuale del corridor

Sulla traiettoria invariata:

| Peso morphology ipotetico | Penalità cumulativa | Return controfattuale |
| ---: | ---: | ---: |
| 0.00 | 0.000000 | 52.426940 |
| 0.01 | 0.785000 | 51.641940 |
| 0.05 | 3.924996 | 48.501944 |
| 0.10 | 7.849991 | 44.576948 |

Questo non prevede il risultato di un retraining: con peso attivo l'actor
potrebbe modificare la traiettoria per rientrare nel corridor.

### Altri vincoli cinematici

Il corridor phase-dependent non ha agito, ma rimangono vincoli strutturali:

- action bounds assoluti;
- truncation bounds;
- target slew rate limiter;
- reference velocity limits;
- reference acceleration limits;
- reference jerk limits;
- reference model low-pass.

Nel rollout:

```text
prosthetic_joint_range_term = 0
commanded-reference OOB     = 0
```

La traiettoria è rimasta nei range larghi senza bisogno di penalità.

## Perché le coppie SEA appaiono plausibili e prive di chattering

L'osservazione visuale è reale, ma va attribuita all'intera catena di controllo.

La policy non genera direttamente coppie. Genera target angolari. Prima della
coppia SEA intervengono:

1. target slew limiter;
2. reference model Butterworth3 jerk-limited a 4 Hz;
3. limiti di velocità/accelerazione/jerk;
4. controllore protesico cascade;
5. PI interno SEA;
6. dinamica motore/molla;
7. saturazioni fisiche del plugin.

Nel bilancio reward:

- `command_rate` pesa complessivamente circa 20.230;
- le penalità specifiche SEA pesano complessivamente circa 0.001;
- non ci sono action clipping o SEA saturation nel rollout.

L'assenza di chattering deriva quindi soprattutto da:

- prior imitativo H0 già regolare;
- penalità sul rateo dei comandi;
- reference governor e filtro;
- dinamica/controllore low-level SEA.

Non è corretto attribuirla esclusivamente all'apprendimento PPO delle 23
iterazioni contenute nel best.

## Somiglianza con la IK prescribed

Un confronto diagnostico tra rollout e IK filtrata a 6 Hz mostra:

| Confronto | Ginocchio | Caviglia |
| --- | ---: | ---: |
| raw target policy vs IK, RMSE | 0.109 rad, circa 6.25° | 0.066 rad, circa 3.78° |
| raw target policy vs IK, correlazione | 0.912 | 0.895 |
| stato fisico vs IK, RMSE | 0.161 rad, circa 9.23° | 0.097 rad, circa 5.53° |
| stato fisico vs IK, correlazione | 0.803 | 0.768 |

Interpretazione:

- non è un replay puntuale della IK;
- esistono differenze cinematiche reali e non trascurabili;
- la forte correlazione è coerente con il prior imitativo e con il contesto
  biologico prescribed;
- la correlazione da sola non dimostra leakage runtime.

La provenance dei pesi dimostra invece una dipendenza statistica esplicita.

## Stato di deployabilità

### Livello 1: deployabilità computazionale

Esito: **sì nello stack corrente**.

L'actor:

- è una MLP compatta;
- legge 35 feature;
- produce due azioni;
- non richiede il value network durante inferenza;
- può essere separato dal critic privilegiato;
- è stato caricato ed eseguito per 500 step consecutivi.

Manca tuttavia un pacchetto embedded/HIL completo:

- export TorchScript/ONNX/C++ validato;
- preprocessing e unità congelati;
- benchmark worst-case a 100 Hz;
- watchdog runtime;
- equivalenza numerica host/target.

### Livello 2: causalità degli input actor

Esito: **plausibile ma condizionale**.

Input direttamente realizzabili:

- encoder q/qdot protesici;
- encoder angolo/velocità motore SEA;
- memoria del controller;
- ultimo comando SEA.

Input che richiedono una pipeline sensoriale equivalente:

- forza normale sinistra normalizzata per BW;
- contatto;
- pulse HS/TO;
- fase ipsilaterale;
- durata del ciclo;
- stato della FSM.

Il simulatore ottiene questi segnali da geometria/contact plugin e detector
calibrato. Una protesi reale richiede:

- sensore di carico;
- calibrazione e filtraggio;
- compensazione bias/temperatura;
- detector HS/TO robusto;
- gestione dropout;
- timing e latenza equivalenti;
- reset coerente della FSM.

`deployable_minimal_observation=false` non introduce automaticamente un oracle,
ma indica che il contratto non è quello sensorialmente minimale e aumenta i
requisiti del runtime.

### Livello 3: deployabilità fisica/hardware

Esito: **non raggiunta**.

Motivi principali:

1. Il best è stato scelto dal massimo return, non da un gate robusto completo.
2. Logical 24 non faceva parte delle otto milestone preregistrate dello screen.
3. Tutte le otto milestone schermate sono state respinte.
4. I seed held-out 126-128 non sono stati aperti.
5. Il plant conserva IK biologica e GRF destra prescribed.
6. Le reserve aiutano il lato biologico e non esistono allo stesso modo su
   hardware.
7. Il contact profile è ancora dichiarato non production-ready.
8. Il rollout nominale dura 5 secondi e contiene 3 cicli validi.
9. Il margine rispetto alla termination di penetrazione è soltanto 0.783 mm.
10. Non sono stati testati rumore, delay, bias, dropout o jitter sensoriale.
11. Non esiste ancora una procedura di handover/reset sensor-only.
12. Non è stata eseguita validazione hardware-in-the-loop o human-in-the-loop.

La descrizione corretta è:

```text
simulation-deployable actor candidate
```

Non:

```text
hardware-deployable prosthesis controller
```

## Interpretazione scientifica complessiva

### Cosa dimostra il rollout

Il rollout dimostra che:

- l'actor H0 e la sua fine-tuning producono una traiettoria protesica plausibile;
- i riferimenti generati sono tracciabili dai SEA senza chattering evidente;
- il contatto online sinistro permette 500 step e 3 cicli validi;
- la policy non usa direttamente IK protesica o target imitativo runtime;
- il critic privilegiato non è necessario all'inferenza;
- il reward ex-novo corrente può preservare un prior imitativo funzionale.

### Cosa non dimostra

Non dimostra che:

- la stessa qualità emergerebbe da pesi casuali;
- le 50 iterazioni abbiano creato il comportamento osservato;
- il checkpoint finale logical 51 abbia la stessa qualità;
- il controller sia robusto fuori dal nominale;
- il risultato sopravviva senza plant prescribed;
- il detector GRF simulativo equivalga a sensoristica reale;
- le reserve non stiano mascherando fragilità del lato biologico;
- la policy sia sicura su hardware.

### Risposta sintetica alla domanda “è davvero ex-novo?”

La risposta cambia a seconda della definizione:

| Definizione di ex-novo | Risposta |
| --- | --- |
| nessuna imitation nella reward PPO corrente | sì |
| nessuna IK protesica come input actor runtime | sì |
| azioni assolute e non delta dalla IK | sì |
| pesi iniziali casuali | no |
| nessun teacher prescribed nella lineage | no |
| plant senza IK/GRF prescribed | no |

La formulazione più precisa è:

> Generazione protesica closed-loop con reward ex-novo, inizializzata da un
> actor imitativo prescribed e valutata in un plant CMC-like ibrido.

## Prossimi passi proposti

### A. Isolare il beneficio reale del pilot PPO

Eseguire H0 e logical 24 con protocollo identico e produrre overlay di:

- azioni;
- served reference;
- cinematica fisica;
- coppie SEA;
- GRF/eventi;
- reward decomposition.

La deriva piccola suggerisce che questa comparazione sia più informativa di un
nuovo training lungo immediato.

### B. Testare la dipendenza dal plant prescribed

Costruire una scala di ablation controllata:

1. configurazione ibrida corrente;
2. reset protesico perturbato e ricostruito da sensori;
3. rumore/ritardo sul carico e sugli encoder;
4. detector HS/TO perturbato;
5. GRF online su entrambi i lati, quando il profilo sarà validato;
6. riduzione progressiva della dipendenza dalla IK biologica.

### C. Attribuzione causale della reward

Eseguire training A/B da H0 con stesso seed e sampling:

- baseline corrente;
- detector/FSM ridotto;
- direct GRF ridotto;
- Morphology Corridor attivo a peso basso;
- command-rate ridotto;
- combinazioni soltanto dopo ablation singole.

I controfattuali offline già disponibili possono essere usati per scegliere
pesi iniziali senza spendere subito un intero pilot lungo.

### D. Preparare il contratto hardware

Congelare esplicitamente:

- ordine e scaling delle 35 feature;
- frequenza di inferenza;
- unità;
- normalizzazione BW;
- filtri sensoriali;
- FSM e reset;
- reference governor;
- limiti e fallback;
- export actor-only;
- watchdog e safe state.

### E. Promozione

Soltanto un candidato development eleggibile dovrebbe affrontare:

- matrice multi-start;
- più seed stochastic;
- rumore/latency;
- orizzonte più lungo;
- held-out sigillato;
- HIL;
- successiva validazione sperimentale.

## File modificati e creati

### File creato da questa attività

```text
reports/user/2026-07-20_audit_exnovo_prescribed_reward_deployability_checkpoint_best.md
```

### Codice sorgente modificato

Nessuno.

L'attività è stata un audit read-only degli artefatti esistenti. Non sono stati
modificati:

- reward;
- ambiente;
- actor;
- critic;
- checkpoint;
- simulator runner;
- plugin SEA;
- plugin GRF;
- profili;
- rollout;
- plot.

## File sorgente e configurazioni consultati

```text
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/asymmetric_rl_module.py
Trajectory Generator/baseline_MLP/target_domain_imitation.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/prosthetic_phase_fsm.py
simulation_runner.py
outer_loop.py
model_loader.py
tools/online_grf_contact/README.md
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
```

## Test e verifiche eseguite

### Verifiche di integrità rollout

- `rollout_summary.json`: `ok=true`;
- 500 step registrati;
- 500 elementi nella policy trace;
- end reason `episode_time_limit`;
- output `.sto` presenti;
- gait events presenti;
- policy trace presente;
- action clipping nullo;
- reward e metriche finite.

### Verifica plot

- sette PNG standard presenti;
- dimensioni immagini coerenti;
- plotter MLP ha selezionato il rollout corretto;
- `No missing channels.`;
- ispezione visuale di segnali SEA/control/reserve e confronto policy/sound leg.

### Verifica checkpoint

- `checkpoint_best_meta.json` -> logical 24;
- `rl_module_best` caricato con successo nel rollout;
- action schema a due uscite;
- actor prefix 35 feature;
- observation completa 84 feature;
- critic privilegiato separato;
- logstd bit-exact rispetto a H0 nell'audit del drift.

### Verifica reward

- caricata la `RewardConfig` effettiva dal training snapshot;
- ricostruiti 500 reward con `compute_reward(...)`;
- somma identica a `episode_return`;
- verificati clip low/high;
- aggregati contributi positivi e penalità;
- eseguite ablation offline GRF/FSM;
- eseguito reweighting offline del Morphology Corridor;
- verificato contributo nullo di imitation/tracking/reference/bio/corridor.

### Verifica prescribed

- actor feature manifest confrontato con observation completa;
- verificata assenza di IK protesica e GRF destra nell'actor;
- verificata presenza di contesto prescribed nel critic-only;
- verificato reset protesico allineato alla posa prescribed;
- verificato override assoluto della reference dopo il reset;
- verificato auto-disable GRF prescribed sinistra;
- verificata permanenza della GRF prescribed destra;
- verificati metadati di calibrazione dei profili online.

### Test automatici di codice

Non è stata modificata logica eseguibile, quindi non è stata necessaria una
nuova suite regression del repository. Le verifiche svolte sono audit numerici
e di provenance sugli artefatti prodotti dal run già completato.

## Limiti dell'analisi

1. La decomposizione reward è esatta per il rollout nominale osservato, ma non
   sostituisce attribution dei gradienti durante training.
2. Le ablation offline mantengono fissa la traiettoria; una policy retrained
   reagirebbe ai nuovi pesi.
3. La similarità con la IK è misurata sullo stesso soggetto/trial AB06 usato
   nella lineage e nel plant.
4. Un episodio da 5 secondi non stima affidabilità di lunga durata.
5. Il best è selezionato per return PPO, non per robustezza.
6. Non sono inclusi HIL, sensor noise, delay o domain randomization.
7. Il modello online GRF è ancora dichiarato non production-ready.

## Conclusione finale

Il risultato visuale è scientificamente interessante e non va liquidato come
semplice replay: le due reference protesiche sono realmente generate dalla rete,
attraversano la catena di controllo e producono coppie SEA e stati articolari in
forward dynamics contro un contatto sinistro online.

Allo stesso tempo, il risultato non rappresenta una scoperta da zero:

- il checkpoint best contiene soltanto 23 update PPO dopo H0;
- l'actor è rimasto molto vicino a H0;
- H0 deriva da teacher e dati prescribed;
- il lato biologico e la GRF destra guidano ancora il plant;
- il detector/FSM GRF domina la reward;
- il Morphology Corridor non ha avuto alcun impatto;
- il nominale non sostituisce il gate robusto e la validazione hardware.

La lettura più corretta è quindi positiva ma circoscritta:

> Il warm start ha prodotto un actor protesico closed-loop molto promettente e
> la fine-tuning con reward ex-novo ne ha preservato la qualità. Il checkpoint è
> un candidato deployable in simulazione e un buon punto di partenza per ablation
> prescribed, sensor robustness e HIL; non è ancora un controller hardware
> validato.
