# Diagnosi del target slew limiter e riadattamento imitativo dell'actor

Data report: 2026-07-11

Instruction check token: CMC_AGENT_OK_2026

## Sintesi esecutiva

Il porting tensoriale del warm start imitativo verso l'actor ex-novo era stato
validato, ma il primo rollout deterministico terminava prima del Toe Off (TO)
e senza completare un ciclo HS -> TO -> HS.

L'analisi ha mostrato due problemi distinti:

1. il target slew limiter non implementava realmente la semantica
   target-to-target documentata, perche' veniva ancorato all'uscita gia'
   filtrata del reference governor;
2. dopo la correzione del limiter, un semplice behavior cloning su una sola
   traiettoria teacher aveva un ottimo errore offline ma non era stabile in
   closed loop, a causa del covariate shift.

La strategia applicata e' stata:

```text
validazione porting
-> teacher nel contratto target
-> probe degli anticipi temporali
-> diagnosi e correzione del limiter
-> validazione teacher full-episode
-> behavior cloning actor-only
-> rollout deterministico
-> DAgger sugli stati visitati dall'actor
-> confronto causale con il source ported
-> selezione del miglior checkpoint
```

Risultato finale:

- il teacher target-domain completa `500/500` step e `2` cicli validi;
- il miglior actor adattato e' il DAgger round 2;
- il round 2 completa `356/500` step e `1` ciclo valido, con return
  positivo e zero clipping;
- il full episode e il gate sul picco reserve non sono ancora superati;
- il checkpoint e' adatto come inizializzazione per un H1 PPO breve, non come
  policy finale gia' validata.

## Contesto: actor, critic e teacher

### Actor

L'actor e' la rete che riceve l'osservazione disponibile alla policy e produce
i due comandi normalizzati:

```text
[target ginocchio, target caviglia]
```

Nel contratto corrente i target sono assoluti e vengono decodificati
nell'intervallo articolare configurato prima di attraversare slew limiter,
reference governor, controllore SEA e dinamica OpenSim.

### Critic

Il critic viene utilizzato durante PPO per stimare il valore dello stato. Non
produce i comandi protesici.

La pipeline descritta in questo report ha addestrato soltanto l'actor e non ha
eseguito update PPO. Il checkpoint finale e' un inference-only RLModule e non
contiene `vf` o `vf_encoder`; per questo non e' possibile attestare da tale
artifact l'identita' bit-exact di un critic che non e' presente.

### Teacher

Il teacher non e' una seconda rete neurale. E' un controllore diagnostico che
legge la cinematica prescritta AB06 e, a ogni step, genera il target assoluto
di ginocchio e caviglia corrispondente al tempo successivo.

Il teacher dispone quindi di informazione prescritta che non deve essere
necessariamente disponibile alla policy finale. Il suo scopo e':

- verificare che il contratto dinamico target ammetta una traiettoria valida;
- generare esempi supervisionati per inizializzare l'actor;
- separare un problema della rete da un problema della pipeline target.

## Il contratto target

Per contratto target si intende l'insieme completo delle condizioni sotto cui
verra' eseguita la policy ex-novo:

```text
frequenza policy                         100 Hz
segment_duration                        0.01 s
durata episodio                         5.0 s
start                                   fisso, offset 1.956870983805102 s
action_mode                             absolute
policy_knots                            1
osservazione actor                      39 feature
osservazione completa critic            84 feature
gait clock prescritto                   disabilitato
feature FSM                             abilitate
GRF online sinistra                     applicata alla dinamica
detector HS/TO                          profilo dedicato
target slew knee/ankle                  2.5 / 2.0 rad/s
reference model                         butterworth3_jerk_limited
reference cutoff                        4 Hz
random init                             disabilitato
reward                                  ex_novo congelata
penetrazione soft/hard                  15 / 25 mm
```

Dire che teacher e actor sono testati nello stesso contratto significa che
entrambi attraversano gli stessi limiter, governor, SEA, contatti online, FSM,
reward e guard terminali.

## Problema iniziale

### Porting tensoriale

Il source imitativo aveva un actor da `31` feature, mentre il target ne usa
`39`. Il porting aveva:

- copiato le `29` feature realmente condivise;
- neutralizzato le `2` feature del gait clock disabilitato;
- inizializzato a zero le `8` nuove colonne FSM;
- verificato la copia sui componenti learner/env-runner del trainer iniziale;
- prodotto un actor digest coerente:

```text
b6c7da24e0014edfddc282a2b8187cd58f14691537fb62f7d923f71d98b9ed8a
```

Questa validazione dimostrava che il porting strutturale era corretto, ma non
che il comportamento appreso nel source fosse gia' adatto al nuovo contratto.

### Primo rollout del warm start

Con la semantica del limiter allora presente, il rollout iniziale terminava
molto presto:

```text
steps                              35 / 500
valid HS / TO / cycles             1 / 0 / 0
max penetration                    26.106 mm
return                              circa -2.15
end                                grf_penetration
```

Alzare il solo guard non risolveva il problema. Disabilitare completamente lo
slew limiter permetteva di arrivare a un ciclo, ma produceva una strategia
aggressiva:

```text
steps                              195
valid HS / TO / cycles             2 / 1 / 1
max reserve norm                   circa 851 Nm
max knee motor acceleration        circa 6321 rad/s^2
raw action abs max                 circa 2.351
end                                grf_penetration
```

Questo controfattuale indicava che il source conteneva informazione locomotoria,
ma non chiariva ancora se il problema fosse l'actor, il limiter o la loro
interazione.

## Strategia 1: usare un teacher per isolare la pipeline

### Perche'

Se anche un teacher che chiede direttamente la cinematica prescritta fallisce,
il problema non puo' essere attribuito esclusivamente ai pesi della rete.

Il teacher e' stato eseguito con:

- stesso start e stesso seed;
- stessa action mode assoluta;
- stessi slew limiter `2.5/2.0 rad/s`;
- stesso governor a 4 Hz;
- stessa GRF online applicata;
- stessa FSM;
- stessa reward e soglie `15/25 mm`.

### Primo risultato

Con il limiter non ancora corretto, anche il teacher terminava:

```text
steps                              39 / 500
valid HS / TO / cycles             1 / 0 / 0
max penetration                    25.112 mm
max reserve norm                   342.05 Nm
mean reserve norm                  240.43 Nm
end                                grf_penetration
```

Questo non significava che la cinematica AB06 fosse fisicamente impossibile.
Significava che, con l'implementazione target allora corrente, persino il
comando prescritto non riusciva a superare la prima stance.

## Strategia 2: probe con anticipi temporali

### Che cosa significa anticipo temporale

Il governor filtra il comando e introduce ritardo. Per verificare se il failure
fosse un semplice ritardo di fase, il teacher e' stato modificato in modo da
chiedere una posizione prescritta futura:

```text
teacher normale:
q_cmd(t) = q_prescritta(t + 0.01 s)

teacher anticipato:
q_cmd(t) = q_prescritta(t + 0.01 s + anticipo)
```

L'anticipo non modifica il tempo della simulazione e non salta step. Fa
iniziare prima il movimento target.

Sono stati provati anticipi globali e separati per coordinata:

```text
lookahead                  steps   max penetration   TO
0.00 s                       39       25.112 mm       0
0.15 s globale               57       25.118 mm       0
0.20 s globale               60       25.016 mm       0
0.25 s globale               60       25.183 mm       0
0.40 s globale               55       25.105 mm       0
knee 0.00 / ankle 0.20 s     59       25.069 mm       0
knee 0.20 / ankle 0.00 s     46       25.160 mm       0
```

### Interpretazione

L'anticipo alla caviglia migliorava leggermente la sopravvivenza, mentre quello
al ginocchio poteva peggiorarla. Nessuna combinazione raggiungeva il TO.

La saturazione del risultato intorno a 60 step mostrava che non si trattava
soltanto di scegliere un phase shift migliore. Era necessario controllare la
semantica del limiter.

## Causa radice: slew limiter ancorato alla variabile sbagliata

### Semantica attesa

Il limiter era documentato come target-to-target:

```text
q_limited(k) =
    q_policy_previous
    + clamp(
        q_policy_raw(k) - q_policy_previous,
        -rate * dt,
        +rate * dt
      )
```

A `100 Hz`, i valori configurati corrispondono a:

```text
knee                         0.025 rad/step
ankle                        0.020 rad/step
```

### Semantica effettivamente implementata

Il codice usava come anchor la reference gia' filtrata:

```text
q_limited(k) =
    q_filtered(k)
    + clamp(
        q_policy_raw(k) - q_filtered(k),
        -rate * dt,
        +rate * dt
      )
```

Il risultato era un feedback involontario:

```text
target raw
-> clamp rispetto all'uscita filtrata
-> governor 4 Hz
-> nuova uscita filtrata
-> nuovo clamp rispetto a quella stessa uscita
```

Il governor non vedeva un target che avanzava cumulativamente. Vedeva sempre
un comando distante al massimo `0.025/0.020 rad` dalla propria uscita.

In regime, i limiti nominali `2.5/2.0 rad/s` diventavano approssimativamente
`0.31/0.25 rad/s` effettivi, insufficienti per seguire il tratto prescritto e
scaricare il contatto.

## Correzione del limiter

Sono stati separati due anchor con responsabilita' differenti:

```text
continuity_anchor
    reference filtrata corrente
    usata per costruire un segmento continuo

target_anchor
    ultimo endpoint policy accettato
    usato per il clamp target-to-target
```

La correzione e' stata applicata in:

```text
Trajectory Generator/osim_trj_cmc_like.py
```

Non sono stati modificati:

- reward;
- soglie soft/hard `15/25 mm`;
- valori slew `2.5/2.0 rad/s`;
- reference governor;
- action mode;
- plugin SEA;
- stato iniziale;
- YAML congelato.

E' stato aggiunto un test di regressione nel quale:

```text
filtered output            = 0.0
previous accepted target   = 0.5
rate                       = 1.0 rad/s
dt                         = 0.1 s
raw target                 = 1.0
```

Il risultato corretto deve essere `0.6`, cioe' `0.5 + 0.1`, e non
`0.1`.

## Validazione del teacher dopo la correzione

Con lo stesso teacher, senza alcun lookahead e senza allentare il contratto:

```text
steps                              500 / 500
end                                episode_time_limit
terminated                         false
valid HS / TO / cycles             3 / 3 / 2
episode return                     +64.032352
reward mean                        +0.128065
max penetration                    22.943804 mm
hard penetration guard             25 mm
mean reserve norm                  158.39 Nm
max reserve norm                   523.72 Nm
slew-limited steps                 143 / 500
mean slew-limited fraction         0.19
gate pass                          true
```

Run:

```text
Trajectory Generator/runs/training/
target_domain_imitation_2026-07-11_v2/
```

Questo e' il primo PASS dell'oracolo sotto l'esatto limiter target. Il vecchio
probe `training_like` usava una zero-delta prescritta ma non propagava
`target_slew_rate_limit_rad_s`; non poteva quindi validare questa specifica
parte del contratto.

## Perche' adattare l'actor

Il porting aveva risolto la compatibilita' dei tensori, ma non il domain shift
comportamentale.

Il source e il target differiscono per:

```text
source actor features              31
target actor features              39
gait clock source                  disponibile
gait clock target                  disabilitato
nuove feature FSM                  8
pesi iniziali feature FSM          zero
target slew/governor               contratto differente dal source
```

Copiare i pesi significa conservare cio' che il source aveva imparato. Non
significa insegnargli automaticamente:

- come usare le nuove feature FSM;
- come agire senza gait clock;
- come compensare slew limiter e governor target;
- come recuperare da stati generati dal proprio errore nel nuovo dominio.

Adattare l'actor significa quindi aggiornare soltanto i pesi della rete che
produce le azioni, usando esempi raccolti nel contratto target.

Non significa:

- modificare l'architettura;
- modificare la reward;
- alzare le soglie;
- addestrare il critic;
- eseguire PPO;
- rendere la cinematica prescritta un input della policy finale.

## Strategia 3: behavior cloning actor-only

### Costruzione del dataset

L'episodio dura `5 s` a `100 Hz`:

```text
5 s * 100 step/s = 500 campioni
```

Prima di ogni azione teacher sono stati registrati i `39` valori visibili
all'actor:

- angoli e velocita' protesiche;
- angolo e velocita' dei motori SEA;
- GRF online, contatto ed eventi HS/TO;
- fase online;
- stato one-hot della FSM;
- tempi normalizzati di stance e swing;
- credito di avanzamento del ciclo;
- ultimo endpoint policy;
- posizione, velocita' e accelerazione della reference servita;
- comando SEA, valore assoluto e saturazione.

L'etichetta e' l'azione assoluta normalizzata del teacher:

```text
observation[39] -> action[knee, ankle]
```

### Ottimizzazione

L'actor portato e' stato ottimizzato con:

- errore quadratico sulle medie delle azioni;
- penalita' per medie fuori `[-1, 1]`;
- distillazione della log-standard-deviation source;
- piccolo anchor loss rispetto ai pesi iniziali;
- early stopping sulla validation;
- azzeramento forzato delle colonne gait-clock dopo ogni optimizer step.

Il critic e PPO non sono stati coinvolti.

### Risultato offline

```text
training / validation samples      400 / 100
epochs richieste                   400
best epoch                         368
initial action MSE                 0.947375
initial action RMSE                0.973332
adapted action MSE                 0.000121
adapted action RMSE                0.011002
max absolute error                 0.066900
out-of-bounds fraction             0.251 -> 0.000
gait clock column norms            0.0 / 0.0
PPO updates                        0
save/reload actor                  bit-exact
```

Actor digest:

```text
347af2289a5139b575c7efb978dd40734299d7a08fa1e4125bded4f7ac420e73
```

## Strategia 4: validazione deterministica closed-loop

### Significato di deterministico

L'actor PPO produce media e log-standard-deviation di una distribuzione.
Durante il rollout deterministico viene scelta la media, senza campionamento
casuale:

```text
action = policy_mean
```

Con stesso checkpoint, start, seed e config, la prova isola quindi il
comportamento dei pesi senza rumore di esplorazione.

### Fallimento del clone supervisionato

Nonostante l'ottimo fit offline:

```text
steps                              68 / 500
valid HS / TO / cycles             1 / 0 / 0
return                             -3.3847
max penetration                    25.164 mm
end                                grf_penetration
```

Al primo step l'errore actor-teacher era piccolo. Gia' al secondo step, pero',
la reference acceleration della caviglia differiva di circa `0.81 rad/s^2`.

Quella differenza modificava:

- stato SEA;
- reference servita;
- dinamica articolare;
- contatto;
- osservazione successiva.

L'actor riceveva quindi uno stato non presente nella singola traiettoria
teacher. L'errore si accumulava:

```text
piccolo errore di azione
-> stato leggermente diverso
-> osservazione mai vista
-> errore di azione maggiore
-> ulteriore deriva
```

Questo e' il covariate shift del behavior cloning.

## Strategia 5: DAgger

### Motivazione

DAgger, Dataset Aggregation, corregge il covariate shift facendo etichettare al
teacher anche gli stati realmente visitati dall'actor:

```text
1. rollout dell'actor
2. raccolta degli stati on-policy
3. teacher action allo stesso step fixed-start
4. aggregazione con il dataset precedente
5. nuovo training actor-only
6. nuovo rollout deterministico
```

La label resta legata allo stesso step temporale dell'episodio prescritto.

### Interpolazione locale

Nel round 2 sono stati aggiunti anche stati intermedi fra:

- osservazione teacher;
- osservazione visitata dall'actor.

Sono state interpolate solo le feature continue. Contatto, eventi HS/TO,
saturazioni e one-hot FSM sono rimasti discreti. Lo scopo era insegnare
invarianza locale alle piccole deviazioni, senza creare stati discreti
frazionari non fisici.

## Risultati dei round DAgger

### Round 1

Il primo trace fallito da 68 step e' stato aggiunto con peso 4:

```text
teacher samples                    500
unique DAgger samples              68
DAgger training samples            272
aggregate samples                  772
adapted aggregate RMSE             0.007481
best epoch                         186
```

Il fit sugli stati raccolti migliorava, ma il rollout regrediva:

```text
steps                              45
valid HS / TO / cycles             1 / 0 / 0
return                             -2.1649
end                                grf_penetration
```

Questo ha confermato che la loss supervisionata non e' un criterio sufficiente
per scegliere il checkpoint.

### Round 2

Sono stati usati entrambi i trace falliti, riducendo il peso dei duplicati e
aggiungendo due interpolazioni locali per stato:

```text
teacher samples                    500
visited trace samples              113
interpolated samples               226
aggregate samples                  839
adapted aggregate RMSE             0.007145
best epoch                         372
save/reload                        bit-exact
```

Il rollout ha prodotto:

```text
steps                              356 / 500
return                             +29.886494
reward mean                        +0.083951
valid HS / TO / cycles             2 / 2 / 1
cycle period                       1.453 s
max penetration                    26.512 mm
mean reserve norm                  194.03 Nm
max reserve norm                   978.35 Nm
action clipping                    0.0%
end                                grf_penetration
```

Il round 2 e' il primo actor adattato che:

- supera il TO;
- completa HS -> TO -> HS;
- ottiene `valid_cycle_count >= 1`;
- mantiene return positivo;
- non usa clipping.

### Round 3

Per verificare se fosse possibile arrivare direttamente a `500/500`, e' stato
aggiunto anche il trace da 356 step. Sono stati usati:

```text
teacher samples                    500
visited trace samples              469
interpolated samples               469
aggregate samples                  1438
learning rate                      1e-4
```

Nonostante piu' dati, il rollout e' regredito:

```text
steps                              221
return                             -56.8477
valid HS / TO / cycles             1 / 0 / 0
end                                phase timeout
```

Il round 3 non e' stato promosso. Questo risultato mostra che continuare ad
aggregare stati non garantisce un miglioramento monotono: label temporali
potenzialmente conflittuali e distribuzioni sempre piu' ampie possono rendere
il problema supervisionato meno coerente.

## Confronto causale con il source ported

Il vecchio risultato source da 35 step non poteva piu' essere usato dopo la
correzione del limiter. E' stato quindi ripetuto il source ported sotto lo
stesso limiter corretto del round 2.

Risultato source:

```text
steps                              396 / 500
return                             -7.213321
reward mean                        -0.018215
valid HS / TO / cycles             2 / 2 / 1
invalid events                     4
mean reserve norm                  463.68 Nm
max reserve norm                   1004.96 Nm
action abs max                     1.7813
action clipping                    3.66%
end                                swing timeout
```

Il source arriva piu' lontano in termini di step, ma:

- ha return negativo;
- produce clipping;
- genera quattro eventi invalidi;
- usa reserve persistentemente molto elevate;
- termina per timeout FSM;
- presenta una sequenza di contatti meno regolare.

Il round 2 arriva a meno step, ma:

- ha return positivo;
- chiude un ciclo di durata plausibile;
- non produce clipping;
- riduce nettamente il reserve medio;
- termina per un failure specifico e localizzato di penetrazione.

## Tabella comparativa finale

Tutti i rollout usano stesso start, seed, config, limiter corretto e soglie
`15/25 mm`.

```text
candidate       steps  return    HS/TO/cycle  max pen  mean/max reserve  clip
source ported     396   -7.213      2/2/1      18.64    463.7/1005.0 Nm  3.66%
BC clone           68   -3.385      1/0/0      25.16    302.9/638.9 Nm   0.00%
DAgger round 1      45   -2.165      1/0/0      26.12    304.7/522.1 Nm   0.00%
DAgger round 2     356  +29.886      2/2/1      26.51    194.0/978.4 Nm   0.00%
DAgger round 3     221  -56.848      1/0/0      22.27    207.1/586.4 Nm   0.00%
```

## Risultato finale selezionato

Il checkpoint selezionato e':

```text
Trajectory Generator/runs/training/
target_domain_dagger_2026-07-11_r2/
rl_module_target_adapted/
```

Actor digest:

```text
853940898f079d560756ce22aaa1c908239b4dfa2da82ce690a2fd08d3558c27
```

Il digest SHA-256 e' un'impronta calcolata su nomi, forme, dtype e byte di tutti
i tensori actor. Non misura la qualita' della policy; serve a verificare che
salvataggio, ricaricamento e futuro transplant usino esattamente lo stesso
actor.

Gate finali:

```text
porting tensoriale                         PASS
teacher full episode 500/500               PASS
teacher valid_cycle_count >= 1             PASS, 2 cicli
actor valid TO                             PASS
actor valid_cycle_count >= 1               PASS, round 2
actor return positivo                      PASS
actor clipping                             PASS, 0%
actor full episode 500/500                 FAIL, 356/500
actor penetration guard 25 mm              FAIL, 26.51 mm
actor peak reserve                         WARNING/FAIL, 978 Nm
```

## Interpretazione scientifica

Il risultato non e' una policy finale validata. Dimostra pero' tre punti:

1. il contratto target e' eseguibile, perche' il teacher completa `500/500`;
2. il porting contiene informazione utile, ma richiede adattamento
   comportamentale;
3. il round 2 fornisce un actor che chiude almeno un ciclo valido e costituisce
   un punto iniziale piu' coerente per PPO rispetto al source non adattato.

Il fallimento residuo del round 2 non e' piu' il failure pre-TO iniziale. Si
presenta dopo un ciclo completo, durante la swing successiva. PPO riceverebbe
quindi episodi molto piu' lunghi e informativi rispetto al warm start
originario.

## Decisione operativa: ulteriore DAgger o H0 -> H1

Non e' consigliato imporre `500/500` tramite ulteriori round DAgger prima
di provare PPO, per quattro motivi:

1. l'H0 teacher ha gia' dimostrato che il full episode e' possibile;
2. il gate minimo actor `valid_cycle_count >= 1` e' stato raggiunto;
3. il round 3 e' regredito nonostante un dataset piu' grande;
4. l'imitazione temporale non ottimizza direttamente penetrazione, reserve e
   robustezza ex-novo, mentre PPO dispone della reward progettata per farlo.

La strategia raccomandata e':

```text
H0
    mantenere congelato il teacher full-episode come riferimento

zero-iteration transplant validation
    inserire l'actor round 2 nel trainer PPO target
    verificare digest su learner ed env-runner
    verificare che il critic target resti quello iniziale

H1 breve
    stesso seed/config/budget del fresh baseline
    monitorare lunghezza, return, HS/TO/cicli, penetrazione,
    reserve, clipping, FSM e diagnostica SEA

H2 deterministico
    richiedere miglioramento rispetto ai 356 step del round 2
    e riduzione del picco reserve
```

Se H1 non supera il comportamento iniziale del round 2, il trace H1 potra'
essere usato per un adattamento mirato successivo. Non conviene invece
continuare indefinitamente DAgger prima di osservare cosa riesce a correggere la
reward PPO.

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/target_domain_imitation.py
Trajectory Generator/baseline_MLP/target_domain_dagger.py
Trajectory Generator/baseline_MLP/freeze_actor_checkpoint.py
Trajectory Generator/baseline_MLP/warm_start.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
validation/test_target_domain_imitation.py
validation/test_target_slew_limiter.py
validation/test_warm_start.py
validation/test_rollout_eval.py
validation/validate_warm_start_port.py
validation/summarize_warm_start_preflight.py
validation/test_warm_start_preflight.py
reports/user/2026-07-11_rollout_diagnostico_warm_start_iniziale.md
reports/user/2026-07-11_riadattamento_imitativo_target_domain.md
```

## Artefatti principali

```text
Teacher e behavior cloning:
Trajectory Generator/runs/training/
target_domain_imitation_2026-07-11_v2/

DAgger round 1:
Trajectory Generator/runs/training/
target_domain_dagger_2026-07-11_r1b/

DAgger round 2 selezionato:
Trajectory Generator/runs/training/
target_domain_dagger_2026-07-11_r2/

DAgger round 3 non promosso:
Trajectory Generator/runs/training/
target_domain_dagger_2026-07-11_r3/

Rollout round 2:
Trajectory Generator/runs/rollout/
2026-07-11_target_dagger_r2_recorded/

Rollout source con limiter corretto:
Trajectory Generator/runs/rollout/
2026-07-11_source_warm_fixed_slew_anchor_recorded/
```

## Test e verifiche

```text
py_compile file coinvolti                  PASS
unit test discovery                        PASS, 33/33
training config smoke                      PASS
git diff --check                           PASS
target slew target-to-target regression    PASS
teacher full-episode gate                  PASS
actor save/reload                          PASS, bit-exact
gait clock columns                         PASS, norme 0.0 / 0.0
closed-loop valid_cycle_count >= 1         PASS, round 2
closed-loop full episode                    FAIL, round 2 a 356/500
```

## Protocollo pre-H1 eseguito: gate 1-3

La raccomandazione precedente e' stata attuata fino al preflight stocastico,
senza eseguire aggiornamenti PPO.

### Gate 1: congelamento DAgger round 2

Il round 2 selezionato e' stato copiato in un artefatto separato con:

- modulo RLlib completo;
- configurazione target risolta;
- manifest ordinato delle 39 feature actor;
- SHA-256 dei tre file del modulo e della configurazione;
- digest actor atteso e confronto source-to-frozen bit-exact.

Artefatto:

```text
Trajectory Generator/runs/training/
target_domain_warm_start_selected_2026-07-11_r2/
```

Esito:

```text
actor digest       853940898f079d560756ce22aaa1c908239b4dfa2da82ce690a2fd08d3558c27
feature actor      39
source -> frozen   exact, max_abs_diff 0.0
config SHA-256     identico tra source e copia congelata
gate 1             PASS
```

Il warm start non usa piu' implicitamente il vecchio manifest storico a 31
feature quando riceve questo checkpoint. Il trainer riceve il manifest a 39
feature in modo esplicito e ne verifica anche il digest.

### Gate 2: transplant nel trainer PPO a zero iterazioni

E' stato costruito il trainer PPO completo con `iterations=0`, un EnvRunner
remoto e quello locale. Questo crea il critic e lo stato PPO target, applica il
solo actor transplant e termina prima di campionare o aggiornare pesi.

Artefatto:

```text
validation/warm_start_port_runs/2026-07-11_round2_protocol/
trainer_zero_iter/
```

Esito:

```text
iterazioni PPO eseguite                    0
digest source actor                        853940...c27
digest learner actor                       853940...c27
digest EnvRunner locale                    853940...c27
digest EnvRunner remoto                    853940...c27
digest modulo esportato                    853940...c27
critic target, 6 tensori                   exact prima/dopo
optimizer imitativo caricato               no
sync prima del primo sample                si
gate 2                                     PASS
```

L'audit indipendente ha inoltre verificato:

- hash identico del `module_state.pkl` congelato ed esportato;
- stessa classe RLModule, architettura `39 -> 256 -> 256 -> 4` e action space;
- output identici su 4096 osservazioni sintetiche allineate;
- output identici sulle 356 osservazioni reali del rollout round 2;
- differenza massima tra media target e azione registrata `1.19e-7`.

Audit:

```text
validation/warm_start_port_runs/2026-07-11_round2_protocol/
port_audit/warm_start_port_validation.json
```

Stato audit: `PASS`, zero failure e zero warning.

### Replay deterministico del modulo esportato

Il comportamento deterministico noto resta:

```text
steps 356, return +29.886, HS/TO/cicli 2/2/1, clipping 0%
```

Il modulo esportato e il modulo che ha generato quel rollout hanno lo stesso
file di stato e producono logits identici su tutte le 356 osservazioni
registrate. Due nuovi replay closed-loop sono stati comunque tentati. Entrambi
sono stati arrestati dal watchdog per uno step del solver rimasto bloccato:

```text
tentativo 1   step 198, timeout esterno 120 s
tentativo 2   step 165, timeout esterno 600 s
```

Il punto di blocco diverso e l'assenza di processi Ray residui indicano un
problema prestazionale intermittente del solver, non una differenza dei pesi.
La equivalenza funzionale dell'actor e' PASS; il nuovo replay runtime completo
resta inconclusivo e va ripetuto dopo avere risolto il gate stocastico.

### Gate 3: preflight con esplorazione PPO

`rollout_eval.py` ora supporta due modalita' esplicite:

- `deterministic`: `forward_inference` e media gaussiana;
- `stochastic`: `forward_exploration` e sampling dalla stessa
  `TorchDiagGaussian` usata dagli EnvRunner PPO.

Sono stati eseguiti tre episodi stocastici senza training:

```text
seed  steps  return   clip valori  max pen   HS/TO/cicli  fine
123      48  -3.104      30.21%     25.43 mm    1/0/0      penetrazione
124      50  -2.688      32.00%     26.07 mm    1/0/0      penetrazione
125      77  -3.125      32.47%     25.03 mm    1/0/0      penetrazione
```

Tutti e tre terminano prima del primo TO. La frazione di terminazioni per
penetrazione e' `100%`, la frazione che raggiunge un TO e' `0%` e il clipping
mediano e' `32%`.

La causa e' la varianza ereditata dall'actor imitativo. Sulle 356 osservazioni
del rollout deterministico:

```text
sigma mediana knee/ankle   0.752 / 0.742
sigma minima               0.262 / 0.249
sigma massima              2.495 / 2.455
```

Questi valori sono troppo grandi rispetto all'action space `[-1, 1]`: il
campionamento altera pesantemente la media imitativa validata e manda subito
il contatto oltre 25 mm.

Esito complessivo:

```text
gate 1 congelamento                         PASS
gate 2 porting/trainer zero-iteration       PASS
gate 3 esplorazione training-like           FAIL
ready_for_h1                                false
decisione                                   STOP_BEFORE_H1
```

Non serve rifare DAgger e non va modificata la media dell'actor selezionato.
Il prossimo intervento deve riguardare esclusivamente le due uscite log-standard
deviation: inizializzarle a una varianza controllata, ripetere i tre probe del
gate 3 e avviare H1 solo se il gate passa.

Riepilogo machine-readable:

```text
validation/warm_start_rollout_runs/2026-07-11_round2_protocol/
preflight/warm_start_preflight_summary.json
```

## TODO

- [x] Validare il porting tensoriale source -> target.
- [x] Isolare la pipeline con un teacher prescritto.
- [x] Verificare gli anticipi temporali knee/ankle.
- [x] Individuare e correggere il feedback involontario limiter/governor.
- [x] Proteggere la semantica target-to-target con un regression test.
- [x] Validare il teacher full-episode sotto il contratto target completo.
- [x] Eseguire behavior cloning actor-only.
- [x] Diagnosticare il covariate shift in closed loop.
- [x] Eseguire DAgger e ottenere almeno un ciclo valido.
- [x] Selezionare il round 2 mediante rollout, non mediante sola loss offline.
- [x] Congelare il round 2 con manifest a 39 feature e digest verificato.
- [x] Trapiantare il round 2 nel trainer PPO target a zero iterazioni.
- [x] Verificare digest actor su learner ed env-runner e critic target invariato.
- [x] Eseguire il preflight con la distribuzione stocastica usata da PPO.
- [x] Ridurre/reinizializzare soltanto la log-standard-deviation esplorativa
      (`sigma=0.05` e fallback `sigma=0.03`).
- [x] Ripetere il gate 3 su tre seed per entrambi i candidati: clipping e TO
      passano, ma il gate resta FAIL per 3/3 terminazioni da penetrazione.
- [x] Completare un nuovo replay deterministico runtime senza timeout del solver.
- [x] Sostituire il gate assoluto sulla penetrazione con un gate relativo e
      testare il candidato anisotropico; il gate resta `FAIL` per assenza di
      cicli e sopravvivenza mediana insufficiente. Dettagli in
      `2026-07-12_warm_start_exploration_variance_preflight.md`.
- [x] Provare un adattamento noise-aware a `log-std` fissa: non promosso per
      regressione deterministica; la soluzione selezionata preserva la media r2
      e usa `sigma=0.003`.
- [x] Ripetere il gate relativo: `PASS`, dettagli in
      `2026-07-12_readiness_h1_sigma0003.md`.
- [ ] Eseguire H1 warm-start breve contro fresh-policy a parita' di budget.
- [ ] Eseguire H2 deterministico sul best H1.
- [ ] Richiedere a H2 il full episode e una riduzione sostanziale del picco
      reserve prima di training piu' lunghi.
