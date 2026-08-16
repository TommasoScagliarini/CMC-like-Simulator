# Addendum V25 — protocollo H0 A/B/C

> Data: 2026-08-04  
> Stato: `PROTOCOL_ONLY_EXECUTION_NOT_AUTHORIZED`  
> `execution_authorized=false`  
> `h0_executed=false`  
> Piano storico integrato, non modificato: `2026-07-23_piano_grf_primaria_h0_sep_detector_training_ready.md`  
> SHA-256 del piano storico al momento dell'addendum: `2dd4e0a06e13dca87ce74a1fe2bb601a1822ea4decc78cd60987ec250f47b520`

## 1. Ambito e precedenza

Questo addendum sostituisce esclusivamente le parti del piano del 2026-07-23
relative al detector V17 e al gate H0 A/B/C. In caso di conflitto prevale
questo documento soltanto per:

- tecnologia, geometria e temporizzazione del detector;
- semantica HS/TO prodotta dal detector;
- definizione dei casi A, B e C;
- matrice delle condizioni H0, conteggio delle unità e gate associati;
- campioni e interfacce specifici del detector binario;
- riferimenti al candidato detector nelle fasi successive.

Sono quindi superati, senza cancellarli dalla storia:

- V17, le due sfere, i raggi, le forze normali, le soglie 0,5/0,25 N e il
  dwell di 30 ms;
- il conteggio ambiguo di 12 casi nella sezione H0 del piano storico;
- ogni requisito che consideri invalido lo stato con entrambi i bit a zero.

Restano invariati e autorevoli tutti gli altri contenuti del piano storico,
in particolare il contratto della GRF primaria, l'oracle canonico, la
semantica SEA, la governance dei trial, l'eventuale percorso H0_sep e il
preflight corridor. Questo addendum non autorizza H0_sep, dati protetti,
training, PPO, corridor, promozione runtime o modifiche alla GRF primaria.

## 2. Provenienza congelata

| Oggetto | Path | SHA-256 |
|---|---|---|
| Freeze globale candidato V25 | `validation/binary_phase_detector_v25_development_candidate_freeze_lock.json` | `04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346` |
| Profilo V25 selezionato | `validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json` | `db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2` |
| Detector analogico legacy H0 | `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json` | `61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e` |
| Sampler detector binario | `binary_phase_detector.py` | `57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6` |
| FSM V20 | `Trajectory Generator/binary_phase_fsm.py` | `0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1` |
| GRF primaria, attestazione di non modifica | `online_grf.py` | `52e39bf9a3b20dd65242f3f9076d76ed788239fe7c3e5b825bc37a9657c4fefa` |
| Actor H0 | `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last/module_state.pkl` | `44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b` |
| Costruttore RLModule H0 | `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last/class_and_ctor_args.pkl` | `5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228` |
| Metadata RLModule H0 | `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last/metadata.json` | `3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12` |
| Config sorgente H0 | `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/training_cfg.resolved.yaml` | `6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e` |
| Metadata full checkpoint H0 | `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last_meta.json` | `adc5e26c84f3a0fb2de88eacea945426d38ea4e80c7f2f0cd1979ea663c9dfff` |

Il candidato è `v25_4b351f67b5b86ab0`. La sua selezione development su
02/04/08 è consumata e non può essere ripetuta, ottimizzata o sostituita. I
trial 05/06 e le riserve 03/07 restano chiusi.

Il freeze V25 storico riporta ancora
`fsm_implementation_contract_id=binary_point_v19+functional_contact_fsm_v1_shadow`
perché è stato creato prima della definizione dell'allowlist runtime V25. La
sorgente FSM identificata dallo SHA sopra è geometry-agnostic ed è rimasta
invariata. Il nuovo receipt/lock deve attestare il mapping esplicito di quella
stessa sorgente al contract runtime V25 definito in questo addendum, senza
modificare, reinterpretare o riscrivere il freeze storico.

Gli hash del codice di integrazione runtime e del driver A/B/C non sono
congelati da questo addendum: prima di qualsiasi H0 devono essere implementati,
verificati e fissati in un receipt separato. La presenza di questo documento
non rende eseguibile il protocollo.

## 3. Contratto detector V25 + FSM V20

Il detector finale è costituito da due punti geometrici force-free sul frame
`/bodyset/foot_l`:

- tallone `[-0.059315516055, -0.06316904531668477, 0.01716156589300303]` m;
- punta `[0.135837908089, -0.06595974250405881, 0.005834590883117434]` m.

Il terreno ha origine `[0.0, 0.0148208231, 0.0]` m e normale
`[0.0, 1.0, 0.0]`. Ogni canale restituisce esclusivamente un booleano:
`CONTACT` quando la signed clearance è minore o uguale a zero, altrimenti
`AIR`. Nel ramo V25 non esistono sfera, raggio, materiale, forza, gradualità,
soglia in newton o legge di contatto. V25 non aggiunge Force o altri componenti
fisici e non può modificare la dinamica.

Questa sostituzione scientifica del detector candidato non elimina il detector
analogico legacy dal confronto H0. Quest'ultimo resta un componente separato,
sensor-only, con `appliesForce=false` nel routing e senza applicazione alla
dinamica. Non è la GRF primaria: serve soltanto a conservare la sorgente eventi
legacy già usata da H0 nei casi A e B.

Il contratto temporale è:

- `detector_sample_dt_s=0.001`;
- `policy_step_s=0.01`;
- esattamente dieci campioni unici in `(previous_time_s, time_s]` per step;
- campione `t0` usato soltanto come baseline, senza attribuzione di eventi;
- `binary_phase_debounce_s=0.005`;
- `0 <= delivered_time_s - confirmed_time_s <= 0.010` s.

La FSM V20 interpreta i quattro contact word come stati tutti validi:

- `AIR=(0,0)`;
- `HEEL=(1,0)`;
- `BOTH=(1,1)`;
- `TOE=(0,1)`.

HS è la prima transizione stabile per 5 ms da `AIR` a qualunque contatto; TO è
la prima transizione stabile per 5 ms da qualunque contatto ad `AIR`.
`event_time_s` resta l'onset fisico, `confirmed_time_s` è la conferma dopo il
debounce e `delivered_time_s` è il boundary policy che riceve l'evento. I
passaggi `HEEL/BOTH/TOE` durante il contatto sono diagnostici e non generano
nuovi HS/TO.

Gli identificatori normativi sono:

- raw detector: `binary_point_clearance_v1`;
- campo config/componente runtime shadow:
  `binary_phase_event_contract_id=binary_point_v25+functional_contact_fsm_v1_shadow`;
- bundle scientifico shadow:
  `primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1_shadow`;
- campo config/componente runtime active:
  `binary_phase_event_contract_id=binary_point_v25+functional_contact_fsm_v1`;
- target bundle active:
  `primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1`.

Il prefisso `primary_grf_split_v1+` appartiene al bundle scientifico e non deve
essere passato nel campo runtime `binary_phase_event_contract_id`. Receipt e
lock devono conservare i due valori in campi distinti; una singola stringa
ambigua è un errore fail-closed.

## 4. Topologia comune e casi A/B/C

A, B e C usano lo stesso actor H0, lo stesso modello fisico, la stessa GRF
primaria ibrida, la stessa configurazione SEA, la stessa policy a 10 ms e
`morphology_weight=0`. Il profilo analogico legacy H0 resta caricato, non
applicato alla dinamica e distinto dalla GRF primaria in tutti i casi, così la
sua rimozione non può causare un fallback implicito agli eventi della primary
online GRF.

A e B caricano inoltre lo stesso profilo V25 e lo stesso sampler force-free a
1 ms. Questa scelta rende identiche geometria, sampling e topologia diagnostica:
l'unico fattore A/B è l'esecuzione della FSM V20 shadow. La sorgente legacy
analogica autorevole deve avere configurazione e routing identici in A e B.
Entrambi impostano esplicitamente lo stesso component contract shadow V25,
debounce `0.005`, sample time `0.001` e profili analogico/V25; A non usa il
default `binary_events_disabled_v1`. Sono vietati fallback o risoluzioni
differenti per caso.

- **A — `legacy_control`**: detector analogico legacy autorevole; V25 è caricato
  e campionato ma la FSM V20 è disabilitata.
- **B — `binary_shadow`**: detector analogico legacy ancora autorevole; differisce
  da A soltanto perché la FSM V20 elabora gli stessi bit V25 e pubblica
  diagnostica shadow. Eventi attivi, osservazioni, pulse, azioni, reward e
  dinamica devono restare bit-identici ad A.
- **C — `binary_active`**: V25 e la FSM V20 diventano l'unica autorità per gli
  eventi sinistri accettati. Il detector analogico resta caricato, ma sul lato
  sinistro è esclusivamente diagnostico e non può produrre o sostituire eventi
  attivi. Può continuare a servire gli eventi destri legacy, se previsti dal
  routing congelato. L'evidenza continua di carico/contatto resta quella della
  GRF primaria. Sono vietati fallback sinistri verso detector analogico, primary
  online GRF o GRF prescritta.

In C la FSM V20 resta il detector di eventi, non sostituisce il phase-state
usato dall'actor. I suoi eventi sinistri confermati devono essere adattati,
prima del calcolo dell'osservazione dello stesso policy step, all'istanza
`ProstheticPhaseFSM` già esistente. Quest'ultima resta l'unico aggregatore di
gait state, pulse, otto feature di fase e termini reward; riceve dal lato
sinistro soltanto eventi V20 e conserva la propria semantica di ciclo. Un
adapter equivalente è ammesso solo se dimostra parità esatta di layout, dtype,
stato e reward con `ProstheticPhaseFSM`. È vietato sostituire direttamente
`_phase_fsm_payload` con il payload V20.

Forma, ordine e dtype delle osservazioni 35/84 restano invariati. La modalità
`binary_active`, il relativo routing e la coesistenza fail-closed tra detector
analogico e V25 devono essere implementati e sottoposti a test prima del receipt
che potrà eventualmente autorizzare l'esecuzione.

## 5. Sei condizioni e contabilità

Il tempo nominale è `1.956870983805102` s. Le sei condizioni preregistrate sono:

| ID | Selezione azione | Start offset [s] | Seed |
|---|---|---:|---:|
| `det_minus020` | deterministic | `1.756870983805102` | 123 |
| `det_nominal` | deterministic | `1.956870983805102` | 123 |
| `det_plus020` | deterministic | `2.156870983805102` | 123 |
| `stoch_nominal_seed123` | stochastic | `1.956870983805102` | 123 |
| `stoch_nominal_seed124` | stochastic | `1.956870983805102` | 124 |
| `stoch_nominal_seed125` | stochastic | `1.956870983805102` | 125 |

Per ogni condizione si eseguono A e B come coppia e, soltanto dopo il PASS
della coppia, C condition-matched. A genera e salva un tape no-clobber con il
contratto di selezione, le innovazioni casuali per-step quando presenti e le
azioni mean/raw/applicate ottenute. B deve rigiocare almeno il noise tape esatto:
il semplice reseeding non è una garanzia sufficiente.

Per le tre condizioni stocastiche il contratto congelato è sigma attesa
`0.005` e il tape contiene le innovazioni standard-normal pre-scaling, ordinate
per step e dimensione d'azione. Il driver deve verificare forma, dtype,
finitezza, lunghezza e SHA del tape prima dell'uso. Le condizioni deterministiche
non consumano innovazioni casuali.

Il futuro execution lock deve scegliere e congelare una delle due modalità
equivalenti per B:

1. replay del noise tape, con azioni ricalcolate dall'actor e confrontate bit per
   bit con il tape A;
2. replay anche dell'action tape A, ma soltanto dopo aver calcolato actor mean e
   raw action di B e averli confrontati bit per bit prima dell'applicazione.

Ogni mismatch deve fallire prima dello step e non può essere mascherato
iniettando l'azione A. C riusa lo stesso contratto e, nelle condizioni
stocastiche, la stessa sequenza di innovazioni condition-matched, ma resta un
rollout closed-loop: l'actor calcola le azioni dalle osservazioni C e le azioni A
non vengono imposte.

La contabilità normativa è:

- 6 unità protocollari paired A/B, ciascuna composta da 2 rollout: 12 rollout;
- 6 unità protocollari C, ciascuna composta da 1 rollout: 6 rollout;
- totale: **12 unità protocollari e 18 rollout reali**.

Un'unità paired A/B è indivisibile. Un output mancante, parziale o non
confrontabile fa fallire l'intera unità. Il conteggio di 12 non deve mai essere
riportato come numero di rollout reali.

## 6. Gate fail-closed

### 6.1 Gate comune per ogni rollout

Ogni rollout deve:

- usare esattamente l'actor, la configurazione e i contratti congelati;
- completare `500/500` step e 5,0 s con `end_reason=episode_time_limit`;
- contenere almeno due cicli completi validi;
- avere penetrazione finita e strettamente minore di 0,025 m;
- avere zero clipping, timeout, safety stop, fallback, valori non finiti e
  hard-invalid runtime/contract;
- produrre 500 campioni validi per ogni metrica per-step obbligatoria;
- preservare layout attore/critic 35/84 e semantica SEA;
- scrivere summary, trace, manifest e receipt strict JSON, no-clobber, atomici e
  privi di NaN/Inf.

Un campo obbligatorio assente non equivale a zero: equivale a `FAIL`.

Per `hard-invalid runtime/contract` si intendono errori quali sample o batch
malformati, mancanti, duplicati, non monotoni, off-grid o non finiti; timestamp
non causali; sorgente/contract errati; fallback; timeout; stato esplicito
`INVALID_EVENT`; evento hard sconosciuto o non accettato. Non coincide con il
contatore diagnostico legacy `invalid_event_count`, che nell'H0 storico può
essere finito e valere 1–2 senza indicare un errore runtime. In A e B questo
contatore deve essere presente, finito e bit-identico, ma non è soggetto a un
gate `== 0`.

### 6.2 Parità paired A/B

Per ciascuna delle sei condizioni A e B devono avere esattamente 500 righe
allineate. È richiesta parità bit per bit, senza tolleranza e senza troncare al
numero minimo di righe, per:

- osservazioni attore e critic, azioni raw/applicate e innovazioni casuali;
- riferimenti serviti, stati e dinamica;
- eventi legacy attivi, pulse, gait state e reward;
- diagnostica fisica SEA, reserve e residual;
- reason di terminazione e tutti i summary field comuni.

Le sole differenze ammesse sono i campi diagnostici top-level il cui nome
inizia esattamente con `binary_phase_` e il loro contenuto. La proiezione attiva
usata dal comparatore elimina questi campi top-level da A e B prima del confronto
bit per bit. Nessun altro campo, sottocampo, prefisso o namespace può essere
escluso. Qualunque differenza residua è terminale e impedisce C.

Prima di applicare questa proiezione, il driver deve inoltre confrontare il
journal grezzo V25 comune ad A e B: baseline `t0` e tutti i 5.000 campioni
booleani a 1 ms devono coincidere bit per bit, inclusi timestamp, ordine e SHA
del trace. Sono esclusi dal confronto soltanto stato, transizioni ed eventi
della FSM V20, che in A è disabilitata e in B è shadow. In questo modo il
namespace diagnostico non può nascondere una differenza nel segnale d'ingresso.

### 6.3 Gate eventi C

C deve inoltre rispettare:

- dieci campioni booleani validi per ogni policy step e baseline `t0` senza
  evento;
- conteggi, cicli e consegna una sola volta coerenti; dopo l'eventuale evento di
  bordo iniziale l'ordine deve essere `HS -> TO -> HS`;
- `confirmed_time_s - event_time_s = 0.005` s entro la sola tolleranza numerica
  del reticolo;
- `0 <= delivered_time_s - confirmed_time_s <= 0.010` s;
- zero eventi duplicati, invertiti, sconosciuti o attribuiti al reset;
- zero eventi hard invalidi, sconosciuti o non accettati nella pipeline binaria;
- nessun evento sinistro attivo proveniente da legacy, prescribed GRF o
  detector analogico.

`AIR=(0,0)` è lo swing normale e non è un errore. Anche `HEEL`, `BOTH` e `TOE`
sono contact word validi. Sono invece invalidi e fail-closed: sample mancanti,
duplicati, non monotoni, off-grid, non booleani o non finiti; batch incompleti;
timestamp non causali; stati FSM sconosciuti; sorgente o contract ID errati;
fallback; trasferimenti parziali; eventi fuori ordine o consegnati più volte.
Le cancellazioni di un candidato prima dei 5 ms sono diagnostica di debounce,
non eventi e non invalidità; devono comunque essere finite e contabilizzate.

Se la baseline `t0` è già in contatto, il reset non genera un evento ma
inizializza uno stance parziale. Il primo rilascio stabile può quindi produrre
un TO legittimo con `startup_partial_stance=true` prima del primo HS; questo TO
di bordo non completa un ciclo e non viola l'ordine. Ogni altro TO senza un HS
precedente è invalido.

L'implementazione C deve inizializzare nello stesso stato di stance parziale
anche `ProstheticPhaseFSM` (o l'adapter bit-equivalente), usando la baseline V25
senza sintetizzare un HS. Alimentare il leading TO a una phase FSM ancora in
`WAIT_HS` è un errore di bootstrap e deve fallire il preflight; serve un test
dedicato sia per reset in `AIR` sia per reset già in contatto.

### 6.4 Non regressione SEA/reserve condition-matched

Ogni C è confrontato esclusivamente con A della stessa condizione; è vietato
usare il cap di un'altra partenza o di un altro seed. B deve già essere
bit-identico ad A.

Per reserve e residual, massimo assoluto e RMS di C devono essere minori o
uguali al valore di A più la sola tolleranza numerica
`max(1e-6 N m, 1e-9 * abs(reference))`. Conteggi di fallback, saturazione o
violazione devono invece soddisfare `C <= A` senza tolleranza.

Le metriche SEA sono congelate separatamente per knee e ankle. Per ogni segnale
continuo la coppia di aggregazioni obbligatorie è episode RMS e massimo
assoluto; per saturazione sono obbligatori count e fraction sui medesimi sample:

| Famiglia | Segnale base | Unità | Aggregazioni |
|---|---|---|---|
| Saturazione | `tau_input_saturated` | count, 1 | count, fraction |
| Errore coppia | `torque_error_nm` | N m | RMS, abs max |
| Coppia molla | `tau_spring_nm` | N m | RMS, abs max |
| Rate coppia molla | `tau_spring_rate_nm_s` | N m/s | RMS, abs max |
| Velocità motore | `motor_speed_rad_s` | rad/s | RMS, abs max |
| Accelerazione motore | `motor_accel_rad_s2` | rad/s² | RMS, abs max |
| Potenza motore | `motor_power_w` | W | RMS, abs max |

Per ciascun aggregato continuo il cap è
`C <= A + max(1e-6, 1e-9 * abs(A))`, dove `1e-6` è espresso nell'unità della
metrica aggregata. Per `tau_input_saturated_count` vale `C <= A`; la fraction
deve usare lo stesso denominatore finito e vale anch'essa `C <= A`. Il futuro
execution receipt deve mappare questi nomi normativi ai campi del trace senza
rinominarne la semantica e deve congelare il metodo di integrazione/derivazione
del rate prima dell'esecuzione. Una metrica mancante, non finita, rinominata
dopo il freeze o non confrontabile chiude il gate in `FAIL`.

## 7. Ordine, arresto e output del gate

Prima si completano e verificano tutte le sei unità paired A/B. Solo se tutte
sono `PASS` possono essere eseguite le sei unità C. Il primo `FAIL` arresta il
protocollo senza rescue, retuning, nuova geometria, modifica FSM o sostituzione
di seed.

Gli esiti terminali sono distinti e non intercambiabili:

- fallimento di A: `ERROR_H0_REFERENCE`; il riferimento non è valido, C resta
  chiuso e H0_sep non è autorizzato;
- mismatch A/B: `ERROR_SHADOW_NONINTERFERENCE`; C resta chiuso e H0_sep non è
  autorizzato;
- fallimento scientifico di C dopo PASS completo A/B:
  `FAIL_H0_V25_COMPATIBILITY`; consente soltanto di proporre un freeze separato
  per H0_sep, soggetto a nuova autorità esplicita;
- PASS completo di C: `PASS_H0_V25_COMPATIBLE`; H0_sep è vietato perché non
  necessario.

Il PASS delle 12 unità stabilisce soltanto `H0_BINARY_V25_CANDIDATE_READY`. Non
promuove V25, non apre 05/06, non rende il corridor training-ready e non
autorizza PPO. Se C fallisse, l'eventuale H0_sep resterebbe soggetto a una nuova
autorità esplicita secondo il piano storico; non viene avviato da questo
protocollo.

## 8. Authority e scope chiuso

I valori normativi da riportare nel futuro protocol lock sono:

```text
execution_authorized=false
h0_executed=false
training_authorized=false
ppo_updates_authorized=false
h0_sep_authorized=false
protected_trial_access_authorized=false
reserve_trial_access_authorized=false
corridor_authorized=false
runtime_promotion_authorized=false
primary_grf_modification_authorized=false
detector_retuning_authorized=false
```

Per passare `execution_authorized` a `true` servirà un successivo lock
no-clobber che congeli almeno: sorgenti runtime/driver e relativi hash,
implementazione `binary_active`, coesistenza analogico/V25, sorgente analogica
legacy comune e autorevole in A/B, esclusione degli eventi analogici sinistri in
C, schema completo dei trace, comparatore bit-exact, metriche e tolleranze SEA,
18 destinazioni di output vuote, timeout, ordine delle unità e receipt di
preflight. Fino ad allora lo scope termina con la definizione del protocollo e
nessun H0 può essere eseguito.
