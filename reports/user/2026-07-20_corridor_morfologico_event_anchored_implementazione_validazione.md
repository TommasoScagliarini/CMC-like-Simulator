# Corridor morfologico event-anchored: implementazione, test e validazione

Data: 2026-07-20

## Sintesi esecutiva

È stato implementato il nuovo contratto di fase morfologica sincronizzato con
gli eventi protesici validi della FSM:

- Heel Strike (HS) corrisponde sempre alla fase morfologica 0;
- Toe Off (TO) corrisponde sempre all'ancora canonica
  `alpha = 0.6223299989`;
- il successivo HS chiude il ciclo alla fase 1 e apre il ciclo seguente;
- stance e swing sono temporizzati separatamente;
- a runtime sono utilizzate soltanto informazioni causali: bootstrap nominale
  prima del primo ciclo completo e mediana degli ultimi cinque cicli protesici
  validi in seguito;
- la progressione è saturata all'estremo del segmento, quindi una stance lunga
  non può entrare nello swing prima di un TO realmente accettato dalla FSM.

È stato inoltre costruito un nuovo profilo AB06 in cui ciascuno dei 123 cicli
prescribed è rimappato separatamente su HS→TO e TO→HS prima di calcolare media
e deviazione standard.

Tutti i gate implementati sono passati:

| Livello | Esito |
|---|---:|
| ricostruzione deterministica del profilo | PASS byte-exact |
| test unitari event-anchored | 11/11 PASS |
| regressione reward preesistente | 32/32 PASS |
| smoke test FSM nel vero ambiente OpenSim | PASS |
| blocked cross-validation cronologica del profilo | PASS |
| replay offline causal vs legacy vs oracle | PASS |
| identità della reward con peso zero | PASS esatto |
| rollout live finale del checkpoint best con peso zero | PASS |
| confronto fisico/policy con rollout originale | differenza esatta zero |

La conclusione corretta è:

> Il nuovo corridor e la nuova sincronizzazione sono validati come strumento
> diagnostico non interferente. Non è ancora validata l'efficacia di un training
> PPO con `morphology_weight > 0` e questa attività non promuove, da sola, la
> policy a deployable su hardware.

Il file di training resta quindi intenzionalmente a:

```yaml
morphology_phase_mode: event_anchored
morphology_weight: 0.0
```

Non è stato modificato il peso delle reserve:

```yaml
reserve_residual_weight: 0.0
```

Questo evita di confondere la validazione del corridor con il problema ancora
aperto della GRF e delle reserve necessarie a compensare una dinamica esterna
non completamente realistica.

## Problema affrontato

Il profilo morfologico precedente era indicizzato con una singola percentuale
del ciclo HS→HS. A runtime la fase dipendeva invece dal periodo e dalla stance
stimati dalla FSM. Se il TO reale avveniva in una frazione del ciclo diversa da
quella media del profilo, l'evento poteva:

- spostare la fase in avanti o all'indietro;
- far confrontare la reference con una porzione cinematicamente sbagliata del
  profilo;
- creare salti dei limiti del corridor e della loss;
- penalizzare una traiettoria plausibile soltanto per un errore di
  sincronizzazione;
- rendere difficile distinguere una vera violazione morfologica da un errore
  del clock.

Questo era particolarmente problematico per un corridor che, per design, deve
lasciare libertà alla rete dentro una banda plausibile. La tolleranza del
corridor non può svolgere correttamente questa funzione se stance e swing non
sono allineati agli stessi eventi usati per costruire il profilo.

## Soluzione implementata

### Profilo prescribed event-warped

Per ciascun ciclo valido sono stati identificati:

```text
HS_i < TO_i < HS_(i+1)
```

La stance di ogni ciclo è rimappata linearmente su:

```text
[HS_i, TO_i] -> [0, alpha]
```

Lo swing è rimappato separatamente su:

```text
[TO_i, HS_(i+1)] -> [alpha, 1]
```

L'ancora `alpha` è la media dei duty factor dei 123 cicli di produzione:

```text
alpha = mean((TO_i - HS_i) / (HS_(i+1) - HS_i))
      = 0.6223299989
```

La griglia contiene 201 punti uniformi più `alpha` inserito esplicitamente,
per un totale di 202 punti. Il TO non è quindi soltanto interpolato: è un punto
esatto del profilo.

Provenance finale:

| Campo | Valore |
|---|---:|
| cicli completi sull'intero span GRF | 124 |
| cicli inclusi nella finestra di produzione | 123 |
| cicli esclusi | 1 |
| motivo esclusione | HS successivo oltre `t_end = 153 s` |
| periodo medio | 1.1282657419 s |
| duty factor medio | 0.6223299989 |
| deviazione standard duty factor | 0.0394725374 |
| punti fase | 202 |
| SHA-256 profilo finale | `33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55` |

Il profilo include ora anche metadati espliciti su:

- lato protesico sinistro;
- lato GRF sinistro usato per HS/TO;
- unità in radianti;
- convenzione di segno OpenSim;
- inversione del solo knee nei plot flexion-positive.

### Mapping causale runtime

Durante la stance:

```text
phase = alpha * clip((t - HS) / T_stance_est, 0, 1)
```

Durante lo swing:

```text
phase = alpha
      + (1 - alpha) * clip((t - TO) / T_swing_est, 0, 1)
```

La priorità delle stime temporali è:

1. bootstrap nominale prima del primo ciclo completo;
2. ultimo ciclo completo, disponibile per retrocompatibilità dei payload;
3. mediana causale delle durate stance e swing degli ultimi cinque cicli validi.

Nel rollout analizzato il bootstrap deriva da:

```text
periodo nominale = 1.58 s
stance nominale  = 0.68 * 1.58 = 1.0744 s
swing nominale   = 0.32 * 1.58 = 0.5056 s
```

Le mediane vengono aggiornate soltanto quando la FSM accetta un ciclo completo
HS→TO→HS che supera i gate anti-fake-cycle. Reset dell'episodio significa anche
reset completo dello storico.

### Retrocompatibilità e fail-fast

Il default della classe `RewardConfig` resta
`legacy_cycle_fraction`. I checkpoint e le configurazioni che non dichiarano
la nuova modalità mantengono quindi il comportamento storico.

La configurazione ex-novo corrente seleziona invece esplicitamente
`event_anchored`.

Un audit indipendente ha rilevato che il vecchio profilo esponeva comunque
`mean_to_phase` e avrebbe potuto essere caricato accidentalmente in modalità
event-anchored. Il runtime è stato quindi irrigidito:

- in modalità event-anchored è obbligatorio
  `phase_parameterization = event_warped_hs_to_to_to_hs_v1`;
- è obbligatorio `canonical_to_phase` finito e strettamente compreso tra 0 e 1;
- un profilo legacy associato per errore alla nuova modalità causa ora un errore
  immediato alla costruzione del wrapper;
- il fallback `mean_to_phase` resta disponibile soltanto nei percorsi legacy.

### Invarianti preservate

- nessuna modifica al plugin C++;
- nessuna modifica alla semantica del comando SEA;
- nessuna modifica alla dimensione o all'ordine dell'osservazione actor;
- nessuna modifica alle azioni del checkpoint best;
- nessuna modifica alla reward quando `morphology_weight = 0`;
- tutta la logica di rete/runtime resta dentro `Trajectory Generator/`;
- tutti gli script di test e validazione introdotti sono dentro `validation/`;
- path e metadati del profilo sono repo-relative e portabili tra macOS e
  Windows.

## Strategia di validazione

La validazione è stata separata in sei livelli, perché ciascuno risponde a una
domanda diversa.

### 1. Invarianti sintetiche

Sono stati testati:

- default legacy bit-compatible;
- metà stance e metà swing nel bootstrap;
- clamp agli estremi di stance e swing;
- uso dell'ultimo ciclo in assenza dello storico robusto;
- priorità della mediana robusta;
- reset dello storico;
- round-trip della configurazione;
- caricamento dell'ancora dal profilo;
- rifiuto di modalità e metadati invalidi;
- rifiuto esplicito del profilo legacy in modalità event-anchored;
- identità esatta della reward tra le due modalità con peso zero.

Esito: 11/11 PASS.

### 2. Riproducibilità del profilo

Il builder è stato eseguito in modalità scrittura e poi in modalità
`--check`. La ricostruzione finale coincide byte-per-byte con il JSON salvato.

Il profilo registra anche gli hash SHA-256 delle sorgenti IK e GRF, il numero di
righe, gli intervalli temporali e l'interpretazione delle unità.

### 3. Blocked cross-validation cronologica

I 123 cicli non sono stati divisi casualmente. Sono stati organizzati in cinque
blocchi temporali contigui. Per ogni fold:

- `alpha` è stimato soltanto sui quattro blocchi di training;
- media e deviazione standard sono stimate soltanto sui blocchi di training;
- il blocco contiguo rimanente è completamente held-out.

Questo evita la leakage ottimistica che si avrebbe mischiando cicli adiacenti
dello stesso regime.

Gate dichiarati:

| Gate | Soglia |
|---|---:|
| coverage aggregata | >= 95% |
| coverage del fold peggiore | >= 85% |
| escursione esterna p95 | <= 10° |
| riduzione dispersione media rispetto al profilo legacy | >= 10% |

Risultati:

| Metrica | Knee | Ankle |
|---|---:|---:|
| coverage aggregata | 99.706% | 96.744% |
| coverage stance | 99.774% | 97.374% |
| coverage swing | 99.594% | 95.700% |
| fold peggiore | 98.554% | 86.059% |
| coverage minima di un singolo ciclo | 89.604% | 73.267% |
| escursione esterna p95 | 1.706° | 8.339° |
| escursione esterna massima | 1.972° | 14.599° |
| riduzione dispersione media | 17.773% | 20.583% |
| riduzione dispersione al TO | 44.501% | 49.894% |

Il PASS dell'ankle va interpretato con cautela. Il Fold 1 rappresenta il regime
più lento:

```text
periodo medio = 1.502 s
duty factor medio = 0.689
coverage ankle = 86.059%
```

È appena 1.06 punti percentuali sopra il gate. Il profilo rappresenta bene
l'insieme AB06, ma questa prova non dimostra robustezza universale e non
giustifica ancora una generalizzazione automatica ad altri soggetti, modelli o
stili.

### 4. Replay offline del checkpoint best

La stessa trace registrata del checkpoint best è stata valutata con tre
contratti:

1. legacy: profilo e fase storici;
2. event-anchored causal: profilo nuovo e sole informazioni passate;
3. oracle retrospettivo: usa il prossimo HS futuro e serve esclusivamente come
   riferimento diagnostico.

Sono stati ricostruiti 4 HS, 3 TO e 3 cicli completi. Il supporto comune tra i
tre schemi è 451/500 step, pari al 90.2%.

Allineamento con l'oracle:

| Errore circolare di fase | Legacy | Event-anchored causal |
|---|---:|---:|
| media | 0.10997 cicli | 0.04620 cicli |
| p95 | 0.30829 cicli | 0.09273 cicli |
| massimo | 0.36851 cicli | 0.17395 cicli |

Il TO causale non regredisce mai. Il minimo salto raw osservato al TO è positivo:
`+0.00672 cicli`. Nel legacy erano presenti anche salti all'indietro.

Il salto massimo dei limiti del corridor migliora:

```text
legacy                0.71619 rad
event-anchored causal 0.44909 rad
oracle retrospettivo  0.08273 rad
```

Non è però eliminato: il massimo salto circolare causale agli eventi resta
0.1678 cicli. Questo è un motivo concreto per non partire con un peso elevato.

Sulla stessa traiettoria congelata:

| Metrica | Legacy | Event-anchored causal |
|---|---:|---:|
| campioni morphology disponibili | 495/500 | 495/500 |
| entrambi i giunti dentro il corridor | 51.717% | 64.646% |
| loss cumulativa | 78.4999 | 41.9689 |
| loss su supporto comune | 75.8673 | 39.3183 |

La riduzione della loss sul supporto comune è 48.175%.

Questo non è apprendimento della rete e non significa che la traiettoria sia
cambiata. La served reference è esattamente la stessa: è il contratto
profilo+fase a essere più coerente con gli eventi.

### 5. Shadow reward

I pesi sono stati applicati offline alla traiettoria congelata, senza aggiornare
la policy:

| Peso ipotetico | Penalità cumulativa | Return controfattuale | Delta relativo |
|---|---:|---:|---:|
| 0.0025 | 0.104922 | 52.322017 | -0.20% |
| 0.0050 | 0.209845 | 52.217095 | -0.40% |
| 0.0100 | 0.419689 | 52.007250 | -0.80% |

Questi numeri misurano soltanto la scala aritmetica sulla traiettoria corrente.
Non predicono come PPO cambierà la policy e non sono una validazione di
convergenza.

Con peso zero:

- return ricostruito: 52.42693952981385;
- return registrato: 52.42693952981385;
- differenza: 0;
- differenza per-step tra legacy, causal e oracle: 0.

### 6. Rollout live non-interferente

È stato rilanciato il vero `rl_module_best` con:

| Parametro | Valore |
|---|---|
| checkpoint | pilot50 `rl_module_best` |
| start offset | 1.956870983805102 s |
| seed | 123 |
| azione | deterministica |
| step | 500 |
| profilo | event-warped finale |
| modalità fase | event-anchored |
| morphology weight | 0 |

Confronto contro il rollout nominale originale:

| Canale | RMSE | massimo assoluto |
|---|---:|---:|
| raw policy action | 0 | 0 |
| applied policy action | 0 | 0 |
| cinematica protesica q | 0 | 0 |
| served reference | 0 | 0 |

Sono inoltre identici esattamente:

- indice e tempo di tutti i 500 step;
- osservazione actor come vettore e come mapping;
- media dell'azione;
- diagnostica del reference governor;
- return;
- numero di cicli validi;
- penetrazione GRF massima.

Metriche comuni:

```text
return                    52.42693952981385
reward media              0.1048538790596277
cicli validi              3
penetrazione GRF massima  0.02421743110349864 m
fine episodio             episode_time_limit
```

Questa è la prova più forte che l'introduzione del nuovo mapping, mantenendo il
peso a zero, non modifica actor, reference, plant o reward.

## Risultati reali del checkpoint rispetto al nuovo corridor

Nel plot finale le percentuali sono pesate per timestep e calcolate sui 495
campioni in cui la morphology è disponibile:

| Giunto | Dentro | Fuori | Frazione fuori | Escursione massima |
|---|---:|---:|---:|---:|
| knee | 393/495 | 102/495 | 20.606% | 0.513 rad |
| ankle | 370/495 | 125/495 | 25.253% | 0.436 rad |

Le sorgenti di fase sono:

```text
bootstrap nominale                    154 campioni
mediana di cicli protesici completati 341 campioni
```

I limiti ricostruiti dal profilo coincidono con quelli loggati con errore
massimo `2.22e-16 rad`.

Il risultato è utile proprio perché non forza una conclusione ottimistica: il
checkpoint produce curve plausibili, ma non è sempre dentro il corridor. Se il
termine verrà attivato, PPO avrà quindi un segnale non nullo su cui lavorare.

Plot finale:

```text
plot/07_15_2026_1_exnovo_50/
09_morphology_corridor_event_anchored_weight0.png
```

Il canvas dichiara esplicitamente:

- modalità `event_anchored`;
- nome del profilo event-warped;
- natura step-weighted delle percentuali;
- numero di campioni morphology disponibili;
- distinzione tra bootstrap e cicli FSM misurati.

## Rischi residui emersi dall'audit

### Morphology unavailable prima del primo HS

Quando la FSM è in `WAIT_HS`, la morphology è unavailable e la loss vale zero.
Nel rollout sano questo riguarda soltanto 5/500 step. Tuttavia, con peso
positivo, una policy patologica che non produce mai il primo HS potrebbe
teoricamente evitare il termine.

Prima di promuovere il peso va quindi scelto e testato uno dei seguenti design:

- timeout esplicito per `WAIT_HS`;
- ledger/clawback dell'availability;
- gate di training che respinga episodi senza primo HS;
- combinazione dei precedenti.

Non è stata introdotta automaticamente una nuova penalità, perché sarebbe una
modifica ulteriore della reward e va isolata sperimentalmente.

### Stato temporale robusto non completamente osservato dall'MLP

La reward usa le mediane storiche delle durate stance/swing. L'actor vede stato
FSM, elapsed time, eventi, fase ciclica online e durata ciclo online, ma non
riceve direttamente le due mediane usate dal corridor.

Questo introduce una piccola componente parzialmente osservata. Le alternative
da confrontare prima di un training lungo sono:

- esporre uno stato di fase event-anchored deployable all'actor;
- usare soltanto timing nominale/eventi per la reward;
- mantenere le mediane ma dimostrare empiricamente che la piccola ambiguità non
  destabilizza PPO.

Aggiungere nuove feature all'actor cambierebbe la dimensione dell'input e non
può essere fatto senza una strategia esplicita di migrazione del checkpoint H0.

### Discontinuità residua

Il mapping causale migliora nettamente il legacy ma non può conoscere il futuro
come l'oracle. Restano:

- salto massimo di fase: 0.1678 cicli;
- salto massimo dei bound: 0.4491 rad;
- salto massimo della loss a valore cinematico fisso: 0.3982.

Questo giustifica un primo peso piccolo e logging per-evento.

### Generalizzazione

La blocked CV usa regimi temporali distinti, ma tutti provengono dallo stesso
recording AB06. Non sono ancora coperti:

- altri soggetti;
- altre morfologie;
- altri modelli OpenSim;
- diversi stili e velocità fuori dal range AB06;
- convenzioni coordinate differenti senza mapping esplicito.

I nuovi metadati di laterality e segno preparano il formato, ma non costituiscono
una prova di generalizzazione.

## Reserve actuator e GRF

Il termine reserve non è stato attivato. La motivazione resta quella discussa:
se la GRF applicata non è dinamicamente coerente con la traiettoria e con il
modello, una reserve elevata può compensare l'errore della GRF invece di
segnalare un difetto della policy.

Penalizzarla direttamente in questa fase rischierebbe di:

- attribuire alla policy un errore esterno;
- spingere la rete verso una traiettoria che compensa un bias della GRF;
- migliorare artificialmente la metrica reserve peggiorando la fisica;
- confondere l'esperimento sul corridor.

La reserve deve quindi rimanere monitorata come gate fisico, ma non diventare
reward finché il ramo GRF non è validato o finché non è disponibile una
scomposizione affidabile tra errore di forcing esterno ed errore cinematico.

## File modificati

### Runtime e configurazione

```text
Trajectory Generator/prosthetic_phase_fsm.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
Trajectory Generator/baseline_MLP/morphology_profiles/
  ab06_prosthetic_event_warped_mean_std_corridor.json
```

### Test e validazione

Tutti gli script introdotti sono in `validation/`:

```text
validation/build_event_warped_morphology_profile.py
validation/test_event_anchored_morphology.py
validation/validate_event_warped_morphology_profile.py
validation/validate_event_anchored_morphology.py
validation/plot_morphology_corridor_rollout.py
```

Sono stati aggiornati anche:

```text
validation/test_reward_function.py
validation/validate_training_config.py
```

Per il confronto del rollout è stato riutilizzato:

```text
validation/compare_rollout_traces.py
```

Non sono stati aggiunti script di validazione fuori dalla cartella
`validation/`.

## Artefatti prodotti

### Validazione profilo

```text
validation/event_anchored_morphology_runs/
  2026-07-20_profile_blocked_cv/
    profile_validation.json
    profile_validation.md
    profile_dispersion_comparison.png
    blocked_cross_validation_coverage.png
```

### Audit offline del checkpoint best

```text
validation/event_anchored_morphology_runs/
  2026-07-20_best/
    event_anchored_morphology_summary.json
    event_anchored_morphology_summary.md
    phase_alignment.png
    corridor_coverage.png
```

### Rollout live e confronto

```text
Trajectory Generator/runs/rollout/validation/
  event_anchored_morphology_runs/
    2026-07-20_weight0_live_rollout_final/

validation/event_anchored_morphology_runs/
  2026-07-20_weight0_live_rollout_final_comparison.json
```

## Test e verifiche eseguiti

```text
python validation/build_event_warped_morphology_profile.py --check
PASS, rebuild byte-exact

python validation/test_event_anchored_morphology.py
PASS, 11/11

python validation/test_reward_function.py
PASS, 32/32

python validation/validate_training_config.py
PASS, tutti gli smoke check

python validation/test_phase_fsm_prescribed_env.py
PASS nel vero ambiente OpenSim

python validation/validate_event_warped_morphology_profile.py --fail-on-gate
PASS

python validation/validate_event_anchored_morphology.py --fail-on-gate
PASS

python -m py_compile <file coinvolti>
PASS

ruff check <file coinvolti>
PASS

git diff --check
PASS
```

## Stato della validazione

| Domanda | Risposta |
|---|---|
| il profilo è riproducibile? | sì, byte-exact |
| HS e TO sono ancorati coerentemente? | sì |
| il runtime usa informazioni future? | no |
| il vecchio profilo può essere caricato per errore in event mode? | no, fail-fast |
| il checkpoint cambia con peso zero? | no, identità esatta |
| il corridor descrive bene AB06? | sì nel complesso, con debolezza ankle nel regime lento |
| il checkpoint best è sempre nel corridor? | no |
| è stata addestrata una policy con peso positivo? | no |
| è dimostrata generalizzazione multi-modello? | no |
| sono risolti GRF e reserve? | no |
| questa prova basta per hardware deployment? | no |

## Prossimo esperimento raccomandato

Il passo successivo non dovrebbe essere un training lungo immediato. È più
informativo un A/B controllato e breve, con un solo fattore modificato:

| Ramo | Morphology weight |
|---|---:|
| controllo | 0 |
| trattamento prudente | 0.0025 |
| trattamento principale | 0.005 |

Protocollo suggerito:

1. stesso checkpoint iniziale per tutti i rami;
2. stessi start, seed, batch e learning rate;
3. 10-20 update iniziali;
4. checkpoint a ogni update;
5. screening multistart e con seed held-out;
6. logging specifico per HS/TO, availability, clamp e salti della loss;
7. nessuna variazione contemporanea del learning rate, per non confondere
   l'effetto del corridor;
8. promozione soltanto se la loss morfologica e le uscite dal corridor
   diminuiscono senza peggiorare cicli, penetrazione, return, SEA, reserve e
   distanza dall'actor iniziale.

Prima di trasformare il pilot in un run da 50-100 update devono inoltre essere
decisi e testati:

- il trattamento di `WAIT_HS` senza morphology disponibile;
- la gestione dello stato temporale robusto non completamente osservato;
- il gate specifico per il regime ankle lento;
- una prima prova su almeno un profilo/modello esterno ad AB06.

Solo dopo questi passaggi avrà senso discutere l'attivazione stabile del
corridor nella reward e il suo contributo alla deployabilità complessiva.
