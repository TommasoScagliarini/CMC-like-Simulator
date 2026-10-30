# Smoke test e analisi del sigma di esplorazione V26C/J20

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Data: 2026-08-31

Instruction check token: CMC_AGENT_OK_2026

## Verdetto

È possibile aumentare il sigma operativo del corrente actor V26C/J20 da
`0.005` a **`0.010`**. Il raddoppio è supportato da:

- `6/6` rollout nominali da `500` step;
- `2/2` rollout aggiuntivi agli start `-0.20 s` e `+0.20 s`;
- almeno due cicli validi in ogni prova (`3` nello start `-0.20 s`);
- zero action clipping;
- zero `phase_timeout_stance` e `phase_timeout_swing`;
- zero `resync_count`, `hs_cancelled_count` e invalid event;
- zero `morphology_causal_contract_failure`;
- superamento del gate cinematico J3/J19 in ogni prova;
- penetrazione massima `26.8901 mm`, sotto il limite hard binding di
  `28 mm`.

`sigma=0.015` è il **ceiling frozen-policy osservato**, non il valore da usare
nel primo training: passa `6/6` seed e `2/2` start perturbati, ma raggiunge
`27.5579 mm`, lasciando appena `0.4421 mm` rispetto all'hard limit.

`sigma=0.020` e `0.030` sono esclusi: sul seed 127 terminano per penetrazione
hard rispettivamente a `199` e `200` step (`28.2061` e `28.1840 mm`).

La conclusione corretta è quindi:

> `sigma=0.010`, isotropico e congelato, è idoneo per un **pilot PPO breve di
> 5-10 iterazioni**. Gli smoke test non autorizzano ancora un training lungo né
> dimostrano che l'aumento produca apprendimento migliore.

## Domanda scientifica

Il checkpoint corrente usa una deviazione standard gaussiana costante di circa
`0.005` per ciascuna delle due azioni. La domanda non era soltanto se un actor
con sigma maggiore riuscisse a produrre un singolo rollout, ma se:

1. la media dell'actor potesse essere preservata esattamente;
2. l'esplorazione aggiuntiva sopravvivesse realmente allo slew limiter;
3. il closed loop restasse stabile su più seed e più start;
4. non comparissero failure FSM, morphology, clipping o saturazioni SEA;
5. esistesse un intervallo utile prima che il rumore diventasse distruttivo o
   venisse in gran parte assorbito dalle sicurezze.

## Che cosa rappresenta sigma

Sigma è la deviazione standard della distribuzione gaussiana dell'azione
normalizzata. Non aumenta la capacità rappresentazionale della rete e non è un
learning rate. Aumenta la dispersione delle azioni campionate attorno alla
media prodotta dall'actor.

Il mapping fisico usa scale `0.75 rad` per il knee e `0.70 rad` per l'ankle.
Con `sigma=0.010`, prima delle sicurezze, una deviazione standard corrisponde a:

| Giunto | rumore raw 1-sigma | gradi |
|---|---:|---:|
| knee | `0.0075 rad` | `0.430 deg` |
| ankle | `0.0070 rad` | `0.401 deg` |

Passare da `0.005` a `0.010` significa:

- sigma `2x`;
- varianza per azione `4x`;
- incremento di entropia della distribuzione 2D pari a `1.3863 nat`.

È quindi un aumento sostanziale, non un aggiustamento cosmetico.

## Actor e invarianti

Sorgente:

```text
Trajectory Generator/runs/training/
MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/
rl_module_last
```

Manifest 35D:

```text
Trajectory Generator/baseline_MLP/validation/
v26c_july_replica_2026-08-26/
v26c_j20_actor_feature_manifest_overlay.json
```

Per ogni candidato è stata creata una copia isolata tramite
`configure_actor_exploration.py`. Sono state modificate soltanto le righe della
testa `log-std`:

```text
pi.1.weight[2:4, :] = 0
pi.1.bias[2:4]      = log(sigma_knee), log(sigma_ankle)
```

Verifiche comuni a tutti gli otto moduli derivati:

- parametri della mean-network esattamente invariati;
- validazione della media sui dati reali: differenza zero;
- save/reload bit-exact;
- `ppo_updates = 0`;
- nessun update del critic;
- pesi log-std nulli e bias costanti;
- actor 35D, observation contract e action semantics invariati.

Il modulo `sigma=0.010` ha actor digest completo
`88e766b33d3d5fa263fe2be1021786684bc7c6ec3c19e3149b39a178abd8f88a`;
la differenza dal digest sorgente riguarda esclusivamente la log-std. La
mean-network resta byte-identica.

## Validazione dell'harness

Prima dello sweep sono stati riprodotti due risultati J19B:

| Prova | risultato corrente | risultato J19B | esito |
|---|---:|---:|---|
| deterministico nominale, penetrazione max | `24.4214028767 mm` | `24.4214028767 mm` | exact |
| `sigma=0.005`, seed 123, penetrazione max | `24.6818251289 mm` | `24.6818251289 mm` | exact |

Anche l'RMS del rumore realizzato a `sigma=0.005`, seed 123, coincide con la
cella J19B D. Questo elimina come spiegazione differenze accidentali di actor,
reset, FSM o runtime.

## Protocollo

Sono stati eseguiti `35` rollout frozen-policy. Nessun PPO è stato avviato.

### Campionamento

- modalità `stochastic_held`;
- hold di `1` step, cioè nuovo campione a `100 Hz`, coerente con il caso
  training-like;
- `500` step / `5 s` per episodio;
- common random numbers: lo stesso seed usa la stessa sequenza normale
  standard tra sigma diversi;
- coarse sweep isotropico: `0.005`, `0.0075`, `0.010`, `0.015`, `0.020`,
  `0.030`;
- ablation: `[knee 0.005, ankle 0.010]` e
  `[knee 0.010, ankle 0.005]`;
- sei seed nominali `123-128` per `0.010`, `0.015` e `0.020`;
- `0.030` fermato dopo il primo hard fail: cinque seed erano già disponibili;
- start nominale `1.956870983805102 s` e perturbazioni `+/-0.20 s`.

### Gate

Un rollout passa soltanto con:

- `500` step e almeno due cicli;
- penetrazione `<= 28 mm`;
- zero clipping, invalid event, timeout stance/swing;
- zero resync, cancellazioni HS e failure morphology;
- telemetria HS/TO coerente;
- ankle minimo `<= -0.03 rad`;
- escursione ankle `>= 0.30 rad`;
- escursione knee `>= 0.60 rad`;
- knee sempre in `[-1.5, 0) rad` e ankle in `[-0.7, 0.7] rad`.

Le bande di penetrazione non sono state riscritte:

- `>20 mm`: diagnostica soft;
- `>=25 mm`: diagnostica storica di luglio;
- `>28 mm`: hard fail binding.

## Risultati nominali multi-seed

La tabella riporta solo i rollout nominali. Per `0.005` sono inclusi il seed
123 e il controllo avverso 127; J19B contiene inoltre i seed storici 124-125.

| sigma | n | 500/pass | max pen mm | seed con >=25 mm | return medio dei completi | rumore dopo slew knee/ankle rad | intervento slew knee/ankle |
|---:|---:|---:|---:|---:|---:|---:|---:|
| `0.005` | 2 | `2/2` | `25.6843` | 1 | `39.4200` | `.00320 / .00313` | `28.6% / 15.2%` |
| `0.010` | 6 | `6/6` | `26.8901` | 2 | `38.6709` | `.00606 / .00604` | `31.7% / 22.2%` |
| `0.015` | 6 | `6/6` | `27.5579` | 2 | `38.5379` | `.00854 / .00827` | `36.7% / 31.7%` |
| `0.020` | 6 | `5/6` | `28.2061` | 4 | `37.8488` | `.01053 / .01008` | `43.1% / 39.8%` |
| `0.030` | 5 | `4/5` | `28.1840` | 3 | `34.1707` | `.01376 / .01262` | `52.3% / 51.2%` |

Il return medio per `0.020` e `0.030` considera soltanto gli episodi completi;
includere le terminazioni porterebbe rispettivamente a `31.2389` e `26.9890`.

## Dose-response sul seed avverso 127

Il seed 127 è il test più informativo perché usa esattamente la stessa
sequenza normale standard a ogni sigma.

| sigma | step | max pen mm | esito |
|---:|---:|---:|---|
| `0.005` | 500 | `25.6843` | PASS |
| `0.010` | 500 | `26.8901` | PASS |
| `0.015` | 500 | `27.5579` | PASS, margine hard `0.4421 mm` |
| `0.020` | 199 | `28.2061` | FAIL penetrazione |
| `0.030` | 200 | `28.1840` | FAIL penetrazione |

Allo step 199 la progressione da `0.005` a `0.020` è quasi monotona:
`25.684 -> 26.890 -> 27.558 -> 28.206 mm`. Tutte le prove sono nello stato
`STANCE_AFTER_HS`; non compaiono timeout, resync o cancellazioni. Il campione
di rumore dell'ultimo step del fail `0.020` è piccolo, quindi la terminazione
non deriva da un singolo outlier: è l'effetto closed-loop accumulato della
sequenza esplorativa.

## Multi-start

| sigma | start | step | cicli | max pen mm | gate |
|---:|---:|---:|---:|---:|---|
| `0.010` | `-0.20 s` | 500 | 3 | `23.5599` | PASS |
| `0.010` | `+0.20 s` | 500 | 2 | `22.9227` | PASS |
| `0.015` | `-0.20 s` | 500 | 2 | `24.0378` | PASS |
| `0.015` | `+0.20 s` | 500 | 2 | `24.7368` | PASS |

Gli start perturbati non sono il punto fragile; il caso limitante resta il
seed nominale 127.

## Quanto rumore sopravvive alle sicurezze

Il confronto è stato eseguito nello stesso stato, applicando lo slew sia alla
media sia all'azione campionata. Il rapporto RMS dopo/prima dello slew vale:

| sigma | knee | ankle |
|---:|---:|---:|
| `0.005` | `85.0%` | `90.8%` |
| `0.010` | `81.1%` | `86.0%` |
| `0.015` | `76.2%` | `78.5%` |
| `0.020` | `70.3%` | `72.1%` |
| `0.030` | `61.4%` | `60.9%` |

A `0.010` la maggior parte dell'aumento esplorativo raggiunge quindi il target
post-slew. Oltre `0.015` compaiono rendimenti decrescenti: si aumenta molto la
dispersione raw, ma una quota crescente viene modificata dallo slew limiter.

Il governor C2 non mostra saturazione sistematica dei propri hard limit. Su
tutta la batteria:

- velocity-limit fraction massima: `0`;
- acceleration-limit fraction non nulla soltanto nello start `-0.20 s` a
  `sigma=0.010` e `0.015`, media `0.164%` in entrambi;
- jerk-limit fraction negli stessi due casi: `0.109%`;
- saturazione `tau_input` SEA: `0`;
- action clipping: `0`.

La divergenza RMS della served rispetto al deterministico cresce comunque con
sigma, perché include sia il filtraggio sia la successiva evoluzione
closed-loop:

| sigma | knee rad | ankle rad |
|---:|---:|---:|
| `0.005` | `.00730` | `.00470` |
| `0.010` | `.01639` | `.00965` |
| `0.015` | `.02638` | `.01393` |
| `0.020` | `.03746` | `.01797` |
| `0.030` | `.06323` | `.02951` |

Slew e C2 smussano il rumore, ma non annullano il covariate shift dinamico.

## Qualità morfologica e return

Sul confronto paired seed 123, la morphology loss media cresce dopo
`sigma=0.010`:

| sigma | knee loss | ankle loss | return |
|---:|---:|---:|---:|
| `0.005` | `.000321` | `.011114` | `39.1980` |
| `0.010` | `.000502` | `.010551` | `39.2477` |
| `0.015` | `.000711` | `.011597` | `38.9385` |
| `0.020` | `.000918` | `.013236` | `38.1827` |
| `0.030` | `.001493` | `.017890` | `36.9407` |

Il plateau/fallimento non dipende da clipping o saturazione: è una perdita di
qualità closed-loop. Un seed completo a `0.030` scende fino a return `28.7106`.

## Ablation ankle-specific

La variante `[knee 0.005, ankle 0.010]` passa `3/3` seed nominali, con return
medio `38.9811` e penetrazione massima `25.2244 mm`. Non mostra però un
miglioramento della morphology loss ankle rispetto all'isotropico `0.010` sui
medesimi seed; un aumento di sigma non è di per sé un segnale ankle-specific.

Questa variante può restare una futura ablation se si vuole limitare la
perturbazione globale, ma i dati correnti non giustificano preferirla al
semplice `0.010/0.010`. La variante opposta `0.010/0.005` è stata eseguita su
un solo seed e resta soltanto diagnostica.

## Confronto storico con luglio

Nei test del 12 luglio il precedente actor DAgger era molto più fragile:

- `sigma=0.005` e `0.010` fallivano intorno a 200 step;
- il primo valore training-like utilizzabile era `0.003`;
- il teacher prescribed tollerava `0.005`, mostrando che la fragilità era
  soprattutto nell'actor closed-loop.

Il corrente J20, invece, completa `6/6` seed a `0.010`, incluso il seed 127 che
fa fallire i candidati superiori. Questo è un miglioramento reale della
robustezza dell'actor corrente; non è corretto importare automaticamente il
limite `0.003` del vecchio lineage.

Riferimenti storici:

- [preflight della varianza del warm start di luglio](2026-07-12_warm_start_exploration_variance_preflight.md);
- [diagnostica teacher/actor e rumore correlato](2026-07-12_teacher_noise_correlated_actor_diagnostics.md);
- [readiness H1 sigma 0.003](2026-07-12_readiness_h1_sigma0003.md);
- [qualificazione J19B corrente](2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md);
- [held-out J19C corrente](2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md);
- [critic rewarm J20 e split morphology](2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).

## Decisione operativa proposta

### Primo pilot

Usare:

```text
sigma_knee  = 0.010
sigma_ankle = 0.010
freeze_logstd = true
```

Checkpoint sperimentale già materializzato:

```text
Trajectory Generator/baseline_MLP/validation/
v26c_sigma_exploration_smoke_2026-08-31/
actors/iso_0p010/rl_module_warm_start
```

Non renderei la log-std apprendibile nel primo pilot: cambierebbe insieme
ampiezza di esplorazione e mean-policy, rendendo impossibile attribuire cause a
un'eventuale regressione. Il primo esperimento deve cambiare una sola
dimensione rispetto al training corrente: il sigma costante.

### Durata e monitoraggio

Eseguire `5-10` iterazioni PPO, non `50`, mantenendo il checkpoint sorgente
immutato. Monitorare a ogni iterazione:

- episode return e length per episodio completo;
- entropy, `mean_kl_loss` e clip fraction PPO;
- sigma realizzato e sigma effettivo post-slew;
- conteggi nelle bande `20/25/28 mm`;
- `phase_timeout_stance/swing`;
- `resync_count` e `hs_cancelled_count`;
- `morphology_causal_contract_failure`;
- morphology loss knee/ankle;
- served-vs-deterministic e served-vs-actual;
- action clipping, governor limits e saturazione SEA;
- deriva della mean-network rispetto a J20.

Stop immediato se compare una penetrazione `>28 mm`, un failure del contratto
morphology/FSM o una regressione sistematica di length/cicli.

### Valori non raccomandati

- `0.015`: conservare come upper smoke ceiling; non usarlo come primo pilot.
- `0.020` e `0.030`: esclusi dal hard gate.
- log-std apprendibile: non testata e non autorizzata da questi risultati.
- training lungo: non autorizzato dai soli frozen-policy smoke test.

## Limiti dell'evidenza

Questi test dimostrano tolleranza comportamentale dell'actor congelato, non
efficacia di apprendimento. Durante PPO la mean-policy cambia e può spostare la
distribuzione degli stati verso regioni non visitate in questa batteria.

Inoltre:

- sei seed sono una batteria utile ma non una stima precisa della probabilità
  di failure rara;
- il risultato riguarda AB06, il modello e il runtime correnti;
- non è una validazione hardware;
- in deployment deterministico sigma non aggiunge rumore, ma durante training
  modifica gli stati da cui l'actor apprende;
- reward, corridor, slew e C2 sono quelli correnti; una modifica di questi
  elementi richiede una nuova qualifica.

## Artefatti

Root dell'esperimento:

```text
Trajectory Generator/baseline_MLP/validation/
v26c_sigma_exploration_smoke_2026-08-31/
```

Contenuto principale:

- `protocol.json`: protocollo, invarianti e amendment adattivi;
- `actors/`: otto moduli constant-sigma isolati;
- `rollouts/`: 35 rollout con summary, watchdog e policy trace;
- `analysis.json`: metriche complete per-run e aggregati;
- `run_metrics.csv`: tabella piatta.

Analizzatore riproducibile, accanto alla root dell'esperimento:

```text
Trajectory Generator/baseline_MLP/validation/
analyze_v26c_sigma_exploration_smoke.py
```

## Test e verifiche eseguite

- `py_compile` dell'analizzatore: PASS;
- validità JSON del protocollo: PASS;
- configurazione actor: `8/8` mean exact, save/reload exact, zero update;
- riproduzione J19B deterministica e sigma `0.005`: exact;
- common-random-number scaling: verificato;
- `35` summary/trace analizzati;
- gate multi-seed e multi-start calcolati automaticamente;
- zero clipping e zero saturazione SEA nell'intera batteria;
- failure `0.020/0.030` preservati, non esclusi dagli aggregati.

## TODO

- [ ] Preregistrare il pilot PPO `5-10` iterazioni con sigma fisso `0.010`.
- [ ] Creare una configurazione/run directory separata senza modificare il
  checkpoint J20 o il canonico lungo.
- [ ] Verificare dopo ogni iterazione i gate sopra elencati e fare rollout
  deterministico dei checkpoint prodotti.
- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto sui dati
  del pilot.
- [ ] Rivalutare sigma dopo l'introduzione dell'actor predittivo o di una nuova
  logica causale del morphology corridor.
- [ ] Ripetere la qualifica per soggetti/modelli diversi da AB06 nel futuro
  percorso EPIC di generalizzazione.
