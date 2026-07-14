# Affidabilita online_grf come segnale reward debole o forte

Data: 2026-06-22

## Problema

L'obiettivo e capire se `online_grf` puo entrare nella reward come segnale forte, debole o solo diagnostico.

Per questa analisi il prescribed GRF AB06 e trattato come oracolo. Il profilo principale da giudicare resta `AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`, cioe la versione piu matura del ramo online/hybrid GRF.

Il punto critico non e se il plugin calcola numericamente la GRF online in modo coerente: quello e gia stato verificato. Il punto e se il segnale e biomeccanicamente affidabile abbastanza da guidare una policy.

## Strategia

Ho usato tre livelli di evidenza:

1. Sintesi dei report storici `online_grf` gia presenti in `reports/user/`.
2. Audit di tutti i rollout che contengono `rollout_episode_online_grf.sto`, confrontati con `models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot`.
3. Audit degli artefatti CMC-like gia presenti in `results/_online_grf_macos_arm64_*`, integrato con i risultati gait-scale documentati nei report storici.

Addendum dopo il follow-up del 2026-06-22: ho rigenerato una coppia CMC-like gait-scale su `[11.99, 13.99] s`, `dt = 0.001`, confrontando `prescribed` e `online` pure-active con il profilo v2. I risultati sono riportati sotto nella sezione "Simulazione CMC-like prescritta vs online".

## Sintesi report storici

| Report | Evidenza principale | Implicazione |
|---|---:|---|
| `2026-06-08_creazione_validazione_grf_online.md` | Audit macOS plugin/Python: max error circa `1.7e-12 N` | La discrepanza non e un bug numerico del plugin. |
| `2026-06-08_correzione_full_wrench_grf_online.md` | Full wrench residual/tangent migliora impulso e run 500 ms, ma restano failure | La correzione migliora la magnitudo, non valida il segnale come sostituto del prescribed. |
| `2026-06-08_analisi_cause_grf_online_non_utilizzabile.md` | Modello di contatto memoryless/basis + residual non riproduce il wrench prescribed | Il limite e strutturale/fisico, non solo parametrico. |
| `2026-06-08_validazione_gait_scale_grf_online.md` | Pure-online 2 s completa ma acceptance `FAIL`; `pelvis_ty_reserve_p95_ratio` circa `11.12x` | Online pura e stabile, ma non validata dinamicamente. |
| `2026-06-08_grf_online_sintesi_e_decisione_architetturale.md` | Magnitude fix porta impulso totale `0.827 -> 0.943`, ma reserve ratio resta circa `10.25x` | Anche quando l'impulso migliora, il mismatch di timing/wrench continua a generare compensazioni. |
| `2026-06-08_validazione_online_grf_training_inference.md` | Profilo hybrid scelto come terreno fisico per training/inference | La scelta hybrid e pragmatica, non una validazione di online come oracolo biomeccanico. |

Conclusione storica: `online_grf` e numericamente stabile e utile per diagnosticare il contatto, ma non e stata validata come sostituto quantitativo del prescribed GRF.

## Audit rollout

Sono stati trovati 11 rollout con `rollout_episode_online_grf.sto`. Escludendo smoke brevissimi, i rollout completi mostrano un pattern molto consistente:

| Rollout | L Fy impulse ratio | L Fy NRMSE | L Fy corr | L contact F1 | L contact recall | R Fy impulse ratio | R Fy NRMSE | R contact F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm` (`sym60`) | `0.212` | `0.835` | `0.450` | `0.779` | `0.639` | `0.788` | `0.466` | `0.882` |
| `MLP_imitation_rollout_06-17-2026_asym_actor_critic_100` (`asym100`) | `0.296` | `0.742` | `0.612` | `0.821` | `0.696` | `0.792` | `0.467` | `0.883` |
| `MLP_imitation_rollout_06-17-2026_resume_best_to_80` | `0.362` | `0.684` | `0.666` | `0.804` | `0.672` | `0.793` | `0.467` | `0.883` |
| `MLP_imitation_rollout_06-19-2026` | `0.355` | `0.684` | `0.680` | `0.839` | `0.723` | `0.794` | `0.467` | `0.883` |
| `MLP_imitation_rollout_06-21-2026` | `0.252` | `0.787` | `0.545` | `0.742` | `0.590` | `0.792` | `0.466` | `0.882` |

Distribuzione sui rollout completi:

| Metrica | Lato sinistro/protesico | Lato destro/sano |
|---|---:|---:|
| Fy impulse ratio | circa `0.212-0.362`, mediana circa `0.291` | circa `0.788-0.794`, mediana circa `0.792` |
| Fy NRMSE | mediana circa `0.766` | mediana circa `0.466` |
| Fy correlazione | mediana circa `0.579` | circa `0.778` |
| Contact F1 | mediana circa `0.792` | circa `0.882` |
| Contact recall | mediana circa `0.656` | circa `0.883` |

Audit sui cicli stance:

| Lato | Cicli analizzati | Fy impulse ratio min/med/max | Contact F1 mediano | Contact recall mediano |
|---|---:|---:|---:|---:|
| Sinistro/protesico | `32` | `0.155 / 0.240 / 0.475` | `0.767` | `0.622` |
| Destro/sano | `24` | `0.717 / 0.763 / 0.831` | `0.933` | `0.874` |

La lettura importante e che il lato sinistro non sbaglia solo in media: sottostima sistematicamente il carico anche dentro i cicli di stance. Il lato destro e molto piu coerente, ma non basta a validare il segnale per reward, perche il caso di interesse e proprio il lato protesico online.

## Audit CMC-like

Gli smoke locali disponibili sono molto brevi, circa 80 ms, quindi non validano il gait completo. Sono comunque coerenti con la storia gia nota:

| Run locale | Durata | L Fy impulse | R Fy impulse | `pelvis_ty` reserve p95 |
|---|---:|---:|---:|---:|
| `_online_grf_macos_arm64_online_sensor_smoke` | `0.079 s` | `1.048` | `1.223` | `8.23 N` |
| `_online_grf_macos_arm64_online_smoke` | `0.079 s` | `0.975` | `1.237` | `100.81 N` |

Anche nello smoke, passando da sensor a active online il reserve verticale di pelvi cresce molto. Gli smoke non sono sufficienti per accettare o rigettare il metodo, ma sono allineati con i report gait-scale.

Evidenza gait-scale dai report storici:

| Caso | Evidenza | Esito |
|---|---:|---|
| Pure online 2 s | total impulse `0.827`, right `0.786`, left `0.868` | Stabile ma non validato |
| Pure online 2 s | `pelvis_ty_reserve_p95_ratio` circa `11.12x`, p95 circa `475.4 N` | `FAIL` dinamico |
| Residual corrected | total impulse `0.943`, right `0.924`, left `0.961` | Impulso quasi corretto |
| Residual corrected | reserve ratio ancora circa `10.25x`, p95 circa `438 N` | Il problema non e solo magnitudo |

Questa e la prova piu forte contro l'uso reward forte: anche quando la magnitudo viene corretta, il sistema richiede ancora reserve elevate. Quindi il mismatch che conta per la dinamica vive anche in timing, COP, momenti e wrench complessivo.

## Simulazione CMC-like prescritta vs online

Su richiesta successiva, ho eseguito una nuova simulazione CMC-like head-to-head:

- finestra: `[11.99, 13.99] s`;
- `dt = 0.001`;
- run `prescribed`: `grf_mode=prescribed`;
- run `online`: `grf_mode=online`, `--no-external-loads`, profilo `AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`;
- il guard di penetrazione della run online e stato portato a `0.12 m` solo per evitare abort anticipati; non modifica la forza, modifica solo la soglia di stop;
- entrambe le run hanno completato `2000` step.

| Run | Status | Wall time |
|---|---:|---:|
| `prescribed` | complete | `171.6 s` |
| `online` | complete | `169.4 s` |

### GRF online vs prescribed

| Lato | Fy impulse ratio | Fy RMSE | Fy NRMSE rms | Fy corr | Contact F1 | COP mean | COP p95 | Moment vec NRMSE | Max penetration |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Sinistro | `0.868` | `135.1 N` | `0.284` | `0.900` | `0.932` | `72.7 mm` | `140.7 mm` | `62.012` | `17.4 mm` |
| Destro | `0.786` | `151.4 N` | `0.320` | `0.915` | `0.971` | `108.0 mm` | `176.0 mm` | `11.331` | `25.4 mm` |

Totale verticale:

| Metrica | Valore |
|---|---:|
| Impulso Fy online | `1256.0 N s` |
| Impulso Fy prescribed | `1519.1 N s` |
| Ratio totale | `0.827` |
| RMSE totale | `215.1 N` |
| NRMSE rms totale | `0.282` |
| Correlazione totale | `-0.144` |

La lettura e sottile ma importante: nella forward CMC-like pure-online, la verticale per lato ha correlazione discreta e impulse ratio non disastroso, soprattutto a sinistra. Pero il totale resta sotto-supportato (`0.827`) e la correlazione totale diventa negativa, segno che la distribuzione temporale del carico tra i due lati non riproduce bene l'oracolo.

### Reserve root

| Reserve root | Prescribed p95 abs | Online p95 abs | Ratio online/prescribed |
|---|---:|---:|---:|
| `pelvis_tx` | `119.11` | `90.12` | `0.76x` |
| `pelvis_ty` | `42.78` | `475.59` | `11.12x` |
| `pelvis_tz` | `61.58` | `81.85` | `1.33x` |
| `pelvis_tilt` | `77.33` | `122.75` | `1.59x` |
| `pelvis_list` | `92.14` | `71.59` | `0.78x` |
| `pelvis_rotation` | `82.72` | `22.01` | `0.27x` |

Questa nuova run conferma il risultato storico: il blocco decisivo non e solo la magnitudo verticale, ma il wrench dinamico complessivo. `pelvis_ty` passa da `42.78` a `475.59`, cioe `11.12x`. Questo e troppo alto per usare `online_grf` come segnale forte di reward.

## Valutazione segnale forte

### 1. Forte quantitativo

Uso candidato: reward dominante su magnitudo, impulso, carico verticale, simmetria o stance-load.

Verdetto: non accettabile.

Motivi:

- Il lato sinistro/protesico nei rollout completi ha impulse ratio Fy mediano circa `0.291`.
- Nei cicli di stance il mediano scende a circa `0.240`.
- NRMSE e correlazione sono troppo deboli per usare il segnale come misura fisica.
- Il bias e diverso tra lato sinistro e destro: usare il segnale per simmetria o carico rischia di premiare una compensazione sbagliata.

Un reward forte sulla magnitudo di `online_grf` spingerebbe la policy verso il modello di contatto online, non necessariamente verso una soluzione biomeccanicamente corretta.

### 2. Forte contact-load consistency

Uso candidato: penalizzare stance con carico insufficiente, oppure imporre coerenza forte tra fase di appoggio e carico.

Verdetto: non accettabile come termine forte.

Motivi:

- Il contact F1 sinistro e discreto, ma il recall mediano e solo circa `0.656` sui rollout completi e circa `0.622` sui cicli stance.
- Questo significa che una parte non trascurabile del contatto prescribed non viene vista come carico online affidabile.
- In single support e transizioni, una penalita forte rischia di punire la policy per un errore del sensore/modello, non per un errore biomeccanico reale.

Possibile uso: debole contact confidence, non vincolo forte.

### 3. Forte dinamico/biomeccanico

Uso candidato: usare `online_grf` per giudicare supporto, compensazioni, simmetria dinamica o qualita biomeccanica.

Verdetto: non accettabile.

Motivi:

- I report gait-scale mostrano reserve root elevate in active online rispetto a sensor/prescribed.
- La correzione dell'impulso non risolve il problema dei reserve, quindi il wrench complessivo non e affidabile.
- COP e momenti non possono essere trattati come validati se il sistema richiede ancora `pelvis_ty` reserve di ordine `10x-11x`.

Questa e la forma piu pericolosa di reward forte: puo trasformare un artefatto del contatto online in obiettivo di apprendimento.

## Decisione finale

Classificazione:

| Uso | Decisione |
|---|---|
| Validated strong reward signal | No |
| Weak-only reward signal | Si, ma solo per contact confidence a basso peso e con gating esterno |
| Diagnostic-only | Si, per magnitudo, impulso, COP, momenti, wrench e simmetria GRF |

Verdetto operativo: `online_grf` non e adatta come segnale forte. Puo restare utile come segnale debole di contatto, combinato con fase, eventi gait e cinematica del piede. Tutto cio che dipende da magnitudo, impulso, COP, momenti o wrench deve restare diagnostica finche non viene validato a gait-scale senza reserve root anomale.

## Raccomandazione per reward 2, 3 e 4

### Reward 2: contact-load consistency

Usabile solo in forma debole.

Forma consigliata:

- usare `online_grf` come confidence di contatto, non come target di carico;
- peso basso;
- attiva solo quando e coerente anche con fase gait, altezza piede e velocita piede;
- nessuna penalita forte sulla magnitudo Fy;
- nessun vincolo su impulso o simmetria GRF.

Interpretazione: puo aiutare a dire "probabilmente il piede e in appoggio", ma non puo dire "questo e il carico corretto".

### Reward 3: criterio biomeccanico legato al supporto

Non va costruita usando la magnitudo di `online_grf` come informazione primaria.

Alternativa piu robusta:

- premiare tracking cinematico e stabilita dinamica con grandezze disponibili indipendentemente dal modello di contatto;
- usare segnali SEA interni fisici, come `tau_spring` o stato attuatore, solo come diagnostica/regolarizzazione leggera se coerenti con l'obiettivo;
- se serve un gating di stance, usare `online_grf` solo come uno dei segnali deboli, non come arbitro.

### Reward 4: compensazioni e qualita dinamica globale

Non usare `online_grf` come segnale forte per giudicare compensazioni.

Motivo: se il contatto online induce gia reserve root elevate, un reward che lo tratta come verita dinamica rischia di premiare proprio la compensazione numerica.

Strategia preferibile:

- prima definire metriche offline di accettazione biomeccanica usando prescribed/oracle e reserve;
- poi cercare proxy reward che non dipendano direttamente dal wrench online;
- usare `online_grf` in training solo come regolarizzatore debole o logging.

## File modificati

- Aggiunto questo report: `reports/user/2026-06-22_online_grf_affidabilita_reward_debole_forte.md`.

Nessun codice, plugin, configurazione o file di training e stato modificato.

## Test e verifiche eseguite

- Letti e sintetizzati i report storici `online_grf` in `reports/user/`.
- Cercati tutti i rollout con `rollout_episode_online_grf.sto`.
- Confrontati i rollout con `AB06_SEASEA_GRF_FullSpan.mot`.
- Calcolati impulse ratio, RMSE/NRMSE/correlazione, contact precision/recall/F1 e penetrazione massima quando disponibile.
- Analizzati gli smoke CMC-like locali in `results/_online_grf_macos_arm64_*`.
- Eseguita nuova simulazione CMC-like `prescribed` vs `online` su `[11.99, 13.99] s`, `dt=0.001`, profilo v2.
- Calcolati GRF, COP, momenti, penetrazione e reserve root sulla nuova run.
- Integrate le metriche gait-scale documentate nei report storici per reserve root e active-vs-sensor ratio.
- Gli output grezzi della nuova run sono stati prodotti in `/private/tmp/cmc_grf_compare_20260622/` e rimossi dopo l'estrazione delle metriche.

## TODO

- Se serve il rapporto active-vs-sensor completo, aggiungere anche la run `online_sensor` sulla stessa finestra e calcolare il ratio `online/sensor`.
- Progettare un candidato reward debole di contact confidence che fonda fase gait, eventi, cinematica piede e `online_grf`.
- Validare ogni candidato reward offline sui rollout esistenti prima di inserirlo nel training.
