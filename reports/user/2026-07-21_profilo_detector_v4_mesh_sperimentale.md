# Profilo detector V4 mesh-based sperimentale: implementazione e design replay

Data: 2026-07-21

## Esito esecutivo

Il profilo V4 sperimentale è stato implementato, verificato geometricamente e
confrontato con il profilo corrente sui dati prescribed già aperti `50–100 s`.

L'esito contiene due risultati distinti:

1. **PASS geometrico**: il nuovo sensore toe è realmente aderente alla mesh del
   piede e non presenta più l'offset non fisico del profilo corrente;
2. **FAIL funzionale del detector**: il toe V4 entra in carico troppo tardi e
   troppo brevemente per garantire continuità fra appoggio heel e appoggio toe.

La decisione è quindi:

> Il profilo
> `AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO_v4_mesh_experimental.json`
> rimane sperimentale ed è **NOT PROMOTABLE**. Non deve essere usato per
> rollout attivo, training o deploy. Il profilo corrente, la FSM e il default
> `legacy_events` restano invariati.

Il risultato negativo non dipende dalla policy e non richiede training: il
difetto è già osservabile riproducendo la cinematica prescribed attraverso i
due sensori virtuali e la FSM di produzione.

## Che cosa significa “profilo V4 mesh-based separato”

La mesh viene usata soltanto offline per scegliere una collocazione geometrica
riproducibile dei sensori. Non viene usata per generare le GRF e non viene
aggiunto un nuovo algoritmo alla FSM.

Il sistema runtime resta quello semplice già definito:

- una sfera virtuale heel;
- una sfera virtuale toe/forefoot;
- i due segnali sono soltanto detector;
- Heel Strike richiede un heel stabile;
- Toe Off richiede l'uscita stabile di entrambi i sensori;
- isteresi e dwell restano quelli della FSM esistente.

Il V4 è contenuto in un nuovo JSON e può essere selezionato soltanto in modo
esplicito. Il profilo storico non è stato sovrascritto.

## Regola geometrica implementata

Per minimizzare le variabili è stato mantenuto il sensore heel corrente, già
aderente alla mesh e coincidente con il marker `L_Heel`. È stata modificata
soltanto `spheres[left_toe].location`.

| Sensore | Posizione locale [m] | Stato |
|---|---|---|
| heel | `[-0.1014100475, -0.0328195377, +0.0139956700]` | invariato |
| toe V4 | `[+0.1531111919, -0.0240498755, +0.0088225182]` | nuovo |

La posizione toe deriva esclusivamente dalla mesh `AM_foot_l.stl`:

- `x = limite anteriore della mesh − raggio`, così il bordo anteriore della
  sfera coincide con la punta della mesh;
- `z = centro della sezione locale della mesh` a quella coordinata `x`;
- `y` scelto in modo che la protrusione sotto la superficie plantare locale
  sia uguale a quella dell'heel, pari a `10.552 mm`;
- raggio, materiale, piano, frame, lato, nomi e sensori destri rimangono
  invariati.

Il builder non legge eventi HS/TO, GRF prescribed, rollout o output della
policy. La geometria non è quindi stata adattata alla temporizzazione del
dataset.

## Gate geometrico

| Metrica toe | Profilo corrente | V4 mesh |
|---|---:|---:|
| Distanza centro–mesh | 41.169 mm | 4.512 mm |
| Gap superficie sfera–mesh | 18.264 mm | 0.000 mm |
| Centro nei bounds assiali della mesh | no | sì |
| Fondo toe rispetto al fondo heel | 41.192 mm più basso | 8.770 mm più alto |

La V4 supera entrambi i controlli geometrici congelati:

- gap sfera–mesh `≤ 5 mm`;
- differenza fra fondi heel/toe `≤ 20 mm`.

Il fatto che il fondo toe sia più alto di `8.770 mm` è coerente con la forma
rocker della mesh nella regione anteriore. Forzare i due fondi alla stessa
quota avrebbe quasi raddoppiato la protrusione toe dentro la mesh e avrebbe
reintrodotto contatti artificialmente precoci.

## Protocollo del replay fixed-geometry

Il confronto temporale non è stato uno sweep. Sono stati congelati prima del
run:

- profilo corrente e profilo V4, entrambi hash-pinned;
- soglia ON `0.5 N`;
- soglia OFF `0.25 N`;
- dwell `30 ms`;
- timestamp primario `confirmed_time_s`;
- `10 ms` come cadenza runtime;
- `1 ms` come sensitivity obbligatoria;
- stessi criteri V3 su conteggi, ordine, timing, semantica e stato di fase.

Sono stati valutati gli stessi 50 cicli completi alle due cadenze. Il blocco
conteneva originariamente 51 cicli, ma l'ultimo HS cade a
`99.968786916 s`: la sua conferma causale richiederebbe di raggiungere il
confine dei `100 s`. Quel solo ciclo è stato classificato a priori come
right-boundary-censored ed escluso con il margine più conservativo
`dwell + 10 ms`.

Intervallo realmente campionato:

| Cadenza | Campioni | Primo | Ultimo | Campioni `t ≥ 100 s` |
|---|---:|---:|---:|---:|
| 10 ms | 4,913 | 50.000 s | 99.120 s | 0 |
| 1 ms | 49,121 | 50.000 s | 99.120 s | 0 |

Il blocco sealed `100–155.045 s` è rimasto chiuso.

### Audit fail-closed del protocollo

Tre dry-run sono stati arrestati e conservati prima di produrre un risultato
scientifico definitivo:

1. V1 ha rilevato che l'ultimo HS non era confermabile senza raggiungere il
   confine sealed;
2. V2 ha rifiutato un overshoot numerico della griglia rispetto
   all'`effective_end`;
3. V3 ha rifiutato insiemi di cicli diversi fra 10 ms e 1 ms.

La V4 finale usa un conteggio intero dei campioni e un common-cycle set scelto
con il margine della cadenza più conservativa. I failure precedenti non sono
stati usati per cambiare geometria, soglie, dwell o gate.

## Risultati del detector

### Conteggi e qualità della fase a 10 ms

| Metrica | Riferimento | Profilo corrente | V4 mesh |
|---|---:|---:|---:|
| Heel Strike | 51 | 49 | **1** |
| Toe Off | 50 | 49 | **0** |
| Cicli validi FSM | 50 attesi | 48 | **0** |
| F1 stance confermata | — | 0.941 | **0.083** |
| IoU stance confermata | — | 0.889 | **0.043** |

La V4 fallisce conteggi, precision/recall, ordine HS–TO–HS, timing, assenza di
eventi invalidi, F1 e IoU. L'unico controllo superato è che la latenza di
conferma degli eventi effettivamente accettati rispetta il dwell.

Il profilo corrente è molto migliore della V4, ma fallisce anch'esso il gate
formale two-sensor su questo blocco per due HS mancanti, un TO mancante e il
timeout già noto. Questo confronto non promuove quindi retroattivamente il
profilo corrente come detector two-sensor definitivo.

### Sensitivity a 1 ms

Il risultato V4 resta `1/51 HS`, `0/50 TO` e `0` cicli validi anche a `1 ms`.
F1 e IoU diventano rispettivamente `0.0819` e `0.0427`.

La quasi identità fra `10 ms` e `1 ms` dimostra che il fallimento V4 non è un
problema di aliasing o di risoluzione temporale.

## Diagnostica della continuità heel→toe

Le metriche seguenti erano preregistrate come diagnostiche, non come nuovi
gate numerici post-hoc.

| Metrica a 10 ms | Profilo corrente | V4 mesh |
|---|---:|---:|
| Toe-on dopo HS | circa 0.02–0.045 s | **0.44–0.58 s** |
| Durata contatto toe | 0.46–0.586 s | **0.05–0.10 s** |
| Picco toe minimo tra i cicli | 316.3 N | **75.8 N** |
| Gap massimo heel-off→toe-on | 0.020 s | **0.50 s** |
| Massimo both-OFF interno | 0.000 s | **0.39 s** |
| TO accettati | 49/50 | **0/50** |

Il meccanismo del fallimento è quindi chiaro:

1. l'heel produce il primo evento HS;
2. l'heel si scarica prima che il toe V4 inizi a sostenere il contatto;
3. la FSM osserva lunghi intervalli senza supporto sensoriale;
4. i primi candidati TO sono troppo precoci e vengono respinti correttamente;
5. seguono eventi `stance_load_too_low`;
6. la stance va in timeout intorno a `52.56 s`;
7. non si forma più alcuna sequenza HS–TO–HS valida.

La nuova sfera è quindi aderente alla mesh ma è troppo anteriore e troppo alta
rispetto alla regione effettivamente portante durante la stance prescribed.

## Interpretazione fisica

Questo esperimento mostra che “aderente alla mesh” è una condizione necessaria
ma non sufficiente. La punta visiva della mesh non coincide necessariamente
con la zona plantare che deve sostenere il contatto virtuale:

- la mesh può rappresentare il guscio del piede e non la suola/scarpa
  load-bearing;
- una sfera posta con il bordo anteriore alla punta può essere troppo avanti
  rispetto alla regione metatarsale;
- replicare l'offset dell'heel è geometricamente coerente, ma non garantisce
  una catena di supporto heel→forefoot continua.

Il buon timing aggregato del profilo corrente deriva invece anche dal toe
molto basso e staccato dalla mesh. Quel risultato non costituisce una valida
geometria fisica: è in parte una compensazione di calibrazione.

## Robustezza della policy

Non è stato eseguito rollout né training con V4. Sarebbe prematuro e poco
informativo: il detector fallisce già sul replay prescribed e cambierebbe
fortemente evidenze di contatto, reward e FSM.

Inoltre il profilo detector è `sensor-only` rispetto alle forze applicate alla
dinamica, ma non è comportamentalmente neutro: il suo aggregato alimenta anche
il detector legacy e alcune evidenze di contatto/reward. Selezionare V4 avrebbe
quindi potuto alterare il rollout anche mantenendo `legacy_events`.

La robustezza ottenuta finora è stata preservata perché:

- il profilo corrente non è stato modificato;
- la training config punta ancora al profilo corrente;
- `phase_fsm_input_mode: legacy_events` resta il default;
- la FSM e i suoi parametri non sono stati modificati;
- il checkpoint best, la policy, la reward e il training non sono stati usati.

## File implementati

Profilo e builder:

- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO_v4_mesh_experimental.json`;
- `validation/build_two_sensor_mesh_profile_v4.py`;
- `validation/test_two_sensor_mesh_profile_v4.py`.

Confronto fixed-geometry:

- `validation/compare_two_sensor_mesh_profiles_prescribed.py`;
- `validation/test_two_sensor_mesh_profile_v4_fixed_replay.py`;
- `validation/two_sensor_mesh_profile_v4_fixed_replay_protocol.json`;
- `validation/two_sensor_mesh_profile_v4_fixed_replay_protocol_v2.json`;
- `validation/two_sensor_mesh_profile_v4_fixed_replay_protocol_v3.json`;
- `validation/two_sensor_mesh_profile_v4_fixed_replay_protocol_v4.json`.

Post-processing:

- `validation/plot_two_sensor_mesh_profile_v4_fixed_replay_summary.py`.

L'audit geometrico esistente
`validation/audit_two_sensor_prescribed_geometry.py` è stato reso neutro
rispetto al profilo selezionato nei titoli dei plot.

## Artefatti

Gate geometrico:

- `validation/two_sensor_geometry_audit_runs/2026-07-21_v4_mesh_profile_design_check/manifest.json`;
- `validation/two_sensor_geometry_audit_runs/2026-07-21_v4_mesh_profile_design_check/event_geometry_metrics.csv`;
- `plot/07_21_2026_two_sensor_geometry_audit_v4_mesh_profile/`.

Gate temporale finale:

- `validation/two_sensor_mesh_profile_v4_fixed_replay_runs/2026-07-21_ab06_50_100_fixed_v4/manifest.json`;
- `validation/two_sensor_mesh_profile_v4_fixed_replay_runs/2026-07-21_ab06_50_100_fixed_v4/profile_comparison_metrics.csv`;
- `plot/07_21_2026_two_sensor_v4_mesh_fixed_replay/v4/`.

Il plot più utile per l'interpretazione è
`03_fixed_replay_summary.png`, che mostra insieme conteggi, F1/IoU, toe-on,
durata toe e gap heel→toe.

## Test e verifiche

Esiti finali:

- 72 test `unittest` combinati dei nuovi moduli e delle regressioni V3/core;
- 11 test diretti della FSM two-sensor;
- totale: **83 controlli PASS**;
- builder V4 riproducibile con `--check`;
- hash invariati di profilo corrente, training config e FSM;
- `py_compile` PASS;
- `ruff check` PASS;
- `git diff --check` PASS;
- ispezione visiva dei plot geometrici e del riepilogo fixed replay;
- hash degli artefatti del manifest finale verificati.

## Decisione e TODO

Stato:

> V4 anterior-flush: PASS geometrico, FAIL detector, non promovibile.

Il prossimo passo corretto non è training e non è ridurre dwell/soglie. È una
V4.1 ancora sperimentale che mantenga heel, FSM e contratto invariati, ma
collochi il secondo sensore sotto una regione **forefoot load-bearing** della
mesh, non all'estremo anteriore. Una regola geometrica semplice candidata è
usare una sezione plantare più posteriore, circa nella regione metatarsale,
con offset dalla suola esplicito e riproducibile.

TODO:

1. definire e congelare la regola geometrica V4.1 forefoot load-bearing senza
   usare il timing per ottimizzarne la posizione;
2. ripetere gate geometrico e fixed replay sul blocco di design;
3. soltanto dopo un PASS, usare un trial prescribed indipendente come holdout;
4. soltanto dopo il PASS prescribed, eseguire shadow e active A/B sul
   checkpoint best;
5. fare training soltanto se l'active A/B dimostra che il detector valido
   riduce realmente la robustezza della policy congelata.
