# Audit geometrico V4 del detector virtuale heel/toe

Data: 2026-07-21

## Esito esecutivo

L'audit geometrico e temporale è stato completato senza modificare FSM,
detector runtime, checkpoint, policy, reward o training.

Il risultato è netto:

> La geometria corrente dei due sensori non è sufficientemente plausibile per
> usare i casi `forefoot_first` come prova che il soggetto abbia realmente
> appoggiato prima l'avampiede. In particolare, il sensore toe è staccato e
> molto più basso della mesh del piede. Prima di cambiare la semantica della
> FSM occorre creare e verificare una geometria sensoriale V4 sperimentale.

Il blocco `50–100 s`, già aperto dalla V3, è stato usato esclusivamente come
failure-analysis. Il blocco sealed `100–155.045 s` non è stato aperto.

## Che cosa rappresenta realmente il riferimento “HeelStrike”

L'oracolo temporale usato dalla validazione prescribed non è un sensore
localizzato sul tallone. È l'inizio del contatto del piede ricavato dalla GRF
verticale sinistra totale:

```text
prescribed left Fy attraversa 20 N -> reference contact start
```

Quindi, dal punto di vista fisico, il timestamp reference significa
`initial_contact` o inizio stance. Il nome storico `heel_strike` non dimostra
che sia stato caricato specificamente il tallone.

Questo implica due cose diverse:

1. un vero contatto forefoot-first potrebbe legittimamente coincidere con il
   reference total-GRF;
2. il reference total-GRF, da solo, non può validare formalmente un detector
   anatomico heel-only.

Nel caso corrente, tuttavia, non è corretto concludere che i cinque eventi
critici siano reali forefoot strike: la geometria toe è incoerente e, in tutti
i cinque casi, il COP prescribed all'istante del contatto è molto più vicino
alla regione heel che alla posizione toe corrente.

## Origine delle posizioni correnti

Le due sfere del profilo detector hanno raggio comune `22.905 mm` e sono
definite sul frame `/bodyset/foot_l`.

| Sensore | Posizione locale [m] | Marker coincidente |
|---|---|---|
| heel | `[-0.101410, -0.032820, +0.013996]` | `L_Heel` |
| toe | `[+0.069139, -0.074012, -0.046862]` | `L_Toe_Lat` |

Il punto toe non era stato scelto come punto plantare del piede. Corrisponde al
marker `L_Toe_Lat`, selezionato dall'euristica storica sui nomi dei marker.

Il profilo stesso è marcato `preliminary` e deriva da una calibrazione breve
rispetto alla GRF prescribed. Non era quindi una geometria sensoriale fisica
definitiva.

## Confronto con la mesh del piede

La mesh `Geometry/AM_foot_l.STL` ha bounds locali:

| Asse | Min [mm] | Max [mm] |
|---|---:|---:|
| x | -110.97 | +176.02 |
| y | -45.22 | +16.00 |
| z | -40.55 | +47.96 |

Risultati delle distanze punto-triangolo esatte:

| Metrica | Heel | Toe |
|---|---:|---:|
| Distanza centro–superficie mesh | 4.79 mm | 41.17 mm |
| Gap sfera–mesh | 0.00 mm | 18.26 mm |
| Fondo sfera sotto il minimo y globale della mesh | 10.50 mm | 51.70 mm |

Il fondo della sfera toe è inoltre `41.19 mm` più basso del fondo della sfera
heel. Il centro toe è fuori anche dai bounds assiali della mesh.

Pertanto:

- la sfera heel interseca la regione della mesh e può essere considerata
  almeno geometricamente connessa al piede;
- la sfera toe è separata dalla mesh di `18.26 mm` anche dopo avere considerato
  il suo raggio;
- il toe virtuale può penetrare il piano molto prima della suola reale della
  mesh e molto prima del tallone.

## Audit dei cinque HS critici

L'audit ha ricostruito a `1 ms` i cinque eventi indipendenti che causavano i
quattro ritardi V3 e la successiva perdita dell'HS. La clearance è firmata:
valore negativo significa penetrazione del piano.

| HS [s] | Heel clr. [mm] | Toe clr. [mm] | COP heel→toe | Heel onset [ms] | Primo impulso heel [ms] | Causa |
|---:|---:|---:|---:|---:|---:|---|
| 83.712891 | -0.11 | -4.66 | 0.253 | +6.11 | 26 | `SHORT_HEEL_PULSE` |
| 86.461720 | -0.80 | -4.25 | 0.278 | -5.72 | 34 | `DEBOUNCE_SAMPLING_10MS` |
| 89.150896 | -1.00 | -1.74 | 0.265 | -8.90 | 36 | `DEBOUNCE_SAMPLING_10MS` |
| 92.727765 | -0.74 | -5.41 | 0.276 | -7.77 | 28 | `SHORT_HEEL_PULSE` |
| 98.179601 | +1.55 | -8.98 | 0.341 | +129.40 | 27 | `GEOMETRIC_HEEL_DELAY` |

La coordinata COP è proiettata sulla linea heel→toe corrente:

- `0` corrisponde alla posizione heel;
- `1` corrisponde alla posizione toe.

Tutti i cinque COP al contatto sono più vicini alla posizione heel. Nei cinque
eventi la parte più bassa della mesh resta inoltre `9.14–11.94 mm` sopra il
piano, mentre la sfera toe risulta già penetrata fino a `8.98 mm`.

La velocità normale toe al contatto è compresa circa fra `-0.64` e
`-0.72 m/s`, mentre la heel scende molto più lentamente. È quindi la grande
leva geometrica introdotta dal punto toe fuori mesh a creare carichi toe
precoci e molto elevati.

### Scomposizione del problema

I cinque eventi non hanno una causa unica:

- due impulsi heel durano solo `26` e `28 ms`: sono fisicamente più brevi del
  dwell da `30 ms`;
- due impulsi durano `34` e `36 ms`: sarebbero confermabili sul segnale a
  `1 ms`, ma non sulla griglia runtime `10 ms`, che non osserva quattro
  campioni consecutivi sufficienti a coprire `30 ms`;
- nell'evento `98.179601 s` il tallone è ancora `1.55 mm` sopra il piano e
  supera `0.5 N` soltanto `129.4 ms` dopo il reference. Qui ridurre il dwell
  non può risolvere il problema.

Questo conferma che un semplice nuovo sweep delle soglie sarebbe una
scorciatoia errata.

## Perché la precedente validazione poteva sembrare corretta

Il vecchio detector ricavava l'inizio contatto dal carico virtuale sinistro
aggregato. In altri termini, heel e toe contribuivano allo stesso segnale e il
crossing veniva chiamato storicamente `heel_strike`.

Con il toe virtuale molto più basso della mesh, il toe poteva portare
l'aggregato sopra soglia con una buona temporizzazione rispetto alla GRF totale,
anche senza un heel locale stabile. La nuova FSM a due sensori ha invece reso
visibile la distinzione:

- heel stabile richiesto per l'evento heel-only;
- toe anticipato registrato come diagnostica `forefoot_first`;
- TO legato all'uscita dal contatto dei sensori.

La lunga validazione precedente non era quindi “falsa”: validava correttamente
un detector di contatto aggregato. Non validava però un sensore anatomico heel
separato dal toe.

## Decisione di progetto

Per ora non viene cambiata la semantica della FSM.

In particolare non viene promosso il toe a generatore di un evento
`initial_contact`, perché la sua geometria corrente rende il risultato
confondente. La semantica heel-only resta l'ipotesi operativa più coerente con
l'obiettivo hardware dichiarato: un sensore al tallone e uno in punta, usati
soltanto come detector.

La V4 corretta deve procedere così:

1. creare un profilo detector sperimentale separato;
2. collocare heel e forefoot su punti plantari ricavati dalla mesh, con offset
   rispetto alla suola comparabili;
3. non sovrascrivere il profilo corrente e non cambiare il default
   `legacy_events`;
4. nella prima prova cambiare soltanto la geometria, mantenendo invariati
   soglie `0.5/0.25 N`, dwell `30 ms` e FSM;
5. ripetere l'audit sui dati già aperti come design check, non come nuova
   validazione;
6. congelare il metodo prima di una nuova validazione su dati non usati per il
   redesign;
7. mantenere chiuso il blocco `100–155.045 s` fino a esplicita autorizzazione;
8. dopo il PASS prescribed, fare prima shadow rollout e poi rollout A/B attivo
   sul checkpoint best congelato;
9. fare training soltanto se il cambio runtime riduce effettivamente la
   robustezza del checkpoint.

Solo dopo avere validato la nuova geometria sarà sensato decidere in modo
definitivo fra:

- `heel_strike` anatomico, confermato dal solo heel;
- `initial_contact`, confermato dal primo sensore stabile e accompagnato dalla
  diagnostica `heel_first/forefoot_first`.

Va ricordato che EPIC/GRF totale può validare direttamente solo la seconda
semantica. Per una validazione formale heel-only servirà un riferimento
localizzato sul tallone o una ground truth virtuale regionale indipendente.

## Implementazione dell'audit

File aggiunti:

- `validation/audit_two_sensor_prescribed_geometry.py`;
- `validation/test_two_sensor_prescribed_geometry_audit.py`.

Artefatti numerici:

- `validation/two_sensor_geometry_audit_runs/2026-07-21_v4_design_audit/manifest.json`;
- `validation/two_sensor_geometry_audit_runs/2026-07-21_v4_design_audit/event_geometry_metrics.csv`.

Plot:

- `plot/07_21_2026_two_sensor_geometry_audit_v4/01_static_sensor_mesh_geometry.png`;
- `plot/07_21_2026_two_sensor_geometry_audit_v4/02_all_hs_initial_contact_geometry.png`;
- `plot/07_21_2026_two_sensor_geometry_audit_v4/03_critical_hs_contact_windows.png`.

Il manifest registra hash e provenance di setup, modello, mesh, profilo e
manifest V3.

## Test e verifiche

Eseguiti con esito positivo:

- 8 test unitari dell'audit;
- 21 test di regressione dello sweep prescribed V3;
- 7 test di regressione del gate di fase;
- 11 test diretti della FSM two-sensor;
- parsing STL binario e distanza punto–triangolo;
- rifiuto hard di qualunque finestra oltre `100 s`;
- test sintetici di impulsi da `29/30/31 ms`;
- distinzione fra dwell ideale e confermabilità a `10 ms`;
- classificazione heel-first/toe-first/simultaneo;
- invarianza della proiezione COP a traslazione e yaw;
- verifica source-level della sfera toe staccata;
- `ruff check`;
- `py_compile`;
- `git diff --check`;
- replay geometrico OpenSim `50–100 s` a `1 ms`;
- ispezione visiva dei tre plot prodotti.

I warning OpenSim sulle mesh non trovate dal visualizer non hanno invalidato
l'audit: lo script risolve e legge direttamente `Geometry/AM_foot_l.STL`, il
cui hash è incluso nel manifest.

## Stato e TODO

Stato corretto:

> Audit V4 completato. La geometria corrente, soprattutto il toe, deve essere
> corretta prima di cambiare semantica o ottimizzare soglie/dwell. Nessun
> training è richiesto in questa fase.

TODO ancora aperti:

1. costruire il profilo detector V4 sperimentale mesh-based senza modificare il
   profilo corrente;
2. rieseguire lo stesso audit mantenendo invariati soglie, dwell e FSM;
3. definire e congelare il successivo protocollo di validazione;
4. validare separatamente la robustezza del checkpoint con shadow e active A/B;
5. aprire il sealed set soltanto dopo nuova decisione esplicita.
