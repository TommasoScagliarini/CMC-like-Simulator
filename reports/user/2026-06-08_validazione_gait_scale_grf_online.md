# Validazione gait-scale della GRF online (profilo full-wrench residual v2)

Data: 2026-06-08

## Problema

La validazione del giorno si era fermata su un acceptance gate `FAIL`
(31 PASS / 12 FAIL) per il miglior candidato
`AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`,
con il blocco fisico dichiarato sul rapporto reserve della pelvi
(`pelvis_ty_reserve_p95_ratio = 1.818x`, soglia `1.5x`) su una finestra di soli
500 ms.

Analizzando il gate sono emersi due difetti di misura che rendevano quel
verdetto poco affidabile:

1. **Finestre troppo corte.** Holdout di calibrazione iter1 = `19 ms`, holdout
   residuale v2 = `100 ms`, run attivo = `500 ms`. Nessuna conteneva eventi di
   gait reali: `reference_count = 0` per heel-strike e toe-off, quindi i quattro
   criteri di timing davano `NaN` e fallivano automaticamente.
2. **Sensor-report sbagliato.** L'acceptance del profilo v2 era stata alimentata
   con `online_grf_calibration_full_wrench_active_iter1.json` (finestra 19 ms),
   non con un holdout del profilo v2. Questo gonfiava i fallimenti su forza e
   momento con numeri non pertinenti.

Inoltre la finestra da 500 ms cade quasi interamente in **appoggio bilaterale**:
entrambi i piedi condividono il carico e il deficit di supporto della GRF online
resta piccolo, mascherando il comportamento in single-support e nelle
transizioni.

## Strategia

Eseguire la validazione su una finestra **gait-scale** rappresentativa, con
eventi di contatto reali, e correggere il cablaggio dell'acceptance gate, senza
modificare codice sorgente del simulatore o del plugin (solo riuso degli script
esistenti).

1. Probe da `0.1 s` della run `online` attiva per confermare comando e
   wall-clock (~`120 s` wall per secondo simulato).
2. Trio forward head-to-head su `[11.99, 13.99]` (2 s), `dt = 0.001`, profilo v2:
   - `online` (attivo, `--no-external-loads`): guida la dinamica;
   - `online_sensor`: riferimento per le reserve;
   - `prescribed`: oracolo dell'impulso.
3. Holdout sensor del profilo v2 sulla finestra IK completa
   (`validate_online_grf.py --no-calibrate`, split 60/40 → holdout `[17.4, 21.0]`,
   ~3.6 s con eventi reali; il residuale viene applicato).
4. `validate_online_grf_forward_drift.py` sul trio gait-scale.
5. `online_grf_acceptance.py` con gli input corretti.

## Soluzione e risultati

### Stabilità dinamica: pure-online regge 2 s

La run `online` attiva ha guidato puramente la dinamica per l'intera finestra
gait-scale:

- `status = complete`, nessun guard trip, nessuna caduta;
- max sink `pelvis_ty` = `0.67 mm`, fall fraction = `0`;
- 5 eventi online rilevati;
- wall-clock `230.6 s`.

Su 500 ms non era rappresentativo; su 2 s con transizioni reali la simulazione
resta integrabile e il bacino non affonda.

### Ma il costo in reserve esplode su gait completo

Il bacino resta alto **solo perché le reserve compensano** il supporto mancante:

| Metrica | 500 ms (vecchio) | 2 s gait-scale (nuovo) | Soglia |
|---|---:|---:|---:|
| `pelvis_ty_reserve_p95_ratio` (online/sensor) | 1.818x | **11.12x** | 1.5x |
| reserve `pelvis_ty` p95 | 45.4 N | **475.4 N** | — |
| reserve growth slope | 25 N/s | **176.7 N/s** | 100 N/s |
| impulso verticale totale | ~1.02 | **0.827** | [0.9, 1.1] |
| impulso destro | 0.953 | **0.786** | [0.8, 1.2] |
| impulso sinistro | 1.077 | 0.868 | [0.8, 1.2] |

Il deficit di supporto, invisibile in doppio appoggio, diventa dominante in
single-support: l'online sotto-supporta (impulso totale `0.83`, destro `0.79`) e
le reserve della pelvi salgono a `11x` il sensor.

### Acceptance gate corretto: verdetto onesto

Con plugin-report v2 (parità `0 N`), sensor-report v2 gait-scale e forward-report
gait-scale, il gate dà `FAIL` **29 PASS / 14 FAIL**, ma i fallimenti sono ora
fisicamente significativi e non più artefatti:

| Tema | Criteri falliti | Valori | Soglia |
|---|---|---|---|
| Reserve (core) | `active_vs_sensor.pelvis_ty_reserve_p95_ratio`; `pelvis_ty_reserve_abs_growth_slope` | 11.12x; 176.7 N/s | 1.5x; 100 |
| Sotto-supporto | `active.total_impulse_ratio`; `active.right.impulse_ratio` | 0.827; 0.786 | [0.9,1.1]; [0.8,1.2] |
| Penetrazione | sensor L/R; active L/R | 19.7/40.1 mm; 17.4/25.4 mm | 15 mm |
| COP orizzontale | sensor L/R; active L | 78/141 mm; 93 mm | 60; 80 mm |
| Tangenziale | force corr min L/R | 0.107; 0.104 | 0.3 |
| Timing | right heel-strike | 127 ms | 120 ms |

Criteri ora **superati** che prima erano falsi FAIL: tutti gli eventi
heel-strike/toe-off (non più NaN), force NRMSE verticale (0.20/0.36), correlazione
verticale (0.92/0.91), contact F1 (0.92/0.87), momento NRMSE e correlazione,
sink, status, durata.

## Cause ordinate per impatto

1. **Calibrazione su finestra non rappresentativa.** Il profilo v2 è tarato su
   500 ms di appoggio bilaterale: generalizza male a single-support/transizioni,
   dove sotto-supporta e fa esplodere le reserve (`11x`).
2. **Geometria del patch denso che sovra-penetra.** Fuori dalla finestra di
   calibrazione il patch v2 penetra fino a `40 mm` (destra) in replay IK.
3. **COP errato**, soprattutto a destra (`141 mm`): la base di contatto non
   riproduce il trasferimento del carico durante l'appoggio.
4. **Tangenziale ancora debole** (correlazione `~0.10`): la legge di attrito
   memoryless non ricostruisce la forma temporale delle componenti orizzontali.

## File creati/prodotti (nessuna modifica al codice sorgente)

- `results/online_grf_gait2s_online/` — run attivo pure-online 2 s.
- `results/online_grf_gait2s_sensor/` — run `online_sensor` 2 s (stati forward
  rappresentativi, utili per la prossima ricalibrazione).
- `results/online_grf_gait2s_prescribed/` — run prescribed 2 s (oracolo).
- `results/online_grf_validation_gait_v2.json` — holdout sensor v2 gait-scale.
- `results/online_grf_forward_drift_gait2s_v2.json` (+ `.png`) — drift/reserve/
  impulso head-to-head.
- `results/online_grf_acceptance_gait2s_v2.json` — verdetto FAIL 29/14 con input
  corretti.
- `results/_probe_online_v2_100ms/` — probe di velocità (eliminabile).

## Test e verifiche

- probe 0.1 s online: `status = complete`;
- trio forward 2 s: tutti `status = complete`, finestra intera, nessun guard trip;
- holdout sensor v2 gait-scale: eventi reali (HS/TO non più NaN);
- forward-drift gait-scale: prodotto JSON + PNG;
- acceptance corretto: `FAIL` 29/14, parità plugin `0 N`;
- nessuna simulazione interrotta; nessuna modifica a codice o plugin.

## Iterazione: ricalibrazione su stati gait-scale

Sono stati tentati due rimedi pre-approvati, ricalibrando sugli stati forward
gait-scale appena prodotti.

### 1. Ricalibrazione del basis con penetrazione vincolata (scartata)

`calibrate_online_grf_basis.py --states-sto <gait2s_sensor> --maximum-penetration
0.012`. La penetrazione è rientrata (L `9.7 mm`, R `11.7 mm`), ma l'accuratezza
verticale è **crollata** (holdout NRMSE-peak verticale `4.3` / `6.1`, F1 `0.60`):
calibrare la *legge di contatto* su 2 s di stati forward con vincolo stretto
peggiora il fit rispetto al basis IK-replay 07/06. Profilo scartato.

### 2. Residuale state-based ri-fittato su stati ATTIVI gait-scale

`calibrate_online_grf_residual.py --profile <v2> --states-sto <gait2s_online>`.
Il fit trova un boost verticale costante (sinistra `+8.9%`, **destra `+27.5%`**,
che mira al sotto-supporto destro). Rivalidato forward su 2 s:

| Metrica | v2 (residuale 500 ms) | Residuale gait-scale | Gate |
|---|---:|---:|---:|
| Impulso verticale totale | 0.827 | **0.943** | [0.9, 1.1] |
| Impulso destro | 0.786 | **0.924** | [0.8, 1.2] |
| Impulso sinistro | 0.868 | 0.961 | [0.8, 1.2] |
| reserve `pelvis_ty` p95 | 475 N | 438 N | — |
| **reserve ratio (online/sensor)** | 11.12x | **10.25x** | 1.5x |
| reserve growth slope | 176.7 N/s | 70.4 N/s | 100 |
| max sink / caduta | 0.6 mm / 0 | 0.6 mm / 0 | ok |

## Diagnosi decisiva: il limite è il timing del wrench, non la magnitudo

Portando l'impulso verticale da `0.83` a `0.94` (magnitudo praticamente corretta),
le reserve sono scese **solo** da `11.1x` a `10.3x`. Questo dimostra
empiricamente, su dati forward rappresentativi, la causa radice già ipotizzata
nell'analisi 08/06:

- le reserve della pelvi seguono il **deficit istantaneo** di forza (corr `0.999`);
- un residuale a rapporto costante × forza normale **scala** il segnale ma non ne
  cambia la **forma temporale**: le correlazioni destre restano negative;
- correggere l'area sotto la curva (impulso) **non** elimina il mismatch di
  timing/distribuzione, quindi le reserve restano `~10x`.

La legge di contatto memoryless state-based, anche con residuale full-wrench
ri-fittato su stati gait-scale, **non può** riprodurre il profilo temporale del
wrench prescribed in dinamica forward. Il tuning dei parametri è quindi
**saturato** intorno a `~10x`: ulteriori residuali costanti o più sfere non
chiuderanno il gate `1.5x`.

## Conclusione

La modalità `online` **non è promovibile a validated** e il blocco è strutturale,
non di calibrazione: con il modello di contatto attuale (basis sparso +
residuale state-based) il rapporto reserve resta `~10x` sull'intero ciclo di
gait, indipendentemente dalla correzione di magnitudo. Il rollout pure-online è
comunque **integrabile e stabile** (nessuna divergenza/caduta): il limite è la
fedeltà fisica temporale del wrench, non la stabilità numerica.

Le opzioni che restano non sono tuning ma scelte di modello/architettura
(modello di contatto fisico completo vs. ridefinizione del gate per lo scopo del
generatore vs. uso di `online_sensor` per la fase imitativa). Vedi TODO.

## TODO

- [ ] Ricalibrare il basis full-wrench sugli stati forward **gait-scale** appena
  prodotti (`results/online_grf_gait2s_sensor/sim_output_states.sto`), invece dei
  500 ms iniziali in doppio appoggio.
- [ ] Vincolare la penetrazione massima nella calibrazione (target ≤ 15 mm) e
  rivedere la geometria/offset del patch denso che sovra-penetra a destra.
- [ ] Includere COP e momento nella funzione obiettivo in modo accoppiato sul
  wrench completo (oggi COP non è target).
- [ ] Valutare una legge tangenziale **stateful** (stiction/deflessione) invece
  di patch memoryless, per alzare la correlazione tangenziale sopra 0.3.
- [ ] Dopo la ricalibrazione, ripetere trio forward 2 s + forward-drift +
  acceptance; accettare solo il profilo più semplice che porta
  `pelvis_ty_reserve_p95_ratio <= 1.5` senza regredire eventi/accuratezza.
- [ ] Considerare un guard inferiore di supporto a runtime (oggi esiste solo il
  guard superiore `online_grf_max_force_bw`): un deficit affonda in silenzio.
- [ ] Non promuovere alcun profilo a `validated` finché
  `active_vs_sensor.pelvis_ty_reserve_p95_ratio <= 1.5` su finestra gait-scale.
- [ ] Ricompilare e riconfermare il plugin su macOS arm64.
