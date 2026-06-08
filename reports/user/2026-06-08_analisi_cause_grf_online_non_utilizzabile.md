# Analisi delle cause che rendono la GRF online non ancora utilizzabile

Data: 2026-06-08

## Obiettivo

Capire perche la GRF online, pur avendo superato verifiche di equivalenza
implementativa e pur riuscendo a riprodurre un impulso verticale plausibile,
non e ancora adatta al training e all'inferenza active.

L'analisi confronta, sulla stessa finestra temporale:

- GRF prescribed;
- GRF `online_sensor`, che osserva la cinematica senza sostituire la GRF;
- GRF `online_candidate`, che viene applicata al modello;
- reserve, coppie biologiche, COP, momenti e stato dei contatti.

Non sono state lanciate simulazioni lunghe. Sono stati analizzati i risultati
gia salvati.

## Risultato principale

Il problema dominante non e la sola quantita media di forza verticale.
La GRF online applica al modello un **wrench esterno temporalmente e
spazialmente diverso** da quello prescribed: sono errati il profilo temporale
delle forze, le componenti tangenziali, il COP e i momenti.

Le reserve del bacino sostituiscono direttamente questo wrench mancante o
errato. La correlazione tra incremento della reserve active rispetto al sensor
e deficit di forza `prescribed - online` e:

| Coordinata root | Correlazione |
|---|---:|
| `pelvis_tx` | 0.9998 |
| `pelvis_ty` | 0.9999 |
| `pelvis_tz` | 0.9445 |

Quindi l'aumento delle reserve non indica principalmente un malfunzionamento
generico del controllore o della static optimization: e la risposta dinamica
attesa a una GRF online non equivalente alla GRF prescribed.

## Evidenze

### 1. Impulso verticale plausibile, ma profilo temporale errato

Nel run active bilanciato da 500 ms:

- rapporto impulso verticale totale active/prescribed: `0.953`;
- rapporto sinistro: `0.869`;
- rapporto destro: `1.053`;
- reserve `pelvis_ty` p95 active: `148.4 N`;
- reserve `pelvis_ty` p95 sensor: `25.0 N`;
- rapporto active/sensor: `5.94x`.

Un profilo successivamente riscalato raggiunge un impulso totale di `0.983`,
ma mantiene un rapporto reserve di `5.81x`. Correggere l'impulso medio non
risolve quindi il problema.

Sulla forza netta active rispetto alla prescribed:

| Asse | Errore medio [N] | Errore assoluto p95 [N] | Correlazione temporale |
|---|---:|---:|---:|
| x | +12.37 | 57.82 | -0.429 |
| y | -35.39 | 152.74 | -0.135 |
| z | -4.78 | 10.25 | 0.734 |

La correlazione verticale negativa spiega perche una buona area sotto la curva
non produce una dinamica equivalente: la forza arriva nei momenti sbagliati.

### 2. Componenti tangenziali assenti o incompatibili

Nel profilo bilanciato:

- quasi tutti i coefficienti di attrito destri sono praticamente nulli;
- la GRF online destra e quindi esattamente `[0, Fy, 0]`;
- la GRF prescribed destra ha invece componenti tangenziali medie non nulle,
  circa `[-13.37, 350.24, +2.07] N`;
- la componente x sinistra esiste, ma ha correlazione negativa con la
  prescribed.

La calibrazione corrente stima prima la sola componente verticale e poi adatta
separatamente `Fx` e `Fz`. L'ottimizzazione non e accoppiata sul wrench
completo e puo far collassare l'attrito a zero.

### 3. COP e distribuzione spaziale errati

Errore medio del COP active rispetto al prescribed:

- sinistra: circa `-55 mm` lungo x;
- destra: circa `+130 mm` lungo z.

Errore RMS COP:

| Lato | x [mm] | y [mm] | z [mm] |
|---|---:|---:|---:|
| sinistro | 55.4 | 17.7 | 8.8 |
| destro | 10.0 | 17.7 | 130.0 |

Il patch calibrato usa solo due contatti a sinistra e tre a destra, selezionati
per riprodurre `Fy`. Questa base e troppo povera e non viene ottimizzata per
riprodurre il trasferimento del COP durante l'appoggio.

### 4. Momenti esterni fortemente errati

Il momento rispetto all'origine del ground presenta errori p95 fino a:

- sinistra: circa `[172, 17, 99] Nm`;
- destra: circa `[154, 44, 91] Nm`.

Le correlazioni dei momenti sinistri sono tutte negative. Sul lato destro il
momento online attorno a y e nullo, mentre il prescribed ha una media di circa
`19 Nm`.

Il plugin applica una forza puntuale e produce soltanto il momento
`contactPoint x force`; non dispone di una coppia libera indipendente. Inoltre
la calibrazione corrente non include mai COP o momenti nella funzione
obiettivo.

### 5. Calibrazione open-loop fragile in active

Il profilo e calibrato su `ik_replay`, ma viene poi usato in una simulazione
forward active. Piccole differenze dello stato distale modificano la
penetrazione e quindi la forza di contatto:

- sinistra: variazione penetrazione p95 `2.10 mm`, variazione forza normale
  p95 `168.3 N`;
- destra: variazione penetrazione p95 `0.76 mm`, variazione forza normale
  p95 `87.0 N`.

Le coordinate del bacino restano molto vicine grazie alle reserve, mentre
caviglia, MTP e relative velocita cambiano maggiormente. Si crea quindi un
feedback: la GRF modifica il moto distale, il moto modifica il contatto e il
contatto modifica nuovamente la GRF. Il fit open-loop non rappresenta questo
effetto.

### 6. Validazione attuale insufficiente per il gait completo

La finestra active da 500 ms resta sostanzialmente in appoggio bilaterale e non
valida bene le transizioni. Nell'holdout offline:

- errore toe-off medio sinistro: `0.154 s`;
- errore toe-off medio destro: `0.600 s`;
- errore toe-off massimo destro: `1.341 s`;
- NRMSE verticale: circa `0.46` a sinistra e `0.42` a destra.

Il contact F1 e l'impulso non bastano per dichiarare il modello pronto per
training e inferenza: servono almeno un ciclo completo e soglie esplicite su
timing degli eventi, wrench e COP.

## Cause ordinate per impatto

1. **Mismatch del wrench esterno completo**: forze temporalmente errate, COP e
   momenti incompatibili con il prescribed.
2. **Funzione obiettivo di calibrazione incompleta**: fit verticale, tangente e
   riscalamento dell'impulso sono separati; COP e momenti non sono target.
3. **Forze tangenziali insufficienti**, soprattutto a destra, dove l'attrito
   calibrato collassa quasi a zero.
4. **Base di contatto troppo sparsa** per rappresentare il trasferimento del
   carico e del COP.
5. **Sensibilita closed-loop non modellata**: il profilo ottenuto su IK replay
   non e robusto alle variazioni dello stato active.
6. **Gate di validazione troppo debole**, perche una breve finestra in doppio
   appoggio non verifica correttamente heel-strike, toe-off e swing.

## Cause escluse come spiegazione primaria

- **Quantita totale di forza verticale**: anche con impulso quasi corretto le
  reserve restano circa `5.8x` il sensor.
- **Caduta o deriva del bacino**: nella finestra active il sink massimo e solo
  circa `0.073 mm`, perche le reserve compensano il mismatch.
- **Errore numerico Python/C++ del sensore online**: l'equivalenza
  implementativa gia validata dimostra che il sensore calcola coerentemente il
  modello configurato, ma non che quel modello riproduca il wrench fisico
  necessario.
- **Problema generico della static optimization**: le reserve root seguono
  quasi perfettamente il deficit di forza esterna.

## Strategia raccomandata

1. Calibrare e validare il **wrench 6D per piede**, usando `Fx/Fy/Fz` e momento
   rispetto a un'origine comune, oppure forza, COP e coppia libera.
2. Ottimizzare congiuntamente posizione, raggio e materiali dei contatti,
   evitando il fit separato verticale/tangenziale.
3. Usare finestre che coprano almeno un ciclo completo e includano heel-strike,
   midstance e toe-off.
4. Calibrare prima sugli stati forward `online_sensor`, poi iterare con rollout
   active brevi e watchdog.
5. Aggiungere al gate active soglie su:
   - correlazione e p95 dell'errore per ogni componente di forza;
   - COP;
   - momento;
   - timing degli eventi;
   - reserve root assolute e rapporto active/sensor.
6. Solo se il patch di contatto non riesce a ricostruire il momento libero,
   valutare una sua rappresentazione esplicita nel modello di contatto.

## File coinvolti nell'analisi

- `validation/analyze_online_grf_active_failure.py`: nuovo audit read-only per
  confrontare run active/sensor e GRF prescribed.
- `validation/calibrate_online_grf_basis.py`: calibrazione attuale basata prima
  su `Fy` e poi sulle componenti tangenziali.
- `validation/tune_online_grf_support.py`: tuning basato sull'impulso verticale
  per lato.
- `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp`: applicazione
  della forza puntuale e calcolo del momento `point x force`.
- `config.py`: le coordinate `pelvis_*` sono gestite come DOF root non attuati
  e sostenuti dalle reserve.
- `results/online_grf_active_failure_analysis_balanced_500ms.json`: risultato
  strutturato dell'audit.

## Verifiche eseguite

- confronto quantitativo active/sensor/prescribed sulla stessa finestra;
- correlazione tra deficit di forza e incremento delle reserve root;
- confronto per asse delle forze;
- confronto COP e momenti per lato;
- confronto penetrazione, forza normale e slip active/sensor;
- ranking delle differenze di `tau_bio` e degli stati articolari;
- compilazione sintattica dello script diagnostico;
- nessuna simulazione lunga avviata.

## TODO

- [ ] Estendere calibrazione e acceptance gate al wrench completo per piede.
- [ ] Creare un test discriminante che separi errore della forza da errore di
  COP/momento.
- [ ] Ricalibrare sugli stati `online_sensor` forward.
- [ ] Validare con rollout active progressivi e watchdog su almeno un ciclo di
  gait completo.
- [ ] Verificare se il patch di contatto puo generare il momento libero
  necessario prima di modificare il plugin C++.
