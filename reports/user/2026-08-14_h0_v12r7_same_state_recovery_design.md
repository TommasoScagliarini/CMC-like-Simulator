# H0 V12R7 — design recovery same-state dopo il terminale V12R6

Data: 2026-08-14

## Esito della milestone

Il fallimento V12R6 è stato diagnosticato e il successore V12R7 è stato
definito come nuova lineage additiva. V12R6 resta terminale e immutabile.

Non verrà eseguito un altro tentativo modificando soltanto il blend P2/R5. Il
protocollo R7 prevede:

1. un solo round observer-only sui sei casi development congelati;
2. azioni servite esclusivamente dal candidato R6 puro;
3. label H0 coerenti interrogate offline sugli stessi stati visitati;
4. un unico fit actor-only `35 -> 512 -> 512 -> 2`;
5. freeze del candidato prima di qualunque development R7;
6. sei rollout puri hardest-first, senza teacher, blend o safety latch.

Il detector V26, il limite di penetrazione di 25 mm, la GRF primaria e la
semantica SEA restano invariati. Q3, checkpoint-zero e reward morphology
positiva rimangono chiusi fino al PASS fisico del candidato R7.

## Problema

R6 ha fallito il primo caso `deterministic_offset_plus_0p20` al passo 179:

- terminazione `grf_penetration`;
- penetrazione `0.0257906750 m`;
- zero clipping, fallback SEA, SO non accettata o non-finiti;
- detector V26 e FSM integri;
- nessun teacher, latch, fit o update durante il rollout.

Il terminale non è la causa primaria. La deriva nasce nel primo stance:

- la distanza dall'azione safe supera `0.05` già al passo 41;
- al passo 60 R6 serve circa `[0.0546, 0.1072]`, contro
  `[0.8082, 0.4169]` della reference safe;
- la reserve sale a `760.75 Nm`, contro `217.21 Nm`;
- il contatto viene scaricato e il TO R6 è accettato a
  `14.808871 s`, circa `0.418 s` prima della reference safe;
- la seconda stance parte quindi fuori distribuzione e raggiunge il limite
  fisico al passo 179.

Non si tratta di un errore del solver o del detector. È covariate shift
closed-loop: piccoli errori imitativi iniziali portano l'actor in stati non
coperti dal corpus.

## Esclusione del rescue tramite alpha

Sulle 212 label same-state del prefisso R4 fallito, nessun blend convesso
P2/R5 raggiunge il gate RMSE `<= 0.006`:

| Variante | RMSE recovery R4 |
| --- | ---: |
| P2, alpha R5 `0.000` | `0.0112564` |
| R6, alpha R5 `0.300` | `0.0105853` |
| migliore griglia `[0,1]`, alpha `0.474` | `0.0104768` |
| R5, alpha R5 `1.000` | `0.0114278` |

Il passaggio proposto in precedenza da alpha `0.300` a `0.268` cambia
l'azione R6 troppo poco: sul tratto tardo lo shift medio della caviglia è
circa `0.00114`, con massimo inferiore a `0.004`. Al passo 150 R6 resta a
circa `-0.1095`, mentre la reference safe è positiva, circa `+0.1177`.

La proiezione della reference safe sul segmento P2-R5 cade inoltre fuori
`[0,1]` nella grande maggioranza degli stati R6. Un alpha statico o schedulato
non dispone quindi della direzione di recovery necessaria.

## Strategia R7 congelabile

### Observer collection

Ordine unico:

1. `deterministic_offset_plus_0p20`;
2. `deterministic_offset_minus_0p20`;
3. `deterministic_offset_nominal`;
4. `stochastic_nominal_seed_126`;
5. `stochastic_nominal_seed_127`;
6. `stochastic_nominal_seed_128`.

La policy R6 resta pura e teacher-free. Il recorder conserva osservazioni,
input fisici, eventi legacy e boundary terminale. Un prefisso terminato
fisicamente può essere accettato come dato di collection, ma non come PASS
fisico. Solo dopo la chiusura e l'hash del replay viene ricostruito offline il
`LegacyGaitShadow` e H0 viene interrogato una volta per ogni stato.

Per il caso `+0.20` il nuovo recorder deve inoltre dimostrare di non perturbare
il comportamento, riproducendo il prefisso R6 esistente di 179 step entro le
tolleranze congelate.

### Corpus e fit

Sorgenti:

- corpus R5: 9.232 righe;
- prefisso R4 `+0.20`: 212 label same-state, finora escluse;
- sei nuovi corpus observer R6.

Il loss usa 13 strati a massa uguale:

- sei casi base;
- sei casi observer;
- un auxiliary stratum R4 failure.

Non viene usato un peso basato soltanto sulla penetrazione, perché la deriva è
visibile molto prima che la penetrazione cresca. Il fit parte dall'actor R6
512, aggiorna una sola volta l'intera mean network, congela `logstd`, mantiene
le colonne clock 0/1 bit-zero e non contiene critic o PPO.

Gate offline:

- globale, per caso base, R4 failure e ogni caso observer:
  RMSE `<= 0.006`, massimo `<= 0.060`;
- reset massimo `<= 0.003`;
- gate separato sul tratto observer `+0.20` dal passo 140 al terminale;
- finitezza, save/reload, `logstd` e clock esatti.

### Development

Il candidato viene congelato prima dei rollout. Ogni caso richiede:

- 500 step e time limit naturale;
- almeno due cicli validi;
- penetrazione strettamente inferiore a `0.025 m`;
- detector V26 ordinato, senza fallback, duplicati o sorgenti non V26;
- zero clipping, non-finiti, fallback SEA e SO non accettata;
- teacher, blending e latch disabilitati.

Non sono autorizzati retry, alpha sweep o refit post-development nella stessa
lineage.

## File introdotti alla milestone

- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_contract.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r7/test_h0_v12r7_recovery_contract.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r7/diagnostics/analyze_v12r6_plus_failure.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r7/__init__.py`.

Collector/labeler, fitter, freezer, runner, Q3 e checkpoint-zero sono in corso
e verranno documentati nelle milestone successive.

## Test e verifiche

- contract R7: 5/5 test PASS;
- Ruff contract/test: PASS;
- diagnostica R6 completa: PASS;
- Ruff e format diagnostica: PASS;
- compilazione diagnostica: PASS;
- `git diff --check`: PASS al termine dell'audit indipendente;
- nessun reset/step ambiente o fit eseguito dalla diagnostica;
- nessun file V12R6 modificato.

## TODO

- [ ] completare e testare collector/labeler observer R7;
- [ ] completare e testare fitter 512 a 13 strati;
- [ ] congelare protocollo ed execution lock R7;
- [ ] eseguire una sola volta collection, labeling, fit e development R7;
- [ ] se R7 passa, eseguire Q3 V26/corridor shadow;
- [ ] creare il checkpoint-zero actor-only con critic 512 fresco;
- [ ] validare identity A/B del corridor a peso zero e pubblicare il comando
  training definitivo.
