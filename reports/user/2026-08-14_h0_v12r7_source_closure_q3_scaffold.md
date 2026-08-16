# H0 V12R7: chiusura sorgente recovery e scaffold Q3

## Stato della milestone

La pipeline sorgente V12R7 per il recovery imitativo same-state e lo scaffold
indipendente V12R7-Q3 sono completi e verificati. La milestone non dichiara
ancora il sistema training-ready: protocol freeze, observer collection, fit,
sviluppo fisico, qualification Q3, checkpoint-zero e A/B positivo del
Morphology Corridor restano stage successivi e fail-closed.

## Problema

Il candidato V12R6 superava i gate offline ma falliva nel caso fisico
`deterministic_offset_plus_0p20`. L'analisi ha mostrato che il cedimento
terminale allo step 179 era la conseguenza di una divergenza iniziata nella
prima stance e non di un errore del detector V26 o del solver. Una diversa
miscelazione statica dei candidati precedenti non poteva soddisfare i gate
offline; mancavano etichette H0 sugli stati realmente visitati dal candidato
V12R6.

Era inoltre necessario evitare che un candidato R7 formalmente valido fosse
poi incompatibile con la qualification Q3 o con il futuro warm-start 2x512.

## Soluzione

È stato implementato un unico round observer-DAgger, senza teacher nel path di
azione:

1. il candidato R6 congelato viene eseguito in pure policy sui sei casi
   development;
2. ogni traiettoria chiusa viene verificata e solo successivamente etichettata
   offline dallo stesso teacher H0 sugli stessi stati;
3. un solo fitter actor-only inizializzato da R6 usa tredici strata a massa
   uguale: sei casi base, sei casi observer e il prefisso recovery R4;
4. il risultato è un actor standard `35 -> 512 -> 512 -> 2`, con log-standard
   deviation byte-identica, colonne clock 0/1 bit-zero, nessun critic e nessun
   update PPO;
5. il candidato può essere congelato soltanto dopo i gate offline e viene poi
   verificato su sei rollout pure-policy completi.

Lo scaffold Q3 separato congela già le interfacce richieste per la successiva
qualification indipendente:

- esatto albero candidato a cinque file;
- actor standard 35-feature e hidden `512, 512`;
- sei baseline seguiti da sei candidati su casi held-out;
- detector binario V26 `binary_active` e profilo/hash esatti;
- Morphology Corridor causal-delayed realmente istanziato ma a peso zero;
- identità byte di reward, azioni e osservazioni nel controllo weight-zero;
- gate fisici assoluti e non-inferiorità condition-matched di reserve,
  residuali e segnali SEA.

## Strategia fail-closed

- Il prefisso fisico terminato per penetrazione è utilizzabile soltanto come
  dato observer, mai come evidenza di autonomia.
- Il caso critico `+0.20` deve riprodurre byte-per-byte i 179 step V12R6 sui
  campi runtime condivisi, dimostrando che il recorder non perturba la policy.
- Il teacher è irraggiungibile durante il rollout ed è caricato solo dopo la
  chiusura e la verifica della replay.
- Sono vietati retry, resume, alpha sweep, blending, safety latch, refit
  post-fisica e aggiornamenti critic/PPO.
- Freeze R7 e Q3 sono namespace separati: Q3 non può aprirsi prima di un ledger
  terminale R7 PASS semanticamente verificato.
- I receipt R7 sono stati allineati preventivamente ai verificatori Q3,
  compresi manifest locale schema 1, contatori di sviluppo, binding del
  candidato e `q3_paths_opened: []`.

## File aggiunti

### Recovery V12R7

- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_probe.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_fitter.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7/freeze_h0_v12r7_recovery.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7/run_h0_v12r7_recovery.py`
- relativi test nella stessa cartella `validation/v12r7/`
- `Trajectory Generator/baseline_MLP/validation/v12r7/diagnostics/analyze_v12r6_plus_failure.py`

### Qualification V12R7-Q3

- `Trajectory Generator/baseline_MLP/validation/v12r7q3/h0_v12r7_q3_artifacts.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/h0_v12r7_q3_qualification_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/h0_v12r7_q3_prerequisites.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/h0_v12r7_q3_qualification_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/test_h0_v12r7_q3_source_scaffold.py`

## Test e verifiche

- suite congiunta V12R7 + scaffold Q3: **59/59 PASS**;
- contract, probe e fitter iniziali: **23/23 PASS**;
- test freezer/runner R7 inclusi nella suite finale;
- `py_compile`: PASS;
- Ruff check: PASS;
- Ruff format check sui file conclusi: PASS;
- attestation read-only degli input R6/R5/R4: PASS;
- contract self-check R7: PASS;
- readiness runtime OpenSim/plugin sulla piattaforma corrente: PASS;
- nessun rollout R7, fit canonico, freeze, Q3, checkpoint o training eseguito
  durante questa milestone.

## Prossimi stage vincolanti

- [ ] Pubblicare e verificare protocol freeze ed execution lock R7.
- [ ] Eseguire una sola pipeline R7: sei collect/label, unico fit e sei
      development rollout.
- [ ] In caso di terminal PASS R7, completare, congelare ed eseguire Q3.
- [ ] Creare il checkpoint-zero full RLlib 2x512 con actor imitativo esatto e
      critic/optimizer/counter freschi.
- [ ] Eseguire l'A/B preregistrata del Morphology Corridor con il primo peso
      positivo autorizzato e pubblicare la config training solo dopo PASS.

