# Daily Report - 2026-08-04

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

1. [Detector V19 — geometria binaria force-free](../user/2026-08-04_detector_v19_geometria_binaria.md)
2. [Detector V20 — FSM binaria HS/TO e gate DEV02/04](../user/2026-08-04_detector_v20_fsm_binaria.md)
3. [Detector V21 — sweep geometrico binario e finalista development](../user/2026-08-04_sweep_geometria_detector_binario_v21.md)
4. [Detector V21 — trial 08 one-shot consumato con errore di contratto oracle](../user/2026-08-04_trial08_v21_one_shot_error_terminale.md)
5. [Trial 08 V22 development — errore booleano prima del replay](../user/2026-08-04_trial08_v22_development_error_booleano.md)
6. [Trial 08 V23 — FAIL per gap geometrico di 12 ms](../user/2026-08-04_trial08_v23_fail_gap_geometrico_12ms.md)
7. [Detector binario V24/V25 — diagnostica clearance e sweep della reach](../user/2026-08-04_detector_v24_v25_clearance_e_sweep_reach.md)

Contesto ereditato:

- [Detector V18 — confronto semantiche del segnale e stop fail-closed](../user/2026-08-03_detector_v18_semantica_segnale_fail_closed.md);
- [Daily report del 2026-07-22](2026-07-22_daily-report.md), usato per la propagazione dei TODO ancora aperti.

## Sintesi

La giornata ha sostituito il detector analogico a due sfere con una sorgente
minimale a due bit force-free, ha implementato una FSM causale separata e ha
attribuito correttamente l'anticipo degli eventi alla geometria del segnale,
non alla GRF primaria né alla logica HS/TO.

La sequenza completa è stata:

1. **V19:** due punti solidali al piede e regola binaria punto-piano; il segnale
   grezzo è finito, booleano, deterministico e non degenere, ma non possiede
   ancora alcun claim HS/TO.
2. **V20:** FSM binaria con debounce globale di 5 ms, onset conservato e
   parità esatta 1 ms/batch 10 ms; il software è corretto, ma la geometria V19
   anticipa gli HS di circa 102-142 ms e fallisce scientificamente DEV02/04.
3. **V21:** sweep geometrico sui soli trial development 02/04; selezionato
   `v21_678b0b5162b706dd`, con 16/16 unità PASS nella verifica FSM reale.
4. **V21 one-shot trial 08:** arresto interno perché il validatore pretendeva
   erroneamente l'uguaglianza fra griglia globale oracle e replay; nessuna
   trace detector è stata campionata e non esiste un risultato scientifico.
5. **V22 development trial 08:** corretto il criterio di coverage, ma un fatto
   descrittivo legittimamente `False` è entrato in `all(checks.values())`;
   secondo arresto interno prima del replay, ancora senza evidenza scientifica.
6. **V23:** separati `assertions` e `facts`; il replay completa regolarmente e
   produce il primo vero FAIL prestazionale sul trial 08: nel plateau 03 a
   2,05 m/s un gap grezzo OFF/OFF di 12 ms genera TO e HS spurii.
7. **V24/V25:** la diagnostica clearance esclude differenze fra modello marker
   e modello runtime; uno sweep locale della sola reach della punta da 27,0 a
   27,5 mm chiude il gap. Il candidato `v25_4b351f67b5b86ab0` supera 24/24
   unità su 02/04/08 e viene congelato come candidato development, non promosso.

Stato terminale della giornata:

`V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED`

Il detector V25 è pronto per i gate H0, non per training o claim protetti.
Trial 05/06, reserve 03/07, corridor a peso positivo e PPO sono rimasti chiusi.

## Separazione fra errori di protocollo e risultati scientifici

| Revisione | Tipo di esito | Significato corretto |
|---|---|---|
| V19 | PASS tecnico raw | I due bit sono validi e non degeneri; nessun claim su HS/TO. |
| V20 | PASS software, FAIL scientifico | La FSM è causale e deterministica; geometria V19 non allineata all'oracle. |
| V21 sweep 02/04 | PASS development | Geometria finalista valida sui soli dati usati per sviluppo. |
| V21 trial 08 | `ERROR_INTERNAL` | Gate oracle troppo restrittivo; detector non campionato, nessun FAIL geometrico. |
| V22 trial 08 | `ERROR` procedurale | Aggregazione booleana errata; detector non campionato, nessun FAIL geometrico. |
| V23 trial 08 | FAIL scientifico | Replay valido; gap OFF/OFF di 12 ms e ciclo spurio a 2,05 m/s. |
| V24 | PASS diagnostico | Gap attribuito alla copertura geometrica, non a mismatch marker/runtime. |
| V25 | PASS development | Reach punta 27,5 mm: 24/24 unità PASS e zero gap interni; H0 e holdout ancora mancanti. |

Gli errori V21/V22 non devono essere reinterpretati come bocciature della
geometria, né cancellati: entrambi gli stage sono consumati e conservati come
evidenza fail-closed. Viceversa V23 non è un errore dell'harness: è una misura
scientifica valida che ha motivato V24/V25.

## Invarianti preservate

Durante l'intera lineage V19-V25 sono rimasti invariati:

- GRF primaria online, relativo contatto fisico e `online_grf.py`;
- supporto ibrido già validato: GRF sinistra online e destra prescritta;
- plugin C++ e semantica dei due SEA;
- FSM V20 durante gli sweep V21 e V25;
- piano detector e regola `signed_clearance <= 0`;
- campionamento detector a 1 ms e consegna alla policy a 10 ms;
- oracle canonici, senza nuova sogliatura della GRF per ogni cadenza;
- forma e ordine delle osservazioni, actor, reward e configurazione attiva;
- default `legacy_events` e morphology reward a peso zero.

Il detector resta force-free e separato dal carico continuo: non aggiunge
sfere, materiali, leggi di contatto o `Force` OpenSim e non viene usato per
generare GRF.

## 1. V19 — segnale binario minimo

V19 ha eliminato la gradualità Hunt-Crossley delle sfere e introdotto due punti
solidali a `/bodyset/foot_l`, uno al tallone e uno alla punta. Il payload
pubblico è separato dallo stream analogico:

```text
binary_phase_sensor_samples = [
  {time_s, left_heel_contact, left_toe_contact}
]
```

La clearance continua è solo diagnostica; nel payload entrano esclusivamente
due `bool`. Il reset campiona `t0` senza emetterlo e ogni step da 10 ms consegna
dieci campioni unici in `(previous_sample_time, t_stop]`.

Il primo placement sulla mesh aveva la punta sempre OFF. Fra le estensioni
15/18/20/25 mm, 25 mm è stata la prima con entrambi i canali sostenuti su
DEV02/04. Il replay completo ha prodotto 143.206 e 143.541 campioni, tutti
finiti e booleani. Questo ha chiuso il problema di qualità del **segnale raw**,
non quello della sua accuratezza biomeccanica.

## 2. V20 — FSM binaria e diagnosi dell'anticipo

La FSM separa le parole grezze dalla fase funzionale:

- `00 -> AIR`;
- `10 -> HEEL`;
- `11 -> BOTH`;
- `01 -> TOE`.

HS è il primo passaggio stabile da AIR a qualunque contatto; TO è il primo
passaggio stabile da contatto ad AIR. Il debounce globale `heel OR toe` è
esattamente 5 ms, non riparte nelle transizioni regionali e conserva come
timestamp l'onset fisico. Un episodio già in stance può produrre un primo TO
marcato come startup partial stance, senza HS sintetico.

L'integrazione iniziale è soltanto `binary_shadow`: non scrive negli eventi
legacy, nelle osservazioni, nella reward, nelle azioni o nella dinamica.

La parità scalar/batch passa, ma 0/16 unità scientifiche V19+V20 superano il
gate. Gli HS sono in mediana 102-142 ms anticipati; il delay software è soltanto
5 ms. È quindi dimostrato che l'errore dominante risiede nel significato
geometrico del segnale, non in un ritardo della FSM.

## 3. V21 — sweep geometrico DEV02/04

Lo sweep ha variato soltanto posizione longitudinale e reach mesh-anchored dei
due punti. Sono stati valutati:

- 900 candidati coarse;
- 11.898 candidati fine;
- 12.798 candidati totali;
- 170 candidati PASS allo screening fine;
- tre finalisti verificati con la FSM V20 reale.

Il finalista congelato `v21_678b0b5162b706dd` usa:

- tallone: x `-0,059315516055 m`, reach `25 mm`;
- punta: x `0,135837908089 m`, reach `27 mm`.

Su DEV02/04 ottiene 16/16 unità PASS, precision/recall 1,0, errore massimo
confirmed HS 24 ms e TO 55 ms, F1 minimo 0,97194, IoU minimo 0,94542 e parità
esatta scalar/batch. Poiché la FSM è rimasta invariata, il miglioramento rispetto
a V19 è evidenza diretta dell'origine geometrica del disallineamento.

Il risultato restava development-only: il candidato non era ancora promosso e
trial 08, 05/06 e 03/07 erano chiusi al momento della selezione.

## 4. Trial 08 — due ERROR procedurali consumati

### V21 one-shot

Il freeze e il ledger sono stati pubblicati prima dell'accesso. Il validatore
richiedeva però, senza necessità scientifica, che la griglia oracle
`[10.678, 154.900]` coincidesse con il replay `[10.690, 154.890]`. Tutte le
quattro view scoreable erano correttamente coperte.

L'esecuzione si è fermata prima di decodificare l'IK e prima del campionamento
dei punti. In accordo al contratto one-shot, lo stage è stato consumato senza
retry con stato `ERROR_INTERNAL_V21_TRIAL08_CONSUMED`.

### V22 development

V22 ha corretto la coverage e ha verificato esattamente i margini 12/10
campioni. Ha però inserito il fatto descrittivo
`global_grid_equality_required=False` nello stesso dizionario delle asserzioni,
poi valutato con `all(checks.values())`.

Anche V22 si è fermata prima del detector. Il nuovo errore ha mostrato una
lacuna di test end-to-end del binding dell'oracle reale, non una regressione
dei dati o della geometria. V22 è stata conservata e non modificata per retry.

## 5. V23 — primo risultato scientifico sul trial 08

V23 separa strutturalmente `assertions` e `facts`, esegue il binding reale nel
preflight e completa il replay. Sei unità su otto passano; le due modalità di
consumo falliscono entrambe soltanto sul plateau 03 a 2,05 m/s.

Il fallimento è localizzato a un unico intervallo interno allo stance:

```text
99.840-99.881 s  heel ON, toe OFF
99.882-99.893 s  heel OFF, toe OFF   (12 ms)
da 99.894 s      heel OFF, toe ON
```

Con debounce 5 ms la FSM genera coerentemente un TO e un HS spurii. F1 0,9584
e IoU 0,9200 restano sopra soglia, ma conteggi e cicli sono 39/38, con flight
minimo 7 ms contro il gate di 30 ms. Questo basta a chiudere V21 in FAIL
scientifico e impedisce qualunque promozione o H0 sul candidato.

## 6. V24/V25 — diagnosi e correzione locale

V24 ha ricampionato le clearance continue attorno al gap usando sia il modello
marker sia quello runtime. Le tracce, le sensibilità alla reach e i bit sono
identici con differenza massima 0 m. È quindi esclusa una divergenza fra i due
modelli.

Nel gap la punta manca il piano di soli 0,009-0,346 mm, mentre il tallone arriva
fino a 2,540 mm. L'incremento minimo stimato della sola reach punta è 0,347 mm;
questo ha motivato uno sweep locale preregistrato senza cambiare x o FSM.

V25 ha confrontato nove geometrie uniche. Il solo candidato eleggibile mantiene
il tallone a 25,0 mm e porta la punta da 27,0 a 27,5 mm. La verifica esatta su
02/04/08 ottiene:

- 24/24 unità PASS;
- precision e recall HS/TO 1,0;
- conteggi, ordine e cicli esatti;
- zero gap interni su 398 cicli oracle;
- parità sequenziale/batch su tutti e tre i trial;
- F1 minimo 0,958970 e IoU minimo 0,921174;
- errore confirmed massimo HS/TO 27/53 ms;
- flight minimo 364 ms;
- prossimità mesh e reach verticale PASS.

Il candidato `v25_4b351f67b5b86ab0` è stato congelato globalmente con lock SHA
`04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346`.
Il run è consumato, non ripetibile e non riselezionabile. L'integrazione dormant
/ shadow è stata completata mantenendo `legacy_events` come default, ma questo
non equivale a promozione active o compatibilità H0.

## File principali modificati o aggiunti

### Runtime e configurazione V19/V20

- `binary_phase_detector.py`;
- `config.py`;
- `path_resolver.py`;
- `model_loader.py`;
- `simulation_runner.py`;
- `Trajectory Generator/binary_phase_fsm.py`;
- `Trajectory Generator/osim_trj_cmc_like.py`;
- `Trajectory Generator/baseline_MLP/training_config.py`;
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`;
- `Trajectory Generator/baseline_MLP/rollout_eval.py`;
- `validation/experimental_detector_profiles/two_point_binary_v19_outsole_25mm.json`;
- `validation/experimental_detector_profiles/README.md`.

### Validazione V19/V20/V21

- `validation/validate_binary_phase_detector_v19_raw_geometry.py`;
- `validation/test_binary_phase_detector_v19.py`;
- `validation/binary_phase_detector_v19_geometry_receipt.json`;
- `validation/validate_binary_phase_fsm_v20_development.py`;
- `validation/test_binary_phase_fsm_v20.py`;
- `validation/test_binary_phase_fsm_env_v20.py`;
- `validation/binary_phase_fsm_v20_development_receipt.json`;
- `validation/sweep_binary_phase_detector_v21_geometry.py`;
- `validation/test_sweep_binary_phase_detector_v21_geometry.py`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/`.

### Trial 08 V21-V23

- `validation/freeze_binary_phase_detector_v21_trial08.py`;
- `validation/validate_binary_phase_detector_v21_trial08_one_shot.py`;
- `validation/test_binary_phase_detector_v21_trial08_one_shot.py`;
- `validation/binary_phase_detector_v21_trial08_freeze_lock.json`;
- `validation/binary_phase_detector_v21_trial08_execution_ledger.json`;
- `validation/binary_phase_detector_v21_holdout_runs/2026-08-04_trial08_one_shot/`;
- `validation/freeze_binary_phase_detector_v22_trial08_development.py`;
- `validation/validate_binary_phase_detector_v22_trial08_development.py`;
- `validation/test_binary_phase_detector_v22_trial08_development.py`;
- `validation/binary_phase_detector_v22_trial08_development_freeze_lock.json`;
- `validation/binary_phase_detector_v22_trial08_development_execution_ledger.json`;
- `validation/binary_phase_detector_v22_development_runs/2026-08-04_trial08_oracle_coverage_fix/`;
- `validation/freeze_binary_phase_detector_v23_trial08_development.py`;
- `validation/validate_binary_phase_detector_v23_trial08_development.py`;
- `validation/test_binary_phase_detector_v23_trial08_development.py`;
- `validation/binary_phase_detector_v23_trial08_development_freeze_lock.json`;
- `validation/binary_phase_detector_v23_trial08_development_execution_ledger.json`;
- `validation/binary_phase_detector_v23_development_runs/2026-08-04_trial08_assertions_facts_fix/`.

### Diagnostica e freeze V24/V25

- `validation/diagnose_binary_phase_detector_v24_clearance_gap.py`;
- `validation/test_binary_phase_detector_v24_clearance_gap.py`;
- `validation/binary_phase_detector_v24_diagnostic_runs/`;
- `validation/sweep_binary_phase_detector_v25_geometry.py`;
- `validation/test_sweep_binary_phase_detector_v25_geometry.py`;
- `validation/binary_phase_detector_v25_geometry_runs/`;
- `validation/freeze_binary_phase_detector_v25_development_candidate.py`;
- `validation/test_freeze_binary_phase_detector_v25_development_candidate.py`;
- `validation/binary_phase_detector_v25_development_candidate_freeze_lock.json`.

## Test, gate e verifiche

### V19

- 11/11 test nuovi PASS;
- 24/24 test routing detector PASS;
- 5/5 test trasporto high-rate/FSM legacy PASS;
- 7/7 test core online-GRF PASS;
- 6/6 test contratto V17 e 12/12 gate development V17 PASS;
- 61/61 test primary-GRF readiness PASS, uno skip atteso;
- smoke OpenSim reset + 10 ms e replay raw DEV02/04 PASS.

### V20

- 61 test/regressioni PASS;
- parità scalar/batch completa PASS;
- 0/16 unità scientifiche PASS per V19+V20, con FAIL attribuito alla geometria;
- trial 08 non aperto in questo stage.

### V21

- 17/17 test specifici PASS e 45/45 regressioni V19/V20/V21 PASS;
- 500 tracce sintetiche: firma dello screening uguale alla FSM reale;
- 12.798 candidati unici valutati, top tre con 16/16 unità PASS;
- strict JSON/JSONL, hash, atomicità, no-clobber, `py_compile`, Ruff e
  `git diff --check` PASS.

### V21/V22 trial 08

- V21: 41/41 test consolidati, freeze e preflight PASS, poi ERROR sul gate
  `grid_start/grid_end/grid_count` prima del detector;
- V22: 15/15 test e coverage reale PASS, poi ERROR sull'aggregazione booleana
  prima del detector;
- per entrambi ledger/receipt e hash terminali verificati, nessun retry.

### V23

- 16/16 test PASS e preflight completo PASS;
- 12 JSON strict e nove artifact verificati per hash/dimensione;
- replay regolare, parità scalar/batch PASS;
- 6/8 unità PASS e 2/8 FAIL scientifiche sul medesimo plateau 03.

### V24/V25

- V24 15/15 test PASS;
- V25 14/14 test PASS;
- freeze V25 20/20 assertion preflight e 5/5 test indipendenti PASS;
- strict JSON/JSONL, hash, finitezza, no-clobber, progress bar/ETA/elapsed,
  `py_compile` e `git diff --check` PASS;
- audit indipendente finale senza blocker.

La creazione retrospettiva di questo daily non ha eseguito nuove simulazioni:
riporta esclusivamente evidenze già registrate nei sette report sorgente.

## Gate e scope rimasti chiusi

- trial protetti 05 e 06 non aperti;
- trial reserve 03 e 07 non aperti;
- trial storico 01 non riaperto;
- nessun candidato promosso nel runtime active;
- nessun H0 A/B/C eseguito;
- nessun warm start o adattamento dell'actor eseguito;
- nessun corridor con morphology reward positivo;
- nessun update PPO o training avviato;
- nessuna modifica a GRF primaria, contatto fisico, plugin C++ o SEA;
- nessun claim Windows: numeri ancora macOS arm64, DLL/parità Windows non
  attestati.

## TODO chiusi il 2026-08-04

- [x] Sostituire la gradualità delle due sfere con due segnali puramente
  booleani e force-free.
- [x] Definire una FSM causale HS/TO separata, con debounce, onset e consegna
  espliciti, API scalar/batch e input fail-closed.
- [x] Dimostrare che la FSM V20 non è la causa dell'anticipo V19.
- [x] Eseguire uno sweep geometrico preregistrato su DEV02/04 con progress bar,
  ETA e tempo trascorso.
- [x] Conservare V21 e V22 come ERROR procedurali senza attribuire loro un FAIL
  scientifico e senza retry degli stage consumati.
- [x] Correggere in V23 coverage e separazione `assertions`/`facts`, includendo
  un test end-to-end dell'oracle reale.
- [x] Localizzare il gap V23 di 12 ms e verificarne l'origine geometrica con le
  clearance continue e la parità marker/runtime.
- [x] Correggere il gap senza cambiare FSM: reach punta 27,5 mm, 24/24 unità
  PASS e zero gap interni.
- [x] Congelare V25 come candidato development no-clobber e integrarlo soltanto
  in modalità dormant/shadow.
- [x] Mantenere separati detector discreto, GRF primaria continua e GRF
  prescritta oracle.

## TODO aperti e propagati

### Prossimo gate V25/H0

- [ ] Congelare prima di qualunque esecuzione un protocollo H0 A/B/C simmetrico
  e content-bound.
- [ ] Implementare e testare il journal V25 del caso A, il replay shadow B,
  l'adapter V20 verso `ProstheticPhaseFSM` e la modalità active C.
- [ ] Coprire esplicitamente startup in AIR e partial stance, primo TO parziale,
  assenza di HS sintetici e assenza di fallback agli eventi legacy sinistri.
- [ ] Completare driver/comparatore bit-exact, tape stocastico, mapping delle
  tracce SEA e destinazioni no-clobber.
- [ ] Congelare un receipt separato di execution-unlock e soltanto allora
  eseguire la matrice H0 A/B/C, mantenendo morphology reward a zero.
- [ ] Verificare layout 35/84, restore/save-reload, finitezza, 500/500 step,
  almeno due cicli, penetrazione sotto 25 mm e non-regressione SEA/reserve.
- [ ] Decidere se occorra adattare H0 solo dopo l'esito del caso active C; un
  eventuale adattamento deve appartenere a una nuova lineage preregistrata.
- [ ] Solo dopo il PASS H0 decidere l'apertura one-shot di 05 e poi 06, senza
  tuning o rescue fra i due trial.
- [ ] Mantenere 03/07 chiusi e non usare 05/06 per selezione geometrica.

### Detector, routing e deployment

- [ ] Mantenere `legacy_events` come default finché V25 active, H0 e i trial
  protetti non abbiano superato tutti i gate.
- [ ] Attestare nel closed-loop che `normal_force_bw` e `in_contact` provengono
  sempre dalla GRF primaria separata e aggiungere/mantenere il regression test
  dual-stream richiesto dal daily 2026-07-22.
- [ ] Eseguire shadow e active A/B a policy congelata prima di decidere un
  fine-tuning; il PASS development prescribed non prova deployability.
- [ ] Validare il detector con rumore e delay realistici, poi HIL, e chiarire
  la ground truth/semantica di `initial_contact` rispetto alla GRF totale.
- [ ] Attestare DLL e parità numerica su Windows x86; fino ad allora il claim
  quantitativo resta macOS arm64.

### Morphology corridor

- [ ] Completare un rollout live shadow da 500 step e un confronto A/B in una
  sessione OpenSim pulita; il replay frozen non lo sostituisce.
- [ ] Definire bordo phase-dependent e scelta fra sicurezza immediata o
  retrospettiva, quindi implementare la riscrittura complete-segment/
  complete-episode prima del GAE.
- [ ] Gestire segmenti incompleti, timeout, bootstrap, `WAIT_HS`, disponibilità
  morfologica, stato temporale robusto e regime ankle lento.
- [ ] Aggiungere logging per evento e, solo dopo i gate, eseguire un A/B corto
  a morphology weight `0/0.0025/0.005`, conservando ogni update e monitorando
  loss, corridor, cicli, penetrazione, return, SEA, reserve e drift actor.
- [ ] Validare qualunque pilot su start/seed held-out e almeno un profilo o
  modello esterno ad AB06 prima di training lungo o promozione.
- [ ] Mantenere le reserve come gate fisico, non come reward, finché l'origine
  dei residui non sia attribuita.

### Training, reward e robustezza

- [ ] Non avviare training sul detector prima dei gate H0 e protected; tenere
  checkpoint H0/best, V9 e tutte le revisioni V19-V25 immutabili e confrontabili.
- [ ] Valutare i candidati con multistart, seed held-out, worst-case recovery,
  cicli, penetrazione, reserve, SEA e clipping, non soltanto con il return.
- [ ] Non modificare reward, soglia hard 25 mm, feature actor o semantica GRF
  per forzare la compatibilità col detector; separare causalmente reward, GRF,
  reserve e FSM.
- [ ] Conservare exact-start, compaction, interleaving, singola epoca e gate
  reserve condition-matched; concentrare recovery/data collection sugli stati
  event-aligned critici e preregistrare la non-regressione reserve.
- [ ] Mantenere i seed 126-128 sigillati fino a un candidato completo, eseguire
  un pilot controllato prima di run lunghi e analizzare return/advantage per
  start.
- [ ] Non ripetere dimezzamenti di learning rate o proiezioni sulle stesse
  trace/seed; riprovare `sigma=0.0075` su almeno tre seed per start solo dopo un
  PASS completo a `sigma=0.005`.
- [ ] Differire memoria ricorrente finché non emerga un limite sequenziale non
  coperto dallo stato Markov e spiegare il TO precoce storico nella seconda
  stance dell'oracolo multi-ciclo.
- [ ] Eseguire prove più lunghe e su trial, velocità e soggetti differenti
  prima di dichiarare generalizzazione o deployment.

### Audit ex-novo, hardware e TODO storici

- [ ] Confrontare H0 e il checkpoint storico logical 24 con protocollo identico
  e overlay di azioni, served, cinematica, SEA, GRF, eventi e reward.
- [ ] Eseguire ablation progressive del plant prescribed e training A/B da H0
  modificando un solo gruppo per volta: detector/FSM, GRF diretto, morphology e
  command-rate.
- [ ] Congelare ordine/scaling feature, frequenza, unità, normalizzazione BW,
  filtri, FSM, reset, governor, limiti, fallback e watchdog del contratto
  hardware.
- [ ] Validare export actor-only, equivalenza host-target, latenza worst-case,
  HIL e infine human-in-the-loop.
- [ ] Portare la GRF online a stato production-ready prima del deployment,
  mantenendola separata dai due sensori discreti.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta,
  TODO storico ancora aperto.

