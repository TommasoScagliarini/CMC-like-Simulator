# H0 V11 weighted full-mean — fit PASS, rollout autonomo FAIL

Data: 2026-08-09

## Esito

Il protocollo H0 V11 `weighted full-mean` è stato implementato, verificato,
congelato ed eseguito nella lineage autorizzata. Il design audit, il fit P0,
i tre round safe DAgger e i fit P1–P3 hanno superato i rispettivi gate.

Il protocollo **non è training ready**. Il primo rollout finale con P3 puro,
senza teacher, blending o safety latch, è terminato al passo 259/500 perché la
penetrazione della GRF ha raggiunto `25,5699117 mm`, superando il limite rigido
`< 25 mm`. Il gate terminale è:

```text
FAIL_H0_PRIMARY_SPLIT_V11_SAFE_DAGGER_PIPELINE
STOP_V11_TERMINAL_NO_RETRY_SWEEP_OR_RESCUE
```

Il checkpoint P3 è stato congelato come artifact pre-finale con ID
`H0_PRIMARY_SPLIT_V11_P3_b9581a50130ed6fe`, ma il FAIL lo rende non
promuovibile. Non esiste un `checkpoint_zero` eleggibile e non viene fornito
un comando di warm-start.

Il detector binario V26 e la relativa FSM non hanno causato il fallimento:
gli eventi osservati erano ordinati e validi. La causa è una instabilità
closed-loop della policy P3 quando viene rimossa la protezione del teacher.

## Problema affrontato

Le lineage H0 precedenti avevano mostrato che il detector V26 era utilizzabile
nel runtime, ma l'adattamento supervisionato dell'actor non soddisfaceva in
modo affidabile i gate offline. V11 è stato autorizzato per verificare se un
fit dell'intera rete delle medie, invece di una correzione residuale più
limitata, potesse ricostruire il teacher coerente con la semantica V26 e
produrre un candidato autonomo.

Il problema non riguardava:

- geometria o timing del detector V26;
- GRF primaria online;
- plugin C++ del contatto o dei SEA;
- semantica SEA;
- morphology reward o PPO.

V11 doveva risolvere due requisiti distinti:

1. raggiungere le soglie di imitazione offline preregistrate;
2. conservare stabilità fisica e cicli validi quando il candidato agisce senza
   teacher nel simulatore completo.

Il primo requisito è stato soddisfatto; il secondo no.

## Soluzione V11 implementata

Il contratto finale è:

```text
AB06_H0_PRIMARY_SPLIT_V11_V26_WEIGHTED_FULL_MEAN_SAFE_DAGGER
h0_primary_split_v11_weighted_full_mean_v1
```

La strategia di fit congelata comprende:

- sorgente H0 fresca per ciascun fit P0–P3;
- actor standard `35 -> 256 -> 256 -> 2`, attivazioni `tanh`;
- aggiornamento supervisionato dell'intera rete delle medie;
- `logstd` congelata e byte-exact;
- modulo sorgente actor-only/inference-only, senza critic presente;
- zero update critic e zero update PPO;
- colonne clock 0 e 1 forzate bit-zero;
- statistiche di normalizzazione calcolate una sola volta sui 3.000 campioni
  base in float64 e convertite in float32;
- `std_floor=1e-4`;
- normalizzazione incorporata nel primo layer prima del salvataggio, senza
  wrapper runtime;
- peso `100` per le righe di reset e peso `1` per le altre righe;
- nessun anchor, hard polish, fallback o sweep.

Ottimizzazione congelata:

- AdamW full-batch, seed `20260807`, weight decay `1e-7` e gradient clip `10`;
- epoche 1–1500 a `3e-4`;
- epoche 1501–2500 a `1e-4`;
- epoche 2501–3000 a `3e-5`;
- LBFGS successivo con learning rate `0,7`, massimo 300 iterazioni e 600
  valutazioni, history 50 e line search `strong_wolfe`.

## Design audit e freeze

Il design audit è stato eseguito una sola volta senza scrivere checkpoint.
Ha riprodotto il fit P0 atteso con:

| Metrica | Osservata | Soglia | Esito |
|---|---:|---:|---|
| RMSE | `0,0049338517` | `<= 0,006` | PASS |
| Errore assoluto massimo | `0,0558278039` | `<= 0,060` | PASS |
| Errore massimo reset | `0,0003790185` | `<= 0,003` | PASS |
| Errore massimo del fold | `4,1723e-7` | `<= 1e-6` | PASS |

Audit di conservazione:

- H0 sorgente byte-exact: PASS;
- `logstd` byte-exact: PASS;
- colonne clock bit-zero: PASS;
- critic assente, con zero parametri: PASS;
- checkpoint candidato scritto dal design audit: zero;
- trial protetti e reserve aperti: nessuno.

Artifact autorevoli:

- design audit receipt SHA-256:
  `96f848d72e2aa72bec4bb108416a84c38730a7b9ccc2d1f36bd172f5a897bacf`;
- preflight receipt SHA-256:
  `e5cc499eac563516054fe10497bd54ae60790750cc4a8cc92c14f876cd5864a5`;
- execution lock SHA-256:
  `deba2d6e69e97f3b6814e8a7dcbd79cbef6f1ec7f938c94b6d6604c87e4404f1`.

Il lock vietava esplicitamente retry, sweep, rescue, update PPO e apertura dei
trial protetti.

## Fit P0–P3

Ogni stage ha ricreato il candidato partendo dall'H0 congelato, usando il
corpus cumulativo disponibile fino a quel momento.

| Stage | Campioni | RMSE | Max abs | Max reset | Gate |
|---|---:|---:|---:|---:|---|
| P0 | 3.000 | `0,004933852` | `0,055827804` | `0,000379018` | PASS |
| P1 | 4.000 | `0,004968897` | `0,056236781` | `0,000573277` | PASS |
| P2 | 5.000 | `0,004881451` | `0,056750298` | `0,000357516` | PASS |
| P3 | 6.000 | `0,004962145` | `0,057175428` | `0,000352509` | PASS |

Tutti e quattro i fit soddisfano le soglie offline. Tuttavia:

- l'RMSE non migliora monotonicamente;
- l'errore massimo peggiora da P0 a P3;
- P3 raggiunge il `95,3%` del limite massimo ammesso;
- il PASS offline misura l'imitazione sugli stati raccolti, non la stabilità
  della distribuzione di stati prodotta autonomamente dall'actor.

Il fit P3 ha prodotto:

- candidate ID: `H0_PRIMARY_SPLIT_V11_P3_b9581a50130ed6fe`;
- candidate freeze SHA-256:
  `f2b496d18e26eb06e73672cb32b197ab54fe56e1cf1edbee4bd0a58fa6c72735`;
- receipt P3 SHA-256:
  `96106563ae9e48fb0b79118fbdc0012d4f5eb7ee08cc5d9e9d9d1614e169279c`.

Questo freeze attesta contenuto e provenienza; non costituisce promozione.

## Tre round safe DAgger

Sono stati eseguiti tre round, ciascuno con un caso deterministico a offset
`-0,20 s` e un caso stocastico nominale con seed 126. I coefficienti student
richiesti erano rispettivamente `0,25`, `0,50` e `0,75`.

| Round | Caso | Step | Penetrazione max | Interventi latch | Dipendenza teacher | Cicli |
|---:|---|---:|---:|---:|---:|---:|
| 1 | deterministic -0,20 | 500 | `24,308 mm` | 253 | 500/500 | 3 |
| 1 | stochastic 126 | 500 | `22,675 mm` | 276 | 500/500 | 2 |
| 2 | deterministic -0,20 | 500 | `24,296 mm` | 258 | 500/500 | 3 |
| 2 | stochastic 126 | 500 | `22,493 mm` | 275 | 500/500 | 2 |
| 3 | deterministic -0,20 | 500 | `23,919 mm` | 262 | 500/500 | 3 |
| 3 | stochastic 126 | 500 | `22,462 mm` | 273 | 500/500 | 2 |

Tutti i gate di raccolta hanno chiuso PASS, ma la lettura closed-loop è
diversa dal semplice conteggio degli episodi completati:

- il latch interveniva nel `50,6–55,2%` degli step;
- tutte le 500 azioni di ogni episodio conservavano una dipendenza dal teacher
  o dal blending;
- quando la penetrazione superava 15 mm, il latch portava
  `effective_alpha=0`, servendo teacher puro;
- il rilascio avveniva soltanto sotto 10 mm durante lo swing;
- il terzo round valutava fisicamente P2; P3 veniva creato dopo quel round e
  non era mai stato eseguito autonomamente prima del test finale.

I round DAgger hanno quindi dimostrato sicurezza **con shield**, non autonomia
del candidato.

## Fallimento del rollout finale

Il primo dei sei casi finali previsti era
`deterministic_offset_minus_0p20`. In questa fase erano disabilitati teacher,
blending e safety latch: le azioni servite coincidevano esattamente con la
media di P3.

Risultato:

- ultimo step completato: `259/500`;
- tempo simulazione: `16,3368709838 s`;
- end reason: `grf_penetration`;
- penetrazione step 258: `24,6584025 mm`;
- penetrazione step 259: `25,5699117 mm`;
- superamento del limite: `0,5699117 mm`;
- cicli validi completati: `1`, contro almeno 2 richiesti;
- azione terminale finita e non clippata:
  `[0,7175263166, 0,3900616467]`;
- reserve norm terminale: `505,923 N m`;
- reserve norm massimo parziale: `581,095 N m`.

La penetrazione non è il risultato di un singolo NaN o salto numerico. Dopo un
minimo locale di `18,786 mm` al passo 248 è cresciuta in modo continuo fino al
superamento del gate al passo 259.

Controlli che restano puliti:

- zero valori non finiti;
- zero action clipping;
- zero timeout;
- zero fallback SEA;
- zero fallback generici;
- zero soluzioni di static optimization non accettate;
- zero routing failure;
- zero step-contract failure;
- dieci finestre di controllo e dieci campioni V26 per ogni step completato;
- zero update actor, critic o PPO durante il rollout.

I warning storici SciPy/SLSQP di clipping interno ai bound e gli eventuali
fallback bounded least-squares verificati nei rollout precedenti non hanno
causato questo stop.

## Perché non è un errore del detector V26

Prima dell'arresto V26 ha prodotto tre eventi funzionali ordinati:

```text
HS 13.935870983805 s
TO 15.089870983804 s
HS 15.601870983804 s
```

L'audit eventi registra:

- duplicati: zero;
- eventi fuori ordine: zero;
- sorgenti non V26: zero;
- fallback: zero;
- eventi hard-invalid: zero;
- timeout FSM: zero.

Il controllo aggregato degli eventi risulta formalmente FAIL soltanto perché
l'episodio termina con 2.590/5.000 campioni e un solo ciclo. È una conseguenza
dello stop fisico, non la sua causa.

La diagnosi è quindi:

```text
deployment/covariate-shift gap fra raccolta teacher-shielded
e rollout autonomo della policy P3
```

Piccoli errori di imitazione, pur entro le soglie offline, cambiano gli stati
visitati dalla policy pura. Il corpus DAgger corrente non dimostra che P3
possa recuperare autonomamente nelle regioni in cui il latch aveva sempre
sostituito lo student con il teacher.

Allentare retroattivamente il limite di penetrazione non risolverebbe il
problema: la traiettoria era ancora in crescita e il candidato mostrava già
una forte regressione delle reserve.

## Stato della pipeline

- design audit V11: PASS;
- preflight e execution lock: PASS;
- fit P0: PASS;
- raccolta round 1: 2/2 PASS;
- fit P1: PASS;
- raccolta round 2: 2/2 PASS;
- fit P2: PASS;
- raccolta round 3: 2/2 PASS;
- fit e freeze P3: PASS come artifact pre-finale;
- primo rollout finale puro: **FAIL terminale**;
- altri cinque rollout finali: non aperti;
- V10Q/qualification: non aperta;
- trial protetti 05/06: non aperti;
- trial reserve 03/07: non aperti;
- zero-update port: non aperto;
- corridor con reward positivo: non aperto;
- PPO: zero update;
- `H0_TRAINING_READY`: **NO**.

Ledger terminale SHA-256:
`130b64f3e653e0d0afa8602c6ee65e0c20204fb3cace53365cdb9e0240bb4efc`.

Gate del rollout fallito SHA-256:
`a73e898d5e618defe67b5fb668bf8bf281fe8d8f4d19649807eaf176f828cfcb`.

Failure receipt SHA-256:
`9b25a6870ccc537b823cd5d9f388939b75c1cd4929b6376670bb8a239bc8727c`.

## File introdotti

Contratto, fit e runner V11:

- `validation/h0_primary_split_v11_weighted_full_mean_contract.py`;
- `validation/h0_primary_split_v11_weighted_fit.py`;
- `validation/run_h0_primary_split_v11_design_audit.py`;
- `validation/run_h0_primary_split_v11_weighted_full_mean.py`.

Test V11:

- `validation/test_h0_primary_split_v11_weighted_full_mean_contract.py`;
- `validation/test_h0_primary_split_v11_weighted_fit.py`;
- `validation/test_run_h0_primary_split_v11_design_audit.py`;
- `validation/test_run_h0_primary_split_v11_weighted_full_mean.py`.

Freeze e receipt:

- `validation/h0_primary_split_v11_design_audit_receipt.json`;
- `validation/h0_primary_split_v11_weighted_full_mean_preflight_receipt.json`;
- `validation/h0_primary_split_v11_weighted_full_mean_execution_lock.json`.

Artifact completi P0–P3, raccolte, trace, checkpoint pre-finale e failure:

- `validation/h0_primary_grf_split_adaptation_runs/`
  `2026-08-09_h0_primary_split_v11_v26_weighted_full_mean_safe_dagger/`.

SHA dei quattro file eseguibili principali:

- contract:
  `1ee981c087973dd417f88214894171be21f75fde54fd821d8a0eb2958db34729`;
- fitter:
  `877bb79d9bf4bd8d87b9d07dbdee127ca259cc8cc82fac1e592ea37032029fdc`;
- design-audit runner:
  `171caafb447a589f081953326fc34cd49422cb48cad526346127907e01ac5b9b`;
- pipeline runner:
  `ada9cc7bdd4c874caee39ce645cba5bc4d2fcf166b8975fd55acf78bba96d559`.

Non sono stati modificati `online_grf.py`, geometria/profilo/materiale della
GRF primaria, plugin C++, semantica SEA, detector V25/V26, FSM V20/V26 o
checkpoint H0 storici.

## Test e verifiche

Prima del design audit e del freeze:

- suite mirata V11: 43/43 PASS;
- `py_compile`: PASS;
- Ruff: PASS;
- `git diff --check`: PASS;
- audit indipendente pre-freeze: GO;
- strict JSON, finitezza e hash dei receipt: PASS.

Dopo l'esecuzione terminale:

- `py_compile`: PASS;
- Ruff: PASS;
- `git diff --check`: PASS;
- strict JSON sui ledger terminali: PASS, nessun NaN/Inf;
- audit forense indipendente: conferma FAIL fisico closed-loop e detector
  regolare;
- rerun suite mirata: 42 PASS, 1 FAIL documentale.

L'unico test post-run fallito è
`test_design_audit_payload_passes_real_contract_without_writing_checkpoint`.
Il test presume che il receipt one-shot del design audit non esista; dopo una
esecuzione valida il receipt esiste necessariamente. Non è una regressione
del runtime né la causa del FAIL V11. Il codice V11 è execution-locked e non è
stato modificato retroattivamente per rendere verde il test.

## TODO e decisione richiesta

- [ ] Autorizzare, se si intende proseguire, una nuova lineage V12 separata.
  V11 non può essere ritentato, salvato o reinterpretato.
- [ ] Aggiungere nel nuovo protocollo un rollout fisico breve, puro e
  unblended dopo ogni candidato P0/P1/P2/P3, prima di usarlo come base
  scientifica dello stage successivo.
- [ ] Rendere frequenza e durata degli interventi del safety latch gate
  espliciti di mancata autonomia, invece di considerare sufficiente il
  completamento dell'episodio protetto.
- [ ] Raccogliere e pesare esplicitamente gli stati delle regioni di recovery,
  evitando che il teacher nasconda la divergenza dello student senza che il
  protocollo la contabilizzi.
- [ ] Correggere nella prossima lineage il test one-shot non idempotente,
  mantenendo immutato il receipt V11 storico.
- [ ] Solo dopo sei rollout autonomi PASS: eseguire V10Q/qualification, poi
  richiedere autorizzazione separata per i trial protetti 05 e 06.
- [ ] Solo dopo tutti i gate: eseguire zero-update save/reload, produrre il
  vero `checkpoint_zero` e pubblicare il comando di warm-start.
- [ ] Mantenere V26, GRF primaria, SEA, morphology reward positivo, corridor
  training e PPO intoccati fino a un `H0_TRAINING_READY` reale.

