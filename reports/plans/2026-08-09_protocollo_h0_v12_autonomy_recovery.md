# Protocollo H0 V12 — autonomia, coverage e recovery

Data: 2026-08-09  
Instruction check token: `CMC_AGENT_OK_2026`

## Stato e autorità

Questo documento definisce il design della nuova lineage H0 V12. Il comando
utente `procedi`, nel contesto della proposta immediatamente precedente,
autorizza progettazione e freeze del protocollo. Non autorizza ancora:

- fit actor;
- design audit numerico consumante;
- reset o step dell'ambiente;
- rollout fisici;
- creazione del claim di esecuzione;
- PPO, update critic o apertura dei trial protetti/reserve.

L'esecuzione richiederà un secondo consenso esplicito e un execution lock
separato, creato no-clobber dopo che fitter, labeler e runner saranno
implementati, testati e hash-bound.

Per rispettare il perimetro del repository dedicato a rete, policy e training,
tutto il codice V12, i test e i futuri artifact vivono sotto
`Trajectory Generator/baseline_MLP/validation/`. Le evidenze V11 restano
immutabili e vengono lette dal loro namespace storico `validation/`.

## Problema V11 da risolvere

V11 ha ottenuto fit offline P0-P3 entro gate, ma P3 ha fallito il primo
rollout puro al passo 259 per penetrazione `25,5699117 mm`, contro il limite
rigido `<25 mm`.

L'audit forense mostra che il problema non è un errore di imitazione sugli
stati shielded:

- `1.597/3.000` righe DAgger (`53,23%`) sono latch-active;
- tutte le `3.000/3.000` azioni di raccolta dipendono dal teacher perché
  `alpha < 1`;
- nel round 3 deterministico il latch copre i passi `214-293`, quindi include
  l'intero ramo in cui il rollout puro termina al passo 259;
- P3 replica il teacher entro circa `0,0015` sullo stato shielded del passo
  259; la trace pura salvata non contiene invece i segnali legacy necessari a
  ricostruire una label teacher coerente sugli stati autonomi;
- tutti i passi puri `201-259` superano il p95 di distanza kNN dal corpus;
- nelle 3.000 righe shielded V11 soltanto 8 hanno penetrazione corrente
  `>=24 mm` e nessuna arriva a `25 mm`.

La causa operativa è quindi covariate shift closed-loop: il teacher mantiene
la raccolta su un ramo diverso da quello percorso dallo student puro.

## Invarianti

V12 non modifica:

- detector geometrico V25 o FSM V26;
- GRF primaria o separazione detector/carico;
- plugin C++ e semantica SEA;
- layout actor `35`, layout completo `84` o feature hardware;
- morphology reward, che resta a peso zero;
- limite fisico di penetrazione `<0,025 m`;
- H0, V11 o qualunque artifact storico;
- trial 05/06 e reserve 03/07;
- logstd, critic o stato PPO.

## Corpus iniziale

P0 parte dal corpus P3 V11 byte-bound:

```text
3.000 righe base coherent-teacher
+ 3.000 righe same-state safe-DAgger V11
= 6.000 righe
```

Le righe shielded V11 non ricevono un nuovo risk multiplier: l'audit ha già
dimostrato che il fit del teacher su quelle regioni è accurato. Restano validi
il peso reset `100`, la normalizzazione sulle sole 3.000 righe base, il fold
nel primo layer, i clock bit-zero e il fit full-mean da H0 fresco.

## Pure probe e label observer-only

Dopo ogni fit P0-P3 viene eseguito un probe deterministico completo sul caso
che ha esposto V11:

```text
deterministic_offset_minus_0p20
500 policy step
teacher non caricato
teacher query = 0
blend = 0
latch = 0
```

Il probe deve persistere, oltre alla trace actor, il payload minimo necessario
a ricostruire successivamente il teacher coerente:

- boundary time;
- angoli e velocità protesiche knee/ankle;
- detector analogico legacy left normal GRF e contatto;
- eventi legacy HS/TO, relativi timestamp causali e phase state;
- body weight, event-contract ID e fingerprint della FSM legacy;
- osservazioni actor e `previous_penetration_m` di ciascuno step.

Soltanto dopo la chiusura del rollout un worker observer-only può caricare H0,
ricostruire sequenzialmente il teacher view e produrre label sugli stessi stati
visitati dallo student. Il labeler:

- non resetta o avanza l'ambiente;
- non serve azioni al plant;
- non modifica probe o checkpoint;
- cambia soltanto le colonne teacher `10:24`;
- conserva le colonne invarianti byte-exact;
- produce un corpus no-clobber legato a probe, trace e hash del teacher.

Questo separa nettamente evidenza di autonomia e acquisizione di label.

## Coverage causale

La coverage usa la stessa normalizzazione float32 di V11. Sono esclusi i clock
disabilitati 0-1 e sono usate le feature 2-34.

Riferimento congelato:

```text
corpus V11 P3: 6000 x 35 float32
metrica: nearest-neighbor RMS-z su 33 feature
leave-one-out p95: 0.07945888479650812
```

La trace terminale pura V11 è anch'essa hash-bound: 164/259 stati superano
il p95 e tutti i 59 passi `201-259` sono OOD; la distanza vale circa `0,22455`
al passo 201 e `0,49344` al passo 259.

Il leave-one-out storico usa `scipy.spatial.cKDTree`, `k=8`, self
esplicitamente escluso e tie-break sull'indice minimo. Le query delle nuove
righe usano invece accumulazione `math.fsum` in ordine feature su un audit dei
64 candidati più vicini; il minimo escluso deve avere margine certificato.
Hash di corpus, normalizzazione, matrice z, indici nearest-neighbor e distanze
sono fissati nel contratto.

La derivazione di riferimento è attestata sul runtime macOS arm64 corrente.
Un audit esteso `k=64` prova che il massimo gruppo di nearest-neighbor a pari
distanza contiene 6 elementi e che `k=8` produce gli stessi indici. Prima di
un execution lock su macOS o Windows, la piattaforma target deve riprodurre
esattamente hash, indici e p95; qualunque divergenza chiude il gate senza
fallback o tolleranza implicita.

Una riga observer-labelled con distanza strettamente maggiore del p95 riceve
peso coverage `100`. La distanza è calcolabile dall'osservazione disponibile
prima dell'azione, quindi non usa informazione futura e non aggiunge feature
all'actor.

## Peso recovery

Sulle sole righe dei pure probe etichettate offline viene calcolato anche il
modifier causale basato su `previous_penetration_m`:

```text
<=10 mm       peso 1
10-15 mm      rampa lineare 1 -> 100
>=15 mm       peso 100
reset         peso 100
OOD > p95     peso 100
```

Il peso finale è il massimo fra reset, recovery e coverage. Ogni caso viene
normalizzato a massa totale esatta `500`: ogni peso raw è moltiplicato per
`500 / somma_pesi_raw_del_caso`. Le righe base e shielded valgono raw `1`, o
`100` se reset. La loss per riga è la MSE media sulle due azioni; la loss del
corpus è `sum(w_i * loss_i) / sum(w_i)`. Così un probe fallito e più corto non
viene diluito solo perché contiene meno righe.

Non sono autorizzati sweep o retuning post-hoc di pesi e soglie.

## Semantica dei probe falliti

Un FAIL puro P0/P1/P2:

- chiude il claim di autonomia di quel candidato;
- non promuove il candidato;
- viene etichettato observer-only;
- può essere seguito esclusivamente dalla raccolta shielded preregistrata per
  acquisire dati correttivi;
- non costituisce retry del probe.

Questa recovery è ammessa soltanto se un gate separato di integrità prova
caso/seed esatti, candidate module e fit receipt, worker claim, trace e replay
payload hash-bound, layout/invarianti e assenza di teacher/blend/latch. Errori
di schema, provenienza, NaN, trace o contratto sono terminali e non possono
aprire né il labeler né la raccolta shielded.

Ogni fit successivo riparte comunque da H0 fresco e usa il corpus cumulativo;
non continua dai pesi del candidato fallito.

Il fit non accetta conteggi o corpus dichiarati liberamente: ogni corpus label
pregresso è incluso nel payload JSON canonico della propria receipt; SHA-256 e
size della receipt vengono ricalcolati e devono impegnare lo stesso record del
corpus, lo stesso conteggio, stage e claim consumati dal fit. Anche i nomi dei
`report_checks` del fitter e le chiavi di ogni receipt sono insiemi esatti
preregistrati. Tutti i payload consumano inoltre lo stesso record del pipeline
claim; H0 è vincolato dal tree SHA-256 autorevole e il teacher dal receipt V10
con hash e size esatti.

Un FAIL puro P3 è terminale. P3 deve superare il probe prima del freeze.

## Raccolta shielded e latch

Le raccolte shielded restano data acquisition, mai evidenza di autonomia. Sono
pubblicati due esiti distinti:

1. `collection_data_gate`: integrità fisica, label same-state, trace e
   provenienza;
2. `latch_dependence_gate`: misura forced takeover e streak.

Il latch-dependence PASS richiede:

```text
forced_teacher_takeover_count = 0
forced_teacher_takeover_fraction = 0
max_consecutive_takeover_steps = 0
latch inactive a fine episodio
```

Un FAIL del secondo gate non rende inutilizzabili label valide, ma vieta ogni
claim di autonomia. L'autonomia è attestata soltanto dai probe puri.

## Ordine congelato degli stage

```text
fit_p0 -> probe_p0 -> label_p0
  -> collect_r1 x2
fit_p1 -> probe_p1 -> label_p1
  -> collect_r2 x2
fit_p2 -> probe_p2 -> label_p2
  -> collect_r3 x2
fit_p3 -> probe_p3 -> label_p3
  -> freeze_p3
  -> final development x6
  -> finalize_development
```

Totale: 26 stage con claim e receipt esclusivi.

L'ordine sopra è il percorso di successo. Un FAIL integro P0-P2 passa prima
obbligatoriamente dal relativo `label_p*`, poi dalla sola collection data-only.
Un FAIL P3 è terminale immediato e non apre `label_p3`.

## Gate fisico del probe

Ogni probe PASS richiede:

- `500/500` step, `5000` control window e `5000` sample detector;
- almeno due cicli validi;
- penetrazione strettamente `<25 mm`;
- terminale normale per time limit;
- zero clipping, NaN, fallback, timeout, invalid event, safety stop, fallback
  SEA e soluzioni SO non accettate;
- V26 attiva, layout `35/84`, morphology a zero;
- zero teacher, blend, latch e update.

P3 congelato deve poi superare sei rollout development autonomi e una futura
qualification indipendente prima di qualunque `checkpoint_zero` o warm-start.

## Governance no-clobber

Il protocol freeze corrente:

- usa soltanto nuovi file V12;
- confina codice, test e output V12 sotto
  `Trajectory Generator/baseline_MLP/validation/`;
- enumera tutte le destinazioni future;
- vieta output dentro gli alberi storici V8-V11 e dentro i dati AB06 protetti;
- congela soltanto lunghezze repo-relative e il budget massimo consentito al
  checkout per restare sotto la soglia prudenziale Windows di 240 caratteri,
  senza incorporare il path assoluto della macchina nel freeze;
- fissa LF tramite `.gitattributes` per i sorgenti hash-bound;
- registra hash di sorgenti, input V11, corpus e failure terminale;
- attesta zero fit, zero reset/step e zero update;
- lascia execution lock, claim e run root assenti.

Il manifest enumera i 26 worker claim, i 26 receipt, i root per-step e tutti
gli artifact noti (`trace`, summary, gate, corpus NPZ). Tutte le scritture di
esecuzione devono essere uguali o discendenti del solo run root V12; gli unici
tre output top-level ammessi sono protocol freeze, design audit ed execution
lock. `.gitattributes` forza LF soltanto sugli artifact testuali e marca NPZ,
NPY, PKL, ZIP e BIN come binari.

Qualunque errore della futura pipeline, dopo l'execution unlock, dovrà
consumare V12 con ledger terminale: nessun retry, rescue o sweep.

## Prossimo gate

Prima dell'esecuzione servono:

1. implementazione additive-only di fitter, labeler e runner V12 dentro
   `Trajectory Generator/baseline_MLP/`;
2. test contract/worker/receipt/path isolation e test post-state idempotenti;
3. design audit numerico no-checkpoint separato;
4. review indipendente degli hash;
5. autorizzazione utente esplicita all'esecuzione one-shot;
6. execution lock no-clobber creato soltanto dopo i punti precedenti.
