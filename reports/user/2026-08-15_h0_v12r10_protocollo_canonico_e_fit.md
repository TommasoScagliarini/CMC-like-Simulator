# H0 V12R10: protocollo canonico, fit W1024 e terminal FAIL

Data: 2026-08-15

## Obiettivo

Trasformare l'esito diagnostico V12R10 in una lineage production congelata,
one-shot e fail-closed, capace di recuperare le label valide di V12R9 senza
riaprire la sua esecuzione terminale. La milestone deve preparare i sei
development pure-policy richiesti prima di Q3 e delle successive qualifiche
checkpoint-zero, Morphology Corridor e training.

## Problema di partenza

V12R9 ha prodotto un corpus valido di 11.875 righe, comprendente 2.431 label
observer, ma il fit W512 ha terminato in FAIL e il relativo candidato non puo
essere promosso o ritentato.

La forense V12R10 ha inoltre identificato un rischio semantico distinto dal
semplice errore di ottimizzazione: nei picchi peggiori il teacher legacy cambia
stato per timeout mentre il detector/FSM V26 continua correttamente a riportare
`STANCE`. Nel corpus non esistono label discordanti per osservazioni
byte-identiche, quindi non e dimostrata un'impossibilita matematica sulle 35
feature; resta pero un alias path-dependent dovuto a history nascosta del
`LegacyGaitShadow`. L'audit ha contato 232 righe teacher in `TIMEOUT` contro
zero righe V26 nello stesso stato e 107 cliff di label senza transizione o
pulse V26 adiacente.

Il dry-fit W1024 gate-aligned ha mostrato che i gate offline possono essere
chiusi senza allentare le soglie, ma non risolve da solo il rischio scientifico
dell'alias. Per questo motivo l'idoneita fisica resta subordinata ai sei
development e a Q3, entrambi senza teacher e fail-closed sulla route V26.

## Soluzione adottata

E stata implementata una lineage canonica V12R10 **import-only**, con actor
standard W1024 e obiettivo gate-aligned:

- importa e attesta corpus, label e prove terminali congelate di V12R9;
- non esegue nuove raccolte, nuovi relabel o query teacher;
- mantiene R6 come base funzionale congelata e tratta il materiale terminale
  R9 soltanto come inizializzazione/evidenza bloccata, mai come predecessore
  promosso;
- consente un solo fit actor, senza retry, resume, sweep o best-state
  selection;
- pubblica candidato, receipt e development con semantica no-clobber;
- vieta qualsiasi dipendenza dal legacy shadow nel runtime.

La strategia e conservativa: il target imitativo corrente non viene cambiato
prima di avere evidenza fisica sul comportamento di H0 alimentato direttamente
dalla V26. Il primo sviluppo fisico ha poi fallito; questa lineage e quindi
chiusa e il successore vincolante e un relabel stateless e
byte-deterministico V26-puro, non l'introduzione di uno shadow legacy online.

## Protocollo production

Il namespace production separa responsabilita e invarianti:

- il **contract** definisce identita, input congelati, topologia W1024,
  autorita, limiti e ordine dei dieci stage;
- il **fitter** porta autonomamente l'algoritmo gate-aligned validato, esegue il
  solo fit consentito e verifica struttura, metriche, save/reload e trapianto
  warm-start con critic preservato;
- il **freezer** costruisce e verifica protocol freeze ed execution lock dalla
  chiusura completa di sorgenti, runtime e input;
- il **runner** orchestra attestazione R9, fit, freeze del candidato, sei
  development hardest-first e receipt terminale, mantenendo contatori espliciti
  di reset, step, update e query.

L'ordine congelato e: attestazione import R9, fit, freeze del candidato,
development `+0.20`, `-0.20`, nominale, seed 126, seed 127, seed 128 e
finalizzazione. Q3, checkpoint-zero e Morphology Corridor non fanno parte di
questa esecuzione: il FAIL del primo development li ha mantenuti chiusi.

## Finding sull'execution lock e correzione

L'audit ha individuato un difetto nella verifica post-avvio: ricostruire
l'execution lock usando l'occupazione corrente del namespace rendeva
inevitabilmente diverso il payload immutabile non appena il runner creava run
root o claim. Un lock pubblicato correttamente sarebbe quindi diventato
inverificabile durante la stessa esecuzione.

La correzione separa ora due concetti:

- l'occupazione **storica pristine**, tutta `true`, viene attestata e conservata
  nel lock al momento della pubblicazione;
- l'occupazione **corrente** viene controllata soltanto quando
  `require_pristine=True`, cioe prima di iniziare la pipeline.

La verifica ordinaria post-avvio usa l'evidenza storica immutabile senza
riscriverla; il controllo pre-run continua invece a fallire se run root, claim,
ledger o attestazione import sono gia presenti. Sono stati aggiunti test sia
per la verifica dopo l'occupazione del namespace sia per la mutazione di ogni
campo atteso.

## Freeze e lock pubblicati

Il protocollo e stato pubblicato con write esclusiva e verificato semanticamente:

- protocol freeze: SHA-256
  `61d107dfc7bea5af6bdd3d27fe14bdb9a0f0c5eb875ae580555b9139924a266f`,
  50.115 byte;
- execution lock: SHA-256
  `e6601a67d251f19586f25f834df09957450a78fac18ace5d620902314888fb82`,
  44.027 byte.

Entrambi chiudono esattamente i sorgenti production, gli input R6/R9/H0, il
corpus, le label observer, le prove diagnostiche e la readiness del runtime.

## Stato dell'esecuzione canonica

Il preflight della pipeline canonica e PASS. Il singolo fit production W1024 e
stato completato e il candidato e stato congelato correttamente:

- candidate ID
  `AB06_H0_V12R10_RECOVERY_W1024:dcae7d71fa4e246dba16b7078fd74a78a5df4e6330e4230225ffc23c767e1113`;
- tree SHA-256
  `dcae7d71fa4e246dba16b7078fd74a78a5df4e6330e4230225ffc23c767e1113`;
- candidate freeze receipt SHA-256
  `4c8750f124afa2c2e1305917d13bf98404c5b5bf06cc6cd81c9df59aa2b3ac0b`,
  3.777 byte.

La qualification e terminata in **FAIL al primo development** hardest-first,
offset deterministico `+0.20`. Il rollout pure-policy e arrivato a 212 step e
2.120 campioni raw sensor, poi si e chiuso con end reason
`grf_penetration`. L'audit della trace ha rifiutato il caso e il runner ha
pubblicato lo stato terminale
`FAIL_H0_V12R10_RECOVERY_PIPELINE_TERMINAL`, con next stage
`STOP_TERMINAL_NO_RETRY`.

Il ledger terminale e stato validato semanticamente e ha SHA-256
`cf50e9450e29abbb8ef9ce759b825a6d5e09905fb6b62d6f2161feed3f6f1cb1`,
7.429 byte. Sono stati completati esattamente tre stage: attestazione import
R9, fit actor e freeze del candidato. Il quarto stage e stato soltanto
tentato; `development_count` resta zero perche non esiste un development
completato e accettato.

I contatori terminali confermano una sola esecuzione del fit e un solo reset
fisico, con:

- zero query teacher e zero dipendenze teacher nelle azioni servite;
- zero blend;
- zero interventi safety, attivazioni latch o rilasci latch;
- zero update critic e zero update PPO;
- zero Q3 aperti, zero checkpoint-zero e Morphology Corridor non abilitato;
- nessuna promozione runtime, nessun retry e nessun resume autorizzato.

V12R10 e quindi terminalmente **non training-ready**. Q3, checkpoint-zero,
validazione Morphology Corridor e preflight training su 12 EnvRunner non sono
stati aperti e non possono essere attribuiti a questo candidato.

## Forense post-terminale e decisione V12R11

La ricostruzione offline del journal completo ha separato il difetto fisico
dall'eccezione del runner. L'eccezione immediata nasce perche il
`pure_policy_trace_audit` richiede esattamente 500 righe e riceve il prefisso
terminale di 212 righe; rivalutato come prefisso, lo stesso journal rispetta
schema, identita, percorso mean+noise, dieci campioni V26 per step, assenza di
payload teacher e contatori online tutti a zero.

Il trigger reale resta la GRF:

- step 211: penetrazione `0.024668982443890087 m`;
- step 212: penetrazione `0.026729949134248383 m`;
- incremento terminale: `2.061 mm` in un policy step;
- rampa monotona step 202--212: `12.793 -> 26.730 mm` in `0.10 s`;
- quattro clipping dell'azione 0 agli step 207--210, con massimo raw
  `1.06784`.

Detector/FSM V26, routing, finitezza, static optimization e SEA restano
regolari. R10 e vivo allo step 179 con `18.338 mm`, dove R6 era gia terminato,
e quindi estende l'orizzonte di 33 step; non chiude pero il medesimo rischio
fisico nella stance successiva all'HS. Il clipping costituisce inoltre un gate
indipendente fallito.

La diagnosi riproducibile e conservata sotto `validation/v12r11/diagnostics/`:

- script SHA-256
  `5d01c07618de9ab136f389a47317f5a3298932849b601db9e0a589b20fbc0392`;
- result SHA-256
  `0c8505ae1c17bc0338d67be6b6aa01dc4dfbee8306a8f9eba3b734e421c18801`.

L'audit architetturale ha poi chiuso il bivio del successore: la proiezione
stateless V26-pura e l'identita `P(x)=x`, quindi il source H0 W256 implementa
gia esattamente `H0(P(x))`. Un nuovo fit W1024 introdurrebbe soltanto errore di
approssimazione del nuovo target. V12R11 deve pertanto eseguire prima un probe
zero-fit dell'H0 originale sulla route V26, caso `+0.20` hardest-first, con i
gate indipendenti 500 step, almeno due cicli, penetrazione `<0.025 m`, zero
clipping e zero dipendenza teacher/legacy. Un suo FAIL vietera altri fit verso
lo stesso H0 e aprira invece una raccolta recovery V26-only in una nuova
lineage.

## File aggiunti o modificati

- `Trajectory Generator/baseline_MLP/validation/v12r10/__init__.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/h0_v12r10_recovery_contract.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/h0_v12r10_recovery_fitter.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/freeze_h0_v12r10_recovery.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/run_h0_v12r10_recovery.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/test_h0_v12r10_recovery_contract.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/test_h0_v12r10_recovery_fitter.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/test_freeze_h0_v12r10_recovery.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/test_run_h0_v12r10_recovery.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/h0_v12r10_recovery_protocol_freeze.json`;
- `Trajectory Generator/baseline_MLP/validation/v12r10/h0_v12r10_recovery_execution_lock.json`;
- `Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/diagnose_h0_v12r10_terminal_fail.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/results/h0_v12r10_terminal_fail_diagnosis.json`.

Gli artefatti terminali del fit e del development tentato risiedono nel run
root canonico `h0_v12r10_run_20260815/` e non sono stati modificati durante la
redazione di questo report.

## Test e verifiche eseguiti

- 64 test focalizzati V12R10: PASS;
- compilazione/import dei moduli production: PASS;
- Ruff lint e format-check: PASS;
- `git diff --check`: PASS;
- audit indipendente della chiusura sorgenti e delle interfacce fra contract,
  fitter, freezer e runner: PASS;
- assenza di import production dai moduli diagnostici: PASS;
- invarianti one-shot, no-clobber, full-tree, critic preservato e assenza di
  update PPO: coperte dai test e dall'audit;
- protocol freeze ed execution lock verificati dopo pubblicazione: PASS;
- preflight della pipeline canonica: PASS;
- candidate freeze e candidate ID verificati dal ledger: PASS;
- ledger terminale riletto e verificato semanticamente: PASS;
- primo development `+0.20`: FAIL dopo 212 step per `grf_penetration`.
- diagnosi offline del journal R10 e confronto R6, ricostruiti due volte in
  modo deterministico: PASS;
- Ruff, format-check e compilazione della diagnosi V12R11: PASS.

## TODO vincolanti

- Conservare l'esito dell'audit di osservabilita come rischio e gate esplicito
  della qualification; l'audit e concluso, ma l'alias non e dichiarato risolto
  dal solo fit offline.
- Conservare V12R10 chiuso al suo terminal FAIL, senza retry, resume,
  promozione o completamento dei cinque development successivi.
- Definire e congelare in un namespace successore il relabel stateless e
  byte-deterministico dell'esatta osservazione V26-35; non introdurre uno
  shadow legacy online.
- Eseguire come primo discriminatore V12R11 il source H0 W256 direttamente
  sulla observation V26-35, senza fit, widening o distillazione.
- Materializzare le label V26-pure per l'audit offline, pubblicare soltanto se
  eleggibile la copia core byte-exact dell'H0 W256 e rieseguire da zero le
  qualification fisiche fail-closed, senza usare un nuovo fit o riutilizzare
  il candidato V12R10 come predecessore promosso.
- Non ritentare, modificare o promuovere V12R9.
- Mantenere Q3, checkpoint-zero e Morphology Corridor chiusi finche il nuovo
  successore V26-puro non ottiene un terminal PASS fisico completo.
- Soltanto dopo tale PASS, eseguire in ordine Q3, checkpoint-zero e la
  qualification del Morphology Corridor sul candidato successore congelato,
  mantenendo tutte le superfici fail-closed.
- Solo dopo tali gate, congelare il protocollo training con warm-start
  imitativo, detector binario e corridor attivi; completare il restore/preflight
  su 12 EnvRunner prima di avviare e monitorare le 50 update previste.
- Non dichiarare il sistema training-ready finche development, Q3,
  checkpoint-zero, Morphology Corridor e preflight training non sono tutti
  terminal PASS sullo stesso candidato successore congelato.
