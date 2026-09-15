# Audit della compatibilità Windows dal pretraining V26 alla rete unificata

Data: **2026-09-15**  
Instruction check token: **CMC_AGENT_OK_2026**  
Commit osservato: `eda70161ba4f9064c5c3fc0d4f8551ada6446474`  
Modalità dell'audit: **sola lettura**, con tre sub-agenti e verifica incrociata del coordinatore.  
Piattaforma di destinazione: **Windows x86_64, Intel/AMD a 64 bit**.

## 1. Sintesi e verdetto

**L'intera pipeline dal pretraining imitativo V26 allo stato attuale non può essere dichiarata pronta per Windows.**

Il runtime condiviso di training e inferenza presenta già accorgimenti concreti per Windows: ordine di caricamento Torch/OpenSim, inizializzazione dei worker Ray, gestione dei processi figli, percorsi normalizzati e funzionamento su CPU. Nelle modifiche esaminate di FSM v3 e morphology non sono emerse nuove dipendenze POSIX obbligatorie.

Sono invece stati identificati problemi concreti nei runner V26C, nell'attestazione J21, nel coordinatore del piano 50/100, nel generatore differito della variante 39 e nel riutilizzo dei risultati dopo lo spostamento su un'altra macchina. Inoltre, nel workspace e nel bundle osservati mancano le DLL Windows richieste dalla configurazione attuale e non esiste una descrizione completa e bloccata nelle versioni dell'ambiente Python/native.

Il giudizio distingue tre aspetti:

| Aspetto | Verdetto |
|---|---|
| Predisposizione del runtime principale a Windows | Buona nelle parti esaminate; nessun nuovo blocco Windows certo individuato nel core condiviso |
| Esecuzione dell'intera catena V26 → V26C/J21 → unified V1 → fattibilità 39 | Non pronta: presenti incompatibilità concrete e condizioni di trasferimento non gestite |
| Attestazione tramite esecuzione nativa Windows | Mancante per la pipeline corrente; le evidenze live considerate riguardano macOS |

Questa distinzione impedisce sia di dichiarare incompatibile tutto il simulatore, sia di estendere la compatibilità del trainer principale a tutti gli strumenti che lo circondano.

## 2. Richiesta, problema e vincoli

L'utente ha richiesto un audit con sub-agenti per verificare la compatibilità e la readiness Windows di quanto implementato dal pretraining imitativo V26 fino a oggi, imponendo di non scrivere codice e non modificare nulla durante l'audit.

La richiesta successiva `create_report dettagliato` autorizza la creazione di questo documento. Non autorizza implicitamente le correzioni descritte nei TODO.

Il problema da chiarire era se un trasferimento del progetto corrente su Windows permettesse di usare in modo coerente:

- training imitativo e inferenza V26;
- teacher, recovery, DAgger e passaggi V26B/V26C;
- warm-up del critic, restore e attestazione J21;
- FSM v3, cancellazione HS, resync e morphology;
- rete unificata V1, pretraining, resume, valutazioni e report;
- software già preparato per la futura variante a 39 feature;
- checkpoint, manifest e risultati storici necessari a quei passaggi.

Il riferimento operativo è Windows/Python a **64 bit**. La dicitura generica «Windows x86» nelle istruzioni root non costituisce evidenza di supporto per un ambiente a 32 bit, che non viene attestato da questo audit.

## 3. Strategia e perimetro dell'indagine

### 3.1 Suddivisione del lavoro

| Responsabile | Perimetro principale |
|---|---|
| Sub-agente `audit_runtime_windows` | Runtime condiviso baseline, training, rollout, checkpoint V26, watchdog, integrazione con simulatore, FSM e morphology |
| Sub-agente `audit_v26_tooling_windows` | Strumenti V26B/V26C, J20/J21, ricostruzione del bundle, analisi sigma e percorsi lunghi |
| Sub-agente `audit_unified_windows` | Rete unificata, piano 50/100, valutazione/reportistica, bundle differito 39 e trasferimento degli artefatti |
| Coordinatore | Configurazione e plugin, dipendenze, documentazione, attributi Git, verifica incrociata dei difetti e semantica Windows delle API |

Il confronto storico ha usato la sequenza di commit di agosto/settembre e le modifiche dal contesto immediatamente precedente al V26 nativo di metà agosto. Sono stati esaminati anche componenti precedenti quando rimangono dipendenze effettive dei percorsi correnti.

### 3.2 Metodo

Sono stati usati:

- lettura di sorgenti, configurazioni, report, manifest e metadati;
- ricerche mirate di API POSIX, path assoluti, launcher e gestione dei processi;
- controllo del percorso effettivo dai parametri CLI alla chiamata che esegue l'operazione;
- `git status`, cronologia, differenze, inventario dei file e `git check-attr`;
- confronti SHA-256 e simulazioni di fine-riga esclusivamente in memoria;
- rappresentazioni `PureWindowsPath` per controllare i separatori;
- ispezione strutturale di pickle con `pickletools.genops`, senza deserializzare oggetti del progetto;
- consultazione dei sorgenti/documentazione ufficiali Python e Microsoft per la semantica di `os.kill`.

Non sono stati eseguiti training, rollout, compilazioni, test del progetto o import del progetto. Non sono state create cache, modificati checkpoint o inviati segnali ai processi. Il training già presente nel workspace non è stato avviato, fermato o riconfigurato dall'audit.

Questa è una revisione statica con verifiche mirate: non è un'esecuzione Windows emulata o nativa e non garantisce l'assenza di altri problemi nelle combinazioni non esercitate.

## 4. Quadro dei riscontri

La priorità indica l'ordine pratico con cui affrontare un'eventuale preparazione Windows. Un difetto condizionale si manifesta soltanto nel percorso o nella condizione indicati.

| ID | Problema | Tipo e condizione | Impatto |
|---|---|---|---|
| W01 | DLL SEA `_ff` e OnlineGRF assenti | Gap della distribuzione osservata | Caricamento dell'ambiente corrente non disponibile su Windows con i soli binari presenti |
| W02 | `os.kill(pid, 0)` nel coordinatore | Difetto ad alta priorità nel percorso attach a 50 | Controllo del processo non innocuo e non affidabile su Windows |
| W03 | Interprete macOS fissato nei runner J20 | Blocco dei runner interessati | Warm-up/restore dedicati non avviabili così come sono |
| W04 | Separatori Windows contro chiavi `/` in J21 | Difetto dell'attestazione | Artefatti integri possono risultare mancanti, inattesi o non registrati nei pin |
| W05 | Hash del piano sensibili a CRLF | Blocco condizionale del preflight congelato | Rifiuto dei sorgenti dopo checkout con conversione dei fine-riga |
| W06 | Sostituzioni del generatore 39 dipendenti da LF | Blocco condizionale della materializzazione | Errore di source drift con sorgenti CRLF |
| W07 | Percorsi assoluti nelle valutazioni | Difetto di migrazione degli artefatti esistenti | CP50 non riutilizzabile direttamente nel confronto a 100 dopo spostamento |
| W08 | Sigma azzerato se manca il report al vecchio path | Difetto della rianalisi dopo trasferimento | Possibili classificazioni numericamente errate senza errore esplicito |

A questi si aggiungono due prerequisiti non chiusi: gestione dei percorsi lunghi e riproducibilità/verifica nativa dell'ambiente.

## 5. Evidenze dettagliate

### W01 — Binari Windows richiesti non presenti

La configurazione in [config.py](../../config.py), righe 30–32, seleziona il basename `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff`. Per le GRF online il basename è `plugins/OnlineGRFContact`.

Il caricamento effettivo avviene in [model_loader.py](../../model_loader.py): SEA alla riga 837 e plugin GRF alla riga 855 quando la modalità non è `prescribed`. I preset correnti della rete unificata utilizzano `online_sensor`.

L'inventario locale ha trovato le corrispondenti librerie macOS, ma non:

- `SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`;
- `OnlineGRFContact.dll`.

È presente `SEA_Plugin_BlackBox_mCMC_impedence.dll`, che non corrisponde al basename `_ff` richiesto. La sua presenza non dimostra equivalenza con il plugin attivo.

Il [README del bundle V26C](<../../Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/README.md>), righe 57–59, documenta esplicitamente la stessa mancanza. Inoltre [.gitignore](../../.gitignore) esclude ordinariamente `plugins/*`, DLL e dati locali del modello: un checkout Git da solo non equivale all'ambiente runtime completo.

**Conseguenza:** il workspace/bundle osservato non costituisce una distribuzione Windows pronta all'avvio. Non è stata ispezionata la macchina Windows dell'utente e non si afferma che quelle DLL manchino anche lì.

È disponibile una procedura di build Windows per [OnlineGRFContact](../../tools/online_grf_contact/README.md), righe 25–32. La documentazione richiede che plugin e binding Python condividano l'ABI OpenSim/Simbody. Nessuna build o verifica ABI è stata eseguita nell'audit.

### W02 — Il controllo del processo nel coordinatore non è portabile

In [run_authorized_plan.py](<../../Trajectory Generator/unified_MLP/validation/run_authorized_plan.py>), riga 101, `wait_for_existing_training()` esegue `os.kill(int(watchdog["pid"]), 0)` e considera vivo il processo se la chiamata ritorna.

Il percorso è raggiunto con `--attach-training-to50`, alle righe 194–195, mentre la milestone non è già conclusa.

Su POSIX il segnale zero è usato per verificare l'esistenza/accessibilità del processo. Su Windows non si può attribuire alla chiamata quella stessa semantica. Nel sorgente CPython 3.10.20 esaminato, zero coincide con `CTRL_C_EVENT`: viene tentato `GenerateConsoleCtrlEvent`; secondo l'esito, il codice può ritornare oppure proseguire verso apertura/terminazione del processo, con possibile errore.

**Conseguenza:** la routine può interferire con il processo e non costituisce una verifica affidabile della sua vitalità. Il comportamento dipende dalla console, dai permessi e dal processo; non è corretto affermare che ogni invocazione termini sempre il training.

Fonti ufficiali: [implementazione CPython](https://github.com/python/cpython/blob/v3.10.20/Modules/posixmodule.c#L7428), [documentazione `os.kill`](https://docs.python.org/3.10/library/os.html#os.kill), [Microsoft GenerateConsoleCtrlEvent](https://learn.microsoft.com/en-us/windows/console/generateconsolectrlevent).

Il difetto è stato accertato tramite lettura del codice: non è stato riprodotto inviando segnali. Il normale trainer supervisionato e il percorso attach del coordinatore sono componenti distinti; il finding riguarda quest'ultimo.

### W03 — J20 warm-up e restore R3 dipendono da un eseguibile macOS

In [v26c_j20_critic_warmup_readiness.py](<../../Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j20_critic_warmup_readiness.py>), riga 232, `INTERPRETER` è fissato a `/opt/anaconda3/envs/envCMC-rllib/bin/python`.

La funzione `sealed_command()` lo inserisce come primo token, alla riga 365. [v26c_j20_critic_warmup_execution.py](<../../Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j20_critic_warmup_execution.py>) deriva quel comando e lo passa a `subprocess.Popen`, righe 1212–1214.

Anche [v26c_j20_restore_audit_r3.py](<../../Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j20_restore_audit_r3.py>) parte dal comando del warm-up, righe 474–475, e conserva `tokens[0]` nel comando del child, righe 512–517.

**Conseguenza:** questi runner dedicati non avviano un interprete Windows con la configurazione implementata. Non emerge un override CLI dell'interprete nei percorsi considerati.

Il riferimento macOS è operativo, perché raggiunge il lancio del processo. Non è soltanto una stringa da conservare per documentare un esperimento passato. L'ostacolo non si estende automaticamente a ogni warm-up del trainer generale.

Diversi launcher V26B presentano invece un default macOS ma espongono `--python`, per esempio `v26b_r0a_rollout.py`, `v26b_r1_rollout.py`, `v26b_r2i_rollout.py` e `v26b_s0d_rollout.py`. Per questi il default è una condizione di avvio da gestire, non lo stesso blocco senza override trovato in J20.

### W04 — L'attestazione J21 confronta separatori non canonicalizzati

In [v26c_j21_training_ready_attestation.py](<../../Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j21_training_ready_attestation.py>):

- `file_set()`, riga 783, costruisce le chiavi con `str(p.relative_to(root))`;
- `check_leaf_set()`, righe 795–800, confronta esattamente gli insiemi con le mappe archiviate;
- il gate T12, righe 1300–1303, cerca analogamente `str(p.relative_to(HERE))` nelle chiavi dei pin.

Su Windows tali stringhe utilizzano `\`, mentre le chiavi storiche considerate utilizzano `/`. La prova in memoria con `PureWindowsPath` conferma che le due rappresentazioni non coincidono.

Le mappe J19B e J19C considerate contengono rispettivamente 114 e 57 percorsi annidati interessati dalla differenza. Il contenuto dei file può rimanere identico: è la rappresentazione della chiave a cambiare.

**Conseguenza:** i gate possono segnalare file mancanti, inattesi o non registrati nei pin e bloccare l'attestazione di artefatti integri.

Il finding non invalida retroattivamente l'attestazione macOS J21. Evidenzia che il suo verificatore non può essere usato come prova Windows senza risolvere la differenza di rappresentazione.

### W05 — Il preflight congelato fallisce con conversione LF → CRLF

In [run_authorized_plan.py](<../../Trajectory Generator/unified_MLP/validation/run_authorized_plan.py>), righe 64–69, `source_is_preserved()` ricalcola SHA-256 dai byte e li confronta con quelli del preflight.

`git check-attr text eol` ha mostrato attributi non specificati per i file unified controllati. Le regole di [.gitattributes](../../.gitattributes) proteggono diverse sorgenti e archivi storici, ma non stabiliscono la stessa politica LF per il nuovo perimetro unified esaminato.

Il [preflight del piano](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/preflight.json>) contiene 69 voci di sorgenti runtime. Il controllo mirato ha quantificato tutte le 17 voci unified: 13 Python e quattro YAML.

| Controllo sulle 17 voci unified | Risultato |
|---|---:|
| Hash correnti corrispondenti al preflight | 17/17 |
| Hash diversi dopo conversione LF → CRLF simulata in memoria | 17/17 |

**Conseguenza:** riutilizzare il preflight congelato dopo un checkout Windows che converte i fine-riga provoca il rifiuto dei sorgenti, anche senza una modifica logica al programma.

La condizione è la conversione effettiva dei byte. Una copia che preserva LF non presenta questa causa. Il finding riguarda il piano congelato, non dimostra che un normale training nuovo su Windows fallisca per lo stesso motivo.

### W06 — Il generatore differito 39 richiede sorgenti con LF

In [build_variant.py](<../../Trajectory Generator/unified_MLP/validation/feasibility39_bundle/build_variant.py>), righe 113–115, il sorgente viene letto come byte e decodificato UTF-8 senza normalizzazione dei fine-riga.

Le sostituzioni multilinea in `_transform()` richiedono però sequenze LF letterali, per esempio alla riga 54 per `env_config` e alla riga 67 per `_actor_in`. `_replace()`, righe 20–23, solleva `ValueError` se il numero atteso di corrispondenze non è presente.

Il confronto in memoria sull'anchor `env_config` del trainer ha dato:

| Contenuto esaminato | Corrispondenze |
|---|---:|
| Sorgente attuale LF | 1 |
| Stesso sorgente convertito in memoria a CRLF | 0 |

**Conseguenza:** la materializzazione della variante fallisce con sorgenti CRLF. Il problema è indipendente da W05: rigenerare il preflight non renderebbe queste sostituzioni compatibili con CRLF.

L'oggetto dell'audit è il software già preparato nel bundle differito per la fase 5. Non si afferma che la campagna 39 sia stata eseguita o che questo finding abbia fermato il training V1 macOS corrente.

### W07 — Le valutazioni esistenti non sono ricollocabili direttamente

In [evaluate_training_milestone.py](<../../Trajectory Generator/unified_MLP/validation/evaluate_training_milestone.py>):

- righe 139–146: il percorso originale `summary["checkpoint"]` deve esistere per verificare il contenuto;
- riga 300: `plan.run_dir` del report precedente deve coincidere con il percorso assoluto corrente;
- righe 316–320: le trace precedenti vengono cercate usando `source_directory` archiviato.

Il [report JSON della milestone 50](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/milestone_evaluation.json>) contiene il percorso `/Users/tommy/...` del run e nove directory caso assolute. Il commento nel valutatore accenna allo spostamento tra OS, ma il percorso implementato solleva un errore se il vecchio checkpoint non è disponibile.

**Conseguenza:** copiare run e valutazioni su Windows non basta per riutilizzare quelle evidenze nel confronto 50/100. Non esiste nei punti osservati una risoluzione coerente rispetto alla nuova root.

Il problema riguarda anche un trasferimento fra due directory macOS. È un difetto di ricollocazione degli artefatti, non una prova che i pesi siano incompatibili o che una valutazione interamente generata su Windows fallisca.

### W08 — La rianalisi sigma può assegnare zero a configurazioni non recuperate

In [analyze_v26c_sigma_exploration_smoke.py](<../../Trajectory Generator/baseline_MLP/validation/analyze_v26c_sigma_exploration_smoke.py>), righe 103–107, `_actor_configuration()` cerca `exploration_configuration_report.json` vicino al checkpoint indicato dal summary. Se il file non esiste a quel percorso, restituisce `sigma=[0.0, 0.0]`.

Gli artefatti dello sweep del 31 agosto, per esempio il [summary del run isotropico 0.010](<../../Trajectory Generator/baseline_MLP/validation/v26c_sigma_exploration_smoke_2026-08-31/rollouts/iso_0p010_seed123_nominal/rollout_summary.json>), riga 3, contengono un percorso assoluto `/Users/tommy/...`.

La stessa analisi raggruppa poi i run usando il sigma recuperato, righe 379–388.

**Conseguenza:** rianalizzando gli artefatti trasferiti, il vecchio path non disponibile può portare sigma diversi nel gruppo zero, anche quando i report sono stati copiati sotto `actors/`. Il valore mancante viene trasformato in un valore numerico utilizzabile, con rischio di risultati errati invece di un errore esplicito.

Non sono state rigenerate analisi o tabelle durante l'audit. La conseguenza è ricavata dal flusso dei dati nel codice e dai percorsi effettivamente archiviati.

## 6. Prerequisiti Windows non chiusi

### 6.1 Percorsi lunghi

L'inventario Git dell'audit comprendeva 60.222 file tracciati. Il massimo percorso relativo osservato era di 233 caratteri, in un checkpoint J20 annidato fino a `class_and_ctor_args.pkl`.

Per la root esemplificativa `C:\Users\tommy\Documents\CMC-like-Simulator - Claude`, il controllo della lunghezza ha individuato 13 file oltre 259 caratteri assoluti e 26 oltre il budget conservativo di 248. Con `C:\cmc` nessuno dei percorsi tracciati considerati supera quei limiti.

Il [manifest del bundle](<../../Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/bundle_manifest.json>), righe 15–19, registra il massimo relativo di 233, il budget conservativo 248 e una root massima di 14 caratteri senza supporto ai percorsi lunghi. Il relativo README, righe 76–82, descrive root corta o configurazione long-path Windows/Git.

Questo è un requisito operativo condizionale, non la prova di un guasto su qualunque installazione Windows. Il conteggio riguarda i file tracciati esaminati, non ogni percorso che Ray potrebbe generare in futuro.

### 6.2 Ambiente e dipendenze

Il [README baseline](<../../Trajectory Generator/baseline_MLP/README.md>), righe 14–15, descrive OpenSim-mCMC 4.5.2, Python 3.12, NumPy 2, Torch 2.7 e Ray 2.55. La procedura di installazione, righe 27–34, parte da un ambiente locale preesistente e installa `ray[rllib]` senza fissare nel comando una versione precisa.

La [validazione unified](<../../Trajectory Generator/unified_MLP/VALIDATION.md>), righe 136–140, documenta invece esecuzioni macOS con Python 3.10.20, Ray 2.55.1 e Torch 2.10.0. Il [README del bundle](<../../Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/README.md>), righe 66–67, dichiara che manca un lock completo delle dipendenze Python e native.

La differenza tra versioni documentate non dimostra da sola un'incompatibilità. Significa però che la ricostruzione dell'ambiente Windows e il restore dei checkpoint non sono determinati completamente dai materiali disponibili.

### 6.3 Verifica nativa Windows

La validazione unified citata dichiara esplicitamente che le esecuzioni dinamiche documentate riguardano macOS. Il bundle J21 attesta una specifica lineage con lo scope `TRAINING_INPUT_ONLY`: non costituisce una certificazione di funzionamento su Windows.

Una verifica nativa dovrà coprire almeno caricamento OpenSim/plugin, worker Ray, training breve, rollout, salvataggio/restore, warm-up e trasferimento degli artefatti effettivamente scelti. Questa verifica non è stata effettuata nell'audit e non viene sostituita dai controlli statici.

## 7. Evidenze favorevoli alla portabilità

### 7.1 Runtime condiviso

| Componente | Evidenza osservata |
|---|---|
| Import Torch/OpenSim | [win_runtime.py](<../../Trajectory Generator/baseline_MLP/win_runtime.py>), righe 33–74: prepara il PATH Windows, importa Torch e ripristina le directory OpenSim |
| Worker Ray | [train_ppo_mlp.py baseline](<../../Trajectory Generator/baseline_MLP/train_ppo_mlp.py>), righe 636 e 1785–1786: bootstrap e hook di preparazione dei worker |
| Avvio dei processi | Trainer baseline, righe 3078–3088, e [process_watchdog.py](<../../Trajectory Generator/baseline_MLP/process_watchdog.py>), righe 277–284: `sys.executable`, argomenti separati e ramo Windows per il gruppo di processi |
| Percorsi | [path_resolver.py](../../path_resolver.py), righe 21–35 e 54–56: normalizzazione e risoluzione rispetto al repository |
| Dispositivo | Trainer baseline, righe 1504 e 1797: `num_gpus=0` nel percorso esaminato |
| Encoding e console | Aperture testo del runtime baseline osservato esplicitamente UTF-8; [progress_display.py](<../../Trajectory Generator/baseline_MLP/progress_display.py>) gestisce console/encoding e fallback ASCII |
| Unified | Bootstrap da `__file__`, shim condiviso, `sys.executable`, hook Ray e preset con risorse relative |

Questi accorgimenti sono implementati. La loro presenza è evidenza favorevole, ma non dimostra che tutte le combinazioni di DLL, Python, Torch e Ray su Windows siano già state provate.

### 7.2 FSM e morphology

Sono state esaminate le modifiche e le integrazioni di FSM v3, cancellazione HS, resync e corridoio causale. Nei punti considerati la logica è Python/NumPy e non introduce nuove dipendenze POSIX obbligatorie. Le tolleranze temporali, incluso `1e-9`, sono esplicite.

Non è stata verificata la parità numerica dei rollout fra sistemi operativi. Il mancato superamento dello screening di cammino a 50 iterazioni, discusso in altri report, è un problema di qualità della policy distinto dalla compatibilità Windows.

### 7.3 Strumenti storici e bundle

Sono presenti accorgimenti multipiattaforma anche in parte degli strumenti storici:

- F0 prevede interpreti Windows e normalizzazione dei percorsi;
- `f0_artifacts.py` contiene gestione Windows per lettura binaria e rename;
- `f2r_refit.py` gestisce un ramo Windows per il commit delle directory;
- restore R3 compone `PYTHONPATH` con `os.pathsep`;
- il bundle usa oggetti con nomi corti, percorsi relativi canonicalizzati e controlli del budget Windows.

Il bundle è dichiarato come archivio di evidenze, non come runtime alternativo. Le stringhe assolute conservate esclusivamente come provenance non sono state classificate automaticamente come difetti: W03, W07 e W08 riguardano stringhe che il codice usa realmente per eseguire o aprire risorse.

## 8. Checkpoint: cosa è stato verificato e cosa no

Il sub-agente del runtime ha ispezionato il best del run canonico [MLP_imitation_native_v26_08-20-2026_june_equiv_100iter](<../../Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/>):

- `checkpoint_best`: 16 pickle, 2.557.509 byte complessivi;
- `rl_module_best`: 2 pickle, 615.265 byte complessivi;
- metadata Ray 2.55.1, formato checkpoint 2.1;
- risorse setup/profili nel YAML canonico espresse con percorsi relativi.

La scansione con `pickletools.genops` non ha rilevato i marker cercati di `PosixPath`/`WindowsPath`, percorsi `/Users/` o Anaconda, dispositivi MPS/CUDA o ricostruzioni cloudpickle di classi/funzioni per valore. Sono emersi stato numerico NumPy e riferimenti alla classe importabile `asymmetric_rl_module`.

Un controllo aggiuntivo del sub-agente unified sui pickle CP50 fino a 2 MB non ha rilevato marker `PosixPath` o percorsi macOS nei file considerati.

**Limite:** si tratta di ispezioni strutturali e ricerche mirate, senza deserializzazione né restore. Non certificano tutti i checkpoint, tutte le dipendenze importabili o l'equivalenza numerica fra Windows e macOS. Non è stato identificato un blocco universale alla portabilità dei pesi; i difetti di migrazione delle valutazioni non devono essere estesi automaticamente alla serializzazione della rete.

## 9. Osservazioni minori non elevate a blocchi principali

- Il fallback di terminazione in `process_watchdog.py`, riga 192, e nel trainer baseline, riga 2869, assume `C:\Windows\System32\taskkill.exe`. È una limitazione condizionale per installazioni Windows altrove, quando il percorso alternativo tramite Job Object non è disponibile.
- Alcune aperture testo in `output.py` usano l'encoding implicito. Nei contenuti STO/CSV correnti esaminati prevalgono dati numerici e intestazioni ASCII: non è stato dimostrato un guasto attuale.
- I metadati delle milestone baseline includono percorsi assoluti. Non è stato dimostrato che ciò blocchi il normale restore; il finding certo riguarda i consumatori espliciti descritti in W07.
- Alcuni strumenti forensi richiamano directory temporanee storiche ma offrono parametri alternativi. Non sono stati presentati come impedimenti al normale training.

Questi punti restano informazioni utili per un audit successivo, con priorità inferiore ai difetti concretamente collegati a un percorso operativo.

## 10. Soluzione e strategia risultante

La soluzione consegnata in questa attività è una diagnosi documentata, con condizioni e riferimenti verificabili. **Nessuna correzione software è stata applicata.**

Per arrivare in futuro a una dichiarazione di readiness Windows, la sequenza coerente con le evidenze è:

1. rendere determinato l'ambiente Windows e disporre delle DLL corrette;
2. correggere i percorsi operativi incompatibili, iniziando dal controllo dei processi e dai runner/validatori necessari;
3. distinguere trasferimento dei pesi, trasferimento dei risultati e riproduzione storica, verificando ciascun contratto;
4. controllare fine-riga e lunghezza dei percorsi del checkout reale;
5. eseguire verifiche native brevi e poi verificare il workflow completo scelto;
6. registrare risultati, versioni e limiti della verifica senza estendere il verdetto a percorsi non esercitati.

Le correzioni dei runner e della canonicalizzazione dovranno conservare gli artefatti storici e i loro hash. L'obiettivo non è riscrivere retroattivamente le evidenze per far passare i controlli, ma rendere espliciti i percorsi e i contratti usati nella nuova esecuzione.

Questa strategia non implica una nuova autorizzazione a modificare il plugin C++, la semantica SEA o i sorgenti congelati.

## 11. Test e verifiche effettivamente eseguiti

| Verifica | Metodo | Esito/limite |
|---|---|---|
| Stato iniziale/finale dell'audit | `git status` e controllo diff/untracked | Pulito; nessuna modifica introdotta dall'audit |
| Scope storico | Cronologia e diff dei sorgenti/configurazioni | Individuati runtime e strumenti introdotti o cambiati nel periodo |
| Binari richiesti | Inventario `plugins/`, configurazione e manifest | DLL `_ff` e OnlineGRF assenti nel workspace/bundle osservati |
| Fine-riga Git | `git check-attr` | Nessuna politica LF specificata per i file unified controllati |
| Hash CRLF | Trasformazione dei byte esclusivamente in memoria | 17/17 voci unified cambiano hash |
| Anchor generatore 39 | Conteggio di una sostituzione sul testo LF/CRLF in memoria | 1 corrispondenza LF, 0 CRLF |
| Separatori J21 | `PureWindowsPath` e confronto delle chiavi | Rappresentazioni Windows diverse dalle chiavi POSIX archiviate |
| Migrazione CP50 | Lettura JSON e tracciamento dei consumatori | Percorso run e nove directory caso assoluti macOS |
| Analisi sigma | Lettura del fallback e delle sorgenti effettive | Dato non reperito convertito in sigma zero e usato nel raggruppamento |
| Percorsi lunghi | Lunghezza dei percorsi tracciati | Massimo relativo 233; rischio dipendente dalla root/supporto long-path |
| Checkpoint V26/CP50 | Ispezione strutturale senza deserializzazione | Nessun marker incompatibile fra quelli cercati; restore non verificato |
| API Windows del controllo processi | Sorgente CPython e documentazione Microsoft | `os.kill(pid, 0)` non utilizzabile come equivalente innocuo del probe POSIX |
| Test del progetto e verifica nativa Windows | Non eseguiti | Nessun nuovo risultato live attribuito a Windows |

I numeri di test passati presenti nei report precedenti non sono stati rieseguiti o presentati come nuovi risultati dell'audit.

## 12. File modificati

Durante l'audit: **nessuno**.

Con il successivo comando `create_report dettagliato` è stato creato soltanto:

- `reports/user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md`.

Non sono stati modificati codice Python/C++, configurazioni, attributi Git, checkpoint, artefatti di validazione, stato del piano o daily report. La creazione del documento non modifica il verdetto né chiude i TODO tecnici.

## 13. TODO aperti da propagare nel prossimo daily

Le attività seguenti derivano dall'audit. Sono da riportare nel prossimo `end_day` e propagare finché non vengono risolte. La loro registrazione non avvia automaticamente correzioni o campagne sperimentali.

- [ ] **W01:** verificare sulla destinazione Windows la disponibilità delle DLL SEA `_ff` e OnlineGRF corrette, la provenienza e l'ABI con OpenSim/Simbody/Python; completare la distribuzione necessaria.
- [ ] **W02:** sostituire il controllo `os.kill(pid, 0)` del percorso attach con una verifica di vitalità non distruttiva e validarla su Windows.
- [ ] **W03:** predisporre un percorso Windows per i runner J20 warm-up e restore R3, conservando la provenance storica e rendendo esplicito l'interprete della nuova esecuzione.
- [ ] **W04:** canonicalizzare le chiavi dei percorsi nell'attestazione J21 e verificare su Windows i gate T12/T18 con artefatti integri e casi negativi.
- [ ] **W05:** definire una politica dei fine-riga coerente con gli hash dei sorgenti unified e verificarla su un checkout Windows reale senza riscrivere i pin storici per nascondere differenze.
- [ ] **W06:** rendere il generatore differito 39 compatibile con LF e CRLF e verificarne la materializzazione rispettando la separazione dalla V1.
- [ ] **W07:** definire e verificare la ricollocazione di run, checkpoint e trace per il riuso delle valutazioni CP50/CP100 mantenendo le verifiche di identità del contenuto.
- [ ] **W08:** distinguere nell'analisi sigma un report mancante da un sigma realmente nullo e verificare la rianalisi degli artefatti trasferiti.
- [ ] Documentare e verificare root di checkout, supporto long-path Windows/Git e percorsi generati dal workflow selezionato.
- [ ] Fissare un ambiente Windows riproducibile con versioni Python/Ray/Torch/NumPy/OpenSim e dipendenze native, compatibile con i checkpoint scelti.
- [ ] Eseguire una verifica nativa Windows di caricamento plugin, reset/step, training breve con worker Ray, rollout, salvataggio/restore e warm-up, registrando gli esiti separatamente dalle evidenze macOS.
- [ ] Verificare su Windows orchestrazione, valutazioni/report e, quando prevista, materializzazione della variante 39; attestare soltanto i percorsi realmente provati.
- [ ] Conservare immutati gli artefatti storici e distinguere esplicitamente portabilità dei pesi, migrazione dei risultati e riproduzione dell'esperimento.

## 14. Riferimenti di contesto

- [Daily del 15 settembre](../daily/2026-09-15_daily-report.md).
- [Valutazione intermedia V1 a 50 iterazioni](2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).
- [Guida di ricostruzione V26 → V26C](2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [Rete unificata V1 e protocollo di parità](2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [README unified](<../../Trajectory Generator/unified_MLP/README.md>) e [VALIDATION](<../../Trajectory Generator/unified_MLP/VALIDATION.md>).
- [README del bundle V26C](<../../Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/README.md>).

I collegamenti a `runs/` e ad altri dati locali identificano le evidenze consultate nel workspace. Alcuni di questi artefatti sono esclusi dal normale checkout Git: la presenza del link nel report non implica che il relativo file sia distribuito dal repository.
