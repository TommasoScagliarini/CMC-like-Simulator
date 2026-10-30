# Audit artefatti, dataset e documentazione — 2026-10-14

Analisi sola lettura del working tree. Nessun file della repository modificato o cancellato. Inventario filesystem completo esclusa .git; verifiche mirate sui contenuti e sugli hash. I valori seguenti sono byte logici espressi in GB decimali, non spazio allocato `du`.

## Inventario

153.445 file presenti, 44.770.244.523 byte. 65.094 file presenti tracciati (13.649.208.129 B), 88.351 ignorati (31.121.036.394 B), zero file non tracciati e non ignorati. L'indice Git contiene inoltre 31 file Prosthesis_SNN gia cancellati prima dell'audit.

| Perimetro | GB totali | GB tracciati | GB ignorati | Valutazione |
| --- | ---: | ---: | ---: | --- |
| Trajectory Generator | 30,664 | 7,280 | 23,384 | Codice, archivi scientifici e run; selezione per lineage |
| Trajectory Generator/unified_MLP_39 | 11,797 | 0,850 | 10,947 | Run ignorati + audit tracciati |
| Trajectory Generator/runs | 9,881 | 0,001 | 9,880 | Checkpoint/output ignorati; registro storico tracciato |
| Trajectory Generator/baseline_MLP | 4,814 | 4,791 | 0,024 | Quasi tutto evidenza tracciata |
| Trajectory Generator/unified_MLP | 2,533 | 0,001 | 2,532 | Linea precedente + run ignorati |
| Trajectory Generator/validation | 1,637 | 1,636 | 0,0005 | Campagne recenti, reset e qualifica |
| validation root | 6,627 | 6,348 | 0,280 | Protocollo, test, dataset, evidenze; no rm globale |
| models | 5,830 | 0,005 | 5,824 | Quasi tutti i dati locali non protetti dal commit |
| results | 0,892 | 0 | 0,892 | Tutti gli output ignorati |
| plot | 0,649 | 0,00008 | 0,649 | Solo plotter.py e tracciato |
| reports | 0,0076 | 0,0076 | 0,000008 | Conservare report e TODO; risparmio disco trascurabile |

Le righe Trajectory Generator e sue sotto-directory si sovrappongono e non vanno sommate.

## Candidati a rimozione tecnica a basso rischio

- 1.993 cache incidentali (__pycache__, .pytest_cache, .ruff_cache, .DS_Store): 48.279.701 B, tutti ignorati.
- opensim.log root: 78.159.231 B, ignorato.
- build/ root, build_online_grf/ e build/ dei due tools C++: 9.185.879 B, 161 file, di cui 81 tracciati. Contengono cache/output CMake, progetti Visual Studio, .obj, .tlog, reply error vscode; sorgenti dei tools stanno fuori da queste build.
- paper/main.aux, paper/main.fdb_latexmk, paper/main.fls, paper/main.log, paper/main.synctex.gz: 98.046 B, 5 file tracciati, rigenerabili dal LaTeX.

Totale tecnico preliminare: 135.722.857 B (~136 MB). Non comprende directory denominate build dentro workspace scientifiche: validation/ab07_detector_investigation_2026_10_06/stage3/build, ad esempio, contiene modelli/dati di evidenza e non una semplice cache CMake. Prima di un'eventuale rimozione, verificare i manifest su ogni file tracciato. 20 file di cache matplotlib aggiungono 4.555.186 B, 19 tracciati, ma non sono stati promossi a rimozione immediata perche in campagne di audit.

.gitignore gia ignora build dei tools e vari prodotti, ma non interamente /build/ e /build_online_grf/, che fanno entrare nell'indice molti prodotti Windows e query/reply IDE. Una successiva manutenzione puo ignorare precisamente queste due directory e i prodotti LaTeX, senza regole generiche che tocchino i workspace congelati.

## File fondamentali ignorati

- models/AB06_SEASEA_Threadmill/data: circa 113 MB, contiene IK, GRF, healthy, CMC tasks/actuators e ExternalForces.
- models/AB07_SEASEA_Threadmill/data: circa 65 MB, include runtime_profile_association.json e fitting/ground_registration.json oltre IK/GRF/healthy.
- models/AB08_SEASEA_Threadmill/data: circa 65 MB, include fitting e IK/GRF/healthy.
- plugins/: 2,61 MB, include binari .dll/.dylib SEA e OnlineGRF e tre varianti backup del SEA. Non considerarli rigenerabili prima di verificare corrispondenza sorgente-build-ABI; sono esclusi dal commit.
- models/*-raw: MAT e conversioni originali AB06/AB07/AB08, oltre derivati SEASEA, quasi tutto il volume models. Derivati potenzialmente riproducibili, input grezzi e provenance da archiviare esternamente.
- Checkpoint/ray state e rollout in tre directory runs: essenziali per continuare o confrontare lineage; .gitignore li dichiara reproducible, ma un training stocastico non garantisce gli stessi pesi byte per byte.

## Archivi ed evidenze da preservare

Il README baseline_MLP/validation dichiara v26c_july_replica_2026-08-26 sigillata: J20/J21 verificano percorsi e SHA-256. Non spostare o riscrivere runner, test, trace, receipt, checkpoint o dataset come cleanup ordinaria.

pipeline_bundle_v26c_2026-08-28 contiene ~247 MiB in objects/sha256, archivio intenzionale della closure V26C con runtime, asset AB06, plugin macOS, parent e precedenti. in_place_manifest censisce ~2,6 GB senza duplicarli. Gli originali restano ancore per import/pickle/pin. Le sole copie nel bundle non chiudono tutti i run ignorati della repo, non includono equivalenti Windows mancanti e non sono un runtime alternativo.

Conservare reports/user (441 file), reports/daily (106 file) e reports/plans (21 file): totale documentale 7,60 MB. Daily 2026-10-13 mantiene 276 TODO; i duplicati/stale possono essere riconciliati come lavoro documentale successivo, preservando storia ed hash degli input congelati. paper/aux e DRAFT sono esplicitamente traccia di ragionamento secondo paper_progress.md; paper/main.tex e sections/*.tex sono il manoscritto attivo.

## Duplicati

50 gruppi di blob Git tracciati >1 MB contengono 2.694.924.815 B di copie teoricamente ridondanti nel working tree. Git gia conserva una sola copia di ogni identico blob, quindi non e una riduzione equivalente del remoto.

Maggiori: GRF AB07 ripetuta 24 volte (846 MB ridondanti), TRC AB07 ripetuta 23 volte (829 MB), episodes_pre_gae.pkl identico nei rami control/corrected_boundary (271,5 MB ridondanti). SHA-256 dei primi/ultimi file dei cinque gruppi maggiori conferma identita byte. Sono soprattutto input copiati per studi indipendenti e campioni di audit: non eliminabili automaticamente; una futura deduplicazione richiede nuovo contratto di archivio/restore e verifica dei consumer.

9 gruppi Python byte-identici: solo 102.302 B di duplicazione, quasi tutta in snapshot storici o supervisor. Le due _bootstrap.py operative unified_MLP e unified_MLP_39 sono identiche (624 B), come i due __init__.py (68 B). Nessun vantaggio materiale da rm basato solo sull'identita dei sorgenti.

## Deliverable temporanei

file_inventory.csv: ogni file, byte, stato Git, gruppo e candidato preliminare.
space_summary.json: riepiloghi per gruppo e cache.
python_exact_duplicates.json: SHA-256 e path dei duplicati Python.
tracked_large_blob_duplicates.json: duplicati grandi dell'indice Git con path e byte.

La verifica commit/remoto e la ricostruzione delle dipendenze runtime sono svolte separatamente dall'agente principale. Questo audit non afferma che ogni ignored sia recuperabile o che ogni file di validation sia da eliminare.

## Leggibilita del backlog e documenti

Analisi testuale dei checkbox, con righe continuate normalizzate per whitespace: 5.672 voci TODO presenti nei 106 daily, 1.227 testi esatti distinti; nei user report 680 voci e 625 testi distinti. La ripetizione tra daily e in gran parte la propagazione espressamente richiesta da AGENTS.md, non prova che le voci siano obsolete.

Il daily del 13 ottobre ha esattamente 276 voci e 276 testi distinti: nessun duplicato letterale dopo la normalizzazione del whitespace. Ci sono famiglie semanticamente sovrapposte ma formulate diversamente; 78 testi coincidono con almeno un TODO dei user report. Dunque una deduplicazione automatica per stringa non risolve il sovraccarico e non autorizza la chiusura dei TODO.

Proposta documentale successiva: un indice breve degli entrypoint attivi e un registro di TODO con ID stabili, stato, fonte e criteri di chiusura; revisione esplicita delle famiglie ripetute per stabilire equivalenze. Conservare i daily/user storici e i pin; mantenere anche la propagazione nei nuovi daily finche la regola AGENTS resta vigente. Scegliere la lettura mirata degli ultimi report e del registro per l'avvio quotidiano, senza eliminare le prove storiche. Nessuna modifica documentale viene applicata da questo audit.

I conteggi per famiglie nel latest daily sono indicatori di ricerca e non una classificazione semantica:
- Windows: 29 voci contenenti il termine.
- morphology/corridor: 18 voci contenenti il termine.
- bootstrap: 8 voci contenenti il termine.
- critic: 10 voci contenenti il termine.
