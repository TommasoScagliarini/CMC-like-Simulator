# Revisione indipendente prima pulizia — 2026-10-14

Attività svolta in sola lettura sulla repository; questo documento temporaneo è l'unico output scritto dall'agente. Letto AGENTS.md, token CMC_AGENT_OK_2026. Backup esterno completo dichiarato dall'utente; nessuna ulteriore richiesta di permesso.

## Perimetro verificato

La decision_matrix.csv ha 2.159 righe GENERATED o GENERATED_CONDITIONAL:

| Gruppo | File | Byte alla matrice | Stato Git |
|---|---:|---:|---|
| Cache incidentali | 1.992 | 48.271.505 | ignorati |
| opensim.log root | 1 | 78.159.231 | ignorato |
| CMake nelle quattro build tecniche | 161 | 9.185.879 | 81 tracciati, 80 ignorati |
| Intermedi paper/main LaTeX | 5 | 98.046 | tracciati |

Tutti i path esistono; nessun candidato o suo antenato è un symlink. Byte live complessivi 135.735.141, 20.480 in più del censimento perché cinque .DS_Store sono mutate. Nessun sorgente risulta nei candidati.

## Eccezioni obbligatorie / prudenti

1. **Preservare models/SEASEA/data/.DS_Store**: già KEEP_PINNED_INCIDENTAL, SHA256 `e37b93510f1a1a7111f17314b4faa40cae4fad2a7ef0650e7a1bc1128a2bb3c9`, 8.196 B.
2. **Nuova eccezione obbligatoria: models/AB06_SEASEA_Threadmill/.DS_Store**: presente erroneamente tra GENERATED, pinnato in validation/ab06_ab07_independent_attribution_2026_10_07/provenance/sources_before.json:90, preparation_provenance_protocol.json:238 e review/preparation_protocol_review.json:208. SHA256 live coincide con pin `e85b766c4dc68758411961ecf500baa06c5b67863938fbbcd070912148314b2c`, 6.148 B. Escluderlo dall'actionlist.
3. **Preservazione prudente consigliata: models/AB06-raw/10_09_18/static/.DS_Store**, 6.148 B, SHA256 `6dfaff32f8e06a2f2b27c4b426f117fd9cbbaed5ebd8ac8824212bd781112cbe`: nominato nello static_file_inventory di validation/ab07_mount_registration_verification_2026_10_07/constraints/raw_floor_evidence.json:77, senza hash. Non è un input fisico ma conservarlo evita di cambiare l'inventario di una verifica recente per soli 6 KB.

I log scientifici nei modelli, run e validation restano fuori dal perimetro. Non è raccomandata la cancellazione generica di opensim.log o build/ annidati.

## Build e runtime

Le quattro directory tecniche sono /build/, /build_online_grf/, /tools/online_grf_contact/build/ e /tools/sea_plugin_relative_d/build/. Solo due librerie runtime sono candidate e hanno copie byte-identiche in plugins/:

- libOnlineGRFContact.dylib, 236.136 B, SHA256 `5597a59a5368825fd207753b59291e72240c1ece3aa5280804f1e9b9c7d6a2b3`.
- libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib, 218.560 B, SHA256 `77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b`.

Entrambi gli hash coincidono con i pin nel frozen_manifest random_pose del 13 ottobre, righe 3295-3296. config.py:32 e :61 caricano plugins/, non le build. I sorgenti dei tools e CMakeLists restano fuori dalle build. Tutti i plugin Windows/macOS presenti in plugins/ vanno conservati. La build Windows conserva prodotti intermedi/progetti e non un'unica DLL runtime.

Nel walk corrente /build/ ha un nuovo `build/CMakeFiles/.DS_Store` non incluso nella matrice; è una cache ordinaria rimuovibile se l'actionlist viene aggiornata. Le altre tre build hanno tutti i file già coperti da righe CMake/cache della matrice.

## Ricerca riferimenti e pin

Eseguita ricerca letterale dei path candidati (e varianti backslash per build) in tutti i JSON/YAML/TOML/Python/XML/Markdown, includendo i file ignorati con rg -uuu ed escludendo .git e i report di questa analisi. Riferimenti residui sono le eccezioni .DS_Store sopra, comandi di build nel README OnlineGRF e vecchi report/handover che nominano CMakeCache. I file CMake generati citano se stessi. Nessun pin individuato per i 161 prodotti CMake, i cinque intermedi LaTeX e il log root. La ricerca path/hash del log root non trova pin nei manifest.

## Ignore proposti

Aggiungere regole precise:

```gitignore
/build/
/build_online_grf/
/paper/main.aux
/paper/main.fdb_latexmk
/paper/main.fls
/paper/main.log
/paper/main.synctex.gz
```

Le build dei tools sono già ignorate da `tools/**/build/`. Non aggiungere una regola globale build/ né una regola globale *.log. Conservare paper/main.tex, main.pdf, sections/ e paper/aux/*.md. Le regole proposte sono consistenti su Windows e macOS perché Git usa slash nei path.

## Esito

Perimetro tecnico idoneo alla rimozione dopo esclusione obbligatoria di AB06_SEASEA_Threadmill/.DS_Store e preservazione del già escluso SEASEA/data/.DS_Store. Raccomandata anche l'eccezione nominativa raw static. Nessuna migrazione del simulatore o semantica SEA richiesta. Nessun training, rollout o test di simulazione avviato dall'agente.
