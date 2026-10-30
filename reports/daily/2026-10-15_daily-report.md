# Daily Report — 2026-10-15

> Daily ricostruito il 2026-10-29 dai report utente del giorno: `end_day` non
> era stato eseguito. Percorsi del 15 ottobre. Dal 2026-10-20 i profili GRF di
> `online_grf_profiles/` sono in `online_GRF/profiles/`, i sorgenti del plugin
> GRF in `online_GRF/sources/` e gli strumenti di preparazione dei modelli in
> `tools/model_preparation_pipeline/` (vedere `AGENTS.md`).

**Stato:** sei interventi di riorganizzazione, ciascuno verificato con hash e
smoke nativi macOS arm64. Nessuna modifica a C++, leggi di controllo,
parametri fisici o pesi; nessun commit o push. Instruction check token:
CMC_AGENT_OK_2026.

## Sorgenti SEA nella repository esterna

Report: [migrazione dei sorgenti SEA](../user/2026-10-15_migrazione_sorgenti_sea_repository_esterna.md).

`SEA_plugin_core - agent` era rimasta senza `Ki`, stato integrale e
anti-windup, mentre la versione PI viva stava in
`tools/sea_plugin_relative_d/`. Ora la repository esterna ha copie
byte-identiche di `SeriesElasticActuator.cpp` e `.h`; `Plugin_interface.cpp`
era già identico e il `CMakeLists.txt` esterno è stato preservato. I file
esterni precedenti sono salvati in `.migration_backups/2026-10-15_before_pi.tar.gz`,
locale e ignorato da Git.

Nel simulatore sono stati rimossi i quattro file SEA vivi e nove prodotti CMake
del root (13 file, 224.604 byte). Aggiornati `AGENTS.md`,
`.vscode/settings.json` (`cmake.configureOnOpen=false`) e `tools/README.md`,
con le build per macOS e Windows. Le librerie installate in `plugins/` non sono
cambiate: la build esterna non è stata installata al loro posto.

Verifiche:
- build esterna AppleClang 17, Release arm64: PASS;
- prova differenziale su 180 valori: differenza massima 0;
- due simulazioni CMC cascade+PI da 30 ms con libreria installata e
  ricompilata: identiche;
- integrità del simulatore e della repository esterna: PASS;
- suite V26C: 17/19, invariata.

Per usare il nuovo layout su Windows servono prima commit e push della
repository esterna. I checker storici H0 e
`verify_sto_numerics.py` di AB07 richiedono i vecchi percorsi, quindi un
checkout di `dba42388c`.

## Pulizia e riorganizzazione dei plugin

Report: [pulizia dei plugin](../user/2026-10-15_pulizia_e_riorganizzazione_plugin.md).

Eliminate sette librerie SEA precedenti e copie di backup (2.153.336 byte). Le
due librerie attive sono state spostate senza modifiche in
`plugins/SEA/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib` e
`plugins/online_grf_contact/libOnlineGRFContact.dylib`.

`path_resolver.py` espone le due directory e rimappa i vecchi percorsi piatti
dei due plugin correnti, anche con separatori Windows. Il caricamento usa il
basename OpenSim, senza prefisso `lib` né estensione. Sono stati aggiornati:
- `config.py`, `model_loader.py`, `visualize.py`;
- pipeline e builder, da cui sono stati rimossi i fallback alle vecchie
  dinamiche SEA;
- la calibrazione, che ora sceglie l'estensione corretta anche su Windows;
- l'inventario del pilot `unified_MLP_39`, i test e la documentazione.

Verifiche:
- hash delle due librerie identici;
- test: 16/16, 52/52, 38/38;
- preflight AB06/07/08: PASS, anche con il vecchio percorso Windows;
- cascade+PI sui tre soggetti: 48 STO identici byte per byte;
- best47: due passi OpenSim, PASS;
- confronto di 72.320 file senza perdite.

Le DLL Windows non erano in questo checkout. Vanno compilate e collocate in
`plugins/SEA/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` e
`plugins/online_grf_contact/OnlineGRFContact.dll`; nessuna DLL è stata creata
rinominando vecchi binari.

## Modelli per famiglia

Report: [riorganizzazione dei modelli](../user/2026-10-15_riorganizzazione_modelli_per_famiglia.md).

`models/` ha ora cinque famiglie: `AB06`, `AB07`, `AB08`, `SEASEA` e
`SEASEA_healthy` (prima `SEASEA - whealthy data`). Ogni soggetto raggruppa
`*-raw`, `*_SEASEA-raw` e `*_SEASEA_Threadmill`. Dieci rinomine sullo stesso
disco, senza copie né symlink. Spostati 7.045 dei 7.060 file (5,83 GB); 22
`.osim` e 6.977 asset numerici byte-identici. Solo 20 XML cambiano, con 70
sostituzioni di percorso e nessun parametro.

Aggiornati:
- `config.py`, `setup_io.py`, `visualize.py`, `.simulator_last_setup.json`;
- `path_resolver.py`, che accetta i vecchi percorsi dei bundle;
- 33 file del generatore, inclusi 19 preset, e nove strumenti;
- `.gitignore`, `AGENTS.md`, `models/README.md` e il nuovo test
  `test_model_layout_paths.py`.

Verifiche:
- 22/22 modelli con `initSystem()`;
- cascade+PI AB06/07/08: 48 STO identici;
- test: 7/7, 37/37, 14/14;
- best47 dal vecchio percorso di setup: PASS.

Restano 21 riferimenti di input già mancanti prima dello spostamento, fra cui
i setup healthy senza dati. La suite V26C passa a 17 PASS, 1 ERROR e 1 SKIP:
il test della source closure ora salta, una riduzione di copertura e non una
correzione.

## Geometry dentro models e ruolo dei profili GRF

Report: [Geometry e profili GRF](../user/2026-10-15_geometry_in_models_e_profili_grf.md).

`Geometry/` è stata spostata in `models/Geometry/`: le otto STL sono
identiche e non sono stati creati symlink. Il percorso condiviso è
`path_resolver.SHARED_GEOMETRY_DIR`, con fallback alle mesh anatomiche di
OpenSim; nessun `.osim` è stato riscritto. Verifiche:
- 402/402 riferimenti mesh risolti;
- 22/22 modelli caricati;
- cascade+PI: 48 STO identici;
- tre test V4 falliscono, come già prima dello spostamento; i loro hash
  congelati non sono stati riscritti.

Ruolo dei profili GRF: descrivono il contatto piede-terreno (piano, velocità
del nastro, sfere, rigidezza, smorzamento, attrito). best47 e i preset usano
`..._online_full_wrench_residual_tangent_v2.json` (20 sfere) e il detector
`..._grf_detector_HS-TO.json` (4 sfere), in modalità `online_sensor` con forze
online applicate a sinistra e GRF prescritte a destra.

## Pulizia dei profili GRF

Report: [pulizia dei profili GRF online](../user/2026-10-15_pulizia_profili_grf_online.md)
e [rimozione finale dei profili di validazione](../user/2026-10-15_rimozione_finale_profili_grf_validazione.md).

Eliminati 21 dei 24 profili:
- prima 19, cioè 14 inutili al workflow e cinque archiviabili indicati
  dall'utente (marker-inferred, physical-basis, row, rocker, rocker-soft);
- poi i due di validazione, `v4_mesh_experimental` e `online_sensor_basis`.

Tutti erano byte-identici al commit `dba42388c` e sono recuperabili con
`git restore`. Restano i tre operativi: `grf_correct_magnitude`,
`grf_detector_HS-TO` e `online_full_wrench_residual_tangent_v2`.

Verifiche: 31 riferimenti GRF in 22 YAML risolti, incluso best47; 37/37 test
dei contratti runtime; best47 PASS. I test geometrici V4, lo smoke legacy e i
protocolli che selezionano i profili eliminati richiedono ora il ripristino
degli originali.

## TODO

Nessun TODO esplicito nei report del giorno, quindi nessuna voce aggiunta al
[registro](../TODO_BACKLOG.md). I report indicano come attività aperte:
- commit e push della repository SEA esterna prima dell'uso su Windows;
- compilazione delle DLL Windows nelle nuove posizioni;
- checkout di `dba42388c` per i replay storici che usano i vecchi percorsi.
