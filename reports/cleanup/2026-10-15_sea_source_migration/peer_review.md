# Verifica indipendente della migrazione SEA

Data: 2026-10-15 (Europe/Rome). Esito: **PASS**.

Verifica finale ripetuta dalla baseline originale dopo le precisazioni documentali richieste da Claude. Tutti i 65.009 file tracciati inizialmente presenti e tutti gli asset del perimetro sono stati nuovamente hashati; i 20 controlli rimangono PASS.

Verifica in sola lettura delle due repository; scrittura limitata a questa relazione e al JSON nella directory temporanea. Non sono stati eseguiti nuovi test, build o simulazioni.

## Integrita del simulatore

Sono stati confrontati con SHA-256 e dimensione tutti i 65.009 file tracciati presenti nella baseline, pari a 13.647.834.509 byte. Risultato:

- 65.003 file byte-identici; inclusa la modifica preesistente a `.gitignore`.
- Unico file tracciato modificato: `AGENTS.md`, aggiornato con la posizione esterna e la descrizione del PI gia presente nei sorgenti.
- Cinque file tracciati rimossi, esattamente quelli previsti: i quattro file `tools/sea_plugin_relative_d/{SeriesElasticActuator.cpp,SeriesElasticActuator.h,Plugin_interface.cpp,CMakeLists.txt}` e `build/.cmake/api/v1/query/client-vscode/query.json`.
- I 116 percorsi gia mancanti prima della migrazione non sono stati ripristinati.
- Permessi dei file tracciati rimasti invariati. HEAD e lista dei percorsi tracciati invariati.
- Tutti i 10 file di `plugins/` sono byte-identici, senza aggiunte o rimozioni.
- `.simulator_last_setup.json` e byte-identico.
- Il backup di `.vscode/settings.json` corrisponde alla baseline. Le sole modifiche semantiche sono la rimozione di `cmake.sourceDirectory` e l'aggiunta di `cmake.configureOnOpen: false`; impostazioni Python/LaTeX preservate.
- `tools/README.md` aggiunto; le vecchie directory SEA e build root sono assenti.

Non sono state rilevate variazioni concorrenti dei file durante gli hash ne modifiche Git durante la verifica.

## Integrita della repository esterna

Confrontati tutti i 177 file della baseline esterna, inclusi sorgenti, file di lavoro e build preesistenti, esclusa soltanto `.git`:

- 174 file preesistenti byte-identici.
- Modificati soltanto `SeriesElasticActuator.cpp`, `SeriesElasticActuator.h` e `.gitignore`.
- Nessun file preesistente rimosso.
- Aggiunti soltanto `README.md`, `SEA_PI_SOURCE_MIGRATION_2026-10-15.json` e `.migration_backups/2026-10-15_before_pi.tar.gz`.
- CPP, header e `Plugin_interface.cpp` esterni corrispondono byte-per-byte ai rispettivi file interni della baseline. L'interface era gia identico e non e stato modificato.
- `CMakeLists.txt` esterno e byte-identico alla baseline: SHA-256 `af685ba6bcc5f647b263430f6e4c0bb03d335af64864c5e56eb71b314ffb234c`. Sono preservati il progetto e gli altri target preesistenti.
- Verificato il contenuto dell'archivio di backup: i cinque file originali CPP, header, interface, CMake e `.gitignore` corrispondono agli hash della baseline esterna, incluse le modifiche non committate precedenti.

## Revisione e limiti

Il nuovo `AGENTS.md` e i due README documentano la nuova collocazione, mantengono il nome runtime del plugin e distinguono la compilazione dalla sostituzione dei binari. I record storici dei percorsi interni rimangono invariati e la documentazione indica il checkout Git per il replay. La guida copre macOS arm64 e Windows x64 e dichiara che la verifica Windows nativa non e stata eseguita.

Sono state lette le ricevute di build, confronto differenziale e smoke della cascata, senza rieseguirli. Le ricevute riportano PASS; 180 valori del probe e 16 tabelle dello smoke hanno differenza massima zero. Queste evidenze sostengono l'equivalenza verificata su macOS; lo smoke breve non qualifica robustezza o generalizzazione.

Il controllo indipendente copre i file tracciati del simulatore, l'intera cartella `plugins/`, i due metadata richiesti e tutti i file della baseline esterna. Le run e i dataset arbitrari ignorati non sono stati nuovamente hashati. Per gli otto prodotti build ignorati rimossi si dispone della ricevuta di rimozione; la baseline indipendente richiesta censiva soltanto il file query tracciato, oltre a plugin e metadata.

Dettagli riproducibili: `final_integrity_check.json`; tutti i 20 controlli automatici risultano veri.

## Sigillo dopo la revisione documentale

I due README ora esplicitano i compilatori macOS, il controllo `OpenSim_DIR` nella cache Windows, il replay V2 e la sequenza di condivisione dei due repository. La guida esterna chiarisce che l'archivio del precedente checkout esterno e locale al Mac. `.gitignore` esterno ignora ora anche `/build_pi/`. Queste precisazioni non modificano sorgenti, CMake o librerie.

Hash SHA-256 correnti:

- `tools/README.md`: `7dbe65e5f7c5c526480b1183a8b8d8080fa639c20db199b19335bae256e9033c`.
- `README.md esterno`: `a79863cd9b576a1017249a30de23f72932f0cbcb6269fa23aafe95d38ba1921c`.
- `.gitignore` esterno: `0ffae1c04b0dbfb22f687644054acc6c03ac38f78c1d66beb52c2a07eba80d47`.
- `final_integrity_check.json`: `600a681f221a52c7f81d9d2b8bdaf09f138265fc1f96028bfe977a67e1520834`.

Riconfermati 65.003 file tracciati byte-identici, un solo AGENTS modificato e cinque rimozioni previste; tutti i dieci plugin e last setup identici. Nel checkout esterno rimangono 174 dei 177 file originali byte-identici, tre modifiche autorizzate e tre aggiunte. Nessun nuovo test, build o simulazione eseguito.
