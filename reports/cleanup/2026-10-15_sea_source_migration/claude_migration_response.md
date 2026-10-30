# Revisione indipendente: migrazione dei sorgenti SEA

Ho lavorato solo con Read/Grep/Glob e non ho letto `simulator_before.json`. `final_integrity_check.json` e `peer_review.md` sono presenti e li ho usati.

**Verdetto:** la migrazione è corretta e verificata su macOS arm64. Si può consegnare dopo due correzioni concrete (punto 4) e tre avvertenze da scrivere nel report.

## 1) La migrazione è esatta e completa?
Sì.
- **Sorgenti:** `SeriesElasticActuator.cpp` e `.h` esterni coincidono byte per byte con i vecchi file interni (SHA `20b122…` e `c8201a…`). `final_integrity_check` li ha ri-hashati per conto suo e io li ho confrontati riga per riga: sono identici. `Plugin_interface.cpp` era già identico (`ac5293…`).
- **CMake:** non aver copiato quello interno è corretto. Per il target plugin, il ramo Mac del CMake esterno (`af685b…`) è equivalente: stesso target, stessi sorgenti, `-fno-char8_t`, arm64, `NO_DEFAULT_PATH`. L'unica differenza è che il ramo MSVC (`CMakeLists.txt:39` esterno) non ha `NO_DEFAULT_PATH`. È però la stessa ricetta Windows usata in passato (`HANDOVER_LLM_2026-05-17.md:19850`).
- **Simulatore:** non resta nessun sorgente SEA. L'unico altro `Plugin_interface.cpp` è quello di `tools/online_grf_contact/`, un componente diverso, ed è giusto averlo conservato.

## 2) Binari e semantica sono preservati?
Sì.
- **Binari:** i 10 file di `plugins/` sono invariati. Dei 177 file esterni iniziali, 174 sono invariati e cambiano solo cpp, h e `.gitignore`. Nessuna rimozione, nessun deploy.
- **Build:** è partita davvero dalla repo esterna (`CMAKE_HOME_DIRECTORY` esterno, `OpenSim_DIR=~/opensim-core-install/lib/cmake/OpenSim`, `-O3 -std=c++17 -arch arm64 -fno-char8_t`).
- **Caricamento:** ogni processo ha caricato una sola libreria, quella attesa (`rebuilt_cascade.log:9,19`), senza avvisi di sostituzione.
- **Probe:** i casi attivano davvero tutti i rami anti-windup:
  - `motor_limit`: per il knee `tau_raw≈1900 Nm`, oltre il limite di 500;
  - `integral_limit`: `Ki·xi=380 > 100`;
  - `unwind`: errore negativo.
- **AGENTS.md:** descrive correttamente il codice (PI, `Kd_Imp`, anti-windup, `Ki=0`).

## 3) Le verifiche sono adeguate? Limiti
Bastano per uno spostamento di sorgenti byte-identici su macOS. Limiti:
- nessuna build né esecuzione su Windows;
- lo smoke dura 30 ms e gli STO hanno 8 decimali: non è una validazione di robustezza;
- i target legacy (`exampleCustomActuator`, `ForwardDynamics_App`) non sono stati compilati. Il rischio è basso perché l'API è solo estesa;
- i comandi dei README non sono stati eseguiti alla lettera: la prova ha usato Ninja e `/usr/bin/clang++` esplicito;
- io non posso calcolare hash: l'identità byte la prendo dalle ricevute, più il confronto testuale che ho fatto io.

## 4) Correzioni

**Obbligatorie prima della consegna**

a) **I report citati non esistono e le prove sono solo in `/tmp`.**
- [tools/README.md:67-69](tools/README.md:67) rimanda a `reports/user/2026-10-15_migrazione_sorgenti_sea_repository_esterna.md` e a `reports/cleanup/2026-10-15_sea_source_migration/`, che non esistono.
- Anche il README esterno (riga 53) e il campo `note` della receipt rimandano al "simulator migration report".
- Tutte le prove stanno in `/tmp`, che macOS può svuotare.

Va scritto il report utente (problema, soluzione, strategia, file, test, come chiede AGENTS). Nella cartella di cleanup vanno copiati:
- le ricevute JSON e i log di preflight, transfer, build, probe, removal e cascade;
- gli script;
- `final_integrity_check.json` e `peer_review.md`.

`build_mac_arm64/` si può escludere.

b) **L'elenco dei controlli storici è incompleto: si rompe anche H0 V2.**
- [run_h0_primary_grf_split_v2_adaptation.py:155-156](validation/run_h0_primary_grf_split_v2_adaptation.py:155) ri-hasha ricorsivamente l'`input_closure` del lock V2.
- Quel lock contiene i vecchi cpp/h ([h0_primary_grf_split_v2_execution_lock.json:176](validation/h0_primary_grf_split_v2_execution_lock.json:176) e riga 186), quindi oggi fallirebbe con "artifact drifted".
- Correzione solo documentale in [tools/README.md:59](tools/README.md:59): aggiungere V2, oppure scrivere "tra cui" per non presentare l'elenco come completo. I lock non vanno toccati.

c) **Avvertenze da scrivere nel report finale:**
- nessuna modifica è committata, né nel simulatore né nella repo esterna. Il checkout Windows riceve il PI solo dopo commit e push della repo esterna, quindi la rimozione nel simulatore non va pushata prima;
- la pulizia di `build/` ha cancellato anche un file tracciato, `build/.cmake/api/v1/query/client-vscode/query.json`, che ora compare come ` D` in git status;
- il backup `.migration_backups/…tar.gz` è escluso da git ed esiste solo su questo Mac.

**Facoltative**
- **README Windows:** dopo il configure, verificare che `OpenSim_DIR` in `build_pi/CMakeCache.txt` punti a `C:/OpenSim-mCMC`, oppure passarlo esplicitamente. Senza `NO_DEFAULT_PATH`, CMake cerca prima negli OpenSim 4.x trovati tramite `PATH` e solo per ultimi nei `PATHS` indicati. Il rischio è concreto, perché su quella macchina esistono anche `C:\OpenSim 4.1` e `C:\OpenSim 4.5`.
- Aggiungere `/build_pi/` al `.gitignore` esterno: oggi non è ignorata, come già `build_agent/`.
- Compilare una volta `ForwardDynamics_App`, per escludere rotture della build completa.
- Nei README, indicare il compilatore esplicito se `CC`/`CXX` vengono impostati da conda.
