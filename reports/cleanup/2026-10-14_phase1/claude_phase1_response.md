La pulizia è stata eseguita correttamente nel perimetro autorizzato. Nessun file mantenuto è andato perso o è stato alterato, e nessuna dipendenza runtime è stata rimossa. Sul worktree il lavoro è completo; prima di chiuderlo serve solo una correzione documentale (punto 5), senza nuove cancellazioni.

### 1) Dipendenze e binari vincolati
- **Dylib:** gli unici binari runtime rimossi sono i due `lib*.dylib` in `tools/*/build/`. In `plugins/` restano copie con SHA identico (77390d0f…, 5597a59a…), uguali ai pin storici e alle ancore. `config.py:32` e `config.py:61` caricano da `plugins/`, e l'`opensim.log` dello smoke conferma che il caricamento è avvenuto proprio da lì.
- **Sorgenti:** sorgenti e `CMakeLists.txt` di entrambi i plugin sono presenti. Il README di OnlineGRF ricompila in `tools/online_grf_contact/build`, quindi la procedura documentata resta valida.
- **Windows:** `build_online_grf/` conteneva solo progetti e intermedi MSVC, nessuna DLL. I suoi 32 file erano tracciati, quindi si recuperano da HEAD. Le DLL Windows ff e OnlineGRF mancavano già prima (gap dichiarato `windows_plugin_binaries: missing`); l'unica DLL presente è invariata.
- **Cache:** nessun manifest JSON pinna `.pyc` o cache; `in_place_manifest` li nomina solo come esclusioni. Nel worktree non resta alcun `.pyc`.
- **Limite non coperto dal preflight (bytecode orfano):** ad esempio `Trajectory Generator/unified_MLP/__pycache__/grf_defaults.cpython-310.pyc` non aveva né sorgente né import nel repo.
  - Python 3 non importa da `__pycache__` senza il `.py`, quindi l'impatto runtime è nullo.
  - Era però l'ultima traccia di quel modulo nel worktree; ora esiste solo nel backup.
  - Allo stesso modo, `CMakeCache.txt`, `flags.make` e `link.txt` che hanno prodotto il dylib pinnato esistono ora solo nel backup.
  - È un limite da annotare, non un'azione da correggere.

### 2) Qualità del confronto byte e dei test
Punti solidi di `phase1_ops.py`:
- scansione completa del worktree, file ignorati inclusi (escluso solo `.git`), con SHA-256 sul contenuto intero;
- controllo che il file non cambi durante l'hash, e interruzione se compare un symlink;
- cancellazione da lista esatta, con ri-verifica dello SHA prima di ogni `unlink` e nessuna rimozione ricorsiva;
- journal scritto riga per riga (2.158 righe).

I conti tornano: 2.159−2+1 = 2.158 file, e 135.714.661+20.480−12.296+6.148 = 135.728.993 B. Dall'ordine di modifica delle evidenze, il controllo finale è stato eseguito dopo reload e smoke, quindi esclude anche effetti collaterali dei test nel repo.

Limiti:
- **Directory:** il confronto è sui file, non sulle directory. `/build/`, `tools/sea_plugin_relative_d/build/` e `tools/online_grf_contact/build/` restano come scheletri vuoti (`CMakeFiles/pkgRedirects`, `CompilerIdC/tmp`). Il motivo è che `rmdir` (`phase1_ops.py:76-90`) risale solo dalle directory dei file rimossi. È solo cosmetico.
- **`.DS_Store`:** 7 `.DS_Store` sotto `unified_MLP*` non erano nel censimento, quindi restano e sono contati come mantenuti. È coerente con il metodo a lista esatta.
- **Reload di best47:**
  - Con input tutto a zero, il confronto "prefisso 35 = vettore completo 84" è debole, perché anche la parte privilegiata vale zero.
  - Non c'è confronto con un'uscita storica.
  - L'integrità dei pesi la garantisce il pin di `module_state.pkl`, identico a quello di `milestone_iteration_000047`.
- **Smoke nativo:**
  - Il comando effettivo (`watchdog_summary.json`) usa `--episode-duration 5 --episode-start-offset-s 1.9569`, non 0.02 come previsto da `integrity_plan.json`.
  - Si chiude per `max_steps` (`end_reason: null`), non per il limite di tempo atteso dal piano. È una deviazione legittima, ma va registrata.
  - La reward è 0.0 su entrambi gli step, compatibile con `clip_low=0`; la reserve arriva a 101 N·m e non c'è un riferimento con cui confrontarla. Lo smoke quindi non qualifica né la reward né i valori numerici.
- **Ancore:** il PASS di `verify_protected_anchors.py` non ha un output salvato. È comunque implicato dal confronto completo: a campione, pin e baseline coincidono.

### 3) Attribuzione dei problemi V26C
L'attribuzione è corretta:
- **Drift di `v26c_j1_collect.py`:** il file attuale coincide con la baseline pre-pulizia e con HEAD (55.320 B). Il manifest di agosto divergeva già da HEAD.
- **FAIL della source closure:** `collect_source_specs` scandisce le directory dal vivo (`materialize_pipeline_bundle.py:421-429`). Il test quindi non è ermetico rispetto ai file aggiunti dopo il sigillo.

Nelle evidenze manca un'esecuzione della suite prima della pulizia, ma l'attribuzione regge per causa:
- le asserzioni fallite toccano solo file mantenuti e byte-identici;
- nessun file cancellato fa parte della closure del bundle.

È corretto non riscrivere `in_place_manifest` e non riportare `j1_collect` a 55.428 B: annullerebbe una modifica legittima presente in HEAD. Lo stesso vale per il documento j14 rev3, che cita un `.pyc` ora rimosso: è una testimonianza storica, non un pin.

Limiti:
- La versione pinnata (SHA `83862a…`) non esiste in nessun file del worktree, né in `objects/`, né tra i file cancellati. Un eventuale recupero passa dalla storia Git, che non ho verificato.
- Il controllo `--verify --full` previsto dal piano non risulta verde e va descritto così.
- Lo script che ha prodotto `bundle_drift_diagnostic.json` non è tra le evidenze.

### 4) Formulazione onesta proposta
«Rimossi 2.158 file generati (135.728.993 B) da lista esatta con verifica SHA. 151.318 file mantenuti su 151.319 sono byte-identici; l'unica variazione è `.gitignore`, attesa. Al controllo finale, eseguito dopo i test, nessun file risulta mancante, aggiunto o rigenerato. Plugin, modelli, profili e checkpoint best47 sono invariati. Su macOS arm64 best47 si ricarica e completa uno smoke OpenSim di 2 step (0,02 s). Bundle V26C: oggetti e controlli OK, in-place 1.356/1.357, con 1 drift e 3 file extra di closure preesistenti. Non verificati: Windows x86, replica best47 di 5 s e qualifica FSMv3, contenuto del backup, recuperabilità dei 2.072 file non tracciati (gli 86 tracciati sono in HEAD).»

Formulazioni da evitare:
- "bundle verificato";
- "build eliminate" (restano gli scheletri vuoti);
- "86 pin storici": sono 84, più 2 ancore operative con `historical_sha256: null`;
- "tre `.DS_Store` verificati dalle ancore": nelle ancore c'è solo `models/SEASEA/data/.DS_Store`; gli altri due sono coperti dal controllo SHA prima della rimozione e dal confronto completo.

### 5) Completezza
La pulizia è completa sul worktree; servono tre correzioni documentali:
1. **Salvare le evidenze:** oggi stanno in `/tmp`, che macOS può svuotare. Vanno copiate in `reports/cleanup/2026-10-14/phase1/`: manifest, journal, result, after-check, controlli di ignore, log dei test, drift diagnostic insieme al suo script. `whole_worktree_before.json` conviene comprimerlo. Il journal è l'unico registro con SHA dei 2.072 file non tracciati rimossi.
2. **Registrare le deviazioni** descritte al punto 2 (smoke a 5 s con offset, output delle ancore non salvato, `--verify --full` non verde).
3. **Al momento del commit:**
   - mettere in stage solo le 86 rimozioni e `.gitignore`, senza `git add -A` o `commit -a`, così le 31 cancellazioni di `Prosthesis_SNN` restano separate;
   - finché il commit non c'è, l'indice traccia ancora quegli 86 path, e le nuove regole di ignore per loro hanno effetto solo dopo il commit.

Opzionali e non bloccanti: eliminare gli scheletri vuoti delle build e i `.DS_Store` residui.
