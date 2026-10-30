# 2026-10-15 — Migrazione dei sorgenti SEA nella repository esterna

Il progetto `/Users/tommy/Documents/SEA_plugin_core - agent` e ora aggiornato
alla stessa versione PI del SEA usata dal simulatore. Dopo la verifica della
copia, della compilazione e dell'equivalenza numerica sono stati rimossi i
quattro file vivi di `tools/sea_plugin_relative_d/`. Tutti i file installati
in `plugins/`, comprese le librerie `.dylib` e `.dll` presenti, sono invariati.

La build esterna e riuscita su macOS arm64. Una prova differenziale su 180
valori e due simulazioni CMC cascade+PI da 30 ms hanno prodotto gli stessi
risultati con la libreria installata e con quella ricompilata. Nessun binario
ricompilato e stato installato al posto di quello corrente.

## Problema e soluzione

I sorgenti SEA esterni erano rimasti senza `Ki`, stato integrale e anti-windup,
mentre la versione PI corrente era nella cartella del simulatore
`tools/sea_plugin_relative_d/`. VS Code puntava alla copia interna e usava
`build/` nel root come directory CMake, generando cache e log anche quando
la configurazione falliva.

La migrazione trasferisce la responsabilita dei sorgenti C++ alla repository
dedicata. Il cascade resta nel controllore Python del simulatore; il PI interno
e la dinamica SEA restano nel plugin C++. Il simulatore continua a caricare
lo stesso basename da `plugins/` e non deve compilare C++ per funzionare.

## File aggiornati e rimossi

Nella repository esterna:

- `SeriesElasticActuator.cpp` e `SeriesElasticActuator.h` sono stati sostituiti
  con copie byte-identiche dei sorgenti PI interni;
- `Plugin_interface.cpp` era gia identico ed e stato preservato;
- `CMakeLists.txt`, comprese modifiche non committate e target dimostrativi,
  e stato preservato integralmente; non e stato sovrascritto con il CMake
  minimale interno;
- aggiunti `README.md` e `SEA_PI_SOURCE_MIGRATION_2026-10-15.json`;
- salvati i cinque file esterni precedenti (CPP, header, interface, CMake e
  `.gitignore`) in `.migration_backups/2026-10-15_before_pi.tar.gz`, con
  verifica degli hash del contenuto; aggiunte le regole di ignore del backup
  e della nuova directory di compilazione `build_pi/`.

Nel simulatore sono stati rimossi soltanto questi file SEA vivi:

```text
tools/sea_plugin_relative_d/SeriesElasticActuator.cpp
tools/sea_plugin_relative_d/SeriesElasticActuator.h
tools/sea_plugin_relative_d/Plugin_interface.cpp
tools/sea_plugin_relative_d/CMakeLists.txt
```

Rimossi inoltre nove file generati dalla vecchia configurazione CMake del
root, incluso un `.DS_Store` ricomparso durante l'ispezione. Totale di questa
operazione: 13 file, 224.604 byte. Nessun `.dll`, `.dylib` o `.so` incluso
nell'elenco delle rimozioni. Le due directory SEA interna e build root sono
state eliminate con soli `rmdir` dopo essere rimaste vuote.
Fra i prodotti generati rimossi c'e anche il file tracciato
`build/.cmake/api/v1/query/client-vscode/query.json`, che compare come
cancellazione in Git insieme ai quattro file SEA.

Aggiornati [AGENTS.md](../../AGENTS.md) con collocazione dei sorgenti e
descrizione PI effettiva, e `.vscode/settings.json` rimuovendo il vecchio
`cmake.sourceDirectory` e impostando `cmake.configureOnOpen=false`. Le
impostazioni Python e LaTeX sono preservate. Aggiunto
[tools/README.md](../../tools/README.md) con istruzioni di build separate per
macOS arm64 e Windows x64 e indicazioni per il replay storico.

## Strategia e verifica della copia

Prima delle modifiche sono stati registrati gli hash dei quattro file interni,
dei file esterni e delle librerie installate, nonche gli HEAD e lo stato Git
delle due repository, entrambe gia sporche. La sincronizzazione e stata
eseguita solo dopo aver ricontrollato gli hash, salvando prima la versione
esterna precedente. Le cancellazioni sono avvenute solo dopo build e probe
numerico riusciti, con ulteriori controlli SHA e un elenco puntuale di file.

| Sorgente PI trasferito | SHA256 comune alla copia interna precedente e a quella esterna attuale |
|---|---|
| `SeriesElasticActuator.cpp` | `20b122bd576c8b70933feaa76fe9a3e521e2b2d48ba048be8c725a5ae7976379` |
| `SeriesElasticActuator.h` | `c8201a61394b4282b05b1f625f7f2cf6611f56986ad4dc3198b3a1afbbbbbf43` |
| `Plugin_interface.cpp` | `ac529358ac3e4b7517766eea112a16065b25c7458fb41967f4d1e40ce7fb1d48` |

Il CMake esterno mantiene SHA256
`af685ba6bcc5f647b263430f6e4c0bb03d335af64864c5e56eb71b314ffb234c`.
Il vecchio CMake interno e i tre sorgenti restano recuperabili dal commit
simulatore `dba42388c32a441a5543f20881612a52781d7a99`. Nessun commit o push
e stato effettuato: i nuovi sorgenti esterni sono modifiche del working tree,
con ricevuta degli hash; HEAD esterno resta
`370bd25cde7c5315b86262b9f76bb5905f747e40`.
Per condividere il nuovo layout con Windows o altre macchine servono commit e
push distinti: rendere disponibili prima i sorgenti aggiornati della
repository esterna. Il backup `.migration_backups/` e ignorato da Git ed
esiste localmente su questo Mac; non viene pubblicato con un commit.

## Test e controlli eseguiti

| Controllo | Risultato |
|---|---|
| Compilazione esterna isolata | PASS, AppleClang 17, Release arm64, SDK `/Users/tommy/opensim-core-install`, solo target `SEA_Plugin_BlackBox_mCMC_impedence_ff` |
| Schema e confronto numerico delle librerie | PASS, 9 casi per ciascun SEA e ciascun modo: 36 righe, 180 valori double, differenza massima 0 |
| Anti-windup e saturazioni | PASS, inclusi clamp integrale/motore, scarica dell'integrale, `Ki=0` e modo impedance |
| Simulazioni CMC cascade+PI dopo rimozione dei sorgenti | PASS, due processi separati, 30 step da 1 ms ciascuno; 6 stati SEA finiti e integrali entrambi non nulli |
| Confronto output simulazioni | PASS, 16 tabelle `.sto`, tutti i valori finiti, differenza massima 0 alla precisione registrata (8 decimali) |
| Integrita indipendente del simulatore | PASS, 65.009 file tracciati inizialmente presenti verificati (13.647.834.509 byte): 65.003 identici, solo `AGENTS.md` modificato e cinque rimozioni tracciate previste |
| Librerie e metadati | PASS, tutti i 10 file di `plugins/` identici; ultimo setup immutato; soltanto due impostazioni CMake cambiate nell'editor |
| Integrita indipendente esterna | PASS, 177 file originali verificati: 174 identici, modificati solo CPP, header e `.gitignore`; nessun file precedente rimosso, tre aggiunte previste |
| Backup esterno precedente | PASS, cinque contenuti estratti in memoria e confrontati con gli hash originali |
| Suite bundle V26C dopo migrazione | 17/19 PASS, con esattamente i due problemi preesistenti gia descritti nel report del 14 ottobre; nessun nuovo test fallito |
| Whitespace delle modifiche | PASS nel simulatore e nei file modificati della migrazione esterna; il diff esterno completo conserva segnalazioni preesistenti nei CMake/build rimasti byte-identici |

La nuova libreria temporanea ha SHA256
`c6a9b66b6eae3013c88c89ff64511335e49d3e0a70b1cedbe006439697e7051a`.
La libreria installata mantiene SHA256
`77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b`.
I binari non sono byte-identici, ma i risultati dei confronti eseguiti lo sono.

Gli smoke CMC usano AB06 PI, GRF prescritte, controller cascade, integrazione
`rk4_bypass`, finestra 11,99–12,02 s. Il massimo assoluto dello stato integrale
e 0,02004851 N-m-s al ginocchio e 0,02527325 N-m-s alla caviglia. I processi
caricano separatamente ciascuna libreria, evitando doppie registrazioni della
stessa classe OpenSim. Log e output sono stati prodotti in `/tmp`, senza
modificare il setup ricordato dal simulatore e senza generare plot.

## Provenance e limiti

Alcuni freezer/checker storici H0, tra cui V1/V2/V3/V7, e il main di
`Trajectory Generator/validation/AB07/prescribed_ground_registered_2026_10_06/verify_sto_numerics.py`
richiedono i vecchi percorsi per calcolare/verificare hash. Il loro rerun dal
working tree attuale richiede quei file e non e piu direttamente eseguibile.
Le funzioni importate dai checker downstream AB06/AB07 non sono state
modificate. Per riprodurre quelle specifiche prove storiche va usato un
checkout separato del commit precedente indicato sopra, con gli stessi input
congelati. Gli script e i manifest storici restano invariati per preservarne
il significato.

I quattro file SEA non appartengono alle 1.888 entry archiviate o alle 1.357
evidenze in_place del bundle V26C. La migrazione non introduce nuovi errori
nella relativa suite: rimangono il drift gia presente di `v26c_j1_collect.py`
e le tre aggiunte rispetto alla closure storica.

Verifica nativa eseguita soltanto su macOS arm64. Il ramo MSVC e la guida
Windows sono preservati, senza affermare una prova Windows nativa. Lo smoke
di 30 ms non e una validazione scientifica completa, un rollout RL best47 o
una prova di generalizzazione. I target dimostrativi esterni, fra cui
`ForwardDynamics_App`, non sono stati eseguiti. Le run e i dataset ignorati
del simulatore non sono stati nuovamente hashati; il controllo indipendente
copre invece tutti i file tracciati presenti, tutti i file di `plugins/` e
tutti i file esterni precedenti.

## Revisione ed evidenze

Tre agenti hanno esaminato separatamente riferimenti, vincoli di provenance e
build/verifiche; un controllo indipendente finale ha superato 20 controlli.
Claude Opus 5.5 con effort `max` ha revisionato i sorgenti, il CMake, le
ricevute e i risultati in sola lettura. La ricevuta del provider conferma
`claude-opus-5-5`, senza fallback. Il verdetto conferma migrazione esatta,
semantica e binari preservati, con verifica adeguata allo spostamento su Mac.
Sono state recepite le richieste di rendere persistenti le evidenze e
includere H0 V2 nella documentazione dei replay storici. Sono stati chiariti
anche stato dei commit, natura locale del backup e rimozione del file query
tracciato. Le guide riportano il compilatore AppleClang usato nella verifica
e un controllo esplicito della selezione dell'SDK Windows. Il CMake esterno
rimane invariato; i comandi di guida, che lasciano libero il generatore su
Mac, non sono stati eseguiti alla lettera: la prova ha usato Ninja e una
directory temporanea isolata.

Le evidenze sono in
[reports/cleanup/2026-10-15_sea_source_migration/](../cleanup/2026-10-15_sea_source_migration/),
con manifest degli artefatti, baseline compressa, ricevute, log, probe e output
dei due smoke. I documenti di migrazione descrivono la nuova collocazione;
nessuna ulteriore copia viva dei sorgenti C++ SEA e stata introdotta nel
simulatore.
