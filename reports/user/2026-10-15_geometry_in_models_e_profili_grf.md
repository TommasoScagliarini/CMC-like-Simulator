# 2026-10-15 — Geometry dentro models e ruolo dei profili GRF

`Geometry/` e stata spostata in `models/Geometry/`. Le otto mesh STL sono
identiche agli originali; la directory precedente non esiste piu e non sono
stati creati symlink. Tutti i 22 modelli continuano a caricare le proprie mesh.

## Strategia e modifiche

Due subagenti hanno verificato separatamente runtime/mesh e profili/strumenti
di validazione. Prima dello spostamento sono stati acquisiti inventario SHA,
caricamenti OpenSim, risoluzione delle mesh e brevi simulazioni di confronto.

Il percorso condiviso e ora `path_resolver.SHARED_GEOMETRY_DIR`:

- `model_loader.py` e `visualize.py` cercano prima in `models/Geometry/`,
  conservando i fallback alle mesh anatomiche installate con OpenSim.
- Il resolver riconosce anche i vecchi indirizzi `Geometry/...`, inclusi
  separatori Windows e checkout riconosciuti. Copie archiviate annidate e
  percorsi esterni esistenti conservano il proprio significato.
- Il builder ABx usa il nuovo percorso per `--foot-mesh`. Il visualizzatore
  mantiene anche la semantica delle directory aggiuntive relative al cwd.
- Gli strumenti live di audit geometrico e costruzione della suola risolvono
  la nuova posizione. Nel builder preregistrato gli identificatori logici dei
  sorgenti e i controlli SHA restano quelli della griglia originale; cambia
  soltanto il lookup fisico. I dieci candidati prodotti sono identici.
- Aggiornati i test interessati e la documentazione in `AGENTS.md`,
  `models/README.md` e `online_grf_profiles/README.md`.

Non e stato necessario riscrivere alcun `.osim`: i riferimenti delle mesh
nei modelli sono nomi di file, risolti attraverso le directory registrate.
L'inventario completo elenca 13 file modificati, oltre alle otto mesh spostate.

## Verifiche

| Controllo | Esito |
| --- | --- |
| Confronto esteso su 72.047 file preesistenti | 72.034 identici, 13 modifiche previste, zero perdite o cambiamenti inattesi |
| Mesh spostate | 8/8, SHA identici |
| File originari dei modelli | 7.060/7.060 byte-identici |
| Caricamento e inizializzazione OpenSim da nuovo processo e cwd `/tmp` | 22/22 modelli |
| Riferimenti mesh nei modelli | 402/402 risolti agli stessi asset; zero mesh mancanti |
| Cascade+PI AB06/07/08, 30 ms ciascuno, prima/dopo | 48 STO identici byte per byte; differenza numerica zero |
| Resolver del simulatore | 9/9 test PASS |
| Contratti runtime delle famiglie MLP | 37/37 test PASS |
| Review runtime con controlli CLI, Windows/UNC, cwd esterno e priorita | 24/24 controlli PASS |
| Registrazione geometrica e default ABx | 31/31 test PASS, senza skip |
| Builder suola, audit e profilo V4 | 44 PASS, 3 FAIL storici V4 |
| Dieci candidati del builder, vecchio sorgente in checkout temporaneo e sorgente aggiornato | Dizionari completi e JSON serializzati identici, senza scrivere nuovi profili |
| Profili GRF | 24/24 caricati dal parser reale e byte-identici alla baseline |
| `git diff --check` | PASS |

Le prove native sono macOS arm64. Windows nativo e apertura della GUI non sono
stati eseguiti; il visualizzatore e stato verificato nella registrazione dei
percorsi e nella risoluzione delle mesh. Le simulazioni sono smoke brevi,
non una nuova qualificazione scientifica di episodi completi.

I tre test V4 erano gia falliti nella baseline degli strumenti: confronto con
metadati di percorsi precedenti, hash del setup precedente e hash storico del
preset di training. La migrazione aggiunge al primo confronto la nuova
posizione fisica della mesh; non cambia le sfere del profilo. Profili, griglia,
protocolli e relativi hash congelati non sono stati riscritti per rendere verdi
questi test. Alcuni readiness protocol conservano anche gli hash della versione
precedente del sorgente live di audit: descrivono quella versione storica,
non il nuovo lookup. Gli snapshot e i manifest restano invariati.

## Che cosa contiene online_grf_profiles

La cartella contiene 24 configurazioni JSON, circa 316 KB nell'inventario
iniziale. Descrivono il contatto piede-terreno: piano del terreno, velocita
del tapis roulant, posizione e raggio delle sfere di contatto, rigidezza,
smorzamento, attrito e coefficienti calibrati. `online_grf.py` legge questi
parametri e configura i componenti del plugin `OnlineGRFContact`.

Nei preset MLP attuali e nel checkpoint best47 sono usati soprattutto:

- `AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`:
  profilo principale delle forze, con 20 sfere di contatto.
- `AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`: rilevatore heel strike
  e toe off, con quattro sfere.

Questi preset usano `online_sensor` con `online_grf_applied_side: [left]`:
le forze online agiscono sulla dinamica a sinistra, mentre sull'altro lato
restano le GRF prescritte. Il detector misura gli eventi. Senza lati applicati,
`online_sensor` opera soltanto come sensore; il default del simulatore root e
invece `prescribed`, che non richiede questi profili.

Gli altri JSON includono tentativi di calibrazione e versioni sperimentali o
storiche. Non tutti servono a una singola esecuzione, ma diversi sono richiamati
da checkpoint e verifiche. Anche i due profili byte-identici `correct_magnitude`
e `residual_tangent_v2` hanno consumatori distinti. Nessun profilo e stato
rimosso. Il README della cartella ora descrive i default effettivi dei preset,
correggendo il precedente riferimento al vecchio candidato `physical_basis`.

Evidenze: [inventari, log e ricevute](../cleanup/2026-10-15_geometry_in_models/README.md).
Nessun commit o push eseguito.
