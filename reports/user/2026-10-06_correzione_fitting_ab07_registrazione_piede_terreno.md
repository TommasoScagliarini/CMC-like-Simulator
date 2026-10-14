# AB07: correzione del fitting e registrazione del piede rispetto al terreno — 2026-10-06

**Esito:** correzione applicata al bundle operativo `models/AB07_SEASEA_Threadmill`, marker ricalibrati e IK OpenSim completa ricalcolata. La nuova cinematica recupera il contatto e quattro cicli del detector nel replay. **La qualifica dinamica V26 resta aperta:** oggi non sono state eseguite nuove integrazioni CMC-like, rollout con rete o training.

**Instruction check token:** `CMC_AGENT_OK_2026`. Richiesta: stabilire la posa geometrica del piede sul piano comune prima della calibrazione dei marker, conservando i parametri SEA e tutta la control chain AB06.

**Aggiornamento successivo, stesso giorno:** su richiesta dell'utente la registrazione è diventata **il percorso obbligatorio e predefinito del builder ABx**. Gli ingressi comuni AB06 vengono selezionati automaticamente e le opzioni di riferimento consentono override espliciti; nessuna calibrazione deve essere scelta a mano per il nuovo soggetto. La modifica del default non ha rigenerato il bundle AB07 o avviato nuove pipeline/rollout. Il dettaglio e le nuove verifiche sono nella sezione finale «Registrazione predefinita per i nuovi soggetti».

## Problema e causa del passaggio da correggere

AB07 parte dal suo modello e dalla sua cinematica sani. Il percorso resta: innesto SEA, adattamento geometrico, calibrazione dei marker e nuova IK dai marker sul modello protesico. Il problema individuato riguarda la **posa statica usata prima di calibrare i marker**. Riutilizzare direttamente gli angoli sani remappati non stabiliva una posa del nuovo hardware coerente con il piano comune.

La calibrazione libera dei marker poteva assorbire questo disallineamento nelle loro coordinate locali. Una nuova IK produceva quindi un buon errore sui marker, ma conservava il piede protesico troppo alto: il tallone statico aveva clearance media **+7,458 mm**, contro **−14,371 mm in AB06**. La precedente IK statica reale aveva confermato questa situazione. Il detector usa punti fissati al piede hardware; il fit dei marker, da solo, non determina la loro quota sul terreno.

Le indagini precedenti hanno escluso un errore di conversione delle posizioni TRC e verificato la parità del sampler e del replay degli eventi. Non hanno identificato un errore fisico univoco della lunghezza del pylon o dello zero articolare: mancava un vincolo indipendente sulla registrazione hardware–terreno. La [diagnosi causale](../../validation/ab07_detector_investigation_2026_10_06/stage2/causal_diagnosis.json) conserva queste evidenze antecedenti alla correzione.

## Metodo applicato prima dei marker

Ho aggiunto il metodo `donor_relative_static_hardware_ground_v1`. L'hardware del piede e il piano restano quelli comuni già usati in AB06. Il riferimento è la posa statica del donor AB06; il metodo non usa contatti binari, tempi HS/TO, forze online o errore dei marker come obiettivi del fitting.

1. Ho mantenuto l'innesto anatomico AB07 già verificato: hip del soggetto, fitting superiore e proporzioni iniziali della gamba. Il mount superiore `osseo/transfemur_offset` resta **Y = −0,188573279693 m**.
2. Ho caricato i **3.041 vertici unici** dello stesso STL `Geometry/AM_foot_l.STL`. Per ogni posa statica ho calcolato la distanza minima dei vertici dal piano, usando la trasformazione nativa OpenSim del piede. Il piano è letto dalla sola sezione `ground` del profilo binario congelato: origine **[0; 0,0148208231; 0] m**, normale **[0; 1; 0]**.
3. Ho calcolato anche l'inclinazione sagittale del piede rispetto a quel piano. I due obiettivi sono le mediane AB06 su **400 pose statiche**: clearance minima STL **−2,136415973 mm** e inclinazione **7,050422788°**.
4. Su **436 pose statiche AB07** ho risolto due sole regolazioni: la posa statica della caviglia, per riprodurre l'inclinazione, e la posizione longitudinale del mount distale, per riprodurre la quota. Per la quota il solver risolve la mediana dell'intero insieme dopo lo spostamento; tiene conto della diversa inclinazione della tibia in ogni frame.
5. Soltanto dopo questa registrazione ho ricalibrato i 12 marker sinistri, usando la TRC statica originale e la nuova posa statica. Ho quindi eseguito la nuova IK dinamica dai marker originali, su tutto il trial.

| Quantità | AB07 prima | AB07 dopo | Riferimento AB06 |
|---|---:|---:|---:|
| Mount distale `pros_ankle/tibia_pylon_offset`, Y | −385,899299 mm | **−403,478429 mm** | −430,000000 mm |
| Clearance minima STL, mediana statica | +23,418365 mm | **−2,136416 mm** | −2,136416 mm |
| Inclinazione del piede, mediana statica | 2,319774° | **7,050423°** | 7,050423° |
| Clearance media punto heel, statica | +7,458 mm | **−14,472 mm** | −14,371 mm |
| Clearance media punto toe, statica | +13,247 mm | **+7,360 mm** | +7,741 mm |

La regolazione allunga il tratto distale di **17,579130 mm** rispetto al solo fitting anatomico e applica **+4,730648501° esclusivamente alla caviglia nel file della posa statica di calibrazione**. Non cambia lo zero, il default o il range del giunto. Non aggiunge un offset alla IK dinamica o ai comandi del controller: gli angoli dinamici sono stati risolti nuovamente dal tool IK.

I residui del solver sono **−1,73×10⁻¹⁸ m** sulla quota e **5,55×10⁻¹⁷ rad** sull'inclinazione. Sono residui numerici, non precisioni sperimentali. Il [manifest operativo](../../models/AB07_SEASEA_Threadmill/data/fitting/ground_registration.json) contiene ingressi, hash, valori prima/dopo e invarianti.

Questa è una **convenzione geometrica relativa al benchmark AB06**. La quota −2,136 mm riproduce la sua registrazione esistente; non è una misura indipendente del pavimento fisico, della scarpa o della compressione della suola. Il piano del generatore GRF primario rimane quello già congelato nel suo profilo; non è stato sostituito con il piano binario e non sono state modificate le sfere di contatto.

## Pipeline e bundle aggiornati

La sequenza del builder è: fitting anatomico → conversione dati → **registrazione geometrica statica** → calibrazione marker → **IK marker OpenSim completa** → bundle. Nella build AB07 documentata qui il passaggio era stato richiesto esplicitamente con `--ground-reference-motion`, `--ground-plane-profile` e `--fit-reference-model`. Dopo il successivo aggiornamento è obbligatorio e usa i riferimenti comuni di default. Il builder rifiuta `--skip-opensim-ik`, perché il fitting richiede una nuova IK. Il preflight controlla anche l'esistenza degli ingressi comuni.

La build è stata eseguita in isolamento sotto `validation/ab07_detector_investigation_2026_10_06/stage3/build/models/`, con log e IK originali sotto `results/ab07_ground_registered_2026_10_06/`. MATLAB inizialmente restava fermo all'avvio nel sandbox; l'esecuzione fuori dal sandbox ha completato le conversioni. Nessuna conversione precedente è stata usata come fallback. Le conversioni numeriche GRF sono identiche alla baseline; cambia soltanto la prima riga del nome nel relativo header.

Il bundle verificato è stato poi promosso nella destinazione richiesta, conservando l'organizzazione AB06: unico modello PI, setup, `data/`, supporti CMC e `data/healthy/`. La nuova `data/fitting/` conserva posa statica registrata e manifest. Il setup è stato rigenerato con percorsi canonici, anziché copiare i percorsi della build isolata.

| Artefatto operativo | SHA256 |
|---|---|
| `AB07_SEASEA_stiff321_500_pi.osim` | `6ea6dd779ecf88b69bc0cb940ed9e2d5225bb986a2d6f986920096ba574cc245` |
| `data/IK_results_AB07_SEASEA.mot` | `7f497b28c9c9483b042d6b84fbc4d9d897cfd6191eb63015599734b484f3e15a` |

La nuova IK contiene **28.612 frame a 200 Hz**, da **10,360 a 153,415 s**, e proviene da un'effettiva esecuzione OpenSim sui marker. La provenance mantiene il modello e la TRC della build isolata effettivamente usati dal solver. L'overlay sano è invariato. Gli intermedi precedenti in `models/AB07_SEASEA-raw` restano storici; i nuovi intermedi sono quelli esplicitamente indicati nel nuovo `pipeline_report.json`.

Prima della promozione ho archiviato il bundle precedente e la provenance statica: **19 file, 67,097 MB**, verificati byte per byte. L'[archive manifest](../../validation/ab07_detector_investigation_2026_10_06/stage3/baseline/archive_manifest.json) associa ogni vecchio percorso al file archiviato. Le diagnosi e le simulazioni precedenti riguardano il vecchio modello `79770a7e…` e la vecchia IK `6803f2fd…`; rimangono disponibili, ma non qualificano il modello appena aggiornato.

## Verifiche eseguite e risultati

**Contratto del fitting:** 29 test PASS, senza skip: 17 di registrazione/CLI, 5 di fitting anatomico e 7 di preparazione. Coprono identità AB06, verifica indipendente in OpenSim del modello serializzato, conservazione dell'XML protetto, offset soltanto statico, idempotenza, gradi/radianti, sensibilità della quota, range, input mancanti e protezione degli output. [Receipt dei test](../../validation/ab07_detector_investigation_2026_10_06/stage3/final_contract_tests.json).

**Conservazione del modello e del controllo:** 14 verifiche di artefatti PASS prima e dopo la promozione. Rispetto alla baseline cambiano soltanto mount distale Y e posizioni dei marker sinistri. Restano identici SEA, assi/default/range articolari, masse/COM/inerzie, biologico, task e reserve. Verificati **20 fingerprint** della catena di controllo/plugin/riferimenti e **tre profili congelati**. Caricamento nativo: **21 coordinate, 76 muscoli, 11 corpi, 6 stati SEA, massa 58,80765053 kg**. [Verifica del bundle canonico](../../validation/ab07_detector_investigation_2026_10_06/stage3/artifact_verification_canonical.json).

**Geometria della coppia GRF alla caviglia:** altri **3 test nativi PASS** sul bundle aggiornato: identità AB06, centro AB07 e rispetto dell'orientamento del frame. Il precedente test AB07 conteneva il mount anatomico numerico −0,385899299 m; dopo la nuova registrazione era un valore di fixture superato. L'aspettativa è stata sostituita con l'origine nativa del piede, che coincide con il pivot dato il child offset zero invariato. Il codice runtime continua a leggere il mount effettivo dal modello. [Receipt dei tre test](../../validation/ab07_detector_investigation_2026_10_06/stage3/ankle_geometry_after_registration.json).

**Calibrazione e IK:** RMS statico medio dei 12 marker calibrati **0,249901 mm**, contro 0,249370 mm prima. RMS marker dinamico medio **10,379907 mm**, contro **10,327793 mm** prima; RMS massimo per frame 28,79482 mm, errore massimo del singolo marker 120,59757 mm. La piccola variazione dell'errore medio non sostituisce l'esame degli outlier.

**IK statica reale dopo la calibrazione:** ulteriore tool IK su 0–0,04 s, **9 frame e 28 marker con peso 1**, nella configurazione di produzione. Tallone medio **−14,403204 mm**, contatto in **9/9 frame**; toe medio +7,189043 mm. Il solver muove il tallone di appena +0,090422 mm rispetto alla posa registrata della stessa finestra. RMS marker medio **0,321232 mm**. La parità dei punti e dei bit con il sampler di produzione è esatta. [Verifica statica](../../validation/ab07_detector_investigation_2026_10_06/stage3/static_marker_ik_recheck.json).

**Detector sulla nuova IK:** ho campionato tutte le pose native a 5 ms per confrontare la geometria, poi ho riprodotto il filtro di produzione a 6 Hz e la griglia a 1 ms nel tratto AB07 **10,360–19,370 s**. Il replay usa il detector heel-qualified V26 reale, debounce di 5 ms e consegna in batch di 10 ms. È una valutazione di pose salvate e FSM del detector; non è un rollout dinamico, una valutazione della rete o una prova dell'actor FSM v3. Il carico misurato >20 N serve soltanto come riferimento nominale di appoggio.

| Metrica | AB07 prima | AB07 dopo | AB06 riferimento |
|---|---:|---:|---:|
| Almeno un sensore ON durante appoggio nominale, intera IK | 19,813% | **96,630%** | 94,707% |
| Heel ON durante appoggio nominale, intera IK | 1,285% | **78,171%** | 59,793% |
| Almeno un sensore ON durante appoggio, tratto filtrato | 12,597% | **99,938%** | 98,960% |
| HS / TO del detector, tratto filtrato | 2 / 2 | **5 / 5** | 5 / 5 |
| Cicli completi del detector | 1 | **4** | 4 |
| Durata media stance associata a HS, tratto filtrato | 0,1310 s | **1,0885 s** | 1,10775 s |

Il riferimento nominale AB07 contiene quattro cicli completi, con stance media **1,00658 s**. Il quinto TO del replay corrisponde all'appoggio parziale già in corso all'inizio della finestra, escluso dall'oracolo dei cicli completi; la stessa situazione è presente nel confronto AB06. AB06 usa il proprio tratto **11,990–21,000 s**: soggetti e fasi diversi, quindi il confronto non è una prova su campioni accoppiati.

Il miglioramento recupera i cicli, ma **non tutti i criteri temporali storici**: gli HS onset anticipano il riferimento nominale di 44,705–65,705 ms, MAE 54,311 ms. Gli HS confermati sono **4/5 entro 50 ms**; quello a 15,499 s anticipa il riferimento 15,559705 s di **60,705 ms**. Gli HS consegnati sono **5/5 entro 60 ms**, massimo 59,705 ms. I quattro TO nominali sono tutti abbinati: onset MAE 27,474 ms, massimo 38,697 ms. Le tolleranze sono quelle diagnostiche storiche registrate nella receipt, non nuove soglie ottimizzate su AB07.

La [receipt del replay](../../validation/ab07_detector_investigation_2026_10_06/stage3/ik_contact_verification.json) conserva eventi, confronti e fonti. Il [grafico prima/dopo/AB06](../../plot/AB07_ground_registration_2026_10_06/ik_contact_comparison.png), disponibile anche PDF e SVG, è sotto `plot/` del root.

## File modificati o aggiunti per questa correzione

- `tools/register_seasea_ground.py`: nuova registrazione geometrica nativa, solver e manifest; usa plugin/path della piattaforma selezionata.
- `tools/build_abx_seasea_pipeline.py`: integrazione prima dei marker, ingressi espliciti, guardie e obbligo di nuova IK in questo percorso.
- `validation/test_ground_registration.py`: regressioni native, unità e guardie CLI.
- `validation/verify_ground_registered_bundle.py`: controlli di modello, setup, supporti e provenance senza dinamica.
- `Trajectory Generator/validation/AB07/verify_ground_registered_ik.py`: confronto delle pose e replay del detector congelato.
- `Trajectory Generator/validation/AB07/test_ankle_geometry.py`: riferimento geometrico nativo al pivot AB07, aggiornato per il nuovo fitting.
- Bundle operativo AB07, nuove evidenze `stage3/`, plot, questo report e aggiornamento del daily del 6 ottobre.

Il plugin C++, il controller, il governor, le FSM e i parametri dei profili non sono stati modificati. `fit_seasea_subject.py` resta invariato rispetto all'inizio di questa correzione. Lo script di promozione rifiuta di sovrascrivere un bundle diverso dalla baseline esattamente archiviata.

Le receipt conservano l'hash del codice effettivamente eseguito. Dopo la registrazione sono state aggiunte tre righe di protezione CLI contro collisioni output/plugin/script; il codice numerico è invariato. Lo [snapshot con hash corrispondente all'esecuzione](../../validation/ab07_detector_investigation_2026_10_06/stage3/source_snapshot/register_seasea_ground_executed.py) e la receipt dei test sul codice finale mantengono esplicita questa differenza, senza riscrivere retroattivamente gli hash.

La build usa argv strutturati e path portabili, con plugin scelto per piattaforma. L'esecuzione corrente è verificata su **macOS arm64**; la verifica live Windows resta da eseguire.

## Riproduzione della preparazione

I sei comandi dei sottoprocessi e i relativi output sono nei log sotto `results/ab07_ground_registered_2026_10_06/ab07_seasea_treadmill_treadmill_01_01/logs/`. Esempio equivalente del builder, da eseguire dalla root con **destinazioni nuove** e interprete OpenSim disponibile:

```sh
python tools/build_abx_seasea_pipeline.py --subject AB07 \
  --output-models-dir validation/AB07_new_ground_build/models \
  --bundle-dir validation/AB07_new_ground_build/models/AB07_SEASEA_Threadmill \
  --results-dir results/AB07_new_ground_build \
  --model-stem AB07_SEASEA_stiff321_500_pi \
  --donor-model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim \
  --fit-reference-model models/AB06-raw/osimxml/AB06.osim \
  --ground-reference-motion models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot \
  --ground-plane-profile validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json \
  --skip-smoke-test
```

Le continuazioni `\` sono per shell Unix; in PowerShell usare il carattere di continuazione previsto dalla shell o una sola riga. Se necessario specificare `--python`, `--matlab` e `--plugin` per l'installazione locale. Il run odierno ha usato Python `envCMC-like`, MATLAB R2024a con `--matlab-nojvm` e il plugin PI `libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`.

## Stato e TODO propagati

La correzione geometrica richiesta è applicata e verificata sulla nuova IK. Il binding operativo registra il nuovo modello e la nuova IK, con stato **geometria/replay verificati, qualifica dinamica pendente**. I precedenti risultati di impulso online al 24,71%, tracking e reserve restano riferiti alla baseline archiviata; oggi non è stato misurato il nuovo carico online. AB07-Q01 avanza sul fitting e sulla geometria, ma rimane parziale per timing e qualifica del binding nel percorso dinamico. G06 avanza soltanto su AB07; G09/G10 non sono dichiarati completati.

- [ ] AB07-Q01 — Qualificare geometria del contatto, piano e riferimenti dei sensori su AB07, mantenendo comuni i parametri SEA, la control chain, le leggi FSM e le soglie degli eventi; rendere esplicito il binding per modello senza aggirare le guardie path/hash.
- [ ] AB07-Q02 — Dopo la correzione geometrica, rieseguire il diagnostico prescribed V26 con criteri dichiarati e verificare carico, HS/TO, durata stance, cicli e reserve; il completamento temporale non basta per qualificare AB07 prima della policy.
- [ ] AB07-Q03 — Investigare il limite biologico comune di tracking MTP, distinguendo osservabilità della IK e dinamica/SO, senza introdurre guadagni specifici per AB07.
- [ ] AB07-Q04 — Eseguire una verifica live della nuova pipeline AB07 su Windows con gli stessi parametri comuni e registrare gli esiti; le prove macOS e quelle Windows storiche AB06 non sostituiscono questa verifica.

## Registrazione predefinita per i nuovi soggetti

Il passaggio mancante era un problema del metodo di preparazione, che potrebbe ripresentarsi su altri soggetti. Non è dimostrato che ogni soggetto presenti lo stesso errore numerico; il vincolo geometrico va comunque applicato a tutti i modelli costruiti con questo hardware. AB06 resta il caso d'identità del metodo, con correzione geometrica nulla usando i suoi stessi riferimenti.

`tools/build_abx_seasea_pipeline.py` ora seleziona automaticamente il donor operativo **AB06 PI321/500**, il modello sano AB06 e la motion statica convertita `static_01` sotto `--models-dir`, insieme al profilo del piano e allo STL comuni. Sono stati eliminati il ramo senza registrazione e il fallback IK remappato dal percorso pubblico. Il precedente default del donor sceglieva prima un modello raw con SEA diversi: adesso il donor PI è richiesto e, se manca, il preflight fallisce senza ripiegare su un modello legacy. Un donor alternativo richiede riferimenti sano/statico forniti esplicitamente e coerenti con esso.

Prima di una build effettiva vengono salvate copie con hash di donor, modello sano di riferimento e motion statica in una cartella dedicata sotto `results/`. Tutti i soggetti della stessa esecuzione usano queste copie. Questo evita che, rigenerando AB06 in un batch, la sua conversione statica modifichi il riferimento che devono usare gli altri soggetti. Il manifest è incluso nella provenance del report di ogni build. `--validate-only` controlla i riferimenti e mostra il percorso obbligatorio senza creare questi snapshot o eseguire la preparazione.

Non serve un agente per calcolare le regolazioni o attivare il passaggio. I file comuni di riferimento devono essere disponibili nell'installazione; il preflight segnala quelli mancanti. L'esecuzione richiede Python/OpenSim con NumPy/SciPy e il plugin adatto ai SEA PI. Le verifiche del contatto dinamico e della control chain restano una fase di qualifica separata: rendere il fitting predefinito non dichiara pronti tutti i soggetti o tutte le installazioni.

Per il nuovo comportamento è stata eseguita una verifica `--all-subjects --validate-only` sui tre soggetti locali AB06/AB07/AB08: tutti selezionano gli stessi riferimenti e dichiarano registrazione e marker IK obbligatorie. Questa verifica non costruisce AB08 o rigenera AB06. Le nuove receipt sono sotto `validation/ab07_detector_investigation_2026_10_06/stage4_default/`. La suite finale passa **43/43 test**, 14 nuovi più i 29 precedenti, senza errori o skip e con fingerprint invariati durante il run. Il nuovo `validation/test_abx_default_ground_registration.py` controlla default, override, input mancanti, ordine delle fasi, arresto su errore e stabilità dei riferimenti durante un batch. Il risultato finale della suite è registrato in `stage4_default/default_tests.json`.

Le nuove modifiche riguardano il builder e i suoi test; algoritmo geometrico, profili, controllori, plugin e bundle operativo AB07 restano quelli già verificati. I quattro TODO sopra restano aperti e propagati nel daily.
