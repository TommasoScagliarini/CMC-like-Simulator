# AB07 SEASEA: preparazione della pipeline e qualifica V26 — 2026-10-06

**Esito:** preparazione del modello e dei dati completata in `models/AB07_SEASEA_Threadmill`; due simulazioni prescribed da 9,01 s completate con i parametri SEA e della control chain di AB06. **La qualifica del contatto e degli eventi V26 su AB07 resta incompleta.** Rimane anche il limite di tracking biologico MTP già documentato in AB06.

**Instruction check token:** `CMC_AGENT_OK_2026`.

La fonte consolidata è [pipeline_verification.json](../../models/AB07_SEASEA_Threadmill/pipeline_verification.json). Il PASS della [verifica finale degli artefatti](../../validation/ab07_pipeline_2026_10_06/final_artifact_validation.json) riguarda integrità, provenance e conservazione dei parametri; non equivale a un PASS biomeccanico complessivo.

## 1. Problema, richiesta e ambito autorizzato

L'obiettivo dell'utente è ottenere per AB07 la stessa pipeline operativa disponibile per AB06, usando una protesi e una catena di controllo comuni ai soggetti. Dopo la prima richiesta di sola verifica, l'utente ha autorizzato l'esecuzione della pipeline AB07 e indicato come destinazione `models/AB07_SEASEA_Threadmill`, con la struttura organizzativa del bundle AB06.

I vincoli rispettati sono:

- mantenere i parametri SEA e della control chain utilizzati in AB06;
- adattare il fitting anatomico ad AB07 secondo un criterio equivalente alla baseline;
- partire dal modello sano e dai dati del soggetto, produrre il modello protesico e ricalcolare la cinematica;
- conservare la separazione fra controllore Python e plugin SEA C++;
- salvare i sette plot canonici di ciascun rollout sotto `plot/`.

Il lavoro riguarda la sessione `10_14_18`, lo statico `static01` e il trial `treadmill_01_01` di AB07. Sono presenti nove trial treadmill AB07 nell'inventario locale, ma non sono stati tutti processati. Non sono stati eseguiti rollout con una policy appresa, caricamenti di checkpoint o training.

Questo report aggiorna l'audit preliminare nel [report di generalizzazione multi-modello](2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md): le sue osservazioni su dry-run, fitting mancante e attesa del via descrivono lo stato precedente a questa esecuzione. La qualifica complessiva del pilot AB06/AB07/AB08 resta distinta dal lavoro completato su questo singolo bundle.

## 2. Verifica del metodo AB06 e strategia adottata

La descrizione dell'utente è corretta nel punto essenziale: la cinematica sana EPIC appartiene al modello sano; dopo l'innesto e il fitting della protesi serve una cinematica calcolata sul modello protesico. La pipeline storica AB06 aveva effettivamente eseguito OpenSim IK dai marker, come documentato nel [report AB06 dell'11 maggio](2026-05-11_ab06_seasea_epic_pipeline.md).

Il limite trovato nel builder ABx generico era diverso: richiamava il runner in dry-run e poteva ripiegare sui dati sani rimappati quando mancava una nuova IK. Questo limite non dimostrava che AB06 fosse stato preparato soltanto rinominando le coordinate sane. È stata corretta la precedente interpretazione troppo ampia.

La catena eseguita è stata:

1. verificare modello sano, dati statici e dinamici, interprete OpenSim, MATLAB e plugin;
2. innestare la catena protesica sinistra dal donor AB06 operativo, preservando la convenzione di massa del benchmark;
3. regolare i montaggi protesici sulle dimensioni del soggetto;
4. convertire i MAT EPIC usando i timestamp di acquisizione e calibrare i marker sullo statico;
5. eseguire realmente OpenSim IK sul modello protesico calibrato;
6. costruire il bundle con nuova IK, GRF, ExternalLoads, reserve, task CMC e overlay healthy;
7. eseguire un controllo prescribed del simulatore e un diagnostico prescribed nel runtime V26, quindi analizzare SEA, tracking, reserve, contatto ed eventi.

Il modello sano EPIC è già personalizzato: non è stato applicato un secondo scaling biologico completo. Il fitting descritto sotto riguarda i montaggi della protesi. RRA non è stato eseguito né adottato; anche nella preparazione operativa AB06 il risultato RRA full-span non era stato adottato perché peggiorava i momenti residui.

## 3. Parametri comuni e modello prodotto

Il donor è [AB06_SEASEA_stiff321_500_pi.osim](../../models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim), SHA-256 `33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1`.

I valori numerici dei componenti SEA corrispondono ad AB06, inclusi i termini PI presenti nel plugin operativo:

| Parametro | Ginocchio | Caviglia |
|---|---:|---:|
| Stiffness, N·m/rad | 321 | 500 |
| Kp interno al plugin | 18 | 11,3 |
| Kd interno al plugin | 11 | 11 |
| Ki interno al plugin | 190 | 123 |
| Limite coppia integrale, N·m | 100 | 100 |
| Optimal force, N·m | 100 | 250 |
| Inerzia motore | 0,01 | 0,01 |
| Smorzamento motore | 0,1 | 0,1 |
| Kd_Imp, proprietà preservata | 5 | 10 |
| Impedence | false | false |
| Limiti del comando normalizzato | −1 / +1 | −1 / +1 |

La cascade Python conserva `kp_outer = 18,85 / 47,125`, `kp_inner = 29,2 / 2,8275`, `ki_inner = 1377 / 213` e limiti integrali `50 / 200`. Il passo di controllo/integrazione è 1 ms, con `rk4_bypass` e `sea_forward_mode=plugin`. Tutti i campi non legati al contesto di `SimulatorConfig` restano quelli AB06; cambiano i riferimenti al bundle, alla finestra temporale, agli output e, nel diagnostico V26, la modalità GRF e i binding dei profili.

Il confronto dei 20 fingerprint di controllo/plugin conferma che `config.py`, i controllori condivisi e il plugin SEA non sono stati cambiati durante questa attività. `CMC_Actuators.xml` e `CMC_Tasks - modified Kp_Kv.xml` sono identici byte per byte ai file del bundle AB06. Le prove sono nel [fingerprint iniziale](../../validation/ab07_pipeline_2026_10_06/control_fingerprint_before.json) e nella verifica finale degli artefatti.

Il modello AB07 prodotto contiene **21 coordinate, 76 muscoli, 11 corpi e 6 stati SEA**, per una massa totale di **58,80765053 kg**, uguale alla massa del modello sano AB07. Le coordinate coincidono con quelle AB06. I nomi dei 76 muscoli coincidono, mentre 33 posizioni nell'ordine del ForceSet differiscono: il simulatore usa associazioni per nome, verificate in questa attività. Questa verifica non certifica automaticamente la compatibilità di un checkpoint futuro.

### Fitting geometrico

È stato introdotto il metodo riproducibile `reference_relative_mounting_fixed_hardware_v1`, riferito al modello sano AB06 e al donor operativo. I rapporti anatomici AB07/AB06 sono `0,95443193` per il tratto superiore e `0,89744023` per la tibia. Sono state cambiate soltanto due traslazioni dei montaggi:

| Giunto e frame | Y originale, m | Y dopo fitting, m |
|---|---:|---:|
| `osseo / transfemur_offset` | −0,20750000 | −0,18857328 |
| `pros_ankle / tibia_pylon_offset` | −0,43000000 | −0,38589930 |

La distanza superiore fra centri passa da 0,41388946 a 0,39502932 m; quella dello shank da 0,43000000 a 0,38589930 m. L'helper conserva anca del soggetto, hardware osseo/ginocchio/piede, assi, orientazioni, range articolari, proprietà SEA, massa/COM/inerzie del graft e mesh. La calibrazione statica successiva modifica intenzionalmente i marker.

Le mesh visuali non sono state scalate. Massa e inerzie seguono la convenzione di redistribuzione già usata dal graft del benchmark: non sono una nuova identificazione dei parametri fisici dell'hardware. La qualifica del contatto resta necessaria anche dopo questo fitting. Dettagli e digest delle sorgenti sono in [pipeline_report.json](../../models/AB07_SEASEA_Threadmill/pipeline_report.json).

## 4. Correzione temporale, calibrazione marker e nuova IK

Durante la verifica è emerso un errore effettivo nel convertitore MATLAB: non riconosceva correttamente il campo `Header` e ricostruiva il tempo usando estremi/frequenze di fallback. Nel trial dinamico usava 10,359 s anziché il primo timestamp marker 10,360 s, alterando l'asse temporale; nello statico assumeva 100 Hz invece dei 200 Hz acquisiti.

Il convertitore ora privilegia il tempo di `Header` per table e struct, verifica lunghezza, valori reali e finiti e monotonia stretta, e rifiuta un Header malformato. Il fallback rimane per i dati privi di Header. La selezione dell'intervallo verifica anche la sovrapposizione positiva fra marker e condizioni. Dopo il fix conversione, calibrazione e IK sono state rigenerate; il bundle finale usa esclusivamente gli artefatti corretti.

| Dato finale | Righe | Frequenza | Intervallo, s |
|---|---:|---:|---|
| Marker / nuova IK dinamica | 28.612 | 200 Hz | 10,360–153,415 |
| GRF | 143.057 | 1.000 Hz | 10,359–153,415 |
| Statico | 436 | 200 Hz | 0–2,175 |

Il millisecondo iniziale aggiuntivo delle GRF garantisce copertura della IK; i timestamp iniziali dei due stream non sono identici. Il vecchio tentativo `IK_full_10p359_153p415.mot` resta fra i diagnostici, ma la provenance e il bundle puntano a `IK_full_10p360_153p415.mot`.

La calibrazione di 12 marker sullo statico produce un errore medio di fit RMS di **0,249 mm**, con massimo **1,397 mm**. La vecchia geometria marker aveva un errore medio di circa 92,098 mm: le due statistiche sono etichettate distintamente nella receipt.

La nuova IK è realmente calcolata sul modello protesico calibrato, senza fallback silenzioso. Rispetto alla cinematica sana rimappata, il ginocchio protesico cambia con RMSE **2,728°** e la caviglia con RMSE **4,245°**: non è una semplice rinomina delle coordinate.

Gli errori marker della IK dinamica sono distinti da quelli del fit statico:

| Metrica dinamica | Valore |
|---|---:|
| Media dell'RMS marker per frame | 10,328 mm |
| P95 dell'RMS marker per frame | 12,712 mm |
| Massimo RMS per frame | 28,859 mm |
| Massimo errore di un singolo marker | 120,722 mm |

Entrambi i massimi si verificano a **85,430 s**, fuori dai due rollout da 9,01 s. Il file aggregato non identifica il marker responsabile del picco; queste metriche documentano il risultato e non costituiscono da sole una qualifica dell'intero trial. Fonte: [bundle_validation.json](../../validation/ab07_pipeline_2026_10_06/bundle_validation.json).

## 5. Simulazioni eseguite e tracking

Entrambe le simulazioni integrano da **10,360 a 19,370 s**, per **9,01 s**. Ciascuna salva 9.010 frame a 1 ms, con ultimo timestamp STO 19,369 s perché il recorder registra prima del passo di integrazione.

| Run | Configurazione | Evidenza del completamento |
|---|---|---|
| [AB07 prescribed](../../results/AB07_prescribed_2026_10_06/) | Simulatore CMC-like, GRF prescribed, nuova IK AB07 | Stato run e output completi |
| [AB07 V26 prescribed](../../results/ab07_v26_left_prescribed_9p01s_2026_10_06/) | Runtime V26, azioni raw dalla IK AB07, GRF online applicate a sinistra | Receipt `completed`, 901 passi da 10 ms e output completi |

Il diagnostico V26 riusa la configurazione risolta di `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`; mantiene governor, limiti, leggi di controllo e profili di contatto/detector. Usa seed 123, reset deterministico e `imitation_initialize_to_target=false`. Le azioni sono ricavate dalla IK AB07 mediante `prescribed_action`; non viene caricata una policy. All'avvio posizione e velocità motore coincidono con quelle articolari, poi gli stati evolvono attraverso la dinamica SEA.

Gli adattamenti diagnostici di soggetto, durata, offset iniziale, registrazione e sorgente delle azioni sono espliciti nella [receipt V26](../../results/ab07_v26_left_prescribed_9p01s_2026_10_06/receipt.json). L'applicazione online sinistra esclude il corrispondente carico prescribed dalla dinamica; il lato destro rimane prescribed.

| Confronto, sui 9.010 frame STO | Ginocchio RMSE / max, ° | Caviglia RMSE / max, ° |
|---|---:|---:|
| Prescribed CMC: actual contro nuova IK | 0,281 / 1,631 | 0,748 / 3,295 |
| V26: actual contro riferimento servito dal governor | 0,151 / 0,754 | 0,159 / 0,863 |
| V26: actual contro IK raw | 7,679 / 23,360 | 3,626 / 13,908 |

Il buon inseguimento del riferimento servito conferma il funzionamento della catena SEA. La differenza maggiore rispetto alla IK raw mostra che governor e passaggio delle azioni modificano il riferimento: non si può usare il solo errore actual/served per affermare che la traiettoria raw sia conservata.

I comandi SEA non raggiungono la saturazione: nel prescribed CMC i massimi assoluti sono circa 0,330/0,290, nel V26 0,300/0,058. Gli stati motore non sono vincolati algebricamente al giunto, le derivate sono finite e la coppia interna del plugin coincide con la legge SEA ricalcolata indipendentemente entro gli errori di arrotondamento degli STO. Le reserve protesiche restano nulle.

## 6. Contatto ed eventi V26: qualifica ancora aperta

I tre profili V26 AB06 sono stati riutilizzati come candidati congelati: modello online primario, detector analogico e detector binario a due punti. Percorsi e SHA-256 sono conservati in [runtime_profile_association.json](../../models/AB07_SEASEA_Threadmill/data/runtime_profile_association.json), che riporta `not_qualified_for_AB07_contact_events`. Non sono stati adattati coefficienti di forza, geometrie dei sensori o soglie degli eventi per ottenere un PASS.

Il diagnostico registra 901 confini da 10 ms e 9.010 campioni binari da 1 ms. L'[analisi completa](../../results/ab07_v26_left_prescribed_9p01s_2026_10_06/qualification_analysis.json) confronta gli eventi con i cicli nominali ricavati dalle GRF misurate:

| Metrica | Riferimento misurato | Runtime V26 |
|---|---:|---:|
| Cicli completi nella finestra | 4 | 1 accettato internamente dalla FSM |
| HS nella sequenza dei cicli completi | 5 | 2 |
| TO | 4 | 2 |
| Durata stance | 0,970–1,075 s | 0,132–0,171 s |
| Frazione stance nei cicli nominali / ultimo ciclo FSM | 64,81–67,12% | 10,43% |
| Picco Fz sinistra, campionato a 10 ms | 625,305 N | 332,624 N |
| Impulso verticale sinistro, sui confini salvati | 2.749,462 N·s | 679,379 N·s |

L'impulso online è il **24,71%** del riferimento. La penetrazione massima primaria osservata sui confini da 10 ms è **10,224 mm**, sotto le soglie comuni di 20/28/30 mm. Il rispetto delle soglie di penetrazione non risolve il deficit di carico o la sincronizzazione degli eventi.

Il matching usa associazioni uno-a-uno entro la tolleranza, conservando eventi mancanti ed extra. Nessuno dei due HS coincide con i cinque HS nominali, sia con la tolleranza diagnostica V26 di 50 ms sui tempi confermati sia con il riferimento storico di 120 ms sui tempi di onset. Per i TO si trova 1 abbinamento su 4 a 80 ms e 2 su 4 a 180 ms. Queste tolleranze provengono da diagnostici AB06 con ambiti diversi: non sono state promosse a nuovi gate globali AB07.

La traccia registra inoltre **un timeout swing a 17,650 s**, con transizione FSM `SWING_AFTER_TO → TIMEOUT`: dalla TO a 15,040 s sono trascorsi 2,610 s, oltre il limite di 2,6 s. Il campo `timeout_exceeded` è un flag per step, azzerato a ogni passo, non un conteggio cumulativo: vale 1 al timeout e 0 nell'ultimo step, mentre lo stato finale resta `TIMEOUT`. La validità interna di un ciclo HS–TO–HS non certifica la sua corrispondenza con l'appoggio misurato. Fonte della semantica: [prosthetic_phase_fsm.py](<../../Trajectory Generator/prosthetic_phase_fsm.py>).

Il replay delle pose salvate, senza nuova integrazione, conferma un disallineamento dei punti binari rispetto al piano di riferimento già nel run prescribed CMC: il tallone risulta in contatto nell'8,42% dei frame e la punta nell'8,11%. Nel run V26 il tallone scende al 2,61%; la punta è circa il 29,56%. Le altezze sono quelle dei punti del profilo binario, non una misura completa della clearance della suola. L'evidenza orienta l'indagine verso geometria del contatto, piano e binding dei sensori; non identifica una causa unica né giustifica un offset arbitrario.

I profili originali hanno guardie runtime su path/hash. Copiarli sotto un nuovo nome AB07 non basta a creare un'associazione valida: la futura gestione per modello dovrà preservare e rendere espliciti quei contratti, senza aggirare le guardie. Il lavoro restante riguarda la qualifica geometrica del contatto/detector, mantenendo comuni SEA, controllori, leggi FSM e soglie.

## 7. Limite biologico MTP e reserve

Il validatore SEA del prescribed CMC restituisce **38 PASS, 7 WARN, 1 FAIL**; quello del V26 **36 PASS, 9 WARN, 1 FAIL**. In entrambi il FAIL è `mtp_angle_r`:

| Run | RMSE MTP, ° | Massimo errore, ° |
|---|---:|---:|
| AB07 prescribed CMC | 11,224 | 28,929 |
| AB07 V26 prescribed | 11,221 | 28,947 |
| AB06 storico, 14 giugno | 13,842 | 29,830 |

La soglia effettiva di FAIL è **RMSE ≥10° oppure massimo ≥30°**: AB07 fallisce per l'RMSE, mentre il massimo resta sotto 30°. Il [report AB06 del 14 giugno](2026-06-14_validazione_simulatore.md) documentava già questo limite. Finestre e configurazioni storiche sono diverse: il confronto attesta la presenza pregressa del problema, non un miglioramento causale AB07.

Il target MTP della IK è nullo; il giunto è libero e i marker del piede biologico sono associati al calcagno, senza marker sulle dita che osservino direttamente questa coordinata. I parametri e i path dei quattro muscoli flex/ext dig/hal destri sono preservati rispetto al sano AB07. Sul prescribed CMC, la static optimization soddisfa la coppia MTP richiesta con residuo nullo e la reserve raggiunge soltanto **1,807 N·m**: l'errore non si spiega con saturazione della reserve o infeasibilità della SO. La causa dinamica precisa resta da identificare.

Nel V26 il deficit di GRF online si accompagna a un maggiore carico sulle reserve dei gradi di libertà non attuati: sui confini da 10 ms la norma media delle reserve è circa **258,831**, di cui **253,414** sulle coordinate non muscle-driven. Queste norme aggregano forze e coppie secondo il diagnostico, quindi non sono una singola coppia in N·m. Il confronto non isola la causa dell'aumento, perché cambia anche il riferimento protesico raw/served. Il tracking del pelvis resta serrato, ma non dimostra che il contatto sia fisicamente qualificato.

I WARN dei validatori comprendono reserve protesiche nulle e tracking molto serrato. Nel V26 manca anche il file di stato specifico di `main.py`, perché gli output sono esportati dall'ambiente: il completamento è attestato dalla receipt dedicata e dalla copertura degli STO.

Fonti: [validazione prescribed CMC](../../validation/ab07_pipeline_2026_10_06/sea_simulation_validation.md), [validazione V26](../../validation/ab07_pipeline_2026_10_06/v26_sea_simulation_validation.md), analisi V26 e verifica finale degli artefatti.

## 8. File modificati e nuovi strumenti

| File / gruppo | Modifica attribuibile a questa attività |
|---|---|
| [tools/build_abx_seasea_pipeline.py](../../tools/build_abx_seasea_pipeline.py) | Destinazione e stem del bundle espliciti, fitting integrato, nuova IK effettiva obbligatoria nel percorso standard, provenance, propagazione degli errori; nessun fallback silenzioso dopo fallimento IK |
| [tools/fit_seasea_subject.py](../../tools/fit_seasea_subject.py), nuovo | Regolazione riproducibile dei due montaggi, controllo delle proprietà protette e dell'identità AB06 |
| [tools/convert_epic_ab06_to_opensim.m](../../tools/convert_epic_ab06_to_opensim.m) | Lettura e validazione dei timestamp Header, fallback soltanto in assenza di Header e controllo della sovrapposizione temporale |
| [scripts/run_opensim_sea_pipeline.py](../../scripts/run_opensim_sea_pipeline.py) | Percorso `--ik-only`, gestione dell'esito OpenSim, eliminazione dell'output atteso obsoleto e associazione esatta delle diagnostiche marker al MOT prodotto |
| [Trajectory Generator/osim_trj_cmc_like.py](<../../Trajectory Generator/osim_trj_cmc_like.py>) | Centro e asse della caviglia letti dal frame parent del PinJoint protesico, anziché dalla stazione tibia fissa a −0,43 m |
| [qualify_prescribed_v26.py](<../../Trajectory Generator/validation/AB07/qualify_prescribed_v26.py>), nuovo | Wrapper diagnostico senza checkpoint, binding AB07 e conservazione della configurazione/profili V26 |
| [analyze_prescribed_v26.py](<../../Trajectory Generator/validation/AB07/analyze_prescribed_v26.py>), nuovo | Analisi di eventi, carico e replay delle pose salvate con provenance e limiti espliciti |
| [validation/ab07_pipeline_2026_10_06/](../../validation/ab07_pipeline_2026_10_06/) | Validator bundle, diagnostica piede/piano, confronto healthy–IK–actual e receipt/log delle verifiche |
| Test in `validation/` e [Trajectory Generator/validation/AB07/](<../../Trajectory Generator/validation/AB07/>) | Regression fitting, preparazione pipeline, timestamp MATLAB, wrapper V26, geometria caviglia e matching offline |

La modifica alla geometria della caviglia conserva AB06 entro 10⁻¹² nelle pose testate; per AB07 corregge una differenza del centro di circa 44,101 mm. È verificato anche il caso con asse orientato. Non cambia i guadagni SEA né il plugin.

Il workspace conteneva già molte modifiche. In particolare `osim_trj_cmc_like.py` e `simulation_runner.py` erano già modificati prima di questa attività; il report non attribuisce l'intero diff Git al lavoro AB07. `prescribed_action.py` era già presente ed è stato utilizzato senza modificarlo.

## 9. Test e verifiche eseguite

| Verifica | Esito | Ambito |
|---|---|---|
| Fitting protesico | 5/5 PASS | Identità AB06, fitting AB07, proprietà protette e riapplicazione |
| Preparazione pipeline | 7/7 PASS | Destinazioni, percorso IK effettivo, provenance ed errori |
| Wrapper prescribed V26 | 4/4 PASS | Configurazione e reset dichiarati, assenza di policy |
| Analisi offline eventi | 4/4 PASS | Matching e gestione di eventi mancanti/extra |
| Geometria caviglia nativa | 3/3 PASS | Pose AB06, centro AB07 e asse orientato; senza integrazione |
| Regression Header MATLAB | 11/11 PASS | Timestamp corretti e rifiuto degli Header malformati |
| Parità GRF Python/plugin C++ | PASS | Errore massimo 1,055×10⁻¹² N, sotto 10⁻⁸ N |
| Artefatti finali e fingerprint | PASS | File, copertura, riferimenti, hash e 20 fingerprint conservati |
| Prescribed CMC e V26 | Durata completata | Due integrazioni da 9,01 s; qualifica biomeccanica distinta |
| Tracking MTP | FAIL | Limite biologico ancora aperto |
| Contatto/eventi V26 AB07 | Non qualificati | Cicli, carico e timing insufficientemente coerenti |

Esecuzione live: **macOS arm64**, Python 3.10.20, OpenSim 4.5.2, NumPy 2.2.6, SciPy 1.15.2, MATLAB R2024a Update 3; interprete `/opt/anaconda3/envs/envCMC-like/bin/python`. Le modifiche mantengono percorsi configurabili e non introducono dipendenze progettate soltanto per macOS; la nuova pipeline non è stata eseguita live su Windows.

La scrittura di questo report non avvia nuove simulazioni. Le prove numeriche sopra sono quelle già eseguite durante la preparazione AB07.

## 10. Organizzazione degli artefatti e riproduzione

Il [bundle AB07](../../models/AB07_SEASEA_Threadmill/) segue l'organizzazione AB06:

```text
models/AB07_SEASEA_Threadmill/
  AB07_SEASEA_stiff321_500_pi.osim
  AB07_SEASEA_stiff321_500_pi_setup.xml
  data/
    IK_results_AB07_SEASEA.mot
    AB07_SEASEA_GRF_FullSpan.mot
    ExternalForces.xml
    CMC_Actuators.xml
    CMC_Tasks - modified Kp_Kv.xml
    runtime_profile_association.json
    healthy/
      AB07_treadmill_01_01_Kinematics_q.sto
      AB07_treadmill_01_01_Actuation_force.sto
  pipeline_report.json
  pipeline_report.md
  pipeline_verification.json
```

Raw convertiti e modello calibrato sono in `models/AB07_SEASEA-raw/`; log e IK nativa sono sotto `results/ab07_seasea_treadmill_treadmill_01_01*`. Gli output dinamici restano nelle due directory di run dedicate.

Sono presenti sette plot per ciascun run:

- [plot prescribed CMC](../../plot/AB07_prescribed_2026_10_06/10_06_2026_1/);
- [plot prescribed V26](../../plot/AB07_v26_left_prescribed_2026_10_06/10_06_2026_1/).

Il settimo plot confronta healthy, nuova IK e cinematica actual. Il V26 è etichettato come diagnostico prescribed senza policy. I cicli dei plot canonici sono quelli del riferimento GRF misurato: vedere quattro cicli nei grafici non significa che la FSM ne abbia riconosciuti quattro. Le receipt di generazione/revisione sono nella directory di validazione.

Comandi equivalenti a quelli eseguiti, dalla root del repository, usando l'interprete con OpenSim e il percorso MATLAB della propria piattaforma:

```sh
python -B tools/build_abx_seasea_pipeline.py \
  --subject AB07 --task treadmill --trial treadmill_01_01 \
  --static-trial static01 \
  --donor-model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim \
  --fit-reference-model models/AB06-raw/osimxml/ab06.osim \
  --bundle-dir models/AB07_SEASEA_Threadmill \
  --model-stem AB07_SEASEA_stiff321_500_pi \
  --python /opt/anaconda3/envs/envCMC-like/bin/python \
  --matlab /Applications/MATLAB_R2024a.app/bin/matlab \
  --matlab-nojvm --skip-smoke-test --force

python -B -u main.py \
  --setup models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi_setup.xml \
  --grf-mode prescribed \
  --output-dir results/AB07_prescribed_2026_10_06 --log

python -B -u "Trajectory Generator/validation/AB07/qualify_prescribed_v26.py" \
  --setup models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi_setup.xml \
  --stage left_applied --duration 9.01 --seed 123 \
  --output-dir results/ab07_v26_left_prescribed_9p01s_2026_10_06
```

I percorsi di interprete/MATLAB nel primo comando documentano l'esecuzione macOS e vanno sostituiti su Windows. Le continuazioni di riga sono sintassi shell POSIX. `--force` rigenera gli artefatti del bundle; per nuove prove usare destinazioni dedicate. Lo smoke breve del builder è stato saltato e sostituito dalle due verifiche dinamiche complete descritte sopra, non da un dry-run. Hash di modello, TRC, nuova IK, sorgenti e profili sono conservati nelle receipt collegate.

## 11. TODO da propagare nei daily

I punti seguenti specificano il lavoro residuo AB07; non chiudono automaticamente G09/G10 del pilot multi-modello e non costituiscono un nuovo lancio di policy o training.

- [ ] AB07-Q01 — Qualificare geometria del contatto, piano e riferimenti dei sensori su AB07, mantenendo comuni i parametri SEA, la control chain, le leggi FSM e le soglie degli eventi; rendere esplicito il binding per modello senza aggirare le guardie path/hash.
- [ ] AB07-Q02 — Dopo la correzione geometrica, rieseguire il diagnostico prescribed V26 con criteri dichiarati e verificare carico, HS/TO, durata stance, cicli e reserve; il completamento temporale non basta per qualificare AB07 prima della policy.
- [ ] AB07-Q03 — Investigare il limite biologico comune di tracking MTP, distinguendo osservabilità della IK e dinamica/SO, senza introdurre guadagni specifici per AB07.
- [ ] AB07-Q04 — Eseguire una verifica live della nuova pipeline AB07 su Windows con gli stessi parametri comuni e registrare gli esiti; le prove macOS e quelle Windows storiche AB06 non sostituiscono questa verifica.

Questi TODO sono riportati anche nel [daily del 6 ottobre](../daily/2026-10-06_daily-report.md). La preparazione del bundle e le due esecuzioni prescribed sono concluse; la readiness per un rollout della policy su AB07 resta subordinata alla qualifica fisica e degli eventi.
