# Daily Report — 2026-10-06

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


**Stato:** giornata chiusa con `create_report` ed `end_day`, attribuita al **6 ottobre 2026** su richiesta esplicita dell'utente.  
**Instruction check token:** `CMC_AGENT_OK_2026`.

La chiusura documentale e l'ultima coppia di controlli sono state completate il 7 ottobre, ora Europe/Rome. I percorsi e i timestamp materiali `2026_10_07` sono conservati. Questa indicazione aggiorna il precedente stato parziale del daily; non modifica la chiusura storica del 5 ottobre.

## Esito finale e report consolidati

Il nuovo [report dei test preliminari V26 e del confronto controllato AB06/AB07](../user/2026-10-06_test_preliminari_v26_ab07_e_confronto_controllato_ab06.md) documenta problema, strategia, metriche, file aggiunti, verifiche e lavoro rimanente. Completa i quattro report già presenti del giorno:

- [Generalizzazione multi-modello EPIC V26](../user/2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md): priorità V26 imitativo → student/DAgger → ex-novo, actor senza descriptor anatomici/identità, corpus/split ancora da fissare, ramo unified39 temporaneamente sospeso.
- [Preparazione AB07](../user/2026-10-06_preparazione_ab07_seasea_pipeline_e_qualifica_v26.md): donor PI, fitting, correzione dei timestamp MATLAB, vera IK e prime verifiche sul bundle antecedente alla registrazione piede–terreno.
- [Correzione geometrica AB07 e default del builder](../user/2026-10-06_correzione_fitting_ab07_registrazione_piede_terreno.md): hardware e piano comuni, nuova IK, archive della baseline e registrazione obbligatoria per nuovi build ABx.
- [Calibrazione AB08 e qualità IK](../user/2026-10-06_verifica_calibrazione_ab08_e_qualita_ik.md): build completata senza smoke dinamico, calibrazione statica verificata, outlier già nel MAT e residui biologici ancora da investigare.

**Risultato principale dei nuovi test:** nel confronto omogeneo guidato dall'IK, AB06 ha impulso verticale −13,80% rispetto al proprio misurato e AB07 +39,99%. Il maggior scostamento AB07 persiste con lo stesso contratto di controllo. Modello, IK e carichi sperimentali cambiano insieme: non è stata isolata una causa anatomica o un parametro del contatto.

Le precedenti frasi sotto sull'attesa del via, su dinamica pendente o su zero-shot non eseguito descrivono fasi antecedenti della giornata. Sono state ricevute autorizzazioni specifiche per preparazione, correzioni, controlli e primo trasferimento; non è stata avviata una campagna di training multi-modello, una migrazione BW o un download del corpus.

## Preparazione e modifiche operative del giorno

AB07 usa il donor AB06 PI e controlli comuni. La registrazione statica hardware–terreno allunga il pylon di 17,579 mm e applica +4,731° solo alla posa statica di calibrazione; zero/range/comando articolare non sono alterati. La nuova IK completa ha 28.612 frame. L'archive conserva 19 file della baseline precedente. Il contatto rilevato nel replay filtrato passa da 12,597% a 99,938% dell'appoggio nominale; questo replay di pose resta distinto dalla successiva dinamica.

Il builder rende obbligatori la registrazione e una nuova IK marker: donor PI e riferimenti comuni sono selezionati automaticamente, senza fallback legacy. Il preflight AB06/AB07/AB08 passa, la suite finale registra 43/43 PASS; una build prepara copie comuni con hash per il batch, mentre `--validate-only` resta read-only.

AB08 è preparato con 28.580 frame, 8,630–151,525 s, ma senza integrazione dinamica. La calibrazione statica è corretta secondo il benchmark comune (RMS 0,223 mm). RMS dinamico medio 11,998 mm, lato protesico praticamente uguale ad AB06; la differenza principale riguarda il lato biologico destro. Il massimo 114,931 mm coincide con il salto originale di `R_Thigh_Upper`; rimuovere gli outlier a scopo diagnostico non spiega l'RMS medio. Non sono stati ripuliti i dati operativi o risolta la qualifica dinamica AB08.

File operativi modificati nelle fasi di preparazione già documentate: `tools/build_abx_seasea_pipeline.py`, nuovo `tools/fit_seasea_subject.py`, `tools/convert_epic_ab06_to_opensim.m`, `scripts/run_opensim_sea_pipeline.py`, geometria del centro/asse caviglia in `Trajectory Generator/osim_trj_cmc_like.py`, nuovo `tools/register_seasea_ground.py`, test/validator e bundle AB07/AB08. La sola geometria della caviglia preserva AB06 nelle pose testate; non cambia SEA o guadagni. I report dettagliati distinguono tali interventi dai diff preesistenti del workspace.

I successivi test qui consolidati aggiungono soltanto harness/audit sotto `Trajectory Generator/validation/`, risultati e plot. In questa fase controller, runtime operativo, plugin C++, profili, modelli, checkpoint e setup canonici restano invariati. Nessun training o nuovo solver IK è stato eseguito per i test e per questa chiusura documentale.

## Tre test preliminari dopo la correzione AB07

| Test | Esito temporale | Evidenza principale |
|---|---|---|
| 1 — AB07 IK-driven, modello ground-registered | Completed, 9,01 s, 901 segmenti / 9.010 campioni | 5 HS, 5 TO, 4 cicli validi; impulso +42,20%, onset HS −89,31 ms, TO +118,47 ms rispetto al misurato |
| 2 — best47 learned AB06 | Completed, 5 s, 500 segmenti / 5.000 campioni | 17 STO e 2 CSV identici allo storico; 500 osservazioni actor e azioni identiche; 3 HS, 3 TO, 2 cicli validi |
| 3 — stesso best47 learned AB07 | Completed, 5 s, 500 segmenti / 5.000 campioni | 1 ciclo actor valido; 1 HS revocato/invalid; impulso +13,86%, correlazione GRF 0,454; knee `u=+1` in 199 campioni |

Il +42,20% del test 1 usa confini post-step a 10 ms; il +13,86% del test 3 usa campioni preintegrazione a 1 ms. Sui primi 5 s il controllo IK-driven AB07 vale +48,657% a 1 ms oppure +48,725% sui confini a 10 ms. Le due percentuali non rappresentano lo stesso insieme di timestamp.

Checkpoint ex-novo: `Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best`, SHA di `module_state.pkl` **2d1d9b30f2b043f4b3b7b4347c06e9b3aace1c7d089b33e2893930554d84bd7c**. Actor 35 / osservazione completa 84, deterministico, seed 123. La riproduzione esatta AB06 attesta quel singolo contratto/runtime; non certifica altri start o Windows.

Nel test 3 i tre clock finali concordano e la penetrazione sinistra massima a 1 ms è 27,929 mm. Sono distinti 199 comandi SEA knee saturati, 621 flag del limite integrale cascade/202 anti-windup e due azioni policy clippate; non ci sono clamp motore/plugin PI. Tracking RMS actual/served knee/ankle 2,195°/0,646°, contro 15,685°/8,584° sulla IK originale. La riduzione dell'impulso rispetto al controllo IK-driven non implica miglioramento della waveform o qualifica della rete. Reward/corridor restano quelli AB06.

Sorgenti: [audit test 1 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-8850f6eb16146587>), [controllo AB06 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-39c61ea830c9e497>), [trasferimento AB07 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-047412ee2e9d7fe0>). Ogni rollout ha sette plot canonici nel root `plot/`. Finitezza e 50 identità SEA passano; MTP e reserve restano diagnostici scientifici distinti. Gli ingressi protetti sono rispettivamente 42/91/119; non sono stati aggiornati i pesi.

## Chiarimento delle divergenze e audit storico

“Prescribed” qui è CMC-like dinamico guidato da IK, senza rete e senza imporre le pose effettive. In entrambi i tipi di rollout la sinistra usa il contatto online applicato e la destra le GRF sperimentali. Il precedente confronto IK/learned cambiava anche action mapping, clock, slew e FSM: non isolava i pesi o il solo soggetto.

Il +42,2% del test 1 confronta l'impulso sinistro simulato con quello sperimentale AB07 sugli stessi timestamp; i 89/118 ms confrontano onset binario e nominali della GRF a 20 N. Non sono percentuali di errore rispetto ad AB06 o alla rete. Onset, conferma, consegna, crossing della forza e accettazione actor sono distinti. Actual/served e actual/IK sono distinti.

L'[audit salvato AB06 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-0036375171d9a8fc>) ricostruisce J1 teacher IK storico sui confini post-step a 10 ms: impulso −17,268%, HS −26,753 ms, TO +73,440 ms. Sono abbinati 3 HS successivi al reset su 4 nominali e tutti i 3 TO; l'HS al reset non è trasportato come nuovo evento. J1 usava absolute teacher, clock off, slew 2,5/2 e FSM diversa, quindi non era una baseline omogenea del test 1. Separatamente il learned AB06 ha impulso −18,440%, HS −33,753 ms e TO +48,440 ms, su supporto STO a 1 ms di 4,999 s: sono disponibili eventi actor/adattatore, non il journal binario completo. Nessuna nuova integrazione o inferenza nell'audit storico.

## Confronto omogeneo finale IK-driven AB06/AB07

[Riepilogo finale — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-f5f2644fc62adf6c>) e [selezione congelata — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-4cadbcfbce5304f4>). I nomi 7 ottobre identificano gli artefatti materialmente prodotti dopo mezzanotte, inclusi nella chiusura del 6 ottobre richiesta dall'utente.

I due controlli usano il contratto del test 1, nessuna rete, stessa integrazione a 1 ms, stessi SEA/controller, governor, filtri, limiti, clock, FSM e tre profili. Reset al primo HS misurato almeno 2 s dopo il t_start originale, con 9 s coperti nei dati. Un ciclo startup resta separato; i tre cicli primari sono scelti prima dei risultati. Le copie setup nell'audit cambiano soltanto t_end; l'offset selezionato evita il clamp del reset senza alterare i setup canonici.

| Misura | AB06 | AB07 |
|---|---:|---:|
| Finestra fisica richiesta [s] | 15,615960838–24,615960838 | 12,591740520–21,591740520 |
| Tre cicli nominali primari [s] | 17,163967986–21,822001051 | 14,087704688–18,690780190 |
| Impulso simulato / misurato [N·s] | 1.522,230 / 1.765,971 | 1.922,050 / 1.373,006 |
| Errore di impulso | **−13,802%** | **+39,988%** |
| Bias onset HS, 4/4 confini | −34,250 ms | −94,464 ms |
| Bias onset TO, 3/3 eventi | +98,653 ms | +122,130 ms |
| Correlazione GRF | 0,944 | 0,880 |
| RMS errore GRF/BW | 0,156 | 0,378 |

L'impulso percentuale è `100 × (somma I_sim / somma I_mis − 1)` sui medesimi tre cicli di ciascun soggetto. Le forze usano il rispettivo BW (masse 78,09885033 / 58,80765053 kg), verificato dai payload runtime. Nessuna waveform è traslata per migliorarne il confronto. Tutti gli onset attesi sono abbinati con tolleranze dichiarate, senza missing/extra primari; conferma e consegna restano separate. Tutti i tre cicli AB07 hanno sovraccarico (+31,70%–+45,06%).

**AB06 conserva `failed`:** i 17 STO hanno 9.000 campioni finiti e i tre clock raggiungono il termine fisico dei 9 s, ma il callback 900 non viene restituito. Dopo l'integrazione, la guardia binaria rifiuta 9,999999998868248 campioni rispetto a 10, errore 1,13175247e−9 oltre tolleranza 1e−9, cioè residuo −1,13175 ps. Il trace resta di 899 righe; non sono ricostruiti observation/reward/eventi finali. Non è una terminazione di contatto e i tre cicli primari sono anteriori e coperti. **AB07 completed:** 900 callback, termine normale. Nessuna soglia/runtime è stata cambiata per far passare la chiusura AB06.

Entrambi passano copertura/griglie/finitudine, 50 identità SEA e 6 controlli non tautologici. Penetrazione sinistra massima campionata a 1 ms: 22,967 / 21,620 mm; zero campioni oltre 28 mm. Il validatore generico conserva FAIL MTP in entrambi (RMS 13,997° / 10,828°). Nessun clamp del comando u, motore o plugin PI; flag del limite cascade knee in 1.096 / 1.459 campioni. Tali evidenze non certificano validità fisiologica.

44 input congelati invariati, 42 file nativi invariati nel postprocessing, 14 PNG revisionati e nessun canale mancante. I binari sono attestati durante il calcolo e ricontrollati alla fine, senza dichiarare un hash preflight retroattivo. Parametri SEA equivalenti: Kp XML 11.3 e 11.300000000000001 rappresentano lo stesso double.

Plot: [AB06, sette figure](../../plot/AB06_paired_prescribed_HS_9s_2026_10_07/10_07_2026_1/) e [AB07, sette figure](../../plot/AB07_paired_prescribed_HS_9s_2026_10_07/10_07_2026_1/). Titoli e manifest AB06 conservano l'errore terminale.

## File aggiunti per i nuovi test e verifiche della chiusura

Nuovi harness/audit: `verify_sto_numerics.py`, `compare_prescribed_runs.py`, `run_control.py`, `compare_control.py`, `verify_learned_rollout_numerics.py`, `run_transfer.py`, `compare_transfer.py`, `verify_transfer_rollout_numerics.py`, `analyze_learned_vs_measured.py`, `prescribed_J1/analyze_saved_prescribed.py`. Nel paired: `select_nominal_windows.py`, `run_pair.py`, `compare_paired_saved.py`, `verify_paired_numerics.py`, `render_prescribed_plots.py`, `consolidate_results.py`, setup derivati, CSV nominali e receipt/manifest/review. Sono tutti nel perimetro `Trajectory Generator/validation/`; source/harness preesistenti sono riusati e gli errori iniziali di postprocessing restano documentati.

Questa richiesta `create_report`/`end_day` modifica soltanto il nuovo user report e questo daily. Non avvia nuove simulazioni, test nativi, training o modifiche agli artefatti congelati. Le verifiche della chiusura controllano collegamenti, dati delle receipt, conservazione del backlog e propagazione dei TODO. Esecuzioni native macOS arm64; Windows live ancora aperto.

## Avanzamento dei TODO esistenti

- G01: ricevuti via specifici per i lavori descritti, non per l'intera campagna futura. G02: best47 congelato per questi test; selezione imitativo/student multi-modello ancora aperta.
- G03–G08: donor PI, interpreti Mac, fitting e vera IK AB07/AB08, folder/provenance indipendenti avanzati; manifest completo multi-trial, generalità e Windows ancora da completare.
- G09/G10 e AB07-Q01/Q02: controlli/replay/riesecuzione dinamica e confronto omogeneo conclusi, qualifica scientifica di carico/timing ancora aperta. La sola riesecuzione di Q02 è esaurita, il suo obiettivo di qualifica no.
- G12/G13: primo protocollo/test learned e controlli eseguiti come esplorazione; qualifica completa/multi-start/held-out non chiusa. G11 sul BW e tutti i lavori G14–G29 restano aperti.
- AB07-Q03/Q04: MTP e Windows aperti. AB08-Q01/Q02/Q03: marker, residui e dinamica tutti aperti. Il ramo 39D resta sospeso; il bug bootstrap PPO resta rinviato e distinto dall'errore binario terminale appena rilevato.

Conservate integralmente le 267 voci aperte precedenti (231 storiche, 29 G, 4 AB07-Q, 3 AB08-Q). Le tre nuove voci sotto sono identiche al nuovo user report. Nessuna checkbox costituisce un lancio automatico.

## TODO dei nuovi test, da propagare

- [ ] V26-T01 — Correggere e validare separatamente la guardia dei confini campionati del detector binario esposta dall'ultimo callback AB06, mantenendo il rifiuto dei batch realmente non validi e verificando casi di rounding/horizon su macOS e Windows; conservare receipt e trace failed del test originale.
- [ ] V26-T02 — Predisporre ed eseguire, quando richiesto, un confronto learned AB06/AB07 a checkpoint e contratto policy congelati, stessi criteri HS/startup/tre cicli nominali e durate comuni; la coppia IK-driven corrente non è quel test della rete.
- [ ] V26-T03 — Estendere la qualifica del trasferimento con start/condizioni e repliche dichiarati, misurando separatamente carico, HS/TO, knee clipping, margine di penetrazione, tracking, MTP e reserve; preservare best47 e tenere distinta la qualifica dai futuri training.

## Integrazioni precedenti della giornata, conservate

Verifica documentale finale PASS: collegamenti locali, corrispondenza dei nuovi TODO con il report e conservazione delle 267 voci precedenti, ora 270 aperte. Revisione indipendente delle basi temporali, dei numeri e della distinzione tra esecuzione e qualifica; nessun nuovo test nativo per questa chiusura.

Le sezioni seguenti conservano risultati e TODO delle fasi precedenti; lo stato finale e gli avanzamenti sono quelli documentati sopra. Le loro frasi di attesa o di assenza di nuovi rollout sono circoscritte alla rispettiva attività, non all'intera giornata chiusa.

**Nuova integrazione AB07 del 6 ottobre:** dopo la precedente attività documentale, l'utente ha autorizzato la pipeline AB07, ora eseguita. La sezione «Preparazione AB07 e verifiche prescribed» registra risultati e TODO attuali; le frasi precedenti sull'attesa del via e sull'assenza di esecuzioni restano la fotografia dell'audit iniziale. Questo aggiornamento tramite `create_report` non costituisce un nuovo `end_day`.

**Stato AB07 più recente:** su richiesta esplicita è stata corretta la registrazione geometrica statica del piede prima della calibrazione marker, con nuova IK completa e promozione in `models/AB07_SEASEA_Threadmill`. La sezione «Correzione del fitting AB07 e registrazione piede–terreno» documenta il nuovo modello; le simulazioni prescribed precedenti riguardano la baseline ora archiviata. Nessuna nuova integrazione dinamica, policy o training eseguita per questa correzione.

**Aggiornamento successivo del builder:** la registrazione geometrica è ora obbligatoria e predefinita per i nuovi build ABx, con donor PI e riferimenti AB06 comuni automatici. Questa modifica del codice è stata verificata tramite test e preflight; non ha rigenerato i bundle o avviato nuove pipeline.

**Aggiornamento di chiusura:** il successivo comando dell'utente `end_day (segnalo al 5/10)` ha registrato la chiusura della giornata nel [daily finale del 5 ottobre](2026-10-05_daily-report.md). Questo file conserva l'integrazione documentale materialmente completata il 6 ottobre e i TODO propagati; la data di completamento del report non è stata retrodatata.

## Report dettagliato di generalizzazione

Creato il [report completo V26 multi-modello](../user/2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md), che consolida la discussione e l'audit del 5 ottobre ed è stato completato il 6 ottobre dopo l'interruzione della stesura. La bozza con nome 5 ottobre è stata rinominata; non è un secondo report incompleto.

La nuova priorità dell'utente è la generalizzazione della catena V26 imitativa → student deployable → ex-novo a più modelli EPIC treadmill. Il ramo `unified_MLP_39` con oscuramento è temporaneamente sospeso; i suoi artefatti e TODO non vengono cancellati o dichiarati risolti. Il parent V26 nativo con 39 feature è distinto da quel ramo sperimentale.

Scelte e vincoli registrati:

- Prima istanza sui tre soggetti locali AB06/AB07/AB08, **in attesa del via esplicito** prima di preparazione, download, conversioni, rollout o training.
- Split per soggetto 70/30 oppure 80/20 da scegliere, con validazione e soggetti test esclusi da tutte le fasi di sviluppo. AB06 resta nello sviluppo.
- Continuazione proposta dei pesi imitativi V26 con episodi di più modelli e conservazione di AB06; poi student comune DAgger e PPO ex-novo sugli stessi domini.
- Actor finale senza descrittori anatomici del paziente o identificativo del soggetto; preprocessing e segnali realmente disponibili devono essere auditati.
- Morphology corridor attuale conservato in prima istanza; diagnostica di copertura e futuro adattamento separati.
- Nessuna domain randomization aggiuntiva nella fase 1; memoria/storia di osservazioni resta un'opzione futura.
- Primo test zero-shot a pesi congelati su nuovo modello proposto e ritenuto utile, **non eseguito**; richiede prima modello qualificato e controlli prescribed.

## Evidenze, lavoro svolto e limiti

L'audit precedente ha trovato la pipeline ABx documentata e 24 trial locali con componenti associati per nome: 8 AB06, 9 AB07, 7 AB08. Il discovery `--validate-only` eseguito il 5 ottobre passa sui tre soggetti, senza eseguire OpenSim. Non è stata verificata tutta l'integrità numerica dei MAT e non esistono nuovi risultati di generalizzazione.

Il builder usa IK in dry-run/fallback, conserva la geometria protesica donor e non gestisce ancora bundle indipendenti per tutti i trial. Il donor default raw differisce negli XML dal PI-tuned; i parametri effettivi devono essere verificati anche dopo gli override runtime. La normalizzazione del carico usa BW calcolato dalla massa OpenSim e richiede un riesame rispetto al vincolo sul paziente.

Oggi sono state completate la documentazione, la lettura mirata del preprocessing, l'identificazione delle sorgenti tramite hash e la revisione del report. I risultati nativi storici citati nei report di maggio/agosto/ottobre non sono stati rieseguiti. Le modifiche di questa richiesta riguardano soltanto report e daily; codice, modelli, config e checkpoint non sono stati modificati. Non sono stati avviati download, conversioni, simulazioni, rollout o training della nuova fase.

Verifiche documentali PASS: collegamenti locali esistenti, cinque completamenti C01–C05 e 29 TODO G01–G29 identici al report, conservazione delle 231 voci aperte univoche del daily precedente, sei hash sorgenti coincidenti e sette percorsi candidati presenti. Le revisioni indipendenti hanno confermato l'audit della pipeline e la ricostruzione della lineage V26; non costituiscono nuove prove numeriche.

## TODO del nuovo report, propagati con gli stessi ID


Gli ID permettono di riprendere il lavoro senza confondere proposte, preparazione e risultati. Le voci di esecuzione restano subordinate al via e ai prerequisiti; nessuna checkbox aperta costituisce un comando di lancio.

### 20.1 Completamenti documentali

- [x] C01 — Recuperare le decisioni della conversazione e distinguere il ramo 39D sospeso dalla catena V26 da generalizzare.
- [x] C02 — Verificare la presenza della pipeline ABx e identificare i limiti di IK, fitting donor e gestione multi-trial.
- [x] C03 — Chiarire continuazione dei pesi imitativi, ruolo dello student/DAgger ed ex-novo sullo stesso insieme di domini.
- [x] C04 — Registrare l'esclusione dei descrittori anatomici dall'actor e la dipendenza BW ancora aperta nel preprocessing.
- [x] C05 — Documentare corridor iniziale invariato, assenza di domain randomization nella fase 1 e attesa del via.

### 20.2 Priorità e preparazione del pilot

- [ ] G01 — Attendere il via esplicito dell'utente prima di download, preparazione dei nuovi modelli, conversioni, rollout o training; rispettare l'ambito concreto del via.
- [ ] G02 — Selezionare e congelare checkpoint imitativo, student/warm-start ed ex-novo di riferimento, con path, hash, config, feature, scaler e action bounds.
- [ ] G03 — Fissare donor, trasformazione protesica e parametri SEA/controller effettivi coerenti con la baseline, verificando anche gli override del runtime.
- [ ] G04 — Identificare interprete OpenSim e dipendenze operative; predisporre verifiche coerenti per macOS arm64 e Windows x86.
- [ ] G05 — Costruire il manifest dei tre soggetti locali e verificare contenuto/schema numerico dei MAT, condizioni, statici e corrispondenza modello–IK–GRF.
- [ ] G06 — Definire e verificare il fitting geometrico protesico per soggetto, distinguendo hardware fisso, regolazioni e convenzione di massa.
- [ ] G07 — Definire il percorso IK effettivo o qualificare esplicitamente il fallback dataset; evitare il falso completamento da dry-run.
- [ ] G08 — Rendere selezione sessioni/trial e destinazioni dei bundle indipendenti, con provenance e senza sovrascritture silenziose.
- [ ] G09 — Qualificare caricamento, reset, posa, contatto, stato SEA, tracking prescribed e reserve sui tre modelli prima della policy.
- [ ] G10 — Predisporre e qualificare profili GRF/detector per modello mantenendo dual-stream, coordinate LEFT e contratti degli eventi.

### 20.3 Ingressi, zero-shot e corpus

- [ ] G11 — Auditare tutti gli ingressi e il preprocessing dell'actor per disponibilità dalla protesi, inclusi BW, eventi, clock, soglie e reset; scegliere e verificare l'eventuale migrazione senza metadati del paziente.
- [ ] G12 — Fissare il protocollo zero-shot iniziale: modello, checkpoint congelati, preprocessing, reference, start/eventi, finestre, modalità della policy e metriche.
- [ ] G13 — Dopo via e gate fisici, eseguire i rollout zero-shot e i controlli concordati, con sette plot canonici per rollout sotto `plot/` e analisi delle cause degli esiti.
- [ ] G14 — Inventariare il corpus EPIC treadmill remoto, scaricare dopo via i componenti necessari e determinare soggetti/condizioni realmente eleggibili.
- [ ] G15 — Scegliere 70/30 oppure 80/20, definire validazione, assegnazioni/seed e regole contro leakage; conservare AB06 nello sviluppo e classificare i soggetti pilot come sviluppo se usati per le scelte.

### 20.4 Training e qualifica multi-modello

- [ ] G16 — Formalizzare la continuazione della rete imitativa V26: pesi/stato optimizer, budget, reward, target per soggetto e distribuzione multi-modello con conservazione di AB06.
- [ ] G17 — Implementare e verificare registro domini, sampler/scheduler, reset, factory/probe e schema effettivo dei worker per tutti i modelli.
- [ ] G18 — Qualificare il teacher/oracolo e la produzione di label nominali e recovery per dominio, con unità, stato, fase e contratto azioni coerenti.
- [ ] G19 — Parametrizzare collector/refit/DAgger multi-modello, bilanciare soggetti e recovery, misurare contributi alla loss/update e preservare le condizioni riuscite.
- [ ] G20 — Preregistrare ed eseguire la qualifica dello student comune su modello×condizione×start, distinguendo forma, served/actual, cicli, contatto, SEA e reserve.
- [ ] G21 — Conservare inizialmente il corridor attuale e analizzarne copertura/allineamento per modello; tenere distinta la futura revisione dai risultati sul test.
- [ ] G22 — Definire critic/optimizer e warm-up multi-modello, verificando immutabilità actor/logstd, valore non clippato, errori per dominio, Adam/KL e restore.
- [ ] G23 — Preregistrare ed eseguire un pilot ex-novo breve sugli stessi domini di training, con budget/gate e valutazioni per soggetto prima di estensioni.
- [ ] G24 — Svolgere il test finale sui soggetti esclusi e documentare fallimenti, costi di apprendimento, condizioni, variabilità e limiti della generalizzazione.
- [ ] G25 — Eseguire la verifica live Windows della pipeline scelta; le verifiche storiche del plugin PI non certificano la nuova campagna.

### 20.5 Lavori rinviati o condizionati a nuove decisioni

- [ ] G26 — Mantenere rinviato il fix bootstrap PPO; definirne in attività separata ambito e confronto prima di modificarlo o attribuirgli gli esiti della campagna.
- [ ] G27 — Riprendere l'adattamento del morphology corridor solo dopo la diagnostica iniziale e con dati/protocollo ammessi dallo split.
- [ ] G28 — Valutare in una fase successiva l'utilità di una storia causale di osservazioni/comandi, senza introdurre descrittori anatomici espliciti o identità del paziente.
- [ ] G29 — Considerare domain randomization soltanto in una fase successiva, con range fisicamente coerenti e protocollo separato.

## Preparazione AB07 e verifiche prescribed

Creato il [report AB07 SEASEA e qualifica V26](../user/2026-10-06_preparazione_ab07_seasea_pipeline_e_qualifica_v26.md), che documenta problema, strategia, modifiche, risultati, test e limiti. Il bundle richiesto è in `models/AB07_SEASEA_Threadmill`, con la struttura del bundle AB06 e provenance consolidata in `pipeline_verification.json`.

Il via dell'utente è stato ricevuto e utilizzato per preparazione AB07 e controlli prescribed. Non estende automaticamente l'esecuzione ad AB08, download, rollout con policy o training. Il lavoro precedente di generalizzazione e i suoi TODO restano conservati; le voci G01/G03/G04/G06/G07/G08 ricevono i seguenti avanzamenti nel solo ambito AB07, senza chiusura generale del pilot:

- Donor AB06 PI operativo; valori SEA e cascade Python conservati, 20 fingerprint di controllo/plugin invariati. Plugin C++ e guadagni condivisi non modificati.
- Fitting AB07 con due regolazioni di montaggio e hardware fisso; massa totale del sano conservata a 58,80765053 kg. Modello prodotto: 21 coordinate, 76 muscoli e 6 stati SEA.
- Convertitore MATLAB corretto per usare e validare i timestamp Header. Nuova IK marker OpenSim effettiva, 28.612 frame da 10,360 a 153,415 s; nessun fallback silenzioso nel percorso standard. Fit marker statico medio RMS 0,249 mm, distinto dall'RMS marker dinamico medio di 10,328 mm.
- Bundle e naming AB07 espliciti, hash e associazione dei profili registrati; esecuzione nativa verificata su macOS arm64. Verifica live Windows e altri soggetti/trial ancora distinti.

Sono concluse due simulazioni da 9,01 s, entrambe con 9.010 frame: CMC-like con GRF prescribed e diagnostico V26 con azioni dalla IK AB07, senza checkpoint. Tracking protesico actual/IK nel CMC: RMSE 0,281° ginocchio e 0,748° caviglia; actual/riferimento servito nel V26: 0,151° e 0,159°. Il governor modifica il riferimento raw: gli RMSE V26 contro IK raw sono 7,679° e 3,626°.

**G09/G10 restano aperti:** con profili V26 AB06 congelati si osservano 2 HS, 2 TO e un ciclo interno FSM contro quattro cicli nominali. Nessun HS è abbinato ai cinque nominali anche nella diagnostica a 120 ms; stance FSM 0,132–0,171 s contro 0,970–1,075 s. L'impulso verticale online sinistro è il 24,71% del riferimento, misurato sui confini salvati da 10 ms. È registrato un timeout swing a 17,650 s; `timeout_exceeded=0` nell'ultimo step è un flag transitorio, non assenza cumulativa di timeout. Associazione modello/profili dichiarata `not_qualified_for_AB07_contact_events`.

Rimane il FAIL biologico `mtp_angle_r`, con RMSE circa 11,22° sopra la soglia 10° e massimo circa 28,95° sotto la soglia 30°. Il problema era già documentato in AB06; la comparazione storica non è un confronto causale su finestre/config identiche. La SO soddisfa la coppia MTP e la reserve non satura. Nel V26 il deficit di carico online si accompagna a reserve maggiori nelle coordinate non attuate; il confronto non ne isola la causa perché cambia anche il riferimento protesico raw/served.

Verifiche eseguite: fitting 5/5, preparazione pipeline 7/7, wrapper V26 4/4, analisi offline 4/4, pose caviglia native 3/3, Header MATLAB 11/11, parità GRF Python/plugin entro 1,055×10⁻¹² N, integrità finale artefatti/fingerprint PASS. Validatore SEA CMC 38 PASS/7 WARN/1 FAIL; V26 36 PASS/9 WARN/1 FAIL, con FAIL MTP. Sette plot per ciascun run sotto `plot/AB07_prescribed_2026_10_06/` e `plot/AB07_v26_left_prescribed_2026_10_06/`. La scrittura del report non ha avviato nuove simulazioni.

### TODO AB07 propagati dal nuovo user report

Le voci seguenti conservano ID e testo del report. AB07-Q01/Q02 dettagliano il lavoro G09/G10, AB07-Q04 quello G25; non costituiscono un comando per lanciare una policy o un training.

- [ ] AB07-Q01 — Qualificare geometria del contatto, piano e riferimenti dei sensori su AB07, mantenendo comuni i parametri SEA, la control chain, le leggi FSM e le soglie degli eventi; rendere esplicito il binding per modello senza aggirare le guardie path/hash.
- [ ] AB07-Q02 — Dopo la correzione geometrica, rieseguire il diagnostico prescribed V26 con criteri dichiarati e verificare carico, HS/TO, durata stance, cicli e reserve; il completamento temporale non basta per qualificare AB07 prima della policy.
- [ ] AB07-Q03 — Investigare il limite biologico comune di tracking MTP, distinguendo osservabilità della IK e dinamica/SO, senza introdurre guadagni specifici per AB07.
- [ ] AB07-Q04 — Eseguire una verifica live della nuova pipeline AB07 su Windows con gli stessi parametri comuni e registrare gli esiti; le prove macOS e quelle Windows storiche AB06 non sostituiscono questa verifica.

## Correzione del fitting AB07 e registrazione piede–terreno

Creato il [report della correzione geometrica AB07](../user/2026-10-06_correzione_fitting_ab07_registrazione_piede_terreno.md), con problema, metodo, pipeline, file modificati, verifiche, hash e TODO. L'utente ha richiesto di fissare la posa del piede sul piano comune prima di calibrare i marker. Il buon fit marker precedentemente assorbiva un disallineamento della posa hardware, lasciando il tallone sopra il piano.

Il nuovo metodo `donor_relative_static_hardware_ground_v1` usa lo STL fisso del piede e la sola geometria del piano congelato per riprodurre le mediane statiche AB06: clearance minima **−2,136416 mm**, pitch **7,050423°**. Mantiene il mount anatomico superiore; regola il mount distale da **Y −0,385899299 a −0,403478429 m**, allungando il pylon di **17,579130 mm**. Applica **+4,730648501° soltanto alla posa statica della caviglia per calibrare i marker**, senza modificare zero/default/range del giunto, comando SEA o IK dinamica con un offset manuale.

Pipeline eseguita in isolamento: innesto anatomico → conversione MATLAB → registrazione statica → calibrazione marker → vera IK OpenSim → bundle. Nuova IK: **28.612 frame, 10,360–153,415 s**. Dopo verifiche, bundle promosso nella cartella richiesta; setup e metadata rigenerati con percorsi canonici. Vecchio bundle/XML/statici preservati in `validation/ab07_detector_investigation_2026_10_06/stage3/baseline/`, **19 file / 67,097 MB**, con archive manifest e hash verificati. I vecchi report, run e plot conservano il loro valore storico; non qualificano il nuovo modello.

Verifiche PASS:

- **29 test** di registrazione, fitting e preparazione, inclusi identità AB06, idempotenza, gradi/radianti, XML protetto, input/output e obbligo di nuova IK per il fitting registrato.
- **14 verifiche di artefatti** prima/dopo promozione; **20 fingerprint comuni e tre profili congelati** invariati. Modello nativo: 21 coordinate, 76 muscoli, 11 corpi, 6 stati SEA, 58,80765053 kg. Cambiano soltanto mount distale Y e posizioni dei marker sinistri rispetto alla baseline.
- Altri **3 test nativi di geometria caviglia PASS** sul bundle aggiornato. L'aspettativa AB07 è stata aggiornata dal vecchio mount anatomico numerico all'origine nativa del piede/pivot; il codice runtime resta invariato e usa il mount effettivo.
- Vera IK statica ulteriore su 9 frame / 28 marker peso 1: heel medio **−14,403204 mm**, contatto 100%, RMS marker **0,321232 mm**, parità esatta con il sampler di produzione.
- Intera nuova IK: contatto di almeno un sensore durante appoggio nominale **19,813% → 96,630%**. Tratto filtrato a 6 Hz / griglia 1 ms: **12,597% → 99,938%**, **5 HS / 5 TO / 4 cicli** contro 2/2/1 prima. Stance media 1,0885 s contro 0,1310 s prima; oracolo nominale medio 1,00658 s. RMS marker dinamico medio **10,327793 → 10,379907 mm**.

La verifica degli eventi è un **replay di pose salvate e della FSM del detector**, non un rollout dinamico o una valutazione dell'actor FSM v3. Rimane un HS confermato con anticipo **60,705 ms**, oltre il riferimento temporale storico di 50 ms: 4/5 confermati entro 50 ms, 5/5 consegnati entro 60 ms. Il TO aggiuntivo riguarda lo stance parziale iniziale. Quota STL e piano sono una convenzione relativa al benchmark AB06, non una misura indipendente del pavimento fisico.

La nuova associazione modello/profili conserva gli stessi tre hash e registra `ground_registration_and_saved_pose_contacts_verified_dynamic_qualification_pending`. Il nuovo `pipeline_verification.json` separa evidenze geometriche attuali da simulazioni precedenti. Nuovo modello SHA **6ea6dd779ecf88b69bc0cb940ed9e2d5225bb986a2d6f986920096ba574cc245**; nuova IK SHA **7f497b28c9c9483b042d6b84fbc4d9d897cfd6191eb63015599734b484f3e15a**.

File di codice: nuovo `tools/register_seasea_ground.py`, integrazione in `tools/build_abx_seasea_pipeline.py`, `validation/test_ground_registration.py`, `validation/verify_ground_registered_bundle.py`, `Trajectory Generator/validation/AB07/verify_ground_registered_ik.py` e test `test_ankle_geometry.py`; inoltre nuovi artefatti `stage3/`, bundle e documentazione. Nessuna modifica a plugin C++, controller, governor, FSM, parametri dei profili o fitting anatomico preesistente. Il plot diagnostico è in `plot/AB07_ground_registration_2026_10_06/`, disponibile PNG/PDF/SVG.

**TODO del nuovo report propagati:** gli ID e i testi AB07-Q01–Q04 sono identici alla lista già presente sopra e restano aperti. Q01 avanza sul fitting e sulla geometria, ma resta parziale per timing/binding dinamico; Q02 richiede nuova dinamica prescribed V26 con il nuovo modello/IK e valutazione del carico online, HS/TO, stance e reserve. Q03 riguarda il limite MTP, Q04 la verifica live Windows. G06 avanza solo su AB07; G09/G10/G25 restano aperti. Non si attribuisce al nuovo modello il precedente impulso online 24,71% e non si avvia una rete o un training con questo aggiornamento.

## Registrazione geometrica obbligatoria e predefinita nel builder ABx

Su richiesta dell'utente, il fitting geometrico statico è diventato il percorso standard: nessun agente deve attivarlo o scegliere a mano le correzioni. Il rischio nasce dal metodo di preparazione e può ripresentarsi su altri soggetti; non si assume che ogni modello abbia lo stesso errore numerico. Con gli stessi ingressi, AB06 conserva l'identità geometrica già verificata.

`tools/build_abx_seasea_pipeline.py` seleziona automaticamente il donor operativo **AB06 PI321/500**, il modello sano AB06, la motion statica convertita `static_01`, lo STL e il piano comuni. Le opzioni precedenti restano come override espliciti. Un donor alternativo richiede riferimenti sano/statico espliciti. Rimossi il percorso senza registrazione e il fallback alla IK remappata; `--skip-opensim-ik` è sempre rifiutato. Il precedente default donor preferiva XML raw con SEA diversi: adesso, se manca il PI corrente, il builder fallisce senza fallback legacy.

Prima di una build viene congelato un insieme di copie donor/healthy/static con hash e manifest sotto `results/`, usato da tutti i soggetti dello stesso batch. Questo evita che la conversione AB06 riscriva la motion di riferimento prima di preparare altri modelli. Le copie persistono per la provenance; `--validate-only` non le crea e non esegue preparazione, calibrazione o IK.

Preflight sui tre soggetti locali **AB06/AB07/AB08 PASS**, con gli stessi riferimenti e registrazione/marker IK dichiarate obbligatorie. Aggiunto `validation/test_abx_default_ground_registration.py`: default, portabilità del filename sano, override, donor alternativo, riferimenti mancanti, ordine delle fasi, arresto su errore e snapshot comune di batch. Suite finale **43/43 PASS**, 14 nuovi più 29 precedenti, zero errori/skip e fingerprint invariati. La suite completa e gli hash finali sono in `validation/ab07_detector_investigation_2026_10_06/stage4_default/default_tests.json`; le evidenze anteriori alla modifica, incluse versioni del builder/report/daily, sono preservate in `stage4_default/baseline/`.

Aggiornato il [report di fitting AB07](../user/2026-10-06_correzione_fitting_ab07_registrazione_piede_terreno.md) per distinguere la build precedente con opzioni esplicite dal nuovo default. Algoritmo geometrico, parametri SEA/controller, profili, plugin e bundle AB07 non sono stati modificati in questa fase. I riferimenti benchmark devono essere disponibili nell'installazione; serve l'ambiente OpenSim con NumPy/SciPy e plugin PI compatibile. La modifica non sostituisce la qualifica dinamica o la verifica live Windows. AB07-Q01–Q04 restano aperti con gli stessi testi già propagati sopra; G09/G10/G25 non vengono chiusi.

## Backlog precedente conservato dal 1° ottobre

Le voci seguenti sono propagate dal daily precedente, rimuovendo soltanto duplicati testuali esatti. Non costituiscono nuove autorizzazioni né un riesame integrale. Gli incarichi del ramo 39D sono temporaneamente sospesi con quel ramo; le formulazioni sulla creazione della variante 39D o sulla prosecuzione V1 possono essere storiche e già superate. Le scelte attuali nel report prevalgono: descriptor anatomici nell'actor esclusi, corridor corrente inizialmente conservato, randomizzazione rinviata e via alle esecuzioni ancora atteso.

Il fix bootstrap PPO resta esplicitamente rinviato. Il TODO generale EPIC ora ha priorità di workstream, ma il suo completamento scientifico non è attestato dalla disponibilità dei tre soggetti locali. Qualifiche Windows, repliche e debiti tecnici non vengono chiusi dalla sola stesura documentale.

- [ ] Verificare una calibrazione effettiva del critic, con distribuzione
  degli errori, incidenza del clipping VF e validazione indipendente;
  il solo warm-up eseguito non dimostra adattamento del valore.

- [ ] Definire e verificare un adattamento actor sufficiente agli ingressi
  oscurati, fissando budget e criteri prima di ulteriori esecuzioni.

- [ ] Approfondire copertura, allineamento e attribuzione del costo morphology,
  distinguendo traiettoria servita, cinematica fisica e disponibilità dei
  campioni prima di giudicare il beneficio della reward.

- [ ] Verificare robustezza su condizioni e seed ulteriori e svolgere la
  verifica live Windows. Il fix bootstrap PPO resta il TODO rinviato già
  riportato sotto; nessuna nuova esecuzione è avviata da questa integrazione.

- [ ] Correggere in un’attività separata il bug bootstrap PPO di Ray/RLlib 2.55.1: bootstrap eliminato ai confini artificiali dei frammenti e propagazione errata prima di una vera terminazione. La correzione è stata esplicitamente rinviata dall’utente; non è applicata a questo pilot. Dettagli nell’[audit del 29 settembre](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] Approfondire quantitativamente l’effetto dell’oscuramento prima di adattare la progressione o qualificare un checkpoint per il successivo training ex-novo. I rollout CP112/CP184 e il confronto specifico CP90 con i rami imitazione/ex-novo a cinque e dieci update sono conclusi; resta distinto il confronto esteso con i checkpoint di fine blocco CP115/140/165/190.

- [ ] Verificare robustezza su condizioni e seed ulteriori; i tre start restano esclusi dal pilot corrente per scelta dell’utente.

- [ ] Eseguire la verifica live Windows della pipeline scelta.

- [ ] Dopo il pilot, analizzare CP90/115/140/165/190 con valutazioni visibili e
  oscurate e quote effettive delle transizioni; i plot canonici restano sotto `plot/`.

- [ ] Risolvere successivamente il bug bootstrap PPO e svolgere verifica live Windows.

- [ ] Confrontare best90 fresh con best80 su finestra comune: tracking per giunto/fase, contatto, cicli, impulso, area positiva ankle e regolarità knee.

- [ ] Valutare robustezza con perturbazioni comuni a policy congelate e repliche indipendenti di training.

- [ ] Eseguire verifica live Windows della pipeline scelta.

- [ ] Isolare in un nuovo esperimento la conservazione del coefficiente KL durante il critic-only, con controllo sottoposto allo stesso trattamento; definire configurazioni e budget prima dell'esecuzione. Il fix non è implementato né il training predisposto.

- [ ] Verificare la sufficienza del riadattamento del critic con diagnostica degli errori e validazione indipendente; un aggiornamento non dimostra convergenza.

- [ ] Sperimentare separatamente regolarità della coppia knee e cinematica, mantenendo tracking e contatto come vincoli e senza modificare contemporaneamente le famiglie.

- [ ] Eseguire la verifica live Windows della pipeline e, prima di conclusioni sulla robustezza, repliche di training e condizioni di valutazione più ampie.

- [ ] Dopo l'ankle, sperimentare separatamente regolarità knee e cinematica
  usando lo stesso banco e controlli; non cambiare insieme le tre famiglie.

- [ ] Eseguire verifica live Windows e, se una candidata è promettente,
  repliche/condizioni più ampie prima di conclusioni sulla robustezza.

- [ ] Riconciliare la collocazione delle figure diagnostiche reward LEFT e
  coppie best80, assenti nei percorsi storici alla chiusura; se servono ancora,
  localizzarle o rigenerarle dagli script conservati. I plot canonici non
  sono coinvolti in questa mancanza.

- [ ] Eseguire lo screening con perturbazioni comuni a policy congelata di
  best80 e precedente last82 contro best72; includere eventuali nuovi
  candidati se ancora utili. Misurare completamento, cicli, penetrazione,
  carico, cinematica per fase e coppie. La parte deterministica è conclusa,
  quella rumorosa no; best80 non è ancora certificato come più robusto.

- [ ] Eseguire il ramo controllo LEFT72 peso 1, con lo stesso fix hard,
  protocollo di resume, inizializzazione e budget del peso 2, se si continua
  il confronto causale. Non usare ritorni di reward diverse come criterio
  comune e non attribuire al peso 2 l'effetto dell'intero insieme di cambiamenti.

- [ ] Distinguere il miglioramento in continuazione dalle cause del divario
  nell'apprendimento LEFT da zero; serve un controllo dedicato.

- [ ] Se si interviene sulle coppie, progettare un confronto dal medesimo
  checkpoint e con budget identico: regolarità della coppia knee e momento
  ankle sotto carico, con scale esplicite e verifica di tracking/contatto.

- [ ] Fissare criteri distinti per completamento, cicli validi, forma delle
  traiettorie e coppie prima della selezione e dell'oscuramento; il solo
  ritorno o completamento temporale non basta.

- [ ] Quantificare separatamente il trascinamento degli ultimi angoli nel
  reset dei diagnostici di escursione FSM e l'incidenza nelle configurazioni
  che li utilizzano; nessun fix FSM applicato oggi.

- [ ] Eseguire la verifica live Windows del fix hard, della ripresa e della
  pipeline scelta. Le prove odierne attestano macOS arm 64.

- [ ] Pianificare repliche di training e condizioni di valutazione sufficienti
  per comparabilità statistica/generalizzazione, distinguendo esperienza,
  target e reward. Nessuna equivalenza statistica V26 è già dimostrata.

- [ ] Se si considera RIGHT last60 per riuso/oscuramento, valutarla sugli
  stessi cinque seed già usati per best58 prima della scelta.

- [ ] Approfondire il transitorio seed 125 confrontando best35, last40, best58,
  last60 e V26 su reference, velocità, contatto e margine di penetrazione.

- [ ] Isolare su RIGHT l'origine del momento GRF invertito nell'ultimo
  appoggio prima di modificarne reference, governor o reward. La diagnosi
  locale LEFT non chiude automaticamente questo controllo.

- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.

- [ ] Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.

- [ ] Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.

- [ ] Risolvere e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start.

- [ ] Costruire dati diagnostici temporalmente allineati e valutare osservabilità delle 35 feature su episodi indipendenti.

- [ ] Scegliere fra curriculum con target mascherati, avvio supervisionato e possibili soluzioni causali di fase/memoria sulla base di evidenze.

- [ ] Se si procede con la maschera, definire il nuovo schema, la normalizzazione, l'eventuale indicatore di disponibilità e lo stato persistente del curriculum, conservando la V1.

- [ ] Implementare e verificare isolamento dei target, coerenza raccolta/PPO e indipendenza degli aggiornamenti ex-novo dalle copie dei target.

- [ ] Definire budget, seed, progressione e criteri di accettazione prima del confronto A/B/C; eseguire poi il pilot e le repliche necessarie.

- [ ] Validare i candidati completamente mascherati ai tre start, con metriche separate di avvio, tracking, cicli validi e protezioni.

- [ ] Qualificare il beneficio della variante scelta nel successivo ex-novo. Trasferimento tecnico, warm-up e resume sono verificati per il confronto CP90 documentato sopra; il recupero cinematico autonomo resta da dimostrare.

- [ ] Misurare in un nuovo run imitativo l'effetto del peso di penetrazione 1.0, senza attribuirgli gli esiti dello smoke storico a 0.05.

- [ ] Completare il benchmark e il verdetto di parità qualitativa secondo il report del 6 settembre, includendo costo di apprendimento e variabilità tra seed.

- [ ] Eseguire le verifiche live Windows della pipeline scelta; riprendere predittività e compensazione del ritardo soltanto in una versione successiva.

- [ ] Fissare checkpoint baseline, manifest degli input e protocollo comune
  di confronto, mantenendo intatti gli artefatti storici.

- [ ] Implementare il valutatore comune sotto `Trajectory Generator/unified_MLP/`,
  con metriche per start e per giunto, copertura dei cicli e gestione esplicita
  dei fallimenti e dei dati mancanti.

- [ ] Completare e riprodurre il benchmark baseline deterministico sui tre start.

- [ ] Definire prima della selezione finale margini di regressione, condizioni
  riservate alla valutazione, numero di repliche e budget di addestramento.

- [ ] Eseguire un vero pretraining e training ex-novo della rete unificata,
  con valutazioni periodiche; pilot imitativo a cinque iterazioni concluso,
  prosecuzione e training ex-novo completo ancora da eseguire.

- [ ] Eseguire il confronto deterministico e stocastico finale, includendo
  variabilità tra seed di training e incertezza delle differenze.

- [ ] Produrre il verdetto quantitativo di parità, regressione o risultato
  inconclusivo, insieme al costo necessario per raggiungere la qualità ottenuta.

- [ ] Eseguire le verifiche live su Windows per attestare anche quella piattaforma.

- [ ] Riprendere actor predittivo e compensazione del ritardo in una versione
  successiva; restano esclusi dal recupero della parità della V1 attuale.

- [ ] Implementare il teacher inverse-governor e il generatore delle label raw.

- [ ] Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`
  ankle su più cadenze e stati del governor.

- [ ] Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e
  recovery.

- [ ] Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.

- [ ] Ripetere il critic-only rewarm quando la reward diventerà realmente
  asimmetrica.

- [ ] Valutare una fascia soft interna e una corretta assegnazione causale del
  credito, senza spostare il corridor finale.

- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.

- [ ] Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con
  `sigma=0.010` isotropico e `freeze_logstd=true`, senza modificare J20 o il
  canonico lungo.

- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i
  rollout deterministici e i gate del pilot.

- [ ] Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM
  v3 in tutti i nuovi candidati.

- [ ] Rivalutare sigma dopo l'actor predittivo o una modifica della logica
  causale del morphology corridor.

- [ ] Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,
  dopo il recupero della qualità AB06.

- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione
  esplicita.

- [ ] Conservare sidecar J8 e artefatti storici senza modifiche retroattive.

- [ ] Riallineare in manutenzione separata le assertion legacy soggette a drift
  e gli altri debiti tecnici/amministrativi già propagati: osservabilità RLlib,
  `cycle_valid`, swing timeout, probe schema, pin V20, `best == last`,
  `fit_p2` V12R3, marker/audit storici e deflessione SEA iniziale.

- [ ] Completare V1 fino a100 e confrontare5/50/100.

- [ ] Creare la variante39 dopo valutazione100; superare isolamento e smoke.

- [ ] Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.

- [ ] Rendicontare tutti gli esiti e costi, distinguendo fattibilità e parità.

- [ ] Eseguire la verifica live Windows separata.

- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale 5/50/100.

- [ ] Svolgere la fase 5: variante 39, isolamento, smoke, confronto A/B/C e trasferimento ex-novo B/C secondo il protocollo.

- [ ] Completare la diagnostica dell'osservabilità e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime.

- [ ] Completare il benchmark di parità con la baseline, includendo costi e variabilità tra seed.

- [ ] Eseguire le verifiche live Windows; mantenere aperti i debiti storici non risolti.

- [ ] Integrare in un esperimento separato la correzione generale dei confini
  PPO, verificando connector completi, restore e riproducibilità. La patch
  di produzione non è stata applicata dall'audit.

- [ ] Quantificare separatamente la normalizzazione delle advantages sulle
  sole transizioni reali rispetto all'inclusione delle righe artificiali.

- [ ] Confrontare training RL standard e corretto con stessa inizializzazione,
  reward, budget e valutazioni; distinguere continuazione da best80 e training
  da zero, con repliche prima di conclusioni sulla stabilità.

- [ ] Proseguire la diagnosi di esplorazione, clipping, nominale/stocastico e
  feedback autoregressivo con interventi separati, senza tuning della reward
  sul solo AB06.

- [ ] Verificare altri start e tratti disponibili di AB06, recupero dopo
  perturbazioni e carichi; distinguere regolarità e forma della coppia senza
  imporre le waveform V26 come target a LEFT.

- [ ] Completare la verifica live Windows; verificare il trasferimento ad
  altri modelli quando disponibili. L'assenza attuale di altri modelli non
  impedisce l'indagine diagnostica sul modello disponibile.

- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).

- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).

- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).

- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

- [ ] Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).

- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).

- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).

- [ ] **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).

- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).

- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).

- [ ] Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

- [ ] Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).

- [ ] Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate. — [fonte 2026-08-27, riga 233](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici. — [fonte 2026-08-27, riga 353](../user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata. — [fonte 2026-08-27, riga 219](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).

- [ ] Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti. — [fonte 2026-08-27, riga 231](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11. — [fonte 2026-08-27, riga 232](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf. — [fonte 2026-08-27, riga 341](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).

- [ ] Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1. — [fonte 2026-08-27, riga 337](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).

- [ ] Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).

- [ ] Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

- [ ] Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint. — [fonte 2026-08-28, riga 250](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).

- [ ] Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati. — [fonte 2026-08-28, riga 299](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).

- [ ] Progettare e validare corridor cadence-conditioned su dati held-out. — [fonte 2026-08-30, riga 346](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] Valutare fascia soft interna e assegnazione causale del credito senza spostare il corridor finale. — [fonte 2026-08-30, riga 360](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] Verificare uno sweep 0–180 ms di anticipo della sola loss e il rapporto con il ritardo phase-normalized 135–160 ms; distinto dalla decomposizione già conclusa. — [fonte 2026-08-30, riga 376](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] Preregistrare il primo rapporto morphology knee/ankle asimmetrico. — [fonte 2026-08-31, riga 606](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] Validare frozen-policy gli orizzonti circa 90 ms knee / 100 ms ankle su più cadenze e stati del governor. — [fonte 2026-08-31, riga 603](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] Definire la loss imitativa raw-space/served-predicted-space per il nuovo teacher, prima di costruire la rete predittiva. — [fonte 2026-08-31, riga 604](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] Riprodurre il benchmark deterministico della stessa baseline sui tre start: la sola prova centrale storica non basta. — [fonte 2026-09-06, riga 247](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Confrontare nominale e stocastico con seed indipendenti di training e incertezza delle differenze. — [fonte 2026-09-06, riga 252](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Preregistrare margini, condizioni held-out, repliche e budget prima della selezione finale. — [fonte 2026-09-06, riga 248](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Fissare baseline, manifest e protocollo comune di parità; conservare gli artefatti storici. — [fonte 2026-09-06, riga 242](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Completare il confronto imitazione/ex-novo con valutazioni periodiche secondo il benchmark: il pretraining V1 è concluso, ma smoke e pilot non esauriscono il confronto finale. — [fonte 2026-09-06, riga 250](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Completare il valutatore comune con metriche per start/giunto, cicli, fallimenti e dati mancanti; i valutatori di singole campagne non attestano l’intero benchmark. — [fonte 2026-09-06, riga 244](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Rendere il verdetto quantitativo di parità/regressione/inconclusivo e il costo per raggiungere la qualità, senza confondere fattibilità tecnica con qualità biomeccanica. — [fonte 2026-09-06, riga 254](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Eseguire e attestare separatamente le verifiche native Windows della pipeline scelta. — [fonte 2026-09-06, riga 256](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] Completare la verifica dell’indipendenza degli aggiornamenti ex-novo dalle copie dei target. L’assenza di nuove copie nel suffisso privilegiato e la conservazione della maschera raccolta/PPO sono documentate; non equivalgono da sole al test specifico degli aggiornamenti. — [fonte 2026-09-14, riga 295](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] Validare candidati completamente mascherati ai tre start, con avvio/tracking/cicli/protezioni distinti; il primo pilot del 1 ottobre non esaurisce questa validazione. — [fonte 2026-09-14, riga 297](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] Costruire dati allineati e misurare osservabilità delle 35 feature su episodi indipendenti. — [fonte 2026-09-14, riga 292](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] Preregistrare ed eseguire A/B/C di osservabilità, trasferimento B/C e repliche: non confondere questo protocollo con le successive ablazioni clock/scala/slew. — [fonte 2026-09-14, riga 97](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).

- [ ] Confrontare curriculum, avvio supervisionato e soluzioni causali di fase/memoria con evidenze; l’adozione del masking non risolve da sola l’osservabilità. — [fonte 2026-09-14, riga 293](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] Verificare sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start. — [fonte 2026-09-14, riga 291](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] Verificare trasferimento, warm-up e resume della variante scelta e misurare il beneficio nel successivo ex-novo; correttezza tecnica e vantaggio di apprendimento sono distinti. — [fonte 2026-09-14, riga 298](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] Completare il confronto V1 5/50/100 sui tre start. Il training fino a100 è superato dalla continuazione fino a300; il confronto specifico 5/50/100 non risulta documentato e resta aperto. — [fonte 2026-09-14, riga 95](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).

- [ ] Manutenzione separata delle assertion legacy e debiti RLlib, cycle_valid, swing timeout, probe schema, pin V20, best==last, fit_p2 V12R3, marker/audit e deflessione SEA iniziale; chiudere solo con prova specifica. — [fonte 2026-09-15, riga 161](../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).

- [ ] **W02:** sostituire il controllo `os.kill(pid, 0)` del percorso attach con una verifica di vitalità non distruttiva e validarla su Windows. — [fonte 2026-09-15, riga 353](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **W03:** predisporre un percorso Windows per i runner J20 warm-up e restore R3, conservando la provenance storica e rendendo esplicito l'interprete della nuova esecuzione. — [fonte 2026-09-15, riga 354](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **W04:** canonicalizzare le chiavi dei percorsi nell'attestazione J21 e verificare su Windows i gate T12/T18 con artefatti integri e casi negativi. — [fonte 2026-09-15, riga 355](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **W05:** definire una politica dei fine-riga coerente con gli hash dei sorgenti unified e verificarla su un checkout Windows reale senza riscrivere i pin storici per nascondere differenze. — [fonte 2026-09-15, riga 356](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **W06:** rendere il generatore differito 39 compatibile con LF e CRLF e verificarne la materializzazione rispettando la separazione dalla V1. — [fonte 2026-09-15, riga 357](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **W07:** definire e verificare la ricollocazione di run, checkpoint e trace per il riuso delle valutazioni CP50/CP100 mantenendo le verifiche di identità del contenuto. — [fonte 2026-09-15, riga 358](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **W08:** distinguere nell'analisi sigma un report mancante da un sigma realmente nullo e verificare la rianalisi degli artefatti trasferiti. — [fonte 2026-09-15, riga 359](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] Fissare un ambiente Windows riproducibile con versioni Python/Ray/Torch/NumPy/OpenSim e dipendenze native, compatibile con i checkpoint scelti. — [fonte 2026-09-15, riga 361](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] Documentare e verificare root di checkout, supporto long-path Windows/Git e percorsi generati dal workflow selezionato. — [fonte 2026-09-15, riga 360](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] Verificare su Windows orchestrazione, valutazioni/report e, quando prevista, materializzazione della variante 39; attestare soltanto i percorsi realmente provati. — [fonte 2026-09-15, riga 363](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] Eseguire una verifica nativa Windows di caricamento plugin, reset/step, training breve con worker Ray, rollout, salvataggio/restore e warm-up, registrando gli esiti separatamente dalle evidenze macOS. — [fonte 2026-09-15, riga 362](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] Validare oracle LEFT nel loop fisico sui tre start; il controllo successivo su un solo start non chiude la richiesta. — [fonte 2026-09-20, riga 199](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] Valutazioni dedicate dei checkpoint nominali e stocastiche, con durata/tracking/contatto e un insieme held-out distinto dalla selezione. — [fonte 2026-09-20, riga 200](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] Ampliare la valutazione best72 oltre cinque seed per condizione: l’evaluation iniziale è conclusa, l’estensione resta aperta. — [fonte 2026-09-22, riga 156](../user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] Isolare con prove controllate gli effetti di traiettoria, risposta dinamica e distribuzione delle azioni; confrontare in rollout comando atteso dopo clipping e media clippata. — [fonte 2026-09-22, riga 157](../user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] Estendere confronti fail/success alla stessa fase e prima della soglia nei fallimenti di training, oltre ai due casi appaiati della 100. — [fonte 2026-09-22, riga 158](../user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] Esplicitare nei riepiloghi automatici clipping raw a monte di RLlib e clipping interno all’ambiente; la distinzione nel testo non attesta l’implementazione automatica. — [fonte 2026-09-22, riga 159](../user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] Preparare confronto causale RIGHT/LEFT da zero con FSM v3, ordine ingressi, inizializzazione, raccolta, Adam/resume e budget comuni; due storie separate non provano il solo effetto del target. — [fonte 2026-09-23, riga 123](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).

- [ ] Verificare separatamente reset dei diagnostici di escursione FSM e incidenza nelle configurazioni che li usano. — [fonte 2026-09-24, riga 140](../user/2026-09-24_fix_contatto_terminale_hard_unified39.md).

- [ ] Valutare best80/last82 e successivi candidati contro best72 con perturbazioni comuni e misure fisiche; il piccolo pilot reward non esaurisce questo confronto. — [fonte 2026-09-24, riga 102](../user/2026-09-24_avvio_left72_penetrazione_peso2_10iter.md).

- [ ] Valutare RIGHT last60 sugli stessi cinque seed prima di scegliere un checkpoint per masking. — [fonte 2026-09-24, riga 266](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] Isolare origine del momento GRF invertito nell’ultimo appoggio prima di cambiare reference/governor/reward. — [fonte 2026-09-24, riga 270](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] Approfondire transitorio seed125 su best35/40/58/60 e V26 con reference, velocità, contatto e penetrazione. — [fonte 2026-09-24, riga 268](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] Sperimentare separatamente regolarità knee e cinematica, mantenendo tracking/contatto come vincoli. — [fonte 2026-09-25, riga 214](../user/2026-09-25_banco_tuning_rapido_reward_left.md).

- [ ] Verificare sufficienza del riadattamento critic con diagnostica degli errori ed evaluation indipendente; un update non dimostra convergenza. — [fonte 2026-09-29, riga 114](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md).

- [ ] Isolare conservazione KL nel critic-only con un controllo sottoposto allo stesso trattamento, configurazioni e budget preregistrati; fix non implementato. — [fonte 2026-09-29, riga 113](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md).

- [ ] Quantificare normalizzazione advantages sulle sole transizioni reali rispetto a righe artificiali. — [fonte 2026-09-29, riga 377](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] Integrare in esperimento separato il fix generale confini PPO, verificando connector completi/restore/riproducibilità: replay isolato non è patch produzione. — [fonte 2026-09-29, riga 374](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] Confrontare RL stock/corretto con stessa inizializzazione/reward/budget/evaluation, distinguendo continuazione best80 e apprendimento da zero, con repliche. — [fonte 2026-09-29, riga 379](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] Valutare altri start/tratti AB06, recupero dopo perturbazione e carichi/contatti; distinguere generalizzazione da replay nominale. — [fonte 2026-09-29, riga 386](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] Valutare robustezza best90/best80 con perturbazioni comuni a policy congelate, distinguendo seed evaluation e repliche training. — [fonte 2026-09-30, riga 114](../user/2026-09-30_training_fresh_100iter_rollout_best90.md).

- [ ] Confrontare best90 fresh e best80 sulla stessa finestra con tracking per giunto/fase, contatto, cicli, impulso/area positiva ankle e regolarità knee. — [fonte 2026-09-30, riga 113](../user/2026-09-30_training_fresh_100iter_rollout_best90.md).

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).

- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

- [ ] **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).

- [ ] **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](../user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).

- [ ] **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](../user/2026-08-24_v26b_rev4e_r2_replay.md).

- [ ] **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).

- [ ] **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).

- [ ] **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](../user/2026-08-25_v26b_b1r2_diagnostica.md).

- [ ] **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).

- [ ] **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).

- [ ] **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).

- [ ] **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).

- [ ] **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).

- [ ] **Condizionato:** Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. — [fonte 2026-08-26, riga 184](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

- [ ] **Condizionato:** Verificare retrospettivamente l’equivalenza semantica del teacher prescritto sotto v3 nell’audit della collection storica. — [fonte 2026-08-26, riga 178](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

- [ ] **Condizionato:** Attestare il gate della collection teacher storica sotto v3 e corridoio corrente con la relativa evidenza. — [fonte 2026-08-26, riga 179](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

- [ ] **Condizionato:** Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop. — [fonte 2026-08-27, riga 256](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).

- [ ] **Condizionato:** Eventuale integrazione/riapertura dei rami LOTO, LOCO, B1R1/B1R2 con protocollo e autorizzazione; conservarne i verdetti storici, senza considerarli già operativi. — [fonte 2026-08-27, riga 343](../user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] **Condizionato:** Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18. — [fonte 2026-08-27, riga 265](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).

- [ ] **Condizionato:** Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato. — [fonte 2026-08-27, riga 376](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).

- [ ] **Condizionato:** Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione. — [fonte 2026-08-27, riga 267](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).

- [ ] **Condizionato:** Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale. — [fonte 2026-08-27, riga 227](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).

- [ ] **Condizionato:** Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati. — [fonte 2026-08-27, riga 192](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).

- [ ] **Condizionato:** Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding. — [fonte 2026-08-27, riga 332](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).

- [ ] **Condizionato:** Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori. — [fonte 2026-08-27, riga 247](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).

- [ ] **Condizionato:** Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy. — [fonte 2026-08-27, riga 431](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

- [ ] **Condizionato:** Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente. — [fonte 2026-08-27, riga 212](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).

- [ ] **Condizionato:** Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati. — [fonte 2026-08-27, riga 205](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).

- [ ] **Vincolo:** Conservare immutati sidecar J8 e artefatti storici. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] **Condizionato:** Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).

- [ ] **Condizionato:** La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento. — [fonte 2026-08-27, riga 435](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

- [ ] **Condizionato:** Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente. — [fonte 2026-08-27, riga 241](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

- [ ] **Condizionato:** Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati. — [fonte 2026-08-28, riga 276](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).

- [ ] **Condizionato:** Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget. — [fonte 2026-08-28, riga 1059](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

- [ ] **Condizionato:** Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training. — [fonte 2026-08-28, riga 1029](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

- [ ] **Condizionato:** Rivalutare sigma dopo actor predittivo o modifica causale del morphology corridor. — [fonte 2026-08-28, riga 1055](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

- [ ] **Condizionato:** Mantenere la served contro corridor retrospettivo HS→TO→HS come metrica finale event-aligned, senza alterare reward passata o spostare il gate per accettare ritardi. — [fonte 2026-08-30, riga 358](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **Condizionato:** Se si interviene sul governor, confrontare cutoff/slew ankle in un’ablation controllata a policy congelata; la misura dei lag correnti non prova il risultato di tale modifica. — [fonte 2026-08-30, riga 374](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **Condizionato:** Se si riprende questa strategia, implementare teacher inverse-governor e label raw da stato effettivamente visitato; validare il beneficio closed-loop. Pista distinta dal RL senza teacher. — [fonte 2026-08-30, riga 392](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **Condizionato:** Preregistrare gate quantitativi per toe-off, saturazione, FSM, tracking e condizioni held-out/multi-start prima di selezionare l’actor predittivo. — [fonte 2026-08-30, riga 394](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **Condizionato:** Preregistrare pilot PPO separato 5–10 update, sigma 0,010 isotropico e logstd congelato, conservando J20/canonico; non sostituirlo con pilot di altra ricetta. — [fonte 2026-08-30, riga 388](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **Condizionato:** Ripetere il critic-only rewarm quando la reward diventa realmente asimmetrica. — [fonte 2026-08-31, riga 346](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).

- [ ] **Condizionato:** Ogni nuovo run usa config/namespace separati senza alterare J20, canonico lungo o GO consumati; è un invariante, non una creazione da rifare ora. — [fonte 2026-08-31, riga 415](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

- [ ] **Condizionato:** Decidere eventuale estensione oltre 10 soltanto dopo rollout deterministici e gate del pilot J20. — [fonte 2026-08-31, riga 419](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

- [ ] **Condizionato:** Nel pilot preregistrato verificare dopo ogni iterazione penetrazione, FSM, loss, clipping/saturazione e deriva actor; valutare deterministicamente i checkpoint prodotti. — [fonte 2026-08-31, riga 417](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

- [ ] **Condizionato:** Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e recovery, se si riprende questa strategia. — [fonte 2026-08-31, riga 610](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] **Condizionato:** Riprendere actor predittivo e compensazione del ritardo soltanto in una versione successiva; attività differita. — [fonte 2026-09-06, riga 257](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **Vincolo:** Monitorare penetrazione 20/25/28 mm, served-vs-actual e FSM v3 in ogni nuovo candidato. — [fonte 2026-09-15, riga 152](../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).

- [ ] **Vincolo:** Preservare gli artefatti storici e distinguere portabilità dei pesi, migrazione dei risultati e riproduzione dell’esperimento. — [fonte 2026-09-15, riga 364](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **Condizionato:** Eventuali ulteriori prove da B devono isolare un fattore per volta. I successivi rami allineati modificano più impostazioni insieme; non attestano la chiusura delle ablazioni proposte né autorizzano il loro avvio. — [fonte 2026-09-20, riga 201](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **Condizionato:** Valutare il significato della guardia knee a 0°, distinguendo sicurezza, durata e fedeltà; eventuale modifica richiede prova separata. — [fonte 2026-09-20, riga 202](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **Vincolo:** Separare masking dalla costruzione di una baseline affidabile e fissare seed/criteri prima di rivendicare parità. — [fonte 2026-09-20, riga 203](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **Condizionato:** Eventuale screening appaiato dei checkpoint LEFT25/29/40 e geometria di contatto nei tratti problematici; i soli rollout CP40 non completano tutte le celle. — [fonte 2026-09-21, riga 151](../user/2026-09-21_risultati_unified39_v26_aligned_40iter.md).

- [ ] **Vincolo:** Valutare masking come passaggio distinto senza presumere che ex-novo risolva automaticamente la robustezza. — [fonte 2026-09-23, riga 278](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).

- [ ] **Vincolo:** Distinguere RIGHT da zero, adattamento LEFT, trasferimento V26 e masking/ex-novo; non attribuire risultati di un braccio agli altri. — [fonte 2026-09-23, riga 125](../user/2026-09-23_avvio_training_39d_riferimento_destro.md).

- [ ] **Condizionato:** Se prioritario, confrontare V26 e best72 con rumore assoluto comune, distinguendo target e policy media: sigma proprie con gaussiani comuni non isolano la sola esplorazione. — [fonte 2026-09-23, riga 273](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).

- [ ] **Vincolo:** Fissare criteri distinti per completamento, cicli, forma e coppie prima della selezione e del masking; non usare il solo return. — [fonte 2026-09-24, riga 272](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] **Vincolo:** Distinguere beneficio della continuazione da spiegazione del divario nell’apprendimento da zero, che richiede un controllo dedicato. — [fonte 2026-09-24, riga 309](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).

- [ ] **Condizionato:** Se si procede, eseguire il controllo LEFT72 peso 1 contro peso 2, con terminale hard corretto, stesso resume, budget e criteri fisici; il solo ramo peso 2 non isola causalità. — [fonte 2026-09-24, riga 307](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).

- [ ] **Vincolo:** Eventuali modifiche reward per le coppie devono usare stesso checkpoint/budget, tracking e contatto come vincoli e confronti separati knee/ankle. — [fonte 2026-09-24, riga 121](../user/2026-09-24_best80_cause_coppie_ankle_knee.md).

- [ ] **Vincolo:** Separare regolarità della coppia da forma legata a reference/carico, giudicando anche tracking e contatto senza copiare V26 o introdurre regole AB06. — [fonte 2026-09-29, riga 170](../user/2026-09-29_indagine_generale_39d_imitazione_e_forma_coppie.md).

## Aggiornamento: preparazione AB08 e verifica della calibrazione

Report: [2026-10-06_verifica_calibrazione_ab08_e_qualita_ik.md](../user/2026-10-06_verifica_calibrazione_ab08_e_qualita_ik.md).

La preparazione AB08 è terminata con codice 0 e bundle in `models/AB08_SEASEA_Threadmill`: nuova IK OpenSim dai marker, 28.580 campioni a 200 Hz, da 8,630 a 151,525 s. Il comando comprendeva `--skip-smoke-test`; nessuna simulazione dinamica, valutazione della rete o training è stata avviata. Parametri SEA identici al donor AB06 PI; controlli comuni, riferimenti AB06 e bundle AB07 invariati.

La verifica indipendente conferma calibrazione statica/geometrica corretta: sui 28 marker RMS 0,223 mm e massimo 0,616 mm, centroidi dei 12 marker sinistri coincidenti con modello/CSV. La registrazione automatica allunga il pylon di 19,477 mm e corregge la sola posa statica della caviglia di +2,509137°. I residui di altezza/inclinazione rispetto al benchmark AB06 sono numericamente nulli.

RMS medio durante il cammino: AB06 10,521 mm, AB07 corretto 10,380 mm, AB08 11,998 mm. Sul lato protesico AB08 è praticamente uguale ad AB06 (11,885 contro 11,836 mm); l'aumento globale riguarda soprattutto il lato destro biologico (11,568 contro 7,028 mm).

Il picco AB08 di 114,931 mm a 56,635 s deriva da un salto del marker `R_Thigh_Upper` già presente nel MAT originale: 123,546 mm in 5 ms, seguito da ritorno di 125,281 mm. Tutti i marker esportati nel TRC corrispondono al MAT entro precisione di stampa. La pipeline passa anche questi campioni alla IK, producendo impulsi nelle coordinate. Gli outlier spiegano i picchi ma non l'RMS medio: esclusione soltanto diagnostica dei frame >30 mm lascia 11,966 mm. Nessun dato operativo è stato corretto nell'indagine.

Evidenze in `validation/ab08_preparation_2026_10_06/` e `validation/ab08_calibration_investigation_2026_10_06/`; receipt consolidata `calibration_verification.json`. Ricalcoli mediante FK sulle pose salvate, controllo dei contratti XML e confronto MAT/TRC; nessun nuovo solver IK o rollout nell'audit. La verifica finale conserva 48 hash operativi/riferimenti. I TODO precedenti e la qualifica AB07 restano aperti secondo gli esiti già riportati.

### TODO AB08 da propagare

- [ ] **AB08-Q01:** Definire e implementare un controllo generale dei marker prima della IK, con politica esplicita per outlier/missing data, conservazione delle sorgenti e tracciabilità dei campioni trattati; ricalcolare e confrontare la IK senza cambiare parametri SEA/control chain.
- [ ] **AB08-Q02:** Analizzare separatamente i residui persistenti durante il cammino, soprattutto marker della coscia destra e `L_Shank_Upper`; verificare l'eventuale beneficio senza attribuire agli outlier la media RMS di 12 mm o introdurre correzioni specifiche AB08 prive di giustificazione.
- [ ] **AB08-Q03:** Qualificare AB08 su contatto/eventi V26, GRF online e tracking dinamico dopo la preparazione e gli eventuali interventi sulla qualità della IK. La build senza smoke test e gli audit statici non chiudono questa qualifica.
