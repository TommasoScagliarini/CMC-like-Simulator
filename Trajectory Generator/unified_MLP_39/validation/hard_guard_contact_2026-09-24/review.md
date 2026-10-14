# Revisione indipendente del recupero contatto al terminale hard

Data: 24 settembre 2026. Revisione conclusiva del design, dell'implementazione e degli artefatti di test degli autori, senza simulazioni, training o modifiche di produzione da parte del revisore. Esito: **PASS entro il perimetro del recupero del contatto hard**; resta separato il problema preesistente dei diagnostici FSM al reset, descritto in fondo.

## Esito del design

Il trasporto dell'esatto snapshot GRF primario attraverso `OnlineGRFPenetrationLimitExceeded` è appropriato per recuperare i termini di contatto persi nel segmento abortito. Va mantenuta la distinzione fra segmento fisico completo, transizione RL terminale e campione di contatto della guardia. Non è necessario modificare FSM, SEA, soglie o pesi della reward per trasportare il dato misurato.

Il producer esaminato conserva l'ereditarietà da `FloatingPointError`, accetta ancora argomenti legacy senza metadata, aggiunge campi opzionali e copia profondamente il payload. Al throw utilizza `_online_grf_info(grf)`, cioè il campione primario effettivamente misurato con schema flat left/right/sensors, non i sensori virtuali del detector.

## Geometria e tempo

Verificato in `SimulationRunner.step_until`: `state = self.state`; ogni sottopasso campiona GRF sullo stesso stato mutabile prima dell'avanzamento successivo. Il tempo del runner è aggiornato dopo ogni sottopasso precedente. Quando la guardia scatta, il wrench misurato e `runner.state` corrispondono dunque allo stesso stato fisico; la proiezione attorno alla caviglia non richiede una ricostruzione di geometria mancante. Questo vale per il percorso attuale di `step_until`, non per ipotetici futuri campioni intermedi su copie di stato.

È utile verificare nei test che `exception.time_s`, tempo dello stato e `runner.current_time` concordino, e che la proiezione usi quella geometria. Il sensore detector non può sostituire il contatto primario.

Mantenere `env.t` al precedente boundary preserva il resto della semantica storica, ma significa che il terminale **non è un segmento completo con tutti i termini sincronizzati**: contatto/forze appartengono a `terminal_contact_time_s`; phase target/governor/FSM restano al boundary logico precedente; alcuni valori fisici possono essere avanzati parzialmente. Il fix deve esplicitare questo limite, non dichiarare ricalcolata l'intera reward su uno stato terminale uniforme.

## Rischi e controlli necessari

- La FSM deve ricevere lo stesso percorso di aggiornamento fallito di prima, senza fabbricare batch di campioni o HS/TO. Il contatto recuperato va iniettato dopo quei passaggi, soltanto per osservazione e reward terminali.
- Il motivo hard deve mantenere precedenza sulla nuova terminazione ordinaria che la penetrazione recuperata potrebbe far rilevare. Safety si applica una sola volta; penetration e anti-flip vengono ricalcolati dalle misure effettive. Il wrapper della reward usa i termini, non deve sommare nuovamente il vecchio reward scalare dell'ambiente.
- `terminated=True, truncated=False` resta invariato: il learner non deve fare bootstrap sullo stato fisico parziale. La chiamata è comunque una transizione RL; `_step_index` dei segmenti completi non deve incrementare come se il segmento fosse stato completato.
- Eccezioni legacy e mocks con solo messaggio devono restare gestibili. Se lo snapshot manca, bisogna dichiararlo assente; non sostituirlo con penetrazione uguale alla soglia, contatto inventato o dati detector.
- `segment_completed=False` e metadata temporali devono arrivare fino al trace e alla telemetria persistente. `episode_telemetry.py` e il writer del rollout selezionano esplicitamente i campi, quindi la sola aggiunta a `info` non li preserva automaticamente.
- I picchi di contatto recuperati vanno registrati al timestamp fisico della guardia, mantenendo distinto il vecchio tempo logico dell'episodio. Gli analizzatori fisici devono continuare a riconoscere il segmento come parziale, anche se i termini di contatto non sono più zero.
- Reset e successivo episodio non devono conservare metadata o contatti terminali precedenti. Le nuove chiavi non devono cambiare dimensioni/ordine delle osservazioni o azioni.
- Controlli necessari: percorso normale invariato; terminale ordinario invariato; hard con snapshot misurato; hard legacy senza snapshot; copia/serializzazione del payload; caso forza/momento/COP noto con verifica quantitativa della penalità; persistenza metadata e reset.

## Conclusione

Non è emerso un impedimento nel design o un disallineamento geometrico nel percorso attuale. I rischi individuati sui writer e sui timestamp sono stati corretti e coperti dai test. Resta intenzionalmente distinta la data fisica del contatto abortito dal tempo mantenuto dagli altri componenti terminali. I test esaminati supportano il recupero corretto dei termini e l'invarianza nominale, senza dimostrare un miglioramento di apprendimento o robustezza.

## Revisione del consumer implementato

Il diff rispetto al backup `before/osim_trj_cmc_like.py` conserva integralmente le chiamate FSM e il loro ordine; il primario recuperato viene inserito successivamente. `_step_index` incrementa solo nel ramo in cui `runner.step_until` ritorna normalmente. Il terminale hard precede il motivo ordinario e mantiene `terminated=True`, `truncated=False`; l'unico `safety_loss` viene assegnato con una condizione OR, senza doppio addebito.

`RewardShapingWrapper.step` ricalcola la reward dai termini ed evita di riclassificare un hard come timeout. Verificato anche il codice RLlib installato: `GeneralAdvantageEstimation` passa `TERMINATEDS` a `compute_value_targets`, che azzera i valori terminali. Il campione parziale dunque non introduce un nuovo bootstrap nel training imitativo corrente.

La proiezione anti-flip controlla che `runner.state.getTime()` coincida col tempo dello snapshot, altrimenti dichiara il termine non disponibile. Questo evita di applicare il wrench a una geometria di un altro istante. Le chiamate normali mantengono la firma senza argomenti, preservando i mock legacy. Uno snapshot strutturato incoerente è rifiutato; un'eccezione legacy senza snapshot resta invece una terminazione valida senza dati inventati.

`_online_info_payload` ricalcola `online_gait` dopo l'iniezione: forza e contatto derivano dal primario recuperato; fase ed eventi restano quelli storici al boundary precedente. Non va descritto tutto `online_gait` come sensore temporalmente uniforme.

Le segnalazioni sulla telemetria sono state risolte: penetrazione, forza, moment loss e momento minimo recuperati riportano il tempo fisico hard, separato dal tempo FSM; i diagnostici esplicitamente non disponibili non creano picchi. Il fixture del test usa lo schema primario reale flat `left/right`, con `normal_force` e `penetration`. Il motivo di indisponibilità legacy viene preservato. I campioni parziali restano nella coda e nel ritorno RL, ma non vengono aggiunti ai denominatori degli errori cinematici dei segmenti completi.

## Risultati di verifica esaminati

Letti `tests.log` (24 test superati) e `telemetry_preflight.log` (15 test superati; non da sommare come test distinti senza controllare la sovrapposizione). Il test consumer esercita il vero `step` e il wrapper, sostituendo l'integrazione e l'acquisizione dell'osservazione: a 31 mm la penalità di penetrazione è 1,890625, la safety è conteggiata una volta e la proiezione di un wrench noto restituisce −16 Nm, con anti-flip loss 1. Controllati anche timestamp geometrico errato, payload incoerente e eccezione legacy.

Lo smoke OpenSim eseguito dall'autore confronta 5 step nominali su istanze fresche con il metodo `step` precedente e quello corretto: osservazioni, reward, termini preesistenti, FSM e contatto primario risultano esattamente identici. Verifica inoltre un contatto fisico reale attraverso la guardia, con soglie abbassate soltanto nell'istanza di test; le soglie di produzione restano 20/28/30 mm. La versione conclusiva di `live_smoke.json` riporta **PASS**, 43,804 s: la guardia interviene dopo un sottopasso realmente integrato, a 13,011 s, mentre il boundary logico resta 13,010 s. Il cambio del solo peso penetration da 1 a 2 produce una differenza 0,23921714223662605 contro l'attesa 0,23921714223662607, entro la tolleranza numerica del test, sul medesimo snapshot e con gli stessi termini grezzi. Dopo il terminale, reset e tre step successivi non conservano il contatto abortito. Questo smoke verifica il trasporto e il calcolo dei termini, non un miglioramento della robustezza appresa.

## Mismatch diagnostico del reset, già presente e fuori dal fix

`nominal_mismatch.json` conserva il primo confronto eseguito riutilizzando la stessa istanza. Il diff ricorsivo contiene esclusivamente `cycle_knee_excursion_rad`, `cycle_ankle_excursion_rad` e i corrispondenti termini diagnostici della reward nei primi quattro step; il quinto coincide. Osservazioni, reward scalare e contatti coincidono in tutti e cinque.

L'origine è leggibile nel codice FSM preesistente, non modificato da questo intervento: `ProstheticPhaseFSM.reset()` azzera gli estremi e le escursioni, ma non i due attributi `_current_knee_angle_rad` e `_current_ankle_angle_rad`, assegnati nell'update (righe 532–533). `reset_from_binary_baseline()` richiama `_bootstrap_partial_stance()` quando il reset avviene in contatto; quest'ultimo registra subito i due attributi attraverso `_record_cycle_kinematics()` (righe 916–920). L'ultimo angolo dell'episodio precedente può quindi inizializzare gli estremi della nuova stance parziale.

Si tratta di un trascinamento di stato nel reset FSM da registrare separatamente in backlog. Nella configurazione imitativa esaminata `phase_min_cycle_knee_excursion_rad=0`, gli apporti event/phase/morphology interessati hanno peso zero e queste due escursioni non fanno parte delle osservazioni; il test non mostra effetti sulla reward scalare o sulla dinamica. Non estendere questa conclusione a configurazioni future che usino tali diagnostici come vincoli. La ripetizione nominale su istanze fresche elimina il confondente e risulta identica. Nessuna modifica FSM è stata effettuata dal revisore.
