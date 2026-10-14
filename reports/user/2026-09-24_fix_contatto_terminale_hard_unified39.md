# 2026-09-24 — Conservazione del contatto nella terminazione hard della 39D

## Richiesta e risultato

Implementata la correzione richiesta prima del confronto fra peso di penetrazione
1 e 2. Quando la protezione hard interrompe il segmento, il simulatore conserva
la misura che l'ha attivata; l'ambiente la usa per i termini di contatto e la
registra come campione di un segmento incompleto. Il percorso è comune ai due
pesi. Nessun training avviato; l'override a peso 2 resta un candidato non attivato
nei preset.

## Problema

`OnlineGRFPenetrationLimitExceeded` trasportava soltanto il messaggio di errore.
L'ambiente intercettava l'eccezione e terminava correttamente l'episodio, ma
l'assenza del normale risultato di `step_until` svuotava il contatto. La safety
restava applicata, mentre penetrazione e momento GRF erano persi. Il terminale
non era quindi privo di penalità: mancavano i contributi dipendenti dalla misura.

L'[audit precedente](2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md)
descrive il comportamento storico; i suoi risultati e trace non sono stati
riscritti.

## Strategia e modifica

1. Il runner allega all'eccezione tempo fisico, lato, penetrazione misurata,
   limite e copia del contatto primario: forze, momenti, COP e sensori.
   Non ricava la misura dal testo né sostituisce la misura con la soglia.
2. L'ambiente recupera questo campione dopo aver saltato il trasporto del batch
   incompleto alla FSM. Calcola la penetrazione con la formula già vigente.
   Il momento alla caviglia viene recuperato soltanto se lo stato usato per la
   geometria coincide temporalmente con il campione.
3. L'esito rimane `grf_penetration_hard`, `terminated=True`, `truncated=False`:
   il contatto recuperato non lo rinomina come normale terminazione a 28 mm.
   Safety applicata una sola volta, nessun bootstrap dal terminale.
4. `segment_completed=False` distingue la transizione terminale da un segmento
   integrato completamente. Il contatore dei segmenti completati e il tempo
   logico dell'ambiente non avanzano; `terminal_contact.time_s` conserva il tempo
   fisico separato. Trace di rollout e telemetria conservano entrambi.
5. Picchi di penetrazione, forza e momento disponibili usano il tempo fisico.
   Gli RMSE cinematici escludono le righe dichiarate incomplete; l'errore
   cinematico terminale rimane assente, senza sostituirlo con quello precedente.
   Reward e fallimento rimangono contati.

Le eccezioni legacy con il solo messaggio continuano a terminare senza far
morire il worker: il contatto è esplicitamente `available=False`, con motivo
registrato. La misura non viene inventata. Nei file legacy senza flag di
completamento, la condizione resta sconosciuta e l'analisi conserva il conteggio
storico.

## Verifiche

Artefatti riproducibili in
[`hard_guard_contact_2026-09-24`](../../Trajectory%20Generator/unified_MLP_39/validation/hard_guard_contact_2026-09-24/).

- **24 test passati**: payload, serializzazione/pickle, indipendenza delle copie,
  soglie e lati applicati, step dell'ambiente, wrapper reward, telemetria e
  analizzatore.
- **15 test di preflight passati** con oggetti RLlib reali, senza cluster Ray
  né learner avviato.
- Con le soglie di produzione 20/28/30 mm, una misura controllata di **31 mm**
  produce loss **1,890625**. Il wrapper applica costo 1,890625 con peso 1 e
  3,78125 con peso 2; stessa transizione e stessa safety.
- Verificata proiezione del momento con centro articolare non nullo: −16 Nm,
  tolleranza 8 Nm, loss 1. Se il timestamp della geometria non coincide, il
  termine viene dichiarato indisponibile, mantenendo la penetrazione misurata.
- **Smoke OpenSim reale**, configurazione LEFT allineata, actor 39D, osservazione
  completa 88D, FSM v3: cinque segmenti nominali per braccio su istanze nuove
  producono osservazioni, reward, termini preesistenti, FSM e contatti
  **esattamente uguali** fra lo `step` salvato prima del fix e quello corrente.
- Nello smoke hard si abbassano esclusivamente in memoria le soglie del test
  per attivare rapidamente la protezione su un contatto reale. Nessuna misura
  viene sintetizzata. Verificati stesso contatto nei due pesi, differenza di
  reward uguale alla loss, timestamp coerente con lo stato fisico, contatore
  non avanzato e reset seguito da tre segmenti senza dati terminali residui.
  Questo è un test software del percorso, non una valutazione della robustezza
  della policy alla soglia di produzione.

Dettagli numerici e configurazione dello smoke in
[`live_smoke.json`](../../Trajectory%20Generator/unified_MLP_39/validation/hard_guard_contact_2026-09-24/live_smoke.json);
comando riproducibile nello script `verify_live.py`, log dei test nella stessa
cartella. Le copie `before/` e il relativo manifest distinguono il fix dalle
modifiche già presenti nel workspace.

## Limiti e conseguenze per il confronto

- FSM v3, soglie di produzione, formule/pesi dei preset, SEA e dinamica non sono
  stati modificati. Il runner root cambia soltanto il payload diagnostico
  dell'eccezione; la logica di training resta in `Trajectory Generator/`.
- Solo i termini di contatto recuperati hanno il nuovo timestamp fisico.
  Prescribed, governor e stato FSM conservano la semantica precedente del
  boundary logico: non si dichiara l'intera reward riallineata temporalmente a
  uno stato terminale completo.
- Il fix cambia il feedback dei futuri terminali hard anche usando checkpoint
  già esistenti, perché il codice base è condiviso. Non cambia pesi appresi né
  risultati storici. Entrambi i futuri bracci 1 e 2 devono usare questo codice;
  i ritorni hard precedenti non vanno confrontati direttamente senza distinguere
  la versione di contabilizzazione.
- La verifica attesta la correttezza del percorso software, non un miglioramento
  dell'apprendimento o della robustezza. Quest'ultimo richiede il confronto
  controllato successivo.
- Esecuzione verificata su macOS arm64; il codice non introduce dipendenze da
  macOS, ma la verifica live Windows non è stata eseguita.

## Osservazione separata emersa nello smoke

Il primo controllo effettuato riutilizzando la medesima istanza dopo reset ha
rilevato differenze soltanto nei diagnostici di escursione del ciclo knee/ankle
e nei relativi termini duplicati; osservazioni, reward e contatti coincidevano.
La verifica su istanze nuove è passata esattamente.

La revisione del codice ha individuato un comportamento preesistente:
`ProstheticPhaseFSM.reset()` azzera le escursioni ma conserva gli attributi degli
ultimi angoli, che il bootstrap del nuovo episodio può riutilizzare. È distinto
dal trasporto del contatto hard e non è stato corretto in questa modifica.
Il campione osservato è conservato in `nominal_mismatch.json`; dettagli in
`review.md`. La sua incidenza fuori dallo smoke non è stata quantificata.

## File modificati

- `simulation_runner.py`: eccezione e payload del campione che attiva la guardia.
- `Trajectory Generator/osim_trj_cmc_like.py`: recupero, validazione, distinzione
  segmento interrotto, priorità hard e controllo temporale della geometria.
- `Trajectory Generator/unified_MLP_39/rollout_eval.py`: metadati nel trace.
- `Trajectory Generator/unified_MLP_39/episode_telemetry.py`: campione e tempi,
  picchi, disponibilità, contatori e denominatori cinematici.
- `Trajectory Generator/unified_MLP_39/validation/analyze_episode_telemetry.py`:
  esclusione dei segmenti incompleti dagli errori cinematici e tempi espliciti.
- Tre test nuovi: `test_hard_guard_payload.py`, `test_hard_contact_reward.py`,
  `test_hard_contact_telemetry.py`; script e artefatti dello smoke nella cartella
  di validazione indicata sopra.
- Questo report e daily del 24 settembre.

## TODO

- [x] Recuperare e verificare la misura di contatto nel terminale hard con percorso
  comune ai pesi 1 e 2.
- [ ] Definire ed eseguire il confronto controllato LEFT72 peso 1 contro 2 con
  budget e criteri fisici comuni; nessun training avviato in questa attività.
- [ ] Verificare separatamente il reset dei diagnostici di escursione della FSM
  e la loro incidenza nelle configurazioni che li utilizzano.
- [ ] Eseguire la verifica live su Windows.
