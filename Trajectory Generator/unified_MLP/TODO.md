# TODO della rete unificata

Ultimo aggiornamento: **2026-09-14**.  
Instruction check token: `CMC_AGENT_OK_2026`.

## Sincronizzazione senza clock prescribed — priorità alta

- [ ] Capire e gestire l'assenza di un segnale prescribed di gait cycle
  nell'actor durante il pretraining imitativo, mantenendo un contratto di
  osservazione coerente con l'ex-novo.

Richiesta dell'utente del 14 settembre: «Dobbiamo cercare di capire come gestire
adeguatamente l'assenza di un segnale prescribed per il gait cycle».

### Problema osservato nel codice

L'actor riceve 35 feature causali, con i due slot del clock prescribed a zero.
Il target della reward segue invece il tempo assoluto della simulazione:
`base_kin.get(t)` restituisce posizioni e velocità prescribed sinistre, senza
shift o riallineamento agli eventi online.

La fase online deriva dagli eventi sinistri heel/toe validati dalle FSM e
dall'ultimo periodo valido. Al reset il periodo è zero; prima di completare
un ciclo valido HS→TO→HS, le feature online sin/cos restano `(0, 1)`.
Non esiste un fallback prescribed. La FSM e gli altri segnali causali sono
già disponibili durante questo intervallo. Il target temporale continua
comunque ad avanzare e può disallinearsi dal movimento effettivo.

La reward guida gli aggiornamenti PPO ma non fornisce all'actor un'informazione
aggiuntiva quando deve scegliere l'azione. Se osservazioni indistinguibili
richiedono azioni diverse a causa della fase prescribed nascosta, aumentare
il training o il peso imitativo non garantisce di risolvere l'ambiguità.

### Riscontro sulle tracce dello smoke — 14 settembre 2026

L'ispezione in sola lettura delle tracce
`runs/rollout/smoke_v1_{early,center,late}/rollout_policy_trace.json` trova
500 + 294 + 295 = **1089 campioni** con periodo online zero e sin/cos online
sempre `(0, 1)`. Nel frattempo il target prescribed percorre circa 48.85° di
escursione knee e 30.48–31.82° ankle. Early resta in `WAIT_HS`; center e late
registrano un TO e sviluppano un timer di swing. I tre reset hanno posizioni,
velocità e stato FSM diversi: non è dimostrata un'ambiguità dell'intero vettore
di osservazione.

La policy esaminata è quella dopo lo smoke di tre iterazioni ridotte, non un
pretraining completo. Il dato evidenzia la difficoltà di avvio del clock,
senza dimostrare impossibilità di apprendere dalle altre feature. Le righe
delle tracce contengono osservazioni pre-step e target post-step: costanza ed
escursioni sono confrontabili qui, ma per costruire coppie di apprendimento
occorre allineare esplicitamente timestamp, osservazione e azione/target.

### Attività da svolgere

- [ ] Misurare ai tre start il tempo necessario per ottenere il primo periodo
  valido, gli episodi senza ciclo valido e l'evoluzione di fase/FSM/target
  durante l'avvio; separare avvio e regime.
- [ ] Verificare se le 35 osservazioni distinguono stati che richiedono azioni
  diverse: cercare osservazioni simili associate a target/fasi prescribed
  differenti e quantificare il disallineamento temporale. Le fasi prescribed
  di confronto sono diagnostica offline, non nuovi ingressi actor.
- [ ] Separare sufficienza delle osservazioni ed efficacia del training:
  confrontare su traiettorie indipendenti la predizione di azioni teacher da
  osservazioni istantanee, brevi storie causali e fase prescribed disponibile
  solo come controllo diagnostico. Nessun clock privilegiato va trasferito
  implicitamente nell'actor di produzione. Un buon fit offline va poi
  verificato in closed loop.
- [ ] Valutare un pretraining supervisionato dello stesso actor finale,
  eventualmente con raccolta di stati visitati dallo student, come alternativa
  o supporto al solo PPO imitativo. Le etichette devono essere comandi
  verificati nella catena slew/C2/SEA, non automaticamente gli angoli prescribed.
  Un teacher più informato non risolve da solo eventuali ambiguità degli
  ingressi student. È una proposta da valutare, non una pipeline implementata;
  non richiede di reintrodurre il passaggio storico 39D→35D.
- [ ] Confrontare possibili soluzioni causali per inizializzazione della fase,
  gestione del periodo non ancora disponibile e, se necessario, memoria.
  Nessuna soluzione o modifica architetturale è stata ancora scelta.
- [ ] Verificare la soluzione scelta in entrambe le modalità, ai tre start e
  su condizioni riservate alla valutazione, includendo disponibilità della
  fase, tracking served/actual, cicli validi e protezioni.

Il successo tecnico dello smoke non risolve questo TODO. La sufficienza dei
segnali per apprendere la sincronizzazione resta da dimostrare. Actor
predittivo e compensazione del ritardo restano attività distinte, rinviate
rispetto alla V1.

Riferimenti: [env.py](env.py),
[fase online del simulatore](../osim_trj_cmc_like.py),
[validazione iniziale](VALIDATION.md).

## Peso della penetrazione distinto per modalità

- [x] Impostare `grf_penetration_weight=1.0` nel pretraining completo e smoke,
  mantenendo `0.05` nell'ex-novo, secondo la decisione del 14 settembre 2026.
  Soglie 20/28 mm e contratto actor conservati; override registrato nei preset
  e in [configs/provenance.json](configs/provenance.json).
- [ ] Valutare in un successivo run imitativo l'effetto del peso 1.0 su
  apprendimento e penetrazione. Lo smoke del 6 settembre usava ancora 0.05.
