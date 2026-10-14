# Audit reward, FSM e terminazioni — 21 settembre 2026

## Esito

La formula scalare della reward non è una differenza nascosta fra i due esperimenti. Tutte le chiavi della configurazione reward storica sono uguali a quelle attuali. Su 2.000 step reali, la funzione storica estratta dal commit `68a62cd8` (17 agosto 2026) e la funzione unified attuale, alimentate con **le stesse loss e reference**, restituiscono reward esattamente uguali: errore massimo **0,0**. Verificato sui rollout 39D 25/29/40 e V26 best87. Questo non rende uguali gli obiettivi cinematici: le loss effettivamente prodotte dipendono da target e normalizzazioni differenti.

Il replay della FSM v2 e v3 sui quattro trace restituisce gli stessi stati e gli stessi otto valori di osservazione FSM in tutti i 2.000 step: errore massimo **0,0**, nessun repair. La minore accuratezza cinematica e il momento GRF sfavorevole dei rollout 39D non sono quindi prodotti da un intervento diretto della v3 durante quei rollout. Non è escluso un effetto della v3 sulla policy appresa durante il training, dove i repair sono effettivamente presenti.

## Differenze causali effettive

### FSM v3

`prosthetic_phase_fsm.py:1713` aggiunge il recupero dopo almeno 80 ms di contraddizione fra stato e latch funzionale di contatto. `:1808` revoca un heel strike giovane se seguito troppo presto da toe off. Il detector e il contratto degli eventi non cambiano.

Lo schema degli otto canali `observation()` (`:1872`) è identico. Dopo un repair possono invece cambiare i valori: stance/swing, evento atteso, tempo trascorso e credito di avanzamento. È quindi inesatto dire che v3 sia sempre invisibile alla rete. I contatori nuovi (`resync_count`, ecc.) restano diagnostici.

Le transizioni di recupero sono degradate: niente bonus di ciclo. Nell'esperimento imitativo allineato, però, i pesi dei bonus/penalità FSM, del clawback e della morfologia sono zero già nel V26 e nella 39D. Il degrado non introduce dunque una penalità scalare nascosta. Restano gli effetti sugli ingressi e sulle terminazioni: una FSM recuperata evita timeout altrimenti possibili e permette all'episodio di continuare. Questo modifica quali stati vengono visitati.

Nelle 40 iterazioni ci sono **133 righe di telemetria con resync**, distribuite su 31 iterazioni; non sono 133 episodi distinti. Le ultime 15 iterazioni non hanno timeout. Non disponiamo dei trace completi degli episodi stocastici di training: non si può ricostruire per ogni penetrazione se fosse preceduta da un repair, né attribuire causalmente tutte le penetrazioni alla v3.

### Guardie fisiche e trasporto degli eventi

Il diff `68a62cd8..c8d2ddca` di `osim_trj_cmc_like.py` documenta fix del 21 agosto, separati dalla v3:

- La penetrazione interna hard oltre 30 mm sollevava l'eccezione con `fail_fast` e poteva far morire il worker. Ora produce una terminazione MDP, `safety_loss` e una transizione conservata nell'apprendimento. Cambia la selezione dei dati e il trattamento del ritorno terminale, oltre ai contatori e ai batch effettivi.
- `reject_continue` prima poteva avanzare soltanto il cursor del sensore e lasciare indietro gli orologi FSM, con guasto allo step dopo. Ora scarta l'evento e committa un candidato temporalmente coerente; se non esiste candidato committabile termina l'episodio. Nel training attuale si contano **3** terminazioni `invalid_binary_event`.
- Il limite temporale adotta una tolleranza di mezzo passo invece di 1e-12 s per evitare un passo microscopico aggiuntivo a fine episodio. Questo conserva gli episodi completi anziché perderli per guasto di trasporto.

La soglia ordinaria di penetrazione **28 mm** è uguale. Nel blocco 36–40, delle 30 terminazioni per penetrazione **29 sono ordinarie e una hard**. Non è dunque corretto attribuire il divario recente 30 contro 4 alla semplice nuova etichetta hard. I fix possono aver modificato l'apprendimento precedente, ma serve un confronto fisico controllato per quantificarlo.

## File e verifica

- `audit.py`: audit riproducibile, nessuna modifica runtime.
- `audit.json`: parità numerica della reward, parità degli ingressi FSM nei replay puliti, contatori training.
- `actor_fsm_replay.json`: replay standard repository sui quattro rollout.

I replay riutilizzano gli eventi e la dinamica registrati; non costituiscono quattro nuove simulazioni chiuse. Il confronto v2/v3 riguarda esclusivamente la diagnosi offline e non modifica la FSM v3 concordata. Nessun training avviato.
