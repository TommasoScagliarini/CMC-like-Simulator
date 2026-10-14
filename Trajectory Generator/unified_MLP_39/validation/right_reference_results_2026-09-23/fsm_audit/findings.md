# Perché i rollout RIGHT registrano un solo ciclo valido

Analisi offline del 23 settembre 2026. Nessuna simulazione aggiuntiva, nessun
replay controfattuale e nessuna modifica alla FSM. Fonti: trace dei rollout,
STO GRF a 1 ms e regole del codice `prosthetic_phase_fsm.py`.

**Il completamento dei 500 step non coincide con una sequenza di passo pulita.**
I due rollout RIGHT attraversano eventi HS/TO ravvicinati che vengono rifiutati
e in parte recuperati dalla FSM v3. I cicli coinvolti diventano degradati e
non incrementano il contatore. Non significa che la gamba compia un solo
movimento nell'intero episodio o che si fermi dopo il primo ciclo.

| Rollout | HS accettati | TO accettati | Cicli validi | Eventi invalidi | Resync |
|---|---:|---:|---:|---:|---:|
| RIGHT best35 | 3 | **6** | **1** | 2 | 2 |
| RIGHT last40 | 3 | **5** | **1** | 3 | 1 |
| LEFT last40 | 3 | 4 | 2 | 0 | 0 |
| LEFT best72 | 3 | 4 | 2 | 0 | 0 |
| V26 best87 | 3 | 4 | 2 | 0 | Campo non presente |

Gli HS di resync non incrementano `valid_hs_count`. Nel reset l'episodio è
inizializzato a metà stance: il primo TO chiude un segmento parziale, il primo
HS avvia il primo ciclo HS→HS completo. Questa convenzione è comune ai rollout
e non spiega il conteggio inferiore RIGHT.

## RIGHT best35

1. HS **13,887 s**, TO **14,587 s**, HS **15,574 s**: unico ciclo valido,
   periodo **1,687 s**, stance **0,700 s** e swing **0,987 s**.
2. TO **15,866 s**. Al boundary **16,000 s** viene registrato
   `hs_too_early_after_to`: anche usando il boundary, sono trascorsi soltanto
   **134 ms**, meno dei **200 ms** minimi di swing. Il timestamp originale
   dell'HS rifiutato non è conservato; può precedere il boundary.
3. Il detector binario resta in stance mentre la FSM è ancora in swing.
   Dopo **80 ms**, a **16,080 s**, viene applicato `heel_strike_resync`,
   ancorato all'inizio della discordanza, **16,000 s**. Il segmento e il ciclo
   vengono marcati non validi; il ciclo corrente diventa degradato.
4. TO **16,208 s**, HS **17,081 s**: l'HS viene accettato, ma il contatore
   non aumenta perché `cycle_reject_reason = cycle_degraded`. Riparte un
   nuovo ciclo pulito; non viene retroattivamente validato quello recuperato.
5. TO **17,382 s**. Nuovo HS precoce registrato a **17,560 s**, entro **178 ms**;
   secondo resync a **17,640 s**. TO **17,704 s**, poi l'episodio finisce a
   **17,990 s** senza un nuovo HS. Il ciclo finale resta degradato/incompleto.

Le righe `resync` e `cycle_degraded` non rappresentano necessariamente altrettanti
cicli fisici indipendenti: il secondo flag può essere conseguenza del primo.

## RIGHT last40

1. HS **13,883 s**, TO **14,284 s**; HS troppo precoce registrato a
   **14,400 s**, entro **116 ms**. Resync a **14,480 s**.
2. TO **14,626 s**, HS **15,585 s**: HS accettato, ma ciclo rifiutato perché
   precedentemente degradato. Il primo possibile ciclo completo non viene contato.
3. TO **16,089 s**, HS **17,103 s**: unico ciclo valido,
   periodo **1,518 s**, stance **0,504 s**, swing **1,014 s**.
4. TO **17,498 s**; HS precoce registrato a **17,670 s**, entro **172 ms**;
   a **17,740 s** compare `double_to_before_hs`. Non segue un secondo resync
   entro la fine. L'episodio termina senza un altro ciclo HS→HS completo.

La stance fraction 0,332 del ciclo valido last40 è inferiore al riferimento
soft 0,35, ma non è la ragione del ciclo mancante: il ciclo è infatti contato
come valido. Non confondere una metrica soft di reward con le regole di accredito
del contatore FSM.

## Il contatto è reale, ma il resync usa un segnale distinto

Il codice `binary_phase_adapter.py:215` passa al recovery il **latch funzionale
stance del detector binario**, non il semplice booleano forza primaria >0.
I sensori heel/toe legacy nel trace sono un'ulteriore sorgente distinta; non
basta guardarne uno per dichiarare errato il detector binario.

Durante gli 80 ms di discordanza che precedono il resync, gli STO delle forze
applicate confermano comunque un carico primario sostenuto:

| Finestra | Forza normale minima–massima | Media | Penetrazione massima |
|---|---:|---:|---:|
| best35 16,000–16,080 s | 90,5–175,7 N | 124,4 N | 6,68 mm |
| best35 17,560–17,640 s | 135,2–165,5 N | 145,9 N | 9,26 mm |
| last40 14,400–14,480 s | 83,9–108,0 N | 94,7 N | 7,31 mm |

Tutti gli 81 campioni di ciascuna finestra superano 50 N. Quindi il recovery
avviene mentre il piede è caricato, non durante una fase primaria completamente
aerea. Questo descrive lo stato visitato: **non dimostra da solo** se il problema
iniziale sia la cinematica appresa, un contatto transitorio compatibile con i
sensori, o una soglia/semantica del detector. Non dimostra nemmeno che il resync
sia la causa dell'irregolarità o che senza v3 l'episodio sarebbe terminato.

## Implicazione per la valutazione

La recovery preserva la continuità della FSM, ma non trasforma automaticamente
la sequenza recuperata in un passo valido. È corretto mantenere il risultato
positivo del **90,7% di episodi completi durante training**, qualificandolo come
robustezza rispetto alle terminazioni. La regolarità degli eventi nei due
checkpoint deterministici resta inferiore ai rollout V26/LEFT osservati.

La configurazione imitativa, uguale anche nei confronti storici, ha
`blend_phase_regular = blend_phase_event_progress = blend_landing_window_contact = 0`
e pesi di penalità degli eventi invalidi/contatto/clawback pari a zero. La
sequenza FSM resta osservata e può causare hard timeout, ma non esiste qui un
termine diretto che premi ogni ciclo valido o penalizzi questi resync. È quindi
possibile migliorare sopravvivenza e imitazione senza perfezionare già l'ordine
dei contatti. Questo non identifica un nuovo bug nella reward.

`metrics.json` conserva tutte le transizioni accettate e i loro timestamp,
i rifiuti, le finestre di carico e le configurazioni. `analyze.py` ricostruisce
l'audit e verifica che il numero di flag di ciclo completato coincida con il
contatore finale; `input_hashes.json` conserva i digest delle fonti.
