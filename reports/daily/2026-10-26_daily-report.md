# Daily Report — 2026-10-26

> Daily ricostruito il 2026-10-29 dai report utente del giorno: `end_day` non
> era stato eseguito.

**Stato:** due stress test sintetici di best47, su fermo/ripartenza e su
variazioni di velocità, e una diagnosi del mancato arresto. Nessuna correzione
funzionale; checkpoint, preset operativo, plugin e sorgenti produttivi
invariati. Instruction check token: CMC_AGENT_OK_2026.

## Banco comune

Harness in `Trajectory Generator/validation/speed_transitions_2026_10_26/`:
`run_transition.py`, `fsm_probe.py`, `analyze_results.py`, `render_canonical.py`.
Configurazione: best47, preset ex novo corrente, actor a 35 feature, FSM
binaria V26 e FSM actor v3, inferenza deterministica, seed 123, AB06
treadmill, avvio a 13,946871 s.

Come si simula il cambio di velocità:
- una mappa temporale regolare modifica la cinematica biologica, con
  derivate calcolate con la regola della catena;
- GRF destre prescritte, COP e momenti seguono la stessa mappa;
- la velocità della superficie di contatto online viene scalata agli stadi
  RK4.

I fattori sono relativi alla superficie calibrata, non velocità validate in
m/s, e l'actor non riceve alcun comando di velocità. Gli hash dei 41 input
protetti sono invariati. Esecuzione solo su macOS arm64.

## Camminata → fermo → ripartenza

Report: [test camminata, fermo e ripartenza](../user/2026-10-26_test_camminata_fermo_ripartenza_best47.md).

Protocollo di 8,31 s:
- camminata fino a 2,307 s;
- rallentamento a zero in 0,5 s;
- fermo richiesto da 2,807 a 5,807 s;
- accelerazione in 0,5 s;
- camminata finale.

La pausa è stata scelta a priori in un doppio appoggio dei dati sorgente
(entrambe le forze verticali ≥ 100 N), senza imporlo alla protesi simulata.

**Esito: la sequenza si completa, ma il fermo non avviene.** Sono stati
completati tutti gli 831 step, con 4 cicli validi, nessun evento rifiutato,
nessun resync, nessun timeout e nessun clipping. Il controllo a velocità
costante fa 9,50 s e 5 cicli; prima della perturbazione le differenze
massime sono di 1,17e-5 rad nelle posizioni e 6,18e-5 nelle azioni.

Durante la pausa la protesi si muove:

| Giunto | Escursione esclusi i primi 0,5 s | Velocità RMS nell'ultima finestra |
|---|---:|---:|
| Ginocchio | 0,660 rad (37,8°) | 0,91 rad/s |
| Caviglia | 0,518 rad (29,7°) | 0,67 rad/s |

Un intero ciclo valido (3,409–5,625 s) cade dentro la pausa. Altri dati:
- penetrazione massima 24,87 mm, sotto la guardia di 28 mm;
- minimo del bacino 0,95242 m;
- `reserve_norm_nm` 778,68 contro 529,18 del controllo. La metrica aggrega
  anche reserve traslazionali in N, quindi l'etichetta N·m era impropria.

La sonda `fsm_probe.py` mostra un limite distinto: con contatto stabile la FSM
actor entra in `TIMEOUT` 2,209 s dopo l'ultimo HS. Manca uno stato di
stazione eretta. Il primo tentativo è stato escluso, perché il suo ultimo
segmento a 8,307129 s non rispettava la griglia del detector: la durata è
stata allineata a 8,31 s e la prova ripetuta.

## Variazioni di velocità del treadmill

Report: [test variazioni di velocità](../user/2026-10-26_test_variazioni_velocita_treadmill_best47.md).

Protocollo di 11,50 s:
- nominale fino a 2 s;
- rallentamento 1 → 0,8;
- plateau lento di 3 s;
- accelerazione 0,8 → 1,2 fra 5,5 e 6 s;
- plateau veloce di 3 s, ritorno a 1 e nominale finale.

**Esito: la sequenza non si completa.** Terminazione dopo 593 step, a
5,930 s, per `grf_penetration` (28,187 mm, guardia 28 mm) durante
l'accelerazione, con fattore 1,1912. È l'attivazione di una guardia, non un
crash, e da sola non prova una caduta. Al momento della terminazione la FSM
è in `STANCE_AFTER_HS`, con 3 HS, 3 TO, 2 cicli validi e nessun evento
invalido. Il controllo nominale completa 9,50 s e 5 cicli, con penetrazione
massima 24,63 mm.

La cadenza cambia, ma il regime lento non è qualificato:
- i periodi nominali sono 1,535–1,592 s;
- i due cicli della prova durano 1,927 s (attraversa il rallentamento) e
  2,104 s (include 174 ms dell'accelerazione);
- il ritardo dell'HS reale rispetto alla temporizzazione biologica sintetica
  sale da 111 a 274 ms.

Con contatti ideali la FSM accetta le cadenze 1 → 0,8 → 1,2 → 1, quindi il
fallimento non dipende dal rifiuto degli intervalli da parte della FSM. Una
singola prova non separa però generatore, tracking, contatto e supporto
biologico. Non sono stati testati il regime a +20%, il recupero e rampe più
lente.

## Diagnosi del mancato arresto

Report: [diagnosi del fermo e della ripartenza](../user/2026-10-26_diagnosi_fermo_ripartenza_best47.md).

Quattro ipotesi, che possono coesistere:

| Ipotesi | Esito |
|---|---|
| Detector/FSM inventano passi | No: la binary V26 mantiene `BOTH` senza eventi. Manca invece un contratto di pausa nella FSM actor v3, che va in `TIMEOUT` |
| Rete non preparata al compito | Sì: training solo sulla camminata, con reward sugli eventi del passo e nessun ingresso di stop o velocità |
| Test impostato male | In parte: all'inizio della pausa la protesi è in `SWING_AFTER_TO`, con ginocchio a circa −53,7°; la forza destra congelata non è un equilibrio statico e il carico verticale varia fra 0,39 e 1,99 BW |
| Oscillazioni dovute al filtro | Esclusa: la policy chiede target oscillanti (0,81 e 0,55 rad); con target costante lo stesso governor si assesta, con escursioni residue sotto 1e-4 rad |

Altri elementi:
- **Replay**: il replay riproduce esattamente le 831 azioni.
- **Training di best47**: 50 update, episodi da 5 s, tre offset dello stesso
  cammino AB06. La reward `ex_novo` ha un periodo nominale di 1,58 s e
  penalizza le stance oltre 1,45 s.
- **Timeout a 10 s**: gli eventi riprendono, ma la pausa diventa una stance di
  4,074 s. Non è una soluzione.
- **Clock e fase**: il clock prescritto è spento; congelare solo la fase o
  solo i timer FSM lascia le oscillazioni.

**Conclusione circoscritta.** best47 non si arresta quando nastro e
riferimenti biologici vengono fermati sinteticamente con questo protocollo.
La catena non ha un comportamento di arresto definito e verificato.

**Strategia proposta, non implementata.** Prima si definiscono:
1. l'intento di arresto;
2. una condizione di appoggio ed equilibrio verificata;
3. gli stati camminata, standing e ripartenza nel contratto FSM e nella
   reward.

Solo dopo si introducono episodi di transizione nel training.

Gli strumenti sono in `diagnosis/`. `replay_stop_policy.py --assert-still`
termina di proposito con `AssertionError: STOP FAILED`: riproduce il sintomo e
non è una regressione.

## TODO

Registro: [TODO_BACKLOG.md](../TODO_BACKLOG.md).

- **STP-T01**, nuovo: arresto, stazione eretta e ripartenza. Viene dal test
  di fermo ed è stato precisato dalla diagnosi in intento, protocollo fisico,
  FSM e obiettivo di apprendimento.
- **SPD-T01**, nuovo: prove mirate con plateau più lunghi e rampe più lente,
  per separare il ritardo del generatore dalla risposta del contatto.
- **V26-T04**, collegato e invariato: avvio autonomo del cammino da fermo.
