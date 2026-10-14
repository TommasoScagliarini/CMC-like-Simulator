# Audit target e contatto — 21 settembre 2026

Analisi offline dei rollout 39D alle iterazioni 25, 29, 40 e del V26 best87. Nessun runtime, checkpoint o configurazione di training modificati.

## Risultati verificati

1. **Non emerge un errore di segno, shift o derivazione del target.** `phase_target.py` riusa `PhaseBasedImitationTarget` con mappa identitaria sulle due coordinate protesiche e shift zero. Il target qdot è la derivata della stessa spline periodica divisa per il periodo locale. Il clock sin/cos dell'actor corrisponde alla fase sinistra e le quattro feature ai target della stessa fase (controllo già riprodotto nell'audit 40iter). La fase del target storico era leggermente anticipata rispetto al nuovo clock: 3,22% di ciclo in media per il ginocchio, 4,52% per la caviglia, conseguenza dei due clock e degli shift calibrati storici (0,465 e 0,452). Non c'è mezzo ciclo errato introdotto nella 39D.

2. **La caviglia deve imparare un movimento sostanzialmente diverso.** Target LEFT: −6,61°..22,05°. Target RIGHT sfasato V26: +4,96°..25,29°. Il target nuovo passa 84/500 campioni sotto zero; quello V26 nessuno. La velocità negativa massima richiesta è −3,22 rad/s contro −2,22. Quindi la porzione negativa oggi mancante non era una competenza raggiunta dal V26. Questo è conseguenza della modifica deliberata, non prova che sia impossibile impararla.

3. **I pesi nominali identici non danno gli stessi pesi per errore angolare assoluto.** La posizione entra nella loss come `w*(errore/range)^2`. Per il ginocchio il coefficiente per rad² passa da 1,14987 a 1,60689 (+39,8%); per la caviglia da 15,88973 a 8,01345 (−49,6%). Il rapporto caviglia/ginocchio passa da 13,82 a 4,99. Un errore di 5° di caviglia pesa 0,1210 nel V26 e 0,0610 oggi, prima dello score non lineare; lo stesso errore al ginocchio pesa 0,00876 e 0,01224. È un cambio concreto del compromesso fra errori angolari e penalità fisiche, dovuto alla normalizzazione per escursione già concordata, non un bug. Non basta da solo a dimostrare la causa dell'esito.

4. **Il target LEFT è più vicino all'IK LEFT originale:** RMSE del template rispetto al prescribed originale 2,97° ginocchio / 1,62° caviglia, contro 6,74° / 7,89° del target V26. L'aver usato la gamba corretta non è un errore evidente nei dati. La media periodica non riproduce esattamente ogni singolo ciclo originale.

5. **Il picco del momento GRF anomalo avviene in stance, non al minimo negativo di caviglia.** A t=17,53 s, fase LEFT 23,57%, last40 presenta ginocchio −14,06° contro target −14,44° e caviglia +14,62° contro target +9,33°. Forza 537,75 N, COPz−anklez +65,29 mm, momento −36,36 Nm. La penalità è 1,2566 per quel singolo step; non è un errore nella sottrazione della reward: `0.1*((36.36−8)/8)^2`.

6. **La postura del ginocchio accompagna una forte differenza nella leva e nel carico.** Allo stesso t=17,53 s, iter25 ha ginocchio −19,45°, caviglia +13,21°, forza 281,52 N e momento +8,35 Nm (nessuna penalità). V26 best87 ha ginocchio −22,35° (molto più flesso del suo target −12,29°), caviglia +16,14°, forza 138,76 N e momento +8,24 Nm. Dunque attribuire la penalità alla sola caviglia sarebbe improprio: il target è quasi perfettamente seguito al ginocchio della 40 proprio quando il carico/leveraggio è sfavorevole. Resta da isolare il ruolo causale dei due giunti mediante interventi controllati; queste sono associazioni fra rollout diversi.

## Artefatti e riproducibilità

- `analyze.py`: lettura trace JSON e IK filtrato con `KinematicsInterpolator(SimulatorConfig())`; nessuna integrazione fisica.
- `metrics.json`: valori, finestre di fase, confronti simultanei dei picchi e normalizzatori.
- `target_contact_comparison.png`: target LEFT/RIGHT sfasato/LEFT originale e momenti/carichi dei quattro rollout.

Limiti: V26 best87 è un checkpoint più maturo; un solo start e seed; i coefficienti sono pre-score; target diversi, stati diversi e diversi pesi appresi impediscono attribuzione causale univoca senza i test oracle complementari.

## Intervento statico controllato sul picco a t=17,53 s

`static_contact_probe.py` ricostruisce tutte le 21 posizioni e velocità generalizzate dal file STO della 40. Mantiene ogni q/qd invariato salvo gli angoli esplicitamente elencati e ricalcola il contatto attraverso gli stessi componenti OpenSim. Non integra nel tempo. Il contatto ricostruito coincide con lo STO 1 kHz entro **0,000078 N**, **0,000066 Nm** sulle componenti del momento globale e **1,1e−8 m** sul COP. La geometria e la legge di contatto sono quindi riprodotte con precisione utile.

| Intervento rispetto alla stessa posa last40 | Forza normale | Momento caviglia | Penetrazione | Penalità anti-flip/step |
|---|---:|---:|---:|---:|
| Nessuno: ginocchio −14,06°, caviglia +14,62° | 539,19 N | −36,75 Nm | 17,37 mm | 1,2915 |
| Solo ginocchio al target −14,44° | 526,89 N | −34,35 Nm | 16,57 mm | 1,0852 |
| **Solo caviglia al target +9,33°** | 541,77 N | **−4,18 Nm** | **13,54 mm** | **0** |
| Entrambi ai target LEFT | 536,56 N | −1,39 Nm | 13,96 mm | 0 |
| Solo ginocchio al valore iter25 −19,45° | 284,55 N | −1,08 Nm | 9,99 mm | 0 |
| Solo ginocchio al valore V26 −22,35° | 168,62 N | +10,29 Nm | 10,55 mm | 0 |

**Conclusione causale locale:** l'errore di caviglia di +5,29° basta, nella posa raggiunta, a spostare il momento da una zona non penalizzata a una fortemente penalizzata, senza aumento apprezzabile del carico totale. Seguire il target sinistro in quella posa elimina la penalità e riduce la penetrazione. Quindi quel picco specifico non dimostra incompatibilità fra target LEFT e contatto: la caviglia poco accurata è una causa diretta della penalità locale. Una maggiore flessione del ginocchio può compensarla riducendo il carico e modificando la geometria, spiegando perché la policy25 o V26 possano evitare il picco con una caviglia comunque positiva. Il confronto descrittivo del paragrafo precedente viene così precisato da un intervento.

**Limiti:** non dimostra che tutta la traiettoria target sia raggiungibile, che la modifica sia eseguibile istantaneamente né che questo sia l'unico problema del training. Le velocità generalizzate sono fisse; quelle cartesiane dei contatti cambiano conseguentemente alla posa. Stati muscolari, interni SEA e controlli sono quelli del reset: non entrano nella legge di contatto, e la validazione contro lo STO conferma la loro irrilevanza per questa misura specifica. Non si ricostruiscono dinamica o accelerazioni con questi stati ausiliari.

Nota temporale: i termini di reward del trace a t=17,53 usano l'ultimo wrench disponibile a 17,529 s, mentre il centro caviglia è aggiornato a 17,53 s. Da qui −36,36 Nm nel trace contro −36,75 Nm nella ricomputazione simultanea della probe. La forza trace coincide esattamente con la riga STO17,529. Questo offset di misura è documentato e non viene attribuito alla differenza fra i due training.

Artefatto numerico completo: `static_contact_probe.json` (incluse tutte le q/qd mantenute e modifiche angolari).
