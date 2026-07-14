# Reward biomeccanica senza target torque

Data: 2026-06-22

## Problema

La discussione su `sym60` e `asym100` ha chiarito che il solo tracking cinematico non garantisce una soluzione biomeccanicamente corretta: `asym100` puo tracciare bene ma produrre una coppia alla caviglia protesica sospetta, mentre `sym60` genera una `tau_spring` molto piu interessante tramite un meccanismo indiretto del cascade.

La soluzione ovvia, cioe rendere la reward "torque-aware", non puo pero usare:

- un target torque derivato dal trial o dal modello corrente;
- la morfologia della coppia osservata in `AB06` o in `sym60`;
- termini dipendenti da variabili interne del controller, come `outer_i_cmd`, `cascade_velocity_error` o il segno di `q_ref - q`;
- una penalita manuale su un burst definito in una finestra temporale specifica.

Il vincolo architetturale rimane: trajectory generator, outer controller e motor driver SEA devono restare moduli separabili. La reward deve quindi premiare proprieta dinamiche generali, non dettagli del controller corrente.

## Vincoli emersi

1. Le reserve non possono essere penalizzate in valore assoluto senza cautela. I dati prescribed sani mostrano differenze sostanziali tra le due gambe, ad esempio nell'`ankle_range`; quindi anche con tracking perfetto, coppie perfette e GRF online ideali potrebbe rimanere una quota inevitabile di compensazione/reserve.
2. La GRF online e informativa ma non abbastanza affidabile da diventare un obiettivo dominante. Le analisi precedenti indicano sottocarico del lato protesico nelle GRF online, quindi una reward forte su quel segnale rischia di insegnare alla policy a ottimizzare un modello di contatto imperfetto.
3. La coppia `tau_spring` di `asym100` non e brutta nel senso classico: e continua, liscia e senza chattering evidente. Penalita generiche su rateo, smoothness o spike non bastano a distinguere una coppia biomeccanicamente sospetta da una utile.
4. Non si puo usare la morfologia della coppia come informazione di reward.

## Reward 2: contact-load consistency

Questa famiglia punta a evitare soluzioni in cui la cinematica e buona ma il lato protesico e dinamicamente scarico o poco coinvolto.

Idea corretta:

- usare GRF/contact come segnale debole di plausibilita;
- evitare che il lato protesico produca tracking mentre il contatto o il carico sono incoerenti;
- ragionare su stance/load consistency, non su una forma di coppia.

Rischio principale:

- forte dipendenza dalla GRF online;
- se la GRF online sottostima il carico protesico, una reward aggressiva puo spingere la policy nella direzione sbagliata.

Strategia consigliata:

- non premiare un valore assoluto di GRF;
- non imporre simmetria rigida tra lato sano e lato protesico;
- usare una confidence di contatto costruita da piu segnali: GRF online, eventi di gait, altezza/velocita del piede, fase ciclica;
- usare il termine solo come regolarizzatore debole;
- preferire penalita di incoerenza grossolana, ad esempio piede in stance ma carico quasi nullo per molti step, invece di target numerici precisi.

Conclusione: reward interessante, ma deve essere confidence-weighted e non puo essere il pilastro principale finche la GRF online non e validata meglio.

## Reward 3: uso meccanico funzionale del SEA

Questa e la direzione piu interessante da approfondire, perche non richiede una shape target della coppia.

L'obiettivo non e dire:

```text
tau_spring deve avere questa forma
```

ma:

```text
quando il SEA spende coppia, potenza o energia, questo sforzo deve avere utilita meccanica plausibile.
```

Segnali ammissibili perche fisici e controller-agnostic:

- `tau_spring`, coppia trasmessa al giunto dalla molla SEA;
- `tau_motor` / `tau_input`, sforzo a monte della molla;
- `tau_spring * qdot_joint`, potenza articolare lato SEA;
- potenza/energia del motore;
- saturazioni e clipping del SEA;
- stato cinematico del piede e della fase di contatto.

Termini possibili, da mantenere deboli:

- penalizzare potenza motore alta che non si traduce in potenza articolare comparabile;
- penalizzare molto `tau_input` con bassa `tau_spring`, cioe energia spesa nel driver senza trasferimento utile al giunto;
- penalizzare coppia SEA significativa quando il contatto e poco credibile;
- penalizzare lavoro SEA non produttivo rispetto al tracking ottenuto, senza imporre segno o forma;
- penalizzare saturazione o uso vicino ai limiti come indicatore di soluzione fragile.

Dipendenza dalla GRF:

- media, non inevitabilmente alta;
- la GRF serve soprattutto per definire quando il piede e carico;
- il termine puo essere progettato anche con gate cinematici e fase, quindi non deve dipendere solo dalla GRF online.

Conclusione: non e una reward di morfologia, ma una reward di efficienza e trasferimento meccanico. E una buona candidata per migliorare la plausibilita biomeccanica senza legarsi al cascade.

## Reward 4: tracking con costo dinamico composito

Questa famiglia cerca di trasformare il tracking da obiettivo unico a obiettivo condizionato:

```text
traccia bene, ma non scaricando il problema su compensazioni, reserve, saturazioni o uso inefficiente del SEA.
```

Componenti candidate:

- tracking cinematico forte su reference/served-reference;
- costo debole su reserve/residual, non assoluto ma rispetto a una baseline o floor inevitabile;
- costo su saturazione SEA e action clipping;
- costo su energia/potenza SEA non funzionale;
- costo su instabilita globale: pelvis, CoM, caduta, posture fuori range;
- contact-load consistency debole e confidence-weighted;
- regolarizzazione su smoothness solo come igiene numerica, non come criterio biomeccanico principale.

Punto importante sulle reserve:

Non va usata una penalita assoluta semplice, perche le asimmetrie sane prescribed indicano che una parte del costo dinamico puo essere strutturale. Una forma piu robusta e:

```text
penalizza solo l'eccesso sopra una baseline inevitabile
```

oppure:

```text
penalizza reserve/residual impulsive, crescenti o concentrate su DOF globali critici
```

Dipendenza dalla GRF:

- bassa o media, a seconda del peso assegnato alla contact-load consistency;
- puo essere progettata come reward distribuita, dove la GRF e un segnale tra molti, non il giudice principale.

Conclusione: e probabilmente la direzione piu robusta come architettura di reward. Non garantisce da sola la coppia "giusta", ma rende piu costose le soluzioni dinamicamente sospette.

## Strategia proposta

Non introdurre una shape reward sulla coppia.

Disegnare invece una reward composita con tre blocchi:

1. Tracking forte:
   - posizione e velocita protesiche;
   - served reference;
   - stabilita globale minima.

2. Uso fisico del SEA:
   - penalita leggera su saturazione, clipping e sforzo motore;
   - penalita su mismatch tra sforzo motore e coppia/potenza effettivamente trasferita alla molla/giunto;
   - eventuale gate di contatto debole per evitare coppie rilevanti in condizioni di piede scarico.

3. Costo dinamico globale:
   - reserve/residual solo sopra una baseline o soglia;
   - penalita su instabilita pelvis/CoM;
   - contact-load consistency debole, mai target GRF rigido.

L'obiettivo e ottenere una policy che non impari "la coppia di AB06", ma che renda economicamente svantaggiose le scorciatoie dinamiche: scarico del lato protesico, uso eccessivo del driver, saturazioni, compensazioni globali e tracking ottenuto con meccanica fragile.

## File modificati

- Creato questo report: `reports/user/2026-06-22_reward_biomeccanica_senza_target_torque.md`

Nessun file di codice o configurazione e stato modificato.

## Test e verifiche considerate

- Analisi precedente `sym60` vs `asym100`: il comportamento positivo di `sym60` non sembra un errore di plot, ma un bacino favorevole legato al segno dell'errore di tracking amplificato dal cascade.
- Test controfattuale precedente: sostituendo l'errore relativo ankle di `asym100` con quello di `sym60` nella finestra critica, il burst positivo si trasforma in coppia negativa, confermando il ruolo del controller.
- Sweep gain parziale: ridurre `Kp_outer/Ki_inner` attenua il burst di `asym100`, ma riduce anche la coppia utile di `sym60`, suggerendo che il controller e parte attiva del fenomeno.
- Analisi GRF precedente: la GRF online sottocarica il lato protesico, quindi non e prudente usarla come target reward forte.

## TODO

- Definire una versione quantitativa minimale dei termini di uso meccanico SEA senza usare target torque o morfologia della coppia.
- Stabilire una baseline/floor per reserve e residual che tenga conto delle asimmetrie sane prescribed.
- Costruire un indicatore di contact confidence non basato solo su GRF online.
- Valutare offline, sui rollout gia disponibili, se i candidati reward 2/3/4 distinguono `sym60` da `asym100` senza codificare esplicitamente la shape della coppia.
- Solo dopo la verifica offline, introdurre i termini in `Trajectory Generator/baseline_MLP/reward_function.py` con pesi deboli e ablation dedicata.
