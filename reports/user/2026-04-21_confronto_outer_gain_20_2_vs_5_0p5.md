# Confronto outer gain 20/2 vs 5/0.5 e analisi del tracking residuo

**Data**: 2026-04-21
**Contesto**: dopo la diagnosi del 2026-04-21
([diagnosi_mancato_tracking.md](2026-04-21_diagnosi_mancato_tracking.md))
sono state eseguite due run sperimentali per testare se abbassare i
guadagni outer PD eliminasse la saturazione motore. Risultati in
[plot/21_04_2026 - 1/](../../plot/21_04_2026%20-%201/) (outer
`Kp=20, Kd=2`) e [plot/21_04_2026 - 2/](../../plot/21_04_2026%20-%202/)
(outer `Kp=5, Kd=0.5`). La diagnosi precedente aveva identificato il
tuning outer come causa primaria della saturazione al 63% sull'ankle;
queste due run servono a confermarlo e a capire quale compromesso
rimane.

## Osservazioni sui grafici

### Plot 02 — Joint/motor states

| Coord | Run 1 (20/2) | Run 2 (5/0.5) |
|---|---|---|
| Ankle joint angle | [0, 0.6] rad — **fisiologico** | [0, **2.0 rad**] — 3x fisiologico |
| Ankle motor angle | traccia il joint pulito | traccia il joint ma entrambi fuori range |
| Knee joint angle | [0, 0.6] rad | [-0.2, 0.5] rad |
| Motor speed ankle | qualche spike | oscillazioni ampie |

Con outer `5/0.5` il PD e' troppo debole: l'ankle non viene piu' riportato
al riferimento IK e il giunto accumula rotazione fino a ~115 deg. Il
motore insegue fedelmente (overlay motor-joint pulito in entrambe le
run), quindi **l'inseguimento motore-giunto funziona**; il problema e'
altrove.

### Plot 05 — `tau_input` e tracking error

In entrambe le run **`tau_input` non satura piu' a ±500 Nm**:

| Run | Ankle `tau_input` range | Knee `tau_input` range |
|---|---|---|
| 1 (20/2) | [-75, +75] Nm | [-30, +40] Nm |
| 2 (5/0.5) | [-100, +50] Nm | [-30, +40] Nm |

Quindi il problema **C1** (saturazione motore) della diagnosi precedente
e' risolto riportando `sea_kd` da 20 a 2 o meno. Questo conferma
quantitativamente che la causa primaria era il tuning outer, non
l'architettura.

### Problema residuo: spike sottili su `tau_ref - tau_spring` (knee)

Nel pannello middle del grafico 05 di entrambe le run il knee presenta
**spike stretti e ad alta frequenza** di `tau_ref - tau_spring`,
sovrapposti a una traccia pulita a bassa frequenza. L'ankle invece ha
una traccia a bassa frequenza senza spike evidenti.

Questo e' lo stesso pattern descritto nel report architetturale del
20/04 ([analisi_architetturale_loop_aperto.md](2026-04-20_analisi_architetturale_loop_aperto.md))
come "glitch single-timestep nel segnale `u`": rari ma violenti eventi
di `|du/dt|` elevato nel ramo knee, non diffusi ma concentrati, che
producono un contributo HPF non fisiologico.

## Ricostruzione della cronologia — perche' ci siamo allontanati dal
funzionamento

Il diff di `config.py` mostra un solo cambiamento di tuning avvenuto nel
20/04 oltre al refactor RK4 + control window:

```diff
 sea_kd: Dict[str, float] = field(default_factory=lambda: {
-    "pros_knee_angle":  2,
-    "pros_ankle_angle": 2,
+    "pros_knee_angle":  20,
+    "pros_ankle_angle": 20,
 })
```

Il `Kd` outer protesico e' stato moltiplicato per 10 nel tentativo di
replicare il set `100/20` del CMC nativo di OpenSim. Con
`Kd_outer = 20`, il termine derivativo `Kd * (qdot_ref - qdot)` amplifica
di 10x gli errori di velocita' — che nel ramo protesico ankle sono
sempre grandi perche' la cinematica di riferimento ha velocita'
angolari elevate nei cambi di direzione. Risultato:

- `outer_pd_cmd` mediano ankle: da ~200 Nm stimati pre-20/04 a **2069 Nm**
  misurati con `100/20` (diagnosi del 21/04)
- saturazione ankle al **63.4%**
- `pros_ankle_angle` che ruota oltre il range dichiarato del PinJoint

**Il problema non e' architetturale**: il refactor RK4+control window
avrebbe retto benissimo con i gain originali. E' stato il cambio di
`sea_kd` da 2 a 20 (e parallelamente il tentativo `100/20`) a portare il
sistema fuori regime.

## Problemi residui ordinati

Ora che la saturazione e' eliminata, restano due problemi distinti.

### R1 — Ankle drift con gain deboli (run 2, `5/0.5`)

Con `Kp_outer = 5, Kd_outer = 0.5` il PD e' troppo lasco: il joint
ankle arriva a 2 rad. Questo non e' un bug ma un trade-off classico
del PD outer:

- `20/2` → tracking stretto, qualche spike residuo
- `5/0.5` → tracking largo, spike ridotti ma range fuori fisiologico

Il compromesso migliore tra queste due run e' **run 1 (20/2)**:
l'ankle resta nel range fisiologico e il knee ha stesso livello di
spike.

### R2 — Spike single-timestep nel ramo knee (indipendente dai gain)

Gli spike sul knee `tau_ref - tau_spring` sono presenti in entrambe le
run con la stessa densita' — **non dipendono dai gain outer**. Sono il
problema diagnosticato il 20/04: la catena
`ID -> SO -> feed-forward -> PD -> u` produce glitch di 1 ms quando la
SO produce un salto discontinuo nelle attivazioni muscolari, propagato
alla matrice `M` e quindi al feed-forward protesico.

Il control window a 3 ms e la SO backtracking mitigano ma non
eliminano questo effetto perche':

1. il feed-forward protesico e' ricalcolato ogni 3 ms (non ogni ms),
   ma un salto nella SO all'istante del ricalcolo produce comunque un
   gradino;
2. il gradino viene poi tenuto costante per 3 substeps di
   integrazione, moltiplicando la sua energia HPF per 3x.

## Raccomandazioni concrete

### Immediate (session corrente)

1. **Ripristinare `sea_kd = {pros_knee: 2, pros_ankle: 2}`** come
   default in [config.py](../../config.py). Il tentativo di usare
   `100/20` come richiesto dal CMC nativo non e' sostenibile col modello
   attuale della protesi.

2. **Validare la run `20/2`** con il validator
   `validation/validate_sim_results.py` per quantificare quanti dei 21
   FAIL precedenti sono stati risolti dalla rimozione della saturazione.

### Medio termine (prossima fase)

3. **Affrontare gli spike knee (R2)** con uno dei seguenti approcci
   alternativi:
   - **smoothing del feed-forward protesico**: filtro LP a 30-50 Hz solo
     su `tau_ff_cmd`, lasciando il PD non filtrato (preserva la
     reattivita' del loop chiuso);
   - **warm start aggressivo della SO**: se le attivazioni tra due
     finestre di controllo cambiano di piu' di una soglia, forzare il
     warm start per ridurre i salti;
   - **feed-forward con rate limiter**: limitare
     `|dtau_ff_cmd/dt| < threshold` per sopprimere direttamente la
     sorgente dello spike.

4. **Non reintrodurre `100/20`** senza aver prima verificato che il
   modello protesico possa fisicamente sostenere la banda richiesta.
   Gli sweep su K/Kp/Kd inner del 20/04 hanno gia' mostrato che nessuna
   scelta passiva risolve: il limite e' la combinazione
   attuatore+cinematica, non il tuning.

## Conclusione

Le due run del 21/04 confermano la diagnosi:

- **La saturazione motore era causata dal tuning outer `100/20`**, non
  dall'architettura RK4+control window.
- **Abbassando l'outer** la saturazione scompare e il motore torna a
  inseguire pulito il giunto.
- **`20/2` e' il miglior compromesso testato finora** tra tracking e
  fisicita': ankle nel range, tau_input mai satura.
- **Rimane un problema di spike knee** indipendente dal tuning outer,
  che e' la naturale prosecuzione della diagnosi del 20/04 (glitch ID/SO
  a livello del feed-forward).

Il simulatore "funzionava" pre-20/04 per la stessa ragione: aveva
`sea_kd = 2`. Tornando a quel valore, la gran parte dei problemi
attuali si risolve senza toccare l'architettura.
