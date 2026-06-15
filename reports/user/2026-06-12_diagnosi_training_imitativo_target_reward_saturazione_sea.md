# Diagnosi del training imitativo: target, reward e saturazione SEA

**Data:** 2026-06-12

## Problema

Il primo training imitativo PPO/MLP, salvato in:

`Trajectory Generator/runs/baseline_mlp_imit_win`

ha completato 40/40 iterazioni e ha prodotto un checkpoint stabile, ma il
rollout deterministico mostra risultati biomeccanici non soddisfacenti:

- imitazione scarsa, soprattutto sul ginocchio protesico;
- saturazione interna frequente del motore SEA del ginocchio;
- comando del ginocchio ad alta frequenza;
- piede protesico quasi sempre scarico;
- reserve biologiche elevate.

L'obiettivo dell'analisi era distinguere le cause dovute alla policy e alla
reward dai limiti del plant SEA, prima di modificare i gain del plugin o
lanciare un nuovo training.

## Strategia

Sono stati analizzati:

- configurazione e metriche del training da 40 iterazioni;
- TensorBoard e `train_iterations.jsonl`;
- rollout deterministico completo in
  `Trajectory Generator/runs/baseline_mlp_imit_win_rollout`;
- cinematica, controlli, diagnostica SEA, potenza, recruitment e GRF online;
- implementazione del target imitativo, della reward, dell'action mapping e del
  controllore cascade;
- cinematica IK e GRF prescribed usate per costruire il gait clock.

Sono state inoltre ricostruite le traiettorie di riferimento effettivamente
servite al controller, per separare:

1. errore della policy rispetto al target imitativo;
2. errore del SEA rispetto al comando prodotto dalla policy.

## Risultati quantitativi

### Qualita dell'imitazione

| Giunto | RMSE totale vs target | Errore comando policy vs target | Errore SEA vs comando |
|---|---:|---:|---:|
| Knee | `24.55 deg` | `24.44 deg` | `0.71 deg` |
| Ankle | `4.16 deg` | `4.15 deg` | `0.37 deg` |

Il SEA segue bene il riferimento prodotto dalla policy. L'imitazione scarsa,
soprattutto al ginocchio, nasce quasi interamente dalla traiettoria scelta dalla
policy, non dall'incapacita del giunto di eseguire il riferimento.

### Saturazione SEA

- comando normalizzato knee con `|u| >= 0.98`: `1.25%`;
- clamp interno motore knee `tau_input = +/-500 Nm`: `33.93%`;
- clamp interno motore ankle: `0%`;
- torque error RMS knee: circa `41.9 Nm`;
- potenza motore knee RMS: circa `15.9 kW`;
- potenza motore knee massima assoluta: circa `68.7 kW`.

La saturazione del `34%` non e la saturazione del comando `u` misurata dalla
reward. E il clamp interno del motore SEA.

### Contenuto frequenziale

- energia del comando knee `u` sopra `10 Hz`: circa `90.7%`;
- picco dominante del comando knee: circa `100 Hz`;
- energia di `qdot_ref` knee sopra `10 Hz`: circa `75.3%`;
- energia della posizione knee sopra `10 Hz`: circa `2.2%`.

La posizione appare relativamente liscia, ma le derivate e il comando del
controller sono fortemente oscillanti.

### Dinamica globale

- frazione di contatto online del piede protesico sinistro: circa `6.9%`;
- frazione di contatto del lato sano destro: circa `66.3%`;
- norma media reserve: circa `799`;
- norma massima reserve: circa `2886`;
- quota muscolare media: circa `0.137`.

La reward imitativa permette una soluzione in cui gli angoli vengono imitati
parzialmente, mentre il piede protesico rimane quasi sempre scarico e la dinamica
globale viene sostenuta dalle reserve.

## Cause individuate

### 1. Target imitativo mal costruito

Il target usa:

`t_target = t - 0.5 * mean_period`

dove `mean_period` e la media globale del file GRF completo.

- periodo medio globale usato: circa `1.132 s`;
- shift usato: circa `0.566 s`;
- periodo locale nella finestra allenata: circa `1.591 s`;
- mezzo periodo locale: circa `0.795 s`.

Per il knee, usare lo shift locale riduce il mismatch tra la cinematica
protesica sperimentale e il target sano anti-fase da circa `14.38 deg` a
`8.46 deg`.

Un singolo shift temporale non allinea correttamente entrambi i giunti:

- knee: shift migliore nella finestra circa `0.75 s`;
- ankle: shift migliore circa `0.43 s`.

Serve quindi una mappatura phase-based e potenzialmente specifica per giunto,
non un semplice shift temporale globale.

### 2. Prefisso iniziale del target internamente incoerente

Nei primi `0.566 s` di ogni episodio, `t - shift` cade prima dell'inizio del
dataset e viene clampato a `t0`.

La posizione target resta costante, ma la velocita target viene comunque letta
come derivata non nulla della spline a `t0`. La reward richiede quindi
simultaneamente posizione ferma e velocita non nulla.

Il clamp motore knee conferma l'impatto:

- prefisso iniziale clampato: `57.8%`;
- parte successiva dell'episodio: `24.5%`.

Con `random_init=false`, tutte le iterazioni ripetono lo stesso prefisso
difettoso.

### 3. Reward cieca alla vera saturazione

La reward calcola `saturation_loss` sul comando normalizzato `u`, oltre la
soglia `|u| >= 0.98`.

Non misura:

- `tau_input_saturated` del plugin;
- distanza di `tau_input_raw` dal limite;
- torque error interno;
- accelerazione e velocita motore;
- potenza motore.

Nel rollout deterministico la penalty media associata alla saturazione di `u`
e circa `0.00062` per step, mentre il clamp motore knee e presente nel `33.93%`
dei campioni e riceve penalty zero.

### 4. Action design e smoothness non controllano il command-rate

La policy memoryless emette tre knot assoluti ogni `10 ms`. La smoothness
attuale misura soltanto le differenze tra i knot prodotti nello stesso step.

Non penalizza:

- salto tra endpoint del segmento precedente e nuovo segmento;
- variazione tra azioni consecutive;
- `qdot_ref` e `qddot_ref`;
- variazione temporale di `u`;
- chattering del motore.

Il peso della smoothness non e necessariamente troppo piccolo in valore
assoluto; e soprattutto applicato alla grandezza sbagliata.

### 5. Gain knee aggressivi: amplificatore, non causa primaria dell'imitazione

Nel modello il knee usa:

- `F_opt = 100 Nm`;
- `Kp = 18`;
- `Jm = 0.01 kg m^2`;
- clamp motore `+/-500 Nm`.

Il guadagno rende facile raggiungere il clamp e il basso `Jm` produce
accelerazioni rotoriche molto elevate. Tuttavia il giunto segue il riferimento
della policy con RMSE di circa `0.71 deg`.

Abbassare subito `Kp` potrebbe ridurre il railing, ma rischia di peggiorare
tracking e bandwidth senza risolvere il target e la reward difettosi. Il tuning
del plant deve quindi seguire una validazione oracle con riferimento corretto.

## Soluzione proposta

Ordine di intervento per ottenere un training imitativo corretto:

1. **Correggere il target imitativo**
   - campionamento phase-based locale/ciclico;
   - eliminazione del clamp iniziale incoerente;
   - verifica separata di fase, segno e range per knee e ankle;
   - target di posizione e velocita mutuamente coerenti.

2. **Penalizzare la vera saturazione e lo stress SEA**
   - loss su `tau_input_saturated`;
   - loss continua sulla prossimita di `tau_input_raw` al clamp;
   - torque error, motor speed/acceleration e motor power;
   - statistiche separate per knee e ankle.

3. **Penalizzare il command-rate corretto**
   - differenza tra segmenti consecutivi;
   - `qdot_ref` e `qddot_ref`;
   - `delta u` e contenuto ad alta frequenza;
   - eventuali limiti diversi per knee e ankle.

4. **Eseguire un rollout oracle**
   - alimentare il controller con il target imitativo corretto;
   - misurare il minimo railing ottenibile dal plant;
   - solo dopo decidere se eseguire sweep su `Kp_knee`, damping o `Jm`.

Un requisito di supporto/carico protesico dovra inoltre essere aggiunto alla
reward imitativa, per evitare soluzioni cinematiche con piede quasi sempre
scarico.

## File modificati

- `reports/user/2026-06-12_diagnosi_training_imitativo_target_reward_saturazione_sea.md`

Nessuna modifica e stata applicata al codice, al modello `.osim`, al plugin C++
SEA o alla semantica del comando SEA. Il turno che avrebbe dovuto implementare
i punti 1-4 e stato interrotto prima delle modifiche.

## Test e verifiche eseguite

- analisi completa degli output STO del rollout deterministico;
- ricostruzione del target sano anti-fase e della cinematica filtrata;
- confronto RMSE/correlazione per knee e ankle;
- separazione errore policy-target ed errore SEA-comando;
- misura clamp motore e saturazione di `u`;
- analisi di torque error, motor power e cascade;
- analisi FFT di posizione, riferimento e controllo;
- confronto tra periodo gait globale e periodo locale;
- analisi dei TensorBoard event file del training;
- verifica del contatto online e delle reserve.

## TODO

- [ ] Implementare un target imitativo phase-based locale/ciclico e coerente in
      posizione/velocita, con validazione separata knee/ankle.
- [ ] Eliminare il prefisso iniziale clampato oppure iniziare gli episodi da un
      punto che disponga della storia necessaria.
- [ ] Esporre e penalizzare nella reward `tau_input_saturated`,
      `tau_input_raw`, torque error e motor power.
- [ ] Aggiungere loss command-rate tra step, `qdot_ref`, `qddot_ref` e
      variazione di `u`.
- [ ] Aggiungere una metrica/reward minima di supporto e carico protesico.
- [ ] Eseguire rollout oracle con target corretto e misurare la saturazione
      minima ottenibile dal plant.
- [ ] Solo dopo l'oracle, valutare sweep conservativo dei parametri knee SEA.
- [ ] Eseguire un nuovo training imitativo soltanto dopo il superamento dei test
      comportamentali e biomeccanici del target/reward corretti.
