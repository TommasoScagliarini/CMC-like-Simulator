# Analisi strutturale del disaccoppiamento bandwidth e proposta cascade impedance + feedforward - 2026-05-15

## Problema

Dopo tre esperimenti (PID outer best/quasi-best, relative-D nel driver, LPF su
`u` outer) il quadro era diventato chiaro:

- il PID quasi-best migliora il tracking cinematico ma genera `motor_speed_dot
  RMS` elevato e dipendenza forte da margini di fase;
- relative-D drop-in destabilizza il sistema togliendo damping implicito al
  giunto;
- LPF su `tau_ref` non riduce il chattering, anzi devasta il tracking gia' a
  `fc = 200 Hz`.

Tutti e tre i sintomi convergevano sullo stesso quesito strutturale: il
sistema cascata `outer PID -> inner SEA driver` ha un disaccoppiamento
frequenziale sufficiente? In caso negativo, qualsiasi sweep di gain o di
filtro e' una toppa locale.

## Soluzione e strategia

Verifica della rule-of-thumb cascade: `BW_inner >> BW_outer`, ratio 5-10x.
Calcolo analitico dei bandwidth a partire dalle equazioni del modello, con i
parametri reali del setup attuale (`AB06_SEASEA_stiff321_500.osim`, outer PID
quasi-best `knee 340/30/120`, `ankle 850/2/300`).

### Inner loop (driver SEA abs-D)

Equazione del rotore + legge di controllo + spring porta a:

```text
Jm * e_ddot + (Bm + Kd) * e_dot + (1 + Kp) * K * e
  = (1 + Kp) * tau_ref - (Kd + Bm) * omega_j - Jm * omega_j_dot
```

con `e = theta_m - theta_j`. La FdT da `tau_ref` a `tau_spring = K*e`:

```text
tau_spring(s) / tau_ref(s) = K(1+Kp) / [Jm*s^2 + (Bm+Kd)*s + (1+Kp)*K]
```

Parametri: `Jm = 0.01`, `Bm = 0.1`, `Kd = 9.7`.

```text
                omega_n [rad/s]   zeta    BW efficace [Hz]
SEA_Knee   Kp=3.9  K=321  ->  397   1.24   ~32 (polo dominante overdamped)
SEA_Ankle  Kp=8.8  K=500  ->  700   0.70   ~111
```

Per il knee `zeta = 1.24` (overdamped), il `-3 dB` non e' a `omega_n` ma al
polo lento `omega_n * (zeta - sqrt(zeta^2 - 1)) ≈ 201 rad/s ≈ 32 Hz`.

### Outer loop (PID su errore cinematico)

Open-loop `L(s) = PID(s) * G_joint(s)` con `G_joint(s) ≈ 1 / (J_joint * s^2)`.
Stime inerzia rotazionale al giunto (rough):

```text
J_knee  ≈ 0.10 kg*m^2
J_ankle ≈ 0.03 kg*m^2
```

Crossover (|L(j*omega_c)| = 1):

```text
Knee   (Kp=340 Kd=30 Ki=120): omega_c ≈ 250 rad/s ≈ 40 Hz
Ankle  (Kp=850 Kd=2  Ki=300): omega_c ≈ 170 rad/s ≈ 27 Hz
```

### Confronto

```text
              BW_inner   BW_outer   ratio    giudizio
Knee          ~32 Hz     ~40 Hz     0.8x     INVERTITO: outer piu' veloce dell'inner
Ankle         ~111 Hz    ~27 Hz     4x       borderline (lontano dal 5-10x)
```

**Diagnosi.** Per il knee non c'e' disaccoppiamento — strutturalmente non e'
una cascata, e' un loop unico con due controllori in serie che si pestano i
piedi. Per l'ankle il rapporto e' 4x: borderline, sotto la rule-of-thumb,
fragile a qualsiasi distorsione di fase.

## Coerenza con i dati gia' osservati

Tutti gli effetti visti finora si spiegano con questo singolo dato
strutturale:

1. **Chattering intrinseco del motor driver** — viene dal fatto che l'outer
   chiede gradienti di `tau_ref` che l'inner non riesce a inseguire entro la
   sua banda. Il motore insegue con errore residuo, e quel residuo si traduce
   in `motor_speed_dot RMS ≈ 494/821 rad/s2` baseline.
2. **Best PID tracking ma piu' chattering del PD** — il PID alza la BW
   dell'outer per migliorare `mean_pros_rmse_deg` da `4.27` a `2.14`; questo
   stringe ulteriormente il rapporto inner/outer, gia' invertito sul knee.
3. **LPF su `u` esplode il loop** — il filtro consuma fase nella catena. Dove
   il margine era gia' nullo (knee) anche `fc = 200 Hz` lo annulla. La
   simulazione o crasha per `non-finite RK4` o sopravvive con tracking
   inutilizzabile (`mean_rmse_deg = 16-446`).
4. **Rel-D in drop-in** — toglie il damping implicito del giunto senza alzare
   la BW inner. Sottrae margine senza compensare. Diverge in `~0.7 s`.

## Implicazioni: tre direzioni vere

Se il problema e' strutturale (rapporto BW), le opzioni sono solo tre
categorie:

### A. Aumentare la banda dell'inner

Concretamente: alzare `Kp_inner` del plugin. Stima per il knee:

```text
Kp_inner = 3.9 -> 12   =>   omega_n = sqrt(13 * 321 / 0.01) ≈ 646 rad/s ≈ 103 Hz
BW efficace passa da ~32 Hz a ~80 Hz
ratio inner/outer passa da 0.8x a ~2x (ancora sotto 5-10x, ma migliorato)
```

Costi: rumore amplificato su `tau_error`, possibile saturazione `tau_input`
piu' frequente, e si entra in una zona dove la rate del comando outer puo'
diventare un problema.

### B. Abbassare la banda dell'outer

Concretamente: ridurre `Ki_outer` (principale colpevole della richiesta di
banda alta) e/o `Kp_outer`. Sweep su `Ki_knee/Ki_ankle` ridotti puo' portare
`omega_c_outer` sotto `BW_inner`. Si paga in `mean_pros_rmse_deg` (la
correzione integrale che spinge il tracking da `2.14` a peggio).

### C. Cambiare struttura: cascade impedance + feedforward filtrato

E' la direzione "ortodossa" per un attuatore SEA in cammino. Riassunto qui.

## Proposta dettagliata: cascade impedance + feedforward dinamico filtrato

### Architettura

```text
q_ref, qdot_ref, qdd_ref
        |
        +-> inverse_dynamics ── tau_ff(t) ── LPF(fc_ff) ── tau_ff_filt
        |                                                       |
        v                                                       |
  K_imp*(q_ref - q) + B_imp*(qdot_ref - qdot) ── tau_fb ─────────+
                                                                  |
                                                              tau_des
                                                                  |
                                                              /F_opt
                                                                  |
                                                                  v
                                                       SEA driver (abs-D)
                                                       motor + spring
                                                       joint
```

Tre pezzi:

1. **Feedforward dinamico filtrato**: `tau_ff = ID(q_ref, qdot_ref, qdd_ref) -
   tau_bio_predicted - tau_ext`, filtrato a `fc_ff < 0.3 * BW_inner`. Per il
   modello attuale: `fc_ff_knee ≈ 8-10 Hz`, `fc_ff_ankle ≈ 30 Hz`. Si calcola
   una volta offline (oppure online riusando `inverse_dynamics.py` step-by-
   step) e da' la coppia "di marcia" che serve.
2. **Feedback impedance puro**: `tau_fb = K_imp * (q_ref - q) + B_imp *
   (qdot_ref - qdot)`. Niente `Ki`. Stima iniziale: `K_imp = K_spring / 4`
   (impedance apparente piu' soft della molla fisica), `B_imp` per `zeta = 0.7`.
3. **Driver SEA inner immutato (abs-D)** — l'inner non viene toccato.

### Perche' funziona meglio

```text
benefit                                              meccanismo
-----------------------------------------------------------------------------
niente integrale = niente windup = niente chattering   Ki = 0 by design
disaccoppiamento cascata garantito                     omega_outer = sqrt(K_imp/J)
                                                       parametro di design,
                                                       non guadagno emergente
grosso della coppia da feedforward, smooth e band-     tau_ff e' deterministica
limitato; il feedback corregge solo errori di modello   e filtrata a fc_ff
attuatore controllato come compliance device           filosofia coerente con SEA
```

### Costo realistico

```text
File da modificare/creare                              LOC stimati
-----------------------------------------------------------------------------
prosthesis_controller.py compute()                    ~50 (riscrittura della cmd)
config.py campi K_imp, B_imp, fc_ff                   ~15
main.py CLI per i nuovi parametri                     ~25
tau_ff_source.py (nuovo)                              ~100 (offline o online)
validation/sweep_impedance_gains.py (nuovo)           ~200
```

Tempo: **1 giornata per primo sweep funzionante, 2-3 giorni per taratura
ragionevole**.

### Variante intermedia (stepping stone)

Prima di andare full-impedance:

```text
Mantieni il PD outer (Kp_outer, Kd_outer), rimuovi Ki, aggiungi tau_ff
filtrato in feedforward.
```

E' un compromesso meno invasivo:

- il PD outer agisce gia' come impedance virtuale (`Kp_outer` su `e_q`,
  `Kd_outer` su `e_qdot`);
- togli solo `Ki` (fonte principale di chattering buildup);
- aggiungi `tau_ff_filt` come correzione predittiva.

Lavoro: **mezza giornata**. Utile come ponte verso lo schema completo, e gia'
sufficiente a verificare se rimuovere `Ki` + aggiungere FF risolve il
problema strutturale o se serve davvero ridisegnare l'impedance.

## File coinvolti nell'analisi (read-only)

Sorgenti di parametri usati nei calcoli:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim` per
  `Jm, Bm, K, Kp_inner, Kd_inner` di SEA_Knee/SEA_Ankle.
- `prosthesis_controller.py` per la struttura del PID outer e il senso dei
  gain (Nm/rad, Nm*s/rad, Nm/(rad*s)).
- `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp` per la legge
  abs-D del driver (per derivare la dinamica di `e`).

Run di riferimento per i ratio osservati:

- `results/_outer_pid_gain_sweep_20260514_223838/full_runs/combo_kkp300_kkd26_kki80_akp750_akd2_aki240`
  (best PID tracking, motor chattering elevato).
- `results/_lpf_sweep_25s_*_20260515` (esperimento LPF, ipotesi bandwidth
  falsificata).
- `results/_relative_d_pid_tradeoff_full_20260515` (rel-D drop-in instabile).

## Verifiche eseguite

Derivazione algebrica esplicita dell'ODE in `e`:

```text
1. sostituzione della legge abs-D in Jm*omega_m_dot = tau_input - tau_spring - Bm*omega_m
2. raggruppamento dei termini in tau_ref e tau_spring -> (1+Kp) factor
3. cambio di variabile: omega_m = omega_j + e_dot, omega_m_dot = omega_j_dot + e_ddot
4. riarrangiamento per portare tutti i termini in e a sinistra
```

Risultato: forma standard 2nd order in `e` con coefficienti `Jm, (Bm+Kd),
(1+Kp)*K`. Da li `omega_n` e `zeta` per identificazione dei coefficienti.

Calcolo bandwidth outer:

```text
|L(j*omega)| = |PID(j*omega)| * |1/(J*omega^2)|
|PID(j*omega)|^2 = Kp^2 + (Kd*omega - Ki/omega)^2
Crossover: |L| = 1
```

Verificato a piu' punti `omega` per knee e ankle.

Nessun nuovo run di simulazione necessario per l'analisi: tutti i dati erano
gia' presenti dagli esperimenti precedenti di oggi.

## Prossimi passi consigliati

In ordine di costo crescente e di valore informativo decrescente:

1. **Stepping stone (PD outer + feedforward filtrato).** Mezza giornata. Toglie
   solo `Ki` e aggiunge `tau_ff_filt`. Test mirato: se gia' questo abbatte il
   chattering, la (1) `Ki ridotto` e il feedforward erano sufficienti.
2. **Cascade impedance + feedforward completo.** 2-3 giornate. Rifa la
   struttura outer come impedance pura + FF. Sweep su `K_imp, B_imp, fc_ff`.
   E' lo schema ortodosso per SEA in cammino.
3. **Aumento `Kp_inner` (direzione A).** Mezza giornata + ricompilazione del
   plugin. Sposta `BW_inner_knee` da ~32 Hz a ~80 Hz. Diretto, ma a quel
   punto serve attenzione al rumore su `tau_error`.

La (1) e' la prima cosa da fare anche solo per disambiguare se il problema
e' dominato dal solo `Ki` o se serve veramente cambiare struttura.
