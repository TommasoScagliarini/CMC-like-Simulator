# PI sull'errore di coppia, residui a regime e target bandwidth motor driver - 2026-05-15

## Problema

L'analisi strutturale del mattino
(`reports/user/2026-05-15_analisi_strutturale_bandwidth_cascata_e_proposta_impedance_ff.md`)
aveva individuato due nodi:

1. mancato disaccoppiamento frequenziale tra outer PID e motor driver
   (knee `BW_outer / BW_inner ≈ 1.25`, ankle `≈ 0.24`);
2. residui sul lato destro dell'ODE in `e` del driver abs-D, che lasciavano
   sospettare un errore di tracking di coppia non nullo a regime.

Le domande sollevate sul controllo erano:

- il driver attuale (P sull'errore di coppia + damping su `omega_m`) raggiunge
  errore zero a regime, oppure no?
- se no, un PI sull'errore di coppia + damping risolve davvero il problema?
- quale e' il target sensato di `BW_inner` per il cammino?
- prima di rifare l'outer, conviene alzare `BW_inner`?

## Soluzione: analisi a regime e target di banda

### Errore a regime del driver abs-D attuale

Equazione di closed-loop in `e = theta_m - theta_j`:

```text
Jm*e_ddot + (Bm+Kd)*e_dot + (1+Kp)*K*e = (1+Kp)*tau_ref - (Kd+Bm)*omega_j - Jm*omega_j_dot
```

A regime con derivate di `e` nulle:

```text
caso (a) - giunto fermo (omega_j = 0, omega_j_dot = 0):
    (1+Kp)*K*e_ss = (1+Kp)*tau_ref
    tau_spring_ss = tau_ref          -> errore zero (merito del feedforward (1+Kp)*tau_ref)

caso (b) - giunto a velocita' costante (omega_j != 0, omega_j_dot = 0):
    (1+Kp)*K*e_ss = (1+Kp)*tau_ref - (Kd+Bm)*omega_j
    tau_spring - tau_ref = -(Kd+Bm)*omega_j / (1+Kp)
```

Sul nostro setup:

```text
Knee  bias = -(9.8/4.9) * omega_j = -2.00 * omega_j  Nm
Ankle bias = -(9.8/9.8) * omega_j = -1.00 * omega_j  Nm
```

Per `omega_j = 5 rad/s` di picco al knee si traduce in ~10 Nm di errore di
coppia che P + damping **non** rimuove. Durante il cammino `omega_j` non e'
mai zero, quindi questo errore e' persistente.

Nota di chiarezza: il caso "stazionario nel senso del controllo" non vuol
dire `omega_j = 0`. Vuol dire "stato del controllore stazionario": le
derivate di `e` sono nulle. Il giunto, intanto, continua a muoversi guidato
dalla cinematica del corpo (`omega_j` viene dal trial IK).

### PI + damping: dimostrazione di azzeramento dell'errore

Legge proposta:

```text
tau_input = tau_ref + Kp*(tau_ref - tau_spring) + Ki*xi - Kd*omega_m
xi_dot    = tau_ref - tau_spring
```

Sostituisco nel rotore `Jm*omega_m_dot = tau_input - tau_spring - Bm*omega_m`
e cambio variabile in `e`:

```text
Jm*e_ddot + (Kd+Bm)*e_dot + (1+Kp)*K*e - Ki*xi = (1+Kp)*tau_ref - (Kd+Bm)*omega_j - Jm*omega_j_dot
xi_dot = tau_ref - K*e
```

Sistema a 3 stati `(e, e_dot, xi)`.

A regime con `omega_j = const` e tutte le derivate degli stati nulle:

```text
xi_dot = 0  =>  tau_ref - K*e_ss = 0  =>  e_ss = tau_ref / K
                                          tau_spring_ss = tau_ref      <-- errore zero
```

Sostituendo nella prima equazione si trova:

```text
xi_ss = (Kd+Bm)*omega_j / Ki
```

L'integratore si "carica" a un valore finito, proporzionale al disturbo
`omega_j`, e produce esattamente la coppia di controllo che compensa il
trascinamento. Il sistema raggiunge errore zero ed `xi` resta limitato.

Per disturbi a frequenza `omega_d != 0` (gait a 6-12 rad/s), la reiezione
non e' esatta ma scala come `|Ki / (j*omega_d)|`. Con `Ki` ragionevole
l'attenuazione e' comunque ampia rispetto al P + damping puro.

### Target di bandwidth

Contenuto frequenziale dell'IK protesico al knee:

```text
f95 ≈ 2-3 Hz       (95% dell'energia)
f99 ≈ 13-15 Hz     (99% dell'energia)
```

Per tracking pulito di f95 con un po' di margine, ed attenuazione naturale
sopra:

```text
BW_position ≈ 5 Hz
BW_velocity ≈ 25 Hz       (rule of thumb 5x rispetto a BW_position)
BW_torque   ≈ 125 Hz      (rule of thumb 5x rispetto a BW_velocity)
```

Stato attuale:

```text
              BW_torque_attuale   BW_torque_target   gap
Knee          ~32 Hz              125 Hz             ~4x da recuperare
Ankle         ~111 Hz             125 Hz             ~1.1x (gia' vicino)
```

Quindi la priorita' e' alzare `BW_torque` del knee. Sull'ankle e' una
rifinitura.

### Stima dei nuovi guadagni inner

`omega_n = sqrt((1+Kp)*K/Jm)`. Per target `BW_torque ≈ 125 Hz`, cioe'
`omega_n ≈ 785 rad/s`:

```text
SEA_Knee (K=321, Jm=0.01):
  (1+Kp) = omega_n^2 * Jm / K = 785^2 * 0.01 / 321 ≈ 19.2
  Kp_inner ≈ 18                       (era 3.9, +4.6x)

  Kd_inner per zeta = 0.7:
  Kd + Bm = 2*0.7*sqrt(0.01*19.2*321) ≈ 11
  Kd_inner ≈ 11                       (era 9.7, +13%)

SEA_Ankle (K=500, Jm=0.01):
  (1+Kp) = 785^2 * 0.01 / 500 ≈ 12.3
  Kp_inner ≈ 11.3                     (era 8.8, +28%)

  Kd_inner per zeta = 0.7:
  Kd + Bm = 2*0.7*sqrt(0.01*12.3*500) ≈ 11
  Kd_inner ≈ 11                       (era 9.7, +13%)
```

Risultato target post-modifica:

```text
              omega_n [rad/s]   zeta    BW efficace [Hz]
SEA_Knee      ≈785              ≈0.70   ~125
SEA_Ankle     ≈785              ≈0.70   ~125
```

I valori sono comodamente sotto il clamping `tau_input <= 500 Nm` del
plugin, e i guadagni inner sono esposti come `<Kp>` / `<Kd>` nell'`.osim`,
quindi **non serve ricompilare nulla**.

## Strategia operativa

Sequenza minima:

1. **prima**: bump dei guadagni inner. Single variable rispetto agli
   esperimenti precedenti: cambia solo `Kp_inner` e `Kd_inner`, niente altro
   (driver abs-D ancora attivo, niente PI ancora, outer PID quasi-best
   invariato).
2. **verifica**: run full sulla stessa finestra di confronto, confronto con la
   baseline `_outer_pid_gain_sweep_20260514_223838/.../combo_kkp340_kkd30_kki120_akp850_akd2_aki300`
   o con un nuovo abs-D quasi-best ricalcolato.
3. **decisione**: se `motor_speed_dot RMS` cala e tracking migliora,
   l'ipotesi "BW insufficiente del driver" e' confermata. Si decide se
   alzare ancora `Kp_inner` o se passare ad aggiungere `Ki` per togliere
   anche l'errore residuo a regime.
4. **dopo**: solo a quel punto si valuta se serve cambiare la struttura
   outer (cascade P-pos + PI-vel, impedance, FF).

Vantaggi di questo ordine:

- intervento minimo (due valori in un `.osim`);
- variabile singola, diagnosi pulita;
- compatibile con qualsiasi outer si voglia mettere dopo;
- la ricompilazione del plugin (necessaria per `Ki` inner) resta come passo
  successivo, da fare solo se serve davvero.

## File modificati

Nessun file di codice modificato in questo step. L'analisi e' analitica e
i guadagni proposti vanno scritti in un nuovo `.osim`:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_fast_inner.osim`
  (da creare, copia di `stiff321_500.osim` con `Kp/Kd` per `SEA_Knee` e
  `SEA_Ankle` aggiornati ai valori sopra).

## Verifiche eseguite (analitiche)

1. Derivazione esplicita dell'ODE di closed-loop PI + damping in `e`:

```text
Jm*e_ddot + (Kd+Bm)*e_dot + (1+Kp)*K*e - Ki*xi = (1+Kp)*tau_ref - (Kd+Bm)*omega_j - Jm*omega_j_dot
xi_dot = tau_ref - K*e
```

2. Analisi a regime: `tau_spring_ss = tau_ref` (errore zero), `xi_ss` finito
   e proporzionale a `omega_j`.

3. Calcolo dei nuovi `Kp_inner` e `Kd_inner` per `BW_target = 125 Hz` e
   `zeta = 0.7`, verifica che siano coerenti con il clamping `tau_input` del
   plugin e con la regola di disaccoppiamento `BW_torque / BW_velocity ≈ 5`.

4. Calcolo del bias di tracking del driver abs-D attuale come funzione di
   `omega_j`: `-(Kd+Bm)*omega_j / (1+Kp)` (knee `-2*omega_j`, ankle
   `-omega_j`).

Nessuna nuova simulazione eseguita in questo step.

## Prossimi passi

```text
1. creare AB06_SEASEA_stiff321_500_fast_inner.osim con i nuovi Kp/Kd inner
2. run full con outer PID quasi-best, stessa finestra di confronto
3. raccogliere metriche (tracking RMSE, motor_speed_dot RMS,
   tau_error RMS, saturazioni, reserves) e confrontare con baseline
4. decisione: se basta, proseguire eventualmente con sweep su Kp_inner per
   ottimizzazione fine; se serve eliminare anche l'errore residuo a regime,
   passare a PI inner (richiede riscrittura plugin C++ + ricompilazione)
5. solo dopo che l'inner e' "veloce abbastanza", scegliere se cambiare
   struttura outer (cascade P-pos + PI-vel o impedance + FF reale-world)
```

Questa sequenza tratta separatamente le due decisioni: bandwidth (su cui c'e'
gia' consenso) e struttura outer (su cui aspetteremo i dati per decidere).
