# Diagnosi del chattering a 88 Hz come modo di loop chiuso outer-D ↔ driver inner - 2026-05-16

## Problema

Dopo il bump dei guadagni inner del 2026-05-16 mattina (vedi
`reports/user/2026-05-16_bump_inner_gains_run_full.md`), l'ispezione dei
plot di [results/_fast_inner_pid_20260516/](../../results/_fast_inner_pid_20260516/)
e [plot/05_16_2026_1/](../../plot/05_16_2026_1/) ha mostrato chattering
visibile su quattro segnali:

- `tau_input` del knee (oscillazione ampia nel comando motore)
- `motor_speed` del knee
- `motor_speed` dell'ankle
- `joint_speed` dell'ankle (osservata visualmente)

La metrica numerica corrispondente era `motor_speed_dot_max` knee passato
da 2506 (baseline best PID) a 8962 rad/s² (`+258 %`).

L'ipotesi iniziale era "ringing del driver inner alla sua risonanza":
calcolo analitico `omega_d = omega_n*sqrt(1 - zeta^2)` con
`omega_n ≈ 781 rad/s` e `zeta ≈ 0.71` da' `f_d ≈ 88 Hz`. La FFT ha
confermato il picco esattamente a 86.2 Hz su tutti i segnali sospetti.

Le tre domande dell'utente: il problema viene dall'outer (tuning o
struttura), dall'inner (tuning o struttura, PI aiuta?), o da altro?

## Soluzione - diagnosi a tre check

Per discriminare tra sorgente del chattering e amplificatore del
chattering, sono stati eseguiti tre check FFT mirati in banda
`80-100 Hz` (centrata sulla risonanza) confrontando le tre run
disponibili:

- `BASE` = `_outer_pid_gain_sweep_20260514_223838/full_runs/combo_kkp300_kkd26_kki80_akp750_akd2_aki240`
- `FAST` = `_fast_inner_pid_20260516` (Kd_knee_outer = 30)
- `KDOUT` = `_fast_inner_kdouter15_20260516` (Kd_knee_outer = 15)

### Check (a) - SO genera 88 Hz?

Spettro di `sim_output_tau_bio.sto` sui DoF biologici principali:

```text
tau_bio[hip_flexion_l]   FAST mag 1.48e3   KDOUT mag 1.48e3   identico
tau_bio[pelvis_tilt]     FAST mag 1.24e3   KDOUT mag 1.24e3   identico
tau_bio[hip_flexion_r]   FAST mag  676     KDOUT mag  676     identico
tau_bio[knee_angle_r]    FAST mag  363     KDOUT mag  363     identico
tau_bio[ankle_angle_r]   FAST mag  294     KDOUT mag  294     identico
```

Il SO produce davvero contenuto a 86 Hz su tutto il corpo biologico.
Ma e' **identico tra FAST e KDOUT**: se fosse la sorgente primaria,
dimezzare `Kd_knee_outer` non avrebbe alcun effetto sul chattering del
driver. Invece l'ha dimezzato (vedi (c)). Quindi il SO **osserva** la
vibrazione via coupling meccanico `M(q)`, non la genera.

### Check (b) - il riferimento IK inietta a 88 Hz?

```text
                              peak       mag       E_88/E_lf
q_REF pros_knee  (file IK)   83.0 Hz    2.49      5.4e-06
q_SIM pros_knee  (FAST)      86.2 Hz    3.80e-02  1.2e-08
q_SIM pros_knee  (KDOUT)     86.2 Hz    3.16e-02  7.9e-09

q_REF pros_ankle (file IK)   96.7 Hz    6.23      7.8e-05
q_SIM pros_ankle (FAST)      86.2 Hz    1.93e-01  1.6e-06
q_SIM pros_ankle (KDOUT)     86.2 Hz    1.86e-01  1.6e-06
```

Due fatti:

- le **frequenze** del riferimento (83 Hz knee, 96.7 Hz ankle) sono
  diverse da quella del chattering (86.2 Hz). Il riferimento non
  inietta a 86 Hz;
- le **magnitudini** del simulato sono 3-6 ordini di grandezza piu'
  piccole del riferimento. Il giunto pros sta **filtrando** il
  riferimento in quella banda, non amplificandolo.

La HF a 86.2 Hz nel q_sim e' quindi una **proprieta' del sistema chiuso**,
non un disturbo iniettato dall'esterno. Corrisponde esattamente al modo
proprio del driver inner.

### Check (c) - decomposizione outer P / D / I

Spettro a 86 Hz dei termini interni del comando outer PID:

```text
joint   knob              P-cmd mag   D-cmd mag   I-cmd mag
knee    FAST  (Kd=30)        12.9       623       8.5e-3
knee    KDOUT (Kd=15)        10.7       259       7.1e-3
ankle   FAST  (Kd=2)         164        210       1.1e-1
ankle   KDOUT (Kd=2)         158        202       1.0e-1
```

Sul knee il D-cmd a 86 Hz **dimezza esattamente** quando dimezzo
`Kd_knee_outer`. P quasi invariato (proviene da `q_sim` che e' gia'
basso), I trascurabile. Il D-term outer e' il canale dominante per
l'iniezione di 86 Hz nel comando.

## Strategia - test di conferma con Kd_outer/2

Per validare la diagnosi e' stata lanciata una run identica a FAST
ma con `Kd_knee_outer 30 → 15` (single variable, solo CLI, nessun
edit `.osim`):

```bash
python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --t-start 11.99 --t-end 21.0 \
  --output-dir results/_fast_inner_kdouter15_20260516 \
  --filter-grf \
  --sea-outer-controller pid \
  --sea-kp-knee 340 --sea-kd-knee 15 --sea-ki-knee 120 \
  --sea-kp-ankle 850 --sea-kd-ankle 2 --sea-ki-ankle 300
```

Stessa finestra `[11.99, 21.00] s`, stesso modello `.osim` con gli
inner gains alzati. Run completata in `wall=746 s`, `status=complete`.

### Risultato metriche (3 run)

```text
                                   BASE    FAST    KDOUT      KDOUT vs FAST
tracking
  knee_rmse_deg                   1.5284  1.2958  1.4327      +10.6 %
  ankle_rmse_deg                  2.7522  2.4512  2.4003       -2.1 %
  mean_pros_rmse_deg              2.1403  1.8735  1.9165       +2.3 %
driver torque error
  knee_tau_err_rms  [Nm]            3.31    1.23    0.97      -21.1 %
  ankle_tau_err_rms [Nm]            1.05    1.00    1.02         ~0
motor speed derivative
  knee_mdot_max  [rad/s2]         2505.7  8962.4  4014.7      -55.2 %
saturazioni / reserves              0/0     0/0     0/0
```

### Risultato chattering 86 Hz

```text
SEGNALE                E_band/E_lf           peak_mag a 86 Hz
                       BASE   FAST   KDOUT     BASE  FAST  KDOUT
tau_input KNEE         1.5%   31%    5.8%      1.5e3 8.0e3 3.3e3   -59 %
motor_speed KNEE       1.5%   16%    3.0%       195   901   373    -59 %
outer_D_cmd KNEE      0.77%   1.3%   0.68%      408   623   259    -58 %
tau_input ANKLE       0.21%   0.40%  0.44%     1.3e3 1.7e3 1.7e3    ~0  (Kd_ankle invariato)
motor_speed ANKLE      6%     8.5%   9%         155   180   174     ~0  (Kd_ankle invariato)
outer_D_cmd ANKLE      49%    71%    81%        254   256   262     ~0
```

Dimezzando `Kd_knee_outer` il chattering knee crolla del ~60 % su tutti
i sintomi. Sull'ankle (Kd_ankle non toccato) il chattering e' invariato.
Questo conferma in modo netto che **il D outer e' il vero canale di
feedback HF** a 88 Hz sul knee.

## Diagnosi finale

Non e' "rumore numerico che eccita una risonanza" - terminologia che
avevo usato in modo impreciso a una domanda precedente. E' un
**modo oscillatorio del loop chiuso outer-D ↔ driver inner**,
auto-sostenuto al guadagno di anello a 88 Hz.

Meccanismo:

```text
1. driver inner ha un polo sotto-smorzato a omega_d = omega_n*sqrt(1-zeta^2)
   = 781*sqrt(1 - 0.5) = 552 rad/s = 88 Hz, con Q = 1/(2*zeta) = 0.71
2. perturbazione broadband (troncamento RK4 ~1e-5, salti del SO ~1e-4,
   derivata del riferimento IK ~1e-7) -> scintilla iniziale a 88 Hz
3. motore esegue piccole oscillazioni a 88 Hz, molla trasmette al
   giunto pros vibrazioni di ordine 1e-2 rad (microscopiche, magnitudo
   spettrale ~3.8e-2)
4. outer D deriva la microvibrazione: omega = 552 rad/s alla risonanza
   amplificata per Kd_outer -> D-cmd con magnitudo ~600 a 86 Hz
5. D-cmd entra in tau_ref outer -> il driver e' eccitato esattamente
   alla SUA risonanza -> motore amplifica
6. loop chiuso: livello stazionario = funzione del guadagno di anello
   a 88 Hz, dominato da Kd_outer * Q_driver / Jm
```

La perturbazione iniziale e' la "scintilla" (qualunque microerrore
broadband). L'energia mantenuta del chattering viene dal **guadagno di
anello** outer-D × Q_driver alla frequenza di risonanza. Senza il loop,
la scintilla si spegnerebbe in pochi cicli.

Il SO non e' driver del loop ma osservatore: vede la vibrazione del
giunto pros via `M(q)` coupling e produce un piccolo ripple a 86 Hz
nei tau_bio. Identico tra FAST e KDOUT proprio perche' non e' lui a
forzare il sistema.

### Risposte alle domande dell'utente

- **outer loop, tuning**: si', il guadagno di anello a 88 Hz e' la
  variabile dominante. Dimezzare `Kd_knee_outer` ha dimezzato il
  chattering.
- **outer loop, struttura**: si', un **filtro LPF sul D-term** (fc ≈ 25-40 Hz)
  romperebbe il loop a 88 Hz mantenendo l'autorita' del D in banda di
  tracking. Intervento chirurgico.
- **inner loop, tuning**: parzialmente. Alzare `Kd_inner` per
  portare `zeta = 1.0` (critically damped) rimuoverebbe il modo a 88 Hz
  dal sistema, ma con costo di -30 % di BW.
- **inner loop, struttura - PI inner**: **inutile per il chattering**.
  Il PI sull'errore di coppia agisce a bassissime frequenze (DC, drift,
  bias `-(Kd+Bm)*omega_j/(1+Kp)`). La risonanza 88 Hz e' molto sopra la
  banda dell'integratore. PI inner risolverebbe l'errore residuo a
  regime studiato il 2026-05-15, non il chattering.
- **altro**: il riferimento IK ha HF a frequenze diverse e in
  magnitudo molto maggiore, ma il loop chiuso filtra. Il SO contribuisce
  con noise broadband, ma non e' driver.

## File modificati / creati

```text
+  results/_fast_inner_kdouter15_20260516/                 (output run conferma)
+  plot/05_16_2026_2/                                       (plot run conferma)
+  reports/user/2026-05-16_diagnosi_chattering_88Hz_loop_outer_inner.md
```

Nessuna modifica a codice Python, `.osim` o plugin C++. Test eseguito
solo via CLI variando un parametro outer.

## Test / verifiche eseguite

1. FFT diagnostica del chattering su 10 segnali per FAST e BASELINE
   (`/tmp/fft_chattering.py`): identificata frequenza dominante 86.2 Hz
   in `tau_input KNEE`, `motor_speed KNEE`, `motor_speed ANKLE`,
   verificato che `pros_*_angle` simulato e' pulito a HF (HF/LF ~1e-5).

2. Calcolo analitico della frequenza naturale smorzata del driver
   inner per entrambi i SEA col tuning post-bump: `f_d ≈ 88 Hz`,
   `Q ≈ 0.71`. Match esatto con il picco osservato.

3. Run di conferma con `Kd_knee_outer 30 → 15`, `wall = 746 s`,
   `status = complete`, 0 saturazioni, 0 reserves protesiche.

4. Confronto metriche FAST vs KDOUT
   (`/tmp/compare_kdouter15.py`): knee chattering -60 % su tutti i
   sintomi, tracking knee +10.6 % (1.30 → 1.43 deg, ancora -6 % vs
   baseline), `motor_speed_dot max` knee -55 %.

5. Tre check diagnostici
   (`/tmp/diagnose_noise_source.py`):
   - (a) `tau_bio` SO: identico FAST/KDOUT su tutti i DoF, esclude SO
     come sorgente primaria;
   - (b) `q_REF` IK picco a 83/96.7 Hz (diversa frequenza) e magnitudo
     3-6 ord. maggiore di `q_SIM`, esclude riferimento come sorgente;
   - (c) outer P/D/I: D-cmd dominante, dimezza esattamente con
     `Kd_outer`, conferma che il D outer e' il canale del feedback HF.

6. Plot rigenerati in [plot/05_16_2026_2/](../../plot/05_16_2026_2/)
   per ispezione visiva.

## Prossimi passi

```text
1. valutare aggiunta di un LPF sul D-term outer (fc 25-40 Hz):
   - rompe il loop a 88 Hz mantenendo autorita' del D in banda tracking
   - intervento chirurgico, non sacrifica guadagno utile
   - verificare se gia' presente come opzione in prosthesis_controller.py
     o va aggiunto come parametro CLI/config

2. opzionale - sweep su Kd_knee_outer in {10, 15, 20, 25, 30}:
   identifica il sweet spot tracking vs chattering empiricamente.
   Meno selettivo del LPF ma piu' semplice.

3. opzionale - bump Kd_inner a 16 per zeta = 1.0: rimuove il modo
   88 Hz dal sistema. Costo -30 % BW. Da provare solo se LPF e sweep
   non bastano.

4. NON procedere con PI inner per affrontare il chattering: e' il
   tool sbagliato per questo problema. Riservare PI inner all'errore
   di tracking di coppia residuo gia' caratterizzato il 2026-05-15.

5. TODO Windows aperto da 2026-05-15: ancora pendente, includere la
   linea di lavoro di oggi (bump inner + diagnosi outer D) nei
   passi da replicare.
```

## TODO aperti (da propagare nei prossimi report)

- Plot comparativo gait-cycle sovrapposto BASE / FAST / KDOUT
  (`plot/05_16_2026_3/`) per ispezione visiva del trade-off.
- Decisione sull'implementazione del LPF sul D-term outer (gia' presente
  o da aggiungere).
- TODO Windows pendente da 2026-05-15: compilare/copiare la DLL `ff`,
  replicare bump inner gains nel `.osim` e CLI `Kd_knee_outer = 15`.
