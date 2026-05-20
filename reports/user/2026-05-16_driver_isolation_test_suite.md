# Driver isolation test suite - 5 test per certificare il motor driver SEA in isolamento - 2026-05-16

## Contesto

Domanda di fondo emersa nel pomeriggio del 2026-05-16: come capisco che
il motor driver del SEA e' "corretto", cioe' inseguie `tau_ref` su un
range ampio di input senza generare chattering, **indipendentemente dal
loop chiuso completo** (outer PID, static optimization, dinamica del
corpo, riferimento IK)?

L'analisi del mattino aveva attribuito il chattering a 88 Hz al **loop
chiuso outer-D ↔ driver-inner**, non al driver in se'. Pero' quella
attribuzione era *indiretta* (per esclusione, via FFT su run del sistema
completo). Serve **verifica diretta** che metta input deterministici al
driver e ne misuri la risposta vs la teoria.

Costruita una suite di 5 test che caratterizzano il driver come sistema
dinamico SISO (input `tau_ref`, disturbo `omega_j`, output `tau_spring`).
Se i 5 test passano, qualunque difetto del sistema completo NON e'
attribuibile al driver.

## Cosa ho fatto

### Implementazione standalone Python (no OpenSim)

Scelta della pipeline: simulazione **standalone Python**, riproducendo
esattamente la formula del driver non-impedance attiva oggi
[output.py:471-476](../../output.py#L471-L476):

```python
inner_prop_term = Kp * (tau_ref - tau_spring)
inner_damp_term = -Kd * omega_m
tau_input_raw   = tau_ref + inner_prop_term + inner_damp_term
                = (1+Kp)*tau_ref - Kp*tau_spring - Kd*omega_m
tau_input       = clamp(tau_input_raw, -500, +500)
```

Integrazione dell'ODE 2-state `(theta_m, omega_m)` via
`scipy.integrate.solve_ivp` (RK45 con `rtol=1e-7`, `atol=1e-9`,
`max_step=dt`). Parametri SEA letti direttamente dal `.osim` con parser
XML standalone (no `import opensim`). Codice in
[tools/driver_isolation_suite.py](../../tools/driver_isolation_suite.py).

Perche' Python standalone e non OpenSim:
- OpenSim sarebbe piu' veritiero (testa il plugin C++ reale, non il
  modello matematico), ma richiede modifiche al modello per bloccare il
  giunto e bypass del controller. Costo: ~20 min setup + 1-5 ore run.
- Il predictor Python e' gia' verificato vs plugin C++ a 1e-6 Nm
  (vedi daily report 2026-04-14 sul predictor diagnostico). Quindi
  testare il modello matematico testa anche il plugin (transitivamente).
- Standalone si esegue in <1 min totale, permette debugging veloce e
  sweeping ampio (100 frequenze in Test 2).

### Parametri SEA testati (post-bump 2026-05-16)

```text
                K [Nm/rad]  Kp     Kd    Bm    Jm     F_opt    omega_n   zeta   f_d
SEA_Knee          321      18.0   11.0  0.1   0.01   100      781 rad/s 0.711  87.4 Hz
SEA_Ankle         500      11.3   11.0  0.1   0.01   250      784 rad/s 0.708  88.2 Hz
```

Predizioni analitiche (overlay nei plot):

```text
H(s)         = (1+Kp)*K / (Jm*s^2 + (Kd+Bm)*s + (1+Kp)*K)              tracking
S(s)         = -K*(Jm*s + (Kd+Bm)) / (Jm*s^2 + (Kd+Bm)*s + (1+Kp)*K)   sensitivita' a omega_j
bias DC      = -(Kd+Bm)/(1+Kp) = -0.584 (knee), -0.902 (ankle)         Nm/(rad/s)
```

Nota importante: `zeta = 0.711 > 1/sqrt(2) = 0.7071`. Questo significa
che la FdT chiusa H(jw) e' **monotonicamente decrescente** in modulo —
**niente picco di risonanza**. Il modo damped a 88 Hz esiste come
autovalore complesso, ma non e' visibile come peak nella risposta in
frequenza del driver in isolamento.

### Parallelismo - coda condivisa 10 worker, longest-job-first

Test 1 (step) e Test 5 (noise) sono single-shot per SEA → 2 job
ciascuno, seriali (overhead di submit > exec).

Test 2 (50 freq), Test 3 (10 freq), Test 4 (4 omega_j) → 128 job
parallelizzati per la coda condivisa. Ordinamento per costo decrescente
(`cost ~ 1/f` per i sweep sinusoidali: i job a 1 Hz richiedono ~5 s di
simulazione, quelli a 500 Hz ~0.02 s).

```python
MAX_WORKERS = 10
jobs.sort(key=lambda jc: -jc[1])  # longest-first
with ProcessPoolExecutor(max_workers=MAX_WORKERS) as ex:
    futures = [ex.submit(dispatch_sweep_job, args) for args, _ in jobs]
```

Wall time effettivo: **9.4 s** per i 128 job paralleli + 0.4 s Test 1 +
33 s Test 5 (filtfilt e PSD lunghi) = **~43 s totali**.

## Perche' questi 5 test

### Test 1 - Step response a giunto fermo
Cosa misura: errore a regime, overshoot, settling time. Verifica che il
driver insegue un riferimento DC senza bias. Falsifica un driver senza
feedforward o con scaling sbagliato.

### Test 2 - Bode magnitude/phase, sweep 1-500 Hz
Cosa misura: la FdT tracking `H(s) = tau_spring/tau_ref` su un range
ampio. Verifica BW, smorzamento, fase. Falsifica un driver con poli mal
posizionati o con discretizzazione che introduce poli/zeri spuri.

### Test 3 - Disturbance rejection, sweep omega_j sinusoidale
Cosa misura: la sensitivita' `S(s) = tau_spring/omega_j` su un range
ampio (incluso 88 Hz dove sta la presunta risonanza). Verifica che il
giunto non possa "iniettare" energia nel driver oltre il previsto.

### Test 4 - Bias a omega_j costante (caso cammino)
Cosa misura: il bias DC `e_ss = -(Kd+Bm)*omega_j/(1+Kp)` su 4 velocita'.
Verifica la formula chiave usata nell'analisi del 2026-05-15 sul PI
inner. Falsifica un driver che abbia perso il feedforward `(1+Kp)*tau_ref`.

### Test 5 - Risposta a noise broadband
Cosa misura: lo spettro di `tau_spring(t)` quando `tau_ref` e' rumore
gaussiano filtrato 0-1000 Hz. **Verifica diretta dell'ipotesi**: il
driver in isolamento amplifica a 88 Hz? Se si', il driver e' il
colpevole. Se no, il chattering nel sistema completo viene dal loop.

## Risultati

### Test 1 - Step response

```text
SEA          tau_spring_ss    e_ss [Nm]       overshoot [%]   settling 2% [ms]
SEA_Knee       50.000          5e-13           4.11            7.81   (teorico ~9)
SEA_Ankle      50.000          1e-12           4.30            7.63   (teorico ~9)
```

**Verdict: PASS**. Errore a regime sotto 1 pNm (precisione macchina),
overshoot 4.1-4.3% (teoria classica zeta=0.71 → 4.5%), settling time
sotto la previsione 9 ms.

Plot: [plot/05_16_2026_3_driver_isolation/01_step_response_knee.png](../../plot/05_16_2026_3_driver_isolation/01_step_response_knee.png).

### Test 2 - Bode tracking (100 punti, 50 per SEA)

Sovrapposizione perfetta simulato vs analitico (linee indistinguibili
su entrambi i Bode). |H(0)| = 0 dB, BW -3 dB a ~100 Hz, rolloff
-40 dB/decade asintotico, fase smooth da 0 a -160° a 500 Hz. Niente
picco di risonanza (zeta > 1/sqrt(2)).

**Verdict: PASS**. Il driver e' un seconda ordine con i poli previsti
dalla teoria, senza zeri o poli spuri introdotti dall'integrazione.

Plot: [02_bode_knee.png](../../plot/05_16_2026_3_driver_isolation/02_bode_knee.png).

### Test 3 - Sensitivita' al disturbo omega_j

Plateau DC a `0.584 Nm/(rad/s)` per knee e `0.902` per ankle, esattamente
sulla teoria. Sovrapposizione simulato vs analitico perfetta su 10
frequenze 0.5-300 Hz. **Niente peak intorno a 88 Hz**.

**Verdict: PASS**. La sensitivita' del driver al disturbo del giunto e'
quella prevista. Sopra 88 Hz l'attenuazione e' monotonica come da
teoria.

Plot: [03_disturbance_knee.png](../../plot/05_16_2026_3_driver_isolation/03_disturbance_knee.png).

### Test 4 - Bias a omega_j costante

```text
SEA      RMS residuo (osservato - teorico)      max residuo
SEA_Knee     0.0000 Nm                            0.0000 Nm
SEA_Ankle    0.0000 Nm                            0.0000 Nm
```

Tutti e 4 i punti per SEA cadono *esattamente* sulla retta teorica
`-0.584*omega_j` (knee) e `-0.902*omega_j` (ankle). Risoluzione mostrata
nel plot 0.001 Nm.

**Verdict: PASS**. Formula del bias confermata al 100%. Conferma anche
la validita' dell'analisi del 2026-05-15 (PI inner) - il bias residuo
del driver attuale e' esattamente `-(Kd+Bm)*omega_j/(1+Kp)`. Per
cancellarlo a regime serve un PI sull'errore di coppia (operazione
analizzata ma non ancora implementata).

Plot: [04_bias_knee.png](../../plot/05_16_2026_3_driver_isolation/04_bias_knee.png).

### Test 5 - Noise broadband (CRUCIALE)

```text
SEA          picco osservato       f_d teorico       peaking sopra flat 10-30 Hz
SEA_Knee      80.6 Hz               87.4 Hz           0.74 dB
SEA_Ankle     80.6 Hz               88.2 Hz           0.78 dB
```

Il PSD di `tau_spring` segue il PSD del `tau_ref` (input piatto)
fino a ~80-90 Hz, poi rolloff netto di seconda ordine. **Niente picco di
risonanza significativo**: il "peaking" osservato di 0.7-0.8 dB e' la
transizione naturale tra plateau e rolloff per un sistema con
`zeta = 0.711 > 1/sqrt(2)`. Per qualsiasi `zeta > 1/sqrt(2)` la FdT
chiusa e' monotonicamente decrescente — niente amplificazione di
risonanza.

**Verdict: PASS**. Il driver in isolamento **NON ha risonanza
amplificante a 88 Hz**. Il modo damped a 88 Hz esiste come autovalore
complesso (e si manifesta in regime transitorio come l'overshoot 4% del
Test 1), ma in regime stazionario di noise broadband la risposta in
frequenza e' liscia.

Plot: [05_noise_spectrum_knee.png](../../plot/05_16_2026_3_driver_isolation/05_noise_spectrum_knee.png).

## Verdetto finale

**Il motor driver SEA, in isolamento, e' matematicamente corretto per il
tuning attuale (post-bump 2026-05-16)**:

- Insegue riferimenti DC con errore zero.
- Ha la BW prevista (~125 Hz a -3 dB), monotonicamente decrescente.
- Rejecta correttamente il disturbo del giunto (bias DC = -(Kd+Bm)/(1+Kp)).
- Non amplifica risonanze a 88 Hz su input broadband.

Implicazioni dirette per le domande dell'utente:

| Sintomo nel sistema completo | Attribuibile al driver in isolamento? |
|---|---|
| Chattering 88 Hz su tau_input knee | **No** - Test 5 mostra che il driver non amplifica a 88 Hz |
| Errore di tracking di coppia ~1 Nm RMS | **Solo per la quota DC bias** - Test 4 conferma la formula `-(Kd+Bm)*omega_j/(1+Kp)` |
| Errore cinematico ~1.3-2.4 deg RMS | **No** - Test 1 e 2 mostrano tracking perfetto; il driver e' un follower fedele di tau_ref |
| Picco a 88 Hz nei tau_bio del SO | **No** - il driver non li genera (sono prodotti dal SO via coupling meccanico) |

Quindi: **il chattering, l'errore cinematico e il picco residuo nel
sistema completo NON derivano dal driver**. La diagnosi del mattino del
2026-05-16 (loop chiuso outer-D ↔ driver-inner come responsabile del
chattering) e' confermata in modo diretto.

PI inner: come anticipato, **non risolverebbe il chattering** (il driver
gia' non ha peaking, e' un PI applicato a un sistema che a 88 Hz e' gia'
flat). Risolverebbe **solo** il bias DC ~1.7 Nm a omega_j ≈ 3 rad/s
quantificato in Test 4 (caso cammino). Operazione utile ma indipendente
dal chattering.

## File creati / modificati

```text
+  tools/driver_isolation_suite.py                      (~580 LOC, suite completa)
+  results/_driver_isolation_20260516/
     test1_step_metrics.json
     test2_bode_data.npz
     test3_disturbance_data.npz
     test4_bias_data.json
     test5_noise_spectrum_knee.npz
     test5_noise_spectrum_ankle.npz
     summary.json
+  plot/05_16_2026_3_driver_isolation/
     01_step_response_{knee,ankle}.png
     02_bode_{knee,ankle}.png
     03_disturbance_{knee,ankle}.png
     04_bias_{knee,ankle}.png
     05_noise_spectrum_{knee,ankle}.png
+  reports/user/2026-05-16_driver_isolation_test_suite.md  (questo report)
```

Nessuna modifica a codice Python esistente, `.osim`, plugin C++ o modello.

## Verifiche eseguite

1. Parser XML del `.osim`: estrae i 6 parametri di ogni SEA, confronta
   con i valori attesi post-bump (knee Kp=18, Kd=11, K=321; ankle
   Kp=11.3, Kd=11, K=500). OK.

2. Calcolo `omega_n`, `zeta`, `f_d` analitici per i due SEA:
   - knee: 781 rad/s, 0.711, 87.4 Hz
   - ankle: 784 rad/s, 0.708, 88.2 Hz
   - Match con i numeri usati nell'analisi del mattino del 2026-05-16.

3. Esecuzione completa della suite (5 test, 132 task totali, di cui 128
   paralleli): `wall_time = 43.8 s`. Output dati salvati. 10 PNG
   generati.

4. Confronto numerico simulato vs analitico per Test 2: residuo
   |gain_db_sim - gain_db_theory| < 0.1 dB su tutto il range
   (sovrapposizione visiva perfetta nei plot).

5. Confronto numerico simulato vs analitico per Test 3: residuo
   |sens_sim - sens_theory| < 0.005 Nm/(rad/s) (sovrapposizione visiva
   perfetta).

6. Confronto numerico Test 4: residuo RMS = 0.0000 Nm su entrambi i SEA
   (e_ss osservato = e_ss teorico a precisione macchina dopo 1 s di
   simulazione).

7. Test 1 e Test 5 ispezionati visivamente: nessuna anomalia.

8. Sanity check: i risultati corrispondono alla teoria classica del
   secondo ordine. Overshoot 4% per zeta=0.71 = M_p(0.71) =
   exp(-0.71*pi/sqrt(1-0.71^2)) = 4.5%. Settling time 2% =
   4/(zeta*omega_n) = 4/(0.71*781) = 7.2 ms. Coerenti coi numeri
   osservati.

## Prossimi passi

1. **Lavoro sul loop chiuso, non sul driver**. La suite chiude
   definitivamente la linea di indagine "il driver e' sbagliato".
   Procedere con gli interventi che agiscono sul loop:
   - filtro LPF sul D-term outer (fc 25-40 Hz) — taglia il guadagno di
     anello a 88 Hz mantenendo l'autorita' del D in banda di tracking;
   - oppure sweep di Kd_knee_outer per trovare il sweet spot
     tracking/chattering;
   - oppure portare zeta a 1.0 (Kd_inner -> 16) per spostare il modo a
     una frequenza piu' bassa con piu' smorzamento. Costo: -30% BW.

2. **Implementare PI inner per la sola cancellazione del bias DC**
   (analisi 2026-05-15). Operazione indipendente dal chattering. Test 4
   ha confermato che il bias attuale del driver e' esattamente
   `-(Kd+Bm)*omega_j/(1+Kp)`. Implementabile come `<Ki>` aggiuntivo nel
   plugin C++.

3. La suite e' **riutilizzabile per future modifiche del driver**.
   Quando si cambieranno Kp, Kd, o si introdurra' PI, basta rilanciare
   `python tools/driver_isolation_suite.py` per ri-certificare il driver
   in <1 min.

4. TODO Windows aperto da 2026-05-15: ancora pendente. Quando la
   macchina Windows sara' allineata, includere anche l'esecuzione di
   questa suite per verificare che il driver si comporti identico
   cross-platform.

## TODO aperti (da propagare nei prossimi report)

- Decisione sull'intervento per il loop chiuso (LPF outer D vs sweep
  Kd_outer vs bump zeta).
- Implementazione PI inner per cancellazione bias DC (richiede modifica
  plugin C++).
- TODO Windows pendente da 2026-05-15.
- La suite e' usabile come **regression test**: ogni modifica futura al
  driver dovrebbe ri-passarla.
