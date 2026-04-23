# Daily Report - 2026-04-22

## Sintesi

La giornata del 22 aprile 2026 ha ruotato attorno a un sospetto preciso:
il plateau di tracking del simulatore plugin-based non dipendesse piu solo
dal tuning outer, ma anche da un limite strutturale del controller interno
del plugin SEA.

Il lavoro si e' mosso in quattro passaggi:

1. **review del plugin C++** `SeriesElasticActuator` per verificare se la
   dinamica SEA e la legge di `tau_input` fossero implementate correttamente;
2. **identificazione di un bias statico** nel ramo non-impedance:
   la legge originale faceva convergere `tau_spring` a una frazione di
   `tau_ref`, non a `tau_ref`;
3. **implementazione di una variante con feedforward realistico** nel
   comando motore, compilata come DLL separata con suffisso `ff`, copiata nel
   simulatore e selezionata in `config.py`;
4. **confronto sistematico dei run completi** per capire se il plugin `ff`
   spostasse davvero il tracking oppure solo la diagnostica interna.

Il risultato finale della giornata e' netto:

- la **dinamica meccanica del SEA** nel plugin e' sostanzialmente corretta;
- la **legge originale di `tau_input`** nel ramo non-impedance aveva davvero
  un errore statico strutturale;
- la variante **`ff` migliora il tracking protesico verso l'IK** in modo
  reale, soprattutto sul knee;
- il salto piu forte resta comunque il passaggio a **`T_control = 1 ms`**;
- i grafici rispetto all'healthy cambiano meno del previsto perche'
  **`ankle_r` e `mtp_r` restano quasi invariati**, quindi il collo di
  bottiglia visivo non e' solo nel tracking dei due DOF protesici.

**Stato finale a fine giornata**:

- plugin attivo: `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff`
- control window: `T_control = 0.001 s`
- integrazione: `integration_dt = 0.001 s`, `rk4_bypass`
- stiffness SEA ankle: `700 N m/rad`
- outer gains protesici:
  - knee `Kp=160`, `Kd=12`
  - ankle `Kp=420`, `Kd=1`
- filtro cinematica IK a `6 Hz` ancora attivo
- ultimo run completo disponibile in `results/`:
  `status=complete`, `T_control=0.001`, plot in `plot/22_04_2026 - 7`

## Report Utente Considerati

- `2026-04-22_plugin_ff_inner_loop_e_analisi_tracking.md`

## 1. Review del plugin SEA

E' stato analizzato il sorgente esterno:

- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\SeriesElasticActuator.cpp`
- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\SeriesElasticActuator.h`

### Dinamica meccanica

La struttura del plant SEA e' risultata coerente:

```text
tau_spring = K * (theta_m - theta_joint)
d(theta_m)/dt = omega_m
d(omega_m)/dt = (tau_input - tau_spring - Bm * omega_m) / Jm
```

Quindi la parte meccanica motore+molla e' stata giudicata corretta.

### Problema nella legge di `tau_input`

Nel ramo non-impedance il plugin usava:

```cpp
tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m;
```

A regime, con velocita' nulla, questo implica:

```text
tau_spring = Kp / (Kp + 1) * tau_ref
```

quindi il torque tracking interno aveva un **bias statico strutturale**.

Per i parametri del modello corrente:

- knee: `Kp = 3.9`  -> `tau_spring ≈ 0.796 * tau_ref`
- ankle: `Kp = 8.8` -> `tau_spring ≈ 0.898 * tau_ref`

La conclusione della review e' stata:

- **plant SEA corretto**
- **inner torque loop non pienamente trasparente**

## 2. Implementazione della variante `ff`

Per correggere il bias senza riscrivere il plugin, e' stata introdotta una
patch minima nel ramo non-impedance:

```cpp
tau_input = tau_ref + Kp * (tau_ref - tau_spring) - Kd * omega_m;
```

Questa scelta:

- introduce un feedforward realistico sul comando motore;
- elimina il bias statico del loop interno;
- non trasforma il SEA in un attuatore ideale;
- mantiene il plugin abbastanza vicino alla struttura originale.

### Build e integrazione

E' stata compilata una DLL separata:

```text
SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

e poi copiata in:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

Infine `config.py` e' stato aggiornato a:

```text
plugin_name = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"
```

### Smoke test

E' stata eseguita una run breve di verifica:

```text
results/_smoke_ff_plugin
```

Esito:

- DLL `ff` caricata correttamente;
- simulazione breve completata;
- output validi salvati senza errori di loading del plugin.

## 3. Confronto dei run principali

I confronti significativi della giornata sono stati quattro:

| Caso | Score | Knee RMS | Ankle RMS | Ankle_r RMS | MTP_r RMS |
|---|---:|---:|---:|---:|---:|
| `3 ms / 700 / no-ff` | 14.700 | 3.369 | 4.645 | 10.327 | 39.875 |
| `3 ms / 700 / ff`    | 13.179 | 2.665 | 4.135 | 10.335 | 40.029 |
| `1 ms / 700 / no-ff` | 11.111 | 3.338 | 4.587 | 3.232  | 11.776 |
| `1 ms / 700 / ff`    |  9.608 | 2.640 | 4.086 | 3.235  | 11.781 |

### Lettura dei risultati

1. **Il passaggio a `1 ms` e' stato la leva piu forte**.
   Il confronto `3 ms -> 1 ms` abbassa molto `score_total` e soprattutto
   riduce i side-effect distali (`ankle_r`, `mtp_r`).

2. **Il plugin `ff` migliora davvero il tracking protesico**.
   Sia a `3 ms` sia a `1 ms`, knee e ankle RMS scendono in modo chiaro.

3. **Il grafico 4 cambia meno di quanto suggeriscono le metriche**.
   Questo perche' `ankle_r` e `mtp_r` migliorano fortemente con `1 ms`,
   ma quasi non cambiano tra `no-ff` e `ff`.

4. **Il caso migliore della giornata** e' risultato:

```text
T_control = 1 ms
SEA_Ankle stiffness = 700
plugin = SEA_Plugin_BlackBox_mCMC_impedence_ff
```

con:

- `score_total = 9.608`
- `knee_tracking_rms_deg = 2.640`
- `ankle_tracking_rms_deg = 4.086`

### Effetto sul comando

Nel confronto `1 ms / 700 / no-ff` vs `1 ms / 700 / ff`, il plugin `ff`
non ha solo migliorato il tracking, ma ha anche ridotto il comando richiesto:

- `knee max|u|: 0.363 -> 0.288`
- `ankle max|u|: 0.310 -> 0.274`
- `max_tau_input_raw_abs: 70.350 -> 20.869 Nm`

Quindi il feedforward interno ha reso il sistema **piu efficace e meno
aggressivo** sul comando motore.

## 4. Interpretazione tecnica di fine giornata

La giornata ha chiarito un punto importante:

- il plateau che si vedeva dopo gli sweep outer **non era solo colpa del
  tuning protesico high-level**;
- c'era davvero un limite strutturale nell'inner loop del plugin, almeno nel
  ramo non-impedance;
- correggere quel limite aiuta in modo misurabile;
- pero' il salto "visivo" verso grafici piu simili all'healthy resta parziale
  perche' i side-effect distali non seguono lo stesso miglioramento dei DOF
  protesici.

In altre parole:

- **`1 ms` sblocca il comportamento globale**
- **`ff` rifinisce e migliora il tracking protesico**
- **il residuo mismatch rispetto all'healthy non sparisce ancora**

## 5. File toccati / aggiornati oggi

Nel repository del plugin:

- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\SeriesElasticActuator.cpp`
- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\CMakeLists.txt`

Nel repository simulatore:

- `config.py`
- `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`
- `reports/user/2026-04-22_plugin_ff_inner_loop_e_analisi_tracking.md`

Output / artefatti principali generati oggi:

- `results/_smoke_ff_plugin`
- `plot/22_04_2026 - 6`
- `plot/22_04_2026 - 7`

## 6. Verifiche eseguite

- review manuale del plugin `SeriesElasticActuator.cpp/.h`
- build CMake della DLL `SEA_Plugin_BlackBox_mCMC_impedence_ff`
- copia DLL nel simulatore
- smoke run breve del plugin `ff`
- analisi quantitativa dei run completi
- verifica dello stato finale del run in `results/sim_output_run_status.txt`

## Stato finale del repository

Il simulatore, a fine giornata, e' configurato per usare:

```text
plugin_name = plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff
T_control = 0.001
SEA_Ankle stiffness = 700
```

L'ultimo run completo disponibile risulta:

```text
status = complete
t_end = 11.06
T_control = 0.001
integration_scheme = rk4_bypass
sea_forward_mode = plugin
```

La base di lavoro per il giorno successivo e' quindi chiara:

- tenere `1 ms` come riferimento stabile;
- mantenere il plugin `ff` come baseline migliore attuale;
- concentrare la prossima analisi non piu sul solo outer loop,
  ma sul perche' `ankle_r` e `mtp_r` restino quasi fermi anche quando
  il tracking protesico migliora in modo evidente.
