# Inference del seed imitativo con GRF ibrida

**Data:** 2026-06-11

## Problema

Individuare con certezza il risultato corretto del training imitativo appena
concluso, selezionare il checkpoint da usare come seed e verificare il suo
comportamento tramite un'inference deterministica completa.

Era inoltre necessario confermare la semantica delle forze di reazione al
suolo: GRF online applicata fisicamente sul lato protesico sinistro e GRF
prescritta applicata sul lato sano destro.

## Risultato del training individuato

Il run corretto e:

`Trajectory Generator/runs/baseline_mlp_imit_win`

Il training e terminato regolarmente con:

- stato `ok: true`;
- motivo di arresto `completed`;
- 40 iterazioni completate su 40;
- return iniziale `7.5496333564789735`;
- return finale e migliore `47.614022485910034`;
- migliore iterazione: 40;
- `vf_explained_var` finale: `0.811668872833252`;
- `vf_loss` finale: `0.25241801142692566`.

Il checkpoint selezionato per l'inference e:

`Trajectory Generator/runs/baseline_mlp_imit_win/rl_module_best`

I checkpoint `rl_module_best` e `rl_module_last` sono identici, coerentemente
con il fatto che il risultato migliore sia stato ottenuto all'ultima
iterazione. Entrambi hanno SHA-256:

`9B42621B90BA40DF140C218E8A1C81F23E3C0D235CC49773F6C6A1647CF4118A`

## Strategia

L'inference e stata eseguita usando esplicitamente i principali parametri del
training:

- reward mode `imitation`;
- action mode `absolute`;
- durata episodio `2.0 s`;
- durata segmento `0.01 s`;
- 3 policy knots;
- osservazione GRF online abilitata;
- lato GRF online applicato: `left`;
- registrazione completa degli output del simulatore.

Il run non conteneva `training_cfg.resolved.yaml`. Per evitare ambiguita tra
training e valutazione, la configurazione necessaria e stata quindi fornita
esplicitamente al comando di inference.

L'esecuzione e stata sorvegliata con timeout per step, stallo e durata totale,
monitorando avanzamento, terminazione e produzione degli output.

## Verifica della GRF ibrida

Il log dell'inference conferma la configurazione attesa:

- lato protesico sinistro: GRF prescritta disabilitata nella dinamica e GRF
  online applicata;
- lato sano destro: GRF prescritta applicata;
- 20 contatti configurati, di cui 8 applicati sul lato sinistro e 12 usati
  come sensori.

La modalita `online_sensor`, insieme a
`online_grf_applied_sides: ["left"]`, realizza quindi la dinamica ibrida
richiesta.

## Risultato dell'inference

Gli output sono stati salvati in:

`Trajectory Generator/runs/baseline_mlp_imit_win_rollout`

Risultati principali:

- stato `ok: true`;
- return code `0`;
- 201 step di policy;
- tempo di esecuzione a parete: circa `371.25 s`;
- episode return: `92.20533942006523`;
- reward media: `0.45873303194062304`;
- reward minima: `0.0`;
- reward massima: `0.9314020007349606`;
- massimo valore assoluto dell'azione: `0.9862317442893982`;
- altezza minima del bacino `pelvis_ty`: `0.9263371789473765`;
- `terminated: false`;
- `truncated: true`, dovuto al normale limite temporale dell'episodio.

L'esecuzione e terminata senza timeout e senza processi residui.

## Diagnostica biomeccanica

La diagnostica SEA mostra:

- saturazione del ginocchio: `679 / 2001` campioni, circa `33.93%`;
- intervallo della coppia di input del ginocchio: `-500` a `+500 N m`;
- saturazione della caviglia: `0 / 2001` campioni;
- intervallo della coppia di input della caviglia: circa `-45.52` a
  `+68.13 N m`.

La diagnostica del recruitment biologico mostra:

- norma media delle reserve: `798.624682179531`;
- norma massima delle reserve: `2886.35099036`;
- quota muscolare media: `0.136969322713643`;
- quota muscolare massima: `0.62308323`;
- norma media delle reserve sui DOF non attuati: `713.436580114932`;
- norma massima delle reserve sui DOF non attuati: `2541.90468981`;
- residuo di equilibrio medio e massimo pari a `0`;
- frequenza media di failure di equilibrio: `0.0184907546226887`.

Sono presenti valori `NaN` esclusivamente nei due canali diagnostici opzionali:

- `SEA_Knee_torque_error_integral_dot`;
- `SEA_Ankle_torque_error_integral_dot`.

Gli altri output e la dinamica della simulazione sono stati completati
regolarmente.

## Output prodotti

La cartella `sim_outputs` contiene:

- 16 file STO completi;
- 2001 righe per ciascun file STO;
- file CSV degli eventi del passo;
- diagnostica SEA, GRF, recruitment, cinematica e dinamica.

## File modificati

- `reports/user/2026-06-11_inference_seed_imitativo_grf_ibrida.md`: creato
  questo report.

Non sono state apportate modifiche al codice del simulatore, al training o al
plugin C++.

## Verifiche eseguite

- verificata la conclusione regolare delle 40 iterazioni di training;
- confrontati hash di `rl_module_best` e `rl_module_last`;
- verificata la selezione del checkpoint migliore;
- eseguita e monitorata un'inference completa;
- verificata la terminazione senza errori o timeout;
- verificata la corretta applicazione della GRF ibrida;
- verificata la presenza e la consistenza dimensionale degli output;
- analizzate saturazioni SEA e diagnostiche del recruitment biologico.

## TODO

- Analizzare e ridurre la saturazione del SEA del ginocchio, attualmente
  prossima al 34% dei campioni.
- Investigare l'elevata richiesta alle reserve biologiche, in particolare sui
  DOF non attuati.
- Salvare sempre `training_cfg.resolved.yaml`, o metadati equivalenti accanto
  al checkpoint, per rendere riproducibile ogni futura inference.
- Popolare correttamente i canali
  `SEA_Knee_torque_error_integral_dot` e
  `SEA_Ankle_torque_error_integral_dot`, oppure marcarli esplicitamente come
  non disponibili.
- Validare direttamente la qualita imitativa con metriche cinematiche dedicate,
  inclusi errore RMSE e coerenza di fase.
- Usare il checkpoint imitativo come warm start dopo il completamento e la
  validazione della reward task-based ex novo.
