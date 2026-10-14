# 2026-09-24 — LEFT peso 2: risultati della prosecuzione da best80 a100

## Esito

Il training ha completato correttamente i 20 aggiornamenti 81–100. La
prosecuzione non ha migliorato il candidato best80 di partenza nei rollout
deterministici esaminati. Il nuovo best82 e last100 completano l'episodio,
ma peggiorano il tracking della caviglia e quasi raddoppiano l'integrale della
sua coppia positiva. La robustezza durante il training resta insufficiente.

L'indicazione è mantenere **best80 come candidato preferibile fra quelli confrontati**.
Questo giudizio riguarda la qualità nominale osservata: manca ancora uno
screening con perturbazioni comuni a policy congelata per confrontarne la
robustezza. Sono stati prodotti i sette plot canonici per ciascuno dei due
nuovi checkpoint.

## Problema e strategia della verifica

La prosecuzione richiesta doveva chiarire se altre 20 iterazioni, partendo
dal best80 con peso di penetrazione 2, migliorassero imitazione, coppie e
completamento degli episodi. Il confronto riguarda questa continuazione;
non costituisce una nuova prova di equivalenza statistica con V26.

La verifica ha separato tre aspetti:

1. Integrità dell'apprendimento: aggiornamenti effettivi, campioni raccolti,
   visibilità dei prescribed e conservazione del checkpoint sorgente.
2. Andamento degli episodi durante il training: completamento, cause di
   arresto e cicli validi, tenendo conto dell'esplorazione e dei cambi di policy.
3. Qualità a pesi fissi: rollout deterministici di best e last confrontati con
   best80 su target, start, reward e finestra di misura comuni.

La conclusione operativa è conservare il candidato precedente e affrontare
separatamente robustezza e qualità delle coppie. Non è stata implementata una
nuova correzione della reward o del controller sulla base di questa analisi.

## Provenienza e correttezza del training

Run: `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT80_penetration_weight2_to100_2026-09-24`.

Origine: `checkpoint_best`, iterazione80, del run
`MLP39_LEFT72_penetration_weight2_10iter_2026-09-24`.
Il restore di actor, critic, Adam e KL era stato verificato all'avvio nel
[report dedicato](2026-09-24_avvio_left_best80_peso2_fino100.md).

- Conclusione: 24 settembre, **19:57:15 CEST**; durata registrata 9.220,58 s,
  circa 2 h 33 min 41 s.
- 20 aggiornamenti, tutti con batch4096: **81.920 nuove transizioni**.
- Actor medio e critic aggiornati a ogni iterazione; disponibilità prescribed
  100%; nessuna riga di telemetria mancante nei batch.
- LEFT guidato dalla fase sinistra, clock visibile, FSMv3, peso penetrazione2
  e resto della configurazione mantenuti nella prosecuzione.
- 179 episodi terminati registrati dalla telemetria, senza duplicati o record
  invalidi. I contatori coincidono con quelli del trainer.
- I 28 file del checkpoint sorgente conservano gli hash precedenti all'avvio.

Il selettore del nuovo run ha salvato **best82** e **last100**. Best82 indica
il migliore fra i nuovi aggiornamenti, non un miglioramento garantito su
best80. Il massimo ritorno medio di training del nuovo ramo è255,13,
contro261,75 riportato per best80 nel ramo sorgente. Sono medie di episodi di
training differenti, non valutazioni appaiate della stessa popolazione.

Il nuovo best82 è inoltre distinto dal precedente *last82*, prodotto dalla
prosecuzione da72. Il suffisso `from80` delle cartelle evita l'ambiguità.

## Completamento e fallimenti nel training

| Intervallo | Episodi completi / terminati | Completamento |
|---|---:|---:|
| 81–85 | 26/39 | 66,7% |
| 86–90 | 28/48 | 58,3% |
| 91–95 | 25/43 | 58,1% |
| 96–100 | 26/49 | **53,1%** |
| Tutto il nuovo ramo | 105/179 | **58,7%** |

I 74 fallimenti comprendono69 terminazioni ordinarie per penetrazione,
2 protezioni hard per penetrazione,2 divergenze del ginocchio e1 evento binario
invalido. **71/74 fallimenti sono quindi associati alla penetrazione**.
Non emerge una crescita stabile del completamento. Anche il confronto
descrittivo con il precedente ramo73–82,54/86 completi (62,8%), non mostra
un miglioramento; non è un confronto statistico controllato.

Il completamento temporale non basta a descrivere una camminata corretta:
dei105 episodi completi,10 registrano zero cicli validi,36 un ciclo,42 due,
16 tre e1 quattro. Inoltre91/105 superano almeno una volta20 mm di penetrazione.
Questa è la soglia soft della penalità, non la soglia di arresto, perciò non
c'è contraddizione con il raggiungimento del termine dell'episodio.

174/179 episodi attraversano almeno un aggiornamento della policy. Le
percentuali sopra descrivono la raccolta durante l'apprendimento e
l'esplorazione; **non sono probabilità di successo di best82 o last100 a
pesi fissi**. I79.731 passi contenuti negli episodi terminati differiscono
dalle81.920 transizioni dei batch anche per i frammenti degli episodi in corso
ai confini della finestra di analisi.

## Rollout deterministici e confronto con best80

I due nuovi rollout hanno usato selezione deterministica, start concordato,
stesso target LEFT e reward con peso penetrazione2. Entrambi completano
500 passi,5 secondi,2 cicli validi, con fine per limite temporale.
Best80 è confrontato tramite il suo rollout deterministico già disponibile.

RMSE e metriche delle coppie sono calcolati nella stessa finestra fisica
13,15–17,95 s. Gli RMSE nella tabella confrontano **giunto fisico e target
imitativo LEFT**. Ritorno e penetrazione massima si riferiscono all'intero
episodio. La coppia è quella elastica SEA applicata al giunto.

| Metrica | Best80 sorgente | Nuovo best82 | Last100 |
|---|---:|---:|---:|
| RMSE ginocchio [°] | **6,38** | 6,63 | 6,74 |
| RMSE caviglia [°] | **4,04** | 5,51 | 5,73 |
| Picco positivo coppia caviglia [Nm] | 29,17 | 30,12 | 29,16 |
| Integrale coppia positiva caviglia [Nm·s] | **8,61** | 16,74 | 16,15 |
| RMS coppia ginocchio, passa-alto10 Hz [Nm] | 8,20 | **7,81** | 9,20 |
| RMS coppia ginocchio, passa-alto20 Hz [Nm] | 3,78 | 3,45 | **3,40** |
| Penetrazione massima [mm] | 14,80 | 15,94 | 14,80 |
| Ritorno deterministico, stessa reward | **305,06** | 256,36 | 259,15 |

L'errore alla caviglia cresce del36,3% in best82 e del41,8% in last100.
L'integrale positivo cresce rispettivamente del94,5% e87,6%: il solo picco,
quasi invariato, nasconde un peggioramento dell'estensione temporale delle
coppie positive. Questo integrale è un indicatore descrittivo, non una
penalità fisiologica che attribuisce identico significato a tutte le fasi.

Il ginocchio non ha un miglioramento uniforme: best82 riduce moderatamente
le componenti sopra10 Hz; last100 le aumenta del12,3%, pur riducendo quelle
sopra20 Hz. La misura usa un Butterworth passa-alto di ordine4 applicato
avanti e indietro. Considerare soltanto20 Hz farebbe perdere parte rilevante
del chattering attorno a11–13 Hz.

Le figure7 confermano il target sinistro corretto e mostrano una caviglia
eccessivamente dorsiflessa in alcune porzioni del ciclo, soprattutto verso
la fine. La riga centrale mostra la gamba destra grezza soltanto come contesto;
il confronto e l'errore usano il target LEFT effettivamente registrato.

## Cosa chiariscono questi risultati sulle coppie

Il peggioramento conserva il meccanismo descritto nel
[report sulle coppie di best80](2026-09-24_best80_cause_coppie_ankle_knee.md).
Nel nuovo best82, al picco intorno a17,485 s, la coppia caviglia è+30,12 Nm;
il comando è+30,33 Nm, composto da circa+1,54 Nm proporzionali e+28,79 Nm
integrali. Il momento GRF del campione più vicino è−29,16 Nm. Nel tratto
sotto carico la correlazione fra coppia elastica e opposto del momento GRF
è0,9987. Non risultano saturazioni dell'ingresso SEA né clamp dell'integrale
della cascata nella finestra analizzata.

Il giunto segue ancora strettamente la reference generata: RMSE
giunto/reference alla caviglia0,44° e0,45° nei nuovi checkpoint, contro0,34°
in best80. Gran parte dell'errore rispetto al prescribed riguarda dunque la
reference prodotta dalla policy, non un'incapacità del SEA di seguirla.
Questo non isola causalmente il contributo del filtro, invariato nei tre casi.

| Costo cumulativo, intero episodio | Best80 | Nuovo best82 | Last100 |
|---|---:|---:|---:|
| Penalità penetrazione | 0 | 0 | 0 |
| Penalità momento caviglia invertito | 4,69 | 15,15 | 10,14 |
| Penalità variazione dei comandi | 75,58 | 74,56 | 76,03 |

La penetrazione nominale rimane sotto20 mm: il suo peso2 non esercita un
costo diretto in questi tre rollout, pur influenzando l'apprendimento sugli
episodi esplorativi. La penalità anti-flip rileva il peggioramento; non è
assente. Le penalità dedicate al rateo e all'intensità della coppia elastica
hanno ancora peso zero. Il costo dei comandi rimane simile nonostante la
diversa qualità delle coppie, coerentemente con la precedente diagnosi della
sua prevalente dipendenza dalla distanza endpoint/reference filtrata.

Queste osservazioni non dimostrano che sia sufficiente aumentare un peso,
né distinguono da sole tutti gli effetti di reward, esplorazione e
ottimizzazione PPO. Mostrano che proseguire con la configurazione invariata
non ha corretto il compromesso osservato. Non certificano inoltre la
validità biomeccanica del modello di contatto.

## Plot canonici e artefatti

- [Set best80 sorgente, usato per confronto](../../plot/09_24_2026_unified39_LEFT_pen2_best80/).
- [Set completo best82](../../plot/09_24_2026_unified39_LEFT_pen2_from80_best82/).
- [Set completo last100](../../plot/09_24_2026_unified39_LEFT_pen2_from80_last100/).
- [Diagnostica contatto/controller best82](../../plot/09_24_2026_LEFT_from80_to100_diagnosis/new_best82_contact_controller.png).

Ciascun set canonico contiene:

1. `01_time_sea_control_reserve.png`
2. `02_time_joint_motor_states.png`
3. `03_gaitcycle_torque_angle_power.png`
4. `04_gaitcycle_joint_velocity_power.png`
5. `05_time_tau_input_tracking_error.png`
6. `06_time_joint_ref_sea_error.png`
7. `07_mlp_policy_vs_sound_leg_error.png`

Rollout salvati sotto `Trajectory Generator/unified_MLP_39/runs/rollout/`,
con prefisso `MLP39_LEFT_pen2_from80_to100_2026-09-24_` e suffissi
`best82_deterministic` / `last100_deterministic`.

Analisi riproducibile in
`Trajectory Generator/unified_MLP_39/validation/left80_to100_results_2026-09-24/`:
`analyze_training.py`, `analyze_rollouts.py`, `verify_artifacts.py`,
`training_summary.json`, `training_cycle_quality.json`, `metrics.json`,
`telemetry/`, `input_hashes.json`, `verification.json`.
Comandi, log e ricevuta dei plot sono nella cartella
`runs/plans/left80_pen2_to100_evaluation_2026-09-24/`.

## Verifiche e modifiche

PASS: conteggio aggiornamenti e batch; visibilità prescribed; integrità
telemetria; ricostruzione della reward dai trace entro1e-5 sul ritorno;
ricostruzione del costo comandi entro1e-8; identità comando=P+I entro1e-6 Nm;
lettura di tutti14 PNG; assenza di canali mancanti; corrispondenza ricevute,
rollout e checkpoint; immutabilità dei28 input analizzati e dei28 file del
checkpoint sorgente. Ispezionate visivamente le figure1 e7 di entrambi i set.

Creati script e risultati di analisi, rollout, plot e questo report;
aggiornato il daily. Nessuna modifica alla logica di training, reward,
controller, FSM o plugin C++ durante questa valutazione.

## Indicazione operativa e TODO

Conservare best80 come riferimento nominale. Non emerge un motivo per
preferire last100 o per estendere automaticamente questo stesso training.
Prima di un'ulteriore modifica occorre separare la verifica di robustezza
dalla scelta di un intervento mirato sulle coppie.

- [x] Verificare conclusione a100 e81.920 nuove transizioni.
- [x] Valutare best e last nominali e produrre i14 plot canonici in root.
- [ ] Confrontare la robustezza di best80 e degli eventuali candidati con
  perturbazioni comuni a policy congelata; il presente risultato non esaurisce
  questo controllo.
- [ ] Se si modifica la reward per le coppie, predisporre un confronto
  controllato dal medesimo checkpoint e con budget identico, usando criteri
  fisici e cinematici comuni.
- [ ] Il controllo peso1 contro2 rimane incompleto; la prosecuzione a100
  non sostituisce quel confronto e non isola causalmente il peso2.

Nessun ulteriore training avviato.
