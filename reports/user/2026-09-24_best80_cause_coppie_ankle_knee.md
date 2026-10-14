# Best80 LEFT: cause delle coppie anomale alla caviglia e del chattering al ginocchio

## Problema e conclusione

Il checkpoint best80 del ramo `MLP39_LEFT72_penetration_weight2_10iter_2026-09-24` migliora il tracking imitativo e completa il rollout deterministico, ma conserva un burst positivo alla caviglia e oscillazioni rapide della coppia al ginocchio. Sono due problemi distinti, accomunati da una reward che valuta soprattutto la cinematica e lascia debolmente vincolata la qualità delle coppie.

La caviglia compensa un momento sfavorevole del contatto; il ginocchio riceve già un comando di coppia oscillante dal cascade. Non emerge un errore di segno o una nuova saturazione del SEA. Il tracking giunto–reference servita resta molto preciso: 0.28° ginocchio e 0.34° caviglia sulla finestra comune 13.15–17.95 s. Questo dimostra l’esecuzione della reference, non la sua qualità dinamica.

La prova statica sul contatto indica che, al picco finale, avvicinarsi ai target LEFT può migliorare anche il momento. Non è quindi dimostrato un conflitto necessario tra imitazione LEFT e contatto accettabile. La soluzione appresa rimane imperfetta; il risultato globale del training dipende inoltre dal compromesso tra score, regolarizzazione, contatto ed esplorazione.

## Strategia e fonti

Analisi offline dei trace a100 Hz e degli STO a1 kHz di best80, last82, LEFT best72, V26 best87, RIGHT last60 e precedente comando diretto del target LEFT. Confronti temporali su 13.15–17.95 s; reward ricostruita sull’intero episodio di500 step. Per le componenti rapide: Butterworth ordine4 passa-alto a10 e20 Hz, fase zero, filtraggio sull’intero segnale e successivo ritaglio. Non si tratta di un filtro introdotto nella simulazione.

Riletti in particolare i report del [22 giugno sul contatto](2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md), [23 giugno sul chattering](2026-06-23_rollout_contact_validity_knee_chattering.md), [25 giugno sul limite jerk](2026-06-25_analisi_knee_jerk2000_e_sweetspot_grf.md), [20 luglio sull’ex-novo](2026-07-20_audit_exnovo_prescribed_reward_deployability_checkpoint_best.md) e [22 settembre su ritardo e coppie](2026-09-22_ritardo_filtro_coppia_ankle_e_proseguimento_unified39.md).

## 1. Caviglia: geometria del contatto e compensazione del cascade

Nel picco di best80 a17.474 s:

| Grandezza | Valore |
|---|---:|
| Coppia elastica SEA | +29.17 Nm |
| Coppia richiesta dal cascade | +29.21 Nm |
| Termine P cascade | +2.59 Nm |
| Termine I cascade | +26.62 Nm |
| Momento GRF nel campione trace vicino,17.470 s | −28.44 Nm |
| Forza normale allo stesso istante STO del picco | 460.28 N |
| Penetrazione al picco | 12.66 mm |
| COP_z − centro caviglia_z nel trace vicino | +60.90 mm |
| Angolo caviglia / target nel trace vicino | 10.74° / 7.30° |

La correlazione sotto carico tra coppia SEA e opposto del momento GRF è0.9984; il residuo RMS della loro somma è0.91 Nm. Il tracking tau_ref–tau_spring ha RMSE0.545 Nm. L’integrale del cascade sostiene la maggior parte della coppia per compensare il carico; non è al clamp e il motore non satura. La presenza di un integrale grande non basta a diagnosticare windup.

La spiegazione fisica è coerente con quella di giugno: posa del piede e distribuzione del contatto determinano COP e momento rispetto all’asse della caviglia; il controllore applica la coppia opposta per mantenere la cinematica. Il segno della sola forza verticale non determina quello del momento. Il valore COP_z è una diagnostica geometrica, mentre la penalità usa la proiezione completa del wrench sull’asse articolare.

Il burst sopra+8 Nm nell’ultimo appoggio occupa17.382–17.693 s. Picco e area positiva migliorano rispetto a best72 ma restano superiori alla baseline V26:

| Rollout | Picco positivo ankle [Nm] | Area positiva [Nm·s] | Penalità momento, intero episodio |
|---|---:|---:|---:|
| LEFT best72 peso1 | 47.44 | 25.02 | 43.11 |
| LEFT best80 peso2 | 29.17 | 8.61 | 4.69 |
| LEFT last82 peso2 | 38.59 | 21.98 | 29.75 |
| V26 best87 | 10.19 | 5.28 | 0.010 |
| RIGHT last60 | 25.89 | 11.03 | 6.04 |

Non tutta la coppia positiva è patologica: la baseline storica ne contiene transitori, già discussi e accettati ad agosto. Qui la questione è la magnitudine, durata e fase sotto carico. Il fenomeno compare anche in RIGHT last60: non è esclusivo del riferimento LEFT.

Il massimo di penetrazione di best80 nell’intero episodio è14.80 mm, dunque il costo di penetrazione è esattamente zero. Il suo peso2 resta rilevante durante l’apprendimento e nei rollout rumorosi, ma non distingue ulteriormente le soluzioni di questo episodio tutte sotto20 mm. Il guard sul momento è presente e attivo: non è stato perso un fix storico.

## 2. Intervento statico sulla posa: imitazione e momento possono migliorare insieme

Ricostruito lo stato cinematico a17.470 s e ricalcolato il contatto con lo stesso modello. La riproduzione della forza registrata differisce meno di0.000023 N; il momento rispetto al ground meno di0.000020 Nm. Le altre coordinate e tutte le velocità generalizzate sono mantenute fisse durante ciascuna modifica.

| Modifica istantanea | Momento GRF ankle [Nm] | Penetrazione [mm] | Costo anti-flip del campione |
|---|---:|---:|---:|
| Posa registrata | −28.84 | 12.22 | 0.678 |
| Solo caviglia al target LEFT | −6.09 | 11.27 | 0 |
| Solo ginocchio al target LEFT | −21.26 | 10.24 | 0.275 |
| Entrambi al target LEFT | +3.08 | 12.47 | 0 |
| Caviglia −1° rispetto alla posa registrata | −22.68 | 10.48 | 0.337 |
| Caviglia +1° rispetto alla posa registrata | −34.67 | 13.94 | 1.111 |

Il dato più informativo è che migliorare la posa rispetto al target elimina localmente la penalità sul momento, anche senza diminuire sistematicamente la penetrazione. Quest’ultima non è quindi un indicatore sufficiente della geometria del contatto.

Limiti: nessuna integrazione nel tempo; lo spostamento istantaneo delle coordinate non rappresenta una traiettoria eseguibile. Non misura effetti futuri su stabilità o coppie. Le velocità cartesiane dei punti di contatto possono cambiare al variare della posa anche mantenendo fisse le velocità generalizzate. Il contatto è ricostruito da posa/velocità; stati muscolari e SEA restano quelli del reset. La riproduzione del contatto registrato valida questa diagnostica locale, non un rollout dinamico. La piccola differenza tra momento statico−28.84 e trace−28.44 deriva dal wrench campionato/cached rispetto allo stato simultaneo.

## 3. Ginocchio: reference apparentemente liscia, velocità e coppia oscillanti

Il cascade implementa:

`qdot_cmd = qdot_ref + Kp_outer × (q_ref − q)`

`tau_ref = Kp_inner × (qdot_cmd − qdot) + I`

Per il ginocchio Kp_inner è29.2. In best80 la componente sopra10 Hz dell’errore di velocità ha RMS0.242 rad/s: il termine P corrispondente vale7.07 Nm RMS. La componente rapida della posizione servita è appena0.00206 rad, circa0.118°; può essere quasi invisibile nel grafico angolare e comunque generare oscillazioni sostanziali nella catena velocità–coppia.

| RMS coppia elastica knee dopo passa-alto | LEFT best72 | LEFT best80 | V26 best87 | Comando diretto LEFT storico |
|---|---:|---:|---:|---:|
| Filtro10 Hz [Nm] | 6.63 | 8.20 | 4.60 | 0.48 |
| Filtro20 Hz [Nm] | 4.11 | 3.78 | 3.28 | 0.15 |

Il miglioramento sopra20 Hz nasconde un peggioramento della componente tra circa10 e20 Hz: in best80 il massimo della PSD filtrata a10 Hz è circa13.18 Hz. Non basta una sola metrica HF per affermare che il chattering sia migliorato. Questi filtri hanno transizioni graduali; la differenza tra RMS non è una separazione ideale esatta in bande.

Il comando di coppia ha già RMS rapido8.03 Nm sopra10 Hz, vicino agli8.20 Nm della molla. Sopra20 Hz la correlazione tau_ref–termine P è0.964. Il problema è quindi già presente a monte della coppia elastica; il loop motore ne influenza la risposta ma non è sufficiente attribuirgli l’origine. Non è dimostrato se la perturbazione iniziale derivi principalmente dagli endpoint, dal contatto o dal feedback: la retroazione li accoppia.

Il comando diretto LEFT storico produce molto meno chattering con la stessa famiglia di controllori. Non è un sostituto validato della policy: ha ritardo, penetrazione massima22.79 mm e una diversa traiettoria. Dimostra però che questa catena non produce inevitabilmente oscillazioni grandi per qualunque riferimento LEFT.

## 4. Perché la reward lascia spazio a questa soluzione

In best80 lo score cinematico è composto da80% reference servita–target e20% stato fisico–target. Non viene richiesto di riprodurre una curva fisiologica di coppia o potenza. Le penalità fisiche esistono, ma hanno scopi e scale differenti.

Bilancio verificato sui500 campioni:

| Contributo | Punti nell’episodio |
|---|---:|
| Score imitativo positivo, servito+fisico | 385.77 |
| Costo command_rate | 75.58 |
| Costo effort normalizzato del comando SEA | 0.258 |
| Torque error SEA | 0.00162 |
| Velocità+accelerazione+potenza motore | 0.00438 |
| Momento GRF sfavorevole | 4.69 |
| Penetrazione | 0 |
| OOB | 0.175 |
| Ritorno ricostruito | 305.06 |

I pesi espliciti `sea_tau_spring_rate_weight` e `sea_tau_spring_effort_weight` sono0. La loss torque_error SEA confronta coppia richiesta ed erogata, non la plausibilità fisiologica: una coppia oscillante eseguita fedelmente può avere poco costo. I termini motore sono normalizzati rispetto a scale ampie (velocità100 rad/s, accelerazione50000 rad/s², potenza50000 W nei default effettivi usati), da cui il contributo minuscolo osservato.

Non sarebbe corretto dire che la reward ignora del tutto la regolarità: command_rate esiste e include jerk e variazione u. Tuttavia70.99 dei suoi75.58 punti, circa94%, provengono da segment_delta, cioè dalla distanza tra endpoint richiesto e stato iniziale della reference. Solo0.435 punti provengono dalla componente variazione u,3.21 dal jerk. Il costo è dunque un proxy misto, dominato dalla geometria del comando e non dalla qualità della coppia sotto carico. Il peso aggiuntivo dedicato u_rate è zero, ma la componente resta presente dentro command_rate.

Questi totali spiegano quali grandezze hanno incidenza numerica, non sono gradienti PPO né dimostrano da soli quale peso sarebbe ottimale. Non dimostrano neppure che l’attuale policy sia l’ottimo della reward: la prova statica mostra almeno una direzione locale favorevole a cinematica e contatto.

## 5. Implicazioni operative

Una prosecuzione da best80 fino100, mantenendo questa configurazione, è ragionevole come prova di maturazione; non è un intervento specifico sulle coppie. Best80 è preferibile a last82 come candidato per la qualità osservata della caviglia, ma resta da verificarne la robustezza a policy congelata. La prosecuzione non è stata avviata.

Per intervenire sulle coppie separerei due obiettivi: contatto/COP/momento sotto carico alla caviglia e oscillazioni della coppia al ginocchio. Aumentare ancora il solo peso di penetrazione non colpisce il burst attuale sotto20 mm. Un eventuale costo sul rateo della coppia knee dovrebbe essere specifico del giunto, normalizzato con una scala significativa e verificato rispetto a tracking e contatto; una penalità generica su tutta la coppia rischia di favorire scarico o perdita di supporto.

Non riproporrei automaticamente il jerk knee2000: il test storico di giugno ridusse il chattering ma peggiorò tracking e contatto. Neppure l’ex-novo garantisce la soluzione: il report del20 luglio attribuiva la regolarità all’intera catena, al prior e ai comandi, non soltanto al cambio di reward.

TODO: screening rumoroso best80; se si prosegue, selezione con metriche cinematiche, completamento e coppie, senza scegliere sul solo ritorno; separato confronto controllato sulla qualità della coppia knee e sul contatto ankle. Non avviare automaticamente questi esperimenti sulla base di questo report.

## Artefatti e verifiche

Codice di produzione, reward, controller, FSM e plugin non modificati. Nessun nuovo training o rollout dinamico. Eseguiti calcoli offline e una ricostruzione statica del contatto.

- [Script e metriche](<../../Trajectory Generator/unified_MLP_39/validation/best80_torque_audit_2026-09-24/>): `analyze.py`, `metrics.json`, `input_hashes.json`, `analysis.log`, `static_contact_probe.py`, relativo JSON e log.
- [Grafico diagnostico](../../plot/09_24_2026_best80_torque_diagnosis/best80_contact_controller.png).
- Verificata ricostruzione del ritorno dei sei rollout entro1e-5; coerenza della scomposizione command_rate entro1e-8; somma P+I e tau_ref entro1e-6 Nm; uniformità temporale STO; riproduzione statica di forza, momento e COP entro le tolleranze documentate. Hash degli input salvati.
