# Guida di ricostruzione della pipeline ex-novo training-ready dal training imitativo V26 di agosto

**Data:** 2026-08-28  
**Perimetro:** AB06, detector gait-cycle V26, FSM attore v3, Morphology Corridor corrente  
**Destinatario:** un LLM o un operatore che debba ricostruire rapidamente la pipeline senza ripetere gli errori storici  
**Stato finale documentato:** `TRAINING_READY_ATTESTED`, 18/18 gate, promozione `TRAINING_INPUT_ONLY`  
**Training PPO/ex-novo successivo:** **non lanciato e non autorizzato**

---

## 0. Risultato finale e distinzione che non deve andare persa

La pipeline conclusa parte esclusivamente dal migliore attore del training imitativo V26 di agosto e produce:

1. un **attore 35D behaviorally qualified**, J19A;
2. un **checkpoint completo actor + critic**, J20, ricaricabile dal percorso RLlib di produzione;
3. un'attestazione separata J21 che dichiara quel checkpoint `training_ready: true` soltanto come input di un futuro pilot conservativo.

Gli artefatti canonici sono:

| ruolo | artefatto canonico | identità |
|---|---|---|
| parent imitativo agosto | `Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best/module_state.pkl` | SHA-256 `0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd` |
| attore finale 35D | `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/rl_module/module_state.pkl` | SHA-256 `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530`; actor digest `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| checkpoint da usare come input di training | `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_critic_warmup_v26c_2026-08-27_r1/checkpoint_last` | 24 file; `module_state.pkl` SHA-256 `57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168`; `learner/state.pkl` SHA-256 `51d97ef016aea2086201b55db807f3273b76784bbafdea6cc9e5c7fea90294be` |
| attestazione finale | `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j21_runs/j21_training_ready_attestation_v26c_2026-08-28_r1/` | result SHA-256 `bc5d46522e08777d3de5ec9f24cd058ec282bbde32dc1653857cb45af264d146` |

Il dettaglio conclusivo è nel report [V26C J21 — attestazione finale training-ready](2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md).

### 0.1 Il vecchio B0820 non è la baseline finale

Il pacchetto B0820 del 20 agosto era stato dichiarato “training-ready” in senso **meccanico/runtime**: manifest coerente, critic warm-up eseguito, checkpoint caricabile e runtime capace di partire. Il successivo training ex-novo di 50 iterazioni ha però dimostrato che l'attore era già comportamentalmente collassato: traiettorie compresse, caviglia quasi sempre positiva, `phase_timeout_stance` e `morphology_causal_contract_failure` persistenti. Quella readiness non certificava forma cinematica, robustezza multistart o comportamento closed-loop.

Quindi:

- **non** usare `B0820_H0`, `B0820_V2_BEST`, `B0820_V3_BEST` o `B0820_V3_LAST` come parent;
- **non** interpretare il report [Risultato training-ready ex-novo B0820](2026-08-20_risultato_training_ready_exnovo_B0820.md) come certificazione scientifica;
- usare la diagnosi successiva in [Fase 0 — gate finale del recupero AB06](2026-08-23_fase0_gate_finale_recupero_ab06.md), che classifica B0820 come **FAIL comportamentale**;
- usare J21 come unica attestazione training-ready corrente.

### 0.2 Luglio è informativo, non operativo

La pipeline di luglio è stata usata per recuperare **metodo, ordine degli stadi, composizione dei dataset e iperparametri**, non pesi, dataset, trace o attori. Nessun artefatto luglio è parent operativo della rete corrente.

È inoltre importante correggere una semplificazione storica: non esiste un run ex-novo datato 15/07. Il 15 luglio fu eseguito un pilot PPO da 50 update partendo dal checkpoint Markov-35 del 13/07; il pilot riuscì tecnicamente ma non produsse checkpoint promuovibili. Ciò che fu realmente utile era l'H0 Markov-35 già robusto prima del pilot. La ricostruzione è documentata in [Architetture actor, adapter e tentativi di rete larga](2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md) e nell'audit [V26C J0 — replica July-faithful su parent agosto](2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

---

## 1. Mappa della lineage

La lineage operativa dell'attore finale è:

```text
August V26 imitation, best iter 87, actor 39D
    |
    +--> J1  teacher prescritto, 500 coppie nominali
    |
    +--> J2  derivazione e fit BASE dello stesso actor nativo 35D
             |
             +--> J3/J5/J6  rollout nominale e probe stocastici
             |       |
             |       +--> J7  dataset Markov nominal + recovery
             |               |
             |               +--> J8  recovery fit 35D
             |                       |
             |                       +--> J9R1  matrice A-F: 5/6; B fallisce
             |                               |
             |                               +--> J10R1 teacher multistart B/C
             |                                       |
             |                                       +--> J18 update vincolato su J8
             |                                               |
             |                                               +--> J19A riproduzione c13
             |                                                       |
             |                                                       +--> J19B A-F: 6/6
             |                                                       +--> J19C G-I: 3/3 held-out
             |                                                               |
             |                                                               +--> J20 critic-only warm-up
             |                                                                       |
             |                                                                       +--> restore audit R3
             |                                                                               |
             |                                                                               +--> J21 attestation
             |
             +--> ramo diagnostico J11 -> J12 -> J14 -> J15R1 -> J16 -> J17
                    (ha spiegato il problema, ma NON è parent dell'attore finale)
```

Il passaggio J18 parte dai **byte reali di J8**, non da J11 o J15R1. J11–J17 sono indispensabili per capire perché il primo DAgger falliva e come progettare la correzione, ma non sono antenati dei pesi finali.

---

## 2. Vocabolario operativo

- **Teacher prescritto:** il generatore di azioni cinematiche di riferimento usato per etichettare uno stato. Non è un checkpoint di luglio.
- **Student:** l'attore 35D che deve funzionare in modalità ex-novo senza le quattro feature privilegiate del teacher imitativo.
- **Fit BASE:** adattamento supervisionato nominale su 500 coppie teacher, con le feature di memoria controller ancora neutralizzate.
- **Markov/recovery fit:** secondo adattamento che riattiva le dieci feature 25–34 e usa stati visitati dallo student sotto rumore o start diversi.
- **DAgger:** raccolta di stati visitati dallo student, etichettati dal teacher allo stesso step. È utile perché lo student in closed loop visita stati diversi da quelli del teacher.
- **Covariate shift closed-loop:** una piccola differenza di azione cambia lo stato successivo; la rete entra quindi in regioni non presenti nel dataset supervisionato, dove l'errore cresce e si auto-amplifica.
- **Mismatch FSM discreto:** divergenza fra student e teacher su contatto/eventi/stato FSM. Troncando il dataset appena compare il primo mismatch si eliminano proprio gli stati che insegnano il recupero.
- **Held-out:** semi stocastici sigillati e non usati per il fit o per scegliere il candidato.
- **Training-ready:** il checkpoint si può usare come **input** di un futuro pilot preregistrato. Non significa deployable, non autorizza PPO e non sigilla comando o iperparametri del pilot.

---

## 3. Invarianti architetturali e scientifiche

### 3.1 Architettura

La topologia nascosta non è stata allargata:

```text
35 input -> 256 tanh -> 256 tanh -> 4 output
```

Le quattro uscite sono due medie d'azione e due log-standard-deviation. Il modulo actor-only ha 10 chiavi contando gli alias encoder; dopo il warm-up J20 il modulo completo ha 16 chiavi, perché si aggiungono le 6 chiavi del critic.

Non esiste un adapter appreso. Il passaggio 39D→35D avviene per nome delle feature e compensazione del bias. Non esiste neppure un actor 25D intermedio nella lineage finale. Nello stadio BASE si usa già l'attore 35D, ma si azzerano le colonne 0, 1 e 25–34: sono quindi 23 le colonne effettivamente attive in quella fase. Nello stadio Markov le colonne controller 25–34 vengono riattivate nello **stesso** attore 35D; le colonne clock 0–1 restano a zero.

I tentativi W512/W1024 di agosto superarono alcuni gate offline ma fallirono closed-loop per penetrazione. Per questa ragione l'aumento di capacità non è stato usato come soluzione; vedere [Architetture actor e tentativi di rete larga](2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md).

### 3.2 Feature 35D

L'ordine è vincolante:

```text
 0 gait_phase_sin
 1 gait_phase_cos
 2 pros_knee_angle
 3 pros_knee_angle_vel
 4 pros_ankle_angle
 5 pros_ankle_angle_vel
 6 SEA_Knee_motor_angle
 7 SEA_Knee_motor_speed
 8 SEA_Ankle_motor_angle
 9 SEA_Ankle_motor_speed
10 online_left_normal_grf_bw
11 online_left_in_contact
12 online_left_heel_strike
13 online_left_toe_off
14 online_left_gait_phase_sin
15 online_left_gait_phase_cos
16 online_left_cycle_duration_s
17 phase_fsm_wait_hs
18 phase_fsm_stance_after_hs
19 phase_fsm_swing_after_to
20 phase_expected_hs
21 phase_expected_to
22 phase_stance_elapsed_norm
23 phase_swing_elapsed_norm
24 phase_cycle_progress_credit
25 pros_knee_angle_previous_endpoint
26 pros_knee_angle_served_ref
27 pros_knee_angle_served_ref_vel
28 pros_knee_angle_served_ref_accel
29 pros_knee_angle_sea_u
30 pros_ankle_angle_previous_endpoint
31 pros_ankle_angle_served_ref
32 pros_ankle_angle_served_ref_vel
33 pros_ankle_angle_served_ref_accel
34 pros_ankle_angle_sea_u
```

L'assenza delle feature controlaterali/privilegiate in ex-novo è **voluta**. Le quattro feature rimosse dal parent 39D sono:

```text
healthy_knee_angle_imitation_target
healthy_knee_angle_imitation_target_vel
healthy_ankle_angle_imitation_target
healthy_ankle_angle_imitation_target_vel
```

### 3.3 Runtime e gate

Il runtime deve essere costruito con il config completo usato da `rollout_eval.run` / `train_ppo_mlp.build_config`, includendo:

- detector binario V26;
- FSM attore `v3`;
- `event_contract_id` e `binary_phase_event_contract_id` corretti;
- debounce/dwell e policy degli eventi invalidi;
- Morphology Corridor e contratto causale correnti;
- reward, timeout, start e soglie di penetrazione correnti.

Il contratto di penetrazione finale è:

| soglia | semantica |
|---|---|
| `> 20 mm` | soft, diagnostica; non blocca |
| `>= 25 mm` | diagnostica comparativa luglio; non blocca |
| `> 28 mm` | hard binding; blocca |

Esattamente 28 mm passa. La semantica delle disuguaglianze va letta letteralmente. Analogamente, per il gate `ankle_min <= -0.03 rad`, un valore `-0.0099` **fallisce** perché `-0.0099 > -0.03`.

I gate comportamentali closed-loop richiedono, salvo diversa preregistrazione esplicita:

- 500 step e `episode_time_limit`;
- almeno 2 cicli validi;
- `phase_timeout_stance = 0`;
- `phase_timeout_swing = 0`;
- `morphology_causal_contract_failure = 0`;
- `resync_count = 0`;
- `hs_cancelled_count = 0`;
- nessun clipping d'azione;
- qualità cinematica: caviglia con fase negativa, ROM caviglia almeno 0,30 rad, ROM ginocchio almeno 0,60 rad, ginocchio strettamente flesso e bounds rispettati;
- nessuna violazione della soglia hard di penetrazione.

### 3.4 Sigma

`sigma = 0.005` non è stata assunta per tradizione. J6 l'ha verificata su tre probe stocastici e J19B/J19C l'hanno nuovamente verificata su sei semi. Il valore finale effettivo è `0.004999999670722372`, con testa log-std state-independent e congelata. Un futuro pilot deve comunque rivalutare sigma: J21 non la rende automaticamente corretta per qualunque training successivo.

---

## 4. Fase A — ottenere il parent imitativo V26 di agosto

### 4.1 Primo tentativo del 17–18 agosto

Fu lanciato un training imitativo da 100 iterazioni con ricetta di giugno e detector V26. Completò 100/100, ma gli episodi duravano circa 250 step invece di circa 500: il timeout hard di swing a 1,3 s tagliava quasi tutti gli episodi.

In parallelo fu scoperto un bug di schema: il probe in `train_ppo_mlp.py` dimensionava l'attore passando da `build_env_config` senza eseguire `make_cmc_env`; vedeva 39 input mentre il runtime ne esponeva 43. Lo stesso difetto storico aveva prodotto 31 contro 35 a giugno. Per ottenere un confronto isometrico con giugno il bug non fu corretto dentro quella lineage; fu trattato come debito di migrazione schema.

Riferimento: [Report approfondito dal training imitativo V26 alla FSM v3](2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md).

### 4.2 Il falso smoke verde e la crash storm

Il timeout swing fu portato a 2,6 s e la policy degli eventi invalidi inizialmente impostata a `reject_continue`. Uno smoke apparve migliore, ma i contatori “zero timeout/zero invalidi” erano congelati perché alcuni episodi morivano insieme ai worker: l'assenza di telemetria non era un vero zero.

La causa della crash storm era doppia:

1. `reject_continue` avanzava il cursore del sensore senza committare il candidato FSM; allo step seguente il tempo precedente risultava off-grid;
2. l'accumulo floating-point al confine dell'orizzonte, con tolleranza `1e-12`, generava un microstep fuori griglia.

Sintomi misurati: 1044 errori off-grid, 639 log worker e circa 30 morti worker per iterazione, con iterazioni da circa 36 minuti.

Correzioni:

- policy adapter `invalid_event_mode = "drop"`;
- un solo percorso di commit del candidato FSM;
- criterio d'orizzonte basato su metà finestra di policy invece del confronto quasi-esatto;
- telemetria verificata sul processo vivo, non dedotta dalla sola assenza di record.

Dopo la correzione: 0 off-grid su 72 iterazioni e circa 7,7 minuti per iterazione.

### 4.3 La continuazione sporca non fu promossa

Una continuazione fino a 200 iterazioni plateaued a circa 65% del return e 84% della lunghezza di giugno. Fu scartata perché la storia conteneva timeout, worker relaunch e cambi di semantica. Non si è continuato a “riparare” la stessa run: si è eseguito un fresh run pulito.

### 4.4 Audit di equivalenza e fresh run pulito

L'audit verificò:

- reward effettivo equivalente a giugno;
- PPO: batch target 4096, minibatch 512, 10 epoche, LR/clip/gamma/lambda/KL invariati;
- 13 EnvRunner, start fisso 1,0 s, orizzonte 5 s;
- `grf_tangent_v2` e `grf_correct_magnitude` byte-identici nonostante il rename;
- soglie riportate alla decisione utente 20/28 mm;
- compensazione V26: timeout swing 2,6 s + drop degli eventi invalidi.

Il fresh run canonico è:

```text
Trajectory Generator/runs/training/
  MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/
```

Risultati:

- 100/100 iterazioni in 14,4 ore;
- 0 errori off-grid;
- migliore iterazione: 87;
- medie ultime 20: return 257,4 contro 268,6 di giugno; lunghezza 494 contro 476;
- 198 terminazioni per penetrazione contro 422 di giugno;
- 113 timeout swing, costo dichiarato della qualifica V26;
- parent finale `rl_module_best/module_state.pkl`, SHA `0ba56eb7…`.

La configurazione risolta è il record autorevole per ricostruire la chiamata: `training_cfg.resolved.yaml` dentro la run. Il pattern storico di lancio era `train_ppo_mlp.py --iterations 100 --output-dir <nuova-run>`, ma non va ricostruito a memoria: un nuovo LLM deve derivare ogni override dal file risolto e dal report [Audit equivalenza giugno per fresh run V26](2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). Il verdetto finale è in [Fresh run june-equivalent nativo V26](2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md).

### 4.5 Perché “4096 environment” era un'interpretazione errata

`4096` era il target del train batch, non il numero di environment. Nel training B0820 c'erano 12 EnvRunner remoti; nel fresh imitativo V26 13. I contatori `num_env_steps_sampled_lifetime` crescono perché sono cumulativi. Un overshoot può anche apparire se un worker muore a metà round e RLlib rilancia l'intero round: non significa che stiano comparendo nuovi environment permanenti. La spiegazione completa è in [Fase 0 — gate finale AB06, §5](2026-08-23_fase0_gate_finale_recupero_ab06.md).

---

## 5. Fase B — perché B0820 fallì nonostante il runtime corretto

La prima catena del 20–21 agosto fece:

```text
V26 imitation 39D -> transplant diretto 35D -> critic warm-up -> PPO ex-novo 50 iter
```

Il checkpoint partiva e FSM/corridoio erano progressivamente corretti, ma il transplant aveva eliminato quattro feature privilegiate senza eseguire un vero refit sul dominio 35D. Le dieci feature controller non ricostruivano da sole la funzione mancante e il clock azzerato contribuiva a un bootstrap iniziale povero. Inoltre B0820 aveva ereditato una testa log-std state-dependent con sigma dell'ordine di 0,5, mentre la lineage robusta di luglio usava sigma costante 0,005.

Il collasso precedeva PPO:

- H0, best e last erano quasi sovrapposti;
- sotto runtime v3 i checkpoint B0820 producevano zero cicli validi ai tre start deterministici;
- caviglia quasi sempre positiva e ROM compresse;
- il KL molto piccolo mostrava che i 50 update non stavano ricostruendo la forma persa.

FSM v3 risolse il lock di stance tramite resync, dwell di 80 ms, cancellazione dei bounced heel-strike e riallineamento dei latch. Il Morphology Corridor fu corretto riarmando il ledger sulle transizioni di repair e portando la tolleranza da `1e-12` a `1e-9`. Queste correzioni eliminarono errori runtime, ma non potevano ricostruire la funzione dell'attore.

Lezioni:

1. FSM e corridoio sono infrastruttura e guardie, non teacher;
2. “arriva a 500 step” non basta: servono forma, cicli, multistart e test stocastici;
3. il critic warm-up deve essere ultimo, dopo la qualifica dell'attore;
4. la perdita delle feature privilegiate richiede Teacher–Student/DAgger sul dominio target.

Riferimenti: [Report approfondito V26→FSM v3](2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md), [Validazione FSM v3 e tolleranza corridoio](2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md), [Fase 0 intermedia](2026-08-22_fase0_freeze_inventario_gate_intermedio.md), [Fase 0 finale](2026-08-23_fase0_gate_finale_recupero_ab06.md).

---

## 6. Fase C — tentativi V26B che hanno preceduto la replica July-faithful

Questa fase produsse molte evidenze utili, ma nessun parent finale. Va conosciuta per evitare di ripetere le stesse deviazioni.

| tentativo | cosa fece | esito/problema | lezione |
|---|---|---|---|
| V1 transplant | 39D→35D per nome, bias compensation, clock zero | transplant e anchor collection tecnicamente PASS | il trapianto preserva byte e schema, non garantisce closed-loop |
| R0 multi-role | un unico dataset con ruoli/target diversi | gate offline FAIL; conflitti misurabili | non mescolare label semanticamente diverse senza disgiunzione e pesi espliciti |
| R0A nominal IK BC | behavior cloning nominale | offline PASS, rollout 493/500 e divergenza | quasi-completamento non equivale a qualifica |
| DAgger R1/R2 | stati student + teacher label | offline PASS; rollout anche 242/500 o 197/500 | un buon RMSE offline non certifica la dinamica closed-loop |
| Rev4b/c/d/e | repliche e bilanciamenti “July-like” | il migliore Rev4e arrivò a 372 step e 1 ciclo, ma penetrò 28,22 mm | imitare il protocollo senza rispettarne esattamente supporto e ordine non basta |
| S0D | distillazione V26→35D | 500/500 nominale, ma ankle ancora prevalentemente positiva; campo B3 nullo | un rollout nominale non copre forma e multistart |
| S1 e varianti | fit supervisionati, pre-gate, 5-fold, Pareto | molte PASS offline, nessun candidato robusto completo | non elevare un pre-gate a selection gate |
| 25D separato | proposta di actor intermedio senza memoria controller | bloccato e poi superseded prima dell'esecuzione | usare un solo actor 35D con maschera, poi riattivare colonne |
| W512/W1024 | maggiore capacità | offline verde, closed-loop penetrazione | la capacità non era il blocker |
| B0/B1, LOTO/LOCO, B1R1/B1R2 | protocolli esplorativi di split/leave-out | NO-GO | restano TODO futuri, non baseline corrente |

Report storici principali:

- [V26B V1 transplant 35D](2026-08-24_v26b_v1_transplant_35d_student.md)
- [V26B R0 gate offline fallito](2026-08-24_v26b_v2_r0_offline_gate_fail.md)
- [Diagnostica fattibilità R0](2026-08-24_v26b_r0_feasibility_diagnostic.md)
- [R0A nominal IK BC](2026-08-24_v26b_r0a_nominal_ik_bc_pass.md)
- [Diagnosi rollout R0A](2026-08-24_v26b_r0a_rollout_ended_early_diagnosis.md)
- [DAgger round 1 offline PASS](2026-08-24_v26b_dagger_round1_offline_pass.md)
- [Diagnosi rollout R1](2026-08-24_v26b_r1_rollout_ended_early_diagnosis.md)
- [Rev4b July DAgger](2026-08-24_v26b_rev4b_july_dagger_s1a.md)
- [Rev4e replay R2](2026-08-24_v26b_rev4e_r2_replay.md)
- [S0D rollout](2026-08-24_v26b_s0d_rollout_pass.md) e [supplemento semantico B3](2026-08-24_v26b_s0d_rollout_b3_semantica_supplemento.md)
- [Rev3n: mediana 5-fold](2026-08-24_v26b_s1_pregate_rev3n_mediana_5fold.md): “Rev3n” era solo una revisione del pre-gate kNN5, non un attore o una fase della pipeline finale
- [S1B A2 rollout eligible](2026-08-24_v26b_s1b_a2_rollout_eligible.md)
- [L20 rollout eligible](2026-08-24_v26b_l20_rollout_eligible.md)
- [Protocollo 25D](2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md)
- [Ramo 35D masked](2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md)
- [B0/B1 NO-GO](2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md)
- [B1R1/LOCO NO-GO](2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md)
- [B1R2B ipotesi falsificata](2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md)

Il punto di svolta fu smettere di inventare nuovi rami e ricostruire esattamente il metodo di luglio, mantenendo però il parent V26 agosto e il runtime corrente.

---

## 7. Fase D — V26C J0–J8: BASE 35D e primo dataset Markov

La radice degli strumenti e degli artefatti V26C è:

```text
Trajectory Generator/baseline_MLP/validation/
  v26c_july_replica_2026-08-26/
```

### J0 — audit e preregistrazione

J0 ricostruì due stadi distinti di luglio:

1. **BASE:** 500 coppie, intera mean-network addestrabile, split random 80/20, batch 64, LR `3e-4`, patience 60, clip 1, logstd weight 0,1, anchor `1e-5`, massimo 400 epoche;
2. **MARKOV:** 16000 self-anchor + 712 recovery + 8000 multistart = 24712 righe, intera mean-network, batch 128, LR `5e-5`, anchor 0,01, logstd congelata, massimo 400 epoche.

J0 scoprì anche un blocker nel builder storico `target_domain_imitation.build_target_env_config`: ometteva 12 chiavi FSM v3 e tutto il corridoio morfologico. Per questo fu scritto un collector additivo che costruisce il **full env config** e fallisce prima di aprire l'ambiente se manca un solo campo.

Report: [V26C J0 — audit e preregistrazione](2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

### J1 — raccolta del teacher nominale

Input unico: parent imitativo agosto `0ba56eb7…`; il teacher esegue 500 step nominali, senza rumore, con V26 + FSM v3 + corridoio.

Risultato:

- 500 step, 2 cicli, 3 HS e 3 TO;
- tutti i contatori FSM/morphology a zero;
- penetrazione massima 22,9438 mm;
- 97/500 step sopra 20 mm;
- dataset `j1_runs/j1_nominal_v26c_2026-08-26_r1/teacher_dataset.npz`.

Il receipt storico dice `FAIL` perché al momento 20 mm era stato interpretato come vincolante. La decisione successiva dell'utente ha fissato 20 mm soft, 25 mm diagnostica luglio e 28 mm hard. Il receipt non è stato riscritto: con il contratto finale J1 è accettabile perché 22,94 < 28 mm.

### J2 — derivazione 39D→35D e fit BASE

J2 deriva direttamente dal parent V26 agosto, non da V1/B0 e non da luglio:

1. mapping per nome, order-preserving;
2. rimozione degli indici 2–5, le quattro feature privilegiate;
3. compensazione float64 del primo bias: `b1 += W1_39[:, dropped] @ mean(dropped_features)`;
4. testa log-std resa costante a sigma 0,005;
5. hard-zero delle colonne 0, 1, 25–34;
6. fit dell'intera mean actor sulle 500 coppie J1; critic escluso.

Iperparametri BASE:

| parametro | valore |
|---|---:|
| seed | 123 |
| train/validation | 400/100, split random |
| batch | 64 |
| LR | `3e-4` |
| epoche max | 400 |
| patience | 60 |
| anchor | `1e-5` |
| logstd weight | 0,1 |

Risultati:

- RMSE globale 0,835215 → 0,013859;
- validation RMSE 0,014867;
- best epoch 378/400;
- output `module_state.pkl` SHA `0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130`;
- nessun critic.

### J3 — prima validazione closed-loop del BASE

Risultato nominale deterministico:

- 500 step, 2 cicli, contatori tutti zero;
- ankle min `-0.1071`, ROM 0,5165 rad;
- knee ROM 0,8486 rad, sempre flesso;
- penetrazione max 27,0497 mm, 106/500 step sopra 20, nessuno sopra 28.

Anche J3 conserva un `FAIL` storico dovuto alla vecchia interpretazione binding dei 20 mm. Sotto il contratto finale è un soft fail accettato. Si noti che `-0.1071 <= -0.03`: il gate di fase negativa passa.

### J4 e J5 — il primo recovery-only, troppo corto

J4 applicò fedelmente l'operatore storico di luglio “tronca prima del primo mismatch discreto”. Il mismatch compariva allo step 13, quindi furono trattenuti solo 12 step recovery. Il dataset era:

- 500 stati nominali ×32 = 16000 self-anchor;
- 12 recovery ×2 = 24;
- totale 16024;
- multistart ancora omesso.

Il fit passò offline, ma il blocco recovery era appena lo 0,15% dell'aggregato e non includeva la regione dopo il mismatch.

J5 R1 fallì tecnicamente perché `-W error` trasformava warning legittimi di SciPy SLSQP in eccezioni. J5 R2 corresse la policy dei warning senza cambiare scienza. Risultato R2: 500 step, 2 cicli, contatori zero, max penetrazione 26,9133 mm. I campioni sopra 20 rimasero **106/500**, uguali a J3; migliorò leggermente il massimo, non la frequenza del soft exceedance.

### J6 — sigma verificata e recovery prefixes

J6 eseguì tre rollout stocastici con seed 123, 124 e 125 sul J2 BASE:

| seed | step/cicli | max penetrazione | prefisso prima del mismatch |
|---:|---:|---:|---:|
| 123 | 500 / 2 | 26,678 mm | 429 |
| 124 | 500 / 2 | 26,954 mm | 273 |
| 125 | 500 / 2 | 27,291 mm | 11 |

Tutti i contatori target erano zero e il rumore realizzato era coerente con 0,005. I prefissi 429+273+11 = 713 dimostrarono che il vecchio J4 a 12 righe non rappresentava la reale copertura stocastica.

### J7 — materializzazione del dataset Markov

J7 costruì un dataset immutabile da 16713 righe:

| blocco | righe | stato | label |
|---|---:|---|---|
| nominale | 16000 | 500 stati del rollout J3 ×32 | mean di J2 sugli stessi stati |
| recovery | 713 | prefissi student-visited dei tre J6 | teacher J1 allo stesso step |

Proprietà:

- ratio recovery/nominal `713/16000 = 0.0445625`, praticamente uguale a luglio `0.0445`;
- clock 0–1 proiettato a zero;
- colonne controller 25–34 reali e non degeneri;
- nessun multistart ancora;
- dataset SHA `bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27`.

Il teacher fornisce label e allineamento temporale; non si usano i suoi stati al posto degli stati visitati dallo student.

### J8 — recovery fit 35D

J8 riparte **fresh da J2**, usa J7 e riattiva le colonne controller 25–34 nello stesso actor 35D.

Iperparametri MARKOV:

- batch 128;
- LR `5e-5`;
- anchor 0,01;
- logstd congelata;
- intera mean-network addestrabile;
- massimo 400 epoche, early stop 60.

Risultato:

- best epoch 107; stop all'epoca 167;
- recovery RMSE 0,088361 → 0,083652;
- shift nominale RMS 0,005033, max 0,022096;
- output SHA `9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82`.

Intoppo di provenance: `actor_feature_manifest.json` di J8 fu copiato byte-identico da J2 e quindi contiene metadati stantii. Il modulo reale J8 fu successivamente verificato dalle trace entro `1.47e-7`; da J18 in avanti la sua identità è sempre derivata dai byte reali del modulo, non dal sidecar.

---

## 8. Fase E — J9–J17: robustezza multistart e diagnosi del DAgger errato

### J9 e J9R1 — 5/6, blocker sulla cella B

J9 fallì tecnicamente dopo la prima cella perché il builder lasciava `output_dir=None`, disabilitando `record_outputs/save`; il runner pretendeva poi 19 file `sim_outputs` inesistenti. J9R1 corresse solo il routing degli output e usò stadio/leaf separati.

J9R1:

- A, C, D, E, F: PASS;
- B, start `-0.20 s`: 500 step ma 0 cicli, FSM in `WAIT_HS` per 500/500;
- 6/6 telemetria valida, 5/6 comportamento;
- penetrazione B a zero era un PASS **vacuo**: il piede non entrava in contatto;
- J7 conteneva zero righe `WAIT_HS` e nessuno start non nominale.

Questo era il blocker reale: il dataset non copriva lo start B. Report: [Qualifica closed-loop J9R1](2026-08-27_qualifica_closed_loop_j9r1.md).

### J10 e J10R1 — teacher multistart

Il primo pacchetto J10 fu respinto prima dell'esecuzione per quattro difetti di contratto:

- indice `WAIT_HS` ricavato dal manifest 39D invece che dal runtime 35D;
- “sim_outputs non vuoti” invece di esattamente 19 file regolari;
- hash e verifica post-commit insufficienti;
- born-invalid/no-clobber non completamente provati.

J10R1 corresse questi aspetti e raccolse due teacher trajectory da 500 step agli start B `-0.20` e C `+0.20`. Entrambe passarono; le 20 righe `WAIT_HS` uniche provenivano tutte da B. Report: [Readiness J10R1](2026-08-27_v26c_j10r1_multistart_teacher_readiness.md) e [Collection J10R1](2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

### J11 e J12 — multistart completo, ma regressione stocastica

J11 fu un **fratello fresh di J8**, non un update di J8: ripartì da J2 e addestrò su 24713 righe:

```text
16000 nominal + 713 recovery + 4000 B + 4000 C
```

Le 500 righe B e C furono ripetute ×8, seguendo la composizione storica. Il fit passò 16/16 offline, migliorando B da RMSE 0,063907 a 0,008264 e C da 0,020248 a 0,007684; best epoch 397/400.

J12 mostrò perché il closed-loop resta il gate primario:

- B fu corretto: 3 cicli e PASS;
- A, C, D passarono;
- E completò 500 step ma solo 1 ciclo;
- F terminò allo step 354 per `phase_timeout:swing`, con un HS cancellato;
- risultato netto 4/6, peggiore del 5/6 di J8;
- tutte le penetrazioni erano sotto 23,2 mm: la penetrazione non era la causa.

Report: [J11 multistart fit](2026-08-27_v26c_j11_multistart_fit_execution.md) e [J12 closed-loop](2026-08-27_v26c_j12_closed_loop_qualification_execution.md).

### J13 — diagnosi del supporto mancante

La diagnosi appaiata J8/J11 stabilì:

- la differenza nasceva già dall'azione al passo 1, non da rumore diverso;
- E/F visitavano valori di `phase_swing_elapsed_norm` oltre il massimo del training per il 42–55% degli step;
- in F un HS veniva cancellato e il clock swing non veniva resettato, portando al timeout;
- il dataset J7 si fermava al primo mismatch discreto e quindi scartava gli stati post-mismatch, cioè gli stati di recupero più utili.

Il DAgger corretto deve interrogare il teacher sugli stati post-mismatch visitati dallo student, non fermarsi prima. Report: [J13 — diagnosi e piano di recovery](2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

### J14 — dataset DAgger post-mismatch

J14 materializzò 854 righe dalle celle J12 E/F:

| origine | righe | pre-mismatch | post-mismatch |
|---|---:|---:|---:|
| E, seed 124 | 500 | 94 | 406 |
| F, seed 125 | 354 | 89 | 265 |
| totale | 854 | 183 | 671 |

Un troncamento in stile J7 avrebbe tenuto le 183 righe meno utili e scartato tutte le 671 correttive.

Intoppi tecnici:

- la preregistrazione includeva se stessa nel digest di root, rendendo il digest autoreferenziale;
- alcuni report citavano hash anteriori all'ultima correzione;
- una seconda invocazione deliberata fu respinta dal no-clobber, come test negativo.

La soluzione fu un record GO additivo con digest self-excluding, pin espliciti e preservazione dei record precedenti. Report: [J14 dataset post-mismatch](2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md).

### J15/J15R1 e J16 — il refit fresco catastrofico

J15 fallì tecnicamente prima del fit: il runner copiato da J11 chiamava `verify_warm_start_source`, ma l'alias non esisteva nel namespace J15. I test usavano un `fake_fit` e non esercitavano il percorso reale. Nessun attore fu scritto.

J15R1 aggiunse l'alias e test statici sui free-name. Ripartì fresh da J2 su:

```text
24713 righe J11 + 854 righe J14 = 25567
```

Offline sembrò riuscire:

- RMSE J14 0,3923 → 0,1816;
- post-mismatch 0,4363 → 0,1990;
- nominale da circa zero a 0,01554.

J16 rivelò il fallimento: 0/6. Cinque celle superarono 28 mm e terminarono per penetrazione; D completò 500 step ma con un solo ciclo.

Report: [J15 execution](2026-08-27_v26c_j15_fresh_refit_execution.md), [J15R1](2026-08-27_v26c_j15r1_fresh_refit_execution.md), [J16 requalification](2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md).

### J17 — causa causale della regressione

La causa non era FSM, sigma, scaling, loader, critic o contaminazione di lineage. Era il modo in cui il refit fresco pesava il dataset:

- le 854 righe J14 erano solo il 3,34% delle righe;
- ma contribuivano l'84,81% della massa iniziale di errore quadratico/gradiente;
- i 16000 self-anchor nominali avevano label uguali al parent, quindi gradiente iniziale quasi nullo;
- J14 era on-policy per J11, mentre J15R1 ripartiva fresh da J2: violazione della premessa DAgger on-policy;
- lo shift del backbone fu 2,73× quello di J11;
- il comando knee si indebolì, il ginocchio collassò sotto carico e la penetrazione seguì.

Quindi “più recovery data” senza mass-balance e preservation funzionale può essere peggiore di nessun recovery. Report: [J17 — diagnosi causale della regressione J16](2026-08-27_v26c_j17_diagnosi_causale_regressione_j16.md).

---

## 9. Fase F — J18/J19: correzione minima sul parent operativo J8

### 9.1 Principio della soluzione

Invece di un refit fresco dall'attore BASE, si aggiorna incrementalmente il migliore attore operativo J8 e si vincola esplicitamente la sua funzione sulle regioni che già superava.

J18 usa quattro blocchi globalmente disgiunti, con priorità semantica A > B > C > D:

| blocco | righe | ruolo |
|---|---:|---|
| A | 500 | intera traiettoria teacher J10-B, target principale |
| B | 14 | prefisso on-policy J8/J9-B prima del mismatch, label J10 same-step |
| C | 2497 | stati delle cinque celle J8 passanti A,C,D,E,F, label mean J8 |
| D | 1210 | supporto unico J7, label mean J8 |
| totale | 4221 | tutte osservazioni distinte |

La loss è:

```text
L(theta) = MSE_A + beta * MSE_B + lambda * MSE_C + lambda * MSE_D
```

Ogni MSE è mediato **dentro il proprio blocco** prima del peso. In questo modo il numero di righe non determina implicitamente la massa del gradiente. C e D sono anchor in output/action space, più informativi di un semplice anchor parametrico.

### 9.2 J18 — griglia vincolata e soft failure del primo gate

Griglia finita:

- `lambda ∈ {1, 3, 10, 30}`;
- `beta ∈ {1, 5}`;
- `lr ∈ {1e-5, 5e-5}`;
- 16 candidati, seed 123, 200 epoche, batch 128;
- intera mean-network; logstd congelata; critic escluso.

Il gate iniziale richiedeva max drift ≤ 0,005 su ogni singola uscita. Nessun candidato sopravvisse e J18 terminò `FAIL_CLOSED_NO_SURVIVOR`, senza salvare attori. L'analisi read-only mostrò che quel massimo single-sample era molto più severo del drift già tollerato dall'attore operativo J8 e non rappresentava la preservazione media.

Senza riaddestrare o rilanciare la griglia, l'eleggibilità fu ricalibrata sui ceiling empirici **bias e RMSE di J8 rispetto a J2**, mantenendo i max drift G1/G2 come diagnostici dichiarati. Il candidato 13 risultò l'unico eleggibile:

```text
lambda = 30
beta   = 1
lr     = 5e-5
best epoch = 191
optimizer steps = 6600
```

Questa ricalibrazione è una decisione storica registrata. In una futura pipeline nessun LLM deve cambiare autonomamente gate o criteri: una modifica architetturale o scientifica richiede approvazione dell'utente.

Report: [J18 update B-only](2026-08-27_v26c_j18_b_only_update_execution.md) e [Ricalibrazione eleggibilità J18](2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).

### 9.3 J19A — riproduzione singola del candidato 13

Poiché J18 fail-closed non aveva scritto l'attore, J19A riprodusse **una sola volta** il candidato 13 con configurazione congelata.

Risultati:

- sei metriche fresh identiche ai valori congelati, delta assoluto zero;
- 36/36 campi di protocollo/provenance identici;
- 11/11 criteri offline;
- `MSE_A = 0.000559701`;
- `MSE_B = 0.000306913`;
- `rmse_C = 0.00303838`;
- `rmse_D = 0.00264411`;
- actor 35D, nessun critic, logstd byte-identica a J8;
- modulo SHA `8153dc97…`, actor digest `d4a13ff7…`.

Il POST-485 mostrava drift alto, ma era la regione bersaglio della correzione e restò diagnostico; nessuna conclusione closed-loop fu tratta da J19A. Report: [J19A — riproduzione singola](2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).

### 9.4 J19B — qualifica A–F

J19B eseguì la matrice development:

- 6/6 comportamento e 6/6 telemetria;
- tutte le celle 500 step;
- B 3 cicli, le altre 2;
- zero timeout stance/swing;
- zero morphology failure, resync, HS cancellation e clipping;
- knee ROM circa 0,83 rad;
- ankle ROM circa 0,52 rad, minimo tra `-0.1184` e `-0.1109` rad;
- nessuna violazione hard 28 mm;
- F max 25,615 mm, diagnostica luglio ma non binding.

Attore invariato byte per byte durante la qualifica. Report: [J19B — qualifica closed-loop A–F](2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).

### 9.5 J19C — held-out G–I

Semi sigillati 126, 127, 128:

- 3/3 e 42/42 gate;
- 500 step e 2 cicli per cella;
- tutti i contatori target a zero;
- ROM coerenti con J19B;
- H max 25,684 mm, 4/500 campioni nella diagnostica 25 mm;
- nessun campione sopra 28 mm;
- rumore realizzato 0,00487–0,00531.

Report: [J19C — held-out G/H/I](2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).

---

## 10. Fase G — J20: aggiungere e verificare il critic senza muovere l'attore

### 10.1 Perché serve

J19A è un export actor-only a 10 chiavi. Un training RLlib richiede il modulo asimmetrico completo actor35/critic84. Caricare l'actor-only in un modulo trainabile crea un critic fresco; occorre dimostrare che:

- il critic riceva gradiente;
- attore e logstd restino esatti;
- l'optimizer contenga stato solo per il critic;
- il checkpoint si ripristini attraverso il percorso reale di produzione.

### 10.2 K1 e K1R1 — il tensore di gradiente nullo

K1 ottenne 28/29: `vf_encoder.0.weight` aveva gradiente tutto zero. Non era un critic morto. Il probe sintetico usava un batch di input interamente zero; per un layer lineare `dL/dW` è proporzionale all'input, quindi il gradiente dei pesi deve essere zero matematicamente.

K1R1 sostituì lo stimolo con una matrice deterministica non degenere 8×84. Risultato:

- 12/12 gate;
- `vf_encoder.0.weight` gradient norm 15,325;
- tutte e 6 le chiavi critic con gradiente positivo;
- nessuna chiave actor con gradiente.

Report: [J20 K1 probe](2026-08-27_v26c_j20_k1_probe.md) e [J20 K1R1](2026-08-27_v26c_j20_k1r1_execution.md).

### 10.3 Warm-up critic-only

Fu eseguita una sola iterazione logica:

- 4096 step nominali;
- attore e logstd congelati e auditati su 13 worker remoti + locale;
- critic fresco `5ce4ef41…` → critic addestrato `2fa9c124…`;
- `vf_loss = 0.002849`, explained variance 0,8018, esplicitamente in-sample;
- actor digest invariato `d4a13ff7…`;
- sigma invariata `0.00499999967`;
- optimizer state solo sugli indici critic 6–11, step 81, LR `1e-4`;
- tutti i 4096 record di telemetria presenti;
- zero `phase_timeout_stance`, `phase_timeout_swing`, morphology failure, resync e HS cancellation.

Il verdetto fu `AWAITING_RESTORE_AUDIT`: 12/12 immediati, ma G9 non era chiuso finché il checkpoint non fosse stato ricaricato dal percorso reale. Report: [J20 critic-only warm-up](2026-08-27_v26c_j20_critic_warmup_execution.md).

### 10.4 Restore audit R1, R2, R3

**R1 — FAIL tecnico.** Timeout a 600 s durante `build_algo`; 115 errori `ModuleNotFoundError: train_ppo_mlp` nei worker. Il driver aveva mutato `sys.path`, ma i processi child non ereditavano quella modifica. Correzione R2: prepend della directory `baseline_MLP` al `PYTHONPATH` dei child.

Report: [Restore audit R1](2026-08-27_v26c_j20_restore_audit_esecuzione.md).

**R2 — FAIL dell'audit, non del checkpoint.** Il restore arrivò al confronto optimizer e mostrò otto differenze: boolean Python contro tensori 0D e literal float64 contro round-trip float32 nei param group. I sei momenti Adam erano identici. L'audit confrontava rappresentazioni invece della semantica canonica RLlib.

Report: [Restore audit R2](2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).

**R3 — PASS.** Canonicalizzazione limitata esattamente alle otto conversioni osservate, momenti lasciati intatti e gate non vacuo sul learning rate. Risultato:

- 13/13;
- zero nuove iterazioni;
- actor, critic e sigma esatti;
- sei coppie di momenti Adam, step e LR esatti;
- marker `RESTORE_AUDIT_PASSED`;
- G9 chiuso.

Report: [Restore audit R3](2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).

R3 lasciò correttamente `training_ready: false`: chiudeva solo il restore, non poteva aggregare autonomamente tutte le evidenze.

---

## 11. Fase H — J21: attestazione training-ready

J21 non costruisce ambiente, non avvia Ray, non campiona e non addestra. È un aggregatore standard-library-only che ri-hasha e collega le evidenze già committate.

Prima dell'esecuzione furono corretti quattro intoppi nel pacchetto preparatorio:

1. testi rimasti a 15 gate mentre il contratto ne conteneva 18;
2. una frase “nessun training” negava impropriamente il warm-up critic-only già svolto;
3. la prova che nessun PPO/ex-novo fosse partito era troppo indiretta;
4. la doppia autorizzazione utente + architetto non era meccanicamente obbligatoria nel GO.

J21 finale:

- 381/381 check nella suite ermetica;
- preflight 18/18 e 139/139 pin;
- GO 141/141 pin;
- esecuzione singola, 0 child process;
- 18/18 gate;
- `verdict = TRAINING_READY_ATTESTED`;
- `training_ready = true`;
- `promotion = TRAINING_INPUT_ONLY`;
- `training_started = false`;
- `launch_authorized = ppo_authorized = ex_novo_authorized = false`;
- `deployable = false`.

I 18 gate coprono lineage agosto, riproducibilità J19A, 6/6 J19B, 3/3 held-out, penetrazione, contatori FSM/morphology, gradiente K1R1, warm-up, restore R3, optimizer, immutabilità, file checkpoint, pin runtime/source e assenza di training a valle.

Report preparatorio e finale: [Preparazione J21](2026-08-28_v26c_j21_preparazione_attestazione_training_ready.md), [Correzione rev1 J21](2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md), [Esecuzione J21](2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md).

---

## 12. Procedura di ricostruzione consigliata a un futuro LLM

Questa è la procedura **minima informata dagli errori**, non la ripetizione cieca di ogni ramo fallito.

### Passo 1 — congelare un nuovo namespace

Non scrivere nei leaf storici. Sono no-clobber e molti GO sono monouso già consumati. Creare una nuova radice versionata dentro `Trajectory Generator/baseline_MLP/validation/`, quindi:

1. copiare o derivare gli strumenti necessari con nuovi nomi di stadio/leaf;
2. aggiornare preregistrazioni e pin;
3. generare nuovi GO separati;
4. conservare gli artefatti storici immutati;
5. eseguire prima tutte le suite e i preflight no-write.

Mai cancellare un leaf esistente per “ritentare”. Un fail tecnico deve produrre uno stadio additivo R1/R2/R3.

### Passo 2 — verificare il parent agosto

Richiedere:

- modulo SHA `0ba56eb7…`;
- manifest 39D coerente;
- config risolto del fresh run;
- detector/corridoio correnti pinnati;
- nessun checkpoint o dataset luglio tra gli input.

### Passo 3 — J1 e J2

1. raccogliere 500 step teacher nominali con full runtime v3/corridor;
2. accettare 20/25 come diagnostiche, bloccare solo oltre 28 mm;
3. derivare il 35D per nome, compensare il bias float64;
4. mascherare clock e memoria controller durante BASE;
5. fit BASE con gli iperparametri recuperati;
6. verificare la cinematica closed-loop prima di qualsiasi Markov/critic.

### Passo 4 — testare sigma e costruire J7/J8

1. eseguire tre probe stocastici;
2. misurare sigma realizzata e mismatch prefixes;
3. costruire 16000 self-anchor nominali + recovery student-visited allo stesso rapporto di luglio;
4. riattivare 25–34 nello stesso attore 35D;
5. fit MARKOV;
6. qualificare A–F.

### Passo 5 — correggere solo le celle fallite

Se si riproduce il pattern J8 5/6:

1. raccogliere il teacher per lo start B e lo start C;
2. non eseguire un refit fresh dall'attore BASE;
3. costruire i quattro blocchi J18 A/B/C/D disgiunti;
4. usare preservation in output space e pesi espliciti per blocco;
5. congelare la griglia finita prima di eseguirla;
6. scegliere solo dopo criteri offline preregistrati;
7. riprodurre una volta il candidato scelto;
8. qualificare 6/6 e poi 3/3 held-out.

Con il design finale già noto, il ramo J11→J17 può essere omesso da una ricostruzione **ottimizzata**, perché non è una dipendenza dei byte finali. Deve però restare documentato come giustificazione causale del design J18.

### Passo 6 — critic e restore

1. costruire il target trainabile asymmetric actor35/critic84;
2. provare il gradiente critic con input non degenere;
3. eseguire una sola iterazione critic-only da 4096 step;
4. verificare actor/logstd bit-exatti su driver e worker;
5. verificare optimizer state solo critic;
6. salvare checkpoint completo;
7. ricaricarlo dal percorso production `train_ppo_mlp.run -> algo.restore_from_path` con zero iterazioni;
8. confrontare modulo, actor, critic, sigma, momenti Adam, step e LR sotto la semantica di conversione reale RLlib.

### Passo 7 — attestare senza lanciare

Aggregare in uno stadio separato, senza import pesanti o runtime, e dichiarare:

```text
training_ready = true
promotion = TRAINING_INPUT_ONLY
deployable = false
launch_authorized = false
ppo_authorized = false
ex_novo_authorized = false
```

Il pilot futuro deve avere una nuova preregistrazione, un nuovo comando verificato, stop rule, monitor, rollback e una nuova autorizzazione utente.

---

## 13. Comandi storici degli stadi V26C

Questi comandi documentano la superficie CLI. **Non rilanciarli in-place**: gli output esistono e i token/GO storici non autorizzano nuove esecuzioni. In un nuovo namespace, sostituire leaf, stage, prereg e GO con versioni nuove e approvate.

Interpreter storico:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python
```

CWD storico:

```bash
cd "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
```

Pattern preflight/esecuzione:

```bash
# J1
python v26c_j1_collect.py --preflight
python v26c_j1_collect.py --authorized-stage V26C-J1-TEACHER-COLLECT --out-dir <NEW_LEAF>

# J2
python v26c_j2_fit.py --preflight
python v26c_j2_fit.py --authorized-stage V26C-J2-BASE-FIT --out-dir <NEW_LEAF>

# J3
python v26c_j3_closed_loop.py --preflight
python v26c_j3_closed_loop.py --authorized-stage V26C-J3-CLOSED-LOOP --out-dir <NEW_LEAF>

# J4/J5: necessari solo per una replica cronologica completa
python v26c_j4_recovery.py --preflight
python v26c_j4_recovery.py --authorized-stage V26C-J4-RECOVERY --out-dir <NEW_LEAF>
python v26c_j5_revalidation.py --preflight
python v26c_j5_revalidation.py --authorized-stage V26C-J5-REVALIDATION --out-dir <NEW_LEAF>

# J6: ripetere solo per i seed preregistrati
python v26c_j6_recovery_probe.py --preflight
python v26c_j6_recovery_probe.py --authorized-stage V26C-J6-RECOVERY-PROBE --seed 123 --out-dir <NEW_LEAF>

# J7
python v26c_j7_materialize.py --preflight
python v26c_j7_materialize.py --materialize --authorized-stage V26C-J7-MATERIALIZE --out <NEW_LEAF>

# J8
python v26c_j8_recovery_fit.py --preflight
python v26c_j8_recovery_fit.py --fit --authorized-stage V26C-J8-RECOVERY-FIT --out <NEW_LEAF>

# J9R1
python v26c_j9r1_closed_loop.py --preflight
python v26c_j9r1_closed_loop.py --run --authorized-stage V26C_J9R1_CLOSED_LOOP --out <NEW_LEAF>

# J10R1
python v26c_j10r1_multistart_teacher.py --preflight
python v26c_j10r1_multistart_teacher.py --collect --authorized-stage V26C_J10R1_MULTISTART_TEACHER --out <NEW_LEAF>

# J11-J16: ramo diagnostico storico, non antenato dei pesi finali
python v26c_j11_multistart_fit.py --preflight
python v26c_j11_multistart_fit.py --fit --authorized-stage V26C_J11_MULTISTART_FIT --out <NEW_LEAF>
python v26c_j12_closed_loop.py --preflight
python v26c_j12_closed_loop.py --run --authorized-stage V26C_J12_CLOSED_LOOP_QUALIFICATION --out <NEW_LEAF>
python v26c_j14_dagger_dataset.py --preflight
python v26c_j14_dagger_dataset.py --materialize --authorized-stage V26C_J14_POST_MISMATCH_DAGGER_DATASET --out <NEW_LEAF>
python v26c_j15_fresh_refit_r1.py --preflight
python v26c_j15_fresh_refit_r1.py --fit --authorized-stage V26C_J15_FRESH_35D_POST_MISMATCH_REFIT --out <NEW_LEAF>
python v26c_j16_closed_loop.py --preflight
python v26c_j16_closed_loop.py --run --authorized-stage V26C_J16_J15R1_CLOSED_LOOP_REQUALIFICATION --out <NEW_LEAF>

# J18/J19A usano GO content-addressed
python v26c_j18_b_only_update.py --preflight-only
python v26c_j18_b_only_update.py --execute --go-file <NEW_APPROVED_GO>
python v26c_j19a_single_reproduction.py --preflight-only
python v26c_j19a_single_reproduction.py --execute --go-file <NEW_APPROVED_GO>

# J19B/J19C
python v26c_j19b_closed_loop.py --preflight
python v26c_j19b_closed_loop.py --run --authorized-stage V26C_J19B_J19A_CLOSED_LOOP_QUALIFICATION --out <NEW_LEAF>
python v26c_j19c_heldout_g_i.py --preflight
python v26c_j19c_heldout_g_i.py --run --authorized-stage V26C_J19C_J19A_HELDOUT_G_I --out <NEW_LEAF>

# K1R1 e J20
python v26c_j20_k1r1_gradient_amendment.py --preflight-only
python v26c_j20_k1r1_gradient_amendment.py --probe --go-file <NEW_APPROVED_GO>
python v26c_j20_critic_warmup_execution.py --preflight-only
python v26c_j20_critic_warmup_execution.py --execute --go-file <NEW_APPROVED_GO>

# Restore R3
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
  v26c_j20_restore_audit_r3.py --execute --go-file <NEW_APPROVED_GO>

# J21
python v26c_j21_training_ready_attestation.py --preflight-only
python v26c_j21_training_ready_attestation.py --execute --go-file <NEW_APPROVED_GO>
```

Nei run reali usare sempre l'interprete assoluto e registrare cwd, argv, config risolto, stdout/stderr, exit code, hash degli input e set esatto dei file committati. Il semplice token `--authorized-stage` non sostituisce un'autorizzazione utente quando lo stadio modifica una scelta scientifica o lancia training.

---

## 14. Catalogo sintetico degli intoppi e delle correzioni

| sintomo | causa | correzione | regola permanente |
|---|---|---|---|
| episodi imitativi ~250 step | timeout swing 1,3 s incompatibile con V26 | 2,6 s per lineage comparabile | misurare distribuzione delle fasi prima del full run |
| contatori apparentemente tutti zero | worker morto prima di pubblicare telemetria | audit dei processi e completezza telemetria | assenza di dato ≠ zero |
| errori off-grid in massa | `reject_continue` avanzava il cursore senza commit FSM | `drop` + single commit | sensore e FSM devono committare atomicamente |
| microstep a orizzonte | confronto float con tolleranza `1e-12` | half-policy-window criterion | non usare uguaglianze quasi-esatte sui tempi integrati |
| batch/lifetime scambiato per environment | contatori RLlib cumulativi e relaunch round | distinguere batch, runner e lifetime | 4096 non è il numero di env |
| B0820 parte ma cammina male | 39→35 senza target-domain refit; sigma ~0,5 | BASE + Markov/DAgger, sigma 0,005 verificata | readiness tecnica non certifica comportamento |
| J1/J3/J5 segnati FAIL | 20 mm trattati inizialmente binding | contratto 20 soft / 25 diag / 28 hard | non riscrivere receipt storici; applicare contratto versionato |
| J4 recovery quasi nullo | stop al primo mismatch, 12 step | J6/J7 con 713 prefix | misurare supporto, non copiare un repeat blindly |
| J5 R1 exception SciPy | `-W error` troppo globale | warning policy mirata | warning numerico noto non deve diventare crash infrastrutturale |
| J8 manifest stantio | sidecar copiato da J2 | identità dai byte + overlay provenance | non fidarsi di un manifest senza cross-check sul modulo |
| J9 output mancanti | `output_dir=None` disabilitava il recorder | J9R1 con output config esplicito | verificare set esatto dei file prima del rollout |
| J9 B 0 penetrazione ma FAIL | nessun contatto, FSM sempre WAIT_HS | teacher multistart J10R1 | una metrica di sicurezza può passare vacuamente |
| J10 readiness respinta | indice 39D, file check debole, pin mancanti | J10R1 live manifest + 19 file + hash | il preflight deve validare lo schema runtime reale |
| J11 offline ottimo, J12 4/6 | shift su E/F e supporto post-mismatch assente | diagnosi J13 | closed-loop è selection gate |
| J14 digest autoreferenziale | root digest includeva il proprio file | digest self-excluding + GO additivo | un artefatto non può contenere il proprio hash stabile |
| J15 NameError | alias non definito, test con fake path | J15R1 + test free-name reali | i test devono esercitare il percorso production |
| J15R1 offline migliora, J16 0/6 | recovery 84,81% della massa gradiente; fresh parent sbagliato | update J18 su J8 con preservation output-space | percentuale di righe ≠ influenza sulla loss |
| J18 nessun survivor | max drift single-sample 0,005 non calibrato | ricalibrazione su bias/RMSE operativi J8 | un gate deve essere giustificato e approvato, non solo severo |
| K1 peso critic gradiente zero | batch sintetico tutto zero | stimolo non degenere 8×84 | un test può costruire artificialmente il fallimento |
| restore R1 timeout/import | `sys.path` driver non propagato ai child | `PYTHONPATH` per child | testare il percorso multiprocesso reale |
| restore R2 otto mismatch | rappresentazioni Python/tensor/float diverse | canonicalizzazione semantica mirata | non confondere serializzazione con corruzione |
| J21 testo 15 vs 18 gate | pacchetto preparatorio non aggiornato | rev2, suite e doppia autorizzazione | testo, schema e gate devono concordare meccanicamente |

---

## 15. Checklist di accettazione prima di dichiarare training-ready

Un futuro LLM deve poter rispondere **sì** a tutto:

- [ ] parent operativo esclusivamente V26 agosto, verificato per hash;
- [ ] nessun peso/dataset/trace luglio negli input;
- [ ] attore unico 35D, 2×256 tanh, nessun adapter e nessun 25D separato;
- [ ] mapping feature per nome e bias compensation documentati;
- [ ] teacher collection full runtime V26/FSM v3/corridor;
- [ ] BASE 500 coppie e Markov/recovery tracciati per riga;
- [ ] sigma misurata, non assunta;
- [ ] dataset DAgger usa stati student-visited e label teacher same-step;
- [ ] pesi dei blocchi espliciti e preservation funzionale;
- [ ] 6/6 A–F e 3/3 G–I held-out;
- [ ] tutte le celle 500 step con cicli validi;
- [ ] `phase_timeout_stance`, `phase_timeout_swing`, `morphology_causal_contract_failure`, `resync_count`, `hs_cancelled_count` tutti zero;
- [ ] nessuna penetrazione >28 mm; bande 20/25 riportate come diagnostiche;
- [ ] caviglia con fase negativa e ROM biologicamente plausibili; ginocchio senza collasso;
- [ ] attore invariato durante qualifica e critic warm-up;
- [ ] critic con gradiente non degenere e digest cambiato;
- [ ] optimizer state solo critic dopo warm-up;
- [ ] restore production con zero iterazioni e stato esatto;
- [ ] attestazione separata, content-addressed, senza ambiente o training;
- [ ] training, PPO, ex-novo e deployment ancora falsi/non autorizzati.

---

## 16. Limiti e TODO aperti

- Il checkpoint è qualificato su **AB06**, non generalizzato automaticamente ad altri soggetti o modelli. Il TODO multimodello basato sul dataset EPIC resta in [Generalizzazione multimodello ed EPIC](2026-08-22_todo_generalizzazione_multimodello_epic.md).
- `sigma = 0.005` è verificata per questa qualifica, ma va rivalutata nel protocollo del pilot.
- Tutte le nove celle A–I superano 20 mm; F e H entrano nella banda diagnostica 25 mm. Nessuna supera 28 mm, ma il margine minimo è circa 2,316 mm e va monitorato.
- LOTO, LOCO, B1R1 e B1R2 restano TODO futuri. Non fanno parte della baseline operativa e nessun LLM deve integrarli senza approvazione esplicita dell'utente.
- Il sidecar J8 stantio resta un debito di provenance storico; non modificarlo retroattivamente.
- Il protocollo del pilot PPO, il comando, gli iperparametri, lo stop rule, il monitor e il rollback **non sono ancora sigillati**.

---

## 17. File modificati e verifiche di questa guida

File creato:

- `reports/user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md`

Nessun codice, checkpoint, dataset, config, preregistrazione, GO o artefatto storico è stato modificato.

Verifiche da eseguire alla chiusura del report:

- esistenza di ogni user report collegato;
- esistenza dei quattro artefatti canonici indicati in §0;
- riscontro dei digest principali nei receipt correnti;
- controllo che la lineage descritta coincida con J18/J19A/J21;
- controllo che il report non autorizzi o suggerisca il lancio in-place di alcun training.

**Conclusione:** la rete corrente è arrivata a training-ready non tramite PPO correttivo né aumento di capacità, ma tramite un bridge supervisionato 39D→35D, raccolta Markov student-visited, diagnosi closed-loop del supporto mancante, update funzionalmente vincolato sul migliore attore J8, qualifica 6/6 + held-out 3/3, warm-up esclusivo del critic, restore audit reale e attestazione separata. Il prossimo training resta una fase nuova e non autorizzata.
