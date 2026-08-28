# Supplemento additivo alla Fase A — rettifica metodologica e protocollo 25D → 35D

**Data**: 2026-08-25 · **Natura**: audit read-only e progettazione. Nessun fit, training, rollout o candidato. Nessun file esistente modificato.
**Rettifica**: questo supplemento è autoritativo sulle affermazioni del §3 del report Fase A (`4c337bcf…`), che resta preservato byte-identico insieme ai suoi receipt (`716a779d…`, `23281268…`).

---

## 1. RETTIFICA — errore metodologico mio

**Ciò che ho scritto in Fase A §3**: «Mismatch discreto — la regola "phase-aligned" di luglio … usa verbatim `truncate_before_discrete_mismatch` … nominal = stati del corpus teacher allo stesso indice assoluto», e da lì ho concluso che «la regola di luglio, applicata fedelmente su questo lineage … non può correggere lo swing».

**Perché è sbagliato**. L'operatore confronta la traccia nominale e le recovery stocastiche **dello stesso attore**, non teacher contro student. È verificabile al sito di chiamata, `target_domain_markov_adaptation.py:331-343`:

```
nominal_rows  = _load_json(nominal_trace_path)          # traccia NOMINALE del source actor
recovery_rows = [_load_json(p) for p in recovery_paths] # rollout STOCASTICI dello STESSO actor
... noise_adaptation.truncate_before_discrete_mismatch(nominal_rows, rows, feature_names)
```

È un filtro di **auto-consistenza sul rumore**: tiene le righe di un rollout perturbato finché lo stato discreto coincide con quello che *quello stesso attore* aveva nella propria traccia nominale allo stesso indice. Lo avevo già registrato io stesso nell'addendum rev4b (`74f7c50a…`, CORREZIONE_1) e nell'emendamento rev4c §ARCHITECTURAL_FRAMING_BINDING, e in Fase A l'ho contraddetto.

**Cosa resta valido del §3**: i numeri sono corretti come **controfattuale teacher-vs-student**, cioè come misura di quanto presto lo studente diverge in fase dal teacher (S1A step 14 per un flicker di contatto; REV4C/REV4E step 62 per un toe-off mancato). È un dato utile e lo conservo con quella etichetta.

**Cosa è ritirato**: la frase «la regola di luglio non può correggere lo swing» e ogni conclusione sul protocollo storico che ne discende, inclusa la voce **TODO-8** del report Fase A. Non è dimostrata: il protocollo di luglio non applicava mai quell'operatore fra teacher e student.

---

## 2. FATTI — la sequenza riuscita del 13 luglio, ricostruita

Fonti: `reports/user/2026-07-13_actor_robust_deployable_critic_warmup.md`, `2026-07-13_sblocco_actor_markov35_e_warmup_critic.md`, `2026-07-13_ablation_memoria_controller_actor.md`, `baseline_MLP/target_domain_markov_adaptation.py`, `baseline_MLP/target_domain_noise_adaptation.py`, `validation/controller_memory_ablation/2026-07-13_markov35_final_gate.json`.

### 2.1 Diagnosi

Trace corretto a `sigma=0.005`, seed 123: rumore RMS 0,005103 / 0,004554; deriva RMS della media 0,240576 / 0,095030; **primo drift > 0,05 allo step 76**, **primo mismatch FSM/eventi allo step 120**, fine allo step 206 per penetrazione. La deriva precede il disallineamento di fase.

Controfattuale per gruppi di feature: sostituire lo stato interno nominale del controller/reference eliminava il **47,52%** della deriva della media. Feature principali nominate: accelerazione della reference servita del ginocchio, endpoint precedente del ginocchio, velocità della reference servita, stato articolare e motor speed del ginocchio.

Causa dichiarata: durante l'imitazione, previous endpoint, reference servita e ultimo comando SEA sono fortemente correlati con l'azione prescribed; l'actor li usa come **proxy autoregressivo** della prossima azione. Una piccola perturbazione dell'azione cambia questi input al passo dopo, la policy sposta la media e amplifica la deviazione closed-loop.

### 2.2 Correzione strutturale — il 25D

`include_controller_state_observation=false`: previous endpoint, reference servita e ultimo comando SEA escono dall'actor; le **14** feature (10 di stato + 4 diagnostiche `_abs`/`_saturated`) si spostano nel suffisso privilegiato del critic; il vettore completo resta 84; **l'actor passa da 39 a 25 feature**.

Warm start imitativo a 25 feature: teacher 500/500, HS/TO/cicli 3/3/2, return 64,0324, penetrazione max 22,944 mm. Adattato **solo** l'actor portato, su 500 coppie osservazione-actor → azione prescribed; 243 epoche, best 183, RMSE 0,011782, errore max 0,074282, sigma 0,005 costante, **0 update PPO**. Digest `5616be85c815eece79c08730d05529a7b41ea60f3e36033fb5276edc83dd2bf7`.

### 2.3 Espansione 25 → 35 con colonne a zero

«L'actor da 25 feature è stato espanso a 35 **azzerando le dieci nuove colonne del primo layer**. Tutti i pesi condivisi sono rimasti invariati; il rollout deterministico iniziale ha mantenuto 500 step e 2 cicli.»

Questo è il passaggio strutturale: si parte da un attore **già 500/500** le cui nuove feature hanno peso nullo, quindi non può ancora usarle come proxy; solo dopo si insegna a usarle.

### 2.4 Il fallimento del primo 35D e la sua causa

Primi candidati: addestrate **solo** le 10 nuove colonne, tutto il resto congelato. Due rifiutati dal gate nominale (scostamento massimo 0,03570 e 0,00611 contro limite 0,005); il conservativo con 96 repliche dei 500 stati nominali passò (RMS 0,001336, max 0,004909) ma fallì gli start alternativi.

Causa accertata nel dataset DAgger, non nell'architettura né nella reward:

```
campioni recovery originali   1500
ancora allineati a FSM         356
label fuori fase              1144   -> 76,27%
primo mismatch nei 3 trace   120 / 119 / 120
```

Le azioni teacher erano associate **per indice temporale fisso**; dopo che il rollout perturbato cambiava contatto o stato FSM, la label apparteneva a una fase discreta diversa → supervisione contraddittoria. Cause secondarie: accelerazioni della reference servita fino a 60 rad/s² che dominavano numericamente gli input; dataset sbilanciato verso il nominale; aggiornare solo le nuove colonne non permetteva di apprendere le interazioni con FSM, contatto e stato fisico. **La reward non era la causa: il guard a 25 mm rivelava correttamente l'errore e non fu modificato.**

### 2.5 Le correzioni e il dataset finale

In `target_domain_markov_adaptation.py`: troncamento automatico di ogni trace al primo mismatch discreto (**same-actor: nominale contro recovery stocastico**); scale fisiche fisse per velocità e accelerazioni; peso indipendente per i dataset degli start alternativi; opzione per adattare l'intera mean-network; `logstd` sempre congelato e critic escluso.

Scale (`MARKOV_CONTROLLER_FEATURE_SCALES`): knee vel 4,0 · knee accel 60,0 · ankle vel 3,5 · ankle accel 55,0 · le altre 1,0.

Dataset finale, 24 712 campioni:

| blocco | campioni | quota | **label** |
|---|---|---|---|
| ancore nominali del source actor | 16 000 | 64,7% | **le medie dell'attore stesso** (`nominal_means`) |
| recovery stocastici phase-aligned | 712 | **2,9%** | teacher a indice fisso |
| teacher su start −0,20 / +0,20 s | 8 000 | 32,4% | teacher |

Il fatto che le ancore nominali portino **la media dell'attore stesso** e non il teacher è verificabile in `build_markov_recovery_dataset`: `actions = concat([tile(nominal_means, …), tile(recovery_labels, …)])`. È auto-distillazione di preservazione del comportamento, non imitazione.

### 2.6 Esito e gate

Attore selezionato `a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21`. RMSE aggregato 0,019657 → 0,008144; shift nominale RMS 0,004175, max 0,031594, p95 0,007956; RMSE start −0,20 s 0,007830, +0,20 s 0,007209.

Closed loop: **3/3 start deterministici a 500/500 con 2 cicli**; **3/3 seed stocastici σ=0,005 a 500/500 con 3 cicli**; clipping 0; penetrazione 23,27–24,60 mm sotto guardie 15/25 mm.

Refinement rifiutato: shift nominale RMS migliore (0,002948) ma start −0,20 s morto allo step 221 con 25,090 mm. **Minimizzare la metrica offline non è monotono rispetto alla robustezza dinamica.**

Il gate di selezione passò con **due voci esplicitamente FALSE**: `nominal_return_not_regressed_vs_25_feature_baseline` e `strict_offline_single_sample_shift_gate`. Razionale registrato: la robustezza closed-loop su start ed esplorazione è il gate primario del warm start.

**La tua ipotesi è confermata in ogni sua parte.**

---

## 3. FATTI — confronto feature-per-feature con il percorso di agosto

| | luglio 13 | agosto V26 |
|---|---|---|
| sorgente | actor 39D target-domain | `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best`, **39D**, digest `5bbc6cbd…` |
| `include_controller_state_observation` alla sorgente | true | **true** (config risolto riga 60) |
| stadio 25D | **sì**, obbligatorio, portato a 500/500 prima di espandere | **mai eseguito** |
| passaggio a 35D | espansione da 25D con **10 colonne a zero** | **trapianto diretto 39D → 35D**, rimosse solo le 4 diagnostiche |
| stato controller nell'actor finale | 10, riappreso con dataset dedicato | 10, **ereditati con i pesi della sorgente** |

Le 10 feature sono le stesse in entrambi i percorsi: `pros_{knee,ankle}_angle_{previous_endpoint, served_ref, served_ref_vel, served_ref_accel, sea_u}`. Il 39D di luglio e quello di agosto differiscono dal 35D per le stesse 4 diagnostiche `_abs`/`_saturated`.

### 3.1 Il proxy autoregressivo è presente e misurato

Contributo del primo layer alle 10 colonne, misurato come `‖W[:,j]‖₂ · std(x_j)` normalizzato (identico su `pi.0.0.weight` e `pi_encoder.0.weight`):

| attore | quota delle 10 colonne | riferimento uniforme |
|---|---|---|
| **V1_35D_transplant** | **0,7125** | 0,2857 |
| S0D | 0,6723 | |
| S1A | 0,6721 | |
| S1C2Z_L20 | 0,6726 | |
| REV4C | 0,6671 | |
| REV4E | 0,6624 | |

Le tre colonne dominanti sono `pros_knee_angle_served_ref_accel`, `pros_ankle_angle_served_ref_accel`, `pros_knee_angle_served_ref_vel` — **esattamente le feature nominate dalla diagnosi di luglio**.

Il valore è già **0,7125 in V1**, cioè ereditato dalla sorgente V26 e non creato dai nostri fit: i fit successivi lo muovono di meno di 5 punti percentuali.

Controfattuale open-loop (sostituire lo stato controller con quello nominale, sulle tracce reali):

| traccia | \|Δmedia\| medio | p90 | max | rapporto alla deviazione totale dal nominale |
|---|---|---|---|---|
| S1A | 0,42399 | 0,96535 | 1,43620 | 1,00 |
| REV4C | 0,16674 | 0,45495 | 0,96483 | 1,03 |
| REV4E | 0,20290 | 0,46342 | 1,10413 | 1,20 |

Il rapporto ≈ 1 significa che **quasi tutta la deviazione dell'attore dal proprio comportamento nominale passa dal canale di stato controller**; le altre 25 feature vi contribuiscono in modo trascurabile.

*Limite dichiarato*: luglio misurò l'eliminazione di deriva in **closed loop sotto perturbazione** (47,52%); questa è una sensibilità **open-loop** su tracce congelate. Stimatori diversi dello stesso fenomeno; i rapporti >1 indicano che le due differenze non sono collineari, quindi la lettura "quota" è approssimata.

### 3.2 Perché il proxy si forma — quantificato

Varianza di `u_IK` spiegata da regressione **lineare** sul corpus teacher (limite inferiore, nessuna rete):

| sottospazio | R² knee | R² ankle |
|---|---|---|
| tutte le 35 | 0,9574 | 0,9274 |
| **solo le 10 di stato controller** | **0,9219** | **0,8623** |
| solo le 25 sensoriali | 0,9478 | 0,9046 |

Dieci feature che sono funzioni dei comandi passati dell'attore spiegano da sole l'86–92% della varianza del bersaglio. La scorciatoia è disponibile e conveniente: è la ragione per cui la rete vi concentra il 66–71% della massa del primo layer.

**Risposta alla tua domanda (3): sì.** Il salto diretto al 35D non ha "reintrodotto" il proxy — **non lo ha mai rimosso**. È stato ereditato dalla sorgente V26 al 71% e conservato da ogni candidato, S1A e L20 inclusi.

---

## 4. FATTI — i due test minimi, già eseguiti in sola lettura

### 4.1 L'alternativa economica è CONFUTATA

Azzerare in loco le 10 colonne del primo layer su un artefatto 35D pulito e sperare che l'attore resti funzionale:

| attore | \|Δmedia\| medio | p90 | max | RMSE vs u_IK prima → dopo |
|---|---|---|---|---|
| S0D | 0,34451 | 0,76183 | 1,66816 | 0,44691 → 0,45555 |
| S1A | 0,32820 | 0,72124 | 1,60879 | **0,08070 → 0,47014** |
| S1C2Z_L20 | 0,32069 | 0,68568 | 1,63564 | 0,35795 → 0,42530 |

Contro un'ampiezza media di comando |u_IK| = 0,37078. L'uscita cambia di circa il 90% della propria ampiezza tipica e l'RMSE di S1A degrada di 5,8×. **Le 25 colonne residue non sono autosufficienti**: non sono mai state addestrate a portare il compito da sole. Il 25D va **riaddestrato**, non ablato — esattamente come fece luglio.

### 4.2 Il percorso raccomandato è FATTIBILE

Identificabilità di `u_IK` sul corpus teacher, vicino più prossimo nel sottospazio:

| sottospazio | dim vive | \|Δlabel\| mediana | p90 | >0,10 | max |
|---|---|---|---|---|---|
| tutte le 35 | 32 | 0,0108 | 0,0464 | 5,6% | 0,2895 |
| **solo le 25 sensoriali** | 22 | **0,0127** | 0,0466 | 5,0% | 0,2895 |

Degrado 1,17×, con R² lineare 0,9478 / 0,9046 contro 0,9574 / 0,9274. **Il contratto 25D conserva l'informazione necessaria a imitare `u_IK`**: lo stadio di imitazione IK a 25 feature è fattibile su base informativa.

---

## 5. PERCORSO RACCOMANDATO — riprodurre l'architettura di luglio sul lineage di agosto

Lineage esclusiva agosto V26. Nessun checkpoint, dataset, label o anchor di luglio: di luglio si riusa **solo il metodo**, e il codice di produzione già presente nel repo.

| stadio | parent (obbligatorio) | operazione | gate fail-closed |
|---|---|---|---|
| **A0 — trapianto 25D** | V26 39D `5bbc6cbd…` | portare le 25 colonne sensoriali, azzerare i due clock, `include_controller_state_observation=false` | 25 colonne bit-identiche alla sorgente; actor identico su learner/EnvRunner/export; nessun peso condiviso alterato |
| **A1 — imitazione IK 25D** | il trapianto A0 | fit supervisionato sul corpus AB06 corrente (500 coppie osservazione-actor → `u_IK`), logstd congelata, 0 PPO, critic escluso | **rollout nominale deterministico 500/500 con ≥2 cicli** sotto guardie v3 invariate; clipping 0; integrità 10 chiavi/save-reload |
| **A2 — espansione 35D a colonne nulle** | il 25D 500/500 di A1 | aggiungere le 10 colonne del primo layer **inizializzate a zero**; tutti gli altri pesi invariati | il rollout deterministico deve riprodurre **bit-per-bit** quello di A1: stessi step, stessi cicli, stessa traccia |
| **A3 — adattamento Markov** | il 35D espanso di A2 | dataset a tre blocchi (§5.1), scale fisiche già in pipeline, intera mean-network addestrabile, logstd congelata, critic escluso, 0 PPO | integrità; shift nominale riportato (RMS/max/p95) **senza soglia inventata**; poi closed loop |
| **A4 — validazione** | il candidato A3 | rollout deterministici sui tre start (nominale, −0,20 s, +0,20 s) | 3/3 a 500/500 con ≥2 cicli, clipping 0, penetrazione entro guardia v3 corrente |

### 5.1 Dataset di A3 — forma di luglio, contenuti di agosto

```
ancore nominali          = stati della traccia nominale dell'attore A2,
                           etichettati con LE MEDIE DELL'ATTORE STESSO      (preservazione)
recovery phase-allineate = rollout stocastici sigma=0.005 dello STESSO attore A2,
                           troncati al primo mismatch discreto contro la
                           PROPRIA traccia nominale                          (correzione)
teacher su start alternativi = corpus AB06 corrente a -0.20 s e +0.20 s      (generalizzazione)
```

Pesi relativi da preregistrare. Riferimento storico: 64,7% / 2,9% / 32,4%. Nessuna interpolazione in questo round.

### 5.2 Ruolo dell'intermedio 25D

**Non è un artefatto di comodo: è il solo punto della catena in cui il proxy autoregressivo non esiste**, perché il canale è fisicamente assente dall'osservazione. Serve a ottenere un attore che cammini 500/500 usando *soltanto* cinematica protesica, stato SEA, GRF ipsilaterale e FSM. Senza questo passaggio, ogni fit successivo parte da pesi che già instradano il 66–71% del primo layer sulla scorciatoia, e il §4.1 mostra che quella dipendenza non è rimovibile a posteriori.

### 5.3 Test minimo prima di spendere il fit A1

Entrambi già eseguiti e riportati al §4: l'identificabilità 25D **passa** (1,17× di degrado) e l'ablazione in loco **fallisce** (0,32–0,34 di scostamento). Prima di A1 resta un solo controllo, offline e read-only: **verificare che il trapianto A0 riproduca la sorgente V26 sulle 25 colonne bit per bit** e che l'osservazione a 25 feature sia effettivamente producibile dall'env corrente con il flag a false, confrontando i nomi generati con l'attesa `35 − 10`. È aritmetica e confronto di manifest, nessuna simulazione.

---

## 6. ALTERNATIVA (una sola)

**Riaddestramento 25D a partire da S0D anziché dalla sorgente V26.** Identica ad A0–A4 salvo il parent di A0: si proietta `S0D` (`481dd0d2…`, già distillato e 500/500) sulle 25 colonne invece della sorgente V26 39D.

*Vantaggio*: S0D è già un artefatto 35D validato in closed loop sul runtime v3, quindi A1 parte più vicino alla soluzione.
*Svantaggio dirimente*: S0D ha imparato con il canale proxy disponibile e vi concentra il 67,2% del primo layer; proiettarlo sulle 25 colonne significa partire da pesi ottimizzati per un'osservazione che non esiste più. Il §4.1 misura questa perdita: |Δmedia| 0,3445 e RMSE 0,4469 → 0,4556.

La scarto come raccomandazione, ma la lascio a te perché costa un fit in meno se il gate di A1 la promuove comunque.

---

## 7. RISCHI

1. **Il flag `include_controller_state_observation=false` cambia il contratto osservativo dell'actor**, quindi A0–A2 richiedono un config risolto diverso da `v3_canonical` (che lo ha `true`). Non è una modifica a FSM/morfologia/contatto/reward/guardie, ma **cambia i pin di runtime** citati in tutti i receipt esistenti. Va deciso esplicitamente come tracciare i due contratti.
2. **A1 potrebbe non raggiungere 500/500.** Luglio ci arrivò da un attore che era già stato addestrato con PPO nel target domain; il nostro parte da imitazione V26. L'identificabilità (§4.2) dice che l'informazione c'è, non che il fit la troverà.
3. **Il gate di A2 è severo**: la riproduzione bit-per-bit del rollout richiede che l'espansione a colonne nulle sia esatta e che l'env produca l'osservazione a 35 feature in modo deterministico. Un solo bit di differenza va trattato come fallimento, non arrotondato.
4. **Le guardie di agosto sono 20/28 mm contro le 15/25 mm di luglio**: le penetrazioni storiche (22,9–24,6 mm) non sono confrontabili con le nostre e non vanno usate come attese.
5. **Il refinement rifiutato di luglio avverte** che migliorare la metrica offline può peggiorare la robustezza. Nessun gate di A3 va formulato come minimizzazione di uno scostamento offline.
6. **Nessuna prova che il proxy sia l'unica causa** dei fallimenti di agosto. È misurato al 66–71% del primo layer e ≈100% della deviazione open-loop, ma il legame causale con i collassi closed-loop di REV4B/D resta un'inferenza, non un esperimento controllato.

---

## 8. DECISIONI CHE SPETTANO A TE

1. **Percorso raccomandato (parent V26) contro alternativa (parent S0D)** per lo stadio A0.
2. **Come tracciare il secondo contratto osservativo** a 25 feature rispetto ai pin `v3_canonical` esistenti.
3. **I pesi relativi dei tre blocchi di A3**: se adottare i rapporti storici 64,7 / 2,9 / 32,4 o preregistrarne altri.
4. **Se autorizzare gli start alternativi a ±0,20 s** come nuova raccolta, dato che richiedono rollout del teacher su start diversi dal nominale.
5. **Se accettare il ritiro di TODO-8** e la riclassificazione dei numeri del §3 Fase A come controfattuale teacher-vs-student.

## 9. TODO propagati

- **TODO-2** — σ = 0,005 placeholder non risolto. *(aperto, ereditato)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3. *(aperto, ereditato)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota a copertura piena. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura come terminatore di REV4E. *(aperto)*
- **~~TODO-8~~** — **RITIRATO** in questo supplemento: si fondava su un'applicazione impropria dell'operatore di luglio.
- **TODO-9** — Le righe di swing ammesse per raggiungibilità falliscono il criterio del 10% da sole (13,5%). *(aperto)*
- **TODO-10** — Nuovo: il proxy autoregressivo è ereditato dalla sorgente V26 al 71,25% del primo layer e presente al 66–67% in ogni candidato 35D, S1A e L20 inclusi. *(nuovo)*
