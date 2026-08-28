# V26C J13 — Diagnosi causale del FAIL J12 e progetto della correzione minima

**Data**: 2026-08-27
**Fase**: analisi. **Nessuna collection, fit, rollout, critic warm-up o PPO eseguito.**
**Nessun file J0–J12, produzione, FSM v3, morphology, reward, SEA/C++ o architettura modificato.**
**Esito**: diagnosi + proposta. **Non autorizzo la fase successiva**; la decisione è dell'architetto.

Ogni affermazione è etichettata **[E] evidenza misurata**, **[I] inferenza**, **[H] ipotesi da testare**.

---

## 1. Problema

J12 ha qualificato in closed loop l'attore J11 sulla matrice A–F e ha dato **FAIL: 4/6**. La cella B
(−0.20 s), che con l'attore J8 falliva con 0 cicli, ora passa con 3 cicli — l'obiettivo della
catena è raggiunto. Ma le celle stocastiche **E (seed 124)** e **F (seed 125)**, che con J8
passavano, ora falliscono. Il netto è 5/6 → 4/6.

La domanda: perché, e qual è la correzione minima e fedele al metodo di luglio.

---

## 2. Evidenze misurate

### 2.1 La divergenza è interamente lato attore e comincia allo step 1 [E]

Confronto trace J9R1 (attore J8) vs J12 (attore J11), stessa matrice, stessi semi:

- rumore applicato, rumore unitario, griglia temporale e **osservazione allo step 1**: **bit-identici**
  in tutte e sei le celle;
- `policy_std_diagnostic` = 0.005 su ogni riga di entrambe le run;
- alla **prima** valutazione, su input identico, J11 emette una media diversa:

| cella | Δ knee | Δ ankle |
|---|---|---|
| A/D/E/F (offset nominale) | −0.016038 | +0.002563 |
| **B (−0.20 s)** | **−0.257452** | **−0.206958** |
| C (+0.20 s) | +0.002365 | +0.041184 |

Tutto il resto è amplificazione closed-loop di quel gap iniziale. **Il rumore è scagionato.**

### 2.2 Il modo di fallimento è uno swing che non si chiude [E]

- **F**: toe-off allo step 94 (32 step prima di J8), swing 94–265, heel-strike allo step 266,
  **annullato allo step 282** come `hs_bounce_cancelled` dal contratto v3. **Il clock di swing non
  viene azzerato dalla cancellazione**: allo step 354 `swing_elapsed_s` = 2.606 > 2.6 s → terminazione
  `phase_timeout:swing`.
- **E**: identico — swing 253→498, `swing_elapsed_s` max 2.461 s — salvato da un heel-strike allo
  step 499, **a un solo step dalla ghigliottina**. E non è un fallimento più lieve di F: è lo stesso
  fallimento che ha esaurito l'episodio prima del clock.
- **Nessuna saturazione**: 0 step clippati in tutte e dodici le trace. Il segnale distintivo è il
  **rate** del comando, non l'ampiezza: slew massimo 0.289/0.297 in E/F contro 0.063–0.084 in
  A/B/C/D (**4.4×**).
- **Penetrazione più bassa**, non più alta: F ha 122 step su 354 in contatto contro 323 su 500 di
  J8. **Il piede sta in aria.**

### 2.3 Gli stati di E/F sono fuori dalla distribuzione di training [E]

Il corpus di training ha **2210 righe uniche** su 24713 (il resto è tiling). Distanza standardizzata
al vicino più prossimo:

| cella | media | p95 | max | frazione > 3σ |
|---|---|---|---|---|
| A | 1.21 | 1.70 | 2.97 | 0.000 |
| B | 1.17 | 1.53 | 2.02 | 0.000 |
| C | 1.16 | 1.54 | 2.03 | 0.000 |
| D | 1.13 | 1.36 | 2.00 | 0.000 |
| **E** | **5.70** | **18.00** | **18.55** | **0.410** |
| **F** | **7.56** | **20.79** | **21.96** | **0.551** |

La colonna che esplode è **`phase_swing_elapsed_norm`**: |z| = 21.9 in E, 23.1 in F. Il training
contiene **zero** stati oltre 0.1888 normalizzato; E arriva a 0.9465, F a 0.9985. Il **42 %** degli
step di E e il **55 %** di F sono oltre il massimo mai visto in training.

Sul ginocchio: l'inviluppo di training si ferma a **−1.0090 rad**; E raggiunge **−1.2137**, F
**−1.3898** — 0.20 e 0.38 rad oltre qualunque riga etichettata.

### 2.4 Il blocco recovery di J7 non aggiunge quasi nessuna copertura [E]

Questo è il fatto centrale. Le tre seed recovery partono **dallo stesso istante** e sono prefissi
della **stessa** traiettoria nominale perturbata a σ 0.005:

| seed | righe | span temporale | knee min |
|---|---|---|---|
| 123 | 429 | 4.28 s | −0.993254 |
| 124 | 273 | 2.72 s | −0.989094 |
| 125 | **11** | **0.10 s** | −0.194414 |

Il minimo del ginocchio passa da −0.9924 (nominale) a −0.9933 (recovery): **+0.0009 rad di nuova
copertura per 713 righe**. Le colonne che il blocco recovery allarga sono quelle di *controller
memory*, non gli angoli articolari.

**E la ragione è la regola di troncamento.** `truncate_before_discrete_mismatch` taglia al **primo**
disallineamento su 8 colonne discrete (contatto, HS, TO, one-hot FSM, expected HS/TO), senza
tolleranza, finestra o lunghezza minima. Prefissi: 429 / 273 / 11, con primo mismatch a 430 / 274 /
**12**. Seed 125 contribuisce 11 righe pur essendo la run più pulita in penetrazione: il prefisso
finisce quando un evento discreto scatta uno step prima o dopo, il che è **casualità di bordo**, non
qualità della traiettoria.

**Conseguenza [I]: il corpus contiene solo stati precedenti alla divergenza. Per costruzione non
può contenere gli stati in cui servirebbe recuperare.**

### 2.5 `phase_fsm_wait_hs` è a zero in tutte le 16713 righe J7 [E]

Quindi la colonna 17 di J8 è **esattamente invariata rispetto a J2** (‖J8−J2‖ = 0.000000): non ha
ricevuto un solo gradiente in 167 epoche. J11 la muove di **0.956386**, il movimento per colonna più
grande dei 35 ingressi, sulla forza di **20 righe** della cella B. Quando l'FSM entra in `wait_hs`,
J8 agisce con pesi del parent mai addestrati.

### 2.6 J11 non è anomalo: riproduce il punto operativo di luglio [E]

| | clock | esogene 2..24 | **controller 25..34** | ctrl/esogene |
|---|---|---|---|---|
| J2 (base doppiamente mascherata) | 0 | 7.669 | **0.000** | 0.000 |
| J8 (early stop a 167) | 0 | 7.662 | **0.151** | 0.020 |
| **J11** (400 epoche) | 0 | 7.872 | **1.096** | **0.139** |
| **attore July selezionato** | 0 | 7.438 | **1.079** | **0.145** |

Colonna per colonna July e J11 coincidono nel profilo e nell'ordine; J8 è uniformemente ~10× sotto.
Profilo di training: July 19770 righe / 400 epoche / best 392; **J11 19770 / 400 / best 397**; J8
13370 / 167 / best 107, con early stop. **J11 ha fatto 5.48× più passi di optimizer di J8.**

**È J8 l'anomalia, non J11.** J11 si è mosso 10.3× più lontano dal parent (‖·‖_F 2.616 vs 0.254), e
J8 è talmente vicino a J2 che sull'aggregato è *marginalmente peggiore* del parent intatto (0.0308669
vs 0.0308652) e **peggiore di J2 anche sulle celle B e C**.

### 2.7 Le colonne controller sono retroazione pura [E]

Verificato nel codice (`osim_trj_cmc_like.py:3059-3073`): `_previous_endpoint` = `_last_policy_endpoint`,
`_sea_u` = `_last_u_sea`, `_served_ref*` = il riferimento servito attraverso il limitatore. Tutte e
dieci sono funzioni dell'azione precedente della policy stessa.

Guadagno del Jacobiano ristretto (differenze finite, 200 stati per blocco):

| | controller | esogene |
|---|---|---|
| J8 | 0.093 | 1.029 |
| J11 | 0.518 (**5.6×**) | 0.906 (**0.88×**) |

Il Jacobiano **totale** è però quasi invariato (+1.2 %): i due effetti si cancellano. J11 ha spostato
autorità dal sensing esogeno alla propria retroazione — **come luglio**.

### 2.8 `hs_bounce_cancelled` su seed 125 precede questa catena [E]

Negli artefatti F0 del **2026-08-22**, sotto FSM v3 canonica, la cancellazione è endemica in tutta la
famiglia B0820, e **seed 125 al nominale è il caso peggiore documentato**: 8 cancellazioni per
`B0820_H0`, 4 per `B0820_V3_BEST`, contro 0–2 degli altri semi. Gli attori July mostrano 0.

**La cella F era già la configurazione più fragile della lineage, cinque giorni prima di J8 e J11.**

### 2.9 Le etichette teacher per gli stati J12 esistono già [E]

`prescribed_teacher_action` dipende **solo dal tempo** (spline IK + config statica), non dallo stato.
E le griglie temporali sono **bit-identiche**:

- celle A/D/E/F (offset nominale) → `time_before` bit-identico a `J1.times`
- cella B → bit-identico a `J10R1 cell B.times`; cella C → idem per C

Le trace J12 registrano `actor_observation_vector_before` a 35D per ogni step. **Stati ed etichette
sono entrambi già artefatti committati.**

---

## 3. Confronto con luglio

| | luglio | qui |
|---|---|---|
| etichetta teacher **same-step** (`teacher_actions[step-1]`) | **sì** | proposta ✓ fedele |
| troncamento al primo mismatch nel **DAgger** | **NO** — `target_domain_dagger.py` non lo chiama mai; ingeriva l'episodio intero (68/45/356 righe) | ✗ **non sarebbe fedele** |
| troncamento nel percorso **recovery/noise** | sì, `target_domain_noise_adaptation.py`, prefissi 119/118/119 | J7 lo usa: 429/273/11 |
| riferimento del confronto | trace perturbata vs **trace nominale dello stesso attore** | J7 usa J3, stesso schema ✓ |
| init del refit | DAgger 11/07: **incatenato** student→student; markov 13/07: **fresh dal base** ogni variante | da decidere |
| pesi sulle righe recovery | **nessuno** — solo molteplicità via `np.tile`, loss non pesata | ✓ |
| monotonia | **NO**: 68 → 45 → 356 → 221 | rischio da dichiarare |
| fragilità stocastica σ ≥ 0.005 risolta con dati recovery | **TENTATO E RESPINTO** (12/07). La correzione fu rimuovere il controller state (39→25 feature) + BC fresco | la stessa strada è ora fuori scope |

**Il punto più importante.** Il TODO vincolante di luglio del 14/07, mai eseguito, dice verbatim:

> «raccogliere dati recovery realmente event-aligned e indipendenti, **includendo la regione
> successiva al mismatch FSM invece di sole interpolazioni pre-evento**»

Luglio aveva **già diagnosticato** che il troncamento lascia scoperta proprio la regione che serve, e
si era impegnato a colmarla. Non l'ha mai fatto. È esattamente il buco che J12 ha reso visibile.

---

## 4. Causa sintetica

**[E+I]** J11 riproduce il punto operativo di luglio e migliora enormemente le celle per cui è stato
fittato (B: 7.8× meglio di J8; C: 2.7×). Muovendosi 10.3× più lontano dal parent di quanto avesse
fatto J8, ha spostato dove il loop chiuso finisce sotto rumore. Su due semi su tre il piede non
torna a contatto, lo swing si allunga e — nel caso di F, la configurazione già documentata come la
più fragile della lineage — una cancellazione di heel-strike che **non azzera il clock di swing**
converte un inciampo recuperabile in un timeout.

**Il motivo per cui l'attore non recupera è che non ha mai visto quegli stati.** Il 42–55 % degli
step di E/F sta oltre il massimo di training su `phase_swing_elapsed_norm`, e il corpus **non può**
contenerli: l'unico meccanismo che aggiunge righe fuori-nominale — il blocco recovery — tronca al
primo mismatch discreto, cioè **prima** che la divergenza cominci.

In una riga: **non è un difetto di J11, è un buco di copertura creato dalla regola di troncamento —
lo stesso buco che luglio aveva identificato e si era impegnato a colmare.**

Il gate offline non poteva vederlo: misura RMSE open-loop sul solo supporto di training, dove la
retroazione è spezzata e la regione di swing prolungato non esiste.

---

## 5. Ipotesi che ho verificato e **scartato**

| ipotesi | esito |
|---|---|
| «J11 amplifica il rumore» | **REFUTATA** [E]. Jacobiano totale +1.2 %. Il rumore è bit-identico. |
| «Il guadagno di retroazione di J11 è anomalo» | **REFUTATA** [E]. 1.096 contro 1.079 di luglio; è J8 (0.151) l'outlier. |
| «Il peso 8 sulle celle B/C è la causa» | **NON DIMOSTRATA** [E]. Il conteggio righe di J11 è quasi identico a quello di luglio (19770 training). Confondente maggiore e non separabile: J11 ha fatto **5.48×** più passi di optimizer di J8, perché J8 si è fermato presto e July/J11 no. **Non attribuisco la causa al peso 8.** |
| «Il rumore o il seme sono diversi» | **REFUTATA** [E]. Bit-identici. |
| «È saturazione del comando» | **REFUTATA** [E]. 0 step clippati ovunque. |
| «È penetrazione» | **REFUTATA** [E]. In E/F è *più bassa*; zero campioni ≥ 25 mm in tutta la matrice. |
| «Tornare a J8» | **Sconsigliata** [E]. J8 è peggiore del parent J2 intatto su B, C e sull'aggregato; non ha mai addestrato la colonna `wait_hs`. Passa E/F per margine, non per competenza. |

---

## 6. Percorso primario raccomandato

**DAgger sugli stati visitati dallo student J11, includendo la regione post-mismatch, con etichette
teacher same-step e senza troncamento.**

### 6.1 Perché questo e non altro

- Colma **esattamente** il buco misurato (§2.3, §2.4).
- L'etichetta same-step è **fedele a luglio** (§3).
- L'assenza di troncamento è **fedele al DAgger di luglio** (che non troncava) ed è **il TODO
  vincolante che luglio non ha mai eseguito** (§3).
- Non tocca architettura, σ, FSM, morphology, reward, SEA o produzione.

### 6.2 Celle e semi — con un vincolo che ho trovato, non inventato

Il protocollo architetturale del **2026-08-23** (`f1_s1_protocollo_tooling_dryrun.md`, rev 3, decisione
vincolante 4, recepita da una correzione bloccante d'audit) stabilisce:

> «seed 123–124 → raccolta/fit; **seed 125 = validation closed-loop held-out**, mai nel fit,
> nell'aggregazione o nella selezione dei batch; 126–128 sigillati»

Quel documento è marcato «PROPOSTA NON ANCORA AUTORIZZATA» — **segnalo che la sua forza vincolante è
una decisione tua, non mia**. Ma finché non lo revochi, mi ci attengo:

| cella | seme | uso proposto |
|---|---|---|
| A, B, C | 123 det | righe di ancoraggio (i tre start) |
| D | 123 stoch | righe di ancoraggio |
| **E** | **124 stoch** | **sorgente correttiva primaria** — 500 righe, ~246 in swing prolungato |
| **F** | **125 stoch** | **ESCLUSA da fit ed etichette.** Resta gate held-out, solo misurata |

**E esibisce lo stesso identico fallimento di F** (§2.2), quindi la copertura correttiva non richiede
F. Questo è ciò che rende il vincolo di seed compatibile con la correzione.

### 6.3 Composizione del dataset proposto

```
D = [ J7 intero            16713 righe ]   ancore esistenti, immutate
  + [ B×8, C×8              8000 righe ]   come in J11, immutate
  + [ E, righe uniche        500 righe ]   NUOVO: stati visitati dallo student J11
                                            etichette = J1.actions[step-1], same-step
```

- **Nessun repeat sulle righe nuove** (repeat = 1) finché non c'è evidenza che ne giustifichi uno.
  Luglio non pesava le righe correttive: l'influenza veniva solo dalla molteplicità, e il fattore 8
  non ha razionale documentato. **Non replico un peso non giustificato.**
- Nessun dedup, nessun bilanciamento: luglio non ne faceva.
- Clock proiettato a zero esatto, come J7/J11.

### 6.4 Fresh da J2, non da J11

Luglio fece entrambe le cose: il DAgger dell'11/07 incatenava student→student; il **markov del 13/07 —
che è l'antenato metodologico di questa lineage — ripartiva fresh dal base per ogni variante**.

Raccomando **fresh da J2**, per tre ragioni: (a) precedente markov; (b) J7/J8/J11 sono tutti fratelli
fresh-da-J2 e il confronto resta interpretabile; (c) il DAgger incatenato di luglio fu
**non-monotono** (68→45→356→221), e incatenare da J11 comporterebbe l'errore.

Iperparametri: **invariati** (seed 123, 400 epoche, batch 128, lr 5e-5, patience 60, clip 1.0,
logstd 0.0, anchor 0.01). Nessun tuning in risposta a un fallimento.

### 6.5 Il rischio centrale, dichiarato

**[H]** L'etichetta same-step in uno stato con fase divergente può essere *phase-invalid*. Luglio lo
misurò: 76.27 % delle righe recovery del markov erano phase-misaligned, e la sua risposta fu proprio
il troncamento. Non troncando, si accetta quel rischio.

Argomento a favore [I]: in uno swing *ritardato*, `q_ref(t)` è più avanti nel ciclo — l'etichetta dice
«dovresti già essere in stance», che spinge il piede verso il contatto. È plausibilmente la correzione
giusta. Ma è un'inferenza, non un'evidenza, ed è **il punto che il gate closed-loop deve arbitrare**.

Contro-evidenza da non ignorare: il DAgger di luglio fu non-monotono, e il tentativo di risolvere la
fragilità σ ≥ 0.005 con dati recovery fu **respinto**.

**Propongo di misurare la frazione phase-invalid come diagnostico preregistrato**, non come gate — non
inventando una soglia.

---

## 7. Alternative scartate, con la ragione

| alternativa | perché scartata |
|---|---|
| **Troncare al primo mismatch** anche qui | Provatamente incapace di colmare il buco: il buco *è* post-mismatch (§2.4). E non è fedele al DAgger di luglio (§3). |
| **Ridurre i passi di optimizer / early stop come J8** | Allontanerebbe da luglio, non ci avvicinerebbe: July e J11 fanno 400 epoche; J8 è l'anomalia. E J8 è peggiore del parent su B/C. |
| **Ridurre il guadagno del blocco controller** | Sarebbe un vincolo architetturale nuovo, fuori scope, e J11 è al punto operativo di luglio. La correzione di luglio (rimuovere il controller state, 39→25) è esplicitamente fuori dai vincoli attuali. |
| **Cambiare σ** | Escluso dall'architetto e sarebbe una scorciatoia: σ 0.005 è il punto già verificato. |
| **Usare la cella F come dato di training** | Violerebbe il seed hygiene del protocollo 23/08 (seed 125 held-out) e il TODO di luglio sulla separazione dei semi. |
| **Tornare a J8** | §5. |
| **Ripetere le righe nuove ×8** | Nessuna evidenza lo giustifica; il fattore 8 di luglio non ha razionale documentato. |
| **Cambiare FSM v3 perché la cancellazione non azzera il clock di swing** | **Esplicitamente vietato.** Lo registro come osservazione (§8), non come proposta. |

---

## 8. Osservazione che non propongo di correggere

**[E]** In F, l'heel-strike dello step 266 viene annullato allo step 282 come `hs_bounce_cancelled`, e
`swing_elapsed_s` **continua ad accumulare dallo step 94**: 2.606 s allo step 354. Una cancellazione
converte quindi un inciampo recuperabile in un timeout terminale.

**Se questo comportamento sia intenzionale nel contratto v3 non l'ho potuto stabilire dalle trace.**
Non propongo di modificarlo: FSM v3 è fuori scope. Lo segnalo perché, se non fosse intenzionale,
sarebbe una causa concorrente indipendente dall'attore — e nessuna quantità di dati DAgger la
risolverebbe.

---

## 9. I trace J12 sono riusabili per l'etichettatura?

**Sì per gli stati, sì per le etichette — con due riserve.**

**Tecnicamente riusabili [E]:**
- le trace registrano `actor_observation_vector_before` a 35D per ogni step;
- le griglie temporali sono **bit-identiche** a J1 (celle nominali) e a J10R1 (B/C);
- il teacher prescritto è funzione **solo del tempo**, quindi l'etichetta per lo step *k* della cella
  E **è** `J1.actions[k]`, un artefatto già committato e pinnato.

**Non serve alcuna nuova collection per ottenere stati o etichette.**

**Le due riserve:**

1. **Cella F esclusa** — seed 125 è il gate held-out (§6.2). Le sue trace restano misurate, mai
   etichettate.
2. **Ruolo dell'artefatto.** Il leaf J12 è evidenza *transazionale di qualifica*. Usarlo come input di
   training confonde i due ruoli e rende il leaf sia record che dataset. **Raccomando di non leggere
   dal leaf J12 in un fit**, ma di materializzare le righe in un leaf di dataset separato, additivo,
   che *pinna* il leaf J12 come sorgente e ricopia le righe con i propri hash — esattamente come J7
   fece con J1/J3/J6.

Quindi: **nessun nuovo rollout è necessario**; serve però uno stadio di materializzazione dedicato.
Questa è una differenza sostanziale rispetto a «serve una nuova collection», e riduce il costo e il
rischio della correzione.

---

## 10. Readiness, acceptance e matrice successiva — proposta

**Non autorizzo nulla di tutto questo.** È una proposta per la tua decisione.

**Stadio 1 — materializzazione del dataset** (nessun fit, nessun rollout)
- pinna: leaf J12 (receipt, `commit_verification pass`, trace cella E), J7, celle B/C J10R1, parent J2;
- verifica che il leaf J12 sia valido (commit_verification true, nessun marker);
- estrae le 500 righe della cella E, proietta il clock a zero esatto, allinea `J1.actions[step-1]`
  provando l'identità temporale a 1e-9 su ogni riga;
- **rifiuta fail-closed** qualsiasi riga proveniente dal seed 125;
- registra come diagnostici: frazione di righe con FSM one-hot diverso dal nominale, distribuzione di
  `phase_swing_elapsed_norm`, inviluppo del ginocchio;
- leaf immutabile, commit atomico, verifica post-commit, marker born-invalid — come J10R1/J11/J12.

**Stadio 2 — fit** (fresh da J2, iperparametri invariati)
- gate offline vincolanti, tutti **disuguaglianze strette contro baseline misurate**, nessuna soglia
  nuova: integrità (chiavi/shape/dtype, clock bit-zero, alias, logstd bit-identica a J2, nessun
  critic, input invariati); RMSE dopo < prima su aggregato, recovery 713, B unique, C unique **e sul
  nuovo blocco E**; dieci norme controller > 0.
- diagnostici non vincolanti: nominal RMSE e shift, clipping, validation MSE, **frazione phase-invalid
  delle nuove etichette**, norme del blocco controller a confronto con J11 e con luglio.

**Stadio 3 — riqualifica closed-loop**
- **la stessa matrice A–F**, gate identici, nessuna soglia nuova;
- F (seed 125) resta genuinamente held-out: non ha generato dati;
- criterio: 6/6 comportamentali e 6/6 telemetria valida, come J12;
- **niente retry mascherati**: un solo tentativo per stadio, leaf transazionale unico, nessuna
  variante, evidenza preservata in ogni esito.

**Regola di arresto che raccomando di preregistrare**: se lo Stadio 3 dà di nuovo FAIL, **non
aggregare un secondo round senza una nuova decisione architetturale**. Il DAgger di luglio fu
non-monotono, e iterare senza un criterio d'arresto è il modo in cui luglio bruciò tre round.

---

## 11. File letti e creati

**Letti** (nessuno modificato): trace e receipt J9R1 e J12 (12 trace, ~51 MB ciascuna); dataset e
receipt J7; probe J6 seed 123/124/125; dataset J1; celle B/C J10R1; moduli J2, J8, J11 e checkpoint
July `rl_module_target_adapted`; aggregato J11; `osim_trj_cmc_like.py`, `target_domain_imitation.py`,
`target_domain_markov_adaptation.py`, `target_domain_noise_adaptation.py`, `target_domain_dagger.py`,
`rollout_eval.py`, `exploration_noise.py`, `prosthetic_phase_fsm.py`; `v26c_j6/j7/j8/j11/j12*.py`;
artefatti F0 del 22/08; report July 11–15/07 e report agosto 22–27/08.

**Creati**: soltanto questo report.

**Verifiche**: `git status` invariato oltre ai tre file già dirty a inizio sessione; nessun leaf,
lock, staging o sentinella creato; nessun runner di stadio eseguito.

---

## 12. Prossimi gate

1. **Tua revisione** di questa diagnosi, e confronto con la tua.
2. Tua decisione su: (a) il vincolo seed 125 held-out del protocollo 23/08 — se è in forza; (b) fresh
   da J2 vs da J11; (c) se accettare il rischio phase-invalid senza troncamento; (d) se la mancata
   riazzeratura del clock di swing su cancellazione vada indagata separatamente.
3. Solo dopo: readiness dello Stadio 1, con la tua autorizzazione esplicita.

---

## 13. TODO propagati

- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri, non toccati.
- **Epic generalizzazione multi-modello** — APERTO; tutto qui è ancora AB06, un trial, tre start.
- **Critic warm-up** — non avviato, non autorizzato. Precondizione non soddisfatta.
- **PPO** — non autorizzato.
- **TODO vincolante di luglio del 14/07** — «includere la regione successiva al mismatch FSM»: mai
  eseguito, ed è la sostanza della correzione proposta.
- **Fattore repeat 8 senza razionale** — non replicato sulle righe nuove; resta un parametro non
  giustificato di J11.
- **Clock di swing non azzerato su `hs_bounce_cancelled`** — NUOVO, osservato, non indagato, non
  proposto per modifica.
- **Seed 125 come caso peggiore documentato dal 22/08** — NUOVO in questo contesto: la cella F era già
  la configurazione più fragile della lineage prima di J8 e J11.
- **`phase_fsm_wait_hs` addestrata su 20 righe** — la colonna con il movimento più grande di J11 poggia
  su 20 righe uniche. Fragile per costruzione; non ho evidenza che sia un problema, ma va sorvegliata.
- **Protocollo 23/08 non autorizzato** — la sua forza vincolante è una decisione dell'architetto.

---

**Fermo per la tua revisione. Non autorizzo la fase successiva.**
