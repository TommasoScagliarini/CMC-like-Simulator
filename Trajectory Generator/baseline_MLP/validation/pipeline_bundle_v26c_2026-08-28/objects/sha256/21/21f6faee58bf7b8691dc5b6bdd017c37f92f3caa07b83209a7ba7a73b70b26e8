# V26B S1C-0 (rev3w) — probe di fattibilità offline: **REACHABLE**, con correzione della diagnosi precedente

**Token:** `V26B-S1C-FEASIBILITY-PROBE` · **Data:** 2026-08-24 · **Nessun episodio, fit, collection, promozione o modifica production.** Tutto read-only sugli artefatti congelati. A2 resta NON-DEPLOYABLE, σ irrisolta.

## 1. Verdetto
**REACHABLE.** Dai 100 stati iniziali campionati lungo la traccia S0D, il comando ammissibile più favorevole porta la reference servita della caviglia sotto **−0,03** entro la finestra B3 in **100/100** casi; minimo peggiore raggiunto **−0,16802** (5,6× la soglia), migliore −0,60253; stato stazionario con comando sostenuto **−0,700000** esatto. `S1C-1: ALLOWED to be preregistered`.

**Limite di portata dichiarato nel verdetto stesso:** è raggiungibilità del **percorso di reference**, e nient'altro. Non afferma che una sequenza di comandi che ci arriva sia compatibile col mantenimento del passo, né che una policy possa essere addestrata a emetterla.

## 2. Catena di produzione ricostruita dal codice (punto 1) — con una correzione sostanziale
File e digest pinnati: `osim_trj_cmc_like.py` `26458424…`, `env_factory.py`, `rollout_eval.py` `5433bcbc…`, config risolto `a870cc38…`. Ordine **esatto** degli operatori:

1. `env_factory.FlattenClipAction.action` (**env_factory.py:135-137**): reshape e **clip a [−1,+1]**; l'env valida ma non clippa (finding F1).
2. `CMCEnv.step` (**osim_trj_cmc_like.py:1830-1844**) → `_action_to_segment`.
3. **Decode assoluto**: `values[:,j] = low + 0,5·(a+1)·(high−low)` con `absolute_bounds_rad` di default (**:215-220**) knee (−1,5, 0), ankle (−0,7, +0,7) → per la caviglia **q = 0,7·a** esatto.
4. **Limitatore di slew sul target** `_limit_target_slew`, ancorato a `self._last_policy_endpoint` — **target-to-target, esplicitamente NON l'uscita filtrata** (commento **:1866-1869**). Con `segment_duration = 0,01` e `policy_knots = 1`, l'endpoint può muoversi al massimo **0,02 rad per step**.
5. `segment_values = vstack((continuity_anchor, limited))`, dove l'ancora di continuità è la **posizione attualmente servita**.
6. `set_segment` → poiché `pros_ref_model == butterworth3_jerk_limited`, dispatch a `_set_third_order_segment` (**:610-612**).
7. **Modello del terzo ordine jerk-limited** (**:700-830**): `q_command = values[-1]`; 10 sotto-passi da 1 ms; `raw_jerk = wc³(q_cmd−qf) − 2wc²·vf − 2wc·af`; clip del jerk; accelerazione vincolata dall'intersezione di ±55, della banda di jerk e della banda di fattibilità in velocità; velocità clippata a ±3,5; `q_next = qf + vf·dt + ½af·dt² + jerk·dt³/6`.
8. La spline servita (`BPoly.from_derivatives`) alimenta `pros_ankle_angle_served_ref`.
9. SEA e giunto: l'angolo realizzato insegue la reference servita (errore medio |·| < 0,01 rad su entrambe le tracce).

**CORREZIONE:** la catena **non** è «6 Hz secondo ordine + 2 rad/s». Il config v3 congelato seleziona `butterworth3_jerk_limited` con **cutoff 4,0 Hz** (righe 52-53), più limiti governor 3,5 rad/s, 55 rad/s², 2750 rad/s³ (righe 54-59). Il modello a 6 Hz esiste nel codice ma **non è il percorso attivo**. La dicitura «6 Hz» del report rev3v è superata su questo punto. Avevi ragione a chiedere di non assumere.

## 3. Validazione del modello offline (e sua potenza discriminante)
Riproducendo i comandi **registrati** attraverso la mia re-implementazione e confrontando con la `pros_ankle_angle_served_ref` **registrata**: errore massimo post-burn-in **1,038e-04 rad** (media 2,1e-05) su S0D e **1,005e-04** su A2 — due ordini di grandezza sotto la tolleranza 1e-3. L'allineamento corretto **non è stato assunto ma determinato**: risulta **k = 1** (il comando dello step *i* si manifesta nella reference servita della riga *i+1*), con errore 1e-04 contro 2e-02 per k = 0 e k = 2.

**Test discriminante:** rifacendo la stessa validazione con il cutoff **sbagliato** di 6 Hz, la validazione **fallisce**. Il modello non è quindi genericamente plausibile: identifica il parametro reale.

Conferma indipendente del limitatore: il minimo dell'endpoint slew-limited che ho ricostruito, **−0,03139**, coincide con il minimo di `pros_ankle_angle_previous_endpoint` **registrato**, −0,03139.

## 4. Perché i comandi negativi convivevano con reference positiva (punto 4)
Il confronto riga-per-riga del report precedente era **semanticamente invalido, due volte**:

1. **Disallineamento temporale.** `served_ref[i]` non risponde a `cmd[i]`: l'allineamento misurato è di **un passo** e il filtro integra su una finestra ben più lunga (wc = 25,13 rad/s → assestamento ~3/wc ≈ 0,12 s ≈ 12 step). Confrontare le due colonne sulla stessa riga confronta un comando con una reference che non lo ha ancora visto.
2. **Confronto con la variabile sbagliata.** La reference non insegue il comando **grezzo** ma l'**endpoint slew-limitato**. E qui sta il fatto centrale: il comando grezzo spazia in [−0,49494, +0,69632] con **11,8 %** di righe negative, mentre l'endpoint slew-limitato spazia solo in [−0,03139, +0,46409] con **0,8 %** negative. Il limitatore **satura sul 91,4 % degli step**.

Quindi: la policy emette plantarflessione solo in **picchi isolati e rapidamente oscillanti**; il limitatore, che concede 0,02 rad per step, li assorbe e l'endpoint non scende mai. Non c'è alcun blocco che rifiuti il negativo — c'è un vincolo di **velocità di variazione** che una richiesta oscillante non può sfruttare.

## 5. Le tre distinzioni richieste (punto 3)
- **(a) Irraggiungibilità matematica: NO.** Il percorso di reference ha guadagno DC unitario e nessuna restrizione di segno: con comando costante −0,7 la reference converge a **−0,700000**.
- **(b) Irraggiungibilità entro l'orizzonte B3: NO.** Finestra B3 = 0,25 di ciclo = **38 step** (ciclo mediano misurato 1,514 s). Da tutti i 100 stati campionati la soglia è attraversata entro la finestra; dagli stati intermedi servono **17–26 step**. Orizzonti dal campione di riga 150: min −0,0967 @20, −0,4437 @38, −0,6788 @50, −0,7109 @100.
- **(c) Impossibilità di valutare B3: SÌ, e resta aperta.** Il campo `pros_ankle_angle_imitation_target_phase` è **identicamente zero** su entrambe le tracce v3, quindi il criterio B3 non è comunque valutabile su traccia registrata. È indipendente da (a) e (b) e va risolto a parte.

**Nessuna causalità dichiarata oltre ciò che modello e dati dimostrano:** il probe stabilisce cosa il percorso di reference può produrre, non cosa una policy addestrabile produrrà, né se il passo sopravviva a quel comando.

## 6. Che cosa cambia rispetto alla mia raccomandazione precedente
Nel report rev3v avevo raccomandato di trattare la caviglia negativa come **questione di configurazione di produzione prima che di apprendimento**, sulla base del fatto che «una policy comanda già −0,49 senza ottenere nulla al giunto». Il probe mostra che quella lettura era **sbagliata nella parte causale**: il comando −0,49 non è mai diventato endpoint, perché isolato e oscillante, e un comando **sostenuto** avrebbe raggiunto −0,7. **Ritiro quella raccomandazione.** Il requisito di plantarflessione è, sui dati, **affrontabile con l'apprendimento**: serve una policy che emetta una richiesta **sostenuta** nella finestra di spinta, non più profonda. Questo è esattamente ciò che il peso localizzato in fase `w` di S1C-1 mira a produrre — con la differenza, ora nota, che l'obiettivo non è comandare *più negativo* ma comandare *negativo più a lungo*.

Suggerisco quindi, se autorizzerai S1C-1, di considerare in sede di revisione del protocollo se la grandezza da premiare debba essere la **persistenza** del comando negativo piuttosto che la sua ampiezza istantanea — ma non modifico rev3v: la griglia e le soglie restano quelle congelate finché non decidi tu.

## 7. Test
`test_v26b_s1c0_probe.py`: **PASS, 40 check** (statici e numerici). Copertura: pin/tamper di rev3w **e del sorgente env** (una modifica al file di produzione invalida la ricostruzione); 4 token negativi più la guardia del fit futuro; assenza di primitive di episodio/fit; costanti del modulo identiche a quelle congelate in rev3w; identità del decode su a = −1/0/+1 e clip degli out-of-range; limitatore di slew (mai oltre 0,02 rad/step, 35 step per raggiungere −0,7, e **un comando oscillante lascia l'endpoint vicino a zero**); modello di reference (guadagno DC unitario, limiti di velocità e accelerazione rispettati, governor dimostrato attivo); **validazione sulle tracce reali e sua discriminanza** (col cutoff 6 Hz la validazione fallisce); coincidenza fra endpoint ricostruito e registrato; monotonia dell'inviluppo nell'orizzonte; logica del verdetto su fixture per tutti e quattro gli esiti, incluso il limite di portata; caso (c); no-clobber.

Trasparenza: un mio assert era stretto di un pelo (1e-4 contro il valore misurato 1,038e-04) e la formulazione diceva «due ordini di grandezza» dove ne vale uno — corretti entrambi prima della consegna, senza toccare né il modello né le soglie di verdetto.

## 8. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3w_s1c0_probe.json` | `92cf4aaf11dc0698054134c49efaaf34e16f5411579da6499ffae4e203dc2b47` |
| `…/v26b_bridge_2026-08-24/v26b_s1c0_probe.py` | `e776122e4e09517ae5d57594b353983deb14ee1100facc6dc94a3deb32394d40` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1c0_probe.py` | `dc8486aae5acf8664f8f4353d203b679cd296cc799fdac15790b3f9e9cc2e912` |
| `…/2026-08-24_V26B_anchors_r1/s1c0_probe/v26b_s1c0_probe_20260824_205625.json` | `00e6141f4051117ef2425d144e062fd9cb61708a70e18e5d38b56864f67d8d2c` |
| `Trajectory Generator/osim_trj_cmc_like.py` (letto, **non modificato**) | `26458424c44f18fa1dda20b830fa5e7e825c583cc5d10e6d019cb3bd9a0c6d24` |

## 9. Stato
**STOP per la tua revisione.** Verdetto **REACHABLE** con motivazione verificabile e riproducibile; `S1C-1` risulta *ammissibile alla preregistrazione* ma **non è autorizzato** e non è stato eseguito nulla. Resta aperto e indipendente il caso (c): finché il campo di fase B3 è nullo, il gate G5 non è valutabile su traccia registrata, quindi anche con S1C-1 il criterio andrebbe misurato in altro modo o il campo va reso disponibile — decisione tua.
