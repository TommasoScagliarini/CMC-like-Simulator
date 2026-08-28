# V26B S1B (rev3s) — protocollo del **bridge supervisionato ancorato**: tooling, test e dry-run

**Token:** `V26B-S1B-ANCHORED-PROTOCOL` · **Data:** 2026-08-24 · **Stadio autorizzato:** protocollo, tooling, test sintetici, dry-run. **Nessun fit, rollout, collection, DAgger, PPO/critic, sigma sweep o modifica production.** Tutto additivo e content-addressed; S0D, S1A, rev3r e ogni artefatto esistente immutati.

## 1. Che cosa è S1B (punto 8, senza ambiguità)
È un **bridge supervisionato ancorato**: le label target sono il corpus AB06 `u_IK` **già congelato**, le label di ancora sono le **azioni già registrate dell'attore stesso sulla propria traccia già registrata**. **Non è DAgger.** DAgger richiede nuova raccolta on-policy *più* rietichettatura da parte di un teacher degli stati appena visitati: qui non si esegue alcun rollout, non si visita alcuno stato nuovo, nessun teacher produce alcuna label nuova. Per completezza storica: il DAgger dell'11/07 era DAgger vero (il teacher rietichettava gli stati visitati dall'attore); la parte *ancore* del 13/07 non lo era — riusava le azioni registrate del source actor, ed è quella costruzione a essere ripresa qui.

## 2. Punto 1 — source==init dimostrato, e i limiti rispetto al 13/07
La traccia S0D disponibile **è** la sua propria nominale deterministica da 500 step:
- receipt rollout `cbec1a671b7cdf2980881ec4ca33f69534e66d94cbbfa86c7b9cd9c1a39412b7`, stato COMPLETE, 500/500 `episode_time_limit`;
- il `--checkpoint` nel comando congelato è **string-identical** al modulo S0D pinnato; `--action-selection deterministic`, `--seed 123`;
- **prova numerica**: il forward numpy dell'attore S0D sugli obs della traccia riproduce le azioni grezze registrate con **max |Δ| = 2,744e-07** e RMSE 8,337e-08 (omologo di luglio: rmse float32 1,19e-07). Le ancore sono quindi source==init nel senso esatto del 13/07.

**Limiti dichiarati rispetto al 13/07** (nessuno aggirato):
1. **Contenuto unico identico, massa diversa.** Le 16000 ancore di luglio erano **un** episodio da 500 step replicato 32 volte (`np.tile`, `--nominal-repeat 32`): il contenuto *unico* era 500 righe, esattamente quello che abbiamo. La differenza è la **massa** (16000 : 8712 = 1,837), che qui si riproduce con un peso esplicito, mai duplicando righe (rev3 vieta gli split su righe tiled).
2. **Nessun alt-start.** Luglio aveva 8000 righe teacher sugli start −0,20/+0,20 s. Non esiste alcun rollout alt-start di S0D: produrlo è **nuova raccolta** e richiede un token separato.
3. **Nessuna riga di recovery.** Luglio ne aveva 712, stocastiche phase-aligned. Non possiamo produrle: σ è irrisolta e nessun rollout stocastico è autorizzato.
4. **Non-equivalenza fondamentale.** Il target del 13/07 era un teacher **già consistente con l'init** (RMSE aggregato prima 0,019657; shift nominale dopo RMS 0,004175): un raffinamento sotto consistenza. Il nostro `u_IK` **confligge** con l'init di 0,3758 (knee) / 0,3014 (ankle). **S1B non è lo stadio di luglio**: è un passo limitato sotto conflitto.

## 3. Punto 2 — corpus congelato, nessuna nuova raccolta, nessun leakage
Si riusa esattamente lo split congelato di rev3l/n/q: **ruolo task 380 righe** (steps 1–190 ∪ 311–500, label `u_IK` same-time) e **ruolo ancora sulle stesse 380 righe** (label = azioni registrate di S0D). Holdout **201–300 (100 righe) mai visto da nessuno dei due ruoli**, embargo 191–200/301–310 (20 righe) escluso da entrambi. Digest di obs e label registrati nel receipt di dry-run. **Deviazione dichiarata:** a luglio le ancore coprivano l'intero episodio; qui sono ristrette alla maschera di training per tenere pulito l'holdout. È una deviazione **conservativa**, dichiarata e non nascosta.

## 4. Blocker strutturale (punto 10) — il più importante di questo report
**Ancore e target vivono sugli stessi 380 stati.** Con input identici, il minimo per riga di `λ_a·(π−u_S0D)² + λ_t·(π−u_IK)²` è la combinazione convessa

    π* = u_S0D + (1/(1+r))·(u_IK − u_S0D),   r = λ_a/λ_t

quindi **sulle righe di training la loss a due ruoli è analiticamente equivalente a una regressione mono-ruolo su una label mescolata**. Il ruolo di ancora non aggiunge lì nulla che il blend non aggiunga; agisce solo *fuori* dalle righe di training, per generalizzazione. A luglio non era così: gli stati delle ancore erano stati che le righe teacher **non** coprivano (start diversi), quindi i due ruoli vincolavano regioni diverse.

**Conseguenza onesta:** S1B, come è specificabile oggi, è una **interpolazione trust-region fra S0D e u_IK**, non la costruzione a due regioni di luglio. **L'unico rimedio reale** è avere stati ancora-only che il task non copre, cioè rollout deterministici di S0D dagli start −0,20/+0,20 s (la struttura esatta di luglio): è nuova raccolta e richiede un token dedicato, che propongo come `V26B-S0D-ALTSTART-COLLECTION`.

## 5. Punti 3–4 — refit e budget finito, con predizioni analitiche falsificabili
Refit dell'**intera mean network 35D a partire esattamente da S0D** per discesa del gradiente: nessun hard replacement, nessuna copia/fusione/modifica chirurgica di pesi, mai S1A come init. Loss `λ_a·mean_ancora(MSE vs u_S0D) + λ_t·mean_task(MSE vs u_IK)` con medie per-ruolo group-balanced (meccanismo T1R), più clip 1.0 e l'anchor parametrico July 1e-5 verso θ_S0D. Numerica congelata: Adam lr 1e-4, batch 256, seed 2026, algoritmi deterministici, clock ri-azzerato, logstd bit-identica. Ogni candidato è bit-riproducibile.

Gap misurato in questo stadio sulle 500 righe: knee mean 0,375806 / RMS 0,503505 / max 1,431833; ankle 0,301448 / 0,382014 / 1,197913. Baseline S0D vs `u_IK` sull'holdout congelato: **[0,621439, 0,456921]**.

| Cand. | r | epoche | drift medio predetto (knee/ankle) | drift max predetto (knee) | miglioramento predetto |
|---|---|---|---|---|---|
| A1 | 3 | 300 | 0,0940 / 0,0754 | 0,3580 | 25,0 % |
| A2 | 5 | 300 | 0,0626 / 0,0502 | 0,2386 | 16,7 % |
| A3 | 8 | 300 | 0,0418 / 0,0335 | 0,1591 | 11,1 % |
| A4 | 12 | 300 | 0,0289 / 0,0232 | 0,1101 | 7,7 % |
| A5 | 20 | 300 | 0,0179 / 0,0144 | 0,0682 | 4,8 % |
| A6 | 8 | **60** | ≤ A3 (non convergente per costruzione) | ≤ 0,1591 | ≤ 11,1 % |

Sei candidati, ricerca **chiusa**: nessuno può essere aggiunto, tolto, ripesato o rilanciato. **Banda di ammissibilità derivata a priori:** P3 (drift max ≤ 0,25) impone r ≥ 4,727; T (miglioramento ≥ 10 %) impone r ≤ 9 → **r ∈ [4,727, 9]**, stretta ma **non vuota**, dichiarata prima di eseguire. Il punto di riferimento di luglio (r = 1,837) è **escluso per predizione** (drift max 0,505, doppio del limite): è infeasible *qui* proprio perché il nostro target confligge, mentre quello di luglio no. Predizione: sopravvivono A2, A3, A6; A1 viola P3, A4 e A5 violano T. A1/A4/A5 restano nel budget come **test di falsificazione del modello analitico**.

## 6. Punto 5 — selezione offline gerarchica fail-closed
Ordine **I → P → T → D** con corto circuito (i livelli successivi restano `not_evaluated`):
- **I integrità**: source==init, 10 chiavi, clock a zero, invarianza clock bit-identica, logstd bit-identica all'init, save/reload esatto, T1/T2 ≤ 1e-5, no critic.
- **P preservazione sulle ancore**: mean|Δ| ≤ 0,10, RMS ≤ 0,15, max|Δ| ≤ 0,25 per giunto, **più la stessa soglia media per ogni strato discreto** (one-hot FSM, contatto, HS, TO) così che nessuna fase del passo venga sacrificata in silenzio. Motivazione dichiarata a priori: luglio ottenne shift RMS 0,004175 e max 0,031594 **sotto ruoli consistenti**; i nostri limiti sono ~12× più larghi perché chiediamo un cambiamento reale, mentre max ≤ 0,25 tiene ogni singola riga sotto il gap strutturale 0,376, cioè nessuna riga ancora viene convertita nel target IK.
- **T miglioramento target**: RMSE vs `u_IK` sull'holdout congelato ≥ **10 %** meglio del baseline S0D, per giunto. Valore a priori, **dichiarato come scelto e non derivato**. Nota vincolante: il baseline è quello **forward-based** congelato [0,621439, 0,456921] (definizione usata da rev3o/q/r); la variante calcolata sulle azioni registrate coincide entro 1,1e-08 ma **non** va usata.
- **D deriva**: ‖θ−θ_S0D‖² ≤ 0,5 e drift medio azioni ≤ 0,10 per giunto sulle 500 righe. Motivazione: S1A, che **non** cammina, aveva 2,4398 e 0,384/0,289; la trust region sta ben dentro l'unico punto di fallimento misurato.
- **Nessuna metrica offline può dichiarare walking**: la selezione decide solo *chi è ammesso* alla prova closed-loop.

**Aritmetica di ammissibilità, esplicita:** con drift ≤ 0,10 e gap 0,376 il miglioramento massimo ottenibile è ~16 %; richiederne ≥ 10 % lascia una finestra reale ma stretta (≈10–16 %). Se nessun candidato ci cade dentro, il protocollo **fallisce chiuso** e lo si riporta, senza rilassare nulla.

## 7. Punto 6 — gate closed-loop preregistrato (token separato)
`V26B-S1B-NOMINAL-ROLLOUT`, non concesso qui. Candidati valutati nell'ordine **congelato A6, A3, A2, A1, A4, A5** ristretto ai sopravvissuti offline (più preservante per primo), **stop al primo PASS** o a budget esaurito. Un solo lancio per candidato, nessun retry, comando canonico rev3c/e/j, media deterministica, seed 123, start nominale. Gate vincolanti identici a rev3r: 500/500 `episode_time_limit`; phase_timeout stance 0 e swing 0 su tutte le righe; morphology causal failure rows_positive 0 / max 0 / false per riga; hs_cancelled max 0 e final 0; resync max ≤ 1 e final ≤ 1; ≥ 1 ciclo valido; penetrazione ≤ 0,020 m e nessuna terminazione `grf_penetration`. Diagnostici obbligatori: forma e correlazione di Pearson vs prescritto, accordo di segno e rapporto di ampiezza, confronto con **S0D** (500/500, 2 cicli, 13,7 mm), con **S1A** (392/500, 0 cicli, 24,7 mm) e con gli omologhi di luglio, eventi invalidi, HS/TO, `dropped_wait_hs`, `terminal_flushed`, azioni raw/applied, clipping, saturazione, reserve, return, B3 solo nella finestra [0,55, 0,80]. **Nessun candidato e nessun corpus di ancore viene promosso prima di un PASS closed-loop.**

## 8. Punto 7 — σ
σ = 0,005 resta **irrisolta e non operativa**: i rollout S1B usano la media deterministica e non la impostano. Una sweep potrà essere preregistrata solo più tardi, e solo su un attore che cammina.

## 9. Punto 10 — confronto formale con S1A e rischio
S1A: stesso init, stesse 380 righe, **nessun ruolo ancora** → drift misurato 0,384/0,289, shift parametrico 2,4398, gate offline superato (0,134/0,122) e closed-loop 392/500 con **zero cicli** (un toe-off, poi 260 righe di swing senza heel strike). S1B: stesse righe, **con** ancora → drift limitato per costruzione a gap/(1+r) ≤ 0,094 già a r = 3. **Ipotesi falsificabile:** un passo ≤ 0,10 preserva il ciclo che un passo di 0,384 ha distrutto? Non esiste evidenza in nessuna direzione per drift intermedi — è esattamente ciò che S1B mette alla prova. **Rischio dichiarato:** se il cammino si perde già a drift 0,04–0,06, la conclusione è che la direzione IK è localmente incompatibile con il ciclo del passo e la linea IK va chiusa.

**Costo visibile in anticipo:** anche nel caso migliore S1B lascia l'holdout intorno a 0,52–0,56, lontanissimo dal criterio 0,15. Chiudere il gap 0,376 a passi ≤ 0,10 richiede **4–6 iterazioni**, ciascuna con la propria raccolta, il proprio fit e il proprio gate closed-loop, ri-ancorandosi ogni volta sulla traccia nominale completa del nuovo attore che cammina (l'ordinamento di luglio, iterato).

## 10. Test e dry-run
`test_v26b_s1b_protocol.py`: **PASS, 63 check**. Copertura: pin/tamper rev3l→rev3s e guardia sul lettore dell'emendamento; **6 token negativi** più le tre guardie di stadio futuro (`V26B-S1B-FIT`, `V26B-S1B-NOMINAL-ROLLOUT`, `V26B-S0D-ALTSTART-COLLECTION`), ciascuna che nomina il proprio token e non è mai sbloccabile qui; assenza nel modulo di qualunque primitiva di fit (`torch`, `backward()`, `opt.step`, `Adam(`), di rollout (`subprocess`, `rollout_eval`, `Popen`) e di tiling; source==init con **verifica che il controllo è vivo** (una tolleranza impossibile lo fa fallire) e S1A elencato fra le sorgenti vietate; split/leakage 380/100/20 con holdout mai visto da entrambi i ruoli; strati discreti; predizioni closed-form riprodotte per tutti e 6 i candidati; griglia finita e `open_search: false`; soglie immutabili di tutti e quattro i livelli e dei gate closed-loop; gerarchia con **ciascun livello che fallisce isolatamente e corto circuito verificato**, incluso il caso di un singolo strato discreto fuori limite; ordine closed-loop ristretto ai sopravvissuti; classificazione bridge-non-DAgger; **no-clobber** verificato sul meccanismo reale (un secondo dry-run riserva un path nuovo e lascia il primo receipt byte-identico).

Dry-run ufficiale: exit 0, receipt `3044f1ed0ac6b7fadd6badf0b9cdbfb168bb1e25879ca165ef5e842b4db8d155`, con `executed_in_this_stage = {fit: false, rollout: false, collection: false, export: false}`. Nessuna nuova directory in `student/` né in `rollouts/`.

**Trasparenza sugli errori miei, tutti trovati e corretti durante lo sviluppo dei test, nessuno in produzione:** una chiave dell'emendamento citata con il nome sbagliato nel tool; una tolleranza di test a 1e-9 su valori che il contratto arrotonda a 6 decimali; un test di no-clobber che puntava alla funzione sbagliata (`_atomic_fill_reserved` invece di `unique_artifact_path`); e soprattutto una **discrepanza reale scoperta e resa esplicita invece di essere assorbita**: il baseline S0D della catena è calcolato sul *forward*, le ancore usano le *azioni registrate*, e le due definizioni differiscono di 1,1e-08 — ora entrambe registrate, con l'indicazione vincolante di usare la prima.

## 11. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3s_s1b_anchored_protocol.json` | `89e8c227eecb2a80350307d6c0315e0f90e42b37ecae1d75fea30f9da2e067a0` |
| `…/v26b_bridge_2026-08-24/v26b_s1b_protocol.py` | `9dbbabdd45c2233b416ccb8ccd81a7b84f5bb01b9423d6db9ec5dc78a75cc2be` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1b_protocol.py` | `4c8cda930e5e1afbb69a9ef1477bab82189701fc8fccfc158f6463b215138160` |
| `…/2026-08-24_V26B_anchors_r1/s1b_protocol/v26b_s1b_protocol_dryrun_20260824_200634.json` | `3044f1ed0ac6b7fadd6badf0b9cdbfb168bb1e25879ca165ef5e842b4db8d155` |
| `…/2026-08-24_V26B_anchors_r1/s1b_protocol_dryrun_20260824_200634.log` (`REAL_EXIT_CODE=0`) | `8a35dc4105450122caa5cb91df4f8db28261e508893238aab99169c2e0b95c68` |

## 12. Stato e decisione richiesta
**STOP per audit Codex.** Nulla è stato addestrato, lanciato o promosso. Due decisioni sono tue: **(a)** procedere con S1B a singolo start così com'è, sapendo che è un'interpolazione trust-region e non la costruzione di luglio; **(b)** oppure autorizzare prima `V26B-S0D-ALTSTART-COLLECTION` (rollout deterministici di S0D dagli start ±0,20 s) per ottenere stati ancora-only e riprodurre davvero la struttura a due regioni del 13/07 — a mio giudizio l'unica via che rende S1B July-faithful invece che July-ispirato.
