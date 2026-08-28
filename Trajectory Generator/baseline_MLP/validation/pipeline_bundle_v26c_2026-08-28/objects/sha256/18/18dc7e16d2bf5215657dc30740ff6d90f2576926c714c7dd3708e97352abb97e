# V26B B1R2-A — revisione indipendente, esito NO-GO e proposta del prossimo esperimento

**Data**: 2026-08-25
**Ruolo**: revisione indipendente di Claude (Opus 5, xhigh). Codex resta architetto e gate owner.
**Esito B1R2-A**: **NO-GO offline**, 6 gate su 9 falliti. Nessun candidato promosso.
**Nessun fit, rollout, collection o DAgger eseguito in questa revisione.** Solo analisi read-only degli artefatti esistenti.

Parent operativo: **V26 agosto imitation → B0 35D masked**. Luglio usato **solo** come riferimento metodologico.
Invarianti rispettate: unico actor 35D, hard mask 0:2 e 25:35 (effective 23D), assenza voluta di controller/controlaterale/gait clock, nessun cambio FSM/morphology/contact/reward/C++/SEA. Sigma non misurata qui.

---

## 1. Integrità degli artefatti — tutto verificato

| artefatto | SHA-256 | esito |
|---|---|---|
| B0 `module_state.pkl` | `aa7ea0fa1bbef8bb6ef2a33ee8ebe5defeeb4959148a589b81ff994cf291171f` | intatto |
| B1 receipt | `e7328ac2eb49787e905ef0919f06c540bdd2ce4ec30d290e711f8b0af42f53b1` | intatto |
| B1 `module_state.pkl` | `778ff748bd33ca658b62aae1b091be7b702242e9faabb8e0192daf7453e80a5b` | intatto |
| B1R1 receipt | `6a604ada500f6f51594500f596ba7a7797cd1350a110b9d0fe1807ee4d2cadbb` | intatto |
| B1R1 `module_state.pkl` | `9ffcdceb7f90f12cbcca0c151f8b667e3003ba4f48a124e630582d76cf0bd980` | intatto |
| B1R2 receipt pre-revisione | `ebf0c8e88762c3bb826754753def2d1980ff0c43cfdbd2108062221cf1cb114f` | intatto |
| B1R2 receipt rev1 | `d0f1e5155d8273651f6055ca9a47601ad5f265720db2d8aca295dabefc84ff4c` | intatto |
| B1R2-A receipt | `a4ebd8d6f023c8749212fe5c24c7fae36675ef2335c3b7a097981494b9262f4e` | nuovo |

**Nessuna promozione, nessun rollout, nessun DAgger.** `candidates/B1R2A_BASE35_BUDGET1200` non esiste (né la sua staging). Gli artefatti B1R2-A stanno in `diagnostics/b1r2a/`, manifest `deployable: false`, `offline_verdict: NO-GO`. Il codice dell'esecutore non contiene alcun identificatore di rollout/collection/PPO/DAgger (verificato per token, non per sottostringa). Produzione, FSM, contatto, reward e `tools/` invariati.

Identità verificata sul receipt: `max |pooled − w·M| = 3,5e−18`, cioè la curva pooled è esattamente la media pesata per righe delle 11 curve.

## 2. Gate B1R2-A a e\*=237

| gate | soglia | osservato | esito |
|---|---|---|---|
| invarianti integrità | — | tutte vere | **PASS** |
| equivalenza funzionale 25D | bit-exact | bit-identica | **PASS** |
| ogni fold migliora su B0 | — | 11/11 | **PASS** |
| RMSE heldout per fold | ≤0,05 | 5 fold sopra soglia | **FAIL** |
| RMSE pooled validation | ≤0,03 | **0,04805** | **FAIL** |
| RMSE finale aggregata | ≤0,02 | **0,03437** | **FAIL** |
| RMSE finale per giunto | ≤0,03 | knee **0,03936** · ankle 0,02853 | **FAIL** |
| max_abs finale per giunto | ≤0,15 | knee **0,18824** · ankle 0,12657 | **FAIL** |
| ricostruzione WAIT | 0,02/0,03/0,15 | agg 0,02703 · knee max **0,18824** | **FAIL** |

Fold heldout: 0,05439 · 0,03114 · 0,03953 · 0,02392 · 0,05343 · 0,03715 · 0,02450 · **0,06168** · 0,02828 · **0,06680** · 0,05892.

## 3. Ipotesi budget: **falsificata**, con la precisazione dell'architetto

I tuoi ricalcoli coincidono con i miei al bit: e\*=237 (RMSE 0,0480536), minimo globale della curva completa a **epoca 1171** (MSE 0,001697248, RMSE 0,0411977), e400 0,0474109 · e600 0,0462016 · e800 0,0434075.

**Non affermo che la curva completa non migliori.** Migliora, in modo lento e monotono nel livello mediano (mediana pooled per finestra: 0,003810 su 1–100 → 0,002521 su 276–400 → 0,002063 su 600–800 → 0,001916 su 1100–1200), e il minimo a 1171 è un bacino genuino, non un picco di rumore: nell'intorno ±25 la mediana è 0,001911 e il **massimo** locale (0,002085) resta sotto il **minimo** locale intorno a 237 (0,002309).

Perché allora e\*=237? Due cause distinte, entrambe quantificate:

1. **La regola non guarda.** Con patience 60 la scansione si arresta a 297. La prima epoca che batte davvero pooled(237) è la **306**: mancata per 9 epoche. Le epoche 298–1200 non vengono mai esaminate, per costruzione.
2. **Anche guardando, i minimi individuali non si allineano.** Gli argmin per fold sono dispersi fra 367 e 1189; al minimo di un fold gli altri sono lontani dal proprio. Il rumore epoca-per-epoca è grande rispetto al miglioramento locale: intorno a 237±25 la curva pooled oscilla fra 0,002309 e 0,003336 (spread 44%).

## 4. Patience/selection **esclusa** come prossimo fattore causale

Questo è il punto che chiudi tu stesso e che confermo con i controfattuali:

| scenario | pooled RMSE | gate 0,03 |
|---|---|---|
| regola luglio, e\*=237 | 0,04805 | FAIL |
| **miglior epoca singola su 1200 (e=1171)** | **0,04120** | **FAIL** |
| **oracle per-fold (ogni fold al proprio best, irraggiungibile)** | **0,03599** | **FAIL** |
| tutti i fold a 0,03 | 0,03000 | FAIL |

Nessuna regola di selezione, per quanto perfetta, passa il gate pooled. Persino l'oracle per-fold — che nessuna regola causale può realizzare, perché richiede 11 epoche diverse simultaneamente — resta **1,20× la soglia in RMSE (1,44× in MSE)**.

**Conclusione**: patience/selection è igiene necessaria più avanti, **non** il fattore causale. Va corretta, ma non prima e non da sola.

Nota di merito: il gate pooled sale da 0,03560 (B1R1, 6 fold, 910 righe) a 0,04805 (B1R2-A, 11 fold, 1313 righe). Non è un peggioramento del modello: è la stessa grandezza misurata su tutte le righe non-WAIT invece che sul 69% più facile. La ricostruzione finale è invariata (B1R1 0,03433 → B1R2-A 0,03437) pur con e\* diversi (358 vs 237).

## 5. Localizzazione dell'errore

**Per fold** (contributo alla MSE pooled a e\*=237):

| fold | tipo | anchor | n | quota MSE | RMSE a e\* | RMSE al proprio best |
|---|---|---|---|---|---|---|
| 9 | ciclo completo | plus020 testa | 144 | **21,2%** | 0,06680 | **0,05947 FAIL** |
| 7 | ciclo completo | nominal testa | 164 | **20,6%** | 0,06168 | **0,05413 FAIL** |
| 0 | ciclo completo | minus020 c1 | 148 | 14,4% | 0,05439 | 0,03593 PASS |
| 4 | ciclo completo | plus020 c1 | 151 | 14,2% | 0,05343 | 0,03792 PASS |
| 2 | ciclo completo | nominal c1 | 151 | 7,8% | 0,03953 | 0,03331 PASS |
| 5 | ciclo completo | plus020 c2 | 153 | 7,0% | 0,03715 | 0,01394 PASS |
| 10 | coda | plus020 | 52 | 6,0% | 0,05892 | 0,02038 PASS |
| 1 | ciclo completo | minus020 c2 | 154 | 4,9% | 0,03114 | 0,01624 PASS |
| 3 | ciclo completo | nominal c2 | 153 | 2,9% | 0,02392 | 0,01578 PASS |
| 8 | coda | nominal | 32 | 0,8% | 0,02828 | 0,01253 PASS |
| 6 | coda | minus020 | 11 | 0,2% | 0,02450 | 0,00484 PASS |

I fold si separano nettamente in due classi:
- **Fallimenti da selezione/rumore** (0, 4, 10): superano il gate al proprio best, lo mancano a e\*=237.
- **Fallimenti strutturali** (7, 9, le due teste): mancano il gate **a qualunque epoca**, anche al proprio minimo su 1200. Sono 308 righe (23,5% dell'held-out) e producono il **41,8%** della MSE pooled.

**Per regione e posizione nell'episodio** (fit finale, tutte le 1500 righe):

| gruppo | n | agg RMSE | knee rmse | ankle rmse | max abs |
|---|---|---|---|---|---|
| WAIT | 187 | 0,02703 | 0,02873 | 0,02521 | **0,18824** |
| **teste** | 308 | **0,04839** | 0,05816 | 0,03607 | 0,14591 |
| ciclo 1 | 450 | 0,03664 | 0,04121 | 0,03142 | 0,11088 |
| ciclo 2 | 460 | **0,02231** | 0,02323 | 0,02135 | 0,07684 |
| code | 95 | 0,02929 | 0,03488 | 0,02233 | 0,09143 |

L'errore **decresce monotonicamente col tempo trascorso dall'inizio dell'episodio**, in tutti e tre gli anchor: testa > ciclo 1 > ciclo 2. Il ginocchio è il giunto dominante ovunque (knee/ankle ≈ 1,3–1,6×). Il `max_abs` che fa fallire il gate WAIT è una singola coppia di righe, minus020 locali 13–14, cioè l'inizio assoluto dell'episodio; è peggiorato rispetto a B1R1 (0,12686 → 0,18824).

## 6. Il reperto strutturale, e i suoi limiti

**Il regime osservativo.** Prima che lo stimatore online completi un ciclo, tre delle 23 colonne effettive sono costanti: `online_left_gait_phase_sin`=0, `..._cos`=1, `..._cycle_duration_s`=0. Il regime copre 945 righe (63%): WAIT 187 + teste 308 + **ciclo 1 450** (il ciclo 1 di ogni anchor è al 100% nel regime, il ciclo 2 allo 0%). Dentro il regime la dimensione effettiva è **20, non 23**.

**Il conflitto.** A parità di firma FSM (SWING/credit-0,5) e dentro lo stesso regime, testa-swing (188 righe) e ciclo1-swing (262) **si sovrappongono**: nessuna colonna li separa (gap massimo 0,277 sd, zero colonne oltre 0,5 sd), e le righe di testa distano dalle righe di ciclo1 quanto le righe di ciclo1 distano fra loro (1,15×). Eppure i target divergono:

| coppie (distanza < 0,15) | mediana \|Δtarget\| |
|---|---|
| testa × ciclo1 | **0,16785** (dentro un anchor) · **0,16366** (fra anchor) |
| ciclo1 × ciclo1 | 0,04487 |
| testa × testa | ~0,00–0,026 |
| ciclo2 × ciclo2 | 0,028–0,039 |

Estrapolando \|Δtarget\| verso distanza nulla sulle coppie sotto 0,15: intercetta **0,196** per testa×ciclo1, contro 0,024 dentro ciclo1 e ≈0 (−0,017) dentro la testa. Dentro ciascun gruppo la mappa è liscia e coerente; è **fra** i due gruppi che diverge.

**Limiti che dichiaro esplicitamente, e che impediscono di chiamare questo un limite informativo:**

- **Non esistono collisioni esatte.** L'audit rev1 ha trovato **zero** coppie entro 1e-8, 1e-6, 1e-4 e 1e-3, su tutte le 1 124 250 coppie, in spazio grezzo e standardizzato. La distanza minima fra due righe qualsiasi è 0,0309 (grezzo) / 0,0756 (standardizzato).
- Le coppie "vicine" qui usate stanno a 0,03–0,15, non a 1e-3. **Una sovrapposizione locale con disaccordo elevato non dimostra la non-identificabilità**: una funzione sufficientemente ripida può in linea di principio separare punti distanti 0,03.
- La stima del "floor" locale (0,068–0,080 con raggio 0,15–0,40) ha pochi campioni ai raggi piccoli (49–100 righe) e confonde curvatura locale e conflitto. **È una stima, non un limite inferiore**, e non la uso per concludere nulla di definitivo.

Quindi la formulazione sostenuta dai dati è: *le due branche sono localmente coerenti ciascuna al proprio interno e in conflitto fra loro a distanze piccole ma non nulle*. Questo è compatibile sia con un limite del contratto osservativo sia con un limite di ripidità della funzione rappresentata — e le due si distinguono sperimentalmente.

## 7. Confronto con luglio: **nominal-only confermato, ma non causale**

Dagli artefatti originali:

| | luglio (13/07, Markov-35) | luglio DAgger R1 (11/07) | corrente V26B B1 |
|---|---|---|---|
| base BC | **nominal-only, 500 step** | teacher 500 campioni | **3 anchor × 500 = 1500 righe** |
| ripetizione | **32×** → 16 000 campioni | 4× sui DAgger | **nessuna**, ogni riga una volta |
| supplemento | recovery 712 (89 unici × 8) | 68 unici → 272 | — |
| totale | 24 712 | 772 | 1500 |
| validation | split casuale 20% | — | 11 fold per segmento, embargo 20 |
| best val MSE | 6,632e-05 (RMSE 0,00814) | — | 0,002309 (RMSE 0,04805) |

**La tua ipotesi è confermata: la base pre-DAgger di luglio era nominal-only.** Ma due riserve pesano più della somiglianza:

1. **Il 6,63e-05 di luglio non è una misura di generalizzazione.** Con `nominal_repeat = 32`, ogni riga unica compare 32 volte; uno split casuale al 20% mette in validation righe che hanno 31 copie identiche in training. Quel numero misura memorizzazione su copie, non generalizzazione. Non è confrontabile col nostro 0,002309 out-of-fold e **non va usato come bersaglio**.
2. **Lo scope nominal-only non rimuoverebbe il conflitto dominante.** Misurato: testa×ciclo1 vale **0,16785 dentro un singolo anchor** contro 0,16366 fra anchor — praticamente identico. Il conflitto è intrinseco al transitorio di inizio episodio dentro una traiettoria, non un artefatto dell'aver messo insieme tre anchor. Per contrasto, il disaccordo puramente fra anchor a input appaiati è 0,045–0,048, cioè un terzo.

**Verdetto**: lo scope dataset è una deviazione metodologica **reale ma secondaria**; non la propongo come prossimo intervento, per evidenza e non per analogia. La matrice multistart resta rimandabile come da indicazione dell'utente.

## 8. Leve valutate ed escluse

| leva | evidenza a favore | perché NON la propongo |
|---|---|---|
| **patience / selection** | e\* mancato per 9 epoche; spread 44% | l'oracle globale (0,04120) e quello per-fold (0,03599) **falliscono entrambi** il gate 0,03 |
| **loss/sampling per segmento** | teste = 41,8% della MSE pooled | teste + ciclo1 sono **50% del dataset**, non affamate; e i due gruppi **si sovrappongono** negli input, quindi ripesare sposta il compromesso fra loro senza scendere sotto il conflitto |
| **scope nominal-only** | luglio era davvero nominal-only | conflitto identico dentro un anchor (0,16785) e fra anchor (0,16366): non lo rimuoverebbe |
| **capacità (hidden width)** | due branche localmente coerenti separate da distanza piccola ma non nulla | romperebbe il warm start byte-identico da B0, che è l'unico parent ammesso |
| **nuova feature (conteggio cicli)** | risolverebbe il conflitto per costruzione | viola il contratto 35D e l'assenza voluta di clock; **decisione tua, non mia** |

## 9. Proposta: **UN** esperimento preregistrabile, una sola variabile

### B1R2-B — riduzione del learning rate

**Variabile unica**: `lr 3e-4 → 1e-4`. Nient'altro cambia: Adam, batch 64, seed 123, MSE piatta, clip-loss 1,0 (penalità sui bound dell'azione, **non** gradient clipping, che resta assente), logstd 0,1, anchor 1e-5, patience 60, eps 1e-9, budget 1200 epoche (baseline già stabilita da B1R2-A), init byte-identico da B0, maschera 0:2 e 25:35, disegno 11 fold = 8 cicli completi + 3 code, embargo 20, WAIT reconstruction-only, tutte le 1500 righe binding, **gate invariati**.

**Ipotesi causale.** Le due branche (testa e ciclo1) sono ciascuna internamente coerente — disaccordo interno ≈0 nella testa, 0,024 in ciclo1 — e confliggono solo fra loro, a distanze piccole ma **non nulle** (minima 0,0309; nessuna collisione esatta). Una funzione più ripida può quindi in principio separarle. Un passo di ottimizzazione più fine consente al modello di raggiungere minimi più ripidi dallo stesso init, invece di rimbalzare in un bacino largo: è coerente con lo spread del 44% osservato sulla curva pooled e con la dispersione degli argmin per fold fra 367 e 1189.

**Perché questa e non le altre.** È l'unica leva che (a) resta dentro ogni invariante — stesso parent, stessa maschera, stesso contratto, stessa architettura; (b) **non** è esclusa dall'argomento dell'oracle, che vincola la *selezione dell'epoca* sulla traiettoria attuale ma non ciò che una traiettoria di ottimizzazione **diversa** può raggiungere; (c) discrimina direttamente fra le due letture ancora aperte del §6 — limite di ripidità rappresentativa contro limite del contratto osservativo.

**Falsificatore, dichiarato prima.** L'ipotesi è falsificata se **una qualsiasi** di queste si verifica:
1. i fold 7 e 9 (le due teste) restano sopra 0,05 di RMSE heldout **al proprio best su 1200 epoche** — cioè il fallimento strutturale sopravvive a un'ottimizzazione più fine;
2. il pooled RMSE a e\* non scende sotto **0,04120**, cioè non batte l'oracle a epoca singola del braccio A: in tal caso il guadagno non viene da un bacino migliore;
3. il conflitto misurato testa×ciclo1 a distanza <0,15 resta ≥0,15 di \|Δtarget\| mediano — invariante del dataset, che deve restare tale, e serve da controllo di sanità.

Se falsificata, la spiegazione residua è il **contratto osservativo** nel regime pre-primo-ciclo, e la decisione successiva — contratto o gate — è tua, non mia: non la anticipo e non propongo di rilassare alcuna soglia.

**Gate**: gli stessi 9 gate binding, importati da `v26b_b1r1_exec`, non rilassati. GO solo se passano tutti; altrimenti NO-GO, stop offline, nessun candidato, nessun braccio C autonomo.

**Costo stimato**: ~11 fold × 1200 epoche + secondo passaggio + fit finale, dello stesso ordine di B1R2-A.

## 10. Cosa non ho fatto

Nessun fit, nessun rollout, nessuna collection, nessun DAgger, nessuna promozione, nessuna soglia toccata, nessun iperparametro modificato. Nessun file di produzione modificato. L'analisi è interamente read-only sugli artefatti già materializzati.

## 11. TODO propagati

- **TODO-2** — σ non assunto, da misurare prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo rimosso per costruzione. *(indirizzato)*
- **TODO-14** — Errore localizzato: 41,8% della MSE pooled da 308 righe di testa; fold 7 e 9 falliscono a **qualunque** epoca. *(aggiornato)*
- **TODO-16** — Patience: e\* mancato per 9 epoche (stop 297, miglioramento a 306). Correzione necessaria ma **non** causale: oracle 0,04120 e 0,03599 falliscono comunque. *(aggiornato, declassato)*
- **TODO-17** — Causa dell'errore sulle teste: identificato un conflitto testa↔ciclo1 a input sovrapposti (0,168 contro 0,045 interno), **senza collisioni esatte**; resta aperto se sia limite di ripidità o di contratto osservativo. B1R2-B lo discrimina. *(aggiornato)*
- **TODO-18** — Nuovo: `max_abs` WAIT peggiorato da 0,12686 a 0,18824 su due righe (minus020 locali 13–14), oltre il gate 0,15. *(nuovo)*
- **TODO-19** — Nuovo: il best val MSE di luglio (6,63e-05) proviene da uno split casuale su dataset ripetuto 32×, quindi **non è una misura di generalizzazione** e non va usato come bersaglio di confronto. *(nuovo)*
