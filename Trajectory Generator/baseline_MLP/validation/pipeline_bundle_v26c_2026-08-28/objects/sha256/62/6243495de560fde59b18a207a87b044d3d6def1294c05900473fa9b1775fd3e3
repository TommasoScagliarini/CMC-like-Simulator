# V26B B1R2-B — addendum correttivo: tre rettifiche vincolanti prima dell'esecuzione

**Data**: 2026-08-25
**Natura**: addendum **additivo** al report `2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md` (`18dc7e16…`), che resta agli atti invariato.
**Stato**: nessun fit, rollout, collection o DAgger eseguito. Nulla di B1R2-B è stato ancora avviato.

L'architetto accetta B1R2-B **solo come screening a una variabile**, con tre correzioni. Tutte e tre sono fondate e le recepisco integralmente. La seconda invalidava una conclusione che avrei potuto trarre e che ora il protocollo vieta esplicitamente; la terza correggeva un errore logico nella diagnostica stessa, che il braccio A dimostra numericamente.

---

## 1. Rettifica — il conflitto testa↔ciclo1 non è un falsificatore

**Cosa avevo scritto.** Nel §9 del report precedente avevo elencato fra i falsificatori: *«il conflitto misurato testa×ciclo1 a distanza <0,15 resta ≥0,15 di |Δtarget| mediano»*.

**Perché è sbagliato.** Quella quantità è calcolata **esclusivamente dal dataset** — osservazioni grezze e label del teacher — e non contiene alcun output del modello. È dunque **invariante rispetto al learning rate**, e rispetto a qualunque iperparametro di ottimizzazione. Un criterio che non può cambiare per effetto dell'intervento non è un falsificatore: non discrimina nulla. Elencarlo come tale era un errore logico, non un errore numerico.

**Cosa diventa.** Un **controllo di integrità/sanità**. Il suo valore atteso è noto e pinnato prima dell'esecuzione; se cambia, non ha detto nulla sull'ipotesi ma segnala che **la pipeline dati si è rotta** (dataset, cache, label, ordinamento o costruzione delle righe), e in quel caso il run va invalidato a prescindere dai gate.

Valori pinnati, calcolati sui 23D effettivi **grezzi**, coppie testa-swing × ciclo1-swing, escluse le coppie della stessa traiettoria entro embargo 20:

| grandezza | valore pinnato |
|---|---|
| righe testa-swing | 188 |
| righe ciclo1-swing | 262 |
| coppie totali | 49 256 |
| coppie a distanza < 0,15 | 92 |
| **mediana \|Δtarget\| a distanza < 0,15** | **0,1670604795217514** |
| mediana \|Δtarget\| su tutte le coppie | 0,4387725368142128 |
| distanza minima fra coppie | 0,030914171900546916 |
| controllo: coppie ciclo1×ciclo1 < 0,15 | 46 → mediana 0,04486663639545441 |
| controllo: coppie testa×testa < 0,15 | 54 → mediana 0,0 |

Il test dell'executor asserisce questi valori **esattamente** (uguaglianza in virgola mobile), prima che qualunque fit possa partire.

## 2. Rettifica — un NO-GO a lr ridotto **non** prova un limite del contratto

**Cosa avevo scritto.** Sempre nel §9: *«Se falsificata, la spiegazione residua è il contratto osservativo nel regime pre-primo-ciclo»*.

**Perché è sbagliato.** Con `lr` ridotto di 3× e budget **fisso** a 1200 epoche, il braccio B percorre meno strada di ottimizzazione del braccio A. Un NO-GO è quindi **confuso fra due cause distinte**:

- l'ipotesi di ripidità rappresentativa è falsa (il modello non può separare le due branche); oppure
- l'ottimizzazione semplicemente **non è arrivata** entro 1200 epoche a lr 1e-4.

Le due sono indistinguibili dal solo verdetto. **La conclusione sul contratto osservativo è quindi vietata dal protocollo di B1R2-B**, qualunque sia l'esito, e non la trarrò.

**Cosa il protocollo registra per rendere leggibile l'esito.** Non cambia la variabile unica; aggiunge soltanto diagnostica al receipt, dichiarata prima:

1. **Convergenza della curva completa** — definita nel §2-bis, che rettifica una formulazione precedente errata.
2. **Avanzamento**: pooled a 100/200/400/600/800/1000/1200, confrontato con le stesse epoche del braccio A, per misurare quanto del divario sia ritardo e quanto sia livello.
3. **Traiettoria di training loss**, per separare "più lento" da "fermo".

Se la curva del braccio B **non converge** entro 1200, l'esito corretto da riportare è *«screening non conclusivo, avanzamento insufficiente»* — non un NO-GO informativo, e certamente non una conclusione sul contratto. In tal caso l'unica mossa successiva legittima sarebbe un braccio a budget adeguato, **da autorizzare esplicitamente**, non da avviare in autonomia.

## 2-bis. Rettifica — l'esaurimento della patience di selezione **non** è convergenza

**Cosa avevo scritto.** Nel punto 1 qui sopra, in prima stesura: *«Convergenza: se la patience di 60 epoche si esaurisce entro le 1200 sulla curva pooled (minimo interno)»*.

**Perché è sbagliato, e il braccio A lo dimostra.** L'esaurimento della patience riguarda la **regola di selezione**: significa che la scansione di luglio ha smesso di guardare, non che la curva si sia assestata. Sono due grandezze diverse. Nel braccio A la scansione si è fermata a **297**, ma il minimo globale della curva pooled completa su 1200 epoche era a **1171**, con MSE 0,001697248 (RMSE 0,0411977) contro 0,002309152 (RMSE 0,0480536) all'e\* selezionato. Il minimo vero era **oltre 870 epoche più tardi e migliore**. Chiamare "minimo interno" l'arresto della scansione era quindi un errore, e va evitata anche quella locuzione.

**Definizione corretta, diagnostica e separata.** La curva pooled **completa** è **convergente al budget** se e solo se:

> il suo **minimo globale** cade **a o prima dell'epoca 1140**, ed è seguito da almeno **60 epoche** senza miglioramento superiore a **1e-9**.

Un argmin oltre 1140 significa **non convergente al budget**. La soglia 1140 è esattamente 1200 − 60: serve margine di budget per poter osservare le 60 epoche di non miglioramento.

**La regola binding di selezione di e\* non cambia**: resta la regola di luglio con patience 60 ed eps 1e-9 sulla curva pooled. Questa diagnostica è puramente interpretativa e non può spostare e\*, un gate o il verdetto. L'epoca di arresto della scansione viene comunque registrata, ma **separatamente** e mai come sinonimo di convergenza.

**Ricaduta retrospettiva sul braccio A.** Sotto questa definizione il **braccio A stesso non era convergente** al proprio budget: argmin 1171 > 1140. Lo registro perché vale in entrambe le direzioni — anche il NO-GO del braccio A non va sovra-interpretato.

## 3. Ipotesi e falsificatori, nella forma corretta

**Screening a una variabile**: `lr 3e-4 → 1e-4`. Nient'altro.

**Ipotesi.** Le due branche (testa e ciclo1) sono ciascuna internamente coerente — disaccordo mediano 0,0 dentro la testa e 0,0449 dentro ciclo1, contro 0,1671 fra loro — e non esistono collisioni esatte (distanza minima 0,0309; l'audit rev1 ha trovato zero coppie entro 1e-3). Una funzione più ripida può quindi in linea di principio separarle. Un passo di ottimizzazione più fine potrebbe permettere di raggiungerla dallo stesso init B0.

**Falsificatori** — ridotti a due, entrambi effettivamente sensibili all'intervento:

1. i fold **7 e 9** (le due teste) restano sopra **0,05** di RMSE heldout **al proprio best sulle 1200 epoche**;
2. il pooled RMSE a e\* non scende sotto **0,04120**, cioè non batte il minimo a epoca singola del braccio A.

**Entrambi vanno letti solo se la diagnostica del §2-bis riporta convergenza della curva completa** (argmin globale ≤ 1140 seguito da 60 epoche senza miglioramento >1e-9). Se la curva è ancora in discesa al bordo del budget, i falsificatori non si applicano e l'esito è "screening non conclusivo".

**Controllo di integrità** (§1): i valori del conflitto devono coincidere esattamente. Non è un falsificatore.

**Gate**: gli stessi 9 gate binding di B1R1/B1R2-A, importati e non rilassati. GO solo se passano tutti.

## 4. Cosa resta invariato

Parent **B0 35D masked** derivato dalla imitation V26 agosto, byte-identico; maschera hard 0:2 e 25:35, effective 23D; dati 3-anchor 1500 righe, tutte binding; 11 fold = 8 cicli completi + 3 code, embargo 20, WAIT reconstruction-only; regola di selezione pooled con patience 60 ed eps 1e-9; budget 1200; Adam, batch 64, seed 123, MSE piatta, clip-loss 1,0 (penalità sui bound dell'azione, **non** gradient clipping, che resta assente); logstd 0,1 congelata; anchor 1e-5; critic mai toccato. Nessun cambio a FSM v3, morfologia, contatto, reward, C++ o SEA. Sigma non misurata.

**Iniezione del learning rate.** `fit_masked` riceve un parametro opzionale keyword-only `lr` con default `None`, risolto alla costante pinnata `J_LR`. `J_LR = 3e-4` **non è toccata**. È l'unico iperparametro iniettabile; tutti gli altri restano a livello di modulo e non sono raggiungibili dal chiamante.

**Che cosa la prova dimostra esattamente** — dichiarato con precisione, perché una prima stesura della prereg sovradichiarava:

- il test chiama `fit_masked` **due volte sullo stesso fold** (fold 6, la coda minus020 da 11 righe) per **4 epoche** ciascuna: una omettendo `lr`, una passando `lr=J_LR` esplicito, e richiede che le due curve di validation registrate coincidano **valore per valore su quelle 4 epoche**;
- una terza chiamata, sempre di **4 epoche** sullo stesso fold, gira a `lr=1e-4` e richiede che la traiettoria differisca e che dopo la **prima epoca registrata** lo stato sia più vicino a B0 di quello del run di default, come un passo più piccolo impone.

**Che cosa la prova NON è.** Non è un confronto con la curva del fold registrata da B1R2-A: quel confronto non viene eseguito e non viene affermato. La prima stesura del campo `equivalence_proof` lo dichiarava; era una sovradichiarazione ed è corretta prima dell'esecuzione.

**Evidenza di regressione più ampia, separata.** Le sei suite di test preesistenti all'iniezione (673 check complessivi) passano invariate, e tre di esse continuano ad asserire che `J_LR` valga 3e-4.

## 5. TODO propagati

- **TODO-2 · 3 · 4 · 5 · 7 · 9** — invariati, aperti.
- **TODO-10** — Proxy autoregressivo rimosso per costruzione. *(indirizzato)*
- **TODO-14** — 41,8% della MSE pooled da 308 righe di testa; fold 7 e 9 falliscono a qualunque epoca. *(aperto)*
- **TODO-16** — Patience: correzione necessaria ma non causale (oracle 0,04120 e 0,03599 falliscono). *(aperto, declassato)*
- **TODO-17** — Conflitto testa↔ciclo1 senza collisioni esatte: resta aperto se sia limite di ripidità o di contratto. **B1R2-B non può chiuderlo**, per la rettifica §2: può solo, in caso di convergenza dimostrata, falsificare l'ipotesi di ripidità. *(aggiornato)*
- **TODO-18** — `max_abs` WAIT peggiorato a 0,18824 su minus020 locali 13–14. *(aperto)*
- **TODO-19** — Il best val MSE di luglio (6,63e-05) viene da split casuale su dataset ripetuto 32×: non è generalizzazione, non è un bersaglio. *(aperto)*
- **TODO-20** — Nuovo: se B1R2-B non converge entro 1200 epoche (argmin globale > 1140), serve un braccio a budget adeguato **da autorizzare esplicitamente**; non avviabile in autonomia. *(nuovo)*
- **TODO-21** — Nuovo: sotto la definizione corretta di convergenza il **braccio A non era convergente** (argmin 1171 > 1140); il suo NO-GO non va sovra-interpretato. *(nuovo)*
