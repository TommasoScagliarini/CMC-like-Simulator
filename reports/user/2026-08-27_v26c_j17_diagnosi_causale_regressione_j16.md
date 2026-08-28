# V26C J17 — Diagnosi causale della regressione J16

**Data**: 2026-08-27
**Stadio**: `V26C_J17_CAUSAL_DIAGNOSIS`
**Natura**: diagnosi. **Nessun fit, rollout, environment, critic, PPO, collection o promozione.**
**Report obbligatorio di fine fase.**

---

## 1. Problema

J12 qualificò l'attore J11 sulla matrice A–F: **4/6**. J14 raccolse 854 righe correttive dalle due
celle fallite. J15R1 rifittò da J2 su 25567 righe, superò il gate supervisionato 17/17 e abbassò
del **60.5%** l'errore sulle righe correttive. J16 rieseguì la **stessa** matrice cambiando **solo
l'attore**: **0/6**. A, B, C, E, F terminano per penetrazione oltre 0.028 m; D completa 500 passi
sotto la barra ma con un solo ciclo valido.

La domanda: **perché un miglioramento supervisionato sulla regione correttiva ha coinciso con una
regressione closed-loop sulle ancore?**

---

## 2. Metodo

Preregistrazione scritta **prima** di ogni calcolo (6 ipotesi, 7 test, 57 artefatti pinnati,
nessuna conclusione), poi GO dell'architetto, poi T2–T7 **da zero**. Nessun frammento della
finestra pre-GO è stato usato o citato.

Ogni affermazione è etichettata **EVIDENZA** (misurata), **INFERENZA** (derivata) o **IPOTESI**.
Una correlazione non è mai chiamata causa. `INCONCLUSIVO` è un esito ammesso.

Due audit indipendenti read-only hanno lavorato in parallelo su lineage/integrità e sulle trace
committate; ho riconciliato personalmente ogni loro numero.

**Validazione preliminare del mio strumento**: il forward numpy che ho usato per tutte le misure
offline riproduce il `policy_mean` registrato dal runtime a **1.4e-7** (rumore float32). Senza
questo, nessun numero offline sarebbe confrontabile con il closed-loop.

---

## 3. Evidenze

### 3.1 Integrità e lineage — nessuna anomalia (T2)

Catena verificata **per hash**, non assunta:

```
runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best
   0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd   39D
   initialization_mode "fresh"; resume_from, warm_start_*, restore: tutti null/false
        ↓
j2_runs/j2_base_v26c_2026-08-26_r1/rl_module
   0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130   35D
        ↓                                    ↓
J11  19bf8a43804774c0…                J15R1  4d084a2a7f0012bd…
```

**Entrambi** dichiarano lo stesso parent J2 e l'hash coincide col disco. La radice non ha parent.
**Nessun artefatto di luglio** è parent, checkpoint o dataset operativo: l'unico file di luglio mai
aperto è un modulo **sorgente** del repository, letto in AST per estrarne un literal.

Liste di feature identiche e nello stesso ordine fra J2, J11 e J15R1. `class_and_ctor_args.pkl`
**byte-identico** fra i tre. Colonne clock 0 e 1 esattamente zero in entrambi gli studenti. Log-std
**bit-identica al parent** in entrambi, σ = 0.004999999670722372. **Nessuna chiave critic** in
nessuno dei tre. Assorbimento delle scale fisiche applicato **in modo identico**, provato
numericamente ricostruendo il forward su input grezzi e scalati.

### 3.2 Delta parametri (T4)

| | ‖J11−J2‖ | ‖J15R1−J2‖ | rapporto |
|---|---|---|---|
| backbone | 2.608696 | 7.136669 | 2.74× |
| output head | 0.182541 | 0.395046 | 2.16× |
| bias | 0.074211 | 0.207760 | 2.80× |
| **globale** | **2.616128** | **7.150613** | **2.73×** |

Relativo alla norma del parent: J11 si è spostato del 17.9%, **J15R1 del 48.9%**. Somma delle norme
del blocco controller: J2 = 0, J11 = 2.608, **J15R1 = 5.760 (2.21×)**.

### 3.3 Il compromesso, misurato offline (T3)

Errore RMSE contro le label del teacher, stessi input congelati:

| blocco | J11 | J15R1 | variazione |
|---|---|---|---|
| ancora nominale (500 uniche) | 0.005379 | 0.015536 | **+188.8%** |
| cella B unique | 0.008264 | 0.017372 | **+110.2%** |
| cella C unique | 0.007684 | 0.019970 | **+159.9%** |
| J7 recovery | 0.084148 | 0.081121 | −3.6% |
| **J14 correttive (854)** | **0.459119** | **0.181555** | **−60.5%** |

In radianti sull'ancora nominale: J2 **0.000000** → J11 **0.002084** → J15R1 **0.006554**.

**Il refit ha scambiato fedeltà sulle ancore per fedeltà sul recovery.** Le celle A–D esercitano
esattamente le ancore peggiorate.

### 3.4 La misura decisiva: massa di gradiente all'inizializzazione (T6)

Errore quadratico di ciascun blocco valutato **al parent J2**, cioè ciò che l'ottimizzatore vede al
passo 0:

| blocco | righe | % righe | massa SE | **% massa** |
|---|---|---|---|---|
| **J7 nominale** | 16000 | 62.58% | **5.63e-10** | **0.00%** |
| J7 recovery | 713 | 2.79% | 11.134 | 3.59% |
| cella B | 4000 | 15.65% | 32.673 | 10.54% |
| cella C | 4000 | 15.65% | 3.280 | 1.06% |
| **J14 correttive** | **854** | **3.34%** | **262.911** | **84.81%** |

Massa totale: J15R1 **309.998** contro J11 **47.086** — **6.58×**.

**Il blocco nominale è auto-distillato dal parent.** Il suo errore all'inizializzazione è
5.6e-10, cioè zero entro il rumore float32. **Non produce gradiente al passo 0 e può opporsi solo
dopo che la rete si è già allontanata.** Nel frattempo il 3.34% delle righe porta l'84.81% della
massa.

L'unica protezione è un `anchor_weight` di **0.01** nello **spazio dei parametri** (L2 verso
l'inizializzazione) — **non** nello spazio delle uscite. Non esiste alcun vincolo che dica «non
cambiare ciò che predici sulle ancore».

### 3.5 Conflitto di label — reale ma moderato (T6)

Distanza standardizzata delle 854 righe J14 dalle 1500 ancore (nominale + B + C):

| soglia | righe entro | differenza di label mediana |
|---|---|---|
| 0.25σ | 9 | 0.0560 |
| 0.5σ | 45 | 0.0645 |
| 1.0σ | 217 | 0.2947 |
| 3.0σ | 463 | 0.1599 |

Dispersione tipica delle label sulle ancore: **0.3983**. **846 ancore su 1500** hanno una riga J14
entro 3σ, e le righe J14 più vicine (p5 = 0.4845σ) sono più vicine alle ancore di quanto molte
ancore lo siano fra loro (mediana 0.3459σ).

**Calibrato**: il conflitto esiste e le regioni si sovrappongono, ma **al raggio più stretto è
modesto** — 16% della dispersione. La correlazione fra prossimità di un'ancora a una riga J14 e il
suo spostamento è **−0.34** (r² ≈ 0.12): le ancore vicine si sono spostate di più, ma questo spiega
solo una piccola parte della varianza. **È una correlazione, non una causa.**

### 3.6 Le trace: il divario nasce al passo 1 (T5)

Le osservazioni **sono registrate** (`actor_observation_vector_before`, 35 valori per passo, in
entrambe le run), quindi il controfattuale è reale: entrambi gli attori applicati agli **stessi**
stati visitati. Nulla è stato ricostruito.

**L'esperimento è controllato per costruzione**: griglia temporale, stato di reset e rumore di
esplorazione sono **bit-identici** fra J12 e J16 (max diff 0.000e+00 su tutte le celle; per la
cella D su tutti i 500 passi). Cambia il checkpoint dell'attore e nient'altro.

Al passo 1 entrambe le run danno alla rete un vettore **bit-identico** e ottengono azioni diverse:
cella A |Δa₀| = 0.0364968776703, cella C **0.0649289488792** — e C è la cella che muore prima (61
passi). Il divario **non può essere stato causato da una differenza di stato**.

**Ordinamento**: azione al passo 1 → cinematica al passo 2–3 → penetrazione oltre la soglia soft ai
passi 11–47.

### 3.7 Il meccanismo, identificato (T5)

Nella finestra che precede la breccia, J16 comanda **sistematicamente meno azione sul ginocchio**:

| cella | J12 media a₀ | J16 media a₀ | Δ |
|---|---|---|---|
| A | 0.638269 | 0.532991 | **−0.105278** |
| B | 0.598534 | 0.505111 | **−0.093423** |
| C | 0.682616 | 0.458334 | **−0.224283** |
| E | 0.609109 | 0.529083 | **−0.080026** |
| F | 0.577881 | 0.533161 | **−0.044720** |

E il ginocchio **collassa in flessione sotto carico**. Cella D, dove il rumore è bit-identico:

| idx | J12 knee | J16 knee | J12 a₀ | J16 a₀ | J12 pen | J16 pen |
|---|---|---|---|---|---|---|
| 26 | −0.241180 | −0.273753 | 0.68733 | 0.49130 | 0.014812 | 0.019450 |
| 30 | −0.240345 | −0.358601 | 0.68067 | 0.49055 | 0.016861 | 0.025642 |
| 32 | −0.239165 | **−0.394834** | 0.67272 | 0.50320 | 0.017683 | **0.026871** |
| 36 | −0.239253 | −0.421815 | 0.67813 | 0.60896 | 0.018927 | 0.024736 |

Escursione del ginocchio su idx 26–40: J12 **0.004504 rad**, J16 **0.148062 rad**. La penetrazione
segue il collasso passo per passo. J16 arriva a **0.001129 m** dalla barra vincolante, poi
sovracorregge (a₀ sale a 0.73).

**Saturazione**: J16 clippa 12 passi nella cella C con azione grezza fino a **1.188135**. J12 non
clippa **mai**, in nessuna cella.

### 3.8 Dove finisce l'attore, nello spazio degli stati

Distanza mediana degli stati visitati dall'ancora più vicina (33 colonne non-clock):

| | A | B | C | D | E | F |
|---|---|---|---|---|---|---|
| **J11 in J12** | 0.475 | 0.473 | 0.427 | 0.456 | 1.947 | 4.474 |
| **J15R1 in J16** | **2.516** | **2.907** | **3.116** | **1.119** | **2.729** | **2.691** |

J11 restava aderente alle ancore nelle celle che superava; **J15R1 sta 5–7× più lontano**, e non è
neppure vicino alle righe correttive (mediana 2.8–4.0σ). Se n'è andato altrove.

### 3.9 Un fatto strutturale che riorienta la lettura

Le 854 righe correttive **sono, riga per riga, la traiettoria che J11 ha percorso in J12**:
verificato con max|differenza| = **0.000e+00** sulle 33 colonne che l'attore usa.

**Ma il fit che le ha consumate è partito fresco da J2.** Gli stati aggregati erano quindi
**off-policy** sia per l'inizializzazione sia per l'attore risultante. È uno scostamento dalla
premessa di DAgger, che vuole gli stati raccolti **dalla policy che si sta migliorando**.

---

## 4. Discriminazione delle ipotesi

### H1 — refit fresco sull'aggregato / deriva nominale — **SOSTENUTA**, confidenza **alta**

*Pro*: il blocco nominale non produce gradiente all'inizializzazione (5.6e-10); la protezione è un
solo penalty di 0.01 nello spazio dei parametri; il backbone si è spostato 2.73×; l'errore
sull'ancora è triplicato offline; le celle delle ancore hanno regredito closed-loop.
*Contro*: nessuna evidenza contraria.
La confidenza è alta perché il meccanismo è **misurato all'inizializzazione**, prima di qualunque
dinamica, e l'esito è coerente con esso in ogni blocco.

### H2 — dominanza del recovery / conflitto di label — **SOSTENUTA per la dominanza, PARZIALE per il conflitto**, confidenza **alta / media**

*Dominanza*: 84.81% della massa in 3.34% delle righe, massa totale 6.58× J11 — **evidenza diretta**.
*Conflitto*: esiste (846/1500 ancore hanno una riga J14 entro 3σ) ma al raggio stretto è modesto
(16% della dispersione), e la correlazione prossimità/spostamento spiega solo ~12% della varianza.
Il conflitto **contribuisce**; non è il motore.

### H3 — scaling / semantica dell'azione / loader — **RIGETTATA**, confidenza **alta**

Liste di feature identiche, `class_and_ctor_args` byte-identico, assorbimento delle scale provato
identico per via numerica, stesso loader, stessa classe, stesse chiavi di stato, forward che
riproduce il runtime a 1.4e-7. Nessuna evidenza a favore.

### H4 — mismatch di supporto covariato — **RIGETTATA come origine, SOSTENUTA come amplificatore**, confidenza **alta**

*Come origine*: **rigettata**. Il divario nasce al passo 1 su un'osservazione bit-identica, dentro
il supporto. Non può essere causato da uno scostamento di distribuzione.
*Come amplificatore*: **sostenuta**. Gli stati visitati da J15R1 stanno 5–7× più lontano dalle
ancore di quanto stessero quelli di J11. La deriva di supporto è una **conseguenza** che poi
amplifica, non la causa prima.

### H5 — contaminazione critic/logstd o lineage inattesa — **RIGETTATA**, confidenza **alta**

Lineage verificata per hash end-to-end; radice senza parent; nessun input operativo di luglio;
log-std bit-identica al parent in entrambi; nessuna chiave critic.
**Sfumatura da dichiarare**: J11 **è** nella catena dei *dati* di J15R1, a un salto — 854 righe sono
stati visitati da J11. È una dipendenza di distribuzione, non di pesi. «J11 non è stato caricato»
resta vero e verificato.

### H6 — deriva knee/ankle che precede la penetrazione — **SOSTENUTA con riserva sull'ordinamento**, confidenza **media-alta**

*Meccanismo*: **identificato**. J16 sotto-comanda il ginocchio prima della breccia in tutte le celle
fallite; il ginocchio collassa sotto carico; la penetrazione segue.
*Ordinamento azione→cinematica*: solido, **per costruzione**.
*Ordinamento cinematica→penetrazione*: **parzialmente mascherato**. In A, B, D, E, F la penetrazione
è identicamente 0.0 in entrambe le run mentre il piede è in aria: una differenza in una grandezza
nulla in entrambe è strutturalmente non misurabile. **Nella cella C, dove il piede è caricato al
reset, le due differenze compaiono nello stesso istante fisico e non sono ordinabili.**

---

## 5. Conclusione calibrata

**Ciò che l'evidenza sostiene.** Il refit fresco ha inseguito una regione piccola, lontana e ad
altissimo errore — 854 righe con l'84.81% della massa di gradiente — mentre le ancore, essendo
**auto-distillate dal parent**, non producevano alcun gradiente con cui difendersi, e l'unico
vincolo era un penalty debole nello spazio dei parametri anziché delle uscite. Il risultato è uno
spostamento del backbone di 2.73× e un errore sulle ancore triplicato. Closed-loop questo si
manifesta come un comando di ginocchio più debole prima della breccia, un ginocchio che collassa
sotto carico, e una penetrazione che segue il collasso fino alla terminazione.

**Ciò che l'evidenza non stabilisce.** Non ho un esperimento controllato che isoli il contributo di
ciascun fattore: non posso quantificare quanta parte della regressione venga dalla dominanza di
massa e quanta dal conflitto locale, perché separarli richiederebbe un training. Non stabilisco che
il collasso del ginocchio sia l'**unico** percorso verso la penetrazione. E l'ordinamento
cinematica→penetrazione resta non risolto nella cella C.

**Un limite che merita di essere isolato.** La deviazione pre-breccia **non è rilevabile dentro una
singola run**: l'errore di tracking di J16 contro il target imitativo è paragonabile a quello di
J12, e migliore in due celle. Solo il confronto appaiato la rivela. Una metrica offline o un
monitor su singola run **non avrebbero intercettato** questa regressione prima della penetrazione.

---

## 6. Alternative scartate

- **«È un bug di scaling, loader o semantica dell'azione»** — rigettata: identità provata su ogni
  contratto strutturale, e il forward riproduce il runtime a 1.4e-7.
- **«L'attore o la sua lineage sono contaminati»** — rigettata: ogni hash verifica, log-std e critic
  sono conformi, la radice non ha parent.
- **«È deriva di supporto covariato»** — rigettata come origine: il divario nasce su uno stato
  identico.
- **«Il conflitto di label locale è il motore»** — ridimensionata: reale ma modesta al raggio
  stretto, r² ≈ 0.12.
- **«La penetrazione è il problema primario»** — ridimensionata: è l'evento terminale, preceduto da
  una deviazione cinematica e, prima ancora, da una differenza di comando.
- **«La cella D è sana e ha solo perso un conteggio»** — rigettata: D sfiora la barra a 0.001129 m,
  passa 112 passi sopra la soglia soft contro gli 81 di J12 in 7 escursioni contro 3, e perde il
  ciclo per un atterraggio di solo avampiede con il ginocchio bloccato fra −0.69 e −0.97 rad.

---

## 7. Proposta di correzione minima — **NON ESEGUITA**

**Raccomandazione: J11 + update recovery-only vincolato, in luogo di un nuovo refit fresco.**

### Perché è preferibile, sull'evidenza

1. **Le ancore possono difendersi.** Partendo da J11 l'errore sulle ancore è 0.002084 rad, non
   zero: producono gradiente **dal passo 0**. Partendo da J2 non ne producono affatto. Questa è la
   differenza strutturale che ha reso il refit fresco fragile.
2. **Il grosso del beneficio è ottenibile senza toccarle.** Il **74.57%** della massa d'errore di
   J11 sulle righe correttive vive a **≥3σ** dalle ancore; solo il 25.43% sta nella zona contesa.
3. **Ripristina la premessa DAgger.** Gli stati correttivi vengono da J11: usarli per aggiornare
   **J11** li rende on-policy, cosa che il refit fresco non era.
4. **J11 ha già superato 4/6.** Il bersaglio è E ed F senza perdere A–D — non ricostruire tutto.

### Prerequisiti

- Vincolo di ancora **nello spazio delle uscite**, non dei parametri: penalizzare
  |f(x) − f_J11(x)| sulle ancore, che è esattamente ciò che mancava.
- Bilanciamento esplicito della **massa di gradiente** per blocco, non del solo conteggio righe: è
  la massa, non il numero di righe, ad aver dominato.
- Nessun cambiamento a FSM v3, morphology, reward, sigma, SEA/C++, produzione, architettura.
  Nessun actor 25D, nessun widening, nessuna feature controlaterale.
- Semi 126–128 sigillati.

### Rischi

- Il conflitto locale è reale: le 45 righe entro 0.5σ chiedono label diverse. Un vincolo troppo
  stretto potrebbe impedire di correggere E/F; troppo lasco riprodurrebbe J16.
- Partire da J11 eredita ciò che ha reso E/F difficili: non è un punto neutro.
- **Nessuna metrica offline garantisce l'esito closed-loop.** J15R1 superò 17/17 offline e fallì
  0/6. Questa è la lezione più solida della fase.
- La deviazione non è rilevabile su singola run: servirà un confronto appaiato contro J11.

### Metriche offline proposte (da preregistrare, non dedotte qui)

Errore sulle ancore in radianti; errore sulle 854 righe correttive; spostamento del backbone dal
punto di partenza; massa di gradiente per blocco; **e soprattutto** il confronto appaiato
J11-vs-candidato sugli stessi stati visitati, che è l'unica misura che avrebbe visto arrivare
questa regressione.

**Non propongo alcuna soglia numerica.** J11 a 0.002084 rad passò 4/6 e J15R1 a 0.006554 fallì 0/6,
ma **due punti non definiscono una barra**: qualunque soglia dovrà essere preregistrata da te.

### Gate closed-loop

La stessa matrice A–F congelata, stesse soglie, unica variabile l'attore, `fail_fast: false`,
PASS solo con 6/6 e 6/6. Nessuna scorciatoia offline la sostituisce.

**Non ho eseguito nulla di tutto questo.** Nessun retry è autorizzato.

---

## 8. File creati e verifiche

| artefatto | SHA-256 |
|---|---|
| `v26c_j17_prereg_causal_diagnosis.json` | `1e765477660e0f0c7d80b91cd677889b974394b1c4dd12b85cfa24bcb991fba4` |
| `v26c_j17_diagnostic_measurements.json` | `5c5f309a5cde39a6233d52afe65d90be17a999879ea8acbecb64f53429434e5c` |
| `v26c_j17_reconciliation_and_corrections.json` | `d165b86732ce78000355f64b220382e6723edccbfc0a65f0e4032189b9ad2d56` |
| record GO dell'architetto (verificato) | `ac5e063750f8284824acebd6815def09561a940200b3e30e3b9ae441155478ca` |

**Verifiche**: 57/57 pin della preregistrazione riverificati da me; 8/8 pin additivi di lineage;
forward validato contro il runtime a 1.4e-7; ogni numero degli audit indipendenti che ho
ri-misurato riprodotto esattamente.

**Riconciliazione**: **nessuna divergenza** su alcuna grandezza verificabile. Gli audit sono andati
**oltre** le mie misure in tre punti, e mi hanno **corretto** in tre punti — l'ordinamento
parzialmente mascherato, la correlazione interna alle run che avevo usato per rigettare il
meccanismo cinematico, e l'offset di un passo fra cinematica pre-step e penetrazione post-step.
Tutte e tre le correzioni sono registrate nell'artefatto di riconciliazione.

**Nulla è stato modificato**: nessun attore, dataset, configurazione, leaf o report preesistente.
Nessun fit, rollout, environment, critic, PPO. Semi 126–128 e G–I **non toccati**.

---

## 9. TODO propagati

- **La correzione proposta non è eseguita** e richiede un tuo GO.
- **`nominal_mean_shift` dichiarato e non misurato nel runner J15R1** — aperto.
- **`policy_std` sempre `null`** — difetto cosmetico ereditato da J12.
- **Nessuna leaf pinna il runner che l'ha scritta**; per J15R1 due runner puntano alla stessa leaf.
- **Il fit J11 non è bit-riproducibile** dai propri artefatti (versione torch e argomenti Adam
  assenti); J15R1 li registra.
- **La leaf J2 non ha `commit_verification.json`** né `actor_digest`.
- **`best_validation_mse` è contaminato** in entrambi i fit dalla ripetizione dei blocchi.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126, 127, 128 e la fase G–I** restano sigillati.
- **Ogni fase si chiude con uno user report**, auditato prima della successiva.

---

## 10. STOP

Diagnosi conclusa. **Nessuna correzione eseguita. J18 non iniziato.**

**Fermo in attesa del tuo audit.**
