# V26B — ramo 35D-masked: quarantena del 25D, protocollo, tooling e preflight

**Data**: 2026-08-25
**Preregistrazione**: `v26b_amendment_b_masked35.json` — SHA-256 `0830d82feb641043dffc4457c366a8dc6aa12979a3d5605e4a424903e19bf3ae`
**Addendum di supersessione**: `v26b_addendum_a0a1_superseded.json` — SHA-256 `cc24eec8cb56a459b7a6d280af9d0232014018638bba9f1e223325295a4b6791`
**Esito**: B0 **GO** · B1 **GO senza bloccanti** · self-test **85/85 PASS**
**Nessun fit, training, rollout, raccolta o candidato. Nessuna modifica a produzione, FSM v3, morfologia, contatto, reward, guardie, C++ o SEA.**

---

## 1. Quarantena del ramo 25D — nulla cancellato

Il ramo A0/A1 è marcato **`SUPERSEDED_BEFORE_EXECUTION`**. Tutti i suoi file restano su disco **byte-identici**, verificato per SHA nel test:

| file superseduto | SHA-256 | stato |
|---|---|---|
| `v26b_amendment_a0a1_25d.json` | `9dfe1229…` | intatto |
| `v26b_contract_25d_v1.yaml` | `b7292a73…` | intatto |
| `v26b_a0_transplant25.py` | `bce45d03…` | intatto |
| `v26b_a1_ik_imitation.py` | `155b0f28…` | intatto |
| `test_v26b_a0a1_25d.py` | `5739ac55…` | intatto |
| receipt preflight A0/A1 | `4d1bec0f…` | intatto |
| report A0/A1 | `61609639…` | intatto |

Stato di esecuzione al momento della supersessione: fit **MAI ESEGUITO**, training **MAI ESEGUITO**, rollout **MAI ESEGUITO**, raccolta **MAI ESEGUITA**, nessun candidato materializzato. **La supersessione non costa alcun esperimento e non invalida alcun risultato.**

Regola di quarantena: quei file restano leggibili come evidenza di progetto, non vanno mai eseguiti né importati. Il test verifica che i nuovi moduli **non contengano alcun `import`** dei moduli superseduti e che nessuno di essi sia caricato nel processo.

## 2. Cosa risolve il nuovo disegno

| problema del ramo 25D | risoluzione nel ramo 35D-masked |
|---|---|
| secondo contratto osservativo e secondo insieme di pin di runtime (TODO-11) | **un solo contratto**: il v3 canonico 35D, invariato. TODO-11 **chiuso** |
| stadio A2 di commutazione di contratto da verificare bit-per-bit | la transizione è **bit-exact per costruzione**: le colonne sono già zero |
| equivalenza col sottospazio 25D assunta architetturalmente | equivalenza **dimostrata**, bit-exact, su dati sintetici e reali |
| A1 bloccato da una raccolta teacher mancante | **nessuna raccolta necessaria**: il dataset esiste già nel lineage V26 |
| manifest e formato di export doppi | **manifest unico 35D**, `c6f86028…` |

## 3. Il canale mascherato

Dieci colonne, derivate **per nome** dal manifest 35D pinnato e poi confrontate con la tupla attesa (una manomissione dell'attesa fallisce chiusa):

`[25..34]` = `pros_{knee,ankle}_angle_{previous_endpoint, served_ref, served_ref_vel, served_ref_accel, sea_u}`

Due garanzie **indipendenti**, ed è il punto del disegno:

- **maschera d'ingresso** — le colonne osservative sono poste a 0.0 prima che la rete le veda;
- **colonne a zero** — le corrispondenti colonne del primo layer sono esattamente zero, congelate, e ripristinate a zero dopo ogni passo di optimizer.

Il test ha rivelato — e ora documenta — che coprono **modi di guasto diversi**: con la maschera attiva, una colonna che perdesse resterebbe innocua perché moltiplica uno zero, quindi l'equivalenza 25D regge comunque; è invece la **transizione unmask** a rilevare la colonna sporca. Ciascun meccanismo fa da rete di sicurezza all'altro.

## 4. B0 — trapianto mascherato, GO

Parent **esclusivamente** V26 agosto 39D `5bbc6cbd…`. Ricetta: il trapianto congelato 39→35 riprodotto da V26 (target sani compensati sulla media, clock azzerato, hidden e testa copiati bit-identici), poi le dieci colonne di stato controller poste a zero.

Verificato nel test, colonna per colonna e tensore per tensore: il risultato è **bit-identico alla ricetta 39→35** su ogni colonna conservata e su ogni altro tensore; differiscono **solo** `pi.0.0.weight` e `pi_encoder.0.weight`, e solo sulle dieci colonne. Digest B0: **`b2429a8217f99ed7ab4e61620480f797d4617c7713b857624e85c095bed57c55`**.

**Scelta di compensazione, dichiarata**: i 4 target sani **sono** compensati sulla media (escono dal vettore per sempre, come nella ricetta congelata); le 10 colonne mascherate **non lo sono**, deliberatamente — restano nel vettore e tornano nella fase Markov, e caricare la loro media nel bias significherebbe precaricare una costante che l'adattamento dovrebbe poi disimparare. L'espansione di luglio partiva da zero pulito per la stessa ragione.

### Le due prove, bit-exact

| prova | dati | esito |
|---|---|---|
| equivalenza funzionale col sottospazio 25D | 256 righe reali di ancora + casuali | **bit-identica**, differenza massima **0.0** |
| transizione mask-on → full35 con colonne a zero | idem, con probe che differiscono sulle colonne mascherate | **bit-identica**, differenza massima **0.0** |
| equivalenza 25D sull'intero dataset B1 | 1500 righe | **bit-identica**, differenza massima **0.0** |

Entrambe sono *load-bearing*: il test verifica che una colonna sporca da `1e-9` faccia fallire la transizione, e che un probe già mascherato venga rifiutato come vacuo.

### Maschera del gradiente, dimostrata su un vero passo di optimizer

Con un `torch.optim.Adam` reale su dati sintetici:
- con ingresso mascherato il gradiente sulle dieci colonne è **identicamente zero**, mentre le colonne conservate ricevono gradiente;
- dopo il passo, `assert_no_masked_update` conferma colonne **bit-identiche e ancora esattamente zero**;
- **controllo negativo**: con ingresso non mascherato il gradiente su quelle colonne è **diverso da zero**, il passo le muove, e la guardia **fallisce chiusa**; il ripristino ripara e la guardia poi passa.

## 5. B1 — fit base, GO senza bloccanti

**Il dataset esiste già e non richiede alcuna raccolta.** Tre traiettorie di ancore V26 pinnate (`A_ISO39CLK_V3`), 500 righe ciascuna:

| start | id | righe | cache label |
|---|---|---|---|
| minus020 | 0 | 500 | `f97ad154…` |
| nominal | 1 | 500 | `3dd878d4…` |
| plus020 | 2 | 500 | `f15d624c…` |

Label: `u_IK` AB06 prescritta ai tempi propri dell'ancora, **lookup float esatto** sulle cache privilegiate pinnate, nessuna interpolazione. Provenienza: nessuna riga S0D, S1A, REV4* o V2_*.

**Split fail-closed leave-one-trajectory-out**, 3 fold da 1000/500 — mai casuale; il test verifica che nel modulo non compaiano `permutation` né `shuffle`. È lo split per traiettoria che avevi richiesto, ora possibile perché il dataset ha tre traiettorie native.

Protocollo per passo di optimizer, in 7 punti dichiarati: maschera ingresso → forward/backward → azzeramento gradiente su colonne mascherate e clock → step → ripristino a zero → `assert_no_masked_update` → ripristino logstd congelata. Congelati: logstd, critic (mai toccato), le dieci colonne, le due del clock. Nessun PPO.

**Diagnostico utile**: l'RMSE iniziale di B0 sul dataset è **0,619705**. È la misura di quanto il canale rimosso stesse portando — coerente con il 66–71% di massa del primo layer che avevo misurato su quel canale — ed è ciò che B1 deve ricostruire usando le sole 25 feature.

## 6. Gate closed-loop, riportati invariati

Completamento 500/500 `episode_time_limit` · **≥ 2 cicli validi** · contatori critici a zero (`phase_timeout_stance`, `phase_timeout_swing`, `morphology_causal_contract_failure`, `hs_cancelled_count`) · resync ≤ 1 · penetrazione ≤ **0,020 m**, la guardia soft v3 corrente, non allentata.

Qualità cinematica biologica: **caviglia q_min ≤ −0,03 rad** (19,3% della profondità di riferimento 0,1552; il test verifica esplicitamente che **−0,0099 NON la soddisfi**, essendo −0,0099 > −0,03 e pari al 33% della soglia in modulo) · ampiezza caviglia ≥ 0,30 rad (54,0% del riferimento) · ampiezza ginocchio ≥ 0,60 rad (70,4%) · ginocchio sempre flesso · dentro i bounds.

Le soglie sono **ridichiarate dentro B1**, non importate dal ramo superseduto: la quarantena regge per costruzione.

## 7. Fase Markov — solo dichiarata

Innesco: **solo dopo** i gate della base. Operazione: rimuovere la maschera; le dieci colonne sono ancora esattamente zero, quindi la transizione è bit-exact. Dataset: ancore nominali dell'attore base etichettate con **le sue stesse medie**, recovery same-actor phase-consistent, multistart solo se separatamente autorizzato. Congelati logstd e critic. **Se adattare l'intera mean-network non è deciso ora**: sarà preregistrato in quella sede.

σ resta **APERTO**: 0,005 non è assunto e va misurato sull'attore base prima di qualunque recovery stocastica.

## 8. File creati (solo additivi)

| file | SHA-256 |
|---|---|
| `v26b_addendum_a0a1_superseded.json` | `cc24eec8cb56a459b7a6d280af9d0232014018638bba9f1e223325295a4b6791` |
| `v26b_amendment_b_masked35.json` | `0830d82feb641043dffc4457c366a8dc6aa12979a3d5605e4a424903e19bf3ae` |
| `v26b_b0_masked35.py` | `835f5b6dcb2a8355efa5132864188164577baef13ceee923a8115ef12fd4fe4c` |
| `v26b_b1_base_fit.py` | `8e2fcff74d4d9fa72972bcd806e0602d4dacc162671cfca02ae84eb03f0cefec` |
| `test_v26b_b0b1_masked35.py` | `8c11fc0fde7c82eed79ee55930914f21beed4c45473ed3281f2103a3f9dd578e` |
| receipt `runs/.../diagnostics/b_masked35/v26b_b_masked35_preflight_receipt.json` | `951abf3759fd696f73052950de9d574aaab7e870ebacef93d7c5f53c9a12acee` |

**Comandi eseguiti**
```
python test_v26b_b0b1_masked35.py       -> {"selftest": "PASS", "checks": 85}
python v26b_b0_masked35.py --preflight  -> verdict GO
python v26b_b1_base_fit.py --preflight  -> verdict GO, blockers []
```

`git status` conferma zero modifiche a `osim_trj_cmc_like.py`, `target_domain_imitation.py`, `target_domain_markov_adaptation.py`, `prosthetic_phase_fsm.py`, `online_grf.py`, `reward_function.py`, al config v3 risolto e a `tools/` (C++/SEA). I moduli congelati riusati (`v26b_student.py`, `f2r_refit.py`) sono immutati. Portabilità: solo `pathlib`, nessuna shell, nessun separatore OS-specifico — verificato da test.

## 9. Comandi futuri, ciascuno da autorizzare

```
python v26b_b0_masked35.py --authorized-stage V26B-B0-MASKED35-TRANSPLANT   # materializza B0
python v26b_b1_base_fit.py --authorized-stage V26B-B1-BASE-FIT-MASKED35     # fit base mascherato
```

Entrambi **rifiutano di eseguire** allo stato attuale, verificato da test.

## 10. Ambiguità che segnalo

1. **Il dataset B1 usa tutte e tre le ancore, inclusi gli start ±0,20 s.** Non è una raccolta — le ancore sono artefatti V26 già pinnati e già usati da V1 — ma introduce il multistart già nella fase base, mentre a luglio arrivava solo alla fase Markov. Lo raccomando (abilita lo split per traiettoria e costruisce la robustezza agli start fin da subito, che è ciò che il refinement rifiutato di luglio dimostrò necessario), ma **è una tua decisione** e il tool supporta anche il solo nominale.
2. **La scelta di non compensare le dieci colonne** è mia, motivata al §4. Cambia il punto di partenza di B1.
3. **L'RMSE iniziale 0,6197** dice che B1 parte lontano: le 25 feature devono ricostruire ciò che il canale rimosso portava. È il costo previsto della rimozione del proxy, non un difetto, ma va tenuto presente nel valutare la fattibilità del gate 500/500.

## 11. TODO propagati

- **TODO-2** — σ = 0,005 non assunto: operativo, da misurare sull'attore base prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota a copertura piena. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura come terminatore di REV4E. *(aperto)*
- **TODO-9** — Righe di swing ammesse per raggiungibilità al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo ereditato da V26 al 71,25%: **indirizzato** dal mascheramento di B0. *(in corso)*
- **~~TODO-11~~** — **CHIUSO**: il contratto unico 35D elimina il secondo insieme di pin di runtime.
