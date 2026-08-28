# V26C J14 — Dataset DAgger post-mismatch: readiness

**Data**: 2026-08-27
**Stadio proposto**: `V26C_J14_POST_MISMATCH_DAGGER_DATASET`
**Stato**: bundle pronto per l'audit Codex. **Leaf non materializzato.**
**Autorizzazione**: `execution_permitted_now: false`. Il comando è congelato, non autorizzato.
**Nessun rollout, fit, collection, critic o PPO. Nessun file J0–J13 modificato.**

---

## 1. Supplemento e correzione architetturale a J13

L'architetto ha **accettato la diagnosi causale** di J13 — gap di supporto post-mismatch / swing
prolungato, non un difetto dell'architettura 35D né rumore diverso; J11 è July-like nel profilo e
J8 è l'outlier poco mosso — e ha emesso sei correzioni di governance vincolanti.

### Cosa J13 ha sbagliato

J13 proponeva di **escludere la cella F** dalla correzione DAgger, sulla base del protocollo F2R
del 23/08 che riservava il seed 125 come gate held-out. Due errori:

1. **Quel protocollo non era autorizzato.** Era marcato «PROPOSTA NON ANCORA AUTORIZZATA», usava
   `JUL_H0` come init operativa, ed è stato superato dalla decisione dell'utente: base operativa
   **August V26 → J2 35D**, luglio solo fonte metodologica. J13 aveva segnalato che la sua forza
   vincolante era una decisione dell'architetto, ma non bastava.
2. **L'esclusione era comunque infondata.** J7 contiene **già** 11 righe dal seed 125. Ho verificato
   direttamente: le righe `[16702:16713]` del dataset J7 sono **bit-identiche** alla trace del probe
   J6 seed 125. Un seme già dentro il corpus di training **non può** essere chiamato held-out.
   J13 avrebbe dovuto controllare la composizione di J7 prima di proporre l'esclusione.

Il report J13 **non è stato modificato**: resta il record di ciò che è stato concluso e corretto.

### Le sei correzioni, recepite

| # | correzione | dove è applicata |
|---|---|---|
| 1 | il protocollo F2R del 23/08 è superato | prereg `architectural_supplement_to_j13` |
| 2 | F/125 non è held-out, e non può esserlo | `seeds.seed_125_is_held_out: false`, con la prova misurata |
| 3 | entrambe le trace fallite autorizzate, tutte le righe, post-mismatch incluso | `SOURCE_CELLS` = E 500 + F 354, `truncation.applied: false` |
| 4 | la prossima A–F è un gate di **regressione**, non held-out statistico | `future_matrix_is_a_regression_gate` |
| 5 | vero held-out = seed 126/127/128, non aperti ora | `SEALED_SEEDS`, rifiuto fail-closed, gate G/H/I definito e non aperto |
| 6 | fit futuro fresh da J2, non da J11 | `future_fit.init` |

Inoltre: FSM v3, morphology, reward, sigma, SEA/C++, architettura e produzione **invariati**; il
mancato azzeramento dello swing clock dopo `hs_cancelled` resta **osservazione diagnostica**, non
change request; frazione phase-invalid e copertura post-mismatch sono **diagnostici preregistrati**,
non nuove soglie.

---

## 2. Cosa fa questo stadio

Materializza un leaf dataset **separato e content-addressed** con le **854 righe correttive**: ogni
step che lo student J11 ha effettivamente visitato nelle due celle J12 fallite, etichettato con il
teacher J1 **allo stesso step**, con il clock proiettato a zero esatto.

**Nessun nuovo rollout.** Gli stati esistono già nelle trace J12 e le etichette esistono già in J1:
il teacher prescritto è funzione **solo del tempo**, e le griglie temporali sono **bit-identiche**.

### Il punto: nessun troncamento

J7 tronca al primo mismatch discreto e tiene solo il prefisso. **È esattamente ciò che ha lasciato
il corpus senza copertura.** Misurato qui:

| cella | seme | righe | primo mismatch | colonna | pre | post |
|---|---|---|---|---|---|---|
| E | 124 | 500 | **95** | `online_left_in_contact` | 94 | **406** |
| F | 125 | 354 | **90** | `online_left_in_contact` | 89 | **265** |

**Un troncamento in stile J7 avrebbe tenuto 183 righe su 854 e scartato 671** — l'intera regione
post-mismatch. Questo stadio le tiene tutte e registra il lato come **flag diagnostico**.

È il TODO vincolante di luglio del 14/07, mai eseguito: «includendo la regione successiva al
mismatch FSM invece di sole interpolazioni pre-evento».

---

## 3. Dove sta davvero il valore — contabilità onesta

Un audit indipendente ha sollevato che **non si deve giustificare lo stadio su «854 righe nuove»**.
Ha ragione, e il receipt lo dice esplicitamente. Misurato contro l'inviluppo di training J11
(`phase_swing_elapsed_norm` max **0.188846**, `pros_knee_angle` min **−1.009047**):

| metà | righe | swing range | knee range | oltre inviluppo |
|---|---|---|---|---|
| E pre | 94 | [0.0000, 0.0000] | [−0.4294, −0.1742] | **0** |
| **E post** | **406** | [0.0000, **0.9465**] | [**−1.2137**, −0.1799] | **211** |
| F pre | 89 | [0.0000, 0.0000] | [−0.4669, −0.1742] | **0** |
| **F post** | **265** | [0.0000, **0.9985**] | [**−1.3898**, −0.1918] | **195** |

Le metà **pre** stanno interamente dentro l'inviluppo, sono al 100 % in `stance_after_hs` e non
entrano mai in swing: informativamente **vuote**. Tutto il valore sta nelle 671 righe post, e in
particolare nelle **406** oltre l'inviluppo. Le pre-righe si tengono perché il troncamento è ciò che
questo stadio esiste per evitare, non perché portino informazione.

### Il flag non è un'etichetta di qualità — e la cella D lo dimostra

Un audit ha trovato il controesempio: nella **stessa** matrice J12, la cella **D** (seed 123,
nominale) diverge allo step **12** — molto **prima** di E (95) o F (90) — e **PASSA** il gate. Il
passo del primo mismatch non porta di per sé alcun segnale di fallimento.

Il receipt lo dichiara, il test lo verifica misurando D e confrontandolo col verdetto J12, e nulla
a valle può filtrare o pesare per quel flag.

### L'assunzione scientifica, dichiarata invece che ereditata

L'etichetta è un riferimento **open-loop indicizzato dal tempo**. Per le 671 righe post-mismatch
accoppia uno stato molto derivato con l'azione che il teacher avrebbe preso a quell'istante sulla
traiettoria nominale.

È la convenzione J7 congelata ed è DAgger-standard — il teacher dice «torna al riferimento», e in
uno swing ritardato questo spinge il piede verso il contatto. **Ma il teacher prescritto non è un
esperto in feedback di stato**, e il riferimento a tempo nominale non è necessariamente l'azione di
recupero corretta da uno stato 5× fuori dall'inviluppo di swing. Luglio misurò il 76.27 % di righe
phase-misaligned nel proprio blocco recovery e rispose col troncamento.

Il receipt la marca come **la maggiore assunzione scientifica del design**, senza alcuna soglia
attaccata. È il gate closed-loop ad arbitrarla, non questo stadio.

---

## 4. Aritmetica dell'aggregato futuro — verificata dai file

Come richiesto, i numeri sono ricalcolati dai file e lo stadio **fallisce chiuso** se non tornano:

```
J7                     16713   (letto dall'npz)
cella B x8      500 x 8 = 4000 (letto dall'npz)
cella C x8      500 x 8 = 4000 (letto dall'npz)
E                        500   (letto dalla trace)
F                        354   (letto dalla trace)
                       -------
TOTALE                 25567   verificato
```

Il test manomette un componente e il totale, e verifica che entrambi sollevino, con il messaggio che
nomina i due numeri. **L'aggregato non è costruito qui**: J14 materializza il solo incremento; il
fit futuro lo assembla e deve riverificare gli stessi conteggi.

---

## 5. Invarianti garantiti

| invariante | come è provato |
|---|---|
| nessun troncamento | `truncation.applied: false`; check AST che `first_discrete_mismatch` non affetti alcuno slice e che nessuna funzione di troncamento esista nel modulo |
| flag diagnostico, mai filtro | non è letto da nulla nel modulo; il receipt lo dichiara col controesempio D |
| time grid bit-identica | confronto `np.array_equal` riga per riga contro `J1.times`; un disallineamento di 1e-6 alla riga 3 è rifiutato nominando la riga |
| teacher same-step | `actions[step-1]`, verificato bit-a-bit per entrambe le celle; le osservazioni del teacher non sono mai lette |
| clock a zero esatto | proiezione + due asserzioni (zero esatto, colonne 2..34 intatte) + **guardia sul contratto** `CLOCK_COLUMNS == (0,1)` |
| solo semi 124/125 | rifiuto fail-closed di ogni altro seme e di ogni seme sigillato, sia nella spec sia nell'array finale |
| seed 126/127/128 mai letti | nessun path pinnato li nomina; asserito staticamente e nel receipt |
| conteggi riga esatti | E deve essere 500, F **354** e non 500; entrambi i casi rifiutati col messaggio che nomina il conteggio |
| provenienza per riga | `cell`, `seed`, `step`, `post_mismatch`, `time_before` nell'npz committato |
| commit atomico | lock esclusivo `O_CREAT\|O_EXCL`, staging fratello, rename, no-clobber |
| leaf nato invalido | marker `TECHNICAL_INVALID` scritto **nello staging prima del rename**, rimosso solo dopo una verifica passata; provato staticamente per numero di riga |
| verifica post-commit | ri-risoluzione e ri-hash di ogni path contro il **receipt committato**, più confronto coi byte staged |
| duplicati contati onestamente | misurati sulle righe **proiettate**; il test dimostra che sulle righe raw il conteggio sarebbe 0 **per la ragione sbagliata** |

### Una fragilità reale trovata dai test

Il test sul contratto del clock ha scoperto che, con `CLOCK_COLUMNS` vuoto, la proiezione diventava
un no-op silenzioso e il controllo di zero riduceva su un array vuoto, sollevando un `ValueError`
oscuro invece di un `J14Error`. **Corretto**: il runner ora asserisce `CLOCK_COLUMNS == (0, 1)`
prima di fidarsene, e il test verifica quattro varianti corrotte.

---

## 6. File creati

| File | SHA-256 |
|---|---|
| `v26c_j14_prereg_dagger_dataset.json` | `4c0720bad952f97c9cb5d68f3bd567f6cf9d9bb9b02508b42d8522268cba8717` |
| `v26c_j14_dagger_dataset.py` | `42629c18bcd8786793cdd25015363c4cd386d07b4cdd868f0b629dd6b20905d1` |
| `test_v26c_j14_dagger_dataset.py` | `f39143980a8eb8781b6da9da32fe310f8519b050a6e84190da2efe2b3bdcb6bc` |
| `v26c_j14_dagger_dataset_authorization.json` | `22d374e549e736b91f4b332042c53fc05ba6be20d64022634ec2d4de77fcbd25` |

**File modificati: nessuno.** Nessun artefatto J0–J13, nessun report esistente (J13 incluso),
nessuna sorgente July, nessuna configurazione di produzione. `git status` mostra solo i tre file già
dirty a inizio sessione.

**18 sorgenti pinnate** (J12 receipt + commit_verification + trace A/E/F, J1 dataset + receipt, J7
dataset + receipt, J10R1 B/C + receipt + commit_verification, J2 module + manifest, e i moduli
congelati J1/J7/J12). **21/21 pin dell'authorization verificati su disco.**

---

## 7. Test e preflight

**Selftest: 169 check, PASS.** Copre, con asserzioni sul **messaggio** e non solo sul tipo:

- preregistrazione e i 18 pin; identità della tabella pin fra prereg e runner;
- il supplemento J13 e la **prova misurata** che le righe `[16702:16713]` di J7 sono il probe
  seed 125;
- niente troncamento (check AST), repeat 1, niente dedup né bilanciamento;
- l'incremento: 854 = 500 + 354, solo semi 124/125, nessun seme sigillato, clock a zero **esatto**;
- primo mismatch E=95 / F=90 su `online_left_in_contact`, e i **671** scartati da un troncamento;
- copertura per metà contro l'inviluppo J11 misurato, con 0 righe pre e 211/195 post oltre;
- il controesempio **cella D**, misurato e confrontato col verdetto J12;
- il trap dei duplicati raw-vs-proiettati, dimostrato numericamente;
- mapping teacher bit-a-bit e identità della griglia temporale;
- **rifiuti**: cella sconosciuta, seme fuori insieme, seme sigillato, conteggio riga errato,
  **F dichiarata 500 invece di 354**, disallineamento temporale, contratto clock corrotto (4
  varianti), hash sorgente manomesso, marker `TECHNICAL_INVALID` su un leaf sorgente, lock stantìo
  accanto a un leaf sorgente, aritmetica futura falsificata (componente e totale);
- write path in root temporanea: rifiuto di una materializzazione autoritativa in root
  redirezionata, lock conteso fail-closed, sentinella concorrente che sopravvive, inventario esatto
  del leaf, doppia materializzazione rifiutata, manomissioni intercettate, path che evade,
  «verifica fallita → marker + leaf preservato»;
- garanzie statiche: marker prima del rename per numero di riga, `except Exception` mai
  `BaseException`, un solo `rmtree` col solo argomento `staging_created`, `O_CREAT|O_EXCL`, e che
  il runner **non nomini mai** torch, ray, PPO, optimizer o forward.

**Preflight inerte: GO**, nessun blocker. Nessuna primitiva di scrittura chiamata (verificato con
monkeypatch), **torch e ray assenti da `sys.modules`**, sentinella mai creata.

**La rivendicazione di inerzia più forte di questo stadio**: non ha bisogno di torch, ray, OpenSim o
di un ambiente in **nessun** punto — nemmeno nel percorso di materializzazione. Legge byte
committati e scrive byte committati. Un check AST verifica che il runner non nomini nemmeno `torch`.

Rifiuti di argomento verificati: `--materialize` senza stage token, e `--out` senza `--materialize`.
Nessun `j14_runs`, lock, staging o sentinella.

---

## 8. Comando futuro esatto

```
cwd:         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter: /opt/anaconda3/envs/envCMC-rllib/bin/python
argv:        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j14_dagger_dataset.py
             --materialize
             --authorized-stage V26C_J14_POST_MISMATCH_DAGGER_DATASET
             --out <repo>/…/j14_runs/j14_dagger_dataset_v26c_2026-08-27_r1
```

Nessun flag opzionale. `OUTPUT_ROOT_OVERRIDE` deve restare non impostata; il runner rifiuta comunque
una materializzazione autoritativa in una root redirezionata. Cattura di stdout/stderr/exit in una
directory additiva **fuori** dal leaf transazionale.

---

## 9. TODO propagati

- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Epic generalizzazione multi-modello** — APERTO. Ancora AB06, un trial, tre start.
- **Il fit che consumerà questo incremento** — NON parte di questo stadio e non autorizzato. Fresh
  da **J2**, iperparametri invariati rispetto a J11 (seed 123, max 400 epoche, batch 128, lr 5e-5,
  patience 60, clip 1.0, logstd congelata con peso 0, anchor 0.01, scaling fisico, full mean actor
  trainable, critic escluso).
- **La matrice A–F successiva** — gate di **regressione** sui failure mode noti, non held-out
  statistico. Va riportata come tale.
- **Gate indipendente G/H/I** — seed 126/127/128 a σ 0.005, start nominale, stessi gate per job.
  **Definito, non aperto, non autorizzato.** Un'estensione a 3 start × 3 semi (9 job) è
  tecnicamente possibile ma triplicherebbe il costo; **non la introduco senza tua decisione**.
- **Assunzione dell'etichetta open-loop** — la maggiore del design. Arbitrata dal gate closed-loop.
- **Frazione phase-invalid** — diagnostico preregistrato, senza soglia.
- **Repeat sulle righe nuove** — 1. Qualsiasi valore diverso sarebbe un numero **nuovo** e
  richiederebbe la propria preregistrazione: il fattore 8 usato altrove non ha razionale documentato
  e non può essere invocato come eredità.
- **Swing clock non azzerato dopo `hs_bounce_cancelled`** — osservazione diagnostica da J13, non
  toccata.
- **Le 183 righe pre-mismatch sono informativamente vuote** — tenute per non troncare, non perché
  contribuiscano.

---

## 10. STOP per audit Codex

**Non autorizzo la fase successiva.** Il leaf non è materializzato, l'authorization dichiara
`execution_permitted_now: false`, e nessun fit, rollout, critic o PPO è stato avviato.

**Fermo in attesa dell'audit dell'architetto.**
