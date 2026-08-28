# V26C — J21: preparazione dell'attestazione di aggregazione delle evidenze (training-ready)

**Data:** 2026-08-28
**Stadio:** `V26C_J21_TRAINING_READY_ATTESTATION`
**Fase:** **PREPARAZIONE SOLTANTO — NIENTE È STATO ESEGUITO**
**Foglia di destinazione:** `…/j21_runs/j21_training_ready_attestation_v26c_2026-08-28_r1/` — **assente**; `j21_runs/` non esiste
**GO:** **solo DRAFT, doppiamente inerte** — `status: "DRAFT"`, `authorises_execution: false`
**Preflight:** **READY — 15/15 gate, 129/129 pin**

---

## 1. Cosa attesterebbe J21, e cosa no

La conclusione target, **solo se tutti e quindici i gate passano**, è che il
checkpoint **già esistente** del warm-up J20 è **`TRAINING_READY` come
checkpoint di INPUT** per un futuro pilota PPO conservativo.

**Non è il permesso di lanciare nulla.** È una dichiarazione su un **artefatto**,
non l'autorizzazione di un'**azione**.

L'oggetto è nominato per hash, non per percorso:

| | |
|---|---|
| directory | `j20_runs/j20_critic_warmup_v26c_2026-08-27_r1/checkpoint_last` (24 file) |
| `module_state.pkl` | `57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168` |
| `learner/state.pkl` | `51d97ef016aea2086201b55db807f3273b76784bbafdea6cc9e5c7fea90294be` |
| digest attore | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| digest critic | `2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410` |

### Semantica finale obbligatoria — su PASS **e** su FAIL

```
training_ready            true SOLO su PASS      training_started        false
promotion  TRAINING_INPUT_ONLY su PASS, NONE     training_iterations_run 0
launch_authorized         false                  sampling                false
next_stage_authorized     false                  rollout                 false
ppo_authorized            false                  ray_started             false
ex_novo_authorized        false                  environment_constructed false
deployable                false
```

Il test `D09` verifica che i due esiti differiscano **soltanto** in
`training_ready` e `promotion`: tutto il resto è falso in entrambi i casi.
`D07` verifica che `deployable` non sia **mai** vero.

## 2. Il punto delicato: checkpoint readiness ≠ pilot readiness

**Il protocollo del pilota NON è sigillato.** Al momento in cui questo pacchetto
è stato scritto **non esiste**, non è pinnato e non è approvato alcun file di
configurazione del pilota né alcun comando di lancio.

L'aggregatore **non li inventa** — sarebbe fabbricare evidenza per una cosa che
non esiste. Il test `E07` verifica staticamente che il sorgente non contenga
`pilot_cfg`, `pilot_command` né `--iterations`.

**Perché `training_ready` non diventa per questo una parola vuota.** Lo scope è
una **lista chiusa** di proprietà di un artefatto identificato per hash, ognuna
misurata da evidenza già committata:

`training_ready: true` **asserisce**: il checkpoint esiste ed è byte-identico a
quello scritto da J20; si ricarica attraverso il percorso di produzione reale
`train_ppo_mlp.run → algo.restore_from_path` e torna esatto (modulo a 16 chiavi,
attore, critic, log-std, σ, sei coppie di momenti Adam, step, e — sotto rev7 —
il learning rate prodotto dal restore); l'attore è quello J19A bit-esatto,
eleggibile su 11 criteri binding e riproducibile su 36 campi esatti; ha
qualificato 6/6 closed-loop e 3/3 held-out con attore invariato; nessuna cella
ha violato la soglia binding di penetrazione; ogni contatore FSM v3 e morfologia
è zero; il critic è stato addestrato e cambiato mentre attore e log-std
restavano bit-esatti, su una sola iterazione logica di 4096 step con i pesi
worker sincronizzati una volta.

`training_ready: true` **NON asserisce**: che una qualsiasi configurazione di
pilota sia corretta, sicura o sigillata; che un comando di lancio sia corretto o
approvato; che iperparametri, numero di iterazioni, criterio di arresto,
monitor di sicurezza o piano di rollback esistano; che la policy risultante sia
deployable o adatta a un soggetto; nulla su luglio come baseline operativa.

Un pilota futuro avrà bisogno di **propria preregistrazione, proprio GO e
propria autorizzazione utente**. Quello che **non** dovrà rifare è discutere se
questo checkpoint è sano.

## 3. Lineage

**Operativa:** `August V26 imitation → J2 35D → J8 → J18 c13 → J19A → J19B /
J19C → J20 critic-only warm-up → J20 R3 restore audit`. Studente: **un solo
attore nativo 35D**.

**Luglio: solo informativo.** Non è input operativo di nessuno stadio aggregato.
Il gate T1 verifica che nessun artefatto di luglio compaia fra i 131 input
pinnati, che il genitore registrato da J19A sia la foglia J18, e che il manifest
delle feature 35D sia pinnato.

## 4. I quindici gate

| Gate | Cosa vincola | Esito preflight |
|---|---|---|
| T1 | lineage agosto operativa, luglio informativo | **PASS** |
| T2 | J19A: riproducibilità 36/36, eleggibilità 11/11 binding | **PASS** |
| T3 | J19B: 6/6 A–F comportamentali e telemetriche, attore invariato | **PASS** |
| T4 | J19C: 3/3 G–I held-out, attore invariato | **PASS** |
| T5 | contratto penetrazione 20/25/28 mm, nessuna violazione hard | **PASS** |
| T6 | contatori FSM v3 e morfologia tutti zero | **PASS** |
| T7 | K1R1 gradiente non degenere 12/12 | **PASS** |
| T8 | warm-up 12/12, attore e log-std bit-esatti, critic cambiato, 4096 step, worker sync, checkpoint completo | **PASS** |
| T9 | R3 13/13, `g9_closed: true`, marker corretti | **PASS** |
| T10 | ottimizzatore vivo esatto sotto rev7, learning rate gateato | **PASS** |
| T11 | foglia warm-up byte-immutabile, `RESTORE_AUDIT_PENDING` intatto | **PASS** |
| T12 | 24 file di checkpoint presenti e pinnati, meta iterazione 1 | **PASS** |
| T13 | runtime, config e sorgenti pinnati (129) + verifica profonda J19B/J19C | **PASS** |
| T14 | J21 non costruisce runtime e non addestra — auto-audit AST | **PASS** |
| T15 | nessuno stadio a monte ha mai rivendicato readiness | **PASS** |

Dettagli misurati rilevanti: `weights_seq_no = 1.0` (worker sync), 4096 step,
`mean_kl_loss = 0.0`, indici Adam `[6..11]`, param group 12, 16 chiavi di modulo,
σ `0.004999999670722372` su entrambe le dimensioni, `logical_iteration = 1`.

**T13 verifica in profondità** anche i 132 file della foglia J19B e i 66 della
J19C tramite le mappe `committed_files_sha256` delle rispettive receipt (già
pinnate): 198 file ri-hashati, non solo le receipt.

## 5. Diagnostiche accettate — dichiarate, non nascoste

Sono scritte nella **preregistrazione prima di qualunque esecuzione** e
riprodotte dall'aggregatore nel risultato.

| cella | max penetrazione (m) | > 20 mm | ≥ 25 mm (di 500) | > 28 mm |
|---|---|---|---|---|
| A | 0.024421 | sì (105) | 0 | **0** |
| B | 0.023473 | sì (98) | 0 | **0** |
| C | 0.022555 | sì (90) | 0 | **0** |
| D | 0.024682 | sì (101) | 0 | **0** |
| E | 0.024457 | sì (100) | 0 | **0** |
| **F** | **0.025615** | sì (100) | **4** | **0** |
| G | 0.024417 | sì (106) | 0 | **0** |
| **H** | **0.025684** | sì (102) | **4** | **0** |
| I | 0.024763 | sì (100) | 0 | **0** |

- **Tutte e nove le celle superano la banda soft di 20 mm.** Diagnostico, mai
  bloccante: il contratto stesso dichiara `binding: false`, e l'aggregatore
  **verifica quella dichiarazione** invece di fidarsene.
- **J19B-F e J19C-H raggiungono la banda luglio di 25 mm**, 4 campioni su 500
  ciascuna. Diagnostico, mai bloccante — e luglio è comunque solo informativo.
  Il gate fallisce se una **terza** cella entra in quella banda: la lista è
  chiusa a F e H.
- **Nessuna cella supera i 28 mm.** È **l'unica soglia binding**, e passa.
  Semantica: `binding_pass` se max ≤ 0.028; fallisce se strettamente > 0.028.

Altre diagnostiche dichiarate: i due criteri J19A `G1` (0.046971) e `G2`
(0.019820) sono **diagnostic only** e registrati, mai gateati; la regione target
post-485 (rmse 0.363, max_abs 0.958) è diagnostica per costruzione; il round
trip pesi ha 58 voci su 8960 non bit-identiche con delta massimo 1.86e-09,
dichiarato non-gate da J19A.

## 6. La foglia warm-up resta byte-immutabile

`RESTORE_AUDIT_PENDING` è ancora al suo posto e al suo hash. La chiusura di G9
si legge **dalla receipt R3** (`g9_closed: true`), **mai** scrivendo in quella
foglia. Il gate T11 lo misura, e il test `F05` verifica staticamente che **ogni**
scrittura dell'aggregatore sia radicata sulla propria foglia `leaf`, quindi non
può raggiungere né la foglia warm-up né alcun'altra.

## 7. Sicurezza dell'aggregatore

Solo libreria standard: `argparse`, `ast`, `hashlib`, `json`, `pathlib`, `sys`,
`datetime`. **Nessun** `train_ppo_mlp`, `ray`, `torch`, `numpy`, `gymnasium`,
`opensim`, `simulation_runner`, `model_loader`, `rollout_eval`, `subprocess`.

Verificato in tre modi indipendenti:

1. **statico** nella suite (AST su import e chiamate);
2. **auto-audit a runtime** — il gate T14 analizza il proprio sorgente e
   fallisce lo stadio se un import o una chiamata proibita compare;
3. **empirico** — dopo un `evaluate()` completo, i moduli pesanti caricati in
   `sys.modules` sono **NESSUNO**.

`--preflight-only` e `--dry-run` non scrivono nulla. `--execute` esige un GO con
`status` esattamente `APPROVED` e stadio esattamente
`V26C_J21_TRAINING_READY_ATTESTATION`, rifiuta i flag proibiti (inclusi
`authorises_training`, `authorises_launch`, `authorises_deployment`) e richiede
**131** pin correnti.

## 8. File creati

| file | sha256 |
|---|---|
| `…/v26c_j21_prereg_training_ready_attestation.json` | `df178ffae521aaf46837258ce2036789c6f8ae9f6b4cfb27b4cd11b85d06da79` |
| `…/v26c_j21_training_ready_attestation.py` | `4cef5965c84abe519bb9f80f4a2a51df3c20d82f1e4ad45960b9d49ffa714cc9` |
| `…/test_v26c_j21_training_ready_attestation.py` | `ce8f608f836adb9e1335ac96048029469afc6a80404a1490e5e9ced9c7dd4be9` |
| `…/v26c_j21_training_ready_go_DRAFT.json` | `b2b4f6014e2fdd43dc9ca0714f24311ee0ba67d3d20ec79488531182cd10716c` |
| `reports/user/2026-08-28_v26c_j21_preparazione_attestazione_training_ready.md` | questo report |

Tutti sotto `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`.

**Nessun file preesistente è stato modificato.** R1, R2, R3, la foglia warm-up,
il checkpoint, le sette preregistrazioni J20, i due DRAFT R3, il GO operativo R3
e ogni file di produzione sono invariati; il preflight li ri-hasha tutti a ogni
esecuzione (129/129).

**Ancoraggio dei pin:** il GO operativo R3 (`fe090682…`) porta una mappa chiusa
di **92** artefatti. J21 lo pinna per hash e **ri-verifica ogni etichetta che
nomina**, ereditando l'intera chiusura senza ricopiare 92 letterali. Aggiunge
**36** artefatti che il GO R3 non copre (contratto penetrazione e modulo, foglia
R3, i 19 file di checkpoint restanti, i due DRAFT, il GO stesso) più la propria
preregistrazione: **129** verificati per hash. Il GO deve pinnarne **131**,
perché aggiunge l'aggregatore e la sua suite — che non possono contenere il
proprio hash e sono quindi pinnati solo dal GO.

## 9. Test e preflight — risultati esatti

| comando | risultato |
|---|---|
| `python test_v26c_j21_training_ready_attestation.py` | **216/216 check passati** |
| `python v26c_j21_training_ready_attestation.py --preflight-only` | **READY — 15/15 gate, 129/129 pin**, destinazione assente |
| `python v26c_j21_training_ready_attestation.py --dry-run` | piano stampato, **nessuna scrittura** |
| `validate_go` sul DRAFT | **`valid: false`**, rifiutato su entrambi i conteggi |

Eseguiti come:

```
cd "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python <file>
```

Copertura della suite: **A** aggregatore read-only (statico + T14 + empirico);
**B** ogni gate non vacuo — **oltre 90 mutazioni distinte**, una per ciascun
modo di fallimento, ognuna che ribalta il proprio gate; **C** un campo
**mancante** è un fallimento e mai un default, incluso il caso insidioso in cui
`False` o `0` presenti non vengono confusi con l'assenza; **D** semantica finale
su PASS e su FAIL; **E** scope di readiness e protocollo pilota non sigillato;
**F** immutabilità della foglia warm-up e radicamento delle scritture; **G**
preflight e GO; **H** diagnostiche accettate riprodotte e dichiarate.

## 10. Comando di esecuzione proposto — **NON eseguito**

Da eseguire **solo** dopo revisione di Codex, un GO operativo `APPROVED` e
un'autorizzazione esplicita dell'utente:

```
cd "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
    v26c_j21_training_ready_attestation.py --execute \
    --go-file v26c_j21_training_ready_architect_go.json
```

Il file `v26c_j21_training_ready_architect_go.json` **non esiste**: esiste solo
il DRAFT inerte. Non l'ho materializzato e non l'ho auto-autorizzato.

## 11. Preoccupazioni che potrebbero impedire un'attestazione veritiera

1. **Il protocollo del pilota non è sigillato.** Se l'architetto intende
   `training_ready` come «pronto a lanciare», questa attestazione **non lo
   dice** e non deve essere letta così. Dice che il checkpoint è un input sano.
   È la distinzione più importante del pacchetto.
2. **Nove celle su nove superano la banda soft di 20 mm**, e due raggiungono i
   25 mm. Sotto il contratto pinnato nessuna è bloccante e nessuna si avvicina
   ai 28 mm, ma un pilota conservativo che addestri più a lungo potrebbe
   spostare quella distribuzione. J21 attesta lo stato **attuale**, non la sua
   stabilità sotto training.
3. **Il critic ha visto 4096 step, una sola iterazione.** `vf_explained_var`
   0.8018 è incoraggiante ma è una singola misura su un warm-up minimo. J21 non
   attesta che il critic sia *buono*, solo che è stato addestrato, è cambiato, e
   si ricarica esatto.
4. **La qualificazione closed-loop è a σ = 0.005**, con 500 step per cella e 2–3
   cicli validi. È una finestra stretta; le nove celle non sono un test di
   robustezza esteso.
5. **T13 eredita 92 pin dal GO R3.** Se quel GO fosse mai riemesso con una mappa
   diversa, J21 lo rileverebbe (il GO è pinnato per hash), ma l'ereditarietà è
   una scelta di design che vale la pena che l'architetto confermi.
6. **`environment_constructed: false` è vero per J21**, non per la storia a
   monte: R3 e J19 hanno costruito environment. Il campo descrive **questo**
   stadio, ed è etichettato come tale nel risultato.

Nessuna di queste impedisce l'attestazione **come è definita**. La prima è quella
che, se fraintesa, la renderebbe falsa.

## 12. TODO

- [ ] Revisione di Codex su preregistrazione, aggregatore, suite e DRAFT GO.
- [ ] Se approvato: GO operativo `APPROVED` **separato** + autorizzazione
      esplicita dell'utente, poi **una sola** esecuzione.
- [ ] **Dopo** un eventuale PASS: preregistrare il pilota PPO conservativo —
      config, comando, iterazioni, criterio di arresto, monitor di sicurezza —
      che J21 deliberatamente **non** inventa.
- [ ] Le tre osservazioni «registrato ma non gateato» di rev7 §`correction_6`,
      ancora aperte.
- [ ] Decisione sul marker `RESTORE_AUDIT_PENDING` della foglia warm-up.
- [ ] Decidere sui due `gcs_server` orfani del 18/08, tuttora **non toccati**.
- [ ] I 2 check `B06`/`I07` della suite R1 e i 4 della suite R2 restano rossi per
      assenza-destinazione post-esecuzione: annotati in rev5 e rev6, non corretti.
- [ ] TODO ereditati: generalizzazione multimodello (epic 22/08); gate finale di
      recupero AB06.

---

**Stato conclusivo:** pacchetto J21 additivo preparato e verificato, **216/216**
check e preflight **READY 15/15 gate, 129/129 pin**. L'aggregatore è
read-only, standard-library-only, e non carica alcun modulo pesante. **Nessuna
esecuzione, nessuna foglia J21, solo un GO DRAFT inerte.** Tutti gli artefatti
precedenti sono preservati e ri-hashati. Nulla è stato addestrato, campionato o
promosso, e `training_ready` resta **falso** finché l'aggregazione non viene
autorizzata ed eseguita.
