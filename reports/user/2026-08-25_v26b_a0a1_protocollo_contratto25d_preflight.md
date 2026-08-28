# V26B A0/A1 — protocollo, contratto 25D, tooling e preflight

**Data**: 2026-08-25
**Preregistrazione**: `v26b_amendment_a0a1_25d.json` — SHA-256 `9dfe122925b060a708fd992768f3ed1737f37a631a3c71e0b13710b2d0764124`
**Esito**: A0 **GO** · A1 **BLOCKED** su un prerequisito dichiarato · self-test **103/103 PASS**
**Nessun fit, training, rollout, raccolta o candidato. Nessuna modifica a produzione, C++ o SEA.**

---

## 1. File creati (solo additivi)

| file | SHA-256 |
|---|---|
| `.../v26b_bridge_2026-08-24/v26b_amendment_a0a1_25d.json` | `9dfe122925b060a708fd992768f3ed1737f37a631a3c71e0b13710b2d0764124` |
| `.../v26b_contract_25d_v1.yaml` | `b7292a73a9d342dd61773ab5bc85d66b1532bc2a0d76fd4d1dad37758fa67fae` |
| `.../v26b_a0_transplant25.py` | `bce45d030f93db94de385e93720b5a1a695ff57ce53f1df2235bfdf1ffa8d593` |
| `.../v26b_a1_ik_imitation.py` | `155b0f28c7be04839c2b7e69e62ac08a87268c8f7a656766883c8a284800c845` |
| `.../test_v26b_a0a1_25d.py` | `5739ac5560aae0c44272253d02937b7774b26fd9774d20df2b49eed0c90ac8a2` |
| receipt `runs/.../diagnostics/a0a1/v26b_a0a1_preflight_receipt.json` | `4d1bec0f43008cf3768b4af8c663476864fafb08269179f08b77e605366bd745` |

`git status` conferma zero modifiche a `osim_trj_cmc_like.py`, `target_domain_imitation.py`, `target_domain_markov_adaptation.py`, `target_domain_noise_adaptation.py`, `prosthetic_phase_fsm.py`, `online_grf.py`, al config v3 risolto e a `tools/` (C++/SEA). Portabilità: solo `pathlib`, nessuna chiamata di shell, nessun separatore specifico di OS — verificato da test.

**Comandi eseguiti**
```
python test_v26b_a0a1_25d.py            -> {"selftest": "PASS", "checks": 103}
python v26b_a0_transplant25.py --preflight   -> verdict GO
python v26b_a1_ik_imitation.py --preflight   -> verdict BLOCKED (A1a assente, atteso)
```

## 2. Contratto osservativo intermedio 25D, versionato

`v26b_contract_25d_v1.yaml` è derivato dal config v3 canonico pinnato `a870cc38…` cambiando **una sola riga**, la 63:

```
include_controller_state_observation: true  ->  false
```

Verifica differenziale eseguita: stesso numero di righe, **una sola riga differente**. Fisica, FSM v3, corridoio morfologico, modello di contatto, reward, target slew limiter, reference governor e ogni guardia di sicurezza restano byte-identici al parent — il test controlla esplicitamente la presenza dei valori `grf_penetration_termination_m: 0.028`, `grf_penetration_penalty_threshold_m: 0.02`, `phase_stance_hard_timeout_s: 2.2`, `pros_ankle_target_slew_rate_limit_rad_s: 2.0`.

Effetto: le dieci feature di stato controller passano al suffisso privilegiato del critic; l'actor diventa 25D e il vettore completo resta **84 = 25 actor + 59 critic**. Il contratto è usato solo da A0 e A1; A2 ripristina il 35D.

## 3. A0 — trapianto 39D → 25D, preflight GO

Parent: **esclusivamente** l'imitation V26 agosto 39D, digest `5bbc6cbd3c7e3ec37524b7b6b69ca017af48057cac5207cf755d3b2f72c2709e`. Una guardia fail-closed rifiuta qualunque percorso contenente `S0D_35D_DISTILLED`, `S1A_`, `S1B_`, `S1C1_`, `S1C2Z_`, `REV4B/C/D/E`, `V2_` o marcatori di luglio.

Rimosse **14 colonne**, indici 39D `[2,3,4,5]` + `[29..38]`: i 4 healthy imitation target (privilegiati e anti-ex-novo, rimossi del tutto) e le 10 feature Markov di stato controller (spostate al critic).

Ricetta, che rispecchia quella congelata di V1 estesa da 4 a 14 colonne:
1. trasferimento per nome, **bit-identico** per ogni colonna conservata;
2. compensazione di bias sulla media: `b1_25 = float32(float64(b1_39) + float64(W1_39[:, removed]) @ mean_removed)`, accumulata in float64;
3. le due colonne del clock prescritto azzerate;
4. hidden layer e testa della media copiati bit-identici;
5. testa log-std sostituita dal placeholder di serializzazione.

Medie: float64 sulle **3 ancore deterministiche pinnate, 1500 righe** — healthy target dalla cache privilegiata (stessa fonte della ricetta V1 congelata), stato controller dalle obs35 delle **stesse tracce e stesse righe**.

Risultato del preflight: **GO**. Digest 25D `d2c0e8758747a522fce2825e2a28fed939bdebd6fb2f2421fb5950dc481b1a83`; `‖delta‖₂` della compensazione 1,626305, variazione massima del bias 0,327662; `n_actor` 25, osservazione 88 → 84; hidden 256; clock a zero; validazione strutturale congelata superata a `width=25`.

## 4. A1 — BLOCKED su un prerequisito dichiarato

**A1a è bloccante e non ancora autorizzato**: serve un rollout **teacher prescribed sotto contratto 25D**, prodotto dall'entry point di produzione `target_domain_imitation.collect_teacher_dataset`, che scrive `teacher_dataset.npz` e `teacher_summary.json` con il proprio `gate_pass`.

Il motivo è la tua stessa regola: il corpus da 500 righe esistente è **visitato da S0D**, e S0D è evidenza diagnostica, mai dato di questo ramo. A1 quindi non ha alcun dataset ammissibile finché A1a non esiste, e il tool fallisce chiuso dicendolo.

**Split fail-closed, mai casuale**: per traiettoria quando ce ne sono almeno due, altrimenti a blocchi contigui temporali con embargo simmetrico (5 fold, embargo 10 step, validazione 0,20). Il test verifica che nel modulo non compaiano né `permutation` né `shuffle`. Razionale registrato: gli step consecutivi di una traiettoria sono fortemente dipendenti, e uno split casuale fa entrare in training i vicini di ogni riga tenuta fuori.

**Invarianti A1**: log-std byte-identica all'init A0, colonne clock a zero, nessun addestramento del critic, nessun update PPO, save/reload esatto.
**Gate offline vincolanti**: invarianti di integrità inclusa la larghezza 25; convergenza del fit (RMSE aggregato strettamente inferiore all'iniziale). Le metriche IK sono riportate **senza soglia inventata**.

## 5. Gate closed-loop di A1, preregistrati e motivati

| gate | valore | motivazione |
|---|---|---|
| completamento | 500/500, `episode_time_limit` | stessa asticella di ogni stadio precedente |
| cicli validi | **≥ 2** | asticella di promozione già in vigore; 1 ciclo si è dimostrato insufficiente |
| contatori critici a zero | `phase_timeout_stance`, `phase_timeout_swing`, `morphology_causal_contract_failure`, `hs_cancelled_count` | contratto v3 invariato |
| resync | ≤ 1 | invariato |
| penetrazione | ≤ **0,020 m** | la guardia soft v3 **corrente**, non allentata |
| **caviglia q_min** | **≤ −0,03 rad** | il 19,3% della profondità di plantarflessione di riferimento (0,1552 rad). **−0,0099 NON la soddisfa**: −0,0099 > −0,03, ed è il 33% della soglia in modulo |
| ampiezza caviglia | ≥ 0,30 rad | 54,0% dell'ampiezza di riferimento 0,5553; esclude un giunto che oscilla attorno a un offset fisso |
| ampiezza ginocchio | ≥ 0,60 rad | 70,4% dell'ampiezza di riferimento 0,8525; un ginocchio che cammina percorre gran parte del range |
| ginocchio sempre flesso e bounds | q < 0 ovunque, 0 step fuori bounds | contratto cinematico |

Riferimento AB06 protesico letto diagnosticamente sul corpus congelato: knee min −1,0165 / max −0,1640 / ampiezza 0,8525; ankle min −0,1552 / max +0,4002 / ampiezza 0,5553, con il 19,40% di righe in plantarflessione.

**Calibrazione su attori esistenti (solo diagnostica, mai dato o parent)**:

| attore | ankle min | ampiezza ankle | ampiezza knee | gate cinematici falliti |
|---|---|---|---|---|
| S0D | +0,0153 | 0,3941 | 0,8224 | `ankle_plantarflexion` |
| S1A | −0,4231 | 0,6683 | 0,6135 | **nessuno** |
| REV4E | −0,0440 | 0,4587 | 0,2968 | `knee_amplitude` |

S1A è l'unico che supera tutti e cinque i gate cinematici, ma fallisce il completamento (392 step, `phase_timeout:swing`). **Nessun attore esistente supera l'insieme completo**: le soglie discriminano e non sono banalmente soddisfacibili.

## 6. A2 e A3, dichiarati

**A2**: ripristino del 35D aggiungendo le dieci colonne del primo layer **inizializzate a zero**, ogni altro peso invariato. Gate: il rollout deterministico deve riprodurre quello di A1 **bit-per-bit** — stessi step, stessi cicli, stessa traccia. Un solo bit di differenza è un fallimento, mai arrotondato. Nessun adattamento può iniziare prima che quel gate passi.

**A3**: masse relative di luglio come preregistrazione iniziale (ancore proprie 64,7% / recovery same-actor 2,9% / multistart 32,4%), adattate ai conteggi reali; le ancore etichettate con **le medie dell'attore stesso**; le recovery troncate al primo mismatch discreto **contro la propria traccia nominale**; nessuna interpolazione. La raccolta ±0,20 s resta autorizzata solo dopo i gate di A1 e A2.

## 7. TODO sigma — conservato e reso operativo

**σ = 0,005 NON è assunto.** Prima di qualunque recovery stocastica per A3, la scala di esplorazione dell'attore A1 va **misurata**, confrontando l'RMS empirico del rumore d'azione per giunto fra rollout stocastici e la nominale dello stesso attore. Precedente registrato: il 13/07 tracce etichettate `sigma003` portavano un rumore empirico di circa 0,03 e furono scartate; la traccia corretta misurò 0,005103 e 0,004554. Uno scostamento fra placeholder serializzato e rumore misurato invalida il dataset di recovery prima che venga costruito.

## 8. Comandi futuri esatti

Nell'ordine, ciascuno da autorizzare separatamente:

```
# A0 - materializzazione del trapianto 25D (attualmente solo preflight)
python v26b_a0_transplant25.py --authorized-stage V26B-A0-TRANSPLANT-25D

# A1a - raccolta teacher sotto contratto 25D (prerequisito bloccante)
#   entry point di produzione: target_domain_imitation.collect_teacher_dataset
#   config: v26b_contract_25d_v1.yaml  (sha b7292a73...)
#   output atteso: runs/.../datasets/A1A_TEACHER_25D/{teacher_dataset.npz, teacher_summary.json}

# A1 - fit di imitazione IK a 25 feature
python v26b_a1_ik_imitation.py --authorized-stage V26B-A1-IK-IMITATION
```

Il terzo comando **rifiuta di eseguire** finché il token non è emesso e A1a non esiste: è verificato da test.

## 9. Ambiguità che segnalo

1. **A1a è una raccolta**, quindi fuori dal perimetro di questo stadio. L'ho dichiarata come prerequisito bloccante invece di sostituirla con dati S0D.
2. **Il contratto 25D introduce un secondo insieme di pin di runtime** accanto a `v3_canonical`. È tracciato per SHA e dichiara il parent, ma resta da decidere come registrarlo nei receipt futuri.
3. **La compensazione di bias è una scelta di disegno** ereditata da V1, non un requisito di luglio: luglio riaddestrava e basta. Sposta il bias fino a 0,327662, quindi il 25D iniziale non è funzionalmente equivalente al V26 — come atteso, dato che le 25 colonne residue non sono autosufficienti (misurato nel supplemento precedente).
4. **Le soglie di ampiezza** 0,30 e 0,60 rad sono mie proposte motivate su frazioni del riferimento; la soglia di caviglia −0,03 è la tua. Sono preregistrate e modificabili solo da te prima di A1.

## 10. TODO propagati

- **TODO-2** — σ = 0,005 non assunto: **reso operativo** al §7 come verifica obbligatoria prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota a copertura piena. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura come terminatore di REV4E. *(aperto)*
- **TODO-9** — Righe di swing ammesse per raggiungibilità al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo ereditato dalla sorgente V26 al 71,25%. *(aperto, indirizzato da A0)*
- **TODO-11** — Nuovo: il contratto 25D crea un secondo insieme di pin di runtime da registrare nei receipt. *(nuovo)*
