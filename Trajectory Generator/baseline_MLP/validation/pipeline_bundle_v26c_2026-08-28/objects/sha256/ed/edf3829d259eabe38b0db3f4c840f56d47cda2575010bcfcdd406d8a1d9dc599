# V26B S1A (rev3r) — rollout nominale deterministico: **392/500, `phase_timeout:swing` → NON IDONEO** come source==init

**Token:** `V26B-S1A-NOMINAL-ROLLOUT` · **Data:** 2026-08-24 · **Esecuzione unica**, exit code 3, durata 378,6 s. Nessun retry, nessun tuning, nessuna promozione, nessuna ancora costruita. Candidato S1A byte-immutabile prima e dopo.

## 1. Esito
Il candidato **non supera 4 dei 7 gate vincolanti**. Non è idoneo come futura source==init per la raccolta ancore in stile luglio.

| Gate vincolante | Osservato | Richiesto | Esito |
|---|---|---|---|
| completion | **392/500**, `phase_timeout:swing` | 500/500, `episode_time_limit` | **FAIL** |
| phase_timeout (tutte le righe) | stance 0, **swing 1** | 0 e 0 | **FAIL** |
| morphology_causal_contract_failure (per riga) | rows_positive 0, max 0.0, failure false | 0 / 0 / false | PASS |
| hs_cancelled_count | max 0, final 0 | 0 e 0 | PASS |
| resync_count | max 0, final 0 | ≤ 1 e ≤ 1 | PASS |
| valid_cycle_count | **0** | ≥ 1 | **FAIL** |
| penetrazione | **0,024663 m**, nessuna terminazione `grf_penetration` | ≤ 0,020 m e nessuna terminazione | **FAIL** |

## 2. Causa: un solo toe-off, poi 2,6 s di swing senza mai un heel strike valido
La traccia preservata dà una timeline FSM con **sole tre transizioni**:

- step 1–131 `STANCE_AFTER_HS` (131 righe di stance iniziale);
- step 132 toe-off valido → `SWING_AFTER_TO`;
- step 132–391 **260 righe consecutive in swing** (2,6 s) senza mai registrare un heel strike valido;
- step 392 `TIMEOUT`.

Contatori finali: **valid_hs 0**, valid_to 1, cicli 0, invalid_event 0, resync 0, hs_cancelled 0. I meccanismi FSM e morphology sono **puliti** (nessun contract failure per riga; `dropped_wait_hs` max 1, `timeout_transition` 1, `terminal_flushed` 1, tutti coerenti con la chiusura per timeout): **non è un guasto dei meccanismi, è l'assenza del ciclo**. Il picco di penetrazione (24,66 mm, sopra il limite di idoneità 20 mm ma sotto la guardia hard 28 mm) cade allo **step 81**, cioè durante la stance iniziale, prima ancora del toe-off.

**Perché la gamba non si riappoggia.** Rispetto al target prescritto sulla griglia: la caviglia ha **offset medio −0,341 rad** e **ampiezza 1,76×** quella prescritta, con accordo di segno appena 0,36 e r di Pearson 0,246; il ginocchio è **sempre negativo** (frazione 1,0), ampiezza 0,672× e r 0,201. Nelle ultime 20 righe il comando grezzo medio della caviglia è **−0,709**: la policy mantiene una configurazione di piede che non produce l'evento di contatto atteso. Azioni grezze fuori banda su 0,5 % (knee) e 3,8 % (ankle) delle righe, 16 righe clippate; reserve max **1038,6 Nm** (oltre il livello report di luglio ≤ 1000 per start), return **−413,4**. B3 **non valutabile** (campo `pros_ankle_angle_imitation_target_phase` identicamente 0, 0 righe nella finestra [0,55, 0,80]): stessa semantica registrata per S0D e R2I, mai sostituita dal minimo globale (il minimo caviglia complessivo, −0,423, è riportato solo come diagnostico distinto).

## 3. Confronto con S0D, la catena e luglio
| Attore | Esito nominale | Cicli | Penetrazione max | Note |
|---|---|---|---|---|
| **S0D** | **500/500** `episode_time_limit` | 2 | 13,7 mm | unico attore della catena con evidenza closed-loop |
| **S1A (questo)** | 392/500 `phase_timeout:swing` | **0** | 24,7 mm | offline G_task 0,134 PASS |
| R0a | 493/500 `joint_divergence` | 2 | — | BC mono-ruolo da V1 |
| R1 | 242/500 `grf_penetration` | — | — | DAgger round 1 |
| R2I | 197/500 `grf_penetration` | 0 HS | — | role-mass-weighted |
| July 11/07 clone BC | 68/500 | 0 | 25,2 mm | «covariate shift del behavior cloning» |
| July 11/07 DAgger r2 | 356/500 | 1 | — | miglior attore della fase BC di luglio |
| July 13/07 markov35 | 500/500 ×3 start | 2 | ≤ 24,6 mm | init che **già camminava**; shift nominale RMS 0,004175 |

S1A sopravvive più a lungo di R2I e persino del DAgger r2 di luglio, ma con **zero cicli validi** contro l'unico ciclo di r2. È la **quarta conferma** in questa catena che un gate offline superato (G_task 0,134/0,122 out-of-sample) non predice la viabilità closed-loop — esattamente ciò che rev3q aveva messo agli atti come estrapolazione e che il 13/07 aveva già dimostrato rigettando un refinement con offline migliore.

## 4. Conseguenza strutturale per lo stadio successivo
Il gate d'ingresso alla raccolta ancore **non è superato**, e per la ragione precisa anticipata nella review documentale: la costruzione ancore del 13/07 richiede la traccia nominale **completa dell'attore stesso** (500 step × `np.tile` 32, source==init provato float32 rmse 1,19e-07). Con 392 step quella traccia **non esiste**, esattamente come per R0a. In più, con 0 cicli validi, costruire DAgger o preservazione su S1A significherebbe preservare un comportamento che non cammina — la spirale documentata 493 → 242 → 197. **Nessuna ancora è stata costruita** (`anchors_not_built: true` nel receipt).

## 5. Conformità
Comando **byte-identico** a quello congelato in rev3r (shape rev3c/e/j, `--no-auto-config`, config v3 `a870cc38…`, offset 1.956870983805102, deterministic, seed 123, `--record-outputs --record-policy-trace`, `--step-timeout-s 900 --startup-timeout-s 900`); pin di produzione verificati (`rollout_eval.py` `5433bcbc…`, YAML v3 `a870cc38…`, corridor `33b1dd7c…`). Nessuna sigma operativa: media deterministica; 0,005 resta placeholder irrisolto. **Candidato immutato**: digest dei 4 file del modulo e del receipt verificati identici **prima e dopo** il rollout, flag `deployable:false` / `rollout_pending:true` / `sigma_unresolved:true` mai toccati (registrati in `candidate_before` e `candidate_after`). Output solo nella nuova job dir sibling, nulla sotto `student/`, nessun alias o promozione. Tutto preservato: traccia, summary, log, receipt.

## 6. Test
`test_v26b_s1a_rollout.py`: **PASS 49 check pre-run**, **PASS 57 post-run** (stage-aware). Copertura: lineage rev3l→rev3r con tamper su tre pin e sui digest dei file del candidato; candidato read-only e non-deployable con **verifica che il controllo dei flag è vivo** (un'aspettativa `deployable:true` viene rifiutata contro l'artefatto reale); comando byte-identico a rev3r più assert strutturali su ogni flag prescritto; **6 token negativi** (inclusi `V26B-S1A-BC-FIT` e `V26B-S0D-NOMINAL-ROLLOUT`) e token positivo provato raggiungendo la guardia no-clobber; **exactly-one-launch** verificato prima e dopo (il secondo lancio è rifiutato); parser whole-trace su fixture con eventi a metà traccia invisibili nell'ultima riga (stance/swing timeout, morphology per riga, max≠final dei contatori); B3 su fixture (not_evaluable con campo nullo, sola finestra [0,55, 0,80]); **fixture isolata per ciascuno dei 7 gate** più il caso combinato `grf_penetration`; assenza di routine di costruzione ancore/DAgger e di qualunque marcatura deployable introdotta dal driver.

**Trasparenza:** due miei errori di costruzione nei test, trovati e corretti **prima** del lancio: un fixture che si aspettava un solo gate fallito per una terminazione `grf_penetration` (il driver ne fa fallire correttamente due) e uno scan a sottostringa troppo grezzo che colpiva un'etichetta documentale («11_07_DAgger_r2») invece di una chiamata. Nessuna soglia toccata, nessun codice di produzione coinvolto.

## 7. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3r_s1a_nominal_rollout.json` | `b20306919d884529bbabbb37b29da3d67a66a1bf1c2b7d2884b583f2a426556c` |
| `…/v26b_bridge_2026-08-24/v26b_s1a_rollout.py` | `f66d7028a15528bb8ad49c1dd463b5302d309ea30320246dd5ea3dafae2fb6a2` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1a_rollout.py` | `aef340323e9f8dbf14cfa54efc2521f20efb2dfca275e3ce265d0e1cce6dcdec` |
| `…/rollouts/s1a_nominal_det/S1A_35D__v3_canonical__nominal__det/v26b_s1a_rollout_receipt.json` | `43eeac0747b5b94904bf8c45dd4e8c4a392ff9fb269e0c89a3f03b27dd7ec72e` |
| `…/S1A_35D__v3_canonical__nominal__det/rollout_policy_trace.json` | `6546befcd4a2e26711137a807cd43a797e47abd62500243fe6c015d7abcfcf21` |
| `…/S1A_35D__v3_canonical__nominal__det/rollout_summary.json` | `46f755f3389e758eb5bfe1158855c9db028cdcb341c7cf5d09204b19f7c252c9` |
| `…/logs/s1a_nominal_det_rollout.log` | `4afe270aed8e75fc977b823693ab8852fabf8aada2fea11e58725e97a903be02` |

## 8. Stato
**STOP per audit Codex.** S1A resta un candidato intermedio non-deployable, ora anche **non idoneo** come source==init. S0D (`481dd0d2…`, 500/500) resta l'unico attore della catena con evidenza closed-loop. Nessuno stadio successivo è eseguibile senza una tua decisione: la raccolta ancore è bloccata dal gate, e ogni altra via richiede un nuovo token.
