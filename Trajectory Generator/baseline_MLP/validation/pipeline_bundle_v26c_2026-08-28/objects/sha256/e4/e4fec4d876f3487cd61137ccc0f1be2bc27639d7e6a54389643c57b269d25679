# V26C J19C — Esecuzione della fase held-out G/H/I

**Data**: 2026-08-27
**Stadio**: `V26C_J19C_J19A_HELDOUT_G_I` — **esecuzione**
**Esecutore**: Opus 5, effort xhigh — braccio operativo. Codex architetto e gate owner.

## VERDETTO: `PASS` — 3/3 celle, 42/42 gate

L'attore J19A regge sui tre semi tenuti sigillati per l'intera catena. È l'ultima prova
preregistrata del percorso.

**Nessuna fase successiva avviata. Nessun critic, nessun PPO, nessun fit.**

---

## 1. Esecuzione unica

| | |
|---|---|
| GO architetto | `c96f4b5555d9680c8896c7c2e8282630d8ffaf9d834f151a23ec9a07796c3781` |
| pin verificati | **10/10** prima del lancio |
| `j19c_runs` prima del lancio | **assente** |
| esecuzioni | **una sola** |
| exit code | **0** |
| retry | **nessuno** |

Comando:

```
PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
    v26c_j19c_heldout_g_i.py --run \
    --authorized-stage V26C_J19C_J19A_HELDOUT_G_I \
    --out j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1
```

Stack **production**, `injected: false`. Fail-fast **solo tecnico/integrità**: un FAIL comportamentale
avrebbe lasciato proseguire le celle rimanenti. Non si è verificato.

## 2. Leaf e lineage

**Leaf**: `j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1/` — **un solo leaf**, nessun altro
contenuto in `j19c_runs`.

**Lineage operativa**: **August V26 imitation → J2 35D → J8 → J18 c13 → J19A → questa
qualificazione**. **July è informativa soltanto**: nessun checkpoint, dataset o label di luglio è mai
stato un input operativo di questa catena.

Evidenza d'ingresso pinnata e verificata: **J19B `PASS` su 6 celle**, con receipt e commit
verification ai loro hash. Preregistrazione sigillata `54a1e6fc1f469ba169c80cc851be3fb1ab551e399553ed26c58aa1f101349e2d`.

---

## 3. Verdetto aggregato

| | |
|---|---|
| verdetto | **PASS** |
| comportamentali | **3 / 3** |
| telemetry-valid | **3 / 3** |
| regola | PASS se e solo se 3/3 comportamentali **E** 3/3 telemetry-valid |
| gate | **42 / 42** — 14 check per cella × 3 celle, **zero falliti** |
| `actor_unchanged` | **True** |

---

## 4. Le tre celle

| cella | seed | verdetto | step | end_reason | cicli validi | HS | TO | WAIT_HS rows |
|---|---|---|---|---|---|---|---|---|
| **G** | **126** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | **0** |
| **H** | **127** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | **0** |
| **I** | **128** | PASS | 500 | `episode_time_limit` | 2 | 3 | 3 | **0** |

**Contatori FSM e morphology — tutti zero in tutte e tre le celle:**

| contatore | G | H | I |
|---|---|---|---|
| `phase_timeout_stance` | 0 | 0 | 0 |
| `phase_timeout_swing` | 0 | 0 | 0 |
| `morphology_causal_contract_failure` | 0 | 0 | 0 |
| `hs_cancelled_count` | 0 | 0 | 0 |
| `resync_count` | 0 | 0 | 0 |
| `action_clipped_steps` | 0 | 0 | 0 |

Nessuna cella termina in anticipo, nessuna richiede risincronizzazione, nessuna cancella un
heel-strike, nessuna satura l'azione.

### Cinematica (rad)

| cella | knee min | knee max | ankle min | ankle max |
|---|---|---|---|---|
| G | −0.992047 | −0.170444 | −0.112351 | 0.400498 |
| H | −0.995600 | −0.160982 | −0.110839 | 0.405529 |
| I | −1.001632 | −0.167479 | −0.111734 | 0.406601 |

Escursioni coerenti fra le tre celle e con quelle di J19B: ginocchio in una banda di ~0.83 rad,
caviglia ~0.52 rad, dispersione fra celle di pochi millesimi di radiante.

### Return e rumore realizzato

| cella | `episode_return` | `realized_noise_rms` (knee, ankle) |
|---|---|---|
| G | 42.2734 | 0.0052220, 0.0050431 |
| H | 42.2455 | 0.0048698, 0.0049668 |
| I | 41.9715 | 0.0050873, 0.0053137 |

Il rumore realizzato sta fra **0.004870 e 0.005314**, coerente con la sigma congelata **0.005**
dell'attore. I return sono in linea con le celle stocastiche di J19B (42.13–42.19).

---

## 5. Penetrazione

Le tre bande, e cosa significano:

- **> 20 mm — SOFT, diagnostico.** Non ha mai bloccato nulla in questa catena.
- **≥ 25 mm — diagnostica July, comparativa.** Registra dove la run July promossa si sarebbe
  collocata. **Non è binding.**
- **> 28 mm — HARD BINDING, unica barra bloccante.** 28 esatti **passano**.

| cella | massimo (mm) | campioni > 20 | campioni ≥ 25 | **campioni > 28** | banda | margine a 28 |
|---|---|---|---|---|---|---|
| **G** | **24.417393735478** | 106 | 0 | **0** | `above_soft_below_july_legacy` | 3.583 mm |
| **H** | **25.684338426321** | 102 | **4** | **0** | `july_legacy_breach_within_hard` | **2.316 mm** |
| **I** | **24.762677044731** | 100 | 0 | **0** | `above_soft_below_july_legacy` | 3.237 mm |

**Nessun campione, in nessuna cella, supera la barra binding a 28 mm.**

**La cella H è diagnosticamente sopra 25 mm — quattro campioni su 500 — e questo NON è un FAIL.**
La soglia July è comparativa per costruzione, e resta 2.316 mm sotto la barra bloccante. È lo stesso
schema osservato in J19B, dove la cella F registrava 25.6151 mm con margine 2.385 mm: una cella
stocastica su tre sfiora la banda July, e nessuna si avvicina alla barra binding.

---

## 6. Integrità e immutabilità

| | SHA-256 |
|---|---|
| **actor J19A `module_state.pkl`** — invariato | `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530` |
| **receipt J19C** | `4224f0201eeccc8ccec911635b5bc18a82e38fb27eeb878457d004d28cb17ffc` |
| **commit verification J19C** | `11096edec7e259a2fe3f6dd2691c26cee0ce0e82b96864bd85a52a00a9e6ef2f` |

- **Verifica post-commit**: `pass = True`, **66 file controllati su 66 registrati**,
  `hash_mismatches = []`, `paths_missing = []`, `receipt_matches_staging_bytes = True`.
- **68 file su disco**: i due non registrati sono il receipt — che non può contenere il proprio
  hash — e `commit_verification.json`, scritto dopo il commit. È lo stesso schema di J18, J19A e
  J19B.
- Marcatore `TECHNICAL_INVALID` **rimosso** solo dopo la verifica.
- **Nessun lock, nessuna staging residua. Un solo leaf.**
- `actor_before == actor_after` su tutti e sette gli artefatti J19A; `actor_unchanged = True`.
  L'attore è stato **letto e mai toccato**.
- `inert`: `fit_executed false`, `critic_touched false`, `ppo_updates 0`, `actor_edited false`,
  `actor_copied false`, `logstd_head_edited false`, `ray_cluster_started false`.

---

## 7. Audit programmatico indipendente dell'architetto

L'architetto ha eseguito un audit programmatico proprio, indipendente da questa esecuzione, e ha
riportato **PASS su tutti i 17 gruppi di controllo booleani**:

| # | gruppo | esito |
|---|---|---|
| 1 | aggregate | PASS |
| 2 | matrix | PASS |
| 3 | 500 step / `end_reason` | PASS |
| 4 | FSM v3 / event source | PASS |
| 5 | target counters | PASS |
| 6 | cycles / events | PASS |
| 7 | WAIT_HS | PASS |
| 8 | 42/42 gates | PASS |
| 9 | telemetry 3/3 | PASS |
| 10 | kinematics | PASS |
| 11 | penetration binding | PASS |
| 12 | penetration semantics | PASS |
| 13 | actor unchanged | PASS |
| 14 | commit | PASS |
| 15 | content set | PASS |
| 16 | one leaf / no workdirs | PASS |
| 17 | GO / pins | PASS |

Il verdetto indipendente coincide con quello del runner: **PASS**.

---

## 8. Limiti

Il risultato è circoscritto a ciò che è stato effettivamente misurato.

- **Tre nuove realizzazioni del rumore alla STESSA partenza nominale**, con semi 126, 127, 128.
  Non sono partenze nuove: tutte e tre iniziano a 1.956870983805102 s, come le celle D, E ed F di
  J19B. Le partenze perturbate ±0.20 s erano già coperte da J19B, non da qui.
- **Nessun soggetto nuovo, nessuna morfologia nuova.** Il modello resta `AB06_SEASEA`.
- **Nessuna generalizzazione** oltre le **nove celle della qualificazione combinata J19B + J19C** —
  le sei A–F di J19B più le tre G/H/I di questa fase. «Nove celle» si riferisce **solo** a quelle
  due qualificazioni preregistrate, **non** a tutte le esecuzioni closed-loop storiche della catena,
  che sono molte di più e appartengono a stadi diversi con attori diversi.
- **Nulla è promosso, nulla è deployable.** Il receipt registra `promotion: NONE` e
  `next_stage_authorized: false`, e nessuno stadio ha dimostrato deployability.
- Il margine minimo alla barra binding è **2.316 mm** (cella H). Non è ampio, ed è coerente con il
  2.385 mm di J19B: lo segnalo perché resti sull'atto.

---

## 9. TODO — prossimo passo e propagati

**Prossimo passo, da preregistrare e validare prima di qualunque esecuzione:**

- **warm-up del solo critic**, preservando **byte per byte** l'attore J19A. Va preregistrato e
  validato come ogni fase precedente.
- **Questo report NON autorizza né il warm-up né PPO.** L'autorizzazione è una tua decisione
  separata.

**TODO futuri, ancora fuori perimetro:**

- **LOTO**, **LOCO**, **B1R1**, **B1R2**;
- l'**Epic** di generalizzazione multi-modello.

**TODO già aperti, propagati:**

- `policy_std` sempre `null`, difetto cosmetico ereditato da J12;
- il sidecar `actor_feature_manifest.json` della leaf J8 resta **stantio**, lasciato invariato per
  decisione architetturale: chi lo consuma deve leggere l'overlay di provenienza;
- la leaf J8 non ha `commit_verification.json`, come la leaf J2;
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1;
- `ENV_MUTATION_POLICY` in J19C resta una **superset conservativa ereditata**, byte-identica a
  J19B: autorizza a variare `episode_start_offset_s`, autorizzazione che questo stadio non ha mai
  esercitato. Documentata nella preregistrazione e riportata in due sezioni esplicite nel receipt;
- le suite readiness J18, J19A, J19B e J19C restano ciascuna con un check storicamente falso dopo
  l'emissione dei rispettivi GO: **non modificate**, per decisione architetturale.

---

## 10. STOP

Esecuzione singola completata, verdetto **PASS 3/3**, 42/42 gate, leaf verificato, attore immutato.

**Nessuna fase successiva avviata. Nessun critic, nessun PPO, nessun fit.**

**Fermo in attesa del tuo audit.**
