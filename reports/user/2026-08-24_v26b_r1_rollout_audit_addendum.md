# V26B — Addendum di audit al rollout R1 (correttivo additivo; Codex HOLD)

**Data:** 2026-08-24 · **Ordine Codex:** correggere ADDITIVAMENTE due difetti di handoff, senza mutare receipt `0fc762fd…`, trace `a73c4d8f…`, report `742012fd…`, rev3e né altri frozen — tutti ri-verificati byte-identici.

## Correzione B — valori autorevoli whole-trace (l'errore era nel TESTO del report, la receipt era corretta)
Il report immutabile `2026-08-24_v26b_r1_rollout_ended_early_diagnosis.md` (§2) dichiara «dropped_wait_hs/terminal_flushed: 0». **Valori autorevoli, ricomputati INDIPENDENTEMENTE dal JSON della trace (mai copiati dalla receipt):**
- `morphology_causal_dropped_wait_hs_count`: **max = 1, last = 0, 109 righe > 0** (primo step positivo 5) — campioni morphology WAIT_HS scartati mentre nessun HS valido veniva mai stabilito (coerente con `valid_hs_count = 0`);
- `morphology_causal_terminal_flushed`: **max = 1, last = 1, 1 riga > 0 (step 242)** — flush terminale a fine episodio;
- confermati: `morphology_causal_failed_closed` 0 righe (failure False), `phase_timeout_stance` 0, `phase_timeout_swing` 0, `dropped_pending`/`cancelled`/`timeout_transition` tutti 0;
- consistenza receipt↔ricomputazione indipendente: **match su tutte le chiavi** (la receipt `corrected_per_row` già portava max=1/last=0 e max=1/last=1).
**Addendum content-addressed** accanto alla receipt: `v26b_r1_rollout_receipt_audit_addendum.json` → **SHA `8391e424ca576dc16fd71d636a32d468c1146bf5598e43a2a76a31941a345d4a`** (pinna receipt/trace/report/rev3e, dichiara i valori autorevoli, registra il misstatement, tool sha incluso).

## Correzione A — selftest post-run-safe (nuovo file; quello pinnato resta intatto)
`test_v26b_r1_rollout.py` (pinnato, 19/19 pre-run = fatto storico) non è ri-eseguibile post-run (asseriva `JOB_DIR` inesistente). Nuovo file additivo **`test_v26b_r1_rollout_postrun.py`** + modulo **`v26b_r1_postrun_audit.py`**: refusal/no-side-effect su JOB_DIR **monkeypatchata in tempdir** (token mancante/errato/no-clobber, dir lasciata intatta), verifica read-only di receipt/trace/report per digest, lineage completa + digest attore, produzione invariata (rollout_eval `5433bcbc…`, config `a870cc38…`), **exact-one-rollout evidence** (una sola job dir, una sola receipt, rc=0 ENDED_EARLY), contatori indipendenti == addendum, receipt/trace byte-identiche prima/dopo la suite. **SELFTEST PASS 23/23, ri-eseguibile in qualunque momento.**

## SHA completi
| artefatto | SHA-256 |
|---|---|
| addendum receipt | `8391e424ca576dc16fd71d636a32d468c1146bf5598e43a2a76a31941a345d4a` |
| `v26b_r1_postrun_audit.py` | `503ab902ea64e36c70c1d5835a6f690800ca273a20b73ba73b23393438845fd6` |
| `test_v26b_r1_rollout_postrun.py` | `9ea306630cdf2e89f0ef1aa1ead793a16567800852bb2e2138b5349e902d5363` |
| receipt rollout (immutata) | `0fc762fd3271b84907a0cc332f49f16461fa2c948d032eb21962c96038687611` |
| trace (immutata) | `a73c4d8f8e26e3e5a95fcc1f0ac42521ea1ba263b3b65583c277b7b75ea8a672` |
| report originale (immutato) | `742012fdc42f1df1d686e0b4b009d93dc0f9750bd63bb7d61d35837a7340f7f3` |
| rev3e (immutato) | `8ac5af69f7914d2a8de19b7da73cd19351876ff3e561660c1b42f51a35b7c04c` |

**STOP.** Nessun nuovo DAgger/rollout/refit. In attesa dell'audit Codex.
