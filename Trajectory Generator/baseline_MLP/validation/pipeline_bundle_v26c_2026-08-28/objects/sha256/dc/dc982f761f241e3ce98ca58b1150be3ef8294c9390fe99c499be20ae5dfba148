# V26B — Supplemento B3 al report del rollout R2I (additivo; supersede SOLO l'interpretazione B3)

**Data:** 2026-08-24 · Task ristretto dell'architetto. Il report `2026-08-24_v26b_r2i_rollout_fail_diagnosis.md` (`aed118fb…`, immutato) riporta «diagnostico B3 (≤−0.03) SODDISFATTO»: **interpretazione superseded**. Quella riga usava il minimo GLOBALE della caviglia; B3 è definito nella finestra late-stance di fase [0.55, 0.80].

**Calcolo corretto** (da `reward_terms.pros_ankle_angle_imitation_target_phase`, verificato indipendentemente e conforme all'audit Codex): campo presente su **197/197 righe, valore 0.0 ovunque, 0 righe in [0.55, 0.80]** → **B3 = `not_evaluable_no_valid_phase_or_cycle` — NON PASS** (coerente con 0 HS validi / 0 cicli). Il minimo globale caviglia **−0.1817615** resta diagnostica DISTINTA e valida. Nota di scope ribadita: questo singolo nominale è un pre-gate diagnostico omologo a R0a/R1; **il gate V3 completo richiede 3 start e resta NON VALUTATO**; nessun secondo start in questo stadio.

**Addendum content-addressed** (accanto alla receipt primaria immutata `6d31bc82…`): `v26b_r2i_rollout_receipt_audit_addendum.json` → **SHA `51f35f3170663be4f66e7434111cc8739dba9407f2613d4e8440a205c63c04b2`** — pinna receipt/trace (`035146ab…`)/simulator log (`2c687285…`)/rev3j, registra gli hash del tooling post-code (`v26b_r2i_rollout.py` `8a874e6a…`, test `32b5f9f4…` — fatti, non parent pins), il campo superseded con la motivazione e il verdetto corretto.

**Test post-run-safe**: `test_v26b_r2i_rollout_postrun.py` — **15/15** (immutabilità/hash degli originali con negativo tamper, B3 reale = not-evaluable, B3 sintetico con finestre valide meet/not-meet, campo mancante fail-closed, addendum no-clobber, originali byte-identici post-suite).
