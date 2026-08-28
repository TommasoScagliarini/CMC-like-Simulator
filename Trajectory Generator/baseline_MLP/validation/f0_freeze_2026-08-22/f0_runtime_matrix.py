"""F0 activity 4: explicit matrix of the non-isometric runtime differences.

Key-by-key comparison of the resolved configurations of every lineage stage
(June imitation, July H0/pilot, V26 imitation, B0820 warmup/v2/v3, editable
canonical, chain snapshot) plus SHA-256 verification of the referenced
profiles. Writes ``manifest/runtime_matrix.json`` and ``.md``.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402

CONFIGS: list[tuple[str, Path]] = [
    ("JUN23_imit", C.JUN23_RUN / "training_cfg.resolved.yaml"),
    ("JUL_warmup_H0", C.JUL_WARMUP / "training_cfg.resolved.yaml"),
    ("JUL_pilot50", C.JUL_PILOT_RUN / "training_cfg.resolved.yaml"),
    ("V26_imit_june_equiv", C.V26_IMIT_RUN / "training_cfg.resolved.yaml"),
    ("B0820_warmup", C.B0820_WARMUP / "training_cfg.resolved.yaml"),
    ("B0820_v2_from_zero", C.B0820_V2_RUN / "training_cfg.resolved.yaml"),
    ("B0820_v3_fixedcorridor", C.B0820_V3_RUN / "training_cfg.resolved.yaml"),
    ("chain_snapshot_B0820", C.CHAIN_SNAPSHOT_CFG),
    ("canonical_editable_now", C.CANONICAL_CFG),
]

# Items declared by the plan (section 4, Fase 0, punto 4) with the evidence key.
DECLARED_ITEMS = [
    ("Profilo GRF applicato", "grf.online_grf_profile"),
    ("Profilo detector GRF legacy HS/TO", "grf.online_grf_detector_profile"),
    ("Blocco binary_phase: modalita FSM", "grf.binary_phase_fsm_mode"),
    ("Blocco binary_phase: profilo detector V25", "grf.binary_phase_detector_profile"),
    ("Blocco binary_phase: contratto eventi", "grf.binary_phase_event_contract_id"),
    ("Blocco binary_phase: policy eventi invalidi", "grf.binary_phase_invalid_event_policy"),
    ("Blocco binary_phase: versione FSM attore", "grf.binary_phase_actor_fsm_version"),
    ("Penetrazione: soglia penalita [m]", "simulation.grf_penetration_penalty_threshold_m"),
    ("Penetrazione: terminazione [m]", "simulation.grf_penetration_termination_m"),
    ("Timeout swing hard [s]", "reward.phase_swing_hard_timeout_s"),
    ("Timeout stance hard [s]", "reward.phase_stance_hard_timeout_s"),
    ("Morphology: profilo", "reward.morphology_profile"),
    ("Morphology: modalita fase", "reward.morphology_phase_mode"),
    ("Morphology: peso", "reward.morphology_weight"),
    ("Morphology: hard termination", "reward.morphology_hard_termination_enabled"),
    ("Morphology: allow_effects sperimentale", "reward.morphology_experimental_allow_effects"),
    ("Morphology: allow_effects causale", "reward.morphology_causal_allow_effects"),
    ("Morphology: contratto eventi causale", "reward.morphology_causal_event_contract_id"),
    ("Reward mode", "reward.reward_mode"),
    ("Exact-start sampling", "parallelism.exact_start_sampling"),
    ("Start offset nominale [s]", "simulation.episode_start_offset_s"),
    ("Start offset choices", "simulation.episode_start_offset_choices_s"),
    ("Gait clock prescritto esposto", "simulation.gait_clock_enable"),
    ("Osservazione diagnostica controller", "simulation.include_controller_diagnostic_observation"),
    ("PPO lr", "ppo.lr"),
    ("PPO batch", "ppo.train_batch_size"),
    ("PPO epoche", "ppo.num_epochs"),
    ("PPO clip", "ppo.clip_param"),
    ("KL guard max minibatch", "supervision.max_minibatch_mean_kl_loss"),
    ("freeze_logstd", "model.freeze_logstd"),
    ("freeze_actor", "model.freeze_actor"),
    ("num_env_runners", "parallelism.num_env_runners"),
]


def main() -> int:
    C.ensure_out_dirs()
    out_json = C.OUT_MANIFEST / "runtime_matrix.json"
    out_md = C.OUT_MANIFEST / "runtime_matrix.md"
    if out_json.exists() or out_md.exists():
        raise FileExistsError("runtime matrix already exists; no-clobber")

    flat: dict[str, dict[str, Any]] = {}
    shas: dict[str, str] = {}
    for name, path in CONFIGS:
        flat[name] = C.flatten(C.load_yaml(path))
        shas[name] = C.sha256_file(path)
    names = [n for n, _ in CONFIGS]
    all_keys = sorted({k for f in flat.values() for k in f})
    rows = []
    for key in all_keys:
        values = [flat[n].get(key, "<absent>") for n in names]
        distinct = {repr(v) for v in values}
        rows.append({"key": key, "values": dict(zip(names, values)), "identical_everywhere": len(distinct) == 1, "absent_in": [n for n in names if key not in flat[n]]})

    profile_shas = {
        "grf_profile_tangent_v2": C.sha256_file(C.GRF_PROFILE_TANGENT_V2),
        "grf_profile_grf_correct_magnitude": C.sha256_file(C.GRF_PROFILE_CORRECT_MAGNITUDE),
        "grf_detector_profile_HS-TO": C.sha256_file(C.GRF_DETECTOR_PROFILE),
        "binary_phase_detector_v25_selected_candidate": C.sha256_file(C.BINARY_DETECTOR_PROFILE),
        "morphology_event_warped": C.sha256_file(C.MORPH_PROFILE_EVENT_WARPED),
        "morphology_legacy_mean_std": C.sha256_file(C.MORPH_PROFILE_LEGACY),
    }

    # Pairwise classification between the two decision-relevant runtimes.
    july = flat["JUL_pilot50"]
    v3 = flat["B0820_v3_fixedcorridor"]
    v2 = flat["B0820_v2_from_zero"]
    pairwise = []
    for key in sorted(set(july) | set(v3)):
        a, b = july.get(key, "<absent>"), v3.get(key, "<absent>")
        if a == b:
            cls = "isometric"
        elif key not in july:
            cls = "assente_in_luglio (nuovo in V26/B0820)"
        elif key not in v3:
            cls = "assente_in_B0820"
        else:
            cls = "non_isometrico"
        if key in ("simulation.iterations", "logging.progress"):
            cls += " (non-runtime: contabilita/logging)"
        pairwise.append({"key": key, "july_pilot50": a, "b0820_v3": b, "class": cls})
    v2_vs_v3 = [{"key": k, "b0820_v2": v2.get(k, "<absent>"), "b0820_v3": v3.get(k, "<absent>")} for k in sorted(set(v2) | set(v3)) if v2.get(k, "<absent>") != v3.get(k, "<absent>")]

    declared = []
    for label, key in DECLARED_ITEMS:
        declared.append({"item": label, "key": key, **{n: flat[n].get(key, "<absent>") for n in names}})

    payload = {
        "schema_version": 2,
        "revision": C.F0_REV,
        "superseded_revisions": C.SUPERSEDED_REVISIONS,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
        "configs": {n: {"path": C.rel(p), "sha256": shas[n]} for n, p in CONFIGS},
        "profile_sha256": profile_shas,
        "grf_profile_sha_identical_tangent_v2_vs_correct_magnitude": profile_shas["grf_profile_tangent_v2"] == profile_shas["grf_profile_grf_correct_magnitude"],
        "declared_items": declared,
        "july_vs_b0820_v3_pairwise": pairwise,
        "july_vs_b0820_v3_non_isometric_count": sum(1 for p in pairwise if p["class"].startswith("non_isometrico")),
        "july_vs_b0820_v3_absent_in_july_count": sum(1 for p in pairwise if p["class"].startswith("assente_in_luglio")),
        "b0820_v2_vs_v3_differences": v2_vs_v3,
        "all_keys": rows,
    }
    C.write_json(out_json, payload)

    lines = [
        f"# F0 — Matrice delle differenze di runtime — revisione {C.F0_REV}",
        "",
        f"Generato: {payload['generated_at_utc']} — git HEAD `{payload['git']['head'][:12]}`.",
        "",
        "## Config confrontate (SHA-256)",
        "",
        C.md_table(["config", "path", "sha256"], [[n, C.rel(p), shas[n][:16]] for n, p in CONFIGS]),
        "",
        "## Profili referenziati (SHA-256)",
        "",
        C.md_table(["profilo", "sha256"], [[k, v] for k, v in profile_shas.items()]),
        "",
        f"Profilo GRF applicato: `tangent_v2` (B0820/giugno) e `grf_correct_magnitude` (luglio) hanno SHA-256 **{'identico' if payload['grf_profile_sha_identical_tangent_v2_vs_correct_magnitude'] else 'DIVERSO'}** -> {'stessa fisica di contatto, solo nome diverso (rename).' if payload['grf_profile_sha_identical_tangent_v2_vs_correct_magnitude'] else 'fisica di contatto diversa: differenza NON isometrica.'}",
        "",
        "## Voci dichiarate dal piano (valori per config)",
        "",
        C.md_table(["voce", "chiave"] + names, [[d["item"], d["key"]] + [d[n] for n in names] for d in declared]),
        "",
        "## Luglio (pilot50) vs B0820 v3: classificazione chiave per chiave",
        "",
        f"Non isometriche: {payload['july_vs_b0820_v3_non_isometric_count']}; assenti in luglio (nuove in V26/B0820): {payload['july_vs_b0820_v3_absent_in_july_count']}.",
        "",
        C.md_table(["chiave", "luglio pilot50", "B0820 v3", "classe"], [[p["key"], p["july_pilot50"], p["b0820_v3"], p["class"]] for p in pairwise if p["class"] != "isometric"]),
        "",
        "## B0820 v2 (from_zero) vs B0820 v3 (fixedcorridor): differenze",
        "",
        C.md_table(["chiave", "v2", "v3"], [[d["key"], d["b0820_v2"], d["b0820_v3"]] for d in v2_vs_v3]) if v2_vs_v3 else "Nessuna differenza nei resolved yaml.",
        "",
        "Nota: la run v2 e la run v3 condividono lo stesso resolved yaml salvo `binary_phase_actor_fsm_version`; la correzione del corridoio causale (tolleranza 1e-12 -> 1e-9, ri-armo sui repair) e' una differenza di **codice**, non di configurazione, e separa le due run insieme alla FSM.",
        "",
    ]
    C.write_text(out_md, "\n".join(lines))
    print(f"[runtime-matrix] written {out_json}; non-isometric={payload['july_vs_b0820_v3_non_isometric_count']} absent-in-july={payload['july_vs_b0820_v3_absent_in_july_count']} grf_sha_identical={payload['grf_profile_sha_identical_tangent_v2_vs_correct_magnitude']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
