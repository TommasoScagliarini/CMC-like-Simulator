"""Audit completed RIGHT smoke training, genuine PPO updates and full resume."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import pickle
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))


def read(path):
    return json.loads(path.read_text())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--iterations", type=int, required=True)
    p.add_argument("--batch", type=int, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--resume", action="store_true")
    a = p.parse_args()
    import numpy as np
    from unified_MLP_39.actor_transition import actor_state_digest, read_contract, reference_from_contract
    rows = [json.loads(line) for line in (a.run / "train_iterations.jsonl").read_text().splitlines() if line.strip()]
    summary = read(a.run / "summary.json")
    assert summary["ok"] and summary["stop_reason"] == "completed" and not summary.get("skipped_iterations")
    assert [r["iteration"] for r in rows] == list(range(1, a.iterations + 1))
    assert reference_from_contract(read_contract(a.run)["target_channels"]) == "right_v26"
    assert summary["initialization_mode"] == ("resume_from" if a.resume else "fresh")
    for row in rows:
        audit = row["unified_phase_audit"]
        assert audit["phase"] == "imitation" and audit["mean_changed"] and audit["critic_changed"]
        batch = audit["target_availability_batch"]
        assert batch["total_steps"] == batch["visible_steps"] == a.batch
        assert batch["matches_expected_train_batch_size"]
        assert all(math.isfinite(row[k]) for k in ("policy_loss", "vf_loss", "mean_kl_loss", "entropy"))
        milestone = a.run / f"milestone_iteration_{row['iteration']:06d}"
        assert (milestone / "checkpoint_last/learner_group/learner/state.pkl").is_file()
        assert (milestone / "rl_module_last/module_state.pkl").is_file()
        assert read_contract(milestone / "rl_module_last") == read_contract(a.run)
    initial = pickle.loads((a.run / "rl_module_initial/module_state.pkl").read_bytes())
    final = pickle.loads((a.run / "rl_module_last/module_state.pkl").read_bytes())
    assert actor_state_digest(initial) == rows[0]["unified_phase_audit"]["actor_before"]
    assert actor_state_digest(final) == rows[-1]["unified_phase_audit"]["actor_after"]
    assert not np.array_equal(np.asarray(initial["pi.1.weight"])[2:], np.asarray(final["pi.1.weight"])[2:])
    state = read(a.run / "rl_module_initial/unified_training_state.json")
    assert state["actor_source"] is None and state["last_completed_iteration"] == 0
    # The same seed and current 39D layout must retain the LEFT run's initial
    # function; no historical actor or hidden warm start enters this experiment.
    left = PACKAGE / "runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21"
    left_rows = [json.loads(line) for line in (left / "train_iterations.jsonl").read_text().splitlines() if line.strip()]
    for field in ("actor_before", "critic_before"):
        assert rows[0]["unified_phase_audit"][field] == left_rows[0]["unified_phase_audit"][field]
    optimizer = pickle.loads((a.run / "checkpoint_last/learner_group/learner/state.pkl").read_bytes())["optimizer"]
    assert optimizer and all(o["state"]["state"] for o in optimizer.values())
    optimizer_steps = sorted({float(s["step"]) for o in optimizer.values() for s in o["state"]["state"].values()})
    assert min(optimizer_steps) > 0
    resume = None
    if a.resume:
        resume = read(a.run / "resume_optimizer_audit.json")
        assert resume["passed"] and all(r["passed"] for r in resume["learners"])
        restoration = read(a.run / "resume_audit.json")
        assert restoration["actor_reinitialized"] is False
        assert rows[-1]["unified_phase_audit"]["actor_before"] == rows[-2]["unified_phase_audit"]["actor_after"]
        assert rows[-1]["unified_phase_audit"]["critic_before"] == rows[-2]["unified_phase_audit"]["critic_after"]
    receipt = {"pass": True, "run": str(a.run.resolve()), "iterations": len(rows),
        "sampled_steps": sum(r["unified_phase_audit"]["target_availability_batch"]["total_steps"] for r in rows),
        "fresh_initialization_matches_left_run": True, "all_prescribed_visible": True,
        "actor_critic_and_logstd_updated": True, "checkpoint_contract_reference": "right_v26",
        "optimizer_steps": optimizer_steps, "resume_audit": resume,
        "last_module_sha256": hashlib.sha256((a.run / "rl_module_last/module_state.pkl").read_bytes()).hexdigest()}
    a.output.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
