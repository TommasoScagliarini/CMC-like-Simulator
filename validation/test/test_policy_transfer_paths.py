"""Preserve frozen V26C inputs when the transfer workspace changes directory.

These checks read the real J20 receipt, YAML and policy bytes, without loading
RLlib or starting a rollout. Historical paths remain immutable provenance.
"""

from __future__ import annotations

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(BASELINE))

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import rollout_eval  # noqa: E402
import train_ppo_mlp  # noqa: E402
import training_config  # noqa: E402
import warm_start  # noqa: E402
from path_resolver import (  # noqa: E402
    LEGACY_POLICY_TRANSFER_DIR,
    POLICY_TRANSFER_DIR,
    POLICY_TRANSFER_PROCEDURES_DIR,
    normalize_cli_existing_path,
    remap_policy_transfer_path,
    resolve_repo_path,
)


J20_RUN = (
    "v26c_july_replica_2026-08-26/j20_runs/"
    "j20_critic_warmup_v26c_2026-08-27_r1"
)
CHECKPOINT_TAIL = J20_RUN + "/checkpoint_last"
MODULE_TAIL = CHECKPOINT_TAIL + "/learner_group/learner/rl_module/default_policy"
SNAPSHOT_TAIL = J20_RUN + "/training_cfg.resolved.yaml"
RECEIPT_TAIL = J20_RUN + "/checkpoint_last_meta.json"
EXPECTED_SHA256 = {
    RECEIPT_TAIL: "11c8dfd5a52adad850f63efa838eaa690b067a6af9fbdf54d1504a923ca13291",
    SNAPSHOT_TAIL: "976f6c5050161f3edf610de1843c186186fdc56ea9d563cf0fc4c1958c14a810",
    MODULE_TAIL + "/module_state.pkl": (
        "57720e2e3fa8a1fd412ba028e2452dead681d57ea1357be1c9f44f152b3cd168"
    ),
}


def saved_addresses(tail: str) -> tuple[str, ...]:
    addresses = []
    for prefix in (
        LEGACY_POLICY_TRANSFER_DIR,
        POLICY_TRANSFER_DIR.relative_to(ROOT).as_posix(),
        POLICY_TRANSFER_PROCEDURES_DIR.relative_to(ROOT).as_posix(),
    ):
        relative = prefix + "/" + tail
        addresses.extend((
            relative,
            "./" + relative,
            relative.replace("/", "\\"),
            str(ROOT / relative),
            "/former-machine/CMC-like-Simulator/" + relative,
            "C:\\archive\\CMC-like-Simulator - Claude\\" + relative.replace("/", "\\"),
            "//server/share/CMC-like-Simulator/" + relative,
        ))
    return tuple(addresses)


class PolicyTransferPathsTests(unittest.TestCase):
    def test_saved_checkpoint_receipt_still_identifies_the_same_policy_bytes(self):
        receipt_path = POLICY_TRANSFER_PROCEDURES_DIR / RECEIPT_TAIL
        receipt_bytes = receipt_path.read_bytes()
        saved_checkpoint = json.loads(receipt_bytes)["checkpoint"]
        checkpoint = POLICY_TRANSFER_PROCEDURES_DIR / CHECKPOINT_TAIL

        self.assertIn(LEGACY_POLICY_TRANSFER_DIR, saved_checkpoint)
        self.assertEqual(resolve_repo_path(saved_checkpoint), checkpoint)
        self.assertTrue((checkpoint / "rllib_checkpoint.json").is_file())
        for tail, expected_sha in EXPECTED_SHA256.items():
            for address in saved_addresses(tail):
                with self.subTest(address=address):
                    resolved = resolve_repo_path(address)
                    self.assertEqual(resolved, POLICY_TRANSFER_PROCEDURES_DIR / tail)
                    self.assertEqual(remap_policy_transfer_path(address), resolved)
                    self.assertEqual(normalize_cli_existing_path(address), str(resolved))
                    self.assertEqual(hashlib.sha256(resolved.read_bytes()).hexdigest(), expected_sha)
        self.assertEqual(receipt_path.read_bytes(), receipt_bytes)

    def test_checkpoint_snapshot_loading_preserves_frozen_configuration(self):
        snapshot = POLICY_TRANSFER_PROCEDURES_DIR / SNAPSHOT_TAIL
        before = snapshot.read_bytes()
        expected = training_config.load(snapshot)

        # Global best47 defaults must not replace a historical J20 snapshot.
        self.assertTrue(expected["model"]["freeze_actor"])
        self.assertEqual(expected["reward"]["morphology_weight"], 0.0025)
        for saved in saved_addresses(CHECKPOINT_TAIL):
            with self.subTest(saved=saved):
                self.assertEqual(training_config.load_resolved_for_checkpoint(saved), expected)
        for saved in saved_addresses(SNAPSHOT_TAIL):
            with self.subTest(saved=saved):
                self.assertEqual(training_config.resolve_config_path(saved), snapshot)
                self.assertEqual(training_config.load(saved), expected)
        self.assertEqual(snapshot.read_bytes(), before)

    def test_rollout_resume_and_raw_actor_source_accept_saved_checkpoint_paths(self):
        checkpoint = POLICY_TRANSFER_PROCEDURES_DIR / CHECKPOINT_TAIL
        module = POLICY_TRANSFER_PROCEDURES_DIR / MODULE_TAIL
        for saved in saved_addresses(CHECKPOINT_TAIL):
            with self.subTest(operation="rollout/resume", saved=saved):
                self.assertEqual(rollout_eval._resolve_input_path(saved), checkpoint)
                self.assertEqual(train_ppo_mlp._resolve_resume_path(saved), checkpoint)
        for saved in saved_addresses(MODULE_TAIL):
            with self.subTest(operation="raw_actor", saved=saved):
                self.assertEqual(warm_start.resolve_source_checkpoint(saved), module)
                self.assertEqual(
                    warm_start.resolve_source_checkpoint(saved + "/module_state.pkl"), module
                )
        for saved in saved_addresses(SNAPSHOT_TAIL):
            with self.subTest(operation="source_config", saved=saved):
                self.assertEqual(
                    warm_start.resolve_source_config(saved, module),
                    POLICY_TRANSFER_PROCEDURES_DIR / SNAPSHOT_TAIL,
                )

    def test_current_addresses_are_stable_for_each_retained_workspace(self):
        # F0/F1/F2R/V26B still supply helpers to the successful V26C procedure.
        for workspace in (
            "f0_freeze_2026-08-22", "f1_ablation_2026-08-23",
            "f2r_bridge_2026-08-23", "v26b_bridge_2026-08-24",
            "v26c_july_replica_2026-08-26",
        ):
            current = POLICY_TRANSFER_PROCEDURES_DIR / workspace
            self.assertTrue(current.is_dir())
            for address in (
                current, str(current.relative_to(ROOT)),
                POLICY_TRANSFER_DIR / workspace,
                LEGACY_POLICY_TRANSFER_DIR + "/" + workspace,
            ):
                with self.subTest(address=address):
                    self.assertEqual(resolve_repo_path(address), current)

    def test_shared_modules_and_other_namespaces_stay_outside_procedures(self):
        for tail in (
            "adaptation/target_domain_imitation.py",
            "utilities/freeze_actor_checkpoint.py",
            "future_procedure/unknown.yaml",
        ):
            expected = POLICY_TRANSFER_DIR / tail
            for prefix in (
                LEGACY_POLICY_TRANSFER_DIR,
                POLICY_TRANSFER_DIR.relative_to(ROOT).as_posix(),
            ):
                with self.subTest(prefix=prefix, tail=tail):
                    self.assertEqual(resolve_repo_path(prefix + "/" + tail), expected)

    def test_root_validation_and_nested_frozen_namespaces_are_not_redirected(self):
        legacy_file = LEGACY_POLICY_TRANSFER_DIR + "/" + SNAPSHOT_TAIL
        for address in (
            "validation/test/test_policy_transfer_paths.py",
            "Trajectory Generator/baseline_MLP/validation_backup/" + SNAPSHOT_TAIL,
            "snapshots/" + legacy_file,
            str(ROOT / "snapshots" / legacy_file),
            "C:/snapshots/CMC-like-Simulator/" + legacy_file,
            "C:/runs/CMC-like-Simulator/" + legacy_file,
            "C:/other-project/" + legacy_file,
        ):
            with self.subTest(address=address):
                self.assertIsNone(remap_policy_transfer_path(address))

    def test_existing_external_workspace_keeps_its_own_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            external = Path(tmp) / "CMC-like-Simulator" / LEGACY_POLICY_TRANSFER_DIR / "config.yaml"
            external.parent.mkdir(parents=True)
            external.write_text("reward:\n  morphology_weight: 123.0\n", encoding="utf-8")
            self.assertIsNone(remap_policy_transfer_path(external))
            self.assertEqual(resolve_repo_path(external), external)
            self.assertEqual(training_config.resolve_config_path(external), external)
            self.assertEqual(training_config.load(external)["reward"]["morphology_weight"], 123.0)

    def test_retired_assets_remain_missing_instead_of_using_current_defaults(self):
        retired_tail = "v12r9zero/retired_config.yaml"
        saved = LEGACY_POLICY_TRANSFER_DIR + "/" + retired_tail
        resolved = resolve_repo_path(saved)
        self.assertEqual(resolved, POLICY_TRANSFER_DIR / retired_tail)
        self.assertFalse(resolved.exists())
        with self.assertRaises(FileNotFoundError):
            training_config.load(saved)
        with self.assertRaises(FileNotFoundError):
            warm_start.resolve_source_checkpoint(
                LEGACY_POLICY_TRANSFER_DIR + "/v12r9zero/rl_module"
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
