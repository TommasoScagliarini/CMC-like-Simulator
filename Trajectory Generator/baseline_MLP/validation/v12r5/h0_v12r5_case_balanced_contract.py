"""Execution-free contract for the additive V12R5 case-balanced lineage.

V12R5 never retries or resumes terminal V12R4.  It admits exactly one new
student-exposed data source: the completed, gated nominal V12R4 label corpus.
The failed V12R4 ``+0.20`` prefix is bound as forensic evidence and is never a
fit input.  The only fit starts from the frozen H0 source, combines the exact
8,732-row P2 corpus with the exact 500-row nominal corpus, and then executes
six pure-policy development rollouts in a preregistered critical-first order.

Importing this module resolves its own source location and imports the frozen
V12R3 constant contract.  It does not read or write run artifacts, load a
model, fit, draw randomness, reset an environment, or step an environment.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


_SOURCE_PATH = Path(__file__).resolve()
_LOCAL_VALIDATION_ROOT = _SOURCE_PATH.parent.parent
_REPO_ROOT = _SOURCE_PATH.parents[4]
for _IMPORT_ROOT in (
    _SOURCE_PATH.parent,
    _LOCAL_VALIDATION_ROOT / "v12r3",
    _LOCAL_VALIDATION_ROOT,
    _REPO_ROOT / "validation",
    _REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    _REPO_ROOT,
):
    if str(_IMPORT_ROOT) not in sys.path:
        sys.path.insert(0, str(_IMPORT_ROOT))

import h0_primary_split_v12r3_autonomy_recovery_contract as v12r3  # noqa: E402


SCHEMA_VERSION = 1250
REVISION = "2026-08-09"
AUTHORITY_TEXT = "esegui i punti 1-6"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V12R5_CASE_BALANCED_V26"
PIPELINE_ID = "H0_V12R5_CASE_BALANCED_SINGLE_FIT_CRITICAL_FIRST"
FIT_CONTRACT_ID = "h0_primary_split_v12r5_case_balanced_full_mean_v1"
CANDIDATE_SELECTION_RULE = "SOLE_CASE_BALANCED_FIT_OUTPUT_FROM_LOCKED_R5_RUN"

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": AUTHORITY_TEXT,
    "authority_scope": "V12R5_EXACT_ONE_SHOT_CASE_BALANCED_DEVELOPMENT",
    "source_implementation_authorized": True,
    "protocol_freeze_publication_authorized": True,
    "execution_lock_authorized": True,
    "actor_fit_execution_authorized": True,
    "candidate_freeze_authorized": True,
    "development_execution_authorized": True,
    "new_environment_collection_authorized": False,
    "qualification_execution_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "detector_or_fsm_change_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r5")
PROTOCOL_FREEZE_PATH = VALIDATION_ROOT / "h0_v12r5_case_balanced_protocol_freeze.json"
DESIGN_AUDIT_PATH = VALIDATION_ROOT / "h0_v12r5_case_balanced_design_audit.json"
PROTOCOL_PUBLICATION_FAILURE_PATH = (
    VALIDATION_ROOT / "h0_v12r5_case_balanced_protocol_publication_failure.json"
)
EXECUTION_LOCK_PATH = VALIDATION_ROOT / "h0_v12r5_case_balanced_execution_lock.json"
EXECUTION_LOCK_PUBLICATION_FAILURE_PATH = (
    VALIDATION_ROOT / "h0_v12r5_case_balanced_execution_lock_publication_failure.json"
)
RUN_ROOT = VALIDATION_ROOT / "h0_v12r5_run_20260809"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_CLAIM_FAILURE_PATH = RUN_ROOT / "pipeline_claim_failure.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH = (
    RUN_ROOT / "terminal_publication_failure.json"
)
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"
FIT_ROOT = RUN_ROOT / "fit"
CORPUS_PATH = FIT_ROOT / "corpus.npz"
CANDIDATE_MODULE_PATH = FIT_ROOT / "rl_module_target_adapted"
CANDIDATE_FREEZE_PATH = RUN_ROOT / "candidate_freeze_receipt.json"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_DEVELOPMENT_RECEIPT_PATH = RUN_ROOT / "final_development_receipt.json"


def _artifact(path: str, sha256: str, size_bytes: int) -> dict[str, Any]:
    return {"path": path, "sha256": sha256, "size_bytes": size_bytes}


P2_ROOT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r3/h0_v12r3_run_20260809/fit/p2"
)
P2_CORPUS_ARTIFACT = _artifact(
    (P2_ROOT / "corpus.npz").as_posix(),
    "42a40869447aec0cdce62a6ba5fcb48da4e5f070fac9e8eb19c4f0a8fccc1990",
    18_358_010,
)
P2_ADAPTATION_REPORT_ARTIFACT = _artifact(
    (P2_ROOT / "adaptation_report.json").as_posix(),
    "776fb10eb10430953105f5e0b818a323449b43f43e70c6649dd7cb4294825324",
    10_097,
)
P2_ADAPTATION_HISTORY_ARTIFACT = _artifact(
    (P2_ROOT / "adaptation_history.json").as_posix(),
    "02341e3b20df60fa279f0a9e5b55b14a2e0c48dccc666e095b27e841c319b1e7",
    1_659,
)
P2_MODULE_TREE = {
    "path": (P2_ROOT / "rl_module_target_adapted").as_posix(),
    "tree_sha256": "be0c9711f1f6b7b9dba9cd4ab546a8379f96444ddf02a111d5f3002b9c7e8c4f",
    "file_count": 3,
    "files": [
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
            "size_bytes": 2_262,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "f6753be7ec55e5276c6be824b3a7550880f3ac4fa26a59f2a056a20f0a5ab3f6",
            "size_bytes": 604_772,
        },
    ],
}

R4_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r4")
R4_RUN_ROOT = R4_ROOT / "h0_v12r4_run_20260809"
R4_TERMINAL_ARTIFACTS = {
    "protocol_freeze": _artifact(
        (R4_ROOT / "h0_v12r4_p3_coverage_protocol_freeze.json").as_posix(),
        "5a56146c2b674c6dfd734c875db1314ca9473fd3beaa23034b0765bb64cf3646",
        18_592,
    ),
    "execution_lock": _artifact(
        (R4_ROOT / "h0_v12r4_p3_coverage_execution_lock.json").as_posix(),
        "95dd5db4ec35c1bc5ab41feabc33b74b025a30209514a9f95229218446bd8c1c",
        5_762,
    ),
    "terminal_ledger": _artifact(
        (R4_RUN_ROOT / "pipeline_ledger.json").as_posix(),
        "4351fd5be9f35f0ab5f5166329344af7e6a8cbf9f0b87691e254d5b7704a805f",
        5_898,
    ),
}

_R4_NOMINAL_ROOT = R4_RUN_ROOT / "collect/deterministic_offset_nominal"
R4_NOMINAL_REUSABLE_ARTIFACTS = {
    "labels": _artifact(
        (_R4_NOMINAL_ROOT / "labels.npz").as_posix(),
        "b7168422c2164050733e0f30f65b031e30e995553f2e64a43f1e73751c2ce7dd",
        793_510,
    ),
    "receipt": _artifact(
        (_R4_NOMINAL_ROOT / "receipt.json").as_posix(),
        "fc587debdd3e958daadc7c2915f2c81c7d2a8462370cf8b2cdb4bae106bf0ed3",
        1_860,
    ),
    "gate": _artifact(
        (_R4_NOMINAL_ROOT / "gate.json").as_posix(),
        "cb929362734306dd9a98ddef4d960c32a44e3cb589ff709e381e13cb6fb7721e",
        689,
    ),
    "summary": _artifact(
        (_R4_NOMINAL_ROOT / "summary.json").as_posix(),
        "c4d1cf17dfbbceeb941104ec7e14cb2831223f8b49d0e5b8ff850393fb8a907a",
        10_665,
    ),
    "trace": _artifact(
        (_R4_NOMINAL_ROOT / "trace.json").as_posix(),
        "28af9ac6029f99dcf6dbf7cddc5faa48a857e40b7e5458ad41ef163804b0aae5",
        6_210_162,
    ),
}

_R4_PLUS_ROOT = R4_RUN_ROOT / "collect/deterministic_offset_plus_0p20"
R4_PLUS_FAILURE_EVIDENCE = {
    "labels_excluded": _artifact(
        (_R4_PLUS_ROOT / "labels.npz").as_posix(),
        "9ac32c57ceed2863ffc8b7b98d09b6590c04324f65e6fd3b104923a1c1f05981",
        342_790,
    ),
    "failure": _artifact(
        (_R4_PLUS_ROOT / "failure.json").as_posix(),
        "95dafae160c7e7033bed1962f77000b785181947a9ac02d32700c251917f5aab",
        61_090,
    ),
    "gate": _artifact(
        (_R4_PLUS_ROOT / "gate.json").as_posix(),
        "3c8534f844fa1131d7320a317ff273e307b85e3755185383a35ca638dfa351b9",
        689,
    ),
    "summary": _artifact(
        (_R4_PLUS_ROOT / "summary.json").as_posix(),
        "f704bce3bfabb2712730263f165ff5adc34c031ddeb5cf698f97ae7245041584",
        8_205,
    ),
}

_SAFE_PLUS_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1p1_v26_residual/teacher_replay/"
    "deterministic_offset_plus_0p20"
)
SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS = {
    "trace": _artifact(
        (_SAFE_PLUS_ROOT / "trace.json").as_posix(),
        "c3ca347011ce79f5c8d1d3235d1a1b2b595eb7fcfafdc1a74f60dc34e640887d",
        13_042_971,
    ),
    "summary": _artifact(
        (_SAFE_PLUS_ROOT / "summary.json").as_posix(),
        "b8853f5e3b2ba6027f3edb310dee427ead087e29bdd0c720ddf3ea56dde53f5f",
        10_206,
    ),
    "gate": _artifact(
        (_SAFE_PLUS_ROOT / "gate.json").as_posix(),
        "90dd5bc18dbe7ff70b49ac69b36d1d92d7d7b41d9ce1403cf6e0bf1666f9fb52",
        3_855,
    ),
    "receipt": _artifact(
        (_SAFE_PLUS_ROOT / "receipt.json").as_posix(),
        "7f89bdc66b91219a7206aca08075e8eff512b5cc25d1791c1e13ac78a4cc8339",
        158_695,
    ),
}

SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT = _artifact(
    "validation/h0_primary_grf_split_v8r1p1_teacher_replay_execution_lock.json",
    "2046f33c06d0aa052af8b7ef2089e33a0befb4bb53c75564853d6a29607df6c7",
    40_779,
)

_FROZEN_EXTERNAL_RUNTIME_SOURCE_ROWS = (
    (
        "Trajectory Generator/baseline_MLP/_bootstrap.py",
        "6943e6d187f1c325f98417106c0928d3e4e0f670102686c925b7c2ec667bd63c",
        1_049,
    ),
    (
        "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
        "5084786c8e6312de2d37744bf327b907ed52ff92cc6e3686b36bd1bde6d21a0f",
        8_081,
    ),
    (
        "Trajectory Generator/baseline_MLP/env_factory.py",
        "a728e07b4343e71ccf515f0ef7947f13735e13c3e39a16a3b5d52ea85d90d559",
        9_701,
    ),
    (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py",
        "258700b5da99aa9110a92039834bf4061ca81b9eb7ea770b93919a0b3105e801",
        55_963,
    ),
    (
        "Trajectory Generator/baseline_MLP/exploration_noise.py",
        "ea716c198cfce5e649a6d7bf5b046365c2dd9c3d3eb8f2c0a84f913e119eeb9f",
        1_467,
    ),
    (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py",
        "56d58e06acba50edebe1239906e1808336f2d343f553de573a69189f4f01b60d",
        18_348,
    ),
    (
        "Trajectory Generator/baseline_MLP/process_watchdog.py",
        "0511fdda6fd0a3d5db0697b56d20226f955c10955fdfb82d6b75d2b6834128e0",
        16_669,
    ),
    (
        "Trajectory Generator/baseline_MLP/progress_display.py",
        "26e0c59698dc98ebc3f1112ea07cdf054c0b283d9797206eb5664ef92dc777c5",
        9_846,
    ),
    (
        "Trajectory Generator/baseline_MLP/reward_function.py",
        "f530a24a4122e51fbf5f40d984821dbe15a380f81375281fbd0f28934c471b5d",
        118_458,
    ),
    (
        "Trajectory Generator/baseline_MLP/rollout_eval.py",
        "8681bd981a2fb13ce50373188b43140bccc548234bf9d680dcbbf8d0dd76e98f",
        59_374,
    ),
    (
        "Trajectory Generator/baseline_MLP/target_domain_imitation.py",
        "442be2222c935e18607e944d946d23bc8151b32c5e57d64036b8e5a00c1e9ed3",
        60_541,
    ),
    (
        "Trajectory Generator/baseline_MLP/training_config.py",
        "bf37b27de2e178c7c439e3855dea4dae529ddc139f6206def5167267b3541155",
        13_222,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/freeze_h0_primary_split_v12_autonomy_recovery.py",
        "ac87eaefe2030e77da73e05ecc1a1f476aaf513f306783724238800321a9f7a4",
        60_997,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/h0_primary_split_v12_autonomy_recovery_contract.py",
        "d146562ed1355ac178e3a0abd922568a92d262e6f2ccecfe5ef1e05135e2a4ea",
        97_514,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v12r3/freeze_h0_primary_split_v12r3_autonomy_recovery.py",
        "dd43031663faeb867f2f6ec2b80ff3885e1dc35d7be05e291b2e266a9b9dedd7",
        30_900,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v12r3/h0_primary_split_v12r3_autonomy_recovery_contract.py",
        "bf4ea138c5391e75aa0648f5bee8cea560298b8dac3a5c01958be6ec6eea18d3",
        46_121,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v12r3/h0_primary_split_v12r3_pure_probe_observer_labeler.py",
        "fc21b3f6e43646a4492372673405e7de43fa0239fb37ae9c36cf5a878fdc92ee",
        82_833,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v12r3/h0_primary_split_v12r3_recovery_weighted_fitter.py",
        "6c7c35eed501ddbe0a4fefecd1f76ea321d15baf6d374d87ce1ffbe3f38deeff",
        71_952,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v12r3/run_h0_primary_split_v12r3_autonomy_recovery.py",
        "350699f731b9afe6d02ff19419b1a6ee0e2371c35e8522354f0eac7280ff2f2c",
        132_547,
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v12r3/run_h0_primary_split_v12r3_design_audit.py",
        "cd648ac5df1f90266f0c36b3433b8701a22717f22eb30c36c29a9783b2cceb10",
        16_953,
    ),
    (
        "Trajectory Generator/baseline_MLP/warm_start.py",
        "84706218dcc4c5cb7f97a8f3f67ef40ba9e064ba5aef25cb6559d1c8a506c34c",
        22_594,
    ),
    (
        "Trajectory Generator/baseline_MLP/win_runtime.py",
        "9088d0e18002b64d70ea38ce9bad2b26c16bfc1e4b77e1caa8afd903261d033f",
        2_751,
    ),
    (
        "Trajectory Generator/binary_phase_adapter.py",
        "8eba604b4774952f41660e714e5a9913fa2b9c431d19f04711103213d780c109",
        28_028,
    ),
    (
        "Trajectory Generator/binary_phase_adapter_v26.py",
        "51c5b9c9b8fc7351b7a90909416cfbac0ca06a0485280ed8d663ef3b4bc91582",
        11_522,
    ),
    (
        "Trajectory Generator/binary_phase_fsm.py",
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1",
        18_908,
    ),
    (
        "Trajectory Generator/binary_phase_fsm_v26.py",
        "6d1c6a231f1ffab93d27ecb5967e8b08c6ead26d5a4205990d99f487e33cf02b",
        5_422,
    ),
    (
        "Trajectory Generator/osim_trj_cmc_like.py",
        "7448d8656fed1f5eaba29dfaac45202f9cc402acaeb148907c99d7cdfdfc0f27",
        194_221,
    ),
    (
        "Trajectory Generator/prosthetic_phase_fsm.py",
        "c647730d028606f967f35b6a2d30ea2b88d4bfae60718851c29eac82abf991e9",
        74_309,
    ),
    (
        "binary_phase_detector.py",
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6",
        14_226,
    ),
    (
        "config.py",
        "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3",
        17_291,
    ),
    (
        "inverse_dynamics.py",
        "81c898f551a81d9af743575e6f6639a3a3bb69ddbe9faffa876f8d3f71df0833",
        9_836,
    ),
    (
        "kinematics_interpolator.py",
        "424d352a461b424ed8f7e318513a85b75d3a6fb1a00155eab1e885e9d3fd4ede",
        9_742,
    ),
    (
        "model_loader.py",
        "401beddc52e2dd8ce4a88208cf5b38b036232cb1bc3ea37704e467852f2ace12",
        58_315,
    ),
    (
        "online_grf.py",
        "52e39bf9a3b20dd65242f3f9076d76ed788239fe7c3e5b825bc37a9657c4fefa",
        31_439,
    ),
    (
        "outer_loop.py",
        "a8a05d10d959c5fa98c29ecf703189af73318fd1291a2257315cd81417f5ef0b",
        4_558,
    ),
    (
        "output.py",
        "7b03d1031202976fd011288e031f8796364eb35c34d1ed8838f8eff971318acb",
        41_834,
    ),
    (
        "path_resolver.py",
        "2a61b8c54ab68c228ea55a6e28b6334ac0a9539da79f86e9a5ef2970bc937a1d",
        6_067,
    ),
    (
        "prosthesis_controller.py",
        "c0ef2da15b5754c9499f8c40072e655e954cc5085a2a8a858aeb6355b4073338",
        21_345,
    ),
    (
        "setup_io.py",
        "9d24df7b272ad6550c2897151b019b53d665a61a715217eeeb6dfbe54a6ae6e3",
        11_031,
    ),
    (
        "simulation_runner.py",
        "b79cb86c6ecb8d63d9555ccbae85acffaeec223904663a04998db5db6edcca19",
        110_192,
    ),
    (
        "static_optimization.py",
        "8c4a060f5d1846eafe609fafee0457870006decb4b310eed34232f3140df3a18",
        35_652,
    ),
    (
        "validation/build_h0_primary_grf_split_v6_teacher_replay_preflight.py",
        "afa7d6959cb6a9337e0c2e0bd463d841ee99ffe49fc1095c8f47241aaab22099",
        22_310,
    ),
    (
        "validation/compare_h0_v25_abc.py",
        "dade1cdd0fd90c31ecf10ce8f84c2027a899e5ccfa53e2e7c486fab41741c028",
        20_701,
    ),
    (
        "validation/h0_forensic_rollout.py",
        "e305729eb896335ebb9f492ce449cc5e7656f4fce5968e3fa4359daf38a02ebd",
        20_916,
    ),
    (
        "validation/h0_primary_grf_split_v6_teacher_replay_contract.py",
        "15432db342be3a776b456c848ae013e25b716d106d6c292f68ca913456b3f19c",
        21_410,
    ),
    (
        "validation/h0_primary_grf_split_v8_teacher_replay_contract.py",
        "b6656d698eb3bb3c238b2a344a0f6b67dc6c499f815791dc321f67e364b7075c",
        16_898,
    ),
    (
        "validation/h0_primary_grf_split_v8r1_teacher_replay_contract.py",
        "1ceabe42ca9ba671a1a58100b17251385d2fd10e3be75ebafeb009ba4544e025",
        8_159,
    ),
    (
        "validation/h0_primary_grf_split_v8r1p1_teacher_replay_contract.py",
        "65f75895c79db94aa47c61ca7be0406af2d707b525f4fb388809262370b18d33",
        5_944,
    ),
    (
        "validation/h0_primary_split_v10_coherent_teacher.py",
        "9c88c6a7873fdb197e56b85f855e0c5a6a597d5f053aa81f08aacdb08dab709f",
        25_334,
    ),
    (
        "validation/h0_primary_split_v10s_blend.py",
        "9e11a94eea1c32c187b7a0cf875d889e530252a3fae64cd017c87a2853b1f57d",
        9_273,
    ),
    (
        "validation/h0_primary_split_v10s_fit.py",
        "8ab610373f06e52c293c3da2cbf7409c035fc6366455c1b8052103e820b90594",
        52_821,
    ),
    (
        "validation/h0_primary_split_v10s_safe_dagger_contract.py",
        "5c6ea0993cb7585e7a588542382350bfdd1fd27b425373ff696d6ccb9cf77e0a",
        48_471,
    ),
    (
        "validation/h0_primary_split_v11_weighted_fit.py",
        "877bb79d9bf4bd8d87b9d07dbdee127ca259cc8cc82fac1e592ea37032029fdc",
        58_985,
    ),
    (
        "validation/h0_primary_split_v11_weighted_full_mean_contract.py",
        "1ee981c087973dd417f88214894171be21f75fde54fd821d8a0eb2958db34729",
        56_432,
    ),
    (
        "validation/h0_primary_split_v9_causal_teacher.py",
        "c17defea4867c2088b2ce373d51f36ea7da9224dae96c0aab13224834218fccc",
        5_047,
    ),
    (
        "validation/h0_primary_split_v9_causal_teacher_contract.py",
        "5c73bdafd388395dae1ba86bafb1cd52b2510f082c3b8ece433cd9308aa81d9e",
        6_460,
    ),
    (
        "validation/h0_v3_so_recovery_contract.py",
        "4c8e9407e2c2c42eb6050210ba795dbc215ae7ca57a304a84d344b88b007a26c",
        22_774,
    ),
    (
        "validation/run_h0_primary_grf_split_v6_teacher_replay.py",
        "582401e7ca258a859e1e7b30c6f5e47559db8d4a4afd2b833d9d0e19fc9c4b9b",
        51_448,
    ),
    (
        "validation/run_h0_primary_grf_split_v8_teacher_replay.py",
        "a0df53ffd776e1ee6bfcaec72995194d58800f5809373a7582dac857adfe6833",
        34_334,
    ),
    (
        "validation/run_h0_primary_grf_split_v8r1_teacher_replay.py",
        "cfc980126a2ef13ae548794ef975dfd885996921bf88aeee11eefa03e146e077",
        21_192,
    ),
    (
        "validation/run_h0_primary_grf_split_v8r1p1_teacher_replay.py",
        "3283e4d010f5e498f3bce4850ccde85d71f9b8620879c02dc48b2f5726aff1a3",
        11_744,
    ),
    (
        "validation/run_h0_primary_split_v10s_safe_dagger.py",
        "4240afdb37b99734f8dbe80b67c9dbef491a43ff65d6bbc78cab2f15a3300faf",
        100_367,
    ),
    (
        "validation/run_h0_primary_split_v9_causal_teacher.py",
        "4c95e53645546cbf1cbb197c605866f54403278fb8aec8d59642874d379ea19d",
        20_339,
    ),
    (
        "validation/run_h0_v25_abc_preflight.py",
        "344d55d7ecd1f5b9342b06438ccec769138823f3467706169dc6fc60a35fd128",
        63_570,
    ),
)

FROZEN_EXTERNAL_RUNTIME_SOURCES = {
    path: _artifact(path, sha256, size_bytes)
    for path, sha256, size_bytes in _FROZEN_EXTERNAL_RUNTIME_SOURCE_ROWS
}
LOCAL_R5_PRODUCTION_SOURCE_NAMES = (
    "h0_v12r5_case_balanced_contract.py",
    "freeze_h0_v12r5_case_balanced.py",
    "h0_v12r5_case_balanced_fitter.py",
    "run_h0_v12r5_case_balanced.py",
)
EXPECTED_PRODUCTION_SOURCE_COUNT = 68

Q2_DESIGN_FREEZE_ARTIFACT = _artifact(
    "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
    "h0_v12r4_q2_qualification_design_freeze.json",
    "d92fac765bd192f43ef2e0420a8529e8bb21860a3f47420ab5948863fb53eaf5",
    32_166,
)
Q2_UNOPENED_PATHS = {
    "protocol_freeze": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_protocol_freeze.json"
    ),
    "execution_lock": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_execution_lock.json"
    ),
    "run_root": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/h0_v12r4_q2_run_20260809"
    ),
    "noise_root": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_noise_tapes"
    ),
}

Q3_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r5q3")
Q3_DESIGN_FREEZE_PATH = Q3_ROOT / "h0_v12r5_q3_qualification_design_freeze.json"
Q3_DESIGN_FREEZE_STATUS = "PASS_H0_V12R5_Q3_QUALIFICATION_DESIGN_FREEZE"
Q3_DESIGN_FREEZE_ARTIFACT = _artifact(
    Q3_DESIGN_FREEZE_PATH.as_posix(),
    "f2764f2cf16abfc168255056fdf0c1407d97c65100bb3965b866f38db084e56d",
    68_465,
)
Q3_DESIGN_FREEZE_REQUIRED_BEFORE_EXECUTION_LOCK = True
Q3_UNOPENED_PATHS = {
    "protocol_freeze": Q3_ROOT / "h0_v12r5_q3_qualification_protocol_freeze.json",
    "execution_lock": Q3_ROOT / "h0_v12r5_q3_qualification_execution_lock.json",
    "run_root": Q3_ROOT / "h0_v12r5_q3_run_20260809",
    "noise_root": Q3_ROOT / "h0_v12r5_q3_qualification_noise_tapes",
}

SOURCE_H0_ID = v12r3.SOURCE_H0_ID
SOURCE_H0_MODULE_PATH = v12r3.SOURCE_H0_MODULE_PATH
SOURCE_H0_TREE_SHA256 = v12r3.SOURCE_H0_TREE_SHA256
TARGET_CONTRACT_ID = v12r3.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r3.EVENT_CONTRACT_ID
TEACHER_ID = v12r3.TEACHER_ID
TEACHER_EVIDENCE_ID = v12r3.TEACHER_EVIDENCE_ID
TEACHER_EVIDENCE_ARTIFACT = copy.deepcopy(v12r3.TEACHER_EVIDENCE_ARTIFACT)
EXPECTED_STEPS = v12r3.EXPECTED_STEPS
EXPECTED_ACTOR_FEATURES = v12r3.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v12r3.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v12r3.EXPECTED_ACTION_DIM
EXPECTED_DTYPE = v12r3.EXPECTED_DTYPE
EXPECTED_CONTROL_WINDOWS = v12r3.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = v12r3.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_SIGMA = 0.005
EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = 10
MORPHOLOGY_WEIGHT = v12r3.MORPHOLOGY_WEIGHT
PENETRATION_LIMIT_M = v12r3.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v12r3.MINIMUM_VALID_CYCLES
OFFLINE_THRESHOLDS = copy.deepcopy(v12r3.OFFLINE_THRESHOLDS)
ACTOR_ARCHITECTURE = copy.deepcopy(v12r3.ACTOR_ARCHITECTURE)
BASE_CORPUS_NORMALIZATION = copy.deepcopy(v12r3.BASE_CORPUS_NORMALIZATION)

P2_CORPUS_ROWS = 8_732
NOMINAL_PASS_ROWS = 500
NOMINAL_STUDENT_EXPOSED_ROWS = 255
CORPUS_ROWS = 9_232
P2_EPISODE_COUNT = 18
CORPUS_EPISODE_COUNT = 19
P2_RESET_ROWS = 18
CORPUS_RESET_ROWS = 19
CASE_TARGET_MASS = 1_000.0
CASE_COUNT = 6
NORMALIZED_TOTAL_MASS = CASE_TARGET_MASS * CASE_COUNT
HARDNESS_GATE_SCALE = 0.060
HARDNESS_MAX = 100.0
HARDNESS_POWER = 2
PURE_POLICY_COUNTER_FIELDS = (
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
)
CRITICAL_WINDOW = {
    "tranche_id": "v8r1p1_base",
    "case_id": "deterministic_offset_plus_0p20",
    "step_start_inclusive": 108,
    "step_end_inclusive": 230,
    "expected_rows": 123,
    "baseline_source": "BOUND_P2_MODULE_RECOMPUTED_NOT_ROUNDED_LITERAL",
    "diagnostic_approx_rmse": 0.0056222444,
    "diagnostic_approx_max_abs_error": 0.03706494,
}

CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "deterministic_offset_plus_0p20",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)

_BASE_START_S = 1.956870983805102
DEVELOPMENT_CASES = (
    {
        "case_id": "deterministic_offset_plus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S + 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "deterministic_offset_nominal",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "deterministic_offset_minus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S - 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "stochastic_nominal_seed_126",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 126,
        "runtime_seed": 126,
        "sigma": EXPECTED_SIGMA,
    },
    {
        "case_id": "stochastic_nominal_seed_127",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 127,
        "runtime_seed": 127,
        "sigma": EXPECTED_SIGMA,
    },
    {
        "case_id": "stochastic_nominal_seed_128",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 128,
        "runtime_seed": 128,
        "sigma": EXPECTED_SIGMA,
    },
)
DEVELOPMENT_CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)
DEVELOPMENT_PATHS = {
    case_id: DEVELOPMENT_ROOT / case_id for case_id in DEVELOPMENT_CASE_IDS
}

WEIGHTING = {
    "name": "P2_ERROR_HARDNESS_THEN_CASE_BALANCED_V1",
    "p2_prediction_source": copy.deepcopy(P2_MODULE_TREE),
    "per_row_error": "MAX_ABS_OVER_TWO_ACTIONS_OF_BOUND_P2_MINUS_TEACHER",
    "hardness_formula": "1+99*min(1,error/0.060)^2",
    "source_risk": "P2_RAW_SAMPLE_WEIGHT_OR_NOMINAL_RESET_100_ELSE_1",
    "raw_combination": "MAX_SOURCE_RISK_AND_HARDNESS",
    "case_balance": "SCALE_EACH_OF_SIX_CASE_IDS_TO_MASS_1000_FLOAT64",
    "case_target_mass": CASE_TARGET_MASS,
    "normalized_total_mass": NORMALIZED_TOTAL_MASS,
    "adaptive_refit": False,
    "post_fit_reweight": False,
    "sweep": False,
    "expected_p2_max_abs_error_sha256": "50a6ac8be1d0f98bf036a29e2937a486e377e3d2b53546dab90721afab04c7cd",
    "expected_hardness_sha256": "54ea40a4b157bdef63de3a22bd110dc40340928dee032bfa8b75320bd2512e4f",
    "expected_source_risk_sha256": "415008f5b86e065eeab6912fc521312025be44fddb7b70bb760c1efc4a6939b3",
    "expected_normalized_weights_sha256": "8eae1d8812d403eacab8b17070c29b318f042c3558b5515a1eb059677911a664",
}

FIT = {
    "fit_contract_id": FIT_CONTRACT_ID,
    "initial_checkpoint_id": SOURCE_H0_ID,
    "continued_from_p2": False,
    "actor_architecture": copy.deepcopy(ACTOR_ARCHITECTURE),
    "normalization": copy.deepcopy(BASE_CORPUS_NORMALIZATION),
    "weighting": copy.deepcopy(WEIGHTING),
    "trainable_scope": "full_mean_network",
    "freeze_logstd_head": True,
    "disabled_clock_columns": [0, 1],
    "adamw": {
        "optimizer": "AdamW",
        "seed": 20260807,
        "full_batch": True,
        "epochs": 3000,
        "learning_rate_schedule": [
            {"start_epoch": 1, "end_epoch": 1500, "learning_rate": 3.0e-4},
            {"start_epoch": 1501, "end_epoch": 2500, "learning_rate": 1.0e-4},
            {"start_epoch": 2501, "end_epoch": 3000, "learning_rate": 3.0e-5},
        ],
        "weight_decay": 1.0e-7,
        "grad_clip_norm": 10.0,
    },
    "lbfgs": {
        "optimizer": "LBFGS",
        "deterministic": True,
        "lr": 0.7,
        "max_iter": 600,
        "max_eval": 1200,
        "tolerance_grad": 1.0e-10,
        "tolerance_change": 1.0e-12,
        "history_size": 50,
        "line_search_fn": "strong_wolfe",
    },
    "optimizer_phase_order": ["adamw", "lbfgs"],
    "row_loss": "MEAN_SQUARED_ERROR_OVER_TWO_ACTIONS",
    "corpus_loss_reduction": "SUM_WEIGHTED_ROW_LOSS_DIVIDED_BY_SUM_WEIGHTS",
    "anchor_enabled": False,
    "hard_polish_enabled": False,
    "fallback_enabled": False,
    "sweep_enabled": False,
}

STAGE_IDS = (
    "attest_locked_inputs",
    "assemble_case_balanced_corpus",
    "fit_case_balanced_candidate",
    "freeze_case_balanced_candidate",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)

PROTOCOL_FREEZE_STATUS = "PASS_H0_V12R5_CASE_BALANCED_PROTOCOL_FREEZE"
EXECUTION_LOCK_STATUS = "PASS_H0_V12R5_CASE_BALANCED_EXECUTION_LOCK"
SOURCE_ATTEST_PASS_STATUS = "PASS_H0_V12R5_LOCKED_INPUT_ATTESTATION"
CORPUS_COMPLETE_STATUS = "COMPLETE_H0_V12R5_CASE_BALANCED_CORPUS"
FIT_COMPLETE_STATUS = "COMPLETE_H0_V12R5_CASE_BALANCED_FIT"
FIT_PASS_STATUS = "PASS_H0_V12R5_CASE_BALANCED_FIT"
CANDIDATE_FREEZE_COMPLETE_STATUS = "COMPLETE_H0_V12R5_CASE_BALANCED_CANDIDATE_FREEZE"
CANDIDATE_FREEZE_STATUS = "PASS_H0_V12R5_CASE_BALANCED_CANDIDATE_FREEZE"
DEVELOPMENT_COMPLETE_STATUS = "COMPLETE_H0_V12R5_CASE_BALANCED_DEVELOPMENT_CASE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R5_CASE_BALANCED_DEVELOPMENT_CASE"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R5_CASE_BALANCED_DEVELOPMENT"
PIPELINE_PASS_STATUS = "PASS_H0_V12R5_CASE_BALANCED_PIPELINE_TERMINAL"
TERMINAL_FAIL_STATUS = "FAIL_H0_V12R5_CASE_BALANCED_PIPELINE_TERMINAL"

Q3_PREREQUISITES = {
    "protocol_freeze": {
        "path": PROTOCOL_FREEZE_PATH.as_posix(),
        "status": PROTOCOL_FREEZE_STATUS,
    },
    "execution_lock": {
        "path": EXECUTION_LOCK_PATH.as_posix(),
        "status": EXECUTION_LOCK_STATUS,
    },
    "candidate_freeze": {
        "path": CANDIDATE_FREEZE_PATH.as_posix(),
        "status": CANDIDATE_FREEZE_STATUS,
    },
    "final_development_receipt": {
        "path": FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix(),
        "status": FINAL_DEVELOPMENT_PASS_STATUS,
    },
    "terminal_ledger": {
        "path": PIPELINE_LEDGER_PATH.as_posix(),
        "status": PIPELINE_PASS_STATUS,
    },
}


def candidate_id(tree_sha256: str) -> str:
    if not isinstance(tree_sha256, str) or len(tree_sha256) != 64:
        raise ValueError("candidate tree hash must be a SHA-256 hex digest")
    try:
        int(tree_sha256, 16)
    except ValueError as exc:
        raise ValueError("candidate tree hash must be hexadecimal") from exc
    return f"AB06_H0_V12R5_CASE_BALANCED:{tree_sha256}"


def canonical_development_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in DEVELOPMENT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V12R5 development case: {case_id!r}")
    return {
        **copy.deepcopy(matches[0]),
        "destination": DEVELOPMENT_PATHS[case_id].as_posix(),
        "behavior": "R5_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V12R5 stage: {stage_id!r}")
    if stage_id == "attest_locked_inputs":
        return {"stage_id": stage_id, "kind": "attestation"}
    if stage_id == "assemble_case_balanced_corpus":
        return {"stage_id": stage_id, "kind": "corpus"}
    if stage_id == "fit_case_balanced_candidate":
        return {"stage_id": stage_id, "kind": "fit"}
    if stage_id == "freeze_case_balanced_candidate":
        return {"stage_id": stage_id, "kind": "candidate_freeze"}
    if stage_id.startswith("development__"):
        case_id = stage_id.removeprefix("development__")
        return {
            "stage_id": stage_id,
            "kind": "development",
            "case": canonical_development_case(case_id),
        }
    return {"stage_id": stage_id, "kind": "finalize"}


def expected_corpus_counts() -> dict[str, Any]:
    return {
        "p2_sample_count": P2_CORPUS_ROWS,
        "nominal_pass_sample_count": NOMINAL_PASS_ROWS,
        "nominal_student_exposed_sample_count": NOMINAL_STUDENT_EXPOSED_ROWS,
        "failed_plus_prefix_sample_count": 0,
        "sample_count": CORPUS_ROWS,
        "episode_count": CORPUS_EPISODE_COUNT,
        "reset_row_count": CORPUS_RESET_ROWS,
        "case_count": CASE_COUNT,
        "case_target_mass": CASE_TARGET_MASS,
        "normalized_total_sample_mass": NORMALIZED_TOTAL_MASS,
        "component_order": ["p2_corpus", "r4_nominal_pass_labels_only"],
    }


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def pure_policy_trace_audit(trace: Any, *, case_id: str) -> dict[str, Any]:
    """Recompute pure-policy evidence from an in-memory development trace."""

    canonical_development_case(case_id)
    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    counters = {name: 0 for name in PURE_POLICY_COUNTER_FIELDS}
    schema_exact = len(rows) == EXPECTED_STEPS
    identity_exact = schema_exact
    action_path_exact = schema_exact
    raw_sensor_exact = schema_exact
    forbidden_teacher_payload_absent = schema_exact
    per_row_zero_counters = schema_exact
    forbidden = {
        "teacher_mean",
        "teacher_action",
        "blended_mean",
        "requested_alpha",
        "effective_alpha",
        "safety_latch_active",
    }
    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            schema_exact = False
            identity_exact = False
            action_path_exact = False
            raw_sensor_exact = False
            forbidden_teacher_payload_absent = False
            continue
        identity_exact = identity_exact and (
            row.get("step") == expected_step
            and row.get("schema_version") == SCHEMA_VERSION
            and row.get("protocol_id") == PROTOCOL_ID
            and row.get("stage_id") == f"development__{case_id}"
            and row.get("case_id") == case_id
        )
        schema_exact = schema_exact and all(
            type(row.get(name)) is int for name in PURE_POLICY_COUNTER_FIELDS
        )
        for name in PURE_POLICY_COUNTER_FIELDS:
            value = row.get(name)
            if type(value) is int:
                counters[name] += value
            per_row_zero_counters = per_row_zero_counters and value == 0
        schema_exact = schema_exact and (
            row.get("teacher_enabled") is False
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
        )
        forbidden_teacher_payload_absent = (
            forbidden_teacher_payload_absent and forbidden.isdisjoint(row)
        )
        mean = row.get("candidate_mean")
        noise = row.get("single_noise")
        action = row.get("raw_action")
        vectors = (mean, noise, action)
        action_path_exact = action_path_exact and all(
            isinstance(value, list)
            and len(value) == EXPECTED_ACTION_DIM
            and all(_finite_number(item) for item in value)
            for value in vectors
        )
        if action_path_exact:
            action_path_exact = all(
                math.isclose(
                    float(action[index]),
                    float(mean[index]) + float(noise[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        raw_sensor_exact = raw_sensor_exact and (
            row.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        )
    zero_counters = all(value == 0 for value in counters.values())
    passed = all(
        (
            schema_exact,
            identity_exact,
            action_path_exact,
            raw_sensor_exact,
            forbidden_teacher_payload_absent,
            per_row_zero_counters,
            zero_counters,
        )
    )
    return {
        "passed": passed,
        "row_count": len(rows),
        "schema_exact": schema_exact,
        "identity_exact": identity_exact,
        "candidate_mean_plus_noise_exact": action_path_exact,
        "raw_sensor_samples_exact": raw_sensor_exact,
        "forbidden_teacher_payload_absent": forbidden_teacher_payload_absent,
        "per_row_zero_counters": per_row_zero_counters,
        "zero_counters": zero_counters,
        "counters": counters,
    }


def _metric_triplet_within_gate(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    names = ("rmse", "max_abs_error", "reset_max_abs_error")
    if not all(_finite_number(value.get(name)) for name in names):
        return False
    return (
        0.0 <= float(value["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"]
        and 0.0
        <= float(value["max_abs_error"])
        <= OFFLINE_THRESHOLDS["max_abs_error_max"]
        and 0.0
        <= float(value["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"]
    )


def _critical_non_regression(summary: Mapping[str, Any]) -> bool:
    baseline = summary.get("critical_window_p2_baseline_metrics")
    candidate = summary.get("critical_window_metrics")
    if not isinstance(baseline, Mapping) or not isinstance(candidate, Mapping):
        return False
    names = ("rmse", "max_abs_error")
    return (
        all(_finite_number(baseline.get(name)) for name in names)
        and all(_finite_number(candidate.get(name)) for name in names)
        and all(
            0.0 <= float(candidate[name]) <= float(baseline[name]) for name in names
        )
        and summary.get("critical_window") == CRITICAL_WINDOW
        and summary.get("critical_window_p2_baseline_recomputed") is True
        and summary.get("critical_window_p2_module_tree") == P2_MODULE_TREE
    )


def fit_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    per_case = summary.get("per_case_metrics")
    per_case_pass = (
        isinstance(per_case, Mapping)
        and set(per_case) == set(CASE_IDS)
        and all(_metric_triplet_within_gate(per_case[name]) for name in CASE_IDS)
    )
    case_mass = summary.get("case_mass")
    case_mass_pass = (
        isinstance(case_mass, Mapping)
        and set(case_mass) == set(CASE_IDS)
        and all(
            _finite_number(case_mass[name])
            and math.isclose(
                float(case_mass[name]), CASE_TARGET_MASS, rel_tol=0.0, abs_tol=1e-9
            )
            for name in CASE_IDS
        )
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == FIT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "fit_exact": summary.get("fit") == FIT,
        "selection_deferred": summary.get("candidate_selection_rule")
        == CANDIDATE_SELECTION_RULE,
        "fresh_h0": summary.get("initial_checkpoint_id") == SOURCE_H0_ID
        and summary.get("continued_from_p2") is False,
        "corpus": summary.get("fit_counts") == expected_corpus_counts()
        and summary.get("sample_count") == CORPUS_ROWS
        and summary.get("episode_count") == CORPUS_EPISODE_COUNT
        and summary.get("reset_row_count") == CORPUS_RESET_ROWS,
        "failed_prefix_excluded": summary.get("failed_plus_prefix_rows_loaded") == 0
        and summary.get("corpus_components")
        == ["p2_corpus", "r4_nominal_pass_labels_only"],
        "weights": summary.get("weighting") == WEIGHTING
        and summary.get("weight_hashes")
        == {
            "p2_max_abs_error_sha256": WEIGHTING["expected_p2_max_abs_error_sha256"],
            "hardness_sha256": WEIGHTING["expected_hardness_sha256"],
            "source_risk_sha256": WEIGHTING["expected_source_risk_sha256"],
            "normalized_weights_sha256": WEIGHTING[
                "expected_normalized_weights_sha256"
            ],
        }
        and case_mass_pass
        and math.isclose(
            float(summary.get("normalized_total_sample_mass", math.nan)),
            NORMALIZED_TOTAL_MASS,
            rel_tol=0.0,
            abs_tol=1e-9,
        ),
        "global_metrics": _metric_triplet_within_gate(summary.get("metrics")),
        "p2_subset_metrics": _metric_triplet_within_gate(
            summary.get("p2_subset_metrics")
        ),
        "nominal_r4_pass_metrics": _metric_triplet_within_gate(
            summary.get("nominal_r4_pass_metrics")
        ),
        "nominal_r4_student_exposed_metrics": _metric_triplet_within_gate(
            summary.get("nominal_r4_student_exposed_metrics")
        ),
        "per_case_metrics": per_case_pass,
        "critical_window_non_regression": _critical_non_regression(summary),
        "optimizer": summary.get("adamw_epochs_run") == 3000
        and summary.get("lbfgs_max_iter") == 600
        and summary.get("lbfgs_max_eval") == 1200
        and summary.get("deterministic_algorithms_enabled") is True,
        "preservation": summary.get("source_h0_byte_exact") is True
        and summary.get("logstd_byte_exact") is True
        and summary.get("critic_present") is False
        and summary.get("disabled_clock_columns_bit_zero") is True
        and summary.get("save_reload_exact") is True,
        "one_fit": summary.get("actor_updates") == 1
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_rescue": summary.get("hard_polish_used") is False
        and summary.get("fallback_used") is False
        and summary.get("sweep_used") is False,
        "v26_unchanged": summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FIT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_promoted": False,
        "next_stage": "FREEZE_CASE_BALANCED_CANDIDATE" if passed else "STOP_TERMINAL",
    }


def candidate_freeze_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    module = summary.get("candidate_module")
    tree_sha256 = module.get("tree_sha256") if isinstance(module, Mapping) else None
    valid_tree = False
    if isinstance(tree_sha256, str) and len(tree_sha256) == 64:
        try:
            int(tree_sha256, 16)
            valid_tree = True
        except ValueError:
            pass
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "selection_rule": summary.get("candidate_selection_rule")
        == CANDIDATE_SELECTION_RULE,
        "fit_passed": summary.get("fit_passed") is True,
        "candidate_identity": valid_tree
        and summary.get("candidate_id") == candidate_id(tree_sha256),
        "candidate_frozen": summary.get("candidate_frozen") is True,
        "preservation": summary.get("source_h0_byte_exact") is True
        and summary.get("logstd_byte_exact") is True
        and summary.get("critic_present") is False
        and summary.get("save_reload_exact") is True,
        "zero_extra_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_promotion": summary.get("runtime_promoted") is False,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CANDIDATE_FREEZE_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_id": summary.get("candidate_id"),
        "candidate_module": copy.deepcopy(module),
        "next_stage": "DEVELOPMENT_CRITICAL_PLUS_FIRST" if passed else "STOP_TERMINAL",
    }


def development_gate(
    summary: Mapping[str, Any], *, case_id: str, trace: Any = None
) -> dict[str, Any]:
    case = canonical_development_case(case_id)
    trace_audit = pure_policy_trace_audit(trace, case_id=case_id)
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == DEVELOPMENT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "case": summary.get("case") == case,
        "pure_candidate": summary.get("teacher_enabled") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and trace_audit.get("passed") is True
        and summary.get("pure_policy_trace_audit") == trace_audit
        and summary.get("pure_policy_trace_row_count") == EXPECTED_STEPS
        and all(
            summary.get(name) == 0 and trace_audit["counters"].get(name) == 0
            for name in PURE_POLICY_COUNTER_FIELDS
        ),
        "full_duration": summary.get("steps") == EXPECTED_STEPS
        and summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and summary.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES,
        "physical": summary.get("phase_valid_cycle_count", -1) >= MINIMUM_VALID_CYCLES
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "zero_invalids": all(
            summary.get(name) == 0
            for name in (
                "action_clipped_values",
                "fallback_count",
                "timeout_count",
                "safety_stop_count",
                "sea_plugin_fallback_count",
                "so_solver_unaccepted_count",
                "hard_invalid_count",
                "invalid_event_count",
                "nonfinite_count",
                "routing_failure_count",
                "step_contract_failure_count",
                "binary_event_failure_count",
            )
        ),
        "sea_reserve": summary.get("sea_reserve_gate_passed") is True,
        "v26_unchanged": summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DEVELOPMENT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "pure_policy_trace_audit": trace_audit,
        "next_stage": "NEXT_FROZEN_STAGE" if passed else "STOP_TERMINAL",
    }


def aggregate_development_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    bindings = summary.get("case_gates")
    ordered = (
        isinstance(bindings, list)
        and len(bindings) == len(DEVELOPMENT_CASE_IDS)
        and all(
            isinstance(binding, Mapping)
            and binding.get("case_id") == case_id
            and binding.get("passed") is True
            for binding, case_id in zip(bindings, DEVELOPMENT_CASE_IDS, strict=True)
        )
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "critical_plus_first": DEVELOPMENT_CASE_IDS[0]
        == "deterministic_offset_plus_0p20",
        "six_of_six": ordered,
        "fixed_candidate": summary.get("candidate_tree_unique_count") == 1,
        "activity": summary.get("new_collection_count") == 0
        and summary.get("development_count") == 6
        and summary.get("environment_reset_calls") == 6
        and summary.get("environment_step_calls") == 3000
        and summary.get("raw_sensor_sample_count") == 30_000
        and summary.get("teacher_query_count") == 0
        and summary.get("pure_policy_trace_row_count") == 3000
        and all(summary.get(name) == 0 for name in PURE_POLICY_COUNTER_FIELDS)
        and summary.get("actor_updates") == 1
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_retry": summary.get("retry_authorized") is False
        and summary.get("resume_authorized") is False
        and summary.get("rescue_authorized") is False,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
        "no_promotion": summary.get("runtime_promoted") is False
        and summary.get("checkpoint_zero_created") is False
        and summary.get("positive_morphology_enabled") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FINAL_DEVELOPMENT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "next_stage": "WAIT_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL",
    }


def contract_self_check() -> dict[str, Any]:
    checks = {
        "additive_paths": all(
            str(path).startswith(f"{VALIDATION_ROOT.as_posix()}/")
            for path in (
                PROTOCOL_FREEZE_PATH,
                DESIGN_AUDIT_PATH,
                EXECUTION_LOCK_PATH,
                RUN_ROOT,
            )
        ),
        "p2_bound": P2_CORPUS_ARTIFACT["sha256"]
        == "42a40869447aec0cdce62a6ba5fcb48da4e5f070fac9e8eb19c4f0a8fccc1990"
        and P2_MODULE_TREE["tree_sha256"]
        == "be0c9711f1f6b7b9dba9cd4ab546a8379f96444ddf02a111d5f3002b9c7e8c4f",
        "r4_terminal_bound": set(R4_TERMINAL_ARTIFACTS)
        == {"protocol_freeze", "execution_lock", "terminal_ledger"},
        "nominal_only": set(R4_NOMINAL_REUSABLE_ARTIFACTS)
        == {"labels", "receipt", "gate", "summary", "trace"}
        and R4_NOMINAL_REUSABLE_ARTIFACTS["labels"]["size_bytes"] == 793_510,
        "failed_plus_forensic_only": set(R4_PLUS_FAILURE_EVIDENCE)
        == {"labels_excluded", "failure", "gate", "summary"},
        "external_runtime_closed": len(FROZEN_EXTERNAL_RUNTIME_SOURCES) == 64
        and all(
            path == record["path"]
            for path, record in FROZEN_EXTERNAL_RUNTIME_SOURCES.items()
        )
        and {
            "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
            "Trajectory Generator/baseline_MLP/target_domain_imitation.py",
            "Trajectory Generator/baseline_MLP/validation/v12r3/"
            "run_h0_primary_split_v12r3_autonomy_recovery.py",
            "Trajectory Generator/osim_trj_cmc_like.py",
            "simulation_runner.py",
        }.issubset(FROZEN_EXTERNAL_RUNTIME_SOURCES)
        and not any(
            PurePosixPath(path).name.startswith("test_")
            for path in FROZEN_EXTERNAL_RUNTIME_SOURCES
        )
        and all(
            len(record["sha256"]) == 64
            for record in FROZEN_EXTERNAL_RUNTIME_SOURCES.values()
        ),
        "corpus_counts": expected_corpus_counts()["sample_count"] == 9_232
        and expected_corpus_counts()["failed_plus_prefix_sample_count"] == 0
        and expected_corpus_counts()["normalized_total_sample_mass"] == 6_000.0,
        "case_balance": CASE_IDS
        == (
            "deterministic_offset_minus_0p20",
            "deterministic_offset_nominal",
            "deterministic_offset_plus_0p20",
            "stochastic_nominal_seed_126",
            "stochastic_nominal_seed_127",
            "stochastic_nominal_seed_128",
        )
        and WEIGHTING["adaptive_refit"] is False
        and WEIGHTING["sweep"] is False,
        "fit_fresh_h0": FIT["initial_checkpoint_id"] == SOURCE_H0_ID
        and FIT["continued_from_p2"] is False
        and FIT["adamw"]["epochs"] == 3000
        and FIT["lbfgs"]["max_iter"] == 600
        and FIT["lbfgs"]["max_eval"] == 1200,
        "critical_first": DEVELOPMENT_CASE_IDS
        == (
            "deterministic_offset_plus_0p20",
            "deterministic_offset_nominal",
            "deterministic_offset_minus_0p20",
            "stochastic_nominal_seed_126",
            "stochastic_nominal_seed_127",
            "stochastic_nominal_seed_128",
        ),
        "stage_order": len(STAGE_IDS) == 11
        and STAGE_IDS[:4]
        == (
            "attest_locked_inputs",
            "assemble_case_balanced_corpus",
            "fit_case_balanced_candidate",
            "freeze_case_balanced_candidate",
        )
        and STAGE_IDS[-1] == "finalize_development",
        "no_collection_authority": AUTHORITY["new_environment_collection_authorized"]
        is False,
        "one_shot": not any(
            AUTHORITY[name]
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "runtime_promotion_authorized",
                "checkpoint_zero_authorized",
                "positive_morphology_authorized",
            )
        ),
        "q2_historical": Q2_DESIGN_FREEZE_ARTIFACT["path"].startswith(
            "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        ),
        "q3_deferred": Q3_DESIGN_FREEZE_REQUIRED_BEFORE_EXECUTION_LOCK is True
        and Q3_DESIGN_FREEZE_ARTIFACT["path"] == Q3_DESIGN_FREEZE_PATH.as_posix()
        and Q3_DESIGN_FREEZE_ARTIFACT["sha256"]
        == "f2764f2cf16abfc168255056fdf0c1407d97c65100bb3965b866f38db084e56d"
        and all(
            str(path).startswith(f"{Q3_ROOT.as_posix()}/")
            for path in Q3_UNOPENED_PATHS.values()
        ),
        "v26_unchanged": EVENT_CONTRACT_ID == "binary_point_v25+heel_qualified_fsm_v2",
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R5_CASE_BALANCED_CONTRACT_SELF_CHECK"
            if passed
            else "FAIL_H0_V12R5_CASE_BALANCED_CONTRACT_SELF_CHECK"
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
    }


__all__ = [
    "ACTOR_ARCHITECTURE",
    "AUTHORITY",
    "CANDIDATE_FREEZE_STATUS",
    "CANDIDATE_MODULE_PATH",
    "CANDIDATE_SELECTION_RULE",
    "CASE_IDS",
    "CORPUS_PATH",
    "CORPUS_ROWS",
    "CRITICAL_WINDOW",
    "DEVELOPMENT_CASES",
    "DEVELOPMENT_CASE_IDS",
    "EVENT_CONTRACT_ID",
    "EXECUTION_LOCK_PUBLICATION_FAILURE_PATH",
    "EXECUTION_LOCK_STATUS",
    "FINAL_DEVELOPMENT_PASS_STATUS",
    "FIT",
    "FIT_CONTRACT_ID",
    "FIT_ROOT",
    "FROZEN_EXTERNAL_RUNTIME_SOURCES",
    "LOCAL_R5_PRODUCTION_SOURCE_NAMES",
    "EXPECTED_PRODUCTION_SOURCE_COUNT",
    "NORMALIZED_TOTAL_MASS",
    "NOMINAL_PASS_ROWS",
    "NOMINAL_STUDENT_EXPOSED_ROWS",
    "OFFLINE_THRESHOLDS",
    "P2_CORPUS_ARTIFACT",
    "P2_CORPUS_ROWS",
    "P2_MODULE_TREE",
    "PIPELINE_CLAIM_FAILURE_PATH",
    "PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH",
    "PIPELINE_PASS_STATUS",
    "PROTOCOL_PUBLICATION_FAILURE_PATH",
    "PURE_POLICY_COUNTER_FIELDS",
    "PROTOCOL_FREEZE_STATUS",
    "PROTOCOL_ID",
    "Q2_DESIGN_FREEZE_ARTIFACT",
    "Q2_UNOPENED_PATHS",
    "Q3_DESIGN_FREEZE_PATH",
    "Q3_DESIGN_FREEZE_ARTIFACT",
    "Q3_DESIGN_FREEZE_STATUS",
    "Q3_PREREQUISITES",
    "Q3_UNOPENED_PATHS",
    "R4_NOMINAL_REUSABLE_ARTIFACTS",
    "R4_PLUS_FAILURE_EVIDENCE",
    "R4_TERMINAL_ARTIFACTS",
    "REVISION",
    "SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS",
    "SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT",
    "SCHEMA_VERSION",
    "STAGE_IDS",
    "TARGET_CONTRACT_ID",
    "TERMINAL_FAIL_STATUS",
    "WEIGHTING",
    "aggregate_development_gate",
    "candidate_freeze_gate",
    "candidate_id",
    "canonical_development_case",
    "contract_self_check",
    "development_gate",
    "expected_corpus_counts",
    "fit_gate",
    "pure_policy_trace_audit",
    "stage_descriptor",
]
