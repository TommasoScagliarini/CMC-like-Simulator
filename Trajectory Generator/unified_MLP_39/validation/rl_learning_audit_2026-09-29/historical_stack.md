# Historical Ray/RLlib stack: V26 and unified39

The V26 benchmark, LEFT best80 and the September 29 audit record **the same Ray 2.55.1 build**, commit `237c2455ebb1ea15a32dd9e1fdeb2d617badc37f`. There is no evidence here of a Ray upgrade separating V26 from unified39. The newly reproduced bootstrap issue is therefore a potentially shared training problem; it must not be presented as the established cause of the LEFT imitation gap.

| Case | Primary checkpoint metadata, relative to the repository |
|---|---|
| Actual V26 benchmark, August 20 | `Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/checkpoint_best/rllib_checkpoint.json` |
| LEFT best80, September 24 | `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT72_penetration_weight2_10iter_2026-09-24/checkpoint_best/rllib_checkpoint.json` |
| Fresh one-batch audit, September 29 | `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29/checkpoint_last/rllib_checkpoint.json` |

The metadata fields are on line 1 of each file. Their hashes, the complete metadata, and the serialized constructor evidence are saved in [historical_stack.json](historical_stack.json).

Disassembling each checkpoint's `class_and_ctor_args.pkl` and `learner_group/learner/class_and_ctor_args.pkl` with Python `pickletools` gives the same relevant settings:

| Setting | All three checkpoints |
|---|---|
| Learner class | stock `ray.rllib.algorithms.ppo.torch.ppo_torch_learner.PPOTorchLearner` |
| `_learner_connector` | `None` |
| `add_default_connectors_to_learner_pipeline` | `True` |
| RLModule/Learner API and ConnectorV2 | enabled |
| `batch_mode` | `truncate_episodes` |
| `rollout_fragment_length` | `auto` |
| `normalize_actions` / `clip_actions` | `True` / `False` |

The V26 resolved configuration additionally records batch 4096, 13 EnvRunners, and `exact_start_sampling: false` (lines 17–31). These settings establish exposure to the same stock connector path under fragmented collection, beyond merely sharing a version number. Inspection did not instantiate an algorithm or unpickle constructors; the JSON preserves literal byte offsets and source hashes.

July checkpoints from July 3, 7, 8 and 10 also record Ray 2.55.1 and the same commit. These are background evidence about the July runtime, not replacements for the actual August V26 benchmark. The August 9 readiness report independently lists Ray 2.55.1 at `reports/user/2026-08-09_h0_v12r2_training_readiness.md:202`; the July 13 multistart report explicitly discusses RLlib 2.55 at `reports/user/2026-07-13_h1bis_multistart_optimizer_resume_h2_approvata.md:22`.

The July 14 fix should be kept distinct. `reports/daily/2026-07-14_daily-report.md:141` and `reports/user/2026-07-14_sampling_ppo_bilanciato_e_gate_robusto.md:55` describe 4622 rows after GAE, including 14 bootstrap rows. Their fix uses `loss_mask` to compact 4608 real transitions and interleave start conditions in minibatches. That is a padding/sampling correction after GAE; the reports do not document a correction to the continuation bootstrap value. The V26 benchmark explicitly disabled that exact-start sampling mode.

Limits:

- Package metadata cannot exclude undocumented local modifications of installed Ray source during historical training. Hashes of the current installed connector/GAE/value-target files are saved as contemporary evidence only.
- Historical V26 fragment inputs and actual computed value targets have not been reconstructed here. No fraction of affected V26 samples, historical loss impact, or counterfactual V26 result is inferred.
- Same software/settings support applicability of the mechanism, not equal impact across targets, actors, critics, reset distributions or training histories. They do not isolate the reason LEFT differs from RIGHT or V26.

No training, simulation, dependency modification, or production-code edit was performed for this historical check.
