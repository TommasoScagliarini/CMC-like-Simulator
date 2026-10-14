"""Exact CP90 fork, gradient, immutable sigma, export and local learner checks."""
import copy
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))
for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[k] = "1"
from baseline_MLP import win_runtime
from unified_MLP_39 import train_ppo_mlp as driver, optimizer_resume, reward_resume, training_runtime
from unified_MLP_39.validation.masking_verification_2026_10_01.sigma_module import SigmaComparisonModule
import numpy as np
import torch
import yaml

SOURCE = PACKAGE / "runs/training/MLP39_LEFT_fresh_best80config_100iter_2026-09-29/milestone_iteration_000090/checkpoint_last"


def configurations():
    base = yaml.safe_load((PACKAGE / "runs/plans/left_best90_masking_100iter_2026-09-30/stage01_mask25.yaml").read_text())
    for treatment in ("learned", "frozen_cp90"):
        cfg = copy.deepcopy(base)
        cfg["simulation"]["iterations"] = 100
        cfg["target_availability"]["target_curriculum_probabilities"] = [0.75]
        cfg["run"]["output_dir"] = f"MLP39_masking_sigma_verification_2026-10-01/{treatment}"
        cfg["provenance"] = {"experiment": "masking_sigma_verification", "sigma_treatment": treatment,
            "source_checkpoint": str(SOURCE.relative_to(ROOT)), "source_iteration": 90,
            "updates": 10, "masked_episode_probability": 0.25, "bootstrap_fix": "deferred_unchanged",
            "authorization": "user: procedi con la verifica completa con anche i training"}
        (HERE / f"{treatment}.yaml").write_text(yaml.safe_dump(cfg,sort_keys=False))


def main():
    os.chdir(ROOT)
    configurations()
    driver._load_training_stack()
    from ray.rllib.core.rl_module.rl_module import RLModule, RLModuleSpec
    sys.argv = [str(HERE / "preflight_sigma.py"), "--config", str(HERE / "learned.yaml")]
    args = driver.parse_args()
    driver._validate_unified_args(args)
    config = driver.build_config(args, args._cfg_reward)
    from unified_MLP_39 import env_factory
    env = env_factory.make_cmc_env(config.env_config)
    learner = config.build_learner(env=env)
    try:
        learner.restore_from_path(SOURCE / "learner_group/learner")
        restored = optimizer_resume.restore_and_audit(learner,SOURCE,lambda owner,fn,**kw:[fn(owner,**kw)])
        original = learner.module["default_policy"]
        state = original.get_state()
        parameter_check = reward_resume.audit_restored_parameters(SOURCE,state,training_runtime.critic_digest)
        trace = json.loads((PACKAGE / "runs/rollout/unified_MLP_39/runs/rollout/MLP39_LEFT_fresh_best90_2026-09-30_deterministic/rollout_policy_trace.json").read_text())
        x = np.asarray([r["actor_observation_vector_before"] for r in trace],np.float32)
        hidden = x.copy(); hidden[:,[0,1,35,36,37,38]]=0
        full = np.zeros((2*len(x), original._n_full),np.float32)
        full[:,:39] = np.concatenate([x,hidden])
        probes = torch.as_tensor(full)
        modules, checks = {}, []
        for treatment in ("learned", "frozen_cp90"):
            cfg = dict(original.model_config,diagnostic_sigma_treatment=treatment)
            module = RLModuleSpec(module_class=SigmaComparisonModule,
                observation_space=original.observation_space,action_space=original.action_space,
                model_config=cfg).build()
            module.set_state(state)
            assert [n for n,_ in module.named_parameters()] == [n for n,_ in original.named_parameters()]
            with torch.no_grad():
                expected = original._policy_logits({"obs":probes})
                actual = module._policy_logits({"obs":probes})
            assert torch.equal(expected,actual)
            # Sampling/inference/train distributions must use identical logits.
            for method in (module.forward_inference,module.forward_exploration,module.forward_train):
                assert torch.equal(method({"obs":probes})["action_dist_inputs"].detach(),expected)
            module.zero_grad()
            logits = module._policy_logits({"obs":probes})
            loss = logits[:,:2].square().mean() + logits[:,2:].square().mean()
            loss.backward()
            assert any(p.grad is not None and p.grad.abs().max()>0 for p in module.pi_encoder.parameters())
            head = module.pi[-1]
            if treatment == "frozen_cp90":
                assert torch.count_nonzero(head.weight.grad[2:]) == 0
                assert torch.count_nonzero(head.bias.grad[2:]) == 0
            before = module.sigma_audit(probes)
            saved = copy.deepcopy(module.get_state())
            with torch.no_grad():
                next(module.pi_encoder.parameters()).add_(0.01)
            changed = module._policy_logits({"obs":probes}).detach()
            assert not torch.equal(changed[:,:2],expected[:,:2])
            if treatment == "frozen_cp90":
                assert torch.equal(changed[:,2:],expected[:,2:])
            else:
                assert not torch.equal(changed[:,2:],expected[:,2:])
            assert module.sigma_audit(probes)["reference_sha256"] == before["reference_sha256"]
            module.set_state(saved)
            folder = HERE / "preflight_exports" / treatment
            folder.mkdir(parents=True,exist_ok=True)
            module.save_to_path(folder)
            reloaded = RLModule.from_checkpoint(str(folder))
            assert torch.equal(reloaded._policy_logits({"obs":probes}),expected)
            custom_config = config.copy(copy_frozen=False)
            custom_config.rl_module(rl_module_spec=RLModuleSpec(module_class=SigmaComparisonModule,
                observation_space=original.observation_space,action_space=original.action_space,
                model_config=cfg))
            custom_learner = custom_config.build_learner(env=env)
            custom_learner.restore_from_path(SOURCE / "learner_group/learner")
            custom_restore = optimizer_resume.restore_and_audit(custom_learner,SOURCE,lambda owner,fn,**kw:[fn(owner,**kw)])
            restored_module = custom_learner.module["default_policy"]
            assert isinstance(restored_module,SigmaComparisonModule)
            assert int(restored_module._sigma_treatment_code) == int(treatment=="frozen_cp90")
            assert torch.equal(restored_module._policy_logits({"obs":probes}),expected)
            checks.append({"treatment":treatment,"original_logits_exact":True,"all_forward_paths_equal":True,
                "gradient_routing_passed":True,"sigma_function_perturbation_test":True,
                "export_reload_exact":True,"custom_learner_restore":custom_restore,**before})
            modules[treatment] = module
        result = {"passed":True,"ppo_updates":0,"probe_rows":len(probes),
                  "full_restore":restored,"parameters":parameter_check,"modules":checks}
        (HERE / "preflight_sigma.json").write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps({"passed":True,"ppo_updates":0,"probe_rows":len(probes),"module_checks":checks}))
    finally:
        env.close()


if __name__ == "__main__":
    main()
