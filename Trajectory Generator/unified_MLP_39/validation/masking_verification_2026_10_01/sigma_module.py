"""Experimental module: original mean and optional immutable CP90 sigma map.

Reference weights are buffers, not parameters: original Adam topology is kept.
Both sampling and PPO forward use the same distribution. Checkpoint metadata
serializes this importable class and the treatment; buffers serialize the map.
"""
import hashlib
import numpy as np
import torch
from unified_MLP_39.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule


class SigmaComparisonModule(AsymmetricActorCriticTorchRLModule):
    def setup(self):
        super().setup()
        treatment = self.model_config.get("diagnostic_sigma_treatment", "learned")
        if treatment not in ("learned", "frozen_cp90"):
            raise ValueError(treatment)
        self._reference_names = tuple(name for name, _ in self.pi.named_parameters())
        for i, (_, value) in enumerate(self.pi.named_parameters()):
            self.register_buffer(f"_sigma_ref_{i}", torch.zeros_like(value))
        self.register_buffer("_sigma_reference_ready", torch.tensor(False))
        self.register_buffer("_sigma_treatment_code", torch.tensor(int(treatment == "frozen_cp90")))

    def set_state(self, state):
        state = dict(state)
        if "_sigma_reference_ready" not in state:
            # The initial fork imports a complete original CP90 actor unchanged.
            for i, name in enumerate(self._reference_names):
                if f"pi.{name}" not in state:
                    raise ValueError(f"Incomplete source actor: pi.{name}")
                state[f"_sigma_ref_{i}"] = state[f"pi.{name}"]
            state["_sigma_reference_ready"] = np.asarray(True)
            state["_sigma_treatment_code"] = self._sigma_treatment_code.detach().cpu().numpy()
        expected = int(self.model_config.get("diagnostic_sigma_treatment", "learned") == "frozen_cp90")
        if int(state["_sigma_treatment_code"]) != expected:
            raise ValueError("Cannot restore a different sigma treatment")
        super().set_state(state)

    def reference_logits(self, obs):
        if not bool(self._sigma_reference_ready):
            raise RuntimeError("CP90 reference has not been restored")
        params = {name: getattr(self, f"_sigma_ref_{i}") for i, name in enumerate(self._reference_names)}
        with torch.no_grad():
            return torch.func.functional_call(self.pi, params, (obs,))

    def _policy_logits(self, batch):
        logits = super()._policy_logits(batch)
        if int(self._sigma_treatment_code) == 0:
            return logits
        reference = self.reference_logits(self._actor_in(batch))
        return torch.cat((logits[..., :self._action_dim], reference[..., self._action_dim:]), dim=-1)

    def sigma_audit(self, probes):
        obs = torch.as_tensor(probes, dtype=torch.float32, device=self._sigma_treatment_code.device)
        reference = self.reference_logits(obs[:, :self._n_actor])
        output = self._policy_logits({"obs": obs})
        delta = torch.max(torch.abs(output[:, self._action_dim:]-reference[:, self._action_dim:])).item()
        h = hashlib.sha256()
        for i in range(len(self._reference_names)):
            h.update(getattr(self, f"_sigma_ref_{i}").detach().cpu().numpy().tobytes())
        if int(self._sigma_treatment_code) == 1 and delta != 0:
            raise AssertionError("Frozen sigma changed on fixed probes")
        return {"treatment": int(self._sigma_treatment_code), "reference_sha256": h.hexdigest(),
                "reference_ready": bool(self._sigma_reference_ready),
                "logstd_max_delta_vs_cp90_on_fixed_probes": delta,
                "parameter_tensors": len(list(self.parameters())),
                "probe_count": len(obs)}


def audit_learner(learner, probes):
    return learner.module["default_policy"].sigma_audit(probes)


def audit_runner(runner, probes):
    module = runner.module
    if not hasattr(module, "sigma_audit"):
        module = module["default_policy"]
    return module.sigma_audit(probes)
