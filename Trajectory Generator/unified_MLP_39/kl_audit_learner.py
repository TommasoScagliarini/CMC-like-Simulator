"""Per-minibatch KL telemetry without changing single-start PPO batches."""
from ray.rllib.algorithms.ppo.torch.ppo_torch_learner import PPOTorchLearner
from baseline_MLP.start_condition_metrics import StartConditionMetricsPPOTorchLearner


class KLAuditPPOTorchLearner(StartConditionMetricsPPOTorchLearner):
    """Reuse the KL audit, omitting the three-start compaction connector."""

    def build(self):
        PPOTorchLearner.build(self)
