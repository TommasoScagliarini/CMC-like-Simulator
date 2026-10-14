"""Process-local partial prescribed mask; no production environment changes."""
from pathlib import Path
import os
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))


def install(mask):
    from unified_MLP_39.env import UnifiedProsthesisEnv
    import numpy as np
    indices = {"clock_hidden": (0, 1), "target_hidden": (35, 36, 37, 38)}[mask]
    original = UnifiedProsthesisEnv._get_observation
    availability = UnifiedProsthesisEnv._availability_info

    def observed(self):
        obs, values = original(self)
        before = obs.copy()
        obs = obs.copy()
        obs[list(indices)] = 0
        retained = [i for i in range(len(obs)) if i not in indices]
        assert np.array_equal(obs[retained], before[retained])
        return obs, dict(zip(self.observation_feature_names, map(float, obs)))

    def info(self):
        return {**availability(self), "diagnostic_mask": mask,
                "clock_visible": mask != "clock_hidden",
                "target_q_qdot_visible": mask != "target_hidden"}

    UnifiedProsthesisEnv._get_observation = observed
    UnifiedProsthesisEnv._availability_info = info


if __name__ == "__main__":
    install(os.environ["CMC_DIAGNOSTIC_MASK"])
    from unified_MLP_39 import rollout_eval as driver
    driver._SCRIPT_PATH = Path(__file__).resolve()
    driver.main()
