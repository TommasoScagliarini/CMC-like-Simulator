"""Online GRF runtime, profiles, detectors, calibration and plugin sources.

Importing the package alone does not load OpenSim or numerical libraries.
"""

import sys

from workspace_bootstrap import ensure_simulator_paths

ensure_simulator_paths()

# Preserve parent namespaces needed by legacy detector pickle globals.
sys.modules.setdefault("online_grf_profiles", sys.modules[__name__])
