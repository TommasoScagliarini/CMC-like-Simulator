"""Opt-in child-process hook for the V12R8 morphology strict-delay runtime."""

from __future__ import annotations

import os


_MARKER = "H0_V12R8_MORPH_CAUSAL_RUNTIME"
_RUNTIME_ID = "h0_v12r8_morph_strict_terminal_delay_v1"

if os.environ.get(_MARKER) == _RUNTIME_ID:
    import h0_v12r8_morphology_causal_runtime as _causal_runtime

    _causal_runtime.install()
