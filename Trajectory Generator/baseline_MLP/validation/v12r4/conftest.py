from __future__ import annotations

import sys
from pathlib import Path


REVISION_ROOT = Path(__file__).resolve().parent
if str(REVISION_ROOT) not in sys.path:
    sys.path.insert(0, str(REVISION_ROOT))
