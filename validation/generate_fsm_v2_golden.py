"""Record the v2 actor-FSM payload sequence for the shared golden script."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fsm_v3_scenarios import GOLDEN_SCRIPT, make_fsm, run_script  # noqa: E402

OUT = Path(__file__).resolve().parent / "fixtures" / "fsm_v2_golden_scripted_sequence.json"
payloads = run_script(make_fsm(), GOLDEN_SCRIPT)
OUT.write_text(json.dumps(payloads, indent=1, sort_keys=True, allow_nan=False))
print("golden:", OUT, "| payload:", len(payloads))
