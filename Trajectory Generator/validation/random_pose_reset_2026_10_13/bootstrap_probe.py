"""Run the reviewed random-reset bootstrap checks without physical integration."""
import sys
from random_pose_reset import main

if __name__ == "__main__":
    sys.argv.insert(1, "--bootstrap-probe")
    raise SystemExit(main())
