"""Read-only Isaac composition check for HG1 historical and candidate USDs.

SimulationApp supplies Isaac's remote asset resolver. No World is created and
no simulation step is issued.
"""

from __future__ import annotations

import runpy
from pathlib import Path

from isaacsim import SimulationApp


app = SimulationApp({"headless": True})
try:
    check = (Path(__file__).resolve().parents[2] /
             "tools/hg1_validate_candidate_mount.py")
    runpy.run_path(str(check), run_name="__main__")
finally:
    app.close()
