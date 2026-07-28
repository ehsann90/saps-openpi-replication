#!/usr/bin/env python3
"""
Watch the progress of autonomous OpenPI on the SAPS LIBERO cream-cheese task.

Usage:
    watch -n 10 \
        python3 tools/monitoring/watch_autonomous_progress.py \
        outputs/autonomous_n20_state0/sweep_summary.json
"""

import json
import sys
from pathlib import Path

summary_path = Path(
    sys.argv[1]
    if len(sys.argv) > 1
    else "outputs/autonomous_n20_state0/sweep_summary.json"
)

if not summary_path.exists():
    print("Waiting for the first completed episode...")
    raise SystemExit(0)

try:
    data = json.loads(summary_path.read_text(encoding="utf-8"))
except json.JSONDecodeError:
    # The sweep may be rewriting the file at this exact moment.
    print("Summary is currently being updated; retrying...")
    raise SystemExit(0)

completed = data["completed_episodes"]
expected = data["expected_episodes"]

print(f"Progress: {completed} / {expected}")
print(f"Complete: {data['complete']}")
print()

print(
    f"{'condition':<10}"
    f"{'done':>8}"
    f"{'success':>10}"
    f"{'timeout':>10}"
    f"{'rate':>10}"
)

for condition in data["conditions"]:
    rate = condition["success_rate"]
    rate_text = f"{rate:.2f}" if rate is not None else "—"

    print(
        f"{condition['condition_id']:<10}"
        f"{condition['completed_episodes']:>4}"
        f"/{condition['expected_episodes']:<3}"
        f"{condition['successes']:>10}"
        f"{condition['timeouts']:>10}"
        f"{rate_text:>10}"
    )
