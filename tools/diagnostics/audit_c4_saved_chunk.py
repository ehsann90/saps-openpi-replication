#!/usr/bin/env python3
"""Audit a saved C4 prediction and a supplied URDF without ROS or robot access."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from saps.physical.c4_target_audit import audit_targets, compare_arm_model
from saps.policies.openpi_droid import json_compatible


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--request-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.urdf, args.request_dir / 'request.json', args.request_dir / 'actions.npz']
    xml = args.urdf.read_text()
    request = json.loads(paths[1].read_text())
    with np.load(paths[2], allow_pickle=False) as bundle:
        actions = bundle['actions']
    report = audit_targets(xml, request, actions)
    report['model_comparison'] = compare_arm_model(xml, args.urdf, report)
    report['inputs'] = [dict(path=str(p.resolve()), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                        for p in paths]
    with args.output.open('x') as stream:
        json.dump(json_compatible(report), stream, indent=2, allow_nan=False)
        stream.write('\n')
    passed = (report['first_chunk_snapshot_checks_passed']
              and report['model_comparison']['flange_model_matches_runtime'])
    print(json.dumps(dict(snapshot_checks_passed=passed, physical_rollout_readiness='NO-GO',
                          output=str(args.output))))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
