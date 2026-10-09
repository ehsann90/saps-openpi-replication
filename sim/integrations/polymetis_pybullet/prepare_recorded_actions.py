"""Validate recorded Isaac targets against the pinned Panda model."""

import argparse
import hashlib
import json
import math
from pathlib import Path


PANDA_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
PANDA_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
AUDIT_FILES = {
    "absolute_franka_hand": "franka_hand_with_drives.json",
    "absolute_robotiq": "robotiq_with_drives.json",
}
ISAAC_HOME = (0.0, -0.4, 0.0, -1.9, 0.0, 1.5, 0.0)


def valid_position(q: list, label: str) -> float:
    """Reject dimension, finite-value, or pinned Panda joint-limit errors."""
    if len(q) != 7 or not all(math.isfinite(float(x)) for x in q):
        raise ValueError(f"{label}: expected seven finite joint positions")
    margins = [
        min(float(x) - lo, hi - float(x))
        for x, lo, hi in zip(q, PANDA_LOWER, PANDA_UPPER)
    ]
    if min(margins) < 0:
        joint = margins.index(min(margins)) + 1
        raise ValueError(f"{label}: Panda joint {joint} exceeds its limit")
    return min(margins)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--matched-home", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)

    manifest = {
        "fairo_revision": "0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2",
        "panda_joint_lower_rad": PANDA_LOWER,
        "panda_joint_upper_rad": PANDA_UPPER,
        "action_hz": 15,
        "conditions": {},
    }
    if args.matched_home:
        valid_position(ISAAC_HOME, "matched HOME")
        manifest["required_home_q_rad"] = ISAAC_HOME
        manifest["initial_tolerance_rad"] = 0.001
        manifest["initial_velocity_tolerance_rad_s"] = 0.001
    shared_actions = None
    for condition, filename in AUDIT_FILES.items():
        path = args.audit_dir / filename
        audit = json.loads(path.read_text())
        actions = audit["actions"]
        if len(actions) != 8:
            raise ValueError(f"{filename}: expected eight actions")
        q0 = audit["initial"]["q_rad"]
        start_margin = valid_position(q0, f"{filename} initial")
        targets = [action["desired_target_rad"] for action in actions]
        margins = [
            valid_position(q, f"{filename} action {index}")
            for index, q in enumerate(targets)
        ]
        policy_actions = [action["policy_action"][:7] for action in actions]
        if any(len(action) != 7 or not all(math.isfinite(x) for x in action)
               for action in policy_actions):
            raise ValueError(f"{filename}: invalid policy action")
        if shared_actions is None:
            shared_actions = policy_actions
        elif policy_actions != shared_actions:
            raise ValueError("Isaac archives do not contain identical arm actions")
        manifest["conditions"][condition] = {
            "audit_file": filename,
            "audit_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "recorded_action_sha256": audit["source_actions_sha256"],
            "initial_q_rad": ISAAC_HOME if args.matched_home else q0,
            "absolute_targets_rad": targets,
            "minimum_initial_limit_margin_rad": start_margin,
            "minimum_target_limit_margin_rad": min(margins),
        }

    manifest["conditions"]["normalized_actions"] = {
        "initial_q_rad": manifest["conditions"]["absolute_franka_hand"][
            "initial_q_rad"
        ],
        "policy_arm_actions": shared_actions,
        "reference_audit": "absolute_franka_hand",
        "mapping": "q_des = measured Panda q + 0.2 * clip(action[:7], -1, 1)",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({
        key: value["minimum_target_limit_margin_rad"]
        for key, value in manifest["conditions"].items()
        if key.startswith("absolute_")
    }, indent=2))


if __name__ == "__main__":
    main()
