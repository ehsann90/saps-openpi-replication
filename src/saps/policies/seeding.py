"""Stable policy seeds for matched SAPS experiments."""

from __future__ import annotations

import hashlib


SEED_PROTOCOL = "saps-policy-seed-v1"


def make_policy_episode_seed(
    *,
    base_seed: int,
    condition_id: str,
    trial_index: int,
    task_id: int,
    initial_state_index: int,
) -> int:
    """Create a stable seed independent of Python's randomized hash function.

    Arbitration mode is deliberately excluded so that matched autonomous,
    takeover, fixed-blending, and cosine trials receive the same policy noise.
    """

    if base_seed < 0:
        raise ValueError("base_seed must be non-negative.")

    if trial_index < 0:
        raise ValueError("trial_index must be non-negative.")

    payload = "|".join(
        (
            SEED_PROTOCOL,
            str(base_seed),
            str(task_id),
            str(initial_state_index),
            condition_id,
            str(trial_index),
        )
    ).encode("utf-8")

    digest = hashlib.sha256(payload).digest()

    # Keep the value inside a signed 31-bit range for portable JAX handling.
    return int.from_bytes(digest[:4], "little") & 0x7FFFFFFF
