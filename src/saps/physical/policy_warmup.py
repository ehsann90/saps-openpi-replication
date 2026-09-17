"""One discarded inference and an advancing observation before an episode."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any, Callable

from saps.physical.live_shadow import (
    CameraPairGate, infer_live_request, prepare_live_request,
    validate_live_policy,
)
from saps.policies.openpi_droid import OpenPiDroidPolicy

PHYSICAL_WARMUP_POLICY_SEED = 20260917


class PolicyWarmup:
    """Owned once by the process lifecycle; failed attempts cannot be retried."""

    def __init__(self, *, warmup_policy_seed: int,
                 policy_episode_seed: int) -> None:
        for seed in (warmup_policy_seed, policy_episode_seed):
            if type(seed) is not int or not 0 <= seed <= 0x7fffffff:
                raise ValueError("Policy seeds must be integers in [0, 2**31-1]")
        if warmup_policy_seed == policy_episode_seed:
            raise ValueError("warmup_policy_seed must differ from policy_episode_seed")
        self.warmup_policy_seed = warmup_policy_seed
        self.policy_episode_seed = policy_episode_seed
        self._attempted = False
        self.record: dict[str, Any] = {
            "attempted": False, "completed": False, "request_count": 0,
            "policy_seed": warmup_policy_seed, "replan_index": 0,
            "audit_model_input": False, "actions_executed": 0,
            "policy_actions_executed": 0, "gripper_commands_issued": 0,
            "target_publications": 0, "response_discarded": False,
            "excluded_from_episode_timing": True,
        }

    def run(
        self, *, observation: Any, sample_dir: Path,
        policy: OpenPiDroidPolicy, ros_now: Callable[[], float],
        config: dict[str, Any],
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        """Validate and archive evidence, returning no actions or observation."""
        if self._attempted:
            raise RuntimeError("Warm-up already attempted in this process lifecycle")
        self._attempted = True
        self.record["attempted"] = True
        timing: dict[str, Any] = {}
        try:
            # Recheck at submission as well as construction, including overrides.
            if self.warmup_policy_seed == self.policy_episode_seed:
                raise ValueError("warmup_policy_seed must differ from policy_episode_seed")
            validate_live_policy(policy, self.record)
            infer_live_request(
                observation=observation, sample_dir=sample_dir, policy=policy,
                index=0, policy_episode_seed=self.warmup_policy_seed,
                ros_now=ros_now, config=config, monotonic=monotonic,
                call_timing=timing, audit_model_input=False,
            )
            response = json.loads((sample_dir / "response.json").read_text())
            for key in (
                "request_started_monotonic_seconds",
                "response_completed_monotonic_seconds",
                "response_completed_ros_seconds",
                "client_round_trip_seconds", "server_timing", "policy_timing",
                "observation_age_at_request_seconds",
                "observation_age_at_response_seconds",
            ):
                self.record[key] = response[key]
            self.record.update(
                completed=True, response_discarded=True,
                actions_returned_shape=response["action"]["shape"],
            )
        finally:
            self.record.update(timing)
            self.record["request_count"] = int(
                "request_start_monotonic_ns" in timing
            )


def prepare_warmed_observation(
    *, warmup: PolicyWarmup, collector: Any, policy: OpenPiDroidPolicy,
    output_dir: Path, observation_timeout: float,
    spin_once: Callable[[], None], ros_now: Callable[[], float],
    config: dict[str, Any], record: dict[str, Any],
    monotonic: Callable[[], float] = time.monotonic,
) -> tuple[Any, Path]:
    """Return only a new, fresh main-seed/index-zero observation; never infer it.

    The caller must own one PolicyWarmup for its entire process invocation.
    Warm-up evidence lives outside episode request directories and counters.
    """
    if warmup.record["attempted"]:
        raise RuntimeError("Warm-up already attempted in this process lifecycle")
    record["warmup"] = warmup.record
    gate = CameraPairGate()
    warmup_dir = output_dir / "warmup"
    warmup_dir.mkdir(exist_ok=False)
    arguments = dict(
        collector=collector, index=0, observation_timeout=observation_timeout,
        spin_once=spin_once, config=config, gate=gate, monotonic=monotonic,
    )
    observation, sample_dir = prepare_live_request(
        **arguments, output_dir=warmup_dir,
        policy_episode_seed=warmup.warmup_policy_seed, record=warmup.record,
    )
    warmup.run(
        observation=observation, sample_dir=sample_dir, policy=policy,
        ros_now=ros_now, config=config, monotonic=monotonic,
    )
    del observation
    barrier = warmup.record["response_completed_ros_seconds"]
    record["post_warmup_source_barrier_ros_seconds"] = barrier
    # The SAME gate demands both camera stamps advance. assemble() also checks
    # robot/gripper age, receive age, skew, and all existing freshness limits.
    observation, sample_dir = prepare_live_request(
        **arguments, output_dir=output_dir,
        policy_episode_seed=warmup.policy_episode_seed, record=record,
        source_barrier_ros_seconds=barrier,
    )
    record["post_warmup_observation_acquired"] = True
    record["first_real_replan_index"] = 0
    return observation, sample_dir
