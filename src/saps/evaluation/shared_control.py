"""Per-step control decisions for SAPS shared autonomy."""

from __future__ import annotations

import dataclasses
from typing import Any

import numpy as np

from saps.arbitration import ActionArbitrator
from saps.arbitration import ArbitrationMode
from saps.arbitration import ArbitrationResult
from saps.arbitration import SAPS_ACTIVITY_THRESHOLD
from saps.policies.action_source import (
    ChunkedPolicyActionSource,
)
from saps.policies.action_source import PolicyActionSample


@dataclasses.dataclass(frozen=True)
class SharedControlDecision:
    """One policy-plus-human shared-control decision."""

    replay_image: np.ndarray
    policy_sample: PolicyActionSample
    arbitration_result: ArbitrationResult

    @property
    def executed_action(self) -> np.ndarray:
        """Return the action that should be sent to LIBERO."""

        return self.arbitration_result.executed_action

    def as_log_dict(self) -> dict[str, Any]:
        """Combine arbitration and policy fields for step logging."""

        return {
            **self.arbitration_result.as_dict(),
            **self.policy_sample.as_log_dict(),
        }


class SharedAutonomyController:
    """Produce one shared-control decision per control step."""

    def __init__(
        self,
        *,
        policy: Any,
        arbitration_mode: ArbitrationMode | str,
        replan_steps: int,
        policy_episode_seed: int | None,
        activity_threshold: float = SAPS_ACTIVITY_THRESHOLD,
    ) -> None:
        self._policy = policy

        self._policy_actions = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=replan_steps,
            policy_episode_seed=policy_episode_seed,
        )

        self._arbitrator = ActionArbitrator(
            mode=arbitration_mode,
            activity_threshold=activity_threshold,
        )

    @property
    def arbitration_mode(self) -> ArbitrationMode:
        """Return the configured arbitration mode."""

        return self._arbitrator.mode

    @property
    def policy_replan_count(self) -> int:
        """Return the number of requested policy chunks."""

        return self._policy_actions.replan_count

    @property
    def sampling_protocol_version(self) -> int | None:
        """Return the policy sampling protocol used so far."""

        return (
            self._policy_actions.sampling_protocol_version
        )

    def decide(
        self,
        *,
        observation: dict[str, Any],
        task_description: str,
        human_action: np.ndarray,
    ) -> SharedControlDecision:
        """Produce the next autonomous or takeover decision."""

        policy_input, replay_image = (
            self._policy.prepare_observation(
                observation,
                task_description,
            )
        )

        policy_sample = (
            self._policy_actions.next_action(
                policy_input
            )
        )

        arbitration_result = (
            self._arbitrator.arbitrate(
                autonomous_action=policy_sample.action,
                human_action=human_action,
            )
        )

        replay = np.asarray(
            replay_image,
            dtype=np.uint8,
        ).copy()
        replay.setflags(write=False)

        return SharedControlDecision(
            replay_image=replay,
            policy_sample=policy_sample,
            arbitration_result=arbitration_result,
        )
