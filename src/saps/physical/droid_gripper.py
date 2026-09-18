"""DROID binary gripper intent and policy-owned Franka Hand evidence."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Callable


@dataclass(frozen=True)
class DroidGripperDecision:
    raw_policy_value: float
    binary_closure: float


def droid_gripper_decision(value: float) -> DroidGripperDecision:
    """Reference threshold is strictly > .5; zero means open, one means closed."""
    if not math.isfinite(value):
        raise ValueError('Non-finite DROID gripper action')
    return DroidGripperDecision(
        raw_policy_value=float(value),
        binary_closure=float(value > .5),
    )


class DroidGripper:
    """Map DROID binary intent onto explicit Franka Move/Grasp primitives."""

    def __init__(
        self,
        hand: Any,
        *,
        maximum_width_m: float,
        measured: Callable[[], dict[str, Any]],
        grasp_force_n: float | None = None,
        grasp_epsilon_inner_m: float | None = None,
        grasp_epsilon_outer_m: float | None = None,
    ) -> None:
        if not math.isfinite(maximum_width_m) or maximum_width_m <= 0:
            raise ValueError('Maximum width must be positive and finite')

        if grasp_force_n is not None:
            if not math.isfinite(grasp_force_n) or grasp_force_n <= 0:
                raise ValueError('Grasp force must be positive and finite')

        for name, value in (
            ('grasp_epsilon_inner_m', grasp_epsilon_inner_m),
            ('grasp_epsilon_outer_m', grasp_epsilon_outer_m),
        ):
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError(f'{name} must be finite and non-negative')

        self.hand = hand
        self.maximum_width_m = float(maximum_width_m)
        self.measured = measured
        self.grasp_force_n = grasp_force_n
        self.grasp_epsilon_inner_m = grasp_epsilon_inner_m
        self.grasp_epsilon_outer_m = grasp_epsilon_outer_m

    def check(self) -> None:
        self.hand.check()

    def check_inference(self) -> None:
        self.check()
        if any(
            request['disposition'] == 'queued'
            for request in self.hand.evidence()['requests']
        ):
            raise RuntimeError('gripper_replacement_pending_before_inference')

    def command(self, value: float) -> dict[str, Any]:
        decision = droid_gripper_decision(value)
        before = self.measured()

        if decision.binary_closure == 0.0:
            request = self.hand.request_move(self.maximum_width_m)
        else:
            if (
                self.grasp_force_n is None
                or self.grasp_epsilon_inner_m is None
                or self.grasp_epsilon_outer_m is None
            ):
                raise RuntimeError('gripper_grasp_parameters_unconfigured')

            request = self.hand.request_grasp(
                0.0,
                force=self.grasp_force_n,
                epsilon_inner=self.grasp_epsilon_inner_m,
                epsilon_outer=self.grasp_epsilon_outer_m,
            )

        record = asdict(decision)
        record.update(measured_before=before)
        record.update(request)
        return record

    def stop(self) -> None:
        self.hand.stop()

    def evidence(self) -> dict[str, Any]:
        return self.hand.evidence()


def finalize_gripper_evidence(result: dict[str, Any], gripper: Any) -> None:
    """Join asynchronous goal events back to immutable per-step decisions."""
    evidence = gripper.evidence()
    result['gripper'] = evidence
    result['gripper_commands_issued'] = sum(
        request['command_issued'] for request in evidence['requests']
    )

    for row in result.get('rows', []):
        step = row.get('gripper')
        if step is None:
            continue

        request = evidence['requests'][step['request_id']]
        step.update(request)
        owner = request.get('duplicate_of', request['request_id'])
        step['lifecycle_events'] = [
            event
            for event in evidence['events']
            if event['request_id'] == owner
        ]