"""ROS action boundary for one already-approved FR3 joint target.

This module does not interpret policy actions and does not compute or modify a
target. It sends exactly one supplied reference/target pair to the validated
``fr3_lab_stack`` action server.

ROS imports are intentionally lazy so the pure SAPS unit-test environment can
import this module without a ROS installation.
"""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np


ACTION_NAME = "/fr3_joint_target"
SERVER_PARAMETER_SERVICE = "/fr3_joint_target_server/get_parameters"


def _vector7(value: Any, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (7,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite seven-joint vector.")
    return vector


def split_ros_time(ros_seconds: float) -> tuple[int, int]:
    """Convert finite non-negative ROS seconds into builtin Time fields."""

    if not math.isfinite(ros_seconds) or ros_seconds < 0.0:
        raise ValueError("reference_ros_seconds must be finite and non-negative.")

    sec = int(math.floor(ros_seconds))
    nanosec = int(round((ros_seconds - sec) * 1_000_000_000))

    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0

    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("Invalid ROS time fields.")
    return sec, nanosec


def _validate_server_evidence(
    *,
    reference_q: Any,
    target_q: Any,
    evidence: dict[str, Any],
    expect_execute: bool,
) -> None:
    """Check boundary invariants without reinterpreting MoveIt success/failure."""

    sent_reference = _vector7(reference_q, "reference_q")
    sent_target = _vector7(target_q, "target_q")

    echoed_reference = _vector7(
        evidence.get("reference_q"),
        "evidence.reference_q",
    )
    echoed_target = _vector7(
        evidence.get("requested_absolute_target"),
        "evidence.requested_absolute_target",
    )

    if not np.allclose(
        echoed_reference,
        sent_reference,
        rtol=0.0,
        atol=1e-12,
    ):
        raise RuntimeError("Server evidence changed the supplied reference_q.")

    if not np.allclose(
        echoed_target,
        sent_target,
        rtol=0.0,
        atol=1e-12,
    ):
        raise RuntimeError("Server evidence changed the supplied target_q.")

    attempts = evidence.get("execution_attempts")
    if not isinstance(attempts, int) or isinstance(attempts, bool):
        raise RuntimeError("Server evidence lacks an integer execution_attempts.")
    if attempts not in (0, 1):
        raise RuntimeError(
            f"Expected at most one execution attempt, received {attempts}."
        )

    attempted = evidence.get("execution_attempted")
    if not isinstance(attempted, bool):
        raise RuntimeError("Server evidence lacks boolean execution_attempted.")

    if not expect_execute and (attempted or attempts != 0):
        raise RuntimeError("Plan-only request unexpectedly attempted execution.")


class JointTargetActionClient:
    """Dedicated one-goal client for ``fr3_lab_stack``."""

    def __init__(
        self,
        *,
        action_name: str = ACTION_NAME,
        parameter_service: str = SERVER_PARAMETER_SERVICE,
    ) -> None:
        import rclpy
        from fr3_lab_stack_interfaces.action import ExecuteJointTarget
        from rcl_interfaces.srv import GetParameters
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor

        if not rclpy.ok():
            raise RuntimeError(
                "rclpy must already be initialized before creating "
                "JointTargetActionClient."
            )

        self._rclpy = rclpy
        self._action_type = ExecuteJointTarget
        self._get_parameters_type = GetParameters
        self.node = rclpy.create_node(
            "saps_physical_p1b_joint_target_client",
            use_global_arguments=False,
            enable_rosout=False,
            start_parameter_services=False,
        )
        self._executor = SingleThreadedExecutor(context=self.node.context)
        self._executor.add_node(self.node)
        self.action = ActionClient(
            self.node,
            ExecuteJointTarget,
            action_name,
        )
        self.parameters = self.node.create_client(
            GetParameters,
            parameter_service,
        )

    def close(self) -> None:
        """Destroy only resources owned by this client; never shut down rclpy."""

        action = getattr(self, "action", None)
        if action is not None:
            action.destroy()
            self.action = None

        parameters = getattr(self, "parameters", None)
        node = getattr(self, "node", None)
        if parameters is not None and node is not None:
            node.destroy_client(parameters)
            self.parameters = None

        executor = getattr(self, "_executor", None)
        if executor is not None and node is not None:
            executor.remove_node(node)

        if node is not None:
            node.destroy_node()
            self.node = None

        if executor is not None:
            executor.shutdown()
            self._executor = None

    def _wait_future(
        self,
        future: Any,
        *,
        timeout: float,
        label: str,
    ) -> Any:
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout must be finite and positive.")

        self._executor.spin_until_future_complete(
            future,
            timeout_sec=timeout,
        )
        if not future.done():
            raise TimeoutError(
                f"Timed out waiting for {label}; do not retry automatically "
                "because execution state may be unknown."
            )

        error = future.exception()
        if error is not None:
            raise RuntimeError(f"{label} failed: {error}")

        return future.result()

    def server_execute_enabled(self, *, timeout: float = 5.0) -> bool:
        """Read the server's execution mode before any goal is sent."""

        from rcl_interfaces.msg import ParameterType

        if not self.parameters.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Joint-target server parameter service is unavailable."
            )

        request = self._get_parameters_type.Request()
        request.names = ["execute"]
        response = self._wait_future(
            self.parameters.call_async(request),
            timeout=timeout,
            label="joint-target server execute parameter",
        )

        if len(response.values) != 1:
            raise RuntimeError("Unexpected execute-parameter response.")

        value = response.values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            raise RuntimeError(
                "Joint-target server execute parameter is not boolean."
            )

        return bool(value.bool_value)

    def send_once(
        self,
        *,
        request_id: str,
        reference_q: Any,
        target_q: Any,
        reference_ros_seconds: float,
        expect_execute: bool,
        timeout: float = 15.0,
    ) -> dict[str, Any]:
        """Send exactly one absolute joint target and return auditable evidence.

        No retry, hold command, target recomputation, action scaling, or second
        goal is issued by this method.
        """

        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("request_id must be non-empty.")

        reference = _vector7(reference_q, "reference_q")
        target = _vector7(target_q, "target_q")
        sec, nanosec = split_ros_time(reference_ros_seconds)

        if not isinstance(expect_execute, bool):
            raise TypeError("expect_execute must be bool.")

        if not self.action.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Joint-target action server is unavailable.")

        actual_execute = self.server_execute_enabled(
            timeout=min(timeout, 5.0),
        )
        if actual_execute != expect_execute:
            raise RuntimeError(
                "Joint-target server execution mode mismatch: "
                f"expected execute={expect_execute}, "
                f"server reports execute={actual_execute}."
            )

        goal = self._action_type.Goal()
        goal.request_id = request_id
        goal.reference_q = reference.tolist()
        goal.target_q = target.tolist()
        goal.reference_stamp.sec = sec
        goal.reference_stamp.nanosec = nanosec

        feedback_stages: list[str] = []

        def feedback_callback(message: Any) -> None:
            feedback_stages.append(str(message.feedback.stage))

        handle = self._wait_future(
            self.action.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            ),
            timeout=timeout,
            label="joint-target goal response",
        )
        if handle is None or not handle.accepted:
            raise RuntimeError("Joint-target goal was rejected.")

        wrapped = self._wait_future(
            handle.get_result_async(),
            timeout=timeout,
            label="joint-target action result",
        )
        if wrapped is None:
            raise RuntimeError("Joint-target action returned no result.")

        result = wrapped.result
        try:
            evidence = json.loads(result.evidence_json)
        except (TypeError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f"Joint-target server returned invalid evidence JSON: {error}"
            ) from error

        if not isinstance(evidence, dict):
            raise RuntimeError(
                "Joint-target evidence JSON must contain an object."
            )

        _validate_server_evidence(
            reference_q=reference,
            target_q=target,
            evidence=evidence,
            expect_execute=expect_execute,
        )

        return {
            "request_id": request_id,
            "action_status": int(wrapped.status),
            "success": bool(result.success),
            "outcome": str(result.outcome),
            "planning_error_code": int(result.planning_error_code),
            "execution_attempted": bool(result.execution_attempted),
            "execution_action_status": int(result.execution_action_status),
            "execution_error_code": int(result.execution_error_code),
            "final_q": [float(value) for value in result.final_q],
            "final_dq": [float(value) for value in result.final_dq],
            "feedback_stages": feedback_stages,
            "server_evidence": evidence,
        }
