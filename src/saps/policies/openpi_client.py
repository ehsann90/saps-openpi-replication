"""OpenPI client adapter for LIBERO observations."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from openpi_client import image_tools
from openpi_client.websocket_client_policy import WebsocketClientPolicy


class OpenPiLiberoPolicy:
    """Prepare LIBERO observations and query an OpenPI policy server."""

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        port: int = 8000,
        resize_size: int = 224,
    ) -> None:
        self._client = WebsocketClientPolicy(host, port)
        self._resize_size = resize_size

    def prepare_observation(
        self,
        obs: dict[str, Any],
        prompt: str,
    ) -> tuple[dict[str, Any], np.ndarray]:
        """Convert a raw LIBERO observation to OpenPI's input format."""

        # LIBERO/OpenPI training preprocessing uses a 180-degree rotation.
        agent_image = np.ascontiguousarray(
            obs["agentview_image"][::-1, ::-1]
        )
        wrist_image = np.ascontiguousarray(
            obs["robot0_eye_in_hand_image"][::-1, ::-1]
        )

        agent_image = image_tools.convert_to_uint8(
            image_tools.resize_with_pad(
                agent_image,
                self._resize_size,
                self._resize_size,
            )
        )
        wrist_image = image_tools.convert_to_uint8(
            image_tools.resize_with_pad(
                wrist_image,
                self._resize_size,
                self._resize_size,
            )
        )

        robot_state = np.concatenate(
            (
                np.asarray(obs["robot0_eef_pos"], dtype=np.float32),
                quaternion_to_axis_angle(
                    np.asarray(obs["robot0_eef_quat"], dtype=np.float32)
                ),
                np.asarray(obs["robot0_gripper_qpos"], dtype=np.float32),
            )
        )

        policy_input = {
            "observation/image": agent_image,
            "observation/wrist_image": wrist_image,
            "observation/state": robot_state,
            "prompt": prompt,
        }

        return policy_input, agent_image

    def infer(self, policy_input: dict[str, Any]) -> np.ndarray:
        """Request one action chunk from the policy server."""

        result = self._client.infer(policy_input)

        if "actions" not in result:
            raise KeyError("OpenPI response does not contain an 'actions' key.")

        actions = np.asarray(result["actions"], dtype=np.float32)

        if actions.ndim != 2 or actions.shape[1] != 7:
            raise ValueError(
                "Expected OpenPI actions with shape [horizon, 7], "
                f"but received {actions.shape}."
            )

        return actions


def quaternion_to_axis_angle(quaternion: np.ndarray) -> np.ndarray:
    """Convert a LIBERO quaternion to a three-dimensional axis-angle vector."""

    quat = np.asarray(quaternion, dtype=np.float32).copy()

    if quat.shape != (4,):
        raise ValueError(
            f"Expected a quaternion with shape (4,), received {quat.shape}."
        )

    quat[3] = np.clip(quat[3], -1.0, 1.0)
    denominator = math.sqrt(max(0.0, 1.0 - float(quat[3]) ** 2))

    if math.isclose(denominator, 0.0, abs_tol=1e-8):
        return np.zeros(3, dtype=np.float32)

    angle = 2.0 * math.acos(float(quat[3]))
    return np.asarray(quat[:3] * angle / denominator, dtype=np.float32)
