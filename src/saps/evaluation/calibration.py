"""Reusable nominal-scene reset for disposable operator calibration."""

from __future__ import annotations

from typing import Any

import numpy as np


CALIBRATION_RESET_ACTION = np.asarray(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    dtype=np.float32,
)


def reset_nominal_calibration_scene(
    *,
    env: Any,
    initial_states: Any,
    initial_state_index: int,
    num_steps_wait: int,
) -> tuple[dict[str, Any], int]:
    """Restore and settle the selected nominal LIBERO initial state."""

    env.reset()
    observation = env.set_init_state(
        initial_states[initial_state_index]
    )
    simulation_steps = 0
    for _ in range(num_steps_wait):
        observation, reward, done, info = env.step(
            CALIBRATION_RESET_ACTION.tolist()
        )
        del reward, info
        simulation_steps += 1
        if done:
            raise RuntimeError(
                "Calibration scene terminated while settling."
            )
    return observation, simulation_steps
