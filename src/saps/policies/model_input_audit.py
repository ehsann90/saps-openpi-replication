"""Opt-in inspection of the actual pinned OpenPI inference boundary."""

from __future__ import annotations

import hashlib
from typing import Any

import numpy as np


def array_evidence(value: Any) -> dict[str, Any]:
    """Describe exact array bytes; shape and dtype qualify the digest."""

    array = np.asarray(value)
    return {
        "shape": list(array.shape),
        "dtype": str(array.dtype),
        "sha256": hashlib.sha256(array.tobytes(order="C")).hexdigest(),
    }


def infer_with_model_audit(
    policy: Any, observation: dict[str, Any], *, noise: np.ndarray,
) -> dict[str, Any]:
    """Tap actual transforms and sampler once, restoring hooks on failure.

    The seeded websocket server calls inference synchronously, so these
    temporary instance hooks cannot overlap another request. No transforms,
    normalization, RNG operations, or sampling calls are replayed here.
    """

    transform = policy._input_transform
    sampler = policy._sample_actions
    audit: dict[str, Any] = {
        "schema_version": 1,
        "boundary": "actual OpenPI Policy._sample_actions observation",
        "prompt": observation["prompt"],
        "canonical_arrays": {
            key: array_evidence(value)
            for key, value in observation.items()
            if isinstance(value, np.ndarray)
        },
    }

    def inspect_transform(inputs: dict[str, Any]) -> dict[str, Any]:
        transformed = transform(inputs)
        audit["transformed_images"] = {
            name: np.array(value, copy=True)
            for name, value in transformed["image"].items()
        }
        return transformed

    def inspect_sampler(rng: Any, model_observation: Any,
                        **kwargs: Any) -> Any:
        audit["model_images"] = {
            name: np.array(value, copy=True)
            for name, value in model_observation.images.items()
        }
        audit["image_masks"] = {
            name: np.array(value, copy=True)
            for name, value in model_observation.image_masks.items()
        }
        for name in ("state", "tokenized_prompt", "tokenized_prompt_mask"):
            audit[name] = np.array(getattr(model_observation, name), copy=True)
        return sampler(rng, model_observation, **kwargs)

    policy._input_transform = inspect_transform
    policy._sample_actions = inspect_sampler
    try:
        result = policy.infer(observation, noise=noise)
    finally:
        policy._input_transform = transform
        policy._sample_actions = sampler
    if "model_images" not in audit:
        raise RuntimeError("OpenPI model audit hook was not reached.")
    result["saps_model_input_audit"] = audit
    return result
