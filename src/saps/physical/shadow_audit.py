"""Validate and persist evidence from the actual OpenPI sampler boundary."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np

from saps.policies.model_input_audit import array_evidence


IMAGE_MAPPING = {
    "observation/exterior_image_1_left": "base_0_rgb",
    "observation/wrist_image_left": "left_wrist_0_rgb",
}
IMAGE_MASKS = {
    "base_0_rgb": True, "left_wrist_0_rgb": True,
    "right_wrist_0_rgb": False,
}


def save_model_audit(
    audit: dict[str, Any] | None, policy_input: dict[str, Any], path: Path,
) -> dict[str, Any]:
    """Require actual batched PI05 images, masks, state, and request identity."""

    if audit is None or audit.get("schema_version") != 1:
        raise ValueError("First P0 request requires model-input audit schema 1.")
    if audit["prompt"] != policy_input["prompt"]:
        raise ValueError("Audited prompt differs from request.")
    expected = {
        key: array_evidence(value) for key, value in policy_input.items()
        if isinstance(value, np.ndarray)
    }
    if audit["canonical_arrays"] != expected:
        raise ValueError("Audited canonical arrays differ from request.")
    arrays = {}
    for group in ("transformed_images", "model_images", "image_masks"):
        if set(audit[group]) != set(IMAGE_MASKS):
            raise ValueError(f"Unexpected OpenPI {group} keys.")
    for name, mask in IMAGE_MASKS.items():
        transformed = np.asarray(audit["transformed_images"][name])
        model = np.asarray(audit["model_images"][name])
        actual_mask = np.asarray(audit["image_masks"][name])
        if transformed.shape != (224, 224, 3) or transformed.dtype != np.uint8:
            raise ValueError("OpenPI transformed images must be 224x224 uint8.")
        if model.shape != (1, 224, 224, 3) or model.dtype != np.float32:
            raise ValueError("OpenPI sampler images must be batched float32.")
        if not np.isfinite(model).all() or model.min() < -1 or model.max() > 1:
            raise ValueError("OpenPI sampler image values must be in [-1, 1].")
        if (actual_mask.shape != (1,) or actual_mask.dtype != np.bool_
                or bool(actual_mask[0]) != mask):
            raise ValueError("OpenPI image mask differs from pinned PI05.")
        if not mask and (np.any(transformed) or np.any(model != -1)):
            raise ValueError("Right-wrist placeholder differs from pinned PI05.")
        arrays[f"transformed/{name}"] = transformed
        arrays[f"model/{name}"] = model
        arrays[f"mask/{name}"] = actual_mask
    for name, shape, dtype in (
        ("state", (1, 32), np.float32),
        ("tokenized_prompt", (1, 200), np.int32),
        ("tokenized_prompt_mask", (1, 200), np.bool_),
    ):
        value = np.asarray(audit[name])
        if value.shape != shape or value.dtype != dtype:
            raise ValueError(f"Unexpected OpenPI model {name} contract.")
        if not np.isfinite(value).all():
            raise ValueError(f"Nonfinite OpenPI {name}.")
        arrays[name] = value
    path.mkdir(exist_ok=False)
    np.savez_compressed(path / "model_input.npz", **arrays)
    for name in IMAGE_MASKS:
        image = arrays[f"transformed/{name}"]
        if not cv2.imwrite(str(path / f"{name}.png"), image[:, :, ::-1]):
            raise OSError(f"Could not write transformed image {name}.")
    return {
        "boundary": audit["boundary"], "prompt": audit["prompt"],
        "image_mapping": IMAGE_MAPPING, "image_masks": IMAGE_MASKS,
        "canonical_arrays": expected,
        "arrays": {key: array_evidence(value) for key, value in arrays.items()},
        "bundle": str(path.name + "/model_input.npz"),
        "visual_images": [f"{path.name}/{name}.png" for name in IMAGE_MASKS],
        "token_count": int(np.count_nonzero(arrays["tokenized_prompt_mask"])),
    }
