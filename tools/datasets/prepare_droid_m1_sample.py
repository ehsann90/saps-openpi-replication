#!/usr/bin/env python3
"""Download and extract a tiny, hash-verified genuine DROID sample."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import tempfile
from typing import Any
from urllib import parse
from urllib import request

import cv2
import h5py
import numpy as np


BUNDLE_SCHEMA_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config-path",
        default="configs/droid_m1_sample.json",
    )
    parser.add_argument(
        "--output-dir",
        default="data/droid_m1",
    )
    parser.add_argument(
        "--skip-download",
        action="store_true",
        help="Require already downloaded, hash-valid raw files.",
    )
    parser.add_argument(
        "--skip-annotation-verification",
        action="store_true",
        help="Do not stream the official annotation index for comparison.",
    )
    parser.add_argument(
        "--force-rebuild",
        action="store_true",
        help="Replace only the generated bundle, never the raw source files.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config_path)
    output_dir = Path(args.output_dir)
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    _validate_config(config)

    episode = config["episode"]
    episode_dir = output_dir / "raw" / episode["uuid"]
    episode_dir.mkdir(parents=True, exist_ok=True)

    source_paths = _obtain_source_files(
        config,
        episode_dir=episode_dir,
        allow_download=not args.skip_download,
    )

    if not args.skip_annotation_verification:
        _verify_language_annotation(config)

    bundle_path = output_dir / "droid_m1_samples.npz"
    metadata_path = output_dir / "droid_m1_samples.json"
    source_manifest_sha256 = hashlib.sha256(config_bytes).hexdigest()

    if bundle_path.exists() or metadata_path.exists():
        if not args.force_rebuild:
            _validate_existing_bundle(
                bundle_path,
                metadata_path,
                source_manifest_sha256=source_manifest_sha256,
            )
            print(f"Validated existing bundle: {bundle_path}")
            return

    arrays, bundle_metadata = _extract_bundle(
        config,
        source_paths=source_paths,
        source_manifest_sha256=source_manifest_sha256,
    )
    _write_bundle_atomic(bundle_path, arrays)
    bundle_metadata["bundle_sha256"] = _sha256(bundle_path)
    _write_json_atomic(metadata_path, bundle_metadata)

    total_bytes = sum(
        int(file_record["size_bytes"])
        for file_record in config["files"]
    )
    print(
        f"Prepared {len(config['bundle']['step_indices'])} samples from "
        f"{episode['uuid']} using {total_bytes} raw bytes."
    )
    print(f"Bundle: {bundle_path}")
    print(f"Metadata: {metadata_path}")


def _validate_config(config: dict[str, Any]) -> None:
    if config.get("schema_version") != 1:
        raise ValueError("Unsupported DROID sample config schema.")

    required_roles = {
        "metadata",
        "trajectory",
        "wrist_video",
        "exterior_video",
    }
    roles = {record.get("role") for record in config.get("files", [])}
    if roles != required_roles:
        raise ValueError(
            "DROID sample config must define exactly these file roles: "
            f"{sorted(required_roles)}."
        )

    steps = config["bundle"]["step_indices"]
    if (
        not isinstance(steps, list)
        or not steps
        or any(not isinstance(step, int) or step < 0 for step in steps)
        or len(set(steps)) != len(steps)
    ):
        raise ValueError(
            "bundle.step_indices must be unique non-negative integers."
        )

    horizon = config["bundle"]["recorded_action_horizon"]
    if not isinstance(horizon, int) or horizon <= 0:
        raise ValueError(
            "bundle.recorded_action_horizon must be positive."
        )


def _obtain_source_files(
    config: dict[str, Any],
    *,
    episode_dir: Path,
    allow_download: bool,
) -> dict[str, Path]:
    source_paths = {}
    bucket = config["dataset"]["bucket"]

    for file_record in config["files"]:
        destination = episode_dir / file_record["relative_path"]
        if destination.exists():
            _verify_file(destination, file_record)
        elif allow_download:
            _download_file(
                bucket=bucket,
                object_name=file_record["object"],
                destination=destination,
            )
            _verify_file(destination, file_record)
        else:
            raise FileNotFoundError(
                f"Required DROID source file is missing: {destination}"
            )
        source_paths[file_record["role"]] = destination

    return source_paths


def _download_file(
    *,
    bucket: str,
    object_name: str,
    destination: Path,
) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded_object = parse.quote(object_name, safe="")
    url = (
        "https://storage.googleapis.com/download/storage/v1/b/"
        f"{bucket}/o/{encoded_object}?alt=media"
    )

    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            prefix=f".{destination.name}.",
            suffix=".download",
            dir=destination.parent,
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            with request.urlopen(url, timeout=120) as response:
                while chunk := response.read(1024 * 1024):
                    temporary.write(chunk)
        temporary_path.replace(destination)
    except Exception:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
        raise


def _verify_file(path: Path, record: dict[str, Any]) -> None:
    actual_size = path.stat().st_size
    expected_size = int(record["size_bytes"])
    if actual_size != expected_size:
        raise ValueError(
            f"DROID source size mismatch at {path}: expected "
            f"{expected_size}, received {actual_size}. Preserve the file "
            "and investigate rather than overwriting it."
        )

    digest = hashlib.md5()  # noqa: S324 - official GCS integrity field.
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    actual_md5 = base64.b64encode(digest.digest()).decode("ascii")
    if actual_md5 != record["md5_base64"]:
        raise ValueError(
            f"DROID source MD5 mismatch at {path}: expected "
            f"{record['md5_base64']}, received {actual_md5}."
        )


def _verify_language_annotation(config: dict[str, Any]) -> None:
    annotation = config["episode"]["language_annotation"]
    encoded_object = parse.quote(annotation["object"], safe="")
    bucket = config["dataset"]["bucket"]
    url = (
        "https://storage.googleapis.com/download/storage/v1/b/"
        f"{bucket}/o/{encoded_object}?alt=media"
    )
    with request.urlopen(url, timeout=120) as response:
        annotations = json.load(response)

    actual = annotations[annotation["key"]][annotation["field"]]
    if actual != annotation["value"]:
        raise ValueError(
            "Official DROID language annotation differs from the pinned "
            f"sample config: expected {annotation['value']!r}, received "
            f"{actual!r}."
        )


def _extract_bundle(
    config: dict[str, Any],
    *,
    source_paths: dict[str, Path],
    source_manifest_sha256: str,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    episode = config["episode"]
    bundle_config = config["bundle"]
    steps = np.asarray(bundle_config["step_indices"], dtype=np.int64)
    recorded_horizon = int(bundle_config["recorded_action_horizon"])

    raw_metadata = json.loads(source_paths["metadata"].read_text())
    _validate_episode_metadata(raw_metadata, episode)

    with h5py.File(source_paths["trajectory"], "r") as trajectory:
        joint_positions = np.asarray(
            trajectory["observation/robot_state/joint_positions"],
            dtype=np.float32,
        )
        gripper_positions = np.asarray(
            trajectory["observation/robot_state/gripper_position"],
            dtype=np.float32,
        )
        joint_velocities = np.asarray(
            trajectory["action/joint_velocity"],
            dtype=np.float32,
        )
        action_gripper_positions = np.asarray(
            trajectory["action/gripper_position"],
            dtype=np.float32,
        )

    trajectory_length = len(joint_positions)
    if trajectory_length != int(episode["trajectory_length"]):
        raise ValueError(
            "DROID trajectory length differs from the sample config: "
            f"expected {episode['trajectory_length']}, received "
            f"{trajectory_length}."
        )
    if int(np.max(steps)) >= trajectory_length:
        raise ValueError("A selected step is outside the DROID trajectory.")

    exterior_images = _read_selected_video_frames(
        source_paths["exterior_video"],
        step_indices=steps,
        width=int(bundle_config["image_width"]),
        height=int(bundle_config["image_height"]),
    )
    wrist_images = _read_selected_video_frames(
        source_paths["wrist_video"],
        step_indices=steps,
        width=int(bundle_config["image_width"]),
        height=int(bundle_config["image_height"]),
    )

    recorded_actions_full = np.concatenate(
        (
            joint_velocities,
            action_gripper_positions[:, np.newaxis],
        ),
        axis=1,
    )
    action_indices = steps[:, np.newaxis] + np.arange(
        recorded_horizon,
        dtype=np.int64,
    )[np.newaxis, :]
    action_indices = np.minimum(action_indices, trajectory_length - 1)
    recorded_action_chunks = recorded_actions_full[action_indices]

    arrays = {
        "step_indices": steps,
        "exterior_images": exterior_images,
        "wrist_images": wrist_images,
        "joint_positions": joint_positions[steps],
        "gripper_positions": gripper_positions[steps, np.newaxis],
        "recorded_actions": recorded_action_chunks,
    }

    source_records = []
    for record in config["files"]:
        source_records.append(
            {
                **record,
                "sha256": _sha256(source_paths[record["role"]]),
            }
        )

    prompt = episode["language_annotation"]["value"]
    sample_records = [
        {
            "identity": f"{episode['uuid']}:step:{int(step)}",
            "episode_uuid": episode["uuid"],
            "step_index": int(step),
            "language_instruction": prompt,
        }
        for step in steps
    ]
    metadata = {
        "bundle_schema_version": BUNDLE_SCHEMA_VERSION,
        "source_manifest_path": "configs/droid_m1_sample.json",
        "source_manifest_sha256": source_manifest_sha256,
        "dataset": config["dataset"],
        "episode": episode,
        "source_files": source_records,
        "total_source_bytes": sum(
            int(record["size_bytes"])
            for record in config["files"]
        ),
        "samples": sample_records,
        "array_contract": {
            name: {
                "shape": list(value.shape),
                "dtype": str(value.dtype),
            }
            for name, value in arrays.items()
        },
        "recorded_action_contract": {
            "dimensions_0_through_6": (
                "DROID normalized relative joint-velocity command"
            ),
            "dimension_7": "DROID gripper position in [0, 1]",
            "source_control_frequency_hz": 15,
            "comparison_purpose": (
                "scale and representation inspection only; not an "
                "accuracy metric"
            ),
        },
    }
    return arrays, metadata


def _validate_episode_metadata(
    raw_metadata: dict[str, Any],
    episode: dict[str, Any],
) -> None:
    expected = {
        "uuid": episode["uuid"],
        "trajectory_length": episode["trajectory_length"],
        "wrist_cam_serial": episode["wrist_camera_serial"],
        "ext1_cam_serial": episode["exterior_camera_serial"],
        "success": True,
    }
    drift = {
        key: {"expected": value, "received": raw_metadata.get(key)}
        for key, value in expected.items()
        if raw_metadata.get(key) != value
    }
    if drift:
        raise ValueError(f"DROID episode metadata drifted: {drift}")


def _read_selected_video_frames(
    path: Path,
    *,
    step_indices: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open DROID MP4: {path}")

    selected = {int(step) for step in step_indices}
    frames = {}
    try:
        for index in range(int(np.max(step_indices)) + 1):
            success, frame = capture.read()
            if not success:
                raise RuntimeError(
                    f"DROID MP4 ended before selected step {index}: {path}"
                )
            if index in selected:
                resized = cv2.resize(frame, (width, height))
                frames[index] = np.ascontiguousarray(resized[..., ::-1])
    finally:
        capture.release()

    return np.stack([frames[int(step)] for step in step_indices])


def _validate_existing_bundle(
    bundle_path: Path,
    metadata_path: Path,
    *,
    source_manifest_sha256: str,
) -> None:
    if not bundle_path.is_file() or not metadata_path.is_file():
        raise FileExistsError(
            "Only one generated DROID bundle file exists. Preserve it and "
            "investigate, or use --force-rebuild after review."
        )

    metadata = json.loads(metadata_path.read_text())
    expected = {
        "bundle_schema_version": BUNDLE_SCHEMA_VERSION,
        "source_manifest_sha256": source_manifest_sha256,
        "bundle_sha256": _sha256(bundle_path),
    }
    drift = {
        key: {"expected": value, "received": metadata.get(key)}
        for key, value in expected.items()
        if metadata.get(key) != value
    }
    if drift:
        raise ValueError(
            "Existing DROID sample bundle provenance differs. Preserve the "
            f"bundle and investigate, or use --force-rebuild: {drift}"
        )


def _write_bundle_atomic(
    path: Path,
    arrays: dict[str, np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as file:
        np.savez_compressed(file, **arrays)
    temporary.replace(path)


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)
        file.write("\n")
    temporary.replace(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


if __name__ == "__main__":
    main()
