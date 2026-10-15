"""Simulation-timestamped paired RGB frames and deterministic MP4 export."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import time
from fractions import Fraction
from pathlib import Path

import numpy as np
from PIL import Image

CAMERA_KEYS = {
    "exterior": "observation/exterior_image_1_left",
    "wrist": "observation/wrist_image_left",
}


def validate_timeline(frames: list[dict], period_s: float) -> None:
    """Require ordered, gap-free samples at the declared simulation cadence."""
    if not frames or not np.isfinite(period_s) or period_s <= 0:
        raise ValueError("Video needs frames and a positive period")
    for index, frame in enumerate(frames):
        if frame["index"] != index:
            raise ValueError("Video frame indices are not consecutive")
        if not np.isfinite(frame["simulation_time_seconds"]):
            raise ValueError("Video timestamp must be finite")
        if index and abs(
            frame["simulation_time_seconds"]
            - frames[index - 1]["simulation_time_seconds"] - period_s
        ) > 1e-4:
            raise ValueError("Video frame spacing differs from simulation time")


def write_metadata(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(value, file, indent=2, allow_nan=False)
        file.write("\n")
    os.replace(temporary, path)


def _frame_image(image: np.ndarray) -> Image.Image:
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Video frame must be H×W×3 uint8 RGB")
    return Image.fromarray(image, mode="RGB")


def _encode_stream(directory: Path, destination: Path, fps: Fraction) -> None:
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-framerate", str(fps), "-start_number", "0",
        "-i", str(directory / "%06d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(destination),
    ]
    subprocess.run(command, check=True, capture_output=True, text=True)


def _inspect_stream(path: Path, expected_frames: int) -> dict:
    result = subprocess.run([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=nb_frames,duration,width,height,r_frame_rate",
        "-of", "json", str(path),
    ], check=True, capture_output=True, text=True)
    stream = json.loads(result.stdout)["streams"][0]
    if int(stream["nb_frames"]) != expected_frames:
        raise RuntimeError(f"Encoded frame count differs for {path}")
    return stream


def export_video(
    frames: list[dict], output: Path, *, period_s: float,
    label: str, video_name: str, source_rollout_id: str,
) -> dict:
    """Encode exact paired samples as one side-by-side stream."""
    validate_timeline(frames, period_s)
    if video_name not in ("policy_observations", "simulation_motion"):
        raise ValueError("Unknown video type")
    output.mkdir(parents=True, exist_ok=True)
    destination = output / f"{video_name}.mp4"
    metadata_path = output / f"{video_name}.json"
    staging = output / f".{video_name}_frames"
    if destination.exists() or metadata_path.exists() or staging.exists():
        raise FileExistsError(destination)
    fps = Fraction(1 / period_s).limit_denominator(1000)
    first = frames[0]["simulation_time_seconds"]
    staging.mkdir()
    start = time.perf_counter()
    records = []
    try:
        for index, frame in enumerate(frames):
            stamp = frame["simulation_time_seconds"]
            images = []
            for role in ("exterior", "wrist"):
                with Image.open(frame[role]) as source:
                    pixels = np.asarray(source.convert("RGB"))
                evidence = frame.get("array_evidence", {}).get(CAMERA_KEYS[role])
                if evidence is not None and (
                    evidence["shape"] != list(pixels.shape)
                    or evidence["dtype"] != str(pixels.dtype)
                    or evidence["sha256"] != hashlib.sha256(
                        pixels.tobytes(order="C")
                    ).hexdigest()
                ):
                    raise ValueError(f"Policy image differs from request: {frame[role]}")
                images.append(_frame_image(pixels))
            left, right = images
            if left.size != right.size:
                raise ValueError("Synchronized views require equal dimensions")
            paired = Image.new("RGB", (left.width * 2, left.height))
            paired.paste(left, (0, 0))
            paired.paste(right, (left.width, 0))
            paired.save(staging / f"{index:06d}.png")
            records.append({"index": index, "simulation_time_seconds": stamp,
                            "simulation_step_index": frame.get("simulation_step_index"),
                            "replan_index": frame["replan_index"],
                            "action_index": frame.get("action_index"),
                            "request_image_evidence": frame.get("array_evidence")})
        _encode_stream(staging, destination, fps)
        stream = _inspect_stream(destination, len(records))
    except BaseException as error:
        write_metadata(metadata_path, {
            "status": "incomplete", "label": label,
            "video_type": video_name, "source_rollout_id": source_rollout_id,
            "encoding_error": f"{type(error).__name__}: {error}",
            "frames": records,
        })
        raise
    finally:
        shutil.rmtree(staging)
    metadata = {
        "status": "complete", "label": label,
        "video_type": video_name, "source_rollout_id": source_rollout_id,
        "frame_count": len(records),
        "initial_simulation_time_seconds": first,
        "final_simulation_time_seconds": frames[-1]["simulation_time_seconds"],
        "simulated_span_seconds": frames[-1]["simulation_time_seconds"] - first,
        "encoded_presentation_seconds": len(records) * period_s,
        "final_frame_presentation_seconds": period_s,
        "frame_period_seconds": period_s, "nominal_fps": str(fps),
        "encoding": "H.264/yuv420p via ffmpeg libx264",
        "intermediate_visual_frames_synthesized": 0,
        "missing_or_dropped_frames": [],
        "encoded_stream": stream,
        "camera_resolution_wh": list(left.size),
        "video_capture_wall_seconds": 0.0,
        "source_observation_capture_wall_seconds": sum(
            frame.get("observation_capture_wall_seconds", 0.0)
            for frame in frames
        ),
        "encoding_wall_seconds": time.perf_counter() - start,
        "frames": records,
    }
    write_metadata(metadata_path, metadata)
    return metadata


def archived_frames(run: Path) -> tuple[list[dict], float, list[dict]]:
    """Load complete saved request pairs and identify incomplete replans."""
    episode = json.loads((run / "episode.json").read_text())
    count = episode["completed_replans"]
    frames = []
    missing = []
    directories = sorted((run / "replans").glob("[0-9][0-9][0-9][0-9]"))
    for directory in directories:
        index = int(directory.name)
        if index >= count:
            observation_path = directory / "observation.json"
            observation = (json.loads(observation_path.read_text())
                           if observation_path.exists() else None)
            missing.append({
                "replan_index": index,
                "reason": ("incomplete replan; no full presentation interval"
                           if observation is not None else "missing observation"),
                "observation_simulation_time_seconds": (
                    observation["simulation_time_seconds"]
                    if observation is not None else None
                ),
                "policy_response_saved": (directory / "response.json").exists(),
            })
    for index in range(count):
        directory = run / "replans" / f"{index:04d}"
        if not (directory / "observation.json").exists():
            raise ValueError(f"Missing policy observation for replan {index}")
        observation = json.loads((directory / "observation.json").read_text())
        execution = json.loads((directory / "execution.json").read_text())
        if execution["status"] != "complete" or execution["actions_executed"] != 8:
            raise ValueError(f"Incomplete archived replan {index}")
        frames.append({
            "index": index, "replan_index": index,
            "simulation_time_seconds": observation["simulation_time_seconds"],
            "simulation_step_index": observation["simulation_step_index"],
            "array_evidence": observation["array_evidence"],
            "observation_capture_wall_seconds": observation.get(
                "capture_seconds", 0.0
            ),
            "exterior": directory / "exterior_raw.png",
            "wrist": directory / "wrist_raw.png",
        })
    period = 8 / 15
    if count * period > episode["simulation_control_seconds"] + 1e-4:
        raise ValueError("Archived complete replans exceed simulated duration")
    return frames, period, missing


class PolicyVideoRecorder:
    """Persist paired frames per completed action before post-run encoding."""

    def __init__(self, output: Path, *, source_rollout_id: str,
                 period_s: float = 1 / 15) -> None:
        output.mkdir(parents=True, exist_ok=True)
        self.output = output
        self.source_rollout_id = source_rollout_id
        self.period_s = period_s
        self.frames: list[dict] = []
        self.dropped_frames: list[dict] = []
        self.capture_wall_seconds = 0.0
        self.finalized = False
        self._persist("recording")

    def _persist(self, status: str) -> None:
        write_metadata(self.output / "simulation_motion_recording.json",
                       self._metadata(status))

    def _metadata(self, status: str) -> dict:
        return {
            "status": status, "video_type": "simulation_motion",
            "source_rollout_id": self.source_rollout_id,
            "frame_count": len(self.frames),
            "frame_period_seconds": self.period_s,
            "initial_simulation_time_seconds": (
                self.frames[0]["simulation_time_seconds"] if self.frames else None
            ),
            "final_simulation_time_seconds": (
                self.frames[-1]["simulation_time_seconds"] if self.frames else None
            ),
            "simulated_span_seconds": (
                self.frames[-1]["simulation_time_seconds"]
                - self.frames[0]["simulation_time_seconds"]
                if self.frames else 0.0
            ),
            "camera_resolution_wh": (
                self.frames[0]["camera_resolution_wh"] if self.frames else None
            ),
            "missing_or_dropped_frames": self.dropped_frames,
            "capture_wall_seconds": self.capture_wall_seconds,
            "frames": [{k: v for k, v in frame.items()
                        if k not in ("exterior", "wrist")}
                       for frame in self.frames],
        }

    def append(self, *, replan_index: int, action_index: int,
               simulation_time_seconds: float, simulation_step_index: int,
               exterior: np.ndarray, wrist: np.ndarray) -> None:
        if self.finalized:
            raise RuntimeError("Video recorder is finalized")
        index = len(self.frames)
        if index == 0 and action_index != -1:
            raise ValueError("First motion frame must be the initial state")
        if self.dropped_frames:
            raise RuntimeError("Cannot append after a dropped action frame")
        if self.frames and abs(simulation_time_seconds
                               - self.frames[-1]["simulation_time_seconds"]
                               - self.period_s) > 1e-4:
            raise ValueError("Action frame skipped or duplicated")
        left = _frame_image(exterior)
        right = _frame_image(wrist)
        if left.size != right.size:
            raise ValueError("Camera resolutions differ")
        paths = {}
        for role, image in (("exterior", left), ("wrist", right)):
            directory = self.output / "raw" / role
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"{index:06d}.png"
            image.save(path)
            paths[role] = path
        self.frames.append({
            "index": index, "replan_index": replan_index,
            "action_index": action_index,
            "simulation_time_seconds": simulation_time_seconds,
            "simulation_step_index": simulation_step_index,
            "camera_resolution_wh": list(left.size), **paths,
        })
        self._persist("recording")

    def note_drop(self, *, replan_index: int, action_index: int,
                  error: str) -> None:
        self.dropped_frames.append({"replan_index": replan_index,
                                    "action_index": action_index,
                                    "error": error})
        self._persist("recording")

    def finalize(self, *, termination_reason: str | None = None) -> dict:
        if self.finalized:
            raise RuntimeError("Video recorder already finalized")
        self.finalized = True
        status = "frames_only"
        error = None
        if self.frames:
            try:
                encoded = export_video(
                    self.frames, self.output, period_s=self.period_s,
                    label="action-boundary motion recording",
                    video_name="simulation_motion",
                    source_rollout_id=self.source_rollout_id,
                )
                status = (
                    "complete" if not self.dropped_frames
                    and termination_reason in (None, "max_replans")
                    else "partial"
                )
            except (OSError, ValueError, RuntimeError, KeyboardInterrupt,
                    subprocess.CalledProcessError) as exc:
                error = f"{type(exc).__name__}: {exc}"
        metadata = self._metadata(status)
        metadata["rollout_termination_reason"] = termination_reason
        metadata["encoding_error"] = error
        if status in ("complete", "partial"):
            encoded["status"] = status
            encoded["missing_or_dropped_frames"] = self.dropped_frames
            encoded["video_capture_wall_seconds"] = self.capture_wall_seconds
            encoded["rollout_termination_reason"] = termination_reason
            write_metadata(self.output / "simulation_motion.json", encoded)
            metadata["encoding"] = encoded["encoding"]
            metadata["nominal_fps"] = encoded["nominal_fps"]
            metadata["encoded_presentation_seconds"] = encoded[
                "encoded_presentation_seconds"
            ]
            metadata["encoding_wall_seconds"] = encoded["encoding_wall_seconds"]
            metadata["encoded_stream"] = encoded["encoded_stream"]
        write_metadata(self.output / "simulation_motion_recording.json", metadata)
        return metadata
