#!/usr/bin/env python3
"""Offline, zero-actuation replay and gripper-state sweep of one DROID request."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import platform
import sys
from typing import Any

import numpy as np

from saps.physical.droid_gripper import droid_gripper_decision
from saps.physical.fr3_forward_kinematics import fr3_tcp_fk
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import (
    DROID_POLICY_INPUT_KEYS,
    OpenPiDroidPolicy,
    map_droid_reference_joint_action,
    prepare_droid_observation,
)


ACTION_SHAPE = (15, 8)
CONFIG_NAME = "pi05_droid"
CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_droid"
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
FIRST_EIGHT = 8
SWEEP_CLOSURES = (0.0, 0.25, 0.50, 0.75, 1.0)
GRIPPER_KEY = "observation/gripper_position"
JOINT_KEY = "observation/joint_position"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def action_evidence(actions: np.ndarray) -> dict[str, Any]:
    if actions.shape != ACTION_SHAPE or not np.issubdtype(actions.dtype, np.floating):
        raise ValueError(f"Expected floating native [15,8] actions, got {actions.shape}/{actions.dtype}")
    if not np.isfinite(actions).all():
        raise ValueError("Native actions contain nonfinite values")
    return array_evidence(actions)  # Hashes original bytes, without dtype coercion.


def _single_npz(path: Path, key: str) -> np.ndarray:
    with np.load(path, allow_pickle=False) as bundle:
        if set(bundle.files) != {key}:
            raise ValueError(f"Unexpected keys in {path}: {bundle.files}")
        return np.array(bundle[key], copy=True)


def _check_file(record: dict[str, Any], path: Path) -> None:
    if record.get("path") != path.name or record.get("sha256") != file_sha256(path):
        raise ValueError(f"Archived file integrity mismatch: {path}")


def select_request(run_dir: Path) -> Path:
    """Choose earliest complete request with open hand and strong first-eight CLOSE."""
    directories = sorted(p for p in run_dir.glob("request_[0-9][0-9][0-9][0-9]") if p.is_dir())
    if not directories:
        raise ValueError(f"No archived requests in {run_dir}")
    for directory in directories:
        observation_path = directory / "observation.npz"
        action_path = directory / "actions.npz"
        if not observation_path.is_file() or not action_path.is_file():
            raise ValueError(f"Incomplete archived request: {directory}")
        with np.load(observation_path, allow_pickle=False) as bundle:
            closure = np.asarray(bundle[GRIPPER_KEY])
            if closure.shape != (1,) or closure.dtype != np.float32 or not np.isfinite(closure).all():
                raise ValueError(f"Invalid gripper state in {directory}")
        actions = _single_npz(action_path, "actions")
        action_evidence(actions)
        if float(closure[0]) <= 0.10 and float(np.max(actions[:FIRST_EIGHT, 7])) >= 0.80:
            return directory
    raise ValueError("No request satisfies closure <= 0.10 and max first-eight gripper >= 0.80")


def load_archive(request_dir: Path) -> tuple[dict[str, Any], np.ndarray, int, int, str, dict[str, Any]]:
    observation_path = request_dir / "observation.npz"
    action_path = request_dir / "actions.npz"
    request_path = request_dir / "request.json"
    response_path = request_dir / "response.json"
    request = json.loads(request_path.read_text(encoding="utf-8"))
    response = json.loads(response_path.read_text(encoding="utf-8"))
    _check_file(request["observation_bundle"], observation_path)
    _check_file(response["action_bundle"], action_path)

    with np.load(observation_path, allow_pickle=False) as bundle:
        if set(bundle.files) != set(DROID_POLICY_INPUT_KEYS):
            raise ValueError("Archive does not contain exactly the five canonical DROID fields")
        arrays = {key: np.array(bundle[key], copy=True) for key in DROID_POLICY_INPUT_KEYS}
    prompt_array = arrays.pop("prompt")
    if prompt_array.shape != () or prompt_array.dtype.kind != "U":
        raise ValueError("Archived prompt is not a scalar Unicode string")
    prompt = str(prompt_array.item())
    original = {**arrays, "prompt": prompt}
    expected_shapes = ((180, 320, 3), (180, 320, 3), (7,), (1,))
    expected_dtypes = (np.uint8, np.uint8, np.float32, np.float32)
    for key, shape, dtype in zip(DROID_POLICY_INPUT_KEYS[:4], expected_shapes, expected_dtypes):
        value = original[key]
        if value.shape != shape or value.dtype != dtype or not np.isfinite(value).all():
            raise ValueError(f"Invalid archived canonical field: {key}")
        if request["canonical_schema"][key] != array_evidence(value):
            raise ValueError(f"Archived canonical field disagrees with request.json: {key}")
    if request["canonical_schema"]["prompt"] != {"type": "str", "value": prompt}:
        raise ValueError("Archived prompt disagrees with request.json")
    if request["prompt"] != prompt:
        raise ValueError("Request prompt disagrees with canonical observation")
    observation = prepare_droid_observation(
        exterior_image=original[DROID_POLICY_INPUT_KEYS[0]],
        wrist_image=original[DROID_POLICY_INPUT_KEYS[1]],
        joint_position=original[JOINT_KEY],
        gripper_position=original[GRIPPER_KEY],
        prompt=prompt,
    )
    if any(array_evidence(observation[key]) != array_evidence(original[key])
           for key in DROID_POLICY_INPUT_KEYS[:4]):
        raise ValueError("Observation preparation changed archived bytes")

    actions = _single_npz(action_path, "actions")
    if response["action"] != action_evidence(actions):
        raise ValueError("Archived actions disagree with response.json")
    seed = int(request["policy_episode_seed"])
    replan = int(request["replan_index"])
    sampling = response["sampling_metadata"]
    if (replan != int(request_dir.name.removeprefix("request_"))
            or int(request["request_index"]) != replan
            or int(response["replan_index"]) != replan
            or int(sampling["replan_index"]) != replan
            or int(sampling["policy_episode_seed"]) != seed
            or int(sampling["protocol_version"]) != 1):
        raise ValueError("Archived seed, replan, or protocol identities disagree")
    noise = sampling.get("noise_sha256")
    if (not isinstance(noise, str) or len(noise) != 64
            or any(c not in "0123456789abcdef" for c in noise)):
        raise ValueError("Archived seeded-noise hash is invalid")
    return observation, actions, seed, replan, noise, {
        "request_dir": str(request_dir.resolve()),
        "files_sha256": {name: file_sha256(request_dir / name) for name in
                         ("observation.npz", "actions.npz", "request.json", "response.json")},
        "canonical_fields": request["canonical_schema"],
        "archived_action": action_evidence(actions),
        "original_gripper_closure": float(observation[GRIPPER_KEY][0]),
        "prompt": prompt,
    }


def changed_gripper_only(original: dict[str, Any], closure: float) -> dict[str, Any]:
    if not np.isfinite(closure) or not 0.0 <= closure <= 1.0:
        raise ValueError("Closure must be finite and in [0,1]")
    candidate = {key: (value.copy() if isinstance(value, np.ndarray) else value)
                 for key, value in original.items()}
    candidate[GRIPPER_KEY] = np.array([closure], dtype=np.float32)
    if candidate[GRIPPER_KEY].shape != (1,) or candidate[GRIPPER_KEY].dtype != np.float32:
        raise AssertionError("Invalid gripper replacement")
    if any(array_evidence(candidate[key]) != array_evidence(original[key])
           for key in DROID_POLICY_INPUT_KEYS[:4] if key != GRIPPER_KEY):
        raise AssertionError("Sweep changed another canonical array")
    if candidate["prompt"] != original["prompt"]:
        raise AssertionError("Sweep changed the prompt")
    return candidate


def ideal_rollout(start_q: np.ndarray, actions: np.ndarray) -> dict[str, Any]:
    """Perfect-target sequential rollout, not measured-state physical execution."""
    action_evidence(actions)
    q = np.empty((FIRST_EIGHT + 1, 7), dtype=np.float64)
    p = np.empty((FIRST_EIGHT + 1, 3), dtype=np.float64)
    q[0] = start_q
    p[0] = fr3_tcp_fk(q[0])[:3, 3]
    for index, action in enumerate(actions[:FIRST_EIGHT]):
        q[index + 1] = q[index] + map_droid_reference_joint_action(action).delta_q_rad
        p[index + 1] = fr3_tcp_fk(q[index + 1])[:3, 3]
    steps = np.diff(p, axis=0)
    total = p[-1] - p[0]
    return {"ideal_q_rad": q.tolist(), "ideal_tcp_xyz_m": p.tolist(),
            "ideal_tcp_steps_m": steps.tolist(),
            "cumulative_translation_m": total.tolist(),
            "cumulative_translation_norm_m": float(np.linalg.norm(total))}


def _sample(policy: OpenPiDroidPolicy, observation: dict[str, Any],
            seed: int, replan: int) -> tuple[np.ndarray, dict[str, Any]]:
    before = {key: array_evidence(observation[key]) for key in DROID_POLICY_INPUT_KEYS[:4]}
    prompt_before = observation["prompt"]
    response = policy.infer(observation, policy_episode_seed=seed, replan_index=replan)
    after = {key: array_evidence(observation[key]) for key in DROID_POLICY_INPUT_KEYS[:4]}
    if before != after or prompt_before != observation["prompt"]:
        raise RuntimeError("Inference mutated a canonical input field")
    actions = np.array(response.actions, copy=True)
    action_evidence(actions)
    sampling = response.sampling_metadata or {}
    return actions, sampling


def _comparison(current: np.ndarray, baseline: np.ndarray) -> dict[str, Any]:
    difference = current.astype(np.float64) - baseline.astype(np.float64)
    arm_difference = difference[:FIRST_EIGHT, :7]
    return {"identical_actions": bool(np.array_equal(current, baseline)),
            "max_abs_action_difference": float(np.max(np.abs(difference))),
            "max_abs_first_eight_arm_difference": float(np.max(np.abs(arm_difference))),
            "rms_first_eight_arm_difference": float(np.sqrt(np.mean(arm_difference**2)))}


def run_gate(policy: OpenPiDroidPolicy, observation: dict[str, Any],
             archived: np.ndarray, seed: int, replan: int, archived_noise: str,
             report: dict[str, Any], arrays: dict[str, np.ndarray]) -> None:
    """Stop after an exact-replay failure; never submit a sweep in that case."""
    arrays["archived_actions"] = archived
    replays: list[np.ndarray] = []
    report["replays"] = []
    report["gate"] = {"passed": False}
    for index in range(3):
        actions, sampling = _sample(policy, observation, seed, replan)
        arrays[f"replay_{index}"] = actions
        replays.append(actions)
        report["replays"].append({"sampling_metadata": sampling, "action": action_evidence(actions),
                                  **_comparison(actions, archived)})
        if (sampling.get("policy_episode_seed") != seed
                or sampling.get("replan_index") != replan
                or sampling.get("noise_sha256") != archived_noise):
            raise RuntimeError("Seeded inference metadata/noise differs from archived request; sweep skipped")
    exact = all(np.array_equal(actions, archived) and
                action_evidence(actions) == action_evidence(archived) for actions in replays)
    exact &= all(np.array_equal(actions, replays[0]) and
                 action_evidence(actions) == action_evidence(replays[0]) for actions in replays[1:])
    report["gate"] = {"passed": bool(exact),
                      "criterion": "Three replays exactly match one another and archived array, dtype and bytes; noise matches archive"}
    if not exact:
        raise RuntimeError("Deterministic archived-request replay failed; sweep skipped")

    baseline = replays[0]
    baseline_rollout = ideal_rollout(observation[JOINT_KEY], baseline)
    report["sweep"] = []
    arrays["closures"] = np.asarray(SWEEP_CLOSURES, dtype=np.float32)
    for index, closure in enumerate(SWEEP_CLOSURES):
        candidate = changed_gripper_only(observation, closure)
        actions, sampling = _sample(policy, candidate, seed, replan)
        arrays[f"sweep_{index}"] = actions
        if (sampling.get("policy_episode_seed") != seed
                or sampling.get("replan_index") != replan
                or sampling.get("noise_sha256") != archived_noise):
            report["sweep"].append({"input_gripper_closure": closure,
                                    "sampling_metadata": sampling,
                                    "action": action_evidence(actions),
                                    "valid_comparison": False})
            raise RuntimeError("Sweep inference metadata/noise differs from archived request")
        rollout = ideal_rollout(observation[JOINT_KEY], actions)
        decisions = ["CLOSED" if droid_gripper_decision(float(value)).binary_closure else "OPEN"
                     for value in actions[:FIRST_EIGHT, 7]]
        report["sweep"].append({
            "input_gripper_closure": closure, "noise_sha256": sampling["noise_sha256"],
            "action": action_evidence(actions), **_comparison(actions, baseline),
            "first_eight_arm_actions": actions[:FIRST_EIGHT, :7].tolist(),
            "raw_gripper_actions_15": actions[:, 7].tolist(),
            "first_eight_gripper_decisions": decisions,
            "first_eight_close_count": decisions.count("CLOSED"),
            **rollout,
            "final_ideal_tcp_delta_vs_archived_m":
                (np.asarray(rollout["ideal_tcp_xyz_m"][-1]) -
                 np.asarray(baseline_rollout["ideal_tcp_xyz_m"][-1])).tolist(),
            "cumulative_translation_delta_vs_archived_m":
                (np.asarray(rollout["cumulative_translation_m"]) -
                 np.asarray(baseline_rollout["cumulative_translation_m"])).tolist(),
        })


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--expected-request", default="request_0006")
    parser.add_argument("--config", type=Path, default=Path("configs/physical_pi05_fr3.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {
        "schema_version": 1, "diagnostic": "g2_archived_gripper_state_sweep",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "status": "failed", "zero_actuation": True,
        "runtime": {"python": sys.version, "platform": platform.platform(), "numpy": np.__version__},
    }
    arrays: dict[str, np.ndarray] = {}
    try:
        request_dir = select_request(args.run_dir)
        if request_dir.name != args.expected_request:
            raise ValueError(f"Earliest qualifying request is {request_dir.name}, expected {args.expected_request}")
        observation, archived, seed, replan, noise, provenance = load_archive(request_dir)
        report["archive"] = provenance
        report["selection"] = {"criterion": "earliest closure <= 0.10 and max first-eight gripper >= 0.80",
                               "selected": request_dir.name}
        report["seed"] = seed
        report["replan_index"] = replan
        report["archived_noise_sha256"] = noise
        config = json.loads(args.config.read_text(encoding="utf-8"))
        expected_policy = {"config": CONFIG_NAME, "checkpoint": CHECKPOINT,
                           "openpi_commit": OPENPI_COMMIT, "action_shape": [15, 8],
                           "reference_future_open_loop_horizon": 8}
        if config.get("policy") != expected_policy:
            raise ValueError("Local physical policy configuration differs from archived DROID identity")
        report["config_sha256"] = file_sha256(args.config)
        policy = OpenPiDroidPolicy(host=args.host, port=args.port)
        policy.validate_policy_identity(config_name=CONFIG_NAME, checkpoint=CHECKPOINT)
        metadata = policy.server_metadata
        seeded = metadata["saps_seeded_sampling"]
        audit = metadata.get("saps_model_input_audit", {})
        if (seeded.get("action_horizon") != ACTION_SHAPE[0]
                or audit.get("schema_version") != 1
                or audit.get("openpi_commit") != OPENPI_COMMIT):
            raise RuntimeError("Seeded server horizon or pinned OpenPI identity differs")
        report["server_metadata"] = metadata
        run_gate(policy, observation, archived, seed, replan, noise, report, arrays)
        report["status"] = "passed"
    except Exception as error:
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        np.savez_compressed(args.output_dir / "actions.npz", **arrays)
        with (args.output_dir / "run.json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"{report['status']}: {args.output_dir / 'run.json'}", flush=True)


if __name__ == "__main__":
    main()
