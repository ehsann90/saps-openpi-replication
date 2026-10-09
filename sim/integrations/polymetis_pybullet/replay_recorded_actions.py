"""Replay recorded DROID joint commands in pinned Panda PyBullet."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import socket
import statistics
import subprocess
import sys
import time

import grpc
from hydra.experimental import compose, initialize_config_dir
from hydra.utils import instantiate
import torch

from direct_step_hold import SERVER, sample, wait_for_server
from polymetis import RobotInterface


CONDITIONS = (
    "absolute_franka_hand", "absolute_robotiq", "normalized_actions"
)


def check_q(q: list, lower: list, upper: list, label: str) -> float:
    """Validate all seven Panda coordinates without changing them."""
    if len(q) != 7 or not all(math.isfinite(float(x)) for x in q):
        raise ValueError(f"{label}: expected seven finite joints")
    margins = [min(x - lo, hi - x) for x, lo, hi in zip(q, lower, upper)]
    if min(margins) < 0:
        raise ValueError(f"{label}: joint {margins.index(min(margins)) + 1} out of bounds")
    return min(margins)


def wait_until(deadline: float) -> None:
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(remaining)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--condition", choices=CONDITIONS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=51073)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    manifest_bytes = args.manifest.read_bytes()
    manifest = json.loads(manifest_bytes)
    condition = manifest["conditions"][args.condition]
    lower = manifest["panda_joint_lower_rad"]
    upper = manifest["panda_joint_upper_rad"]
    initial_q = condition["initial_q_rad"]
    check_q(initial_q, lower, upper, "initial")
    required_home = manifest.get("required_home_q_rad")
    if required_home is not None and initial_q != required_home:
        raise ValueError("condition initial q differs from required HOME")
    if args.condition.startswith("absolute_"):
        targets = condition["absolute_targets_rad"]
        if len(targets) != 8:
            raise ValueError("expected eight absolute targets")
        for index, target in enumerate(targets):
            check_q(target, lower, upper, f"absolute target {index}")
    else:
        actions = condition["policy_arm_actions"]
        if len(actions) != 8 or any(len(action) != 7 for action in actions):
            raise ValueError("expected eight seven-coordinate actions")
        if any(not all(math.isfinite(x) for x in action) for action in actions):
            raise ValueError("nonfinite policy action")

    with socket.socket() as check:
        check.bind(("127.0.0.1", args.port))
    args.output.mkdir(parents=True)
    command = [SERVER, "-s", "127.0.0.1", "-p", str(args.port)]
    result = {
        "condition": args.condition,
        "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "action_hz": manifest["action_hz"],
        "server_command": command,
        "port": args.port,
        "actions": [],
        "panda_limits_rad": {"lower": lower, "upper": upper},
        "requested_initial_q_rad": initial_q,
    }
    server = client = robot = None
    try:
        with (args.output / "server.log").open("w") as log:
            server = subprocess.Popen(command, stdout=log, stderr=log)
            wait_for_server(args.port, server)
            result["server_ready"] = True
            overrides = [
                "robot_client=franka_sim", "robot_model=franka_panda",
                "gui=false", "use_real_time=false", "ip=127.0.0.1",
                f"port={args.port}",
            ]
            with initialize_config_dir(
                config_dir="/opt/fairo/polymetis/polymetis/conf",
                job_name="recorded_actions",
            ):
                cfg = compose(config_name="launch_robot", overrides=overrides)
            result["hydra_overrides"] = overrides
            client = instantiate(cfg.robot_client)
            loaded_lower = list(client.env.robot_model_cfg.joint_limits_low)
            loaded_upper = list(client.env.robot_model_cfg.joint_limits_high)
            if loaded_lower != lower or loaded_upper != upper:
                raise ValueError("manifest limits differ from loaded Panda model")
            result["model_path"] = client.env.robot_description_path
            result["physics_dt_s"] = client.env.sim.getPhysicsEngineParameters()[
                "fixedTimeStep"
            ]
            client.env.reset(joint_pos=initial_q, joint_vel=[0.0] * 7)
            result["reset_q_dq"] = [
                values.tolist() for values in client.env.get_current_joint_pos_vel()
            ]
            client.run_no_wait()
            for _ in range(100):
                try:
                    robot = RobotInterface(ip_address="127.0.0.1", port=args.port)
                    break
                except (grpc.RpcError, AssertionError):
                    time.sleep(0.1)
            if robot is None:
                raise TimeoutError("client metadata timeout")
            metadata = robot.metadata
            result["metadata"] = {
                "version": metadata.polymetis_version,
                "dof": metadata.dof,
                "ee_link_name": metadata.ee_link_name,
                "Kq": list(metadata.default_Kq),
                "Kqd": list(metadata.default_Kqd),
                "Kx": list(metadata.default_Kx),
                "Kxd": list(metadata.default_Kxd),
                "control_hz": metadata.hz,
                "urdf_sha256": hashlib.sha256(
                    metadata.urdf_file.encode()
                ).hexdigest(),
            }
            result["controller_type_from_pinned_source"] = (
                "HybridJointImpedanceControl"
            )
            stable = False
            velocity_tolerance = manifest.get(
                "initial_velocity_tolerance_rad_s", 0.01
            )
            for _ in range(100):
                state = robot.get_robot_state()
                if max(abs(v) for v in state.joint_velocities) < velocity_tolerance:
                    stable = True
                    break
                time.sleep(0.02)
            result["initial"] = sample(state, time.monotonic())
            if not stable:
                raise RuntimeError("Panda did not reach stationary initial state")
            result["initial_limit_margin_rad"] = check_q(
                result["initial"]["q"], lower, upper, "measured initial"
            )
            if required_home is not None:
                position_error = max(
                    abs(q - home) for q, home in zip(
                        result["initial"]["q"], required_home
                    )
                )
                velocity = max(abs(v) for v in result["initial"]["dq"])
                result["initial_home_check"] = {
                    "max_abs_position_error_rad": position_error,
                    "max_abs_velocity_rad_s": velocity,
                    "passed": (
                        position_error < manifest["initial_tolerance_rad"]
                        and velocity < velocity_tolerance
                    ),
                }
                if not result["initial_home_check"]["passed"]:
                    raise RuntimeError("measured Panda state differs from HOME")
            robot.start_cartesian_impedance()
            result["policy_running"] = robot.is_running_policy()
            if not result["policy_running"]:
                raise RuntimeError("hybrid controller did not start")

            period = 1.0 / manifest["action_hz"]
            start = time.monotonic() + 0.015
            for index in range(8):
                planned = start + index * period
                wait_until(planned)
                issue_state = robot.get_robot_state()
                issue = sample(issue_state, start)
                q = issue["q"]
                if index == 0 and required_home is not None:
                    if (
                        max(abs(x - h) for x, h in zip(q, required_home))
                        >= manifest["initial_tolerance_rad"]
                        or max(abs(v) for v in issue["dq"])
                        >= velocity_tolerance
                    ):
                        raise RuntimeError("first command boundary differs from HOME")
                if args.condition.startswith("absolute_"):
                    target = targets[index]
                    clipped = None
                else:
                    clipped = [max(-1.0, min(1.0, x)) for x in actions[index]]
                    target = [x + 0.2 * u for x, u in zip(q, clipped)]
                margin = check_q(target, lower, upper, f"issued target {index}")
                delta = [t - x for t, x in zip(target, q)]
                before = time.monotonic()
                update_index = robot.update_desired_joint_positions(
                    torch.tensor(target, dtype=torch.float32)
                )
                after = time.monotonic()
                result["actions"].append({
                    "index": index,
                    "planned_issue_wall_s": planned - start,
                    "issue_state": issue,
                    "command_start_wall_s": before - start,
                    "command_return_wall_s": after - start,
                    "update_index": update_index,
                    "policy_action": actions[index] if clipped is not None else None,
                    "clipped_action": clipped,
                    "desired_delta_rad": delta,
                    "target_q_rad": target,
                    "target_limit_margin_rad": margin,
                })
            wait_until(start + 8 * period)
            result["final_boundary"] = sample(robot.get_robot_state(), start)
            for current, following in zip(
                result["actions"], result["actions"][1:]
            ):
                current["end_boundary"] = following["issue_state"]
            result["actions"][-1]["end_boundary"] = result["final_boundary"]
            result["policy_running_at_end"] = robot.is_running_policy()
            policy_log = robot.terminate_current_policy(
                return_log=True, timeout=15.0
            )
            ticks = [sample(state, start) for state in policy_log]
            for tick_index, tick in enumerate(ticks):
                tick.pop("wall_s")
                tick["sim_time_s"] = tick_index * result["physics_dt_s"]
            (args.output / "server_policy_log.json").write_text(
                json.dumps(ticks, indent=2) + "\n"
            )
            result["server_policy_log_count"] = len(ticks)
            intervals = [
                b["robot_time_s"] - a["robot_time_s"]
                for a, b in zip(ticks, ticks[1:])
            ]
            result["median_server_wall_period_s"] = statistics.median(intervals)
            result["fault_codes"] = sorted({t["error_code"] for t in ticks})
            result["all_commands_successful"] = all(
                t["prev_command_successful"] for t in ticks
            )
            result["policy_terminated"] = not robot.is_running_policy()
            result["control_success"] = (
                result["fault_codes"] == [0]
                and result["all_commands_successful"]
                and result["policy_running_at_end"]
                and result["policy_terminated"]
                and len(ticks) > max(a["update_index"] for a in result["actions"])
            )
    except Exception as exc:
        result["control_success"] = False
        result["failure"] = repr(exc)
    finally:
        if robot is not None:
            robot.channel.close()
        if client is not None:
            if client._runner is not None:
                client.kill_run()
            result["client_thread_stopped"] = client._runner is None
            client.channel.close()
            client.env.sim.disconnect()
        if server is not None:
            server.terminate()
            try:
                result["server_exit_code"] = server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                result["server_exit_code"] = server.wait(timeout=3)
                result["forced_server_kill"] = True
        try:
            with socket.create_connection(("127.0.0.1", args.port), timeout=0.2):
                result["port_closed"] = False
        except OSError:
            result["port_closed"] = True
        result["success"] = (
            result.get("control_success", False)
            and result.get("client_thread_stopped", False)
            and result["port_closed"]
            and not result.get("forced_server_kill", False)
        )
        (args.output / "replay.json").write_text(
            json.dumps(result, indent=2) + "\n"
        )
        print(json.dumps({
            key: value for key, value in result.items()
            if key not in ("actions", "initial", "final_boundary")
        }, indent=2))
    if not result["success"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
