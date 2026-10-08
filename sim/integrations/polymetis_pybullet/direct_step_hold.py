"""Run a finite Panda target/hold test with the pinned Polymetis stack."""

import argparse
import hashlib
import json
from pathlib import Path
import socket
import statistics
import subprocess
import sys
import time

from hydra.experimental import compose, initialize_config_dir
from hydra.utils import instantiate
import grpc
import torch

from polymetis import RobotInterface
from polymetis_pb2 import RobotState


SERVER = "/opt/fairo/polymetis/polymetis/build/run_server"


def sample(state: RobotState, started: float) -> dict:
    return {
        "wall_s": time.monotonic() - started,
        "robot_time_s": state.timestamp.seconds + state.timestamp.nanos * 1e-9,
        "q": list(state.joint_positions),
        "dq": list(state.joint_velocities),
        "joint_torques_computed": list(state.joint_torques_computed),
        "prev_joint_torques_computed": list(state.prev_joint_torques_computed),
        "prev_joint_torques_computed_safened": list(
            state.prev_joint_torques_computed_safened
        ),
        "motor_torques_measured": list(state.motor_torques_measured),
        "motor_torques_external": list(state.motor_torques_external),
        "error_code": state.error_code,
        "prev_command_successful": state.prev_command_successful,
        "prev_controller_latency_ms": state.prev_controller_latency_ms,
    }


def wait_for_server(port: int, process: subprocess.Popen) -> None:
    for _ in range(100):
        if process.poll() is not None:
            raise RuntimeError(f"server exited: {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError("server readiness timeout")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=51073)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    with socket.socket() as check:
        check.bind(("127.0.0.1", args.port))
    args.output.mkdir(parents=True)
    command = [SERVER, "-s", "127.0.0.1", "-p", str(args.port)]
    result = {"server_command": command, "port": args.port, "samples": []}
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
                job_name="step_hold",
            ):
                cfg = compose(config_name="launch_robot", overrides=overrides)
            result["hydra_overrides"] = overrides
            client = instantiate(cfg.robot_client)
            result["client_type"] = type(client).__name__
            result["model_path"] = client.env.robot_description_path
            result["physics_timestep_s"] = client.env.sim.getPhysicsEngineParameters()[
                "fixedTimeStep"
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
                "rest_pose": list(metadata.rest_pose),
                "urdf_sha256": hashlib.sha256(
                    metadata.urdf_file.encode()
                ).hexdigest(),
            }
            result["panda_loaded"] = "panda_link" in metadata.urdf_file
            result["control_hz"] = metadata.hz
            result["controller_type_from_pinned_source"] = (
                "HybridJointImpedanceControl"
            )
            for _ in range(100):
                initial = robot.get_robot_state()
                if max(abs(v) for v in initial.joint_velocities) < 0.01:
                    break
                time.sleep(0.02)
            else:
                raise RuntimeError("start was not stationary")
            started = time.monotonic()
            result["samples"].append(sample(initial, started))
            result["initial_q"] = list(initial.joint_positions)
            result["initial_dq"] = list(initial.joint_velocities)
            robot.start_cartesian_impedance()
            result["policy_running"] = robot.is_running_policy()
            if not result["policy_running"]:
                raise RuntimeError("policy did not start")
            target = torch.tensor(result["initial_q"], dtype=torch.float32)
            target[0] += 0.03
            result["target_q"] = target.tolist()
            result["target_issue_s"] = time.monotonic() - started
            result["update_index"] = robot.update_desired_joint_positions(target)
            for _ in range(100):
                result["samples"].append(sample(robot.get_robot_state(), started))
                time.sleep(0.02)
            result["hold_s"] = time.monotonic() - started
            result["final_q"] = result["samples"][-1]["q"]
            result["final_dq"] = result["samples"][-1]["dq"]
            result["fault_codes"] = sorted({
                s["error_code"] for s in result["samples"]
            })
            result["commands_successful"] = all(
                s["prev_command_successful"] for s in result["samples"][1:]
            )
            result["policy_running_at_end"] = robot.is_running_policy()
            policy_log = robot.terminate_current_policy(
                return_log=True, timeout=15.0
            )
            logged = [sample(state, started) for state in policy_log]
            for entry in logged:
                entry.pop("wall_s")
            (args.output / "server_policy_log.json").write_text(
                json.dumps(logged, indent=2) + "\n"
            )
            result["server_policy_log_count"] = len(logged)
            intervals = [
                later["robot_time_s"] - earlier["robot_time_s"]
                for earlier, later in zip(logged, logged[1:])
            ]
            if intervals:
                result["observed_server_tick_median_s"] = statistics.median(
                    intervals
                )
            result["policy_terminated"] = not robot.is_running_policy()
            result["control_success"] = (
                result["fault_codes"] == [0]
                and result["commands_successful"]
                and result["policy_running_at_end"]
                and result["policy_terminated"]
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
        (args.output / "step_hold.json").write_text(
            json.dumps(result, indent=2) + "\n"
        )
        print(json.dumps({k: v for k, v in result.items() if k != "samples"},
                         indent=2))
    if not result["success"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
