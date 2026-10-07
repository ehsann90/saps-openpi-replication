"""Pure sequencing checks for unlimited SIM-P6 replanning."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.policy_execution import (
    GripperRuntime, InvalidPolicyResponse, command_fresh_hold,
    execute_chunk, run_rollout_loop,
)


class RolloutLoopTest(unittest.TestCase):
    def test_next_capture_has_no_step_after_terminal_hold(self) -> None:
        class World:
            current_time_step_index = 0
            current_time = 0.0

            def step(self, *, render):
                self.current_time_step_index += 1
                self.current_time = self.current_time_step_index / 60

        class Articulation:
            def __init__(self):
                self.q = np.zeros(9, dtype=np.float32)
                self.targets = self.q.copy()

            def get_dof_positions(self):
                return self.q

            def get_dof_velocities(self):
                return np.zeros(9, dtype=np.float32)

            def get_dof_limits(self, *, dof_indices):
                return np.full((1, 7), -2.0), np.full((1, 7), 2.0)

            def set_dof_position_targets(self, values, *, dof_indices):
                self.targets[np.asarray(dof_indices)] = np.asarray(values).reshape(-1)

            def get_dof_position_targets(self, *, dof_indices):
                return self.targets[np.asarray(dof_indices)].reshape(1, -1)

        world = World()
        handles = SimpleNamespace(
            world=world, fr3=Articulation(), arm_indices=np.arange(7),
            finger_indices=np.arange(7, 9),
            target=SimpleNamespace(get_world_pose=lambda: (
                np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
            )),
        )
        pose = lambda config: (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        scene = SimpleNamespace(get_tcp_pose=pose)
        simulation_manager = SimpleNamespace(
            SimulationManager=SimpleNamespace(get_physics_dt=lambda: 1 / 60)
        )
        terminal_steps = []
        capture_steps = []

        def hold(index):
            record = command_fresh_hold(handles)
            self.assertEqual(len(record["measured_arm_q_rad"]), 7)
            self.assertEqual(len(record["active_arm_q_target_rad"]), 7)

        def capture(index):
            capture_steps.append(world.current_time_step_index)
            if index:
                self.assertEqual(capture_steps[-1], terminal_steps[-1])
            return index

        def execute(index, response):
            result = execute_chunk(
                handles, {"droid": {"gripper_max_width_m": 0.08}},
                np.zeros((8, 8)), physics_dt=1 / 60,
                terminal_evidence_steps=0,
            )
            terminal_steps.append(result["terminal_hold"]["simulation_step_index"])
            self.assertEqual(result["simulated_execution_seconds"], 8 / 15)
            self.assertEqual(world.current_time_step_index, terminal_steps[-1])
            return result

        with patch.dict(sys.modules, {
            "isaac_fr3.scene": scene,
            "isaacsim": SimpleNamespace(),
            "isaacsim.core": SimpleNamespace(),
            "isaacsim.core.simulation_manager": simulation_manager,
        }):
            summary = run_rollout_loop(
                hold=hold,
                capture=capture,
                infer=lambda index, observation: index,
                execute=execute, record=lambda *items: None,
                emergency_hold=lambda: None, max_replans=2,
            )
        self.assertEqual(summary["completed_replans"], 2)
        self.assertEqual(capture_steps, [0, 32])
        self.assertEqual(terminal_steps, [32, 64])

    def test_gripper_transition_persists_across_chunks(self) -> None:
        runtime = GripperRuntime()
        first = runtime.request_intent(0.7, 0.08)
        runtime.ramp_steps = 12
        second = runtime.request_intent(0.9, 0.06)
        self.assertEqual(runtime.ramp_steps, 12)
        third = runtime.request_intent(0.5, 0.04)
        self.assertTrue(first["transition"])
        self.assertEqual(first["from"], "OPEN")
        self.assertFalse(second["transition"])
        self.assertEqual(runtime.intent, "OPEN")
        self.assertTrue(third["transition"])
        self.assertEqual(runtime.ramp_origin_width_m, 0.04)
        self.assertEqual(runtime.ramp_steps, 0)

    def test_unlimited_default_orders_hold_capture_infer_execute(self) -> None:
        events = []
        sim_steps = [0]

        def hold(index):
            events.append((index, "hold"))

        def capture(index):
            events.append((index, "capture"))
            if index:
                self.assertEqual(events[-2], (index, "hold"))
                self.assertEqual(events[-3], (index - 1, "record"))
            return index

        def infer(index, observation):
            events.append((index, "infer"))
            self.assertEqual(index, observation)
            self.assertEqual(sim_steps[0], 32 * index)
            return [index] * 15

        def execute(index, response):
            events.append((index, "execute"))
            self.assertEqual(len(response), 15)
            sim_steps[0] += 32
            return {
                "status": "complete", "actions_executed": 8,
                "terminal_hold": {"applied": True},
            }

        def record(index, observation, response, execution):
            events.append((index, "record"))

        result = run_rollout_loop(
            hold=hold, capture=capture, infer=infer, execute=execute,
            record=record, emergency_hold=lambda: None,
            max_replans=3,
        )
        self.assertEqual(result["termination_reason"], "max_replans")
        self.assertEqual(result["inference_requests"], 3)
        self.assertEqual(result["completed_replans"], 3)
        self.assertEqual(result["policy_actions_executed"], 24)
        self.assertEqual([i for i, kind in events if kind == "infer"], [0, 1, 2])
        self.assertEqual(events[:5], [
            (0, "hold"), (0, "capture"), (0, "infer"),
            (0, "execute"), (0, "record"),
        ])

    def test_default_is_unlimited_and_interrupt_flushes(self) -> None:
        indices = []
        holds = []

        def infer(index, observation):
            indices.append(index)
            if index == 2:
                raise KeyboardInterrupt
            return index

        result = run_rollout_loop(
            hold=lambda index: holds.append(("boundary", index)),
            capture=lambda index: index,
            infer=infer,
            execute=lambda index, response: {
                "status": "complete", "actions_executed": 8,
                "terminal_hold": {"applied": True},
            },
            record=lambda *items: None,
            emergency_hold=lambda: holds.append(("interrupt", None)),
        )
        self.assertEqual(indices, [0, 1, 2])
        self.assertEqual(holds[-1], ("interrupt", None))
        self.assertEqual(result["termination_reason"], "manual_interrupt")
        self.assertEqual(result["inference_requests"], 3)
        self.assertEqual(result["completed_replans"], 2)
        self.assertEqual(result["policy_actions_executed"], 16)

    def test_invalid_response_stops_before_second_inference(self) -> None:
        indices = []
        emergency = []

        def infer(index, observation):
            indices.append(index)
            raise InvalidPolicyResponse("short action chunk")

        result = run_rollout_loop(
            hold=lambda index: None, capture=lambda index: index,
            infer=infer, execute=lambda *items: self.fail("must not execute"),
            record=lambda *items: self.fail("must not record"),
            emergency_hold=lambda: emergency.append(True),
        )
        self.assertEqual(indices, [0])
        self.assertEqual(emergency, [True])
        self.assertEqual(result["termination_reason"], "invalid_policy_response")
        self.assertEqual(result["completed_replans"], 0)

    def test_safety_abort_does_not_infer_again_or_repeat_hold(self) -> None:
        calls = []
        result = run_rollout_loop(
            hold=lambda index: calls.append(("hold", index)),
            capture=lambda index: index,
            infer=lambda index, observation: calls.append(("infer", index)),
            execute=lambda index, response: {
                "status": "aborted", "actions_executed": 2,
                "failure": "joint limit", "terminal_hold": {"applied": True},
            },
            record=lambda *items: None,
            emergency_hold=lambda: calls.append(("emergency", None)),
        )
        self.assertEqual(calls, [("hold", 0), ("infer", 0)])
        self.assertEqual(result["termination_reason"], "safety_violation")
        self.assertEqual(result["policy_actions_executed"], 2)

    def test_completed_chunk_without_terminal_hold_stops_rollout(self) -> None:
        indices = []
        emergency = []
        result = run_rollout_loop(
            hold=lambda index: None, capture=lambda index: index,
            infer=lambda index, observation: indices.append(index),
            execute=lambda index, response: {
                "status": "complete", "actions_executed": 8,
                "terminal_hold": {"applied": False},
            },
            record=lambda *items: None,
            emergency_hold=lambda: emergency.append(True),
        )
        self.assertEqual(indices, [0])
        self.assertEqual(emergency, [True])
        self.assertEqual(result["termination_reason"], "runtime_error")


if __name__ == "__main__":
    unittest.main()
