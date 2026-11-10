"""Offline snapshot checks; neither a trajectory validator nor motion approval."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from saps.physical.discrete_verifier import candidate_target, limits_from_urdf
from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.fr3_forward_kinematics import fr3_flange_fk, fr3_flange_jacobian
from saps.physical.single_action import (
    first_action_from_chunk, JOINT_MARGIN, singularity_metric,
)


def audit_targets(xml: str, request: dict, actions: Any) -> dict:
    """Check each prediction against one saved q, preserving runtime constants.

    No timestamp is made fresh and no future measured state is simulated.
    All rows use the same observation q, never the preceding proposed target.
    """
    first_action_from_chunk(actions)
    if tuple(request['joint_names']) != FR3_JOINT_NAMES:
        raise ValueError('Saved joint order does not match FR3 arm order')
    q = np.asarray(request['joint_positions'], dtype=np.float64)
    limits = limits_from_urdf(xml)
    measured_metric = singularity_metric(q)
    rows = []
    for index, action in enumerate(actions):
        row = candidate_target(action, q, limits)
        reasons = list(row['rejection_reasons'])
        if row['minimum_target_margin_rad'] < JOINT_MARGIN:
            reasons.append('target_inside_existing_joint_margin')
        metric = singularity_metric(row['proposed_q_target_rad'])
        if measured_metric['rejected']:
            reasons.append('measured_singularity_gate_rejected')
        if metric['rejected']:
            reasons.append('target_singularity_gate_rejected')
        row.update(index=index, inside_execution_window=index < 8,
                   joint_margin_rad=JOINT_MARGIN, target_singularity=metric,
                   snapshot_checks_passed=not reasons, rejection_reasons=reasons)
        rows.append(row)
    return dict(
        reference='saved observation q for every row; fresh measured q required at execution',
        measured_singularity=measured_metric, rows=rows,
        first_chunk_snapshot_checks_passed=all(r['snapshot_checks_passed'] for r in rows[:8]),
        all_predictions_snapshot_checks_passed=all(r['snapshot_checks_passed'] for r in rows),
        safe_to_execute=False, physical_rollout_readiness='NO-GO',
        policy_actions_executed=0, gripper_commands_issued=0,
        unevaluated=['live state/action age and command deadlines',
                     'actual future measured q', 'tool transform and payload',
                     'self/environment/tool collision and workspace clearance',
                     'controller/forwarder health and delivery',
                     'velocity/acceleration and robot-native safety',
                     'operator authorization and emergency stop'])


def compare_arm_model(xml: str, source: Path, report: dict) -> dict:
    """Compare URDF-derived Pinocchio flange FK/Jacobian to runtime NumPy FK.

    Non-arm joints are locked only for this offline arm comparison. Tool mesh,
    inertia and contact-envelope approval are deliberately not inferred.
    """
    import pinocchio as pin
    from saps.physical.fr3_kinematics import Fr3PinocchioKinematics

    full = pin.buildModelFromXML(xml)
    locked = [i for i in range(1, full.njoints) if full.names[i] not in FR3_JOINT_NAMES]
    model = pin.buildReducedModel(full, locked, pin.neutral(full)) if locked else full
    independent = Fr3PinocchioKinematics(
        model, pin, urdf_xml=xml, source_xacro_path=source,
        end_effector_frame='fr3_link8')
    positions = [report['rows'][0]['measured_q_rad']]
    positions.extend(row['proposed_q_target_rad'] for row in report['rows'])
    fk_errors, jacobian_errors = [], []
    for q in positions:
        manual = fr3_flange_fk(q)
        jacobian = fr3_flange_jacobian(q)
        fk_errors.append(float(np.max(np.abs(independent.forward_kinematics(q) - manual))))
        jacobian_errors.append(float(np.max(np.abs(independent.jacobian(q) - jacobian))))
    tolerance = 1e-10  # Numerical equivalence only; not a physical safety threshold.
    return dict(
        backend='Pinocchio', version=pin.__version__, base='fr3_link0', tip='fr3_link8',
        positions_checked=len(positions),
        locked_non_arm_joints=[full.names[i] for i in locked],
        maximum_fk_element_error=max(fk_errors),
        maximum_jacobian_element_error=max(jacobian_errors),
        numerical_comparison_tolerance=tolerance,
        flange_model_matches_runtime=(max(fk_errors + jacobian_errors) <= tolerance),
        frames=[frame.name for frame in full.frames],
        tool_geometry_verified=False, payload_verified=False,
        limitation='Arm flange numerical comparison only; tool/TCP and collision behavior remain unverified')
