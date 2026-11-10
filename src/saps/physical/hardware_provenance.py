"""Recorded development sources, strict experiment pins, and independent safety guards."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import runpy
import subprocess
from types import SimpleNamespace
from typing import Any

from saps.physical.shadow_ros import git_identity
from saps.physical.ros_packages import lab_runtime_file


def hardware_timing_helpers(directory: Path, *, strict: bool = True) -> tuple[Any, dict]:
    if not strict:
        # Explicit development mode records actual code rather than treating
        # the reviewed experiment manifest as a physical safety mechanism.
        from saps.physical.rollout_provenance import repository_snapshot
        identity = repository_snapshot(directory)
        helper = lab_runtime_file(directory, 'timing_evidence.py')
        identity.update(provenance_mode='development',
                        timing_helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest())
        return SimpleNamespace(**runpy.run_path(str(helper))), identity
    manifest_path = Path(__file__).resolve().parents[3] / 'configs/zed_robotiq_lab_sources.json'
    manifest = json.loads(manifest_path.read_text())
    identity = git_identity(directory)
    if identity.get('dirty', True):
        raise ValueError('New hardware requires a reviewed, committed clean lab checkout')
    subprocess.run(['git', '-C', str(directory), 'merge-base', '--is-ancestor',
                    manifest['baseline_commit'], 'HEAD'], check=True, timeout=10)
    tracked = subprocess.check_output(['git', '-C', str(directory), 'ls-files', '-z'], timeout=10)
    paths = [name for name in tracked.decode().split('\0') if name]
    actual = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in paths}
    if actual != manifest['files_sha256']:
        raise ValueError('Lab source tree differs from reviewed hardware manifest')
    saps = git_identity(Path(__file__).resolve().parents[3])
    if saps['dirty']:
        raise ValueError('New hardware execution requires a clean committed SAPS checkout')
    identity.update(source_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                    verified_files_sha256=actual, baseline_commit=manifest['baseline_commit'])
    return SimpleNamespace(**runpy.run_path(str(lab_runtime_file(directory, 'timing_evidence.py')))), identity


def arm_geometry_blockers(config: dict, *, joint_space: bool = False) -> list[str]:
    """Describe unresolved hardware findings, not joint-space authorization.

    Kept under its historical name for existing diagnostic callers. Joint-space
    findings are reported separately from enforced runtime rejection reasons.
    Cartesian callers still cannot substitute the historical Hand TCP.
    """
    if config.get('hardware_profile') != 'zed_robotiq_2f85':
        return []
    requirements = [
        'mounted-tool collision envelope, clearance and workspace coverage remain unverified',
        'configured controller end-effector frame and hybrid-gain compatibility remain unverified',
        'end-effector and payload inertial settings have not been matched to measured hardware',
        'native effort/motion restrictions (torque-slew and applicable velocity/acceleration '
        'protection) and operator emergency-stop arrangements are not established by source inspection',
    ]
    if not joint_space:
        requirements.insert(0, 'Cartesian-tool TCP/FK dependencies remain unverified')
    return requirements


def require_supported_arm_geometry(config: dict, *, joint_space: bool = False) -> None:
    """Reject unsupported Cartesian-tool use; no profile blacklist for joint control.

    Joint-space motion is admitted by the explicit execution entry point and
    existing per-command/runtime checks, not a geometry approval boolean.
    """
    if joint_space:
        return
    blockers = arm_geometry_blockers(config, joint_space=joint_space)
    if blockers:
        raise ValueError('Robotiq arm execution blocked: ' + '; '.join(blockers)
                         + '. Config booleans cannot authorize execution. '
                         'Use subscriber-only shadow inference and standalone diagnostics.')
