"""Development evidence capture and separate, fail-closed experiment qualification."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess

from saps.physical.hardware_provenance import arm_geometry_blockers
from saps.physical.shadow_config import OPENPI_COMMIT
from saps.physical.shadow_ros import git_identity

SAPS_BASE = 'dc5ce5e1ee038d793f7b672cc72a5fffce8d06c2'


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def repository_snapshot(directory: Path, prefixes: tuple[str, ...] = ()) -> dict:
    """Hash staged/unstaged diffs and actual relevant tracked/untracked files.

    Diff contents are not logged. Untracked source content is covered by file
    hashes; ignored local hardware config is captured separately by the caller.
    """
    identity = git_identity(directory)
    if not identity.get('available'):
        return identity
    directory = Path(identity.get('repository_root', directory))
    def git(*args):
        return subprocess.check_output(['git', '-C', str(directory), *args], timeout=10)
    identity['diff_sha256'] = {
        kind: hashlib.sha256(git('diff', '--binary', '--no-ext-diff', '--no-textconv', *args)).hexdigest()
        for kind, args in (('unstaged', ()), ('staged', ('--cached',)), ('head', ('HEAD',)))
    }
    paths = git('ls-files', '--cached', '--others', '--exclude-standard', '-z').decode().split('\0')
    identity['source_sha256'] = {
        name: file_hash(directory / name) for name in sorted(set(paths))
        if name and (not prefixes or name.startswith(prefixes)) and (directory / name).is_file()
    }
    return identity


def ancestor(directory: Path, commit: str) -> bool:
    return subprocess.run(['git', '-C', str(directory), 'merge-base', '--is-ancestor', commit, 'HEAD'],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10).returncode == 0


def capture_provenance(root: Path, lab: Path, igd: Path, config_path: Path, config: dict) -> dict:
    """Capture provenance without requiring a clean checkout or matching manifest."""
    repos = dict(
        repository=repository_snapshot(root, ('src/', 'scripts/', 'tools/', 'configs/', 'patches/',
                                               'Makefile', 'requirements', 'pyproject', 'docker', 'compose')),
        lab=repository_snapshot(lab), igd=repository_snapshot(igd),
        openpi=repository_snapshot(root / 'third_party/openpi', ('src/', 'packages/', 'scripts/', 'pyproject')))
    path = root / 'configs/zed_robotiq_lab_sources.json'
    manifest = dict(path=str(path), available=path.is_file())
    try:
        expected = json.loads(path.read_text())
        files = expected['files_sha256']
        actual = repos['lab'].get('source_sha256', {})
        manifest.update(sha256=file_hash(path), baseline_commit=expected['baseline_commit'],
                        baseline_is_ancestor=ancestor(lab, expected['baseline_commit']),
                        matches=actual == files,
                        changed=sorted(p for p in actual.keys() & files.keys() if actual[p] != files[p]),
                        added=sorted(actual.keys() - files.keys()), missing=sorted(files.keys() - actual.keys()))
    except (OSError, ValueError, KeyError, TypeError) as error:
        manifest.update(matches=False, error=f'{type(error).__name__}: {error}')
    blockers = []
    for key in ('repository', 'lab'):
        if not repos[key].get('available') or repos[key].get('dirty', True):
            blockers.append(key + ': unreviewed or unavailable working tree')
    if not manifest.get('matches'):
        blockers.append('lab source differs from the reviewed manifest')
    if not manifest.get('baseline_is_ancestor', False):
        blockers.append('lab baseline ancestry is unqualified')
    saps_ancestor = ancestor(root, SAPS_BASE)
    if not saps_ancestor:
        blockers.append('SAPS runtime ancestry is unqualified')
    if (repos['openpi'].get('commit') != OPENPI_COMMIT or repos['openpi'].get('dirty', True)):
        blockers.append('OpenPI must remain clean at the unchanged policy pin')
    return dict(**repos, manifest=manifest, saps_baseline=SAPS_BASE, saps_baseline_is_ancestor=saps_ancestor,
                config=dict(path=str(config_path), sha256=file_hash(config_path)),
                requested_hardware_config=config,
                effective_hardware=dict(status='not_observed',
                                        note='File configuration is requested state, not device readback'),
                qualification=dict(status='qualified_sources' if not blockers else 'unqualified_sources',
                                   blockers=blockers))


def require_qualified_sources(provenance: dict) -> None:
    blockers = provenance['qualification']['blockers']
    if blockers:
        raise ValueError('Strict experiment provenance rejected: ' + '; '.join(blockers))


def validate_mode(mode: str, config: dict, *, execution: bool = False,
                  max_replans=None, max_chunks=None) -> None:
    if mode not in ('development', 'strict'):
        raise ValueError('provenance mode must be development or strict')
    if mode == 'development' and execution:
        if config.get('integration_stage') != 'c4':
            raise ValueError('Development execution is scoped to the explicit C4 profile')
        if (type(max_replans) is not int or max_replans <= 0
                or type(max_chunks) is not int or max_chunks <= 0):
            raise ValueError('Development execution requires positive finite integer replan and chunk limits')


def readiness_report(config: dict, provenance: dict) -> dict:
    """Keep unverified hardware findings distinct from actual runtime failures."""
    physical = arm_geometry_blockers(config, joint_space=True)
    return dict(
        observation=dict(status='not_checked'),
        policy_inference=dict(status='not_checked'),
        physical_runtime=dict(status='unverified' if physical else 'not_checked',
                              runtime_checks='not_checked', blockers=[],
                              unverified_conditions=physical),
        reproducible_experiment_qualification=dict(
            status='not_qualified' if physical or provenance['qualification']['blockers'] else 'not_checked',
            sources=provenance['qualification'],
            note='Qualified sources alone do not qualify hardware readiness or a completed experiment'))
