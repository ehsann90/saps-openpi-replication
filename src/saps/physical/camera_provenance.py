"""Camera software evidence without treating installed packages as checkouts."""
from __future__ import annotations

import hashlib
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from saps.physical.shadow_ros import git_identity


def _camera_identity(path: Path) -> dict:
    """Preserve camera revision/diff fields while resolving checkout identity."""
    identity = git_identity(path)
    identity['revision'] = identity.get('commit')
    if identity['available']:
        try:
            diff = subprocess.check_output(
                ['git', '-C', identity['repository_root'], 'diff', '--binary',
                 'HEAD'], text=True, stderr=subprocess.PIPE, timeout=10)
            identity['tracked_diff_sha256'] = hashlib.sha256(
                diff.strip().encode()).hexdigest()
        except (OSError, subprocess.SubprocessError) as error:
            identity.update(tracked_diff_sha256=None,
                            diff_error=f'{type(error).__name__}: diff unavailable')
    return identity


def camera_software_provenance() -> dict:
    """Keep effective versions and source hashes separate from Git identity."""
    from ament_index_python.packages import get_package_share_directory
    from franka_zed_cameras import camera_validation

    module = Path(camera_validation.__file__).resolve()
    root = module.parent.parent
    lab_identity = _camera_identity(root)
    result = dict(lab_repository=lab_identity, source_sha256={},
                  packages={})
    for name in ('camera_validation.py', 'camera_pairing.py', 'zed_cameras.py'):
        path = module.with_name(name)
        result['source_sha256'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ('launch/dual_zed.launch.py', 'config/zed_cameras.yaml'):
        path = root / name
        if path.is_file():
            result['source_sha256'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ('zed_wrapper', 'zed_components', 'zed_msgs'):
        try:
            share = Path(get_package_share_directory(name)).resolve()
            manifest = share / 'package.xml'
            source_directory = manifest.resolve().parent
            identity = _camera_identity(source_directory)
            if not identity['available']:
                identity['source_kind'] = 'installed_package_without_git_identity'
            result['packages'][name] = dict(
                version=ET.parse(manifest).getroot().findtext('version'),
                package_share=str(share), source_directory=str(source_directory),
                repository=identity,
                package_xml_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
        except (OSError, LookupError, ET.ParseError) as error:
            result['packages'][name] = dict(error=str(error))
    header = Path('/usr/local/zed/include/sl/Camera.hpp')
    try:
        content = header.read_bytes()
        result['sdk_header_sha256'] = hashlib.sha256(content).hexdigest()
        parts = [re.search(r'#define ZED_SDK_' + part + r'_VERSION\s+(\d+)',
                           content.decode())
                 for part in ('MAJOR', 'MINOR', 'PATCH')]
        result['sdk_version_from_headers'] = (
            '.'.join(part.group(1) for part in parts) if all(parts) else None)
    except OSError as error:
        result.update(sdk_version_from_headers=None, sdk_header_error=str(error))
    return result
