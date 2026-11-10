"""Offline Fusion-v13 tool audit; never imports a profile or contacts a robot.

CAD inputs are read from an explicit directory; transform evidence is read
from an immutable Git revision without checking out its simulation files.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import uuid
import zipfile

import numpy as np


REVISION = '4e1557ee2b579574c8b5c701b19df94c5c336667'
CAD = 'sim/cad_reference/fr3_robotiq_zed'
FRAMES = CAD + '/measurements/fusion_frames_20261013_102402.json'
FACES = CAD + '/measurements/hg1_fusion_faces_20261013T083300663567Z.json'
TRANSFORMS = 'sim/assets/FR3_2F85_ZED_transforms/fusion_v13_candidate.json'
ROOT = Path(__file__).resolve().parents[2]


def rigid_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3),
                               atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-8)):
        raise ValueError('Invalid rigid transform')
    return t


def inertia_check(value):
    i = np.asarray(value, dtype=float)
    if (i.shape != (3, 3) or not np.isfinite(i).all()
            or not np.allclose(i, i.T, atol=1e-12, rtol=0)):
        raise ValueError('Invalid or asymmetric CoM inertia')
    eigenvalues = np.linalg.eigvalsh(i)
    if eigenvalues[0] <= 0 or eigenvalues[-1] > sum(eigenvalues[:2]) + 1e-12:
        raise ValueError('Nonphysical principal moments')
    return eigenvalues.tolist()


def combine(components):
    """Combine complete, non-overlapping bodies; no missing-value defaults.

    Each tensor is about its body's CoM, in its local axes; each transform
    maps those axes into the flange. All inputs use kg, m and kg m^2.
    """
    if not components:
        raise ValueError('No components')
    rows = []
    for body in components:
        if any(body.get(k) is None for k in ('mass', 'com', 'inertia', 'transform')):
            raise ValueError('Incomplete component mass properties')
        m = float(body['mass'])
        c = np.asarray(body['com'], dtype=float)
        if not np.isfinite(m) or m <= 0 or c.shape != (3,) or not np.isfinite(c).all():
            raise ValueError('Invalid mass or CoM')
        t = rigid_transform(body['transform'])
        i = np.asarray(body['inertia'], dtype=float)
        inertia_check(i)
        rows.append((m, t[:3, :3] @ c + t[:3, 3],
                     t[:3, :3] @ i @ t[:3, :3].T))
    mass = sum(m for m, _, _ in rows)
    center = sum(m * c for m, c, _ in rows) / mass
    tensor = np.zeros((3, 3))
    for m, c, i in rows:
        d = c - center
        tensor += i + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    inertia_check(tensor)
    return dict(mass_kg=mass, flange_com_m=center.tolist(),
                com_inertia_flange_axes_kg_m2=tensor.tolist())


def git_bytes(path):
    return subprocess.check_output(['git', 'show', f'{REVISION}:{path}'], cwd=ROOT)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def audit(cad_dir, original, nominal):
    record = json.loads(git_bytes(TRANSFORMS))
    frames_bytes, faces_bytes = git_bytes(FRAMES), git_bytes(FACES)
    frames, faces = json.loads(frames_bytes), json.loads(faces_bytes)
    hashes = {}
    for name, expected in [
            ('FR3_2F85_ZED.f3z', record['source_f3z_sha256']),
            ('FR3_2F85_ZED.step', record['source_step_sha256'])]:
        hashes[name] = digest((cad_dir / name).read_bytes())
        if hashes[name] != expected:
            raise ValueError('CAD hash mismatch: ' + name)
    if (cad_dir / 'robotiq_manual.pdf').exists():
        hashes['robotiq_manual.pdf'] = digest((cad_dir / 'robotiq_manual.pdf').read_bytes())
    if (digest(frames_bytes) != record['source_frames_sha256']
            or digest(faces_bytes) != record['source_faces_sha256']):
        raise ValueError('Transform export hash mismatch')
    with zipfile.ZipFile(cad_dir / 'FR3_2F85_ZED.f3z') as archive:
        if archive.testzip() is not None:
            raise ValueError('Corrupt F3Z')
        description = json.loads(archive.read('DesignDescription.json'))
        version = description['designDescription']['currentVersion']
        if version != '13':
            raise ValueError('Expected Fusion v13')

    occurrences = frames['occurrences']
    flange = next(o for o in occurrences
                  if o['component_name'].endswith('Link_Flange'))
    root_from_flange = np.array(flange['transform2']['matrix_mm'], dtype=float)
    root_from_flange[:3, 3] *= .001
    offset = np.eye(4)
    offset[2, 3] = .107  # v13 physical flange face in Link_Flange local coordinates
    root_from_flange = rigid_transform(root_from_flange @ offset)
    flange_from_root = np.linalg.inv(root_from_flange)
    transforms = {}
    for o in occurrences:
        if o['path'].startswith('Franka_Research_3_v2:'):
            continue
        t = np.array(o['transform2']['matrix_mm'], dtype=float)
        t[:3, 3] *= .001
        transforms[o['path']] = dict(
            component=o['component_name'],
            has_child_occurrences=any(p['path'].startswith(o['path'] + '+')
                                      for p in occurrences),
            T_flange_from_component=rigid_transform(flange_from_root @ t).tolist())
    checks = {}
    for suffix, key in [('Defeatured 2F-85 PAD OPEN_base.step:1', 'hand'),
                        ('W-008-COUPLING_W-50-4M6-1D6_C-062:1', 'coupling'),
                        ('ZED M STEP:1', 'camera')]:
        actual = next(v['T_flange_from_component'] for k, v in transforms.items()
                      if k.endswith(suffix))
        expected = rigid_transform(record['matrices_m'][f'T_flange_from_{key}_cad'])
        error = float(np.max(np.abs(np.array(actual) - expected)))
        checks[key + '_candidate_matrix_max_error'] = error
        if error > 1e-8:
            raise ValueError('v13 transform reproduction failed: ' + key)
    planes = {}
    for face in faces['faces']:
        p = flange_from_root @ np.r_[np.array(face['point_root_mm']) * .001, 1]
        n = flange_from_root[:3, :3] @ face['outward_normal_root_unit']
        planes[face['label']] = dict(sample_point_m=p[:3].tolist(),
                                     outward_normal=n.tolist())

    step = (cad_dir / 'FR3_2F85_ZED.step').read_text()
    counts = {token: len(re.findall(r'\b' + token + r'\b', step)) for token in
              ['MANIFOLD_SOLID_BREP', 'CLOSED_SHELL', 'OPEN_SHELL',
               'MATERIAL_DESIGNATION', 'MASS_DENSITY', 'DENSITY', 'MASS_MEASURE']}
    # Inspect declared solid representations, not physical-material assignments.
    entities = {int(n): (k, v) for n, k, v in
                re.findall(r'#(\d+)\s*=\s*([A-Z_]+)\s*\((.*?)\);', step, re.S)}
    refs = lambda value: [int(n) for n in re.findall(r'#(\d+)', value)]
    solid_counts = {}
    for definition, (kind, value) in entities.items():
        if kind != 'PRODUCT_DEFINITION':
            continue
        name = re.findall(r"'([^']*)'", value)[0]
        shapes = {n for n, (k, v) in entities.items()
                  if k == 'PRODUCT_DEFINITION_SHAPE' and refs(v)[-1:] == [definition]}
        representations = {refs(v)[1] for k, v in entities.values()
                           if k == 'SHAPE_DEFINITION_REPRESENTATION'
                           and refs(v)[0] in shapes}
        linked = {refs(v)[1] for k, v in entities.values()
                  if k == 'SHAPE_REPRESENTATION_RELATIONSHIP'
                  and refs(v)[0] in representations}
        solids = {n for r in representations | linked for n in refs(entities[r][1])
                  if entities.get(n, ('',))[0] == 'MANIFOLD_SOLID_BREP'}
        solid_counts[name] = len(solids)

    original_data, nominal_data = json.loads(original.read_bytes()), json.loads(nominal.read_bytes())
    # This supplied identifier is evidence from the operator's candidate,
    # not an independent device listing read from the robot.
    generic_id = nominal_data['deviceId']
    if generic_id != 'ee-no-gripper':
        raise ValueError('Expected supplied Generic Device candidate identifier')
    draft = dict(id=str(uuid.uuid5(uuid.NAMESPACE_URL, REVISION + '/incomplete-physical-tool')),
                 name='FR3 Robotiq ZED Fusion-v13 — INCOMPLETE — DO NOT ACTIVATE',
                 deviceId=generic_id,
                 inertial=dict(mass=None, centerOfMass=dict(x=None, y=None, z=None),
                               inertia={k: None for k in ('x11', 'x12', 'x13', 'x22', 'x23', 'x33')}),
                 tcp=dict(rotation=dict(pitch=None, roll=None, yaw=None),
                          translation=dict(x=None, y=None, z=None)))
    def field_layout(value):
        return ({key: field_layout(item) for key, item in value.items()}
                if isinstance(value, dict) else None)
    if field_layout(draft) != field_layout(original_data):
        raise ValueError('Unexpected profile field layout')
    tensor = np.diag([.004180, .005080, .001250])
    report = dict(
        status='incomplete_not_for_import_or_activation', source_revision=REVISION,
        source_branch='experiment/isaacsim-fr3-droid', fusion_version=version,
        source_hashes={**hashes, FRAMES: digest(frames_bytes), FACES: digest(faces_bytes),
                       TRANSFORMS: digest(git_bytes(TRANSFORMS)),
                       str(original): digest(original.read_bytes()),
                       str(nominal): digest(nominal.read_bytes())},
        source_transform_status=record['status'], checks=checks,
        convention='p_F = T_F_from_component @ p_component; m, kg, kg m^2; matrices by rows',
        T_root_from_physical_flange=root_from_flange.tolist(),
        tool_occurrence_transforms=transforms, face_supporting_planes=planes,
        step_entity_counts=counts, step_product_solid_declarations=solid_counts,
        material_assignments_verified=False, physical_solid_validity_verified=False,
        catalog=dict(robotiq_mass_kg=.921, robotiq_com_m=[0, 0, .060],
                     robotiq_inertia_kg_m2=tensor.tolist(),
                     robotiq_principal_moments=inertia_check(tensor),
                     inertia_reference_point='not explicitly established in cited section',
                     zed_mini_mass_kg=.0629, known_catalog_mass_sum_kg=.9839,
                     coupling_included=True, installed_coupling_equivalence_verified=False),
        combined=dict(status='not_numerically_determined', mass_kg=None, flange_com_m=None,
                      com_inertia_flange_axes_kg_m2=None,
                      mass_equation_kg='0.9839 + m_printed_mounts + m_extra_hardware + delta_coupling'),
        tcp_candidates=dict(manufacturer_local_m=[0, 0, .174],
                            simulated_grip_T_flange=record['matrices_m']['T_flange_from_usd_grip'],
                            physical_grasp_frame=None),
        generic_device_identifier=dict(value=generic_id, source=str(nominal),
                                       independently_verified_from_generic_export=False),
        validation=dict(complete_inertial_model_verified=False,
                        combined_positive_definiteness='not testable: missing inputs',
                        manufacturer_tensor_algebra='symmetric, positive definite, triangle inequalities pass',
                        profile_field_layout='matches exported top-level/nested names; null values intentionally not importable',
                        robot_writes_performed=False),
        unresolved=[
            'assigned material/density and physical print fill or measured properties for both mount parts',
            'ZED Mini CoM and inertia in its exported CAD frame',
            'manufacturer inertia origin/axes and manufacturer-to-CAD reference transform',
            'included coupling identity equivalence or replacement mass-property correction',
            'hardware fasteners/cable inventory and supported mass allocation',
            'qualified mating/contact faces and physical grasp-frame selection',
            'Generic Device identity confirmation from a Generic profile export'])
    return report, draft


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cad-dir', type=Path, required=True)
    p.add_argument('--original-profile', type=Path, required=True)
    p.add_argument('--nominal-profile', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    args = p.parse_args()
    report, draft = audit(args.cad_dir, args.original_profile, args.nominal_profile)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    for name, data in [('audit.json', report),
                       ('Robotiq_ZED_Fusion_v13.INCOMPLETE_NOT_FOR_IMPORT.json', draft)]:
        (args.output_dir / name).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    with (args.output_dir / 'component_inventory.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            'occurrence_path', 'component', 'physical_group',
            'individual_mass_kg', 'mass_evidence', 'assigned_density_verified'])
        writer.writeheader()
        for path, row in report['tool_occurrence_transforms'].items():
            if row['has_child_occurrences']:
                continue
            group = ('coupling' if '+GRP-CPL-' in path else
                     'gripper' if path.startswith('Robotiq_') else
                     'camera' if '+ZEDM:' in path else 'custom_mount')
            writer.writerow(dict(occurrence_path=path, component=row['component'],
                                 physical_group=group,
                                 individual_mass_kg=.0629 if group == 'camera' else '',
                                 mass_evidence=('manufacturer ZED Mini catalog' if group == 'camera' else
                                                'aggregate Robotiq scope; no separate mass assigned' if group in ('coupling', 'gripper') else
                                                'unknown installed material/print properties'),
                                 assigned_density_verified=False))
    print(json.dumps(dict(status=report['status'], output=str(args.output_dir),
                          reproduced_transform_checks=report['checks'])))


if __name__ == '__main__':
    main()
