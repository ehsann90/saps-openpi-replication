"""Interactively export six planar CAD face references from an open Fusion design.

Install as a Fusion Python script. This module imports Fusion only inside run()
so its rigid-transform checks can be tested with ordinary Python.
"""

import csv
import json
import math
import traceback
from datetime import datetime, timezone
from pathlib import Path


LABELS = (
    "fr3_flange_mating",
    "coupling_fr3_side",
    "coupling_robotiq_side",
    "robotiq_base_mating",
    "left_pad_contact",
    "right_pad_contact",
)
OUTPUT_DIR = Path(__file__).resolve().parent / "exports"
LENGTH_SCALE_MM_PER_CM = 10.0
MATRIX_TOLERANCE = 1e-8
NORMAL_TOLERANCE = 1e-7
PLANE_TOLERANCE_CM = 1e-5


def matrix_rows(matrix):
    """Read a Fusion Matrix3D as column-vector row-major numeric rows."""
    return [[float(matrix.getCell(row, col)) for col in range(4)]
            for row in range(4)]


def validate_rigid_matrix(rows):
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError("Occurrence transform must be 4x4")
    if any(not math.isfinite(value) for row in rows for value in row):
        raise ValueError("Occurrence transform contains a nonfinite value")
    if max(abs(rows[3][i] - [0, 0, 0, 1][i]) for i in range(4)) > MATRIX_TOLERANCE:
        raise ValueError("Occurrence transform has a non-affine last row")
    rotation = [row[:3] for row in rows[:3]]
    for i in range(3):
        for j in range(3):
            product = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            if abs(product - (1.0 if i == j else 0.0)) > MATRIX_TOLERANCE:
                raise ValueError("Occurrence rotation is not orthonormal")
    a, b, c = rotation
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - a[1] * (b[0] * c[2] - b[2] * c[0])
                   + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if abs(determinant - 1.0) > MATRIX_TOLERANCE:
        raise ValueError("Occurrence transform is not right-handed rigid")


def xyz(value):
    result = [float(value.x), float(value.y), float(value.z)]
    if any(not math.isfinite(component) for component in result):
        raise ValueError("Point or normal contains a nonfinite value")
    return result


def norm(vector):
    return math.sqrt(sum(component * component for component in vector))


def unit_normal(vector):
    length = norm(vector)
    if length <= 0 or abs(length - 1.0) > NORMAL_TOLERANCE:
        raise ValueError("Face evaluator did not return a unit normal")
    return vector


def transform_point(rows, point):
    return [sum(rows[i][j] * point[j] for j in range(3)) + rows[i][3]
            for i in range(3)]


def transform_normal(rows, normal):
    """A rigid occurrence rotation maps normals; translation does not."""
    return unit_normal([sum(rows[i][j] * normal[j] for j in range(3))
                        for i in range(3)])


def inverse_point(rows, root_point):
    rotation = [row[:3] for row in rows[:3]]
    delta = [root_point[i] - rows[i][3] for i in range(3)]
    return [sum(rotation[j][i] * delta[j] for j in range(3))
            for i in range(3)]


def max_error(left, right):
    return max(abs(a - b) for a, b in zip(left, right))


def signed_plane_residual(point, plane_point, outward_normal):
    return sum((point[i] - plane_point[i]) * outward_normal[i]
               for i in range(3))


def mm_matrix(rows):
    result = [row[:] for row in rows]
    for row in result[:3]:
        row[3] *= LENGTH_SCALE_MM_PER_CM
    return result


def record_face(label, face, adsk):
    face = adsk.fusion.BRepFace.cast(face)
    if face is None:
        raise ValueError("Selection is not a BRepFace")
    occurrence = face.assemblyContext
    native = face.nativeObject
    if occurrence is None or native is None:
        raise ValueError("Select the face in the root assembly, not an isolated component")
    if not native.body.isSolid:
        raise ValueError("Outward normal requires a closed solid body")
    if adsk.core.Plane.cast(native.geometry) is None:
        raise ValueError("Selected face is not planar")

    rows = matrix_rows(occurrence.transform2)
    validate_rigid_matrix(rows)
    native_sample_object = native.pointOnFace
    proxy_sample_object = face.pointOnFace
    native_sample = xyz(native_sample_object)
    proxy_sample = xyz(proxy_sample_object)
    ok, local_normal_object = native.evaluator.getNormalAtPoint(native_sample_object)
    if not ok or local_normal_object is None:
        raise ValueError("Could not evaluate the local outward normal")
    local_normal = unit_normal(xyz(local_normal_object))
    ok, proxy_normal_object = face.evaluator.getNormalAtPoint(proxy_sample_object)
    if not ok or proxy_normal_object is None:
        raise ValueError("Could not evaluate the assembly-face outward normal")
    proxy_normal = unit_normal(xyz(proxy_normal_object))

    root_point = transform_point(rows, native_sample)
    root_normal = transform_normal(rows, local_normal)
    inverse_proxy_point = inverse_point(rows, proxy_sample)
    root_residual_cm = signed_plane_residual(root_point, proxy_sample, proxy_normal)
    local_residual_cm = signed_plane_residual(inverse_proxy_point, native_sample,
                                              local_normal)
    normal_error = max_error(root_normal, proxy_normal)
    roundtrip_error_cm = max(
        max_error(inverse_point(rows, root_point), native_sample),
        max_error(transform_point(rows, inverse_proxy_point), proxy_sample),
    )

    native_sample_on_native = bool(native.isPointOnFace(native_sample_object))
    proxy_sample_on_proxy = bool(face.isPointOnFace(proxy_sample_object))
    root_point_object = adsk.core.Point3D.create(*root_point)
    inverse_proxy_object = adsk.core.Point3D.create(*inverse_proxy_point)
    transformed_native_on_proxy = bool(face.isPointOnFace(root_point_object))
    inverse_proxy_on_native = bool(native.isPointOnFace(inverse_proxy_object))
    diagnostics = {
        "label": label,
        "occurrence_path": occurrence.fullPathName,
        "component_name": occurrence.component.name,
        "body_name": native.body.name,
        "face_native_entity_token": native.entityToken,
        "face_proxy_entity_token": face.entityToken,
        "face_temp_id_session_only": native.tempId,
        "native_sample_component_cm": native_sample,
        "proxy_sample_assembly_cm": proxy_sample,
        "transformed_native_sample_assembly_cm": root_point,
        "inverse_proxy_sample_component_cm": inverse_proxy_point,
        "signed_root_plane_residual_cm": root_residual_cm,
        "signed_local_plane_residual_cm": local_residual_cm,
        "normal_max_component_error": normal_error,
        "roundtrip_error_cm": roundtrip_error_cm,
        "native_sample_on_native_trim": native_sample_on_native,
        "proxy_sample_on_proxy_trim": proxy_sample_on_proxy,
        "transformed_native_on_proxy_trim": transformed_native_on_proxy,
        "inverse_proxy_on_native_trim": inverse_proxy_on_native,
    }
    if (abs(root_residual_cm) > PLANE_TOLERANCE_CM
            or abs(local_residual_cm) > PLANE_TOLERANCE_CM
            or normal_error > NORMAL_TOLERANCE
            or roundtrip_error_cm > MATRIX_TOLERANCE):
        raise ValueError("CAD face frame or plane mismatch: "
                         + json.dumps(diagnostics, sort_keys=True))

    # A supporting plane can agree while a cross-context containment API
    # rejects a point. Keep this distinction visible; do not qualify the trim.
    trim_qualified = all((native_sample_on_native, proxy_sample_on_proxy,
                          transformed_native_on_proxy, inverse_proxy_on_native))
    diagnostics["plane_and_normal_qualified"] = True
    diagnostics["trim_containment_qualified"] = trim_qualified
    diagnostics["status"] = ("qualified" if trim_qualified
                             else "plane_qualified_trim_containment_unresolved")

    return {
        "label": label,
        "occurrence_path": occurrence.fullPathName,
        "component_name": occurrence.component.name,
        "body_name": native.body.name,
        "face_geometry_type": native.geometry.objectType,
        "face_object_type": native.objectType,
        "face_native_entity_token": native.entityToken,
        "face_proxy_entity_token": face.entityToken,
        "face_temp_id_session_only": native.tempId,
        "face_area_cm2": float(native.area),
        "point_kind": "Native BRepFace.pointOnFace sample; not a centroid or bolt centre",
        "point_component_cm": native_sample,
        "point_component_mm": [x * LENGTH_SCALE_MM_PER_CM for x in native_sample],
        "point_root_cm": root_point,
        "point_root_mm": [x * LENGTH_SCALE_MM_PER_CM for x in root_point],
        "proxy_point_kind": "Assembly proxy BRepFace.pointOnFace sample; may differ from native sample",
        "point_proxy_assembly_cm": proxy_sample,
        "point_proxy_assembly_mm": [x * LENGTH_SCALE_MM_PER_CM for x in proxy_sample],
        "outward_normal_component_unit": local_normal,
        "outward_normal_root_unit": root_normal,
        "outward_normal_proxy_assembly_unit": proxy_normal,
        "T_root_from_component": {
            "matrix_fusion_cm": rows,
            "matrix_mm": mm_matrix(rows),
        },
        "validation": diagnostics,
    }


def write_export(document_name, rows):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    stem = "hg1_fusion_faces_" + timestamp
    payload = {
        "schema": "fusion_cad_planar_face_audit_v2",
        "design": document_name,
        "exported_at_utc": datetime.now(timezone.utc).isoformat(),
        "root_frame": "Active Fusion design root component origin",
        "convention": "T_A_from_B maps column coordinates B into A; right-handed",
        "length_note": "Fusion API uses cm; exported mm = cm * 10; normals are unitless",
        "point_note": "Native and proxy pointOnFace samples are separate; neither is a centroid or bolt-pattern centre",
        "faces": rows,
    }
    json_path = OUTPUT_DIR / (stem + ".json")
    csv_path = OUTPUT_DIR / (stem + ".csv")
    with json_path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
        stream.write("\n")
    fields = [
        "label", "occurrence_path", "component_name", "body_name",
        "face_geometry_type", "face_object_type", "face_native_entity_token",
        "face_proxy_entity_token", "face_temp_id_session_only", "face_area_cm2",
        "point_kind", "point_component_cm", "point_component_mm",
        "point_root_cm", "point_root_mm", "proxy_point_kind",
        "point_proxy_assembly_cm", "point_proxy_assembly_mm",
        "outward_normal_component_unit", "outward_normal_root_unit",
        "outward_normal_proxy_assembly_unit", "validation_status",
        "signed_root_plane_residual_cm", "signed_local_plane_residual_cm",
        "T_root_from_component_matrix_fusion_cm", "T_root_from_component_matrix_mm",
    ]
    with csv_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in fields if key in row}
            flat["T_root_from_component_matrix_fusion_cm"] = row["T_root_from_component"]["matrix_fusion_cm"]
            flat["T_root_from_component_matrix_mm"] = row["T_root_from_component"]["matrix_mm"]
            flat["validation_status"] = row["validation"]["status"]
            flat["signed_root_plane_residual_cm"] = row["validation"]["signed_root_plane_residual_cm"]
            flat["signed_local_plane_residual_cm"] = row["validation"]["signed_local_plane_residual_cm"]
            for key, value in flat.items():
                if isinstance(value, (list, dict)):
                    flat[key] = json.dumps(value, separators=(",", ":"))
            writer.writerow(flat)
    return json_path, csv_path


def run(context):
    import adsk.core
    import adsk.fusion

    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None or app.activeDocument is None:
            raise RuntimeError("Open the FR3_2F85_ZED Fusion design first")
        rows = []
        for label in LABELS:
            selection = ui.selectEntity(
                "HG1: select planar solid face for " + label,
                "PlanarFaces",
            )
            if selection is None:
                raise RuntimeError("Selection cancelled; no files were written")
            row = record_face(label, selection.entity, adsk)
            if any((old["occurrence_path"], old["face_temp_id_session_only"])
                   == (row["occurrence_path"], row["face_temp_id_session_only"])
                   for old in rows):
                raise ValueError("The same face was selected for two labels")
            rows.append(row)
        json_path, csv_path = write_export(app.activeDocument.name, rows)
        unresolved = [row["label"] for row in rows
                      if row["validation"]["status"] != "qualified"]
        note = ("\nTrim containment unresolved for: " + ", ".join(unresolved)
                if unresolved else "")
        ui.messageBox("HG1 face export complete:\n{}\n{}{}".format(
            json_path, csv_path, note))
    except Exception:
        ui.messageBox("HG1 face export stopped; review the error:\n" + traceback.format_exc())
